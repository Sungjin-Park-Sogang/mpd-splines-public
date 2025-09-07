import os
import pathlib
import pickle
import random
import time
import h5py
import numpy as np
import torch
import yaml
from tqdm import tqdm

# CuRobo imports
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.robot import JointState, RobotConfig

# MPD visualization imports
import sys
sys.path.append('/isaac-sim/curobo/examples/mpd-splines')
from mpd.plotting.visualize_trajectories_curobo import save_trajectory_usd

def interpolate_trajectory(trajectory, target_length=128):
    """
    Interpolate trajectory to a fixed target length using linear interpolation.
    
    Args:
        trajectory: torch.Tensor of shape [T, dof] where T is variable length
        target_length: int, desired trajectory length
    
    Returns:
        torch.Tensor of shape [target_length, dof]
    """
    current_length = trajectory.shape[0]
    dof = trajectory.shape[1]
    
    if current_length == target_length:
        return trajectory
    
    if current_length < 2:
        # Handle edge case: if trajectory is too short, just repeat the single point
        return trajectory.repeat(target_length, 1)
    
    # Create normalized indices for original and target trajectories
    original_indices = torch.linspace(0, current_length - 1, current_length, device=trajectory.device)
    target_indices = torch.linspace(0, current_length - 1, target_length, device=trajectory.device)
    
    # Interpolate all DOFs using torch.nn.functional.interpolate
    # Reshape trajectory to [1, dof, current_length] for interpolate function
    trajectory_reshaped = trajectory.transpose(0, 1).unsqueeze(0)  # [1, dof, current_length]
    
    # Use torch.nn.functional.interpolate for linear interpolation
    interpolated_reshaped = torch.nn.functional.interpolate(
        trajectory_reshaped,
        size=target_length,
        mode='linear',
        align_corners=True
    )  # [1, dof, target_length]
    
    # Reshape back to [target_length, dof]
    interpolated_trajectory = interpolated_reshaped.squeeze(0).transpose(0, 1)
    
    return interpolated_trajectory


def interpolate_batch_trajectories(trajectories_list, success_flags, target_length=128):
    """
    Interpolate a batch of trajectories to fixed length, keeping only successful ones.
    
    Args:
        trajectories_list: List of torch.Tensor, each of shape [Ti, dof] (variable length)
        success_flags: torch.Tensor of shape [batch_size] with boolean success flags
        target_length: int, desired trajectory length
    
    Returns:
        torch.Tensor of shape [num_successful, target_length, dof]
    """
    successful_trajectories = []
    
    for i, (traj, success) in enumerate(zip(trajectories_list, success_flags)):
        if success.item():  # Only process successful trajectories
            interpolated_traj = interpolate_trajectory(traj, target_length)
            successful_trajectories.append(interpolated_traj)
    
    if successful_trajectories:
        return torch.stack(successful_trajectories, dim=0)
    else:
        # Return empty tensor with correct shape if no successful trajectories
        device = trajectories_list[0].device if trajectories_list else torch.device('cpu')
        dof = trajectories_list[0].shape[1] if trajectories_list else 7
        return torch.empty(0, target_length, dof, device=device)


def motion_planning_from_pairs(pairs_file, batch_size=100, max_attempts=5, planner_timeout=10.0, 
                              save_usd=True, output_dir="./trajectory_output", save_sample_trajectories=5, 
                              max_pairs=None, interpolate_length=128):
    tensor_args = TensorDeviceType()
    
    # Load pre-generated start/goal pairs
    print(f"Loading start/goal pairs from: {pairs_file}")
    try:
        pairs_data = torch.load(pairs_file)
        all_start_states = pairs_data['start_states'].to(tensor_args.device)  # [N, dof]
        all_goal_states = pairs_data['goal_states'].to(tensor_args.device)   # [N, dof]
        metadata = pairs_data['metadata']
        
        print(f"Loaded {len(all_start_states)} start/goal pairs")
        print(f"Robot: {metadata['robot_file']}, World: {metadata['world_file']}")
        
        # Use metadata to get config files, or fallback to defaults
        world_file = metadata.get('world_file', 'collision_table.yml')
        robot_file = metadata.get('robot_file', 'franka.yml')
        
    except Exception as e:
        print(f"Error loading pairs file: {e}")
        return None, None
    
    # Determine how many pairs to process
    total_available_pairs = len(all_start_states)
    if max_pairs is not None:
        total_pairs = min(max_pairs, total_available_pairs)
    else:
        total_pairs = total_available_pairs
    
    print(f"Processing {total_pairs} pairs (out of {total_available_pairs} available)")
    
    # Initialize CuRobo motion generation
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_file,
        world_file,
        tensor_args,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        use_cuda_graph=True,
        num_trajopt_seeds=12,
        num_graph_seeds=1,
        num_ik_seeds=30,
    )
    motion_gen = MotionGen(motion_gen_config)
    
    # Calculate number of batches needed
    num_batches = (total_pairs + batch_size - 1) // batch_size
    all_trajectories = []
    all_success_flags = []
    
    print(f"Processing {total_pairs} trajectory pairs in {num_batches} batches of {batch_size}")
    
    # Process trajectories in batches with progress tracking
    for batch_idx in tqdm(range(num_batches), desc="Planning trajectory batches", unit="batch"):
        # Calculate actual batch size for this iteration (handle remainder)
        current_batch_size = min(batch_size, total_pairs - batch_idx * batch_size)
        
        # Get start and goal states for current batch
        start_idx = batch_idx * batch_size
        end_idx = start_idx + current_batch_size
        
        batch_start_states = all_start_states[start_idx:end_idx]  # [batch_size, dof]
        batch_goal_states = all_goal_states[start_idx:end_idx]   # [batch_size, dof]

        # Convert to JointState objects
        start_joint_states = JointState.from_position(batch_start_states)
        
        # Convert goal joint states to poses via forward kinematics
        goal_joint_states = JointState.from_position(batch_goal_states)
        fk_result = motion_gen.rollout_fn.compute_kinematics(goal_joint_states)
        
        # Create Pose objects from FK result
        goal_poses = Pose(
            position=fk_result.ee_pos_seq,      # [batch, 3]
            quaternion=fk_result.ee_quat_seq    # [batch, 4] (w, x, y, z)
        )

        # Generate trajectories for current batch
        result = motion_gen.plan_batch(
            start_joint_states,
            goal_poses,
            MotionGenPlanConfig(
                max_attempts=max_attempts, enable_graph=False, enable_graph_attempt=1, enable_opt=True
            ),
        )
        
        # Extract individual trajectories from batch result
        # result.optimized_plan.position shape: [batch_size, T_variable, 7]
        batch_trajectory_list = []
        for traj_idx in range(current_batch_size):
            # Extract single trajectory for this batch item
            single_traj = result.optimized_plan.position[traj_idx]  # [T, 7]
            batch_trajectory_list.append(single_traj)
        
        # Interpolate successful trajectories to fixed length
        interpolated_trajectories = interpolate_batch_trajectories(
            batch_trajectory_list, 
            result.success, 
            target_length=interpolate_length
        )
        
        # Store interpolated trajectories and success flags
        if interpolated_trajectories.shape[0] > 0:  # Only add if there are successful trajectories
            all_trajectories.append(interpolated_trajectories)
        
        all_success_flags.append(result.success)
        
        # Print batch progress
        batch_success_rate = result.success.float().mean().item()
        print(f"Batch {batch_idx + 1}/{num_batches}: {current_batch_size} trajectories, success rate: {batch_success_rate:.2%}")
    
    # Combine all trajectories and success flags
    if all_trajectories:
        all_trajectories = torch.cat(all_trajectories, dim=0)  # [N_successful, interpolate_length, 7]
    else:
        # Create empty tensor if no successful trajectories
        all_trajectories = torch.empty(0, interpolate_length, 7, device=tensor_args.device)
    
    all_success_flags = torch.cat(all_success_flags, dim=0)  # [total_pairs]
    
    # Print final statistics
    overall_success_rate = all_success_flags.float().mean().item()
    total_successful = all_success_flags.sum().item()
    print(f"\nFinal Results:")
    print(f"Total pairs processed: {total_pairs}")
    print(f"Successful trajectories: {total_successful}")
    print(f"Overall success rate: {overall_success_rate:.2%}")
    print(f"Interpolated trajectory tensor shape: {all_trajectories.shape}")
    print(f"Trajectory length after interpolation: {interpolate_length}")
    
    # Verify all trajectories have same length
    if total_successful > 0:
        print(f"All trajectories have uniform length: {all_trajectories.shape[1] == interpolate_length}")
    
    # Save sample trajectories as USD files
    if save_usd and total_successful > 0:
        print(f"\nSaving USD files...")
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # all_trajectories now contains only successful trajectories [N_successful, interpolate_length, 7]
        num_successful_trajs = all_trajectories.shape[0]
        num_to_save = min(save_sample_trajectories, num_successful_trajs)
        
        # Sample random successful trajectories to save
        if num_to_save > 0:
            save_indices = torch.randperm(num_successful_trajs)[:num_to_save]
            
            for i, traj_idx in enumerate(save_indices):
                # Extract single trajectory [interpolate_length, 7]
                single_trajectory = all_trajectories[traj_idx]  # Shape: [interpolate_length, 7]
                
                # Create JointState from trajectory
                traj_js = JointState.from_position(single_trajectory)
                
                # Save as USD
                save_path = os.path.join(output_dir, f"trajectory_{i:03d}.usd")
                try:
                    saved_path = save_trajectory_usd(
                        q_traj=traj_js,
                        robot_file=robot_file,
                        world_file=world_file,
                        save_path=save_path,
                        dt=0.1,  # 100ms between frames
                        visualize_robot_spheres=False
                    )
                    print(f"  Saved trajectory {i+1}/{num_to_save}: {saved_path}")
                except Exception as e:
                    print(f"  Failed to save trajectory {i+1}: {e}")
            
            print(f"USD files saved to: {output_dir}")
        else:
            print("No successful trajectories to save as USD.")
    
    return all_trajectories, all_success_flags
    

if __name__ == "__main__":
    # setup_curobo_logger("error")
    
    # First generate start/goal pairs (or use existing ones)
    pairs_file = "./start_goal_samples/franka_collision_table_dist_0.1-10.0_pairs_1000000.pt"  # Generated by generate_start_goal_pairs.py
    
    # Then do motion planning from those pairs
    trajectories, success_flags = motion_planning_from_pairs(
        pairs_file=pairs_file,       # Pre-generated start/goal pairs
        batch_size=50,               # Process 50 at a time
        max_attempts=5,              # Max planning attempts per trajectory
        planner_timeout=10.0,        # Timeout per planning attempt
        save_usd=True,               # Save trajectories as USD
        output_dir="./trajectory_usd_output",  # Output directory
        save_sample_trajectories=5,  # Save 5 sample successful trajectories
        max_pairs=100,              # Process only first 1000 pairs for testing
        interpolate_length=128       # Fixed trajectory length for MPD training
    )