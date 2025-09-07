import os
import torch
import numpy as np
from tqdm import tqdm

# CuRobo imports
from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState, RobotConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig


def generate_start_goal_pairs(
    total_pairs=1000000,
    batch_size=10000,
    min_distance_threshold=0.1,
    max_distance_threshold=10.0,
    output_file=None,  # Auto-generate filename if None
    output_dir="./start_goal_samples",
    world_file="collision_table.yml",
    robot_file="franka.yml"
):
    """
    Generate a large number of valid start/goal joint state pairs.
    
    Args:
        total_pairs: Total number of pairs to generate
        batch_size: Number of pairs to generate per batch (for memory management)
        min_distance_threshold: Minimum L2 distance between start and goal (rad)
        max_distance_threshold: Maximum L2 distance between start and goal (rad)
        output_file: Output file path for saving pairs (auto-generated if None)
        output_dir: Directory to save the pairs file
        world_file: World configuration file
        robot_file: Robot configuration file
    
    Returns:
        Path to saved pairs file
    """
    
    tensor_args = TensorDeviceType()
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Auto-generate filename if not provided
    if output_file is None:
        robot_name = os.path.splitext(robot_file)[0]  # Remove .yml extension
        world_name = os.path.splitext(world_file)[0]  # Remove .yml extension
    
        # Foramt: franka_table_dist_0.1-10.0_pairs_1000000.pt
        filename = f"{robot_name}_{world_name}_dist_{min_distance_threshold}-{max_distance_threshold}_pairs_{total_pairs}.pt"
        output_file = os.path.join(output_dir, filename)
    else:
        # If custom filename provided, ensure it's in the output directory
        if not os.path.dirname(output_file):
            output_file = os.path.join(output_dir, output_file)
    
    print(f"Will save pairs to: {output_file}")
    
    # Setup CuRobo components
    print("Initializing CuRobo components...")
    robot_world_config = RobotWorldConfig.load_from_config(
        robot_file, world_file, tensor_args=tensor_args, pose_weight=[1, 1, 1, 1]    
    )
    curobo_fn = RobotWorld(robot_world_config)
    
    # Calculate number of batches
    num_batches = (total_pairs + batch_size - 1) // batch_size
    
    all_start_pairs = []
    all_goal_pairs = []
    total_generated = 0
    
    print(f"Generating {total_pairs} start/goal pairs in {num_batches} batches")
    print(f"Distance constraints: {min_distance_threshold:.2f} <= distance <= {max_distance_threshold:.2f} rad")
    
    # Generate pairs in batches
    for batch_idx in tqdm(range(num_batches), desc="Generating pair batches", unit="batch"):
        current_batch_size = min(batch_size, total_pairs - batch_idx * batch_size)
        
        # Generate more samples than needed to account for filtering
        oversample_ratio = 2.0  # Generate 2x more to handle filtering
        generation_batch_size = int(current_batch_size * oversample_ratio)
        
        batch_start_pairs = []
        batch_goal_pairs = []
        pairs_needed = current_batch_size
        
        attempts = 0
        max_attempts = 10
        
        while len(batch_start_pairs) < pairs_needed and attempts < max_attempts:
            attempts += 1
            
            # Generate start and goal states
            try:
                start_states = curobo_fn.sample_trajectory(
                    batch=generation_batch_size, horizon=1, mask_valid=True
                ).squeeze(-2)  # [batch, dof]
                
                goal_states = curobo_fn.sample_trajectory(
                    batch=generation_batch_size, horizon=1, mask_valid=True
                ).squeeze(-2)  # [batch, dof]
                
                # Calculate distances between start and goal
                distances = torch.norm(start_states - goal_states, dim=-1)  # [batch]
                
                # Filter based on distance constraints
                valid_mask = (distances >= min_distance_threshold) & (distances <= max_distance_threshold)
                valid_indices = torch.where(valid_mask)[0]
                
                if len(valid_indices) > 0:
                    # Take only what we need
                    remaining_needed = pairs_needed - len(batch_start_pairs)
                    take_count = min(len(valid_indices), remaining_needed)
                    selected_indices = valid_indices[:take_count]
                    
                    batch_start_pairs.append(start_states[selected_indices])
                    batch_goal_pairs.append(goal_states[selected_indices])
                
            except Exception as e:
                print(f"  Warning: Sampling failed in batch {batch_idx + 1}, attempt {attempts}: {e}")
                continue
        
        # Combine batch results
        if batch_start_pairs:
            batch_starts = torch.cat(batch_start_pairs, dim=0)[:pairs_needed]
            batch_goals = torch.cat(batch_goal_pairs, dim=0)[:pairs_needed]
            
            all_start_pairs.append(batch_starts)
            all_goal_pairs.append(batch_goals)
            total_generated += len(batch_starts)
            
            # Print batch statistics
            batch_distances = torch.norm(batch_starts - batch_goals, dim=-1)
            print(f"  Batch {batch_idx + 1}/{num_batches}: Generated {len(batch_starts)} pairs")
            print(f"    Distance stats: min={batch_distances.min():.3f}, max={batch_distances.max():.3f}, mean={batch_distances.mean():.3f}")
        else:
            print(f"  Warning: Failed to generate pairs for batch {batch_idx + 1}")
    
    # Combine all pairs
    if all_start_pairs:
        final_start_pairs = torch.cat(all_start_pairs, dim=0)
        final_goal_pairs = torch.cat(all_goal_pairs, dim=0)
        
        print(f"\nFinal results:")
        print(f"  Total pairs generated: {total_generated}/{total_pairs}")
        print(f"  Success rate: {total_generated/total_pairs*100:.1f}%")
        
        # Save to file
        save_data = {
            'start_states': final_start_pairs.cpu(),
            'goal_states': final_goal_pairs.cpu(),
            'metadata': {
                'total_pairs': total_generated,
                'min_distance_threshold': min_distance_threshold,
                'max_distance_threshold': max_distance_threshold,
                'world_file': world_file,
                'robot_file': robot_file,
                'dof': final_start_pairs.shape[1]
            }
        }
        
        torch.save(save_data, output_file)
        print(f"  Saved pairs to: {output_file}")
        print(f"  File size: {os.path.getsize(output_file) / (1024*1024):.1f} MB")
        
        return output_file
    else:
        print("Error: No pairs were successfully generated!")
        return None


def load_start_goal_pairs(pairs_file):
    """
    Load start/goal pairs from saved file.
    
    Returns:
        tuple: (start_states, goal_states, metadata)
    """
    data = torch.load(pairs_file)
    return data['start_states'], data['goal_states'], data['metadata']


def validate_pairs_file(pairs_file):
    """
    Validate and print statistics about a saved pairs file.
    """
    if not os.path.exists(pairs_file):
        print(f"File not found: {pairs_file}")
        return False
    
    try:
        start_states, goal_states, metadata = load_start_goal_pairs(pairs_file)
        
        print(f"Pairs file validation: {pairs_file}")
        print(f"  Total pairs: {len(start_states)}")
        print(f"  DOF: {start_states.shape[1]}")
        print(f"  Distance constraints: {metadata['min_distance_threshold']:.2f} - {metadata['max_distance_threshold']:.2f}")
        print(f"  Robot: {metadata['robot_file']}")
        print(f"  World: {metadata['world_file']}")
        
        # Calculate actual distance statistics
        distances = torch.norm(start_states - goal_states, dim=-1)
        print(f"  Actual distances: min={distances.min():.3f}, max={distances.max():.3f}, mean={distances.mean():.3f}")
        print(f"  File size: {os.path.getsize(pairs_file) / (1024*1024):.1f} MB")
        
        return True
    except Exception as e:
        print(f"Error validating file: {e}")
        return False


if __name__ == "__main__":
    # Test with smaller number first
    output_file = generate_start_goal_pairs(
        total_pairs=1_000_000,   # Start with 10K for testing
        batch_size=10_000,
        min_distance_threshold=0.1,
        max_distance_threshold=10.0,
        # Foramt: franka_table_dist_0.1-10.0_pairs_1000000.pt
    )
    
    if output_file:
        print(f"\n{'='*50}")
        validate_pairs_file(output_file)