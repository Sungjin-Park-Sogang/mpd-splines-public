#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#

# Standard Library

# Third Party
import torch

# CuRobo
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.util.usd_helper import UsdHelper
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig

def generate_collision_free_random_states(motion_gen, curobo_fn, max_attempts=100):
    """
    Generate collision-free random start and goal joint states.
    
    Args:
        motion_gen: CuRobo MotionGen instance
        curobo_fn: RobotWorld instance for collision checking
        max_attempts: Maximum attempts to find valid states
    
    Returns:
        tuple: (start_state, goal_state) as JointState objects
    """
    # Method 1: Use RobotWorld's built-in collision-aware sampling
    try:
        # Sample 2 collision-free joint configurations
        valid_configs = curobo_fn.sample(n=2, mask_valid=True)
        
        if valid_configs.shape[0] >= 2:
            start_state = JointState.from_position(valid_configs[0].view(1, -1))
            goal_state = JointState.from_position(valid_configs[1].view(1, -1))
            return start_state, goal_state
    except Exception as e:
        print(f"Built-in sampling failed: {e}")
    
    # Method 2: Manual sampling with collision checking
    joint_limits = motion_gen.kinematics.get_joint_limits()
    lower_limits = joint_limits.position[0]
    upper_limits = joint_limits.position[1]
    
    valid_states = []
    attempts = 0
    
    while len(valid_states) < 2 and attempts < max_attempts:
        # Generate random joint configuration
        random_q = lower_limits + torch.rand_like(lower_limits) * (upper_limits - lower_limits)
        random_q_batch = random_q.view(1, -1)  # (1, dof)
        
        # Check for collisions
        try:
            is_valid = curobo_fn.validate(random_q_batch)
            if is_valid.any():  # If any configuration in batch is valid
                valid_states.append(JointState.from_position(random_q_batch))
                print(f"Found valid state {len(valid_states)}/2 after {attempts+1} attempts")
        except Exception as e:
            print(f"Validation failed: {e}")
        
        attempts += 1
    
    if len(valid_states) < 2:
        print(f"Warning: Could not find 2 collision-free states after {max_attempts} attempts")
        # Fallback to original random generation
        random_start = lower_limits + torch.rand_like(lower_limits) * (upper_limits - lower_limits)
        random_goal = lower_limits + torch.rand_like(lower_limits) * (upper_limits - lower_limits)
        return (JointState.from_position(random_start.view(1, -1)), 
                JointState.from_position(random_goal.view(1, -1)))
    
    return valid_states[0], valid_states[1]

def demo_motion_gen(js=True):
    # Standard Library
    import torch
    PLOT = True
    tensor_args = TensorDeviceType()
    # world_file = "collision_table.yml"
    world_file = "collision_test.yml"

    robot_file = "franka.yml"
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_file,
        world_file,
        tensor_args,
        interpolation_dt=0.01,
        # trajopt_dt=0.15,
        # velocity_scale=0.1,
        use_cuda_graph=True,
        # finetune_dt_scale=2.5,
        interpolation_steps=10000,
    )

    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()

    # motion_gen.warmup(enable_graph=True, warmup_js_trajopt=js, parallel_finetune=True)
    # robot_cfg = load_yaml(join_path(get_robot_configs_path(), robot_file))["robot_cfg"]
    # robot_cfg = RobotConfig.from_dict(robot_cfg, tensor_args)
    retract_cfg = motion_gen.get_retract_config()
    
    # Create RobotWorld for collision checking
    config = RobotWorldConfig.load_from_config(robot_file, world_file, pose_weight=[1, 1, 1, 1])
    curobo_fn = RobotWorld(config)
    
    # Generate collision-free random start and goal states
    print("Generating collision-free random start and goal states...")
    start_state, goal_state = generate_collision_free_random_states(motion_gen, curobo_fn)
    
    # Compute retract pose for non-js planning
    state = motion_gen.rollout_fn.compute_kinematics(goal_state)
    retract_pose = Pose(state.ee_pos_seq.squeeze(), quaternion=state.ee_quat_seq.squeeze())
    
    print(f"Start joints: {start_state.position.squeeze()}")
    print(f"Goal joints: {goal_state.position.squeeze()}")
    
    # Validate the generated states
    start_valid = curobo_fn.validate(start_state.position)
    goal_valid = curobo_fn.validate(goal_state.position)
    print(f"Start state collision-free: {start_valid.item()}")
    print(f"Goal state collision-free: {goal_valid.item()}")
    if js:
        result = motion_gen.plan_single_js(
            start_state,
            goal_state,
            MotionGenPlanConfig(max_attempts=1, time_dilation_factor=0.5),
        )
    else:
        result = motion_gen.plan_single(
            start_state,
            retract_pose,
            MotionGenPlanConfig(
                max_attempts=1,
                timeout=5,
                time_dilation_factor=0.5,
            ),
        )
        new_result = result.clone()
        new_result.retime_trajectory(0.5, create_interpolation_buffer=True)
        print(new_result.optimized_dt, new_result.motion_time, result.motion_time)
    print(
        "Trajectory Generated: ",
        result.success,
        result.solve_time,
        result.status,
        result.optimized_dt,
    )

    world_model = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), world_file))
    )
    save_name = "temp"
    UsdHelper.write_trajectory_animation_with_robot_usd(
        robot_model_file="franka.yml",
        world_model=world_model,
        q_start = result.interpolated_plan[0],
        q_traj=result.interpolated_plan,
        dt=0.02,
        visualize_robot_spheres=False,
        save_path=save_name + ".usd",
        base_frame="/" + save_name,
    )
    print("Trajectory is saved as usd file.")

    print("Trajectory: ")
    print(result.interpolated_plan.position.unsqueeze(0))

    # collision distance를 구하기 (RobotWorld는 이미 생성되었음)
    d_world, d_self = curobo_fn.get_world_self_collision_distance_from_joint_trajectory(result.interpolated_plan.position.unsqueeze(0))

    print(f"Collision distance:\n{d_world}")


def demo_motion_gen_batch():
    tensor_args = TensorDeviceType()
    world_file = "collision_cubby.yml"
    robot_file = "franka.yml"
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
    robot_cfg = load_yaml(join_path(get_robot_configs_path(), robot_file))["robot_cfg"]
    robot_cfg = RobotConfig.from_dict(robot_cfg, tensor_args)
    retract_cfg = motion_gen.get_retract_config()
    state = motion_gen.rollout_fn.compute_kinematics(
        JointState.from_position(retract_cfg.view(1, -1))
    )

    retract_pose = Pose(state.ee_pos_seq.squeeze(), quaternion=state.ee_quat_seq.squeeze())
    start_state = JointState.from_position(retract_cfg.view(1, -1) + 0.6)

    retract_pose = retract_pose.repeat_seeds(2)
    retract_pose.position[0, 0] = -0.3
    result = motion_gen.plan_batch(
        start_state.repeat_seeds(2),
        retract_pose,
        MotionGenPlanConfig(
            max_attempts=5, enable_graph=False, enable_graph_attempt=1, enable_opt=True
        ),
    )
    traj = result.optimized_plan.position.view(2, -1, 7)  # optimized plan
    print("Trajectory Generated: ", result.success)


if __name__ == "__main__":
    setup_curobo_logger("error")
    
    demo_motion_gen(js=True)
    # demo_motion_gen_batch()
    

    
