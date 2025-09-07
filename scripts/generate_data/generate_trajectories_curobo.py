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

def sample_and_plan_batch_trajectories(num_pairs=20, max_attempts=100, planner_timeout=10.0):
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

    robot_world_config = RobotWorldConfig.load_from_config(
        robot_file, world_file, tensor_args=tensor_args, pose_weight=[1, 1, 1, 1]    
    )
    curobo_fn = RobotWorld(robot_world_config)

    start_joint_tensors = curobo_fn.sample_trajectory(batch=num_pairs, horizon=1, mask_valid=True)
    goal_joint_tensors = curobo_fn.sample_trajectory(batch=num_pairs, horizon=1, mask_valid=True)

    # Convert start states to JointState objects
    start_joint_states = JointState.from_position(start_joint_tensors.squeeze(-2))  # Remove horizon dimension [batch, 1, dof] -> [batch, dof]
    
    # Convert goal joint states to poses via forward kinematics
    goal_joint_states_temp = JointState.from_position(goal_joint_tensors.squeeze(-2))
    fk_result = motion_gen.rollout_fn.compute_kinematics(goal_joint_states_temp)
    
    # Create Pose objects from FK result
    goal_poses = Pose(
        position=fk_result.ee_pos_seq,      # [batch, 3]
        quaternion=fk_result.ee_quat_seq    # [batch, 4] (w, x, y, z)
    )

    print("Start joint states shape:", start_joint_states.position.shape)
    print("Goal poses shape; ", goal_poses.shape)

    result = motion_gen.plan_batch(
        start_joint_states,
        goal_poses,
        MotionGenPlanConfig(
            max_attempts=5, enable_graph=False, enable_graph_attempt=1, enable_opt=True
        ),
    )
    traj = result.optimized_plan.position.view(num_pairs, -1, 7)  # optimized plan
    print("Trajectory Generated: ", result.success)
    

if __name__ == "__main__":
    # setup_curobo_logger("error")
    # run_experiment(experiment)
    sample_and_plan_batch_trajectories()