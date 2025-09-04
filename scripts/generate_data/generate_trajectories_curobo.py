import os
import pathlib
import pickle
import random
import time
from copy import deepcopy, copy
import h5py
import numpy as np
import torch
import yaml
from tqdm import tqdm

from experiment_launcher import single_experiment_yaml, run_experiment
from experiment_launcher.utils import fix_random_seed
from mpd.paths import DATA_GENERATION_CFGS_PATH
from torch_robotics import environments, robots
from torch_robotics.torch_utils.torch_timer import TimerCUDA
from torch_robotics.torch_utils.torch_utils import to_numpy, DEFAULT_TENSOR_ARGS

from scipy.spatial.transform import Rotation

# CuRobo imports
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig


class GenerateDataCuRobo:
    def __init__(
        self,
        env_id,
        robot_id,
        min_distance_robot_env=0.02,
        tensor_args=DEFAULT_TENSOR_ARGS,
        debug=False,
        env_tr=None,
        robot_tr=None,
        world_file="collision_test.yml",
        robot_file="franka.yml",
        interpolate_num=128,  # Fixed trajectory length for compatibility
        save_usd=False,  # Enable USD trajectory visualization
        usd_save_path="trajectory_viz",  # USD file save path (without extension)
        **kwargs,
    ):
        self.tensor_args = tensor_args
        self.debug = debug
        self.interpolate_num = interpolate_num
        self.save_usd = save_usd
        self.usd_save_path = usd_save_path
        
        # -------------------------------- Load env, robot, task ---------------------------------
        # Environment (keep for compatibility with original pose region configs)
        if env_tr is not None:
            self.env_tr = deepcopy(env_tr)
        else:
            env_class = getattr(environments, env_id)
            self.env_tr = env_class(
                precompute_sdf_obj_fixed=False, precompute_sdf_obj_extra=False, tensor_args=tensor_args
            )

        # Robot from torch_robotics (keep for compatibility)
        if robot_tr is not None:
            self.robot_tr = deepcopy(robot_tr)
        else:
            robot_class_tr = getattr(robots, robot_id)
            self.robot_tr = robot_class_tr(tensor_args=tensor_args, **kwargs)

        # --------------------------------------------------------------------------------------------
        # Setup CuRobo MotionGen
        self.world_file = world_file
        self.robot_file = robot_file
        
        # Normalize device for CuRobo/Warp: ensure explicit CUDA index (e.g., 'cuda:0')
        base_dev = self.tensor_args.get("device", torch.device("cuda:0"))
        curobo_device = None
        if isinstance(base_dev, torch.device):
            if base_dev.type == "cuda":
                idx = base_dev.index
                if idx is None:
                    idx = torch.cuda.current_device() if torch.cuda.is_available() else 0
                curobo_device = f"cuda:{idx}"
            else:
                curobo_device = "cpu"
        else:
            # string path
            dev_str = str(base_dev)
            if dev_str.startswith("cuda") and ":" not in dev_str:
                curobo_device = "cuda:0"
            else:
                curobo_device = dev_str

        curobo_tensor_args = TensorDeviceType(
            device=curobo_device, dtype=self.tensor_args.get("dtype", torch.float32)
        )
        print(f"CuRobo tensor_args device: {curobo_tensor_args.device}, dtype: {curobo_tensor_args.dtype}")
        
        # Load CuRobo motion generator with original settings
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_file,
            world_file,
            curobo_tensor_args,
            interpolation_dt=0.01,
            use_cuda_graph=True,  # Restore original setting
            interpolation_steps=10000,
        )
        
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.warmup()
        
        # Create RobotWorld for collision checking and state validation
        robot_world_config = RobotWorldConfig.load_from_config(
            robot_file, world_file, tensor_args=curobo_tensor_args, pose_weight=[1, 1, 1, 1]
        )
        self.curobo_fn = RobotWorld(robot_world_config)
        
        # Get joint limits for state generation
        self.joint_limits = self.motion_gen.kinematics.get_joint_limits()
        self.lower_limits = self.joint_limits.position[0]
        self.upper_limits = self.joint_limits.position[1]

    def generate_collision_free_random_states(self, max_attempts=100):
        """
        Generate collision-free random start and goal joint states.
        
        Args:
            max_attempts: Maximum attempts to find valid states
        
        Returns:
            tuple: (start_state, goal_state) as JointState objects
        """
        # Method 1: Use RobotWorld's built-in collision-aware sampling
        try:
            # Sample 2 collision-free joint configurations
            valid_configs = self.curobo_fn.sample(n=2, mask_valid=True)
            
            if valid_configs.shape[0] >= 2:
                start_state = JointState.from_position(valid_configs[0].view(1, -1))
                goal_state = JointState.from_position(valid_configs[1].view(1, -1))
                return start_state, goal_state
        except Exception as e:
            if self.debug:
                print(f"Built-in sampling failed: {e}")
        
        # Method 2: Manual sampling with collision checking
        valid_states = []
        attempts = 0
        
        while len(valid_states) < 2 and attempts < max_attempts:
            # Generate random joint configuration
            random_q = self.lower_limits + torch.rand_like(self.lower_limits) * (self.upper_limits - self.lower_limits)
            random_q_batch = random_q.view(1, -1)  # (1, dof)
            
            # Check for collisions
            try:
                is_valid = self.curobo_fn.validate(random_q_batch)
                if is_valid.any():  # If any configuration in batch is valid
                    valid_states.append(JointState.from_position(random_q_batch))
                    if self.debug:
                        print(f"Found valid state {len(valid_states)}/2 after {attempts+1} attempts")
            except Exception as e:
                if self.debug:
                    print(f"Validation failed: {e}")
            
            attempts += 1
        
        if len(valid_states) < 2:
            if self.debug:
                print(f"Warning: Could not find 2 collision-free states after {max_attempts} attempts")
            # Fallback to original random generation
            random_start = self.lower_limits + torch.rand_like(self.lower_limits) * (self.upper_limits - self.lower_limits)
            random_goal = self.lower_limits + torch.rand_like(self.lower_limits) * (self.upper_limits - self.lower_limits)
            return (JointState.from_position(random_start.view(1, -1)), 
                    JointState.from_position(random_goal.view(1, -1)))
        
        return valid_states[0], valid_states[1]

    def interpolate_trajectory_to_fixed_length(self, trajectory):
        """
        Interpolate variable-length trajectory to fixed length.
        
        Args:
            trajectory: torch.Tensor or np.ndarray of shape (original_length, 7) - joint positions
            
        Returns:
            np.ndarray of shape (interpolate_num, 7) - interpolated trajectory
        """
        from scipy.interpolate import interp1d
        
        # Convert to numpy for easier handling
        if isinstance(trajectory, torch.Tensor):
            traj_numpy = trajectory.detach().cpu().numpy()
        else:
            traj_numpy = trajectory
            
        original_length = traj_numpy.shape[0]
        dof = traj_numpy.shape[1]
        
        # If already at target length, return as-is
        if original_length == self.interpolate_num:
            return traj_numpy
        
        # Create time indices for original and target trajectories
        original_indices = np.linspace(0, 1, original_length)
        target_indices = np.linspace(0, 1, self.interpolate_num)
        
        # Interpolate each joint dimension separately
        interpolated_trajectory = np.zeros((self.interpolate_num, dof))
        
        for joint_idx in range(dof):
            # Create interpolation function for this joint
            interp_func = interp1d(
                original_indices, 
                traj_numpy[:, joint_idx], 
                kind='linear',
                bounds_error=False,
                fill_value='extrapolate'
            )
            
            # Interpolate to target length
            interpolated_trajectory[:, joint_idx] = interp_func(target_indices)
        
        return interpolated_trajectory

    def save_trajectory_usd(self, result_or_traj, trajectory_id=0):
        """
        Save trajectory as USD file for visualization.
        Accepts either a MotionGenResult (with interpolated_plan) or a joint trajectory [T,dof].
        
        Args:
            result_or_traj: MotionGenResult or trajectory array/tensor/JointState
            trajectory_id: ID for naming the USD file
        """
        if not self.save_usd:
            return

        try:
            from mpd.plotting.visualize_trajectories_curobo import (
                save_motiongen_result_usd,
                save_trajectory_usd as save_qtraj_usd,
            )

            # Generate save name with trajectory ID
            save_name = f"{self.usd_save_path}_{trajectory_id:04d}"

            print(f"Saving trajectory USD: {save_name}.usd")

            # Dispatch based on input type
            if hasattr(result_or_traj, "interpolated_plan"):
                # MotionGenResult
                save_motiongen_result_usd(
                    result_or_traj,
                    robot_file=self.robot_file,
                    world_file=self.world_file,
                    save_path=save_name + ".usd",
                    dt=0.02,
                    visualize_robot_spheres=False,
                    base_frame="/" + save_name,
                )
            else:
                # Joint trajectory-like input
                save_qtraj_usd(
                    result_or_traj,
                    robot_file=self.robot_file,
                    world_file=self.world_file,
                    save_path=save_name + ".usd",
                    dt=0.02,
                    visualize_robot_spheres=False,
                    base_frame="/" + save_name,
                )

            print(f"✓ Trajectory saved as USD: {save_name}.usd")

        except Exception as e:
            print(f"Failed to save trajectory USD: {e}")
            if self.debug:
                import traceback
                traceback.print_exc()

    def get_start_and_goal_states(
        self,
        q_pos_start=None,
        ee_pose_start=None,
        q_pos_goal=None,
        ee_pose_goal=None,
        n_joint_position_goal=1,
        sample_joint_position_goals_with_same_ee_pose=False,
        min_distance_q_pos_start_goal=0.0,
        debug=False,
    ):
        """
        Generate or validate start and goal states for trajectory planning.
        Similar to the original pb_ompl implementation but using CuRobo.
        """
        if q_pos_start is not None:
            q_pos_start_tensor = torch.tensor(q_pos_start, **self.tensor_args).view(1, -1)
            assert self.curobo_fn.validate(q_pos_start_tensor).any(), f"q_pos_start={q_pos_start} is NOT valid"
        if q_pos_goal is not None:
            q_pos_goal_tensor = torch.tensor(q_pos_goal, **self.tensor_args).view(1, -1)
            assert self.curobo_fn.validate(q_pos_goal_tensor).any(), f"q_pos_goal={q_pos_goal} is NOT valid"

        if q_pos_start is not None and q_pos_goal is not None:
            if np.linalg.norm(q_pos_start - q_pos_goal) < min_distance_q_pos_start_goal:
                print(f"q_pos_start={q_pos_start} and q_pos_goal={q_pos_goal} are too close")
                return [], []

        for i in tqdm(range(1000), disable=True):  # max tries
            if q_pos_start is not None:
                q_pos_start_tmp = copy(q_pos_start)
            else:
                if ee_pose_start is not None:
                    # Use IK to find joint configuration for given end-effector pose
                    try:
                        ee_pose_tensor = torch.tensor(ee_pose_start, **self.tensor_args).view(1, 4, 4)
                        pose = Pose.from_matrix(ee_pose_tensor)
                        ik_result = self.motion_gen.compute_ik(pose)
                        if ik_result.success.any():
                            q_pos_start_tmp = to_numpy(ik_result.js_solution.position.squeeze())
                        else:
                            raise ValueError("IK failed for start pose")
                    except Exception as e:
                        if debug:
                            print(f"IK failed for start pose: {e}")
                        continue
                else:
                    # Generate random collision-free start state
                    start_state, _ = self.generate_collision_free_random_states()
                    q_pos_start_tmp = to_numpy(start_state.position.squeeze())

            if q_pos_goal is not None:
                q_pos_goal_tmp = copy(q_pos_goal)
            else:
                if ee_pose_goal is not None:
                    # Use IK to find joint configuration for given end-effector pose
                    try:
                        ee_pose_tensor = torch.tensor(ee_pose_goal, **self.tensor_args).view(1, 4, 4)
                        pose = Pose.from_matrix(ee_pose_tensor)
                        ik_result = self.motion_gen.compute_ik(pose)
                        if ik_result.success.any():
                            q_pos_goal_tmp = to_numpy(ik_result.js_solution.position.squeeze())
                        else:
                            raise ValueError("IK failed for goal pose")
                    except Exception as e:
                        if debug:
                            print(f"IK failed for goal pose: {e}")
                        continue
                else:
                    # Generate random collision-free goal state
                    _, goal_state = self.generate_collision_free_random_states()
                    q_pos_goal_tmp = to_numpy(goal_state.position.squeeze())

            q_pos_goal_tmp_l = []
            # check if the distance between the start and goal states is greater than min_distance_q_pos_start_goal
            if np.linalg.norm(q_pos_start_tmp - q_pos_goal_tmp) < min_distance_q_pos_start_goal:
                if debug:
                    print(f"{i}")
                continue

            if sample_joint_position_goals_with_same_ee_pose:
                # To generate several trajectories from the same joint position start state to
                # multiple joint position goals, resample a new goal state with the same end-effector pose
                start_state = JointState.from_position(torch.tensor(q_pos_goal_tmp, **self.tensor_args).view(1, -1))
                fk_result = self.motion_gen.rollout_fn.compute_kinematics(start_state)
                ee_pose_goal_tensor = torch.eye(4, **self.tensor_args).view(1, 4, 4)
                ee_pose_goal_tensor[0, :3, 3] = fk_result.ee_pos_seq.squeeze()
                ee_pose_goal_tensor[0, :3, :3] = fk_result.ee_rot_seq.squeeze()
                pose_goal = Pose.from_matrix(ee_pose_goal_tensor)
                
                for _ in range(n_joint_position_goal):
                    try:
                        ik_result = self.motion_gen.compute_ik(pose_goal)
                        if ik_result.success.any():
                            q_pos_tmp = to_numpy(ik_result.js_solution.position.squeeze())
                            q_pos_goal_tmp_l.append(q_pos_tmp)
                    except:
                        pass
                assert len(q_pos_goal_tmp_l) > 0, f"len(joint_position_goal_l)={len(q_pos_goal_tmp_l)} == 0"
            else:
                q_pos_goal_tmp_l = [q_pos_goal_tmp] * n_joint_position_goal

            break

        # start state is the same for all goals
        q_pos_start_l = [q_pos_start_tmp] * len(q_pos_goal_tmp_l)

        return q_pos_start_l, q_pos_goal_tmp_l

    def run(
        self,
        num_trajectories,
        joint_position_start,
        joint_position_goal,
        planner_allowed_time=4.0,
        interpolate_num=250,
        simplify_path=True,
        fit_bspline=False,
        bspline_num_control_points=20,
        bspline_degree=5,
        bspline_zero_vel_at_start_and_goal=True,
        bspline_zero_acc_at_start_and_goal=True,
        max_tries=1000,
        debug=False,
    ):
        """
        Run trajectory planning using CuRobo MotionGen.
        """
        assert max_tries >= num_trajectories, (
            f"max_tries must be greater than the number of desired trajectories."
            f" max_tries={max_tries} < num_trajectories={num_trajectories}"
        )

        num_trajectories_generated = 0
        results_dict = {}

        # Handle None values by generating random collision-free states
        if joint_position_start is None or joint_position_goal is None:
            if debug:
                print("Generating random collision-free start and goal states...")
            random_start, random_goal = self.generate_collision_free_random_states()
            
            if joint_position_start is None:
                start_state = random_start
            else:
                start_state = JointState.from_position(torch.tensor(joint_position_start, **self.tensor_args).view(1, -1))
                
            if joint_position_goal is None:
                goal_state = random_goal
            else:
                goal_state = JointState.from_position(torch.tensor(joint_position_goal, **self.tensor_args).view(1, -1))
        else:
            # Convert numpy arrays to JointState objects
            start_state = JointState.from_position(torch.tensor(joint_position_start, **self.tensor_args).view(1, -1))
            goal_state = JointState.from_position(torch.tensor(joint_position_goal, **self.tensor_args).view(1, -1))

        # planning
        if debug:
            start_pos = joint_position_start if joint_position_start is not None else start_state.position.squeeze().cpu().numpy()
            goal_pos = joint_position_goal if joint_position_goal is not None else goal_state.position.squeeze().cpu().numpy()
            print(f"joint_position_start: {start_pos}")
            print(f"joint_position_goal: {goal_pos}")
            
        for i in range(max_tries):
            s_time = time.perf_counter()
            
            # Plan trajectory using CuRobo
            plan_config = MotionGenPlanConfig(
                max_attempts=1,
                timeout=planner_allowed_time,
                time_dilation_factor=0.5,
            )
            
            result = self.motion_gen.plan_single_js(
                start_state,
                goal_state,
                plan_config,
            )
            
            planning_time = time.perf_counter() - s_time
            if debug:
                print(f"planning time: {planning_time:.3f} s")

            if result.success.any():
                # Extract trajectory data and apply fixed-length interpolation
                original_trajectory = to_numpy(result.interpolated_plan.position)
                sol_path = self.interpolate_trajectory_to_fixed_length(original_trajectory)
                
                results_dict_plan = {
                    "success": True,
                    "solve_time": planning_time,
                    "sol_path": sol_path,
                    "sol_path_interpolated": sol_path,  # Fixed-length interpolated path
                    "sol_path_simplified": sol_path,   # Same as interpolated for consistency
                    "optimized_dt": to_numpy(result.optimized_dt) if hasattr(result, 'optimized_dt') else 0.01,
                    "motion_time": to_numpy(result.motion_time) if hasattr(result, 'motion_time') else self.interpolate_num * 0.01,
                    "interpolation_num": self.interpolate_num,  # Fixed length
                    "bspline_params": None,  # B-spline fitting not implemented yet
                }
                
                # Add collision distance information using fixed-length trajectory
                try:
                    # Convert fixed-length trajectory back to tensor for collision checking
                    sol_path_tensor = torch.tensor(sol_path, **self.tensor_args).unsqueeze(0)
                    d_world, d_self = self.curobo_fn.get_world_self_collision_distance_from_joint_trajectory(
                        sol_path_tensor
                    )
                    results_dict_plan["collision_distance_world"] = to_numpy(d_world.squeeze())
                    results_dict_plan["collision_distance_self"] = to_numpy(d_self.squeeze())
                except Exception as e:
                    if debug:
                        print(f"Failed to compute collision distances: {e}")
                    results_dict_plan["collision_distance_world"] = None
                    results_dict_plan["collision_distance_self"] = None
                
                # Save trajectory as USD file if enabled
                self.save_trajectory_usd(result, num_trajectories_generated)
                
                results_dict[num_trajectories_generated] = results_dict_plan

                num_trajectories_generated += 1
                if debug:
                    print(f"num_trajectories_generated: {num_trajectories_generated}/{num_trajectories}")

            if num_trajectories_generated >= num_trajectories:
                break

        return results_dict

    def terminate(self):
        """
        Cleanup resources (CuRobo doesn't require explicit cleanup like PyBullet)
        """
        pass


def get_random_pose_from_region(pose_region):
    """
    Generate random end-effector pose from pose region specification.
    Same as original implementation.
    """
    ee_pose_target = np.eye(4)
    # random position
    ee_pose_target[0, 3] = np.random.choice(
        [np.random.rand(1).item() * (high - low) + low for low, high in pose_region["translation"]["x"]]
    )
    ee_pose_target[1, 3] = np.random.choice(
        [np.random.rand(1).item() * (high - low) + low for low, high in pose_region["translation"]["y"]]
    )
    ee_pose_target[2, 3] = np.random.choice(
        [np.random.rand(1).item() * (high - low) + low for low, high in pose_region["translation"]["z"]]
    )
    # random orientation
    rotation_base = Rotation.from_matrix(np.array(pose_region["rotation"]["base"]))
    rotation_random_around_base = Rotation.from_euler(
        "xyz",
        [
            np.random.choice(
                [np.random.rand(1).item() * (high - low) + low for low, high in pose_region["rotation"]["x"]]
            ),
            np.random.choice(
                [np.random.rand(1).item() * (high - low) + low for low, high in pose_region["rotation"]["y"]]
            ),
            np.random.choice(
                [np.random.rand(1).item() * (high - low) + low for low, high in pose_region["rotation"]["z"]]
            ),
        ],
        degrees=True,
    )
    rotation_target = rotation_base * rotation_random_around_base
    ee_pose_target[:3, :3] = rotation_target.as_matrix()
    return ee_pose_target


def get_random_ee_pose_from_cfg_file(env_id, robot_id, cfg_file_path):
    """
    Generate random end-effector poses from configuration file.
    Same as original implementation.
    """
    # create a random start and goal end-effector pose based on a config file describing the target pose limits
    with open(cfg_file_path, "r") as f:
        cfg_ee = yaml.load(f, Loader=yaml.Loader)

    assert (
        env_id == cfg_ee["env_id"] or env_id == cfg_ee["env_id"] + "ExtraObjects"
    ), f"env_id mismatch: {env_id} != {cfg_ee['env_id']}"
    assert robot_id == cfg_ee["robot_id"], f"robot_id mismatch: {robot_id} != {cfg_ee['robot_id']}"

    pose_region_ids = list(cfg_ee["pose_regions"].keys())

    ee_pose_start = None

    # randomly move between pose regions or go to one of the pose regions
    if cfg_ee["move_between_pose_regions"] and random.choice([True, False]):
        # move between two pose regions
        assert len(pose_region_ids) >= 2, f"len(pose_region_ids)={len(pose_region_ids)} < 2"
        pose_region_id_start, pose_region_id_goal = np.random.choice(pose_region_ids, size=2, replace=False)
        ee_pose_start = get_random_pose_from_region(cfg_ee["pose_regions"][pose_region_id_start])
        ee_pose_goal = get_random_pose_from_region(cfg_ee["pose_regions"][pose_region_id_goal])
    else:
        # move to one of the pose regions
        pose_region_id = np.random.choice(pose_region_ids)
        ee_pose_goal = get_random_pose_from_region(cfg_ee["pose_regions"][pose_region_id])

    return ee_pose_start, ee_pose_goal


def generate_trajectories_run(
    generate_data_curobo_worker,
    env_id,
    robot_id,
    min_distance_robot_env,
    task_id,
    joint_position_start,
    joint_position_goal,
    planner_allowed_time,
    interpolate_num,
    simplify_path,
    fit_bspline,
    bspline_num_control_points,
    bspline_degree,
    bspline_zero_vel_at_start_and_goal,
    bspline_zero_acc_at_start_and_goal,
    tensor_args,
    world_file,
    robot_file,
    num_trajectories=1,
    max_tries=1,
    debug=False,
    save_usd=False,
    usd_save_path="trajectory_viz",
):
    """
    Worker function for parallel trajectory generation using CuRobo.
    """
    if generate_data_curobo_worker is None:
        # For multi-threading, create a new worker
        generate_data_curobo_worker = GenerateDataCuRobo(
            env_id,
            robot_id,
            min_distance_robot_env=min_distance_robot_env,
            tensor_args=tensor_args,
            world_file=world_file,
            robot_file=robot_file,
            interpolate_num=interpolate_num,  # Pass interpolate_num parameter
            save_usd=save_usd,  # Pass USD save option
            usd_save_path=usd_save_path,
            debug=debug,
        )

    start_planning_time = time.time()  # start the timer only after the worker is created
    results_dict = generate_data_curobo_worker.run(
        num_trajectories=num_trajectories,  # each worker generates one trajectory
        max_tries=max_tries,
        joint_position_start=joint_position_start,
        joint_position_goal=joint_position_goal,
        planner_allowed_time=planner_allowed_time,
        interpolate_num=interpolate_num,
        fit_bspline=fit_bspline,
        simplify_path=simplify_path,
        bspline_num_control_points=bspline_num_control_points,
        bspline_degree=bspline_degree,
        bspline_zero_vel_at_start_and_goal=bspline_zero_vel_at_start_and_goal,
        bspline_zero_acc_at_start_and_goal=bspline_zero_acc_at_start_and_goal,
        debug=debug,
    )
    end_planning_time = time.time()

    # Add the task_id to all the results
    for k, v in results_dict.items():
        results_dict[k]["task_id"] = task_id

    # Update timing information
    results_dict.update(
        {
            "run_planning_time": end_planning_time - start_planning_time,
            "start_planning_time": start_planning_time,
            "end_planning_time": end_planning_time,
        }
    )

    return results_dict


@single_experiment_yaml
def experiment(
    ############################################################################
    # env_id: str = 'EnvDense2D',
    # env_id: str = 'EnvSimple2D',
    # env_id: str = 'EnvNarrowPassageDense2D',
    # robot_id: str = 'RobotPointMass2D',
    # env_id: str = 'EnvPlanar2Link',
    # robot_id: str = 'RobotPlanar2Link',
    # env_id: str = 'EnvPlanar4Link',
    # robot_id: str = 'RobotPlanar4Link',
    # env_id: str = 'EnvSpheres3D',
    # env_id: str = 'EnvSpheres3DExtraObjectsV00',
    # env_id: str = 'EnvTableShelf',
    # env_id: str = 'EnvPilars3D',
    # robot_id: str = 'RobotPanda',
    env_id: str = "EnvWarehouse",
    robot_id: str = "RobotPanda",
    ############################################################################
    start_task_id: int = 49400,
    # num_tasks: int = 5,
    num_tasks: int = 100,
    num_trajectories_per_task: int = 1,
    ############################################################################
    sample_joint_position_goals_with_same_ee_pose: bool = False,
    cfg_file: str = "None",
    # cfg_file: str = "EnvTableShelf-RobotPanda.yaml",
    # cfg_file: str = "EnvWarehouse-RobotPanda.yaml",
    # cfg_file: str = "EnvWarehouse-RobotPanda_v01.yaml",
    ############################################################################
    min_distance_robot_env: float = 0.00,
    min_distance_q_pos_start_goal: float = 0.0,  # minimum distance between start and goal joint positions
    # CuRobo motion planner parameters
    planner_allowed_time: float = 10.0,
    # path simplification methods (CuRobo handles this internally)
    simplify_path: bool = True,
    # bspline parameters (not implemented yet)
    fit_bspline: bool = False,
    bspline_num_control_points: int = 12,
    bspline_degree: int = 5,
    bspline_zero_vel_at_start_and_goal: bool = True,
    bspline_zero_acc_at_start_and_goal: bool = True,
    interpolate_num: int = 128,  # number of waypoints to interpolate the path
    ############################################################################
    # CuRobo configuration files
    world_file: str = "collision_test.yml",
    robot_file: str = "franka.yml",
    ############################################################################
    # USD Visualization
    save_usd: bool = False,  # Enable USD trajectory visualization
    usd_save_path: str = "trajectory_viz",  # USD file save path (without extension)
    ############################################################################
    n_parallel_jobs: int = 1,  # Set to 1 to debug
    # n_parallel_jobs: int = os.cpu_count(),
    debug: bool = False,
    ############################################################################
    # MANDATORY
    seed: int = int(time.time()),
    # seed: int = 49400,
    results_dir: str = f"data/env-robot",
    ############################################################################
    **kwargs,
):
    fix_random_seed(seed)

    print(f"\n\n-------------------- Generating data with CuRobo --------------------")
    print(f"Env:   {env_id}")
    print(f"Robot: {robot_id}")
    print(f"World file: {world_file}")
    print(f"Robot file: {robot_file}")
    print(f"start_task_id:  {start_task_id}")
    print(f"num_tasks:  {num_tasks}")
    print(f"num_trajectories_per_task:  {num_trajectories_per_task}")
    print(f"\n\n---------------------------------------------------------------")

    # Let CuRobo automatically detect and use the best device
    print(f"CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"CUDA device count: {torch.cuda.device_count()}")
        print(f"Current CUDA device: {torch.cuda.current_device()}")
    
    # Use traditional tensor_args for torch_robotics compatibility
    tensor_args = {"device": "cuda" if torch.cuda.is_available() else "cpu", "dtype": torch.float32}
    print(f"torch_robotics tensor device: {tensor_args['device']}")

    ####################################################################################################################
    # Create the tasks - start and goal joint positions or end-effector poses
    q_pos_start = None
    ee_pose_start = None
    q_pos_goal = None
    ee_pose_goal = None

    generate_data_curobo_worker = GenerateDataCuRobo(
        env_id,
        robot_id,
        min_distance_robot_env=min_distance_robot_env,
        tensor_args=tensor_args,
        world_file=world_file,
        robot_file=robot_file,
        interpolate_num=interpolate_num,  # Pass interpolate_num parameter
        save_usd=save_usd,  # Pass USD save option
        usd_save_path=usd_save_path,
        debug=debug,
    )

    print("\nGenerating tasks (pose targets for batched planning)...")
    task_id_l = []
    start_q_l = []
    goal_pose_l = []  # list of 4x4 numpy arrays

    for i in tqdm(range(start_task_id, start_task_id + num_tasks)):

        # Reset per-task hints
        local_ee_pose_start = None
        local_ee_pose_goal = None

        if cfg_file != "None":
            # generate start and goal poses based on a config file
            cfg_file_path = os.path.join(DATA_GENERATION_CFGS_PATH, cfg_file)
            local_ee_pose_start, local_ee_pose_goal = get_random_ee_pose_from_cfg_file(
                env_id, robot_id, cfg_file_path
            )
            print(f"Config file exists.")

        # config 파일 사용안하는 경우
        else:
            print(f"Config file does not exist.")
            

        # Resolve start joint config
        q_start_np = None
        try:
            if q_pos_start is not None:
                q_start_np = np.asarray(q_pos_start).reshape(-1)
            elif local_ee_pose_start is not None:
                ee_pose_tensor = torch.tensor(local_ee_pose_start, **tensor_args).view(1, 4, 4)
                ik_res = generate_data_curobo_worker.motion_gen.compute_ik(Pose.from_matrix(ee_pose_tensor))
                if ik_res.success.any():
                    q_start_np = to_numpy(ik_res.js_solution.position.squeeze())
            if q_start_np is None:
                # fallback to random valid start
                s_state, _ = generate_data_curobo_worker.generate_collision_free_random_states()
                q_start_np = to_numpy(s_state.position.squeeze())
        except Exception as e:
            if debug:
                print(f"Failed to resolve start state for task {i}: {e}")
            continue

        # Resolve goal pose
        goal_pose_np = None
        try:
            if local_ee_pose_goal is not None:
                goal_pose_np = np.asarray(local_ee_pose_goal).reshape(4, 4)
            elif q_pos_goal is not None:
                # Convert target joint to EE pose via FK
                q_goal_tensor = torch.tensor(q_pos_goal, **tensor_args).view(1, -1)
                fk = generate_data_curobo_worker.motion_gen.rollout_fn.compute_kinematics(
                    JointState.from_position(q_goal_tensor)
                )
                T = np.eye(4, dtype=np.float32)
                T[:3, :3] = to_numpy(fk.ee_rot_seq.squeeze())
                T[:3, 3] = to_numpy(fk.ee_pos_seq.squeeze())
                goal_pose_np = T
            else:
                # fallback: sample a valid random goal joint and convert to pose
                _, g_state = generate_data_curobo_worker.generate_collision_free_random_states()
                fk = generate_data_curobo_worker.motion_gen.rollout_fn.compute_kinematics(g_state)
                T = np.eye(4, dtype=np.float32)
                T[:3, :3] = to_numpy(fk.ee_rot_seq.squeeze())
                T[:3, 3] = to_numpy(fk.ee_pos_seq.squeeze())
                goal_pose_np = T
        except Exception as e:
            if debug:
                print(f"Failed to resolve goal pose for task {i}: {e}")
            continue

        # Enforce min joint distance if joint goal known (optional)
        if q_pos_goal is not None:
            try:
                if np.linalg.norm(q_start_np - np.asarray(q_pos_goal).reshape(-1)) < min_distance_q_pos_start_goal:
                    if debug:
                        print(f"Skip task {i}: start/goal joints too close")
                    continue
            except Exception:
                pass

        # Replicate per requested trajectories per task
        rep = max(1, int(num_trajectories_per_task))
        task_id_l.extend([i] * rep)
        start_q_l.extend([q_start_np.copy() for _ in range(rep)])
        goal_pose_l.extend([goal_pose_np.copy() for _ in range(rep)])

    assert (
        len(task_id_l) == len(start_q_l) == len(goal_pose_l)
    ), f"len(task_id_l)={len(task_id_l)} != len(start_q_l)={len(start_q_l)}"
    print(f"\n----------\nPrepared {len(task_id_l)}/{num_tasks} start/goal pose pairs\n----------\n")

    # Early exit if no tasks prepared
    if len(task_id_l) == 0:
        print("No valid tasks were prepared. Writing empty dataset and timing stats.")
        pathlib.Path(results_dir).mkdir(parents=True, exist_ok=True)
        # Save empty timing stats
        with open(pathlib.Path(results_dir) / "timing_stats.pkl", "wb") as fp:
            pickle.dump(
                {
                    "num_plans": 0,
                    "batch_size": max(1, int(n_parallel_jobs)),
                    "wall_time": 0.0,
                    "planning_computation_time": 0.0,
                    "max_batch_runtime": 0.0,
                    "total_compute_time": 0.0,
                    "effective_speedup": 0.0,
                },
                fp,
            )
        # Save empty dataset file with metadata
        hf = h5py.File(os.path.join(results_dir, "dataset.hdf5"), "w")
        hf.attrs["num_trajectories_desired"] = num_tasks * num_trajectories_per_task
        hf.attrs["num_trajectories_generated"] = 0
        hf.close()
        print(f"Dataset saved to: {os.path.join(results_dir, 'dataset.hdf5')} (empty)")
        return

    ####################################################################################################################
    # Generate data via cuRobo batched planning (pose goals)
    with TimerCUDA() as t_generate_data:
        results_dict_l = []
        # Repurpose n_parallel_jobs as batch size for GPU planning
        batch_size = max(1, int(n_parallel_jobs))
        ta = generate_data_curobo_worker.motion_gen.tensor_args
        for b_start in tqdm(range(0, len(task_id_l), batch_size), disable=False):
            b_end = min(len(task_id_l), b_start + batch_size)
            task_ids_b = task_id_l[b_start:b_end]
            start_q_b = np.stack(start_q_l[b_start:b_end], axis=0)
            goal_pose_b = np.stack(goal_pose_l[b_start:b_end], axis=0)

            # Build inputs
            start_state = JointState.from_position(
                torch.as_tensor(start_q_b, device=ta.device, dtype=ta.dtype)
            )
            goal_pose = Pose.from_matrix(
                torch.as_tensor(goal_pose_b, device=ta.device, dtype=ta.dtype)
            )

            plan_config = MotionGenPlanConfig(
                max_attempts=1,
                timeout=planner_allowed_time,
                time_dilation_factor=0.5,
                enable_graph=False,
            )

            start_planning_time = time.time()
            result = generate_data_curobo_worker.motion_gen.plan_batch(start_state, goal_pose, plan_config)
            end_planning_time = time.time()

            # Collect per-sample outputs
            results_dict_run = {
                "run_planning_time": end_planning_time - start_planning_time,
                "start_planning_time": start_planning_time,
                "end_planning_time": end_planning_time,
            }

            # Paths aligned to batch order (includes failures)
            try:
                paths = result.get_paths()
            except Exception:
                # Fallback: use interpolated_plan directly (no trimming)
                paths = [result.interpolated_plan[i] for i in range(len(task_ids_b))]

            for bi in range(len(task_ids_b)):
                if not bool(result.success[bi].item()):
                    continue

                path_js = paths[bi]
                original_traj = to_numpy(path_js.position)
                sol_path = generate_data_curobo_worker.interpolate_trajectory_to_fixed_length(original_traj)

                # Attempt to extract per-sample optimized_dt and motion_time
                optimized_dt = None
                try:
                    if result.optimized_dt is not None:
                        opt_dt_np = to_numpy(result.optimized_dt)
                        if np.ndim(opt_dt_np) == 0:
                            optimized_dt = float(opt_dt_np)
                        else:
                            optimized_dt = float(opt_dt_np[bi])
                    else:
                        optimized_dt = 0.01
                except Exception:
                    optimized_dt = 0.01

                motion_time_val = None
                try:
                    mt = result.motion_time
                    if isinstance(mt, torch.Tensor):
                        mt_np = to_numpy(mt)
                        motion_time_val = float(mt_np if mt_np.ndim == 0 else mt_np[bi])
                    else:
                        motion_time_val = float(mt)
                except Exception:
                    motion_time_val = generate_data_curobo_worker.interpolate_num * optimized_dt

                results_dict_plan = {
                    "success": True,
                    "solve_time": results_dict_run["run_planning_time"],
                    "sol_path": sol_path,
                    "sol_path_interpolated": sol_path,
                    "sol_path_simplified": sol_path,
                    "optimized_dt": optimized_dt,
                    "motion_time": motion_time_val,
                    "interpolation_num": generate_data_curobo_worker.interpolate_num,
                    "bspline_params": None,
                    "task_id": task_ids_b[bi],
                }

                # Collision distances
                try:
                    traj_tensor = torch.as_tensor(sol_path, device=ta.device, dtype=ta.dtype).unsqueeze(0)
                    d_world, d_self = generate_data_curobo_worker.curobo_fn.get_world_self_collision_distance_from_joint_trajectory(
                        traj_tensor
                    )
                    results_dict_plan["collision_distance_world"] = to_numpy(d_world.squeeze())
                    results_dict_plan["collision_distance_self"] = to_numpy(d_self.squeeze())
                except Exception as e:
                    if debug:
                        print(f"Collision distance failed (batch idx {bi}): {e}")
                    results_dict_plan["collision_distance_world"] = False
                    results_dict_plan["collision_distance_self"] = False

                # Optionally save USD per-trajectory
                try:
                    if save_usd:
                        generate_data_curobo_worker.save_trajectory_usd(sol_path, trajectory_id=len(results_dict_l))
                except Exception as e:
                    if debug:
                        print(f"USD save failed (batch idx {bi}): {e}")

                # Add with incremental local key
                results_dict_run[len([k for k in results_dict_run.keys() if isinstance(k, int)])] = results_dict_plan

            results_dict_l.append(results_dict_run)

    generate_data_curobo_worker.terminate()

    ####################################################################################################################
    # Save timing information stats
    # grab timestamps from worker runs
    if len(results_dict_l) > 0:
        start_times = [r["start_planning_time"] for r in results_dict_l]
        end_times = [r["end_planning_time"] for r in results_dict_l]
        run_times = [r["run_planning_time"] for r in results_dict_l]
        # this is the window when actual planning computation happened
        planning_computation_time = max(end_times) - min(start_times)
    else:
        start_times = []
        end_times = []
        run_times = []
        planning_computation_time = 0.0

    print("-" * 80)
    print(f"Total generation wall time: {t_generate_data.elapsed:.4f} sec")
    print(f"Sequential compute window: {planning_computation_time:.4f} sec")
    print(f"Max batch runtime: {(max(run_times) if run_times else 0.0):.4f} sec")
    print(f"Total batch compute time (sum): {sum(run_times):.4f} sec")
    speedup = (sum(run_times) / planning_computation_time) if planning_computation_time > 0 else 0.0
    print(f"Effective speedup vs. naive sum: {speedup:.2f}x")

    # Create results directory if it doesn't exist
    pathlib.Path(results_dir).mkdir(parents=True, exist_ok=True)
    
    with open(pathlib.Path(results_dir) / "timing_stats.pkl", "wb") as fp:
        pickle.dump(
            {
                "num_plans": len(task_id_l),
                "batch_size": max(1, int(n_parallel_jobs)),
                "wall_time": t_generate_data.elapsed,
                "planning_computation_time": planning_computation_time,
                "max_batch_runtime": (max(run_times) if run_times else 0.0),
                "total_compute_time": sum(run_times),
                "effective_speedup": speedup,
            },
            fp,
        )

    ####################################################################################################################
    # Merge results for hdf5 format
    results_dict = {}
    num_trajectories_generated = 0
    for results_dict_run in results_dict_l:
        # drop if no trajectory was generated
        if len(results_dict_run) == 0:
            continue
        for k, v in results_dict_run.items():
            if k in ["run_planning_time", "start_planning_time", "end_planning_time"]:
                continue
            num_trajectories_generated += 1
            for kk, vv in v.items():
                if kk == "bspline_params" and vv is None:
                    continue
                elif kk == "bspline_params" and vv is not None:
                    tt, cc, k = vv
                    for x, y in zip(["bspline_params_tt", "bspline_params_cc", "bspline_params_k"], [tt, cc, k]):
                        if x in results_dict:
                            results_dict[x].append(y)
                        else:
                            results_dict[x] = [y]
                    continue

                if vv is None:
                    vv = False  # hdf5 does not support None

                if kk in results_dict:
                    results_dict[kk].append(vv)
                else:
                    results_dict[kk] = [vv]

    print("\n\n------------------------------------------------------------")
    num_trajectories_desired = num_tasks * num_trajectories_per_task
    print(
        f"Generated {num_trajectories_generated}/{num_trajectories_desired} trajectories"
        f" in {t_generate_data.elapsed:.3f} s"
    )

    # save results to disk
    hf = h5py.File(os.path.join(results_dir, "dataset.hdf5"), "w")
    for k, v in results_dict.items():
        hf.create_dataset(f"{k}", data=v, compression="gzip")
    # metadata
    hf.attrs["num_trajectories_desired"] = num_trajectories_desired
    hf.attrs["num_trajectories_generated"] = num_trajectories_generated
    hf.close()

    print(f"Dataset saved to: {os.path.join(results_dir, 'dataset.hdf5')}")


if __name__ == "__main__":
    setup_curobo_logger("error")
    run_experiment(experiment)
