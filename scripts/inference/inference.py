import time
from functools import partial

# import isaacgym


from dotmap import DotMap

import gc
import os
from pprint import pprint

import numpy as np
import torch
from einops._torch_specific import allow_ops_in_compiled_graph  # requires einops>=0.6.1

from experiment_launcher import single_experiment_yaml, run_experiment
from mpd.inference.inference import EvaluationSamplesGenerator, GenerativeOptimizationPlanner, render_results_pybullet
from mpd.metrics.metrics import PlanningMetricsCalculator
from mpd.utils.loaders import get_planning_task_and_dataset, load_params_from_yaml, save_to_yaml
# from torch_robotics.isaac_gym_envs.motion_planning_envs import (
#     MotionPlanningIsaacGymEnv,
#     MotionPlanningControllerIsaacGym,
# )
from torch_robotics.robots import RobotPanda
from torch_robotics.torch_kinematics_tree.utils.files import get_robot_path
from torch_robotics.torch_utils.seed import fix_random_seed
from torch_robotics.torch_utils.torch_utils import get_torch_device, to_torch, to_numpy

from curobo.types.base import TensorDeviceType
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig


allow_ops_in_compiled_graph()


@single_experiment_yaml
def experiment(
    ########################################################################
    # Configuration path defining the model and the inference parameters
    # cfg_inference_path: str = './cfgs/config_EnvNarrowPassageDense2D-RobotPointMass2D_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvPlanar2Link-RobotPlanar2Link_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvPlanar4Link-RobotPlanar4Link_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvSimple2D-RobotPointMass2D_00.yaml',
    # cfg_inference_path: str = './cfgs/config_EnvSpheres3D-RobotPanda_00.yaml',
    # cfg_inference_path: str = "./cfgs/config_EnvWarehouse-RobotPanda-config_file_v01_00.yaml",
    cfg_inference_path: str = "./cfgs/config_Test.yaml",

    ########################################################################
    # Select the start and goal from the training or validation/test set.
    selection_start_goal: str = "validation",  # training, validation/test
    ########################################################################
    # number of start and goal states to evaluate
    n_start_goal_states: int = 3,
    ########################################################################
    save_results_single_plan_low_mem: bool = False,
    ########################################################################
    # Visualization options
    render_joint_space_time_iters: bool = False,  # Disable to avoid ffmpeg issues
    render_joint_space_env_iters: bool = False,
    render_env_robot_opt_iters: bool = False,
    render_env_robot_trajectories: bool = False,
    render_pybullet: bool = False,
    render_pybullet_trajectories: bool = False,  # Enable PyBullet rendering instead
    # USD export options
    save_usd: bool = True,
    usd_save_path: str = "trajectory_viz_inference",
    ########################################################################
    device: str = "cuda:0",  # cpu, cuda
    debug: bool = False,
    ########################################################################
    # MANDATORY
    # seed: int = int(time.time()),
    seed: int = 2,
    results_dir: str = "logs",
    ########################################################################
    **kwargs,
):
    # Set random seed for reproducibility
    fix_random_seed(seed)

    device = get_torch_device(device)
    # Explicitly set CUDA device index to avoid Warp/PyTorch conflicts
    if device.type == "cuda":
        explicit_device = torch.device("cuda", 0)  # Force cuda:0
        curobo_tensor_args = TensorDeviceType.from_basic("cuda", 0)
    else:
        explicit_device = device
        curobo_tensor_args = TensorDeviceType(device=torch.device("cpu"), dtype=torch.float32)
    
    # Keep MPD-compatible tensor_args for backward compatibility  
    tensor_args = {"device": explicit_device, "dtype": torch.float32}

    print(f"Default tensor args: {tensor_args}")
    print(f"cuRobo tensor args: {curobo_tensor_args}")

    # Save and load the inference configuration
    args_inference = DotMap(load_params_from_yaml(cfg_inference_path))

    if "cvae" in args_inference.planner_alg:
        if args_inference.model_selection == "bspline":
            args_inference.model_dir = args_inference.model_dir_cvae_bspline
        elif args_inference.model_selection == "waypoints":
            args_inference.model_dir = args_inference.model_dir_cvae_waypoints
        else:
            raise NotImplementedError
    else:
        if args_inference.model_selection == "bspline":
            args_inference.model_dir = args_inference.model_dir_ddpm_bspline
        elif args_inference.model_selection == "waypoints":
            args_inference.model_dir = args_inference.model_dir_ddpm_waypoints
        else:
            raise NotImplementedError

    args_inference.model_dir = os.path.expandvars(args_inference.model_dir)

    save_to_yaml(args_inference.toDict(), os.path.join(results_dir, "args_inference.yaml"))

    print(f"\n-------------------------------------------------------------------------------------------------")
    print(f"cfg_inference_path:\n{cfg_inference_path}")
    print(f"Model:\n{args_inference.model_dir}")
    print(f"--------------------------------------------------------------------------------------------------")

    ################################################################################################################
    # Load dataset, environment, robot and planning task.
    # Override training parameters.
    args_train = DotMap(load_params_from_yaml(os.path.join(args_inference.model_dir, "args.yaml")))
    args_train.update(
        **args_inference,
        gripper=True,
        reload_data=False,
        results_dir=results_dir,
        load_indices=True,
        tensor_args=tensor_args,
    )
    planning_task, train_subset, _, val_subset, _ = get_planning_task_and_dataset(**args_train)

    ################################################################################################################
    # Generator of evaluation samples
    evaluation_samples_generator = EvaluationSamplesGenerator(
        planning_task,
        train_subset,
        val_subset,
        selection_start_goal=selection_start_goal,
        planner="RRTConnect",
        tensor_args=tensor_args,
        debug=debug,
        render_pybullet=render_pybullet or render_pybullet_trajectories,  # Enable GUI mode if any PyBullet rendering is requested
        **args_inference,
    )

    ################################################################################################################
    generative_optimization_planner = GenerativeOptimizationPlanner(
        planning_task,
        train_subset.dataset,
        args_train,
        args_inference,
        tensor_args,
        sampling_based_planner_fn=partial(
            evaluation_samples_generator.generate_data_ompl_worker.run,
            planner_allowed_time=10.0,
            interpolate_num=args_inference.num_T_pts,
            simplify_path=True,
        ),
        debug=debug,
        robot_config="franka.yml",
        world_model="collision_test.yml",
        curobo_tensor_args=curobo_tensor_args,
    )

    ################################################################################################################
    # Metrics calculator
    planning_metrics_calculator = PlanningMetricsCalculator(planning_task)

    ################################################################################################################
    # Plan for several start and goal states sequentially
    if selection_start_goal == "training":
        idx_sample_l = np.random.choice(np.arange(len(train_subset)), n_start_goal_states)
    else:
        idx_sample_l = np.random.choice(np.arange(len(val_subset)), n_start_goal_states)
    for idx_sg, idx_sample in enumerate(idx_sample_l):
        print(f"\n-------------------------------------------------------------------------------------------------")
        print(f"----------------PLANNING {idx_sg+1}/{n_start_goal_states}------------------")
        print(f"--------------------------------------------------------------------------------------------------")

        results_single_plan = DotMap(t_generator=0.0, t_guide=0.0)

        q_pos_start, q_pos_goal, ee_pose_goal = evaluation_samples_generator.get_data_sample(idx_sg)

        print("\n----------------START AND GOAL states----------------")
        print(f"q_pos_start: {q_pos_start}")
        print(f"q_pos_goal: {q_pos_goal}")
        print(f"ee_pose_goal: {ee_pose_goal}")

        if debug:
            evaluation_samples_generator.add_start_goal_marker(q_pos_start, q_pos_goal)

        ############################################################################################################
        # Run motion planning inference
        print(f"\n----------------PLAN TRAJECTORIES----------------")
        print(f"Starting inference...")
        results_single_plan = generative_optimization_planner.plan_trajectory(
            q_pos_start, q_pos_goal, ee_pose_goal, results_ns=results_single_plan, debug=debug
        )
        print(f"...inference finished.")

        ############################################################################################################
        # Show in pybullet the best trajectory
        if render_pybullet and results_single_plan.q_trajs_pos_best is not None:
            time.sleep(3)
            ########################
            # Visualize in Pybullet
            q_pos_path = to_numpy(results_single_plan.q_trajs_pos_best)
            # add panda grippers to the path
            if (
                isinstance(planning_task.robot, RobotPanda)
                and q_pos_path.shape[1] == 7
                and evaluation_samples_generator.generate_data_ompl_worker.pbompl_interface.robot.num_dim == 9
            ):
                q_pos_path = np.concatenate((q_pos_path, np.zeros((q_pos_path.shape[0], 2))), axis=-1)
            evaluation_samples_generator.generate_data_ompl_worker.pbompl_interface.execute(
                q_pos_path, sleep_time=planning_task.parametric_trajectory.dt
            )

        ############################################################################################################
        # Compute motion planning metrics
        print(f"\n----------------METRICS----------------")
        results_single_plan.metrics = planning_metrics_calculator.compute_metrics(results_single_plan)

        print(f"t_inference_total: {results_single_plan.t_inference_total:.3f} sec")
        print(f"t_generator: {results_single_plan.t_generator:.3f} sec")
        print(f"t_guide: {results_single_plan.t_guide:.3f} sec")

        print(f"metrics:")
        # 출력이 너무 길어
        # print(results_single_plan.metrics)

        # Save data
        results_single_plan_to_save = results_single_plan
        if save_results_single_plan_low_mem:
            results_single_plan_to_save = DotMap(
                t_generator=results_single_plan.t_generator,
                t_guide=results_single_plan.t_guide,
                t_inference_total=results_single_plan.t_inference_total,
                q_pos_start=q_pos_start,
                q_pos_goal=q_pos_goal,
                ee_pose_goal=ee_pose_goal,
                control_points_iters=results_single_plan.control_points_iters,
                metrics=results_single_plan.metrics,
                isaacgym_statistics=results_single_plan.isaacgym_statistics,
            )
        torch.save(
            results_single_plan_to_save,
            os.path.join(results_dir, f"results_single_plan-{idx_sg:03d}.pt"),
            _use_new_zipfile_serialization=True,
        )

        ############################################################################################################
        # Render sampling results

        # Render sampling results with enhanced PyBullet support
        if render_pybullet_trajectories:
            render_results_pybullet(
                args_inference,
                planning_task,
                q_pos_start,
                q_pos_goal,
                results_single_plan,
                idx_sg,
                results_dir,
                render_joint_space_time_iters=render_joint_space_time_iters,
                render_joint_space_env_iters=render_joint_space_env_iters,
                render_planning_env_robot_opt_iters=render_env_robot_opt_iters,
                render_planning_env_robot_trajectories=render_env_robot_trajectories,
                render_pybullet_trajectories=render_pybullet_trajectories,
                debug=debug,
            )
        
        # Optionally save the best trajectory as USD for replay
        try:
            if save_usd and results_single_plan.q_trajs_pos_best is not None:
                from mpd.plotting.visualize_trajectories_curobo import save_trajectory_usd

                if results_single_plan.q_trajs_pose_best is not None:
                    q_traj_best = results_single_plan.q_trajs_pos_best

                save_path_usd = os.path.join(results_dir, f"{usd_save_path}_{idx_sg:04d}.usd")  

                save_trajectory_usd(
                    q_traj=q_traj_best,
                    robot_file="franka.yml",
                    world_file="collision_test.yml",
                    save_path=save_path_usd,
                    visualize_robot_spheres=False,
                )
                print(f"\u2713 Saved USD: {save_path_usd}")
        except Exception as e:
            print(f"[WARN] Failed to save USD for trajectory {idx_sg}: {e}")
        
        # print(f"[INFO] Print results.\n{results_single_plan}")

        ############################################################################################################
        # empty memory
        del results_single_plan
        gc.collect()
        torch.cuda.empty_cache()

    ################################################################################################################
    # clean up
    evaluation_samples_generator.generate_data_ompl_worker.terminate()
    gc.collect()
    torch.cuda.empty_cache()


if __name__ == "__main__":
    run_experiment(experiment)
