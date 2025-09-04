import os
from pathlib import Path
from typing import Optional, Union

import numpy as np
import torch

from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.state import JointState
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_world_configs_path, join_path, load_yaml


def _to_joint_state(q_traj: Union[JointState, torch.Tensor, np.ndarray]) -> JointState:
    """Convert various trajectory inputs into a cuRobo JointState.

    Accepts:
      - JointState with position [T, dof]
      - torch.Tensor with shape [T, dof]
      - np.ndarray with shape [T, dof]
    """
    if isinstance(q_traj, JointState):
        return q_traj

    if isinstance(q_traj, np.ndarray):
        q_traj = torch.from_numpy(q_traj)

    if isinstance(q_traj, torch.Tensor):
        assert q_traj.ndim == 2, "q_traj tensor must be of shape [T, dof]"
        return JointState.from_position(q_traj)

    raise TypeError("Unsupported q_traj type. Use JointState, torch.Tensor, or np.ndarray.")


def save_trajectory_usd(
    q_traj: Union[JointState, torch.Tensor, np.ndarray],
    robot_file: str,
    world_file: Optional[str],
    save_path: str,
    dt: float = 0.2,
    visualize_robot_spheres: bool = False,
    base_frame: Optional[str] = None,
) -> str:
    """Save a joint-space trajectory as USD for later replay.

    Args:
      q_traj: Trajectory [T, dof] or cuRobo JointState.
      robot_file: Robot config filename (e.g., 'franka.yml').
      world_file: World config filename (e.g., 'collision_test.yml'), or None for empty world.
      save_path: Output .usd filepath ('.usd' is appended if missing).
      dt: Time step between frames.
      visualize_robot_spheres: If True, also writes collision spheres animation.
      base_frame: USD stage root path. Defaults to '/<save_stem>'.

    Returns:
      The path of the written USD file.
    """
    # Normalize save path
    if not save_path.endswith(".usd"):
        save_path = save_path + ".usd"

    # Compute default base frame
    if base_frame is None:
        base_frame = "/" + Path(save_path).stem

    # Convert to JointState
    q_traj_js = _to_joint_state(q_traj)
    # Use the first waypoint as start state if available
    q_start_js = JointState.from_position(q_traj_js.position[0:1, :])

    # Load world configuration
    world_model = None
    if world_file is not None:
        world_model = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), world_file)))

    # Write USD via cuRobo helper
    UsdHelper.write_trajectory_animation_with_robot_usd(
        robot_model_file=robot_file,
        world_model=world_model,
        q_start=q_start_js,
        q_traj=q_traj_js,
        dt=float(dt),
        visualize_robot_spheres=visualize_robot_spheres,
        save_path=save_path,
        base_frame=base_frame,
    )

    return save_path


def save_motiongen_result_usd(
    result,
    robot_file: str,
    world_file: Optional[str],
    save_path: str,
    dt: float = 0.02,
    visualize_robot_spheres: bool = False,
    base_frame: Optional[str] = None,
) -> str:
    """Save a MotionGenResult's interpolated trajectory as USD.

    Args:
      result: cuRobo MotionGenResult with 'interpolated_plan' (JointState).
      robot_file: Robot config filename.
      world_file: World config filename, or None.
      save_path: Output .usd filepath.
      dt: Time step between frames.
      visualize_robot_spheres: If True, also writes collision spheres animation.
      base_frame: USD stage root path.
    """
    if getattr(result, "interpolated_plan", None) is None:
        raise ValueError("MotionGenResult has no 'interpolated_plan' to save.")

    q_js = result.interpolated_plan
    return save_trajectory_usd(
        q_traj=q_js,
        robot_file=robot_file,
        world_file=world_file,
        save_path=save_path,
        dt=dt,
        visualize_robot_spheres=visualize_robot_spheres,
        base_frame=base_frame,
    )

