"""
Mock implementation of pb_ompl functions for inference-only usage
"""
import numpy as np
import pybullet as p
from scipy.interpolate import splprep, splev


def add_box(client, position, size, orientation=None, color=(1.0, 0.0, 0.0, 1.0)):
    """
    Mock implementation of add_box from pb_ompl
    Adds a visual box marker to the pybullet environment
    """
    if orientation is None:
        orientation = [0, 0, 0, 1]  # quaternion
    
    # Create a box visual shape
    visual_shape = client.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=size,
        rgbaColor=color
    )
    
    # Create a collision shape (optional, for markers we might not need collision)
    collision_shape = client.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=size
    )
    
    # Create the body
    body_id = client.createMultiBody(
        baseMass=0,  # Static object
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
        baseOrientation=orientation
    )
    
    return body_id


def fit_bspline_to_path(
    path,
    # Original mock signature
    degree=3,
    n_control_points=10,
    zero_vel_at_endpoints=True,
    zero_acc_at_endpoints=True,
    # Adapter for dataset loader keyword args (so calls don't fail)
    bspline_degree=None,
    bspline_num_control_points=None,
    bspline_zero_vel_at_start_and_goal=None,
    bspline_zero_acc_at_start_and_goal=None,
    debug=False,
):
    """
    Mock implementation of pb_ompl.pb_ompl.fit_bspline_to_path with compatible signature.

    Accepts both the original mock args and the dataset loader's keyword args:
      - bspline_degree -> degree
      - bspline_num_control_points -> n_control_points
      - bspline_zero_vel_at_start_and_goal -> zero_vel_at_endpoints
      - bspline_zero_acc_at_start_and_goal -> zero_acc_at_endpoints

    Returns (tt, cc, k) to match downstream expectations, where:
      - tt: knot vector (from scipy splprep)
      - cc: control points as array [n_control_points, dof]
      - k:  spline degree
    """
    # Map aliased arguments if provided
    if bspline_degree is not None:
        degree = bspline_degree
    if bspline_num_control_points is not None:
        n_control_points = bspline_num_control_points
    if bspline_zero_vel_at_start_and_goal is not None:
        zero_vel_at_endpoints = bspline_zero_vel_at_start_and_goal
    if bspline_zero_acc_at_start_and_goal is not None:
        zero_acc_at_endpoints = bspline_zero_acc_at_start_and_goal

    if path.shape[0] < 2:
        raise ValueError("Path must have at least 2 waypoints")

    # Transpose path for splprep (it expects (n_dof, n_waypoints))
    path_T = path.T
    n_dof = path_T.shape[0]

    # Create parameter values for the path
    u = np.linspace(0, 1, path.shape[0])

    # Fit B-spline using scipy
    try:
        # splprep returns (tck, u) where tck is (t, c, k)
        k_fit = int(min(degree, max(1, path.shape[0] - 1)))
        tck, _ = splprep(path_T, u=u, s=0, k=k_fit)
        t, c, k = tck

        # Generate pseudo "control points" by sampling the spline at uniform parameters
        # (This is a simplified surrogate; true control point solving is more involved.)
        u_control = np.linspace(0, 1, n_control_points)
        control_points_list = []

        for i in range(n_dof):
            control_vals = splev(u_control, (t, [c[i]], k))
            if isinstance(control_vals, tuple):
                control_vals = control_vals[0]
            control_points_list.append(control_vals)

        # Assemble control points as [dof, n_control_points] to match downstream expectations
        control_points = np.array(control_points_list)  # [dof, n_control_points]

        # Apply endpoint constraints in a simple manner
        if zero_vel_at_endpoints or zero_acc_at_endpoints:
            # Enforce end points along the second axis (control point index)
            control_points[:, 0] = path[0]
            control_points[:, -1] = path[-1]

        # Return (tt, cc, k) to match loader expectations
        tt = t
        cc = control_points  # [dof, n_control_points]
        return tt, cc, k

    except Exception as e:
        # Fallback: linear interpolation when spline fit fails
        if debug:
            print(f"B-spline fitting failed: {e}. Using linear interpolation.")

        u_control = np.linspace(0, 1, n_control_points)
        control_points = np.zeros((n_dof, n_control_points))
        for i in range(n_dof):
            control_points[i, :] = np.interp(u_control, u, path[:, i])

        # Simple knot vector and degree surrogate
        tt = u_control
        k = 1
        cc = control_points  # [dof, n_control_points]
        return tt, cc, k
