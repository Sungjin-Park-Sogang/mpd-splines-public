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


def fit_bspline_to_path(path, degree=3, n_control_points=10, zero_vel_at_endpoints=True, zero_acc_at_endpoints=True):
    """
    Mock implementation of fit_bspline_to_path from pb_ompl
    Fits a B-spline to a given path
    
    Args:
        path: numpy array of shape (n_waypoints, n_dof)
        degree: B-spline degree
        n_control_points: number of control points
        zero_vel_at_endpoints: whether to enforce zero velocity at endpoints
        zero_acc_at_endpoints: whether to enforce zero acceleration at endpoints
    
    Returns:
        Tuple (tck, control_points) where:
        - tck: B-spline representation (t, c, k)
        - control_points: numpy array of control points
    """
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
        tck, _ = splprep(path_T, u=u, s=0, k=min(degree, path.shape[0]-1))
        t, c, k = tck
        
        # Generate control points
        u_control = np.linspace(0, 1, n_control_points)
        control_points_list = []
        
        for i in range(n_dof):
            control_vals = splev(u_control, (t, [c[i]], k))
            if isinstance(control_vals, tuple):
                control_vals = control_vals[0]
            control_points_list.append(control_vals)
        
        control_points = np.array(control_points_list).T
        
        # Apply endpoint constraints if requested
        if zero_vel_at_endpoints or zero_acc_at_endpoints:
            # This is a simplified implementation
            # In a full implementation, you'd enforce derivative constraints
            control_points[0] = path[0]   # Start point
            control_points[-1] = path[-1] # End point
        
        return tck, control_points
        
    except Exception as e:
        # Fallback: linear interpolation
        print(f"B-spline fitting failed: {e}. Using linear interpolation.")
        u_control = np.linspace(0, 1, n_control_points)
        control_points = np.zeros((n_control_points, n_dof))
        
        for i in range(n_dof):
            control_points[:, i] = np.interp(u_control, u, path[:, i])
        
        # Create a simple tck representation
        tck = (u_control, control_points.T.tolist(), 1)
        
        return tck, control_points