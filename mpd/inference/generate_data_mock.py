"""
Mock implementation of GenerateDataOMPL for inference-only usage
This provides minimal functionality needed for inference without pb_ompl dependencies
"""
import numpy as np
import pybullet as p
from pybullet_utils import bullet_client


class MockPbOMPLInterface:
    """Mock interface for pb_ompl functionality"""
    
    def __init__(self, robot_tr, pybullet_client):
        self.robot_tr = robot_tr
        self.pybullet_client = pybullet_client
        
    def is_state_valid(self, q_pos):
        """
        Mock state validation - assumes all states are valid for inference
        In a real implementation, this would check for collisions
        """
        # For inference, we'll be more permissive and assume states are valid
        # if they're within joint limits
        if hasattr(self.robot_tr, 'q_limits'):
            q_lower, q_upper = self.robot_tr.q_limits
            q_lower = q_lower.cpu().numpy() if hasattr(q_lower, 'cpu') else q_lower
            q_upper = q_upper.cpu().numpy() if hasattr(q_upper, 'cpu') else q_upper
            
            # Check if all joints are within limits
            return np.all(q_pos >= q_lower) and np.all(q_pos <= q_upper)
        else:
            # If no joint limits available, assume valid
            return True
    
    def get_ee_pose(self, q_pos):
        """
        Get end-effector pose from joint configuration
        """
        try:
            # Try to use torch_robotics robot for forward kinematics
            import torch
            from torch_robotics.torch_utils.torch_utils import to_torch, to_numpy, DEFAULT_TENSOR_ARGS
            
            q_pos_torch = to_torch(q_pos, **DEFAULT_TENSOR_ARGS)
            if q_pos_torch.dim() == 1:
                q_pos_torch = q_pos_torch.unsqueeze(0)
                
            # Get end-effector pose using the robot's forward kinematics
            ee_pose = self.robot_tr.get_EE_pose(q_pos_torch)
            ee_pose_np = to_numpy(ee_pose.squeeze(0))
            
            # Extract position and orientation
            if ee_pose_np.shape == (3, 4):  # Homogeneous transform matrix (3x4)
                position = ee_pose_np[:3, 3]
                rotation_matrix = ee_pose_np[:3, :3]
                
                # Convert rotation matrix to quaternion
                from scipy.spatial.transform import Rotation
                quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
                
            elif ee_pose_np.shape == (4, 4):  # Full homogeneous transform matrix (4x4)
                position = ee_pose_np[:3, 3]
                rotation_matrix = ee_pose_np[:3, :3]
                
                # Convert rotation matrix to quaternion
                from scipy.spatial.transform import Rotation
                quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
                
            else:
                # Fallback: assume identity pose
                position = np.array([0.0, 0.0, 0.0])
                quaternion = np.array([0.0, 0.0, 0.0, 1.0])
            
            return position, quaternion
            
        except Exception as e:
            print(f"Warning: Could not compute EE pose: {e}")
            # Return a default pose
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])


class GenerateDataOMPLMock:
    """
    Mock implementation of GenerateDataOMPL for inference-only usage
    """
    
    def __init__(
        self,
        env_id,
        robot_id,
        env_tr=None,
        robot_tr=None,
        gripper=True,
        grasped_object=None,
        min_distance_robot_env=0.02,
        tensor_args=None,
        pybullet_mode="DIRECT",
        debug=False,
        **kwargs
    ):
        self.env_tr = env_tr
        self.robot_tr = robot_tr
        self.tensor_args = tensor_args
        self.debug = debug
        
        # Setup minimal pybullet client
        self.pybullet_client = bullet_client.BulletClient(
            connection_mode=p.GUI if pybullet_mode == "GUI" else p.DIRECT
        )
        self.pybullet_client.setGravity(0, 0, 0)
        
        # Create mock pb_ompl interface
        self.pbompl_interface = MockPbOMPLInterface(robot_tr, self.pybullet_client)
        
        if debug:
            print("Using mock GenerateDataOMPL for inference - collision checking will be simplified")
    
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
        duration_visualization=2.0,
        wait_time_after_visualization=4.0,
        debug=False,
    ):
        """
        Mock implementation of trajectory generation
        Returns a simple linear interpolation between start and goal
        """
        import numpy as np
        
        results_dict = {
            "trajectories": [],
            "success": True,
            "planning_time": 0.1  # Mock planning time
        }
        
        # Generate simple linear interpolation trajectories
        for i in range(num_trajectories):
            # Create linear interpolation between start and goal
            trajectory = np.linspace(joint_position_start, joint_position_goal, interpolate_num)
            results_dict["trajectories"].append(trajectory)
        
        if debug:
            print(f"Mock: Generated {num_trajectories} linear interpolation trajectories")
        
        return results_dict
    
    def terminate(self):
        """
        Mock implementation of termination
        Cleans up pybullet client if needed
        """
        try:
            if hasattr(self, 'pybullet_client'):
                self.pybullet_client.disconnect()
        except Exception as e:
            if self.debug:
                print(f"Warning during mock termination: {e}")
        
        if self.debug:
            print("Mock GenerateDataOMPL terminated")