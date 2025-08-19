"""
PyBullet rendering utilities for trajectory visualization
"""
import time
import numpy as np
import pybullet as p
from pybullet_utils import bullet_client
from torch_robotics.torch_utils.torch_utils import to_numpy


class PyBulletTrajectoryRenderer:
    """
    Enhanced PyBullet renderer for trajectory visualization
    """
    
    def __init__(self, robot_tr, env_tr=None, gui_mode=True, debug=False):
        """
        Initialize PyBullet renderer
        
        Args:
            robot_tr: Robot model from torch_robotics
            env_tr: Environment model (optional)
            gui_mode: Whether to show GUI
            debug: Debug mode
        """
        self.robot_tr = robot_tr
        self.env_tr = env_tr
        self.debug = debug
        
        # Setup PyBullet with conflict handling
        try:
            self.client = bullet_client.BulletClient(
                connection_mode=p.GUI if gui_mode else p.DIRECT
            )
        except Exception as e:
            if gui_mode and "GUI/GUI_SERVER connection" in str(e):
                print("Warning: GUI mode already in use, falling back to DIRECT mode")
                self.client = bullet_client.BulletClient(connection_mode=p.DIRECT)
                gui_mode = False  # Update gui_mode for later use
            else:
                raise e
        self.client.setGravity(0, 0, 0)
        
        # Robot and environment IDs
        self.robot_id = None
        self.env_objects = []
        self.trajectory_lines = []
        
        # Setup visualization
        if gui_mode:
            self._setup_camera()
            self._setup_visualization()
        
        # Load robot and environment
        self._load_robot()
        if env_tr is not None:
            self._load_environment()
    
    def _setup_camera(self):
        """Setup camera position for better viewing"""
        self.client.resetDebugVisualizerCamera(
            cameraDistance=2.5,
            cameraYaw=50,
            cameraPitch=-35,
            cameraTargetPosition=[0, 0, 0.5]
        )
        
        # Clean up GUI
        self.client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.client.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        self.client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    
    def _setup_visualization(self):
        """Setup additional visualization elements"""
        # Add ground plane
        self.client.loadURDF("plane.urdf", [0, 0, -0.1])
        
        # Set background color
        self.client.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    
    def _load_robot(self):
        """Load robot URDF"""
        try:
            if hasattr(self.robot_tr, 'robot_urdf_file'):
                robot_urdf_path = self.robot_tr.robot_urdf_file
                self.robot_id = self.client.loadURDF(
                    robot_urdf_path,
                    basePosition=[0, 0, 0],
                    useFixedBase=True
                )
                
                if self.debug:
                    print(f"Loaded robot URDF: {robot_urdf_path}")
                    num_joints = self.client.getNumJoints(self.robot_id)
                    print(f"Robot has {num_joints} joints")
            else:
                print("Warning: No robot URDF file available")
                
        except Exception as e:
            print(f"Error loading robot: {e}")
    
    def _load_environment(self):
        """Load environment objects"""
        try:
            # Load environment objects if available
            if hasattr(self.env_tr, 'get_collision_geometry'):
                # This is a simplified approach - you may need to adapt based on your environment
                print("Environment loading not fully implemented - add specific environment loading logic")
                
        except Exception as e:
            print(f"Error loading environment: {e}")
    
    def render_trajectory(self, q_trajectory, sleep_time=0.05, show_path=True, path_color=[1, 0, 0]):
        """
        Render trajectory animation
        
        Args:
            q_trajectory: Joint trajectory (numpy array or torch tensor)
            sleep_time: Time between frames
            show_path: Whether to draw end-effector path
            path_color: RGB color for path visualization
        """
        if self.robot_id is None:
            print("Error: No robot loaded")
            return
        
        # Convert to numpy if needed
        if hasattr(q_trajectory, 'cpu'):
            q_trajectory = to_numpy(q_trajectory)
        
        print(f"Rendering trajectory with {len(q_trajectory)} waypoints...")
        
        # Store EE positions for path visualization
        ee_positions = []
        
        for i, q_pos in enumerate(q_trajectory):
            # Set joint positions
            num_joints = min(len(q_pos), self.client.getNumJoints(self.robot_id))
            for joint_idx in range(num_joints):
                self.client.resetJointState(
                    self.robot_id,
                    joint_idx,
                    q_pos[joint_idx]
                )
            
            # Get end-effector position for path visualization
            if show_path:
                try:
                    ee_pos, ee_quat = self.robot_tr.robot.get_ee_pose(q_pos)
                    if hasattr(ee_pos, 'cpu'):
                        ee_pos = to_numpy(ee_pos)
                    ee_positions.append(ee_pos[:3])  # Only position, not orientation
                except:
                    # Fallback: use forward kinematics or approximation
                    pass
            
            # Update simulation
            self.client.stepSimulation()
            
            # Sleep for animation
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Show progress
            if i % 20 == 0 and self.debug:
                print(f"Progress: {i+1}/{len(q_trajectory)}")
        
        # Draw end-effector path
        if show_path and len(ee_positions) > 1:
            self._draw_path(ee_positions, path_color)
        
        print("Trajectory rendering completed")
    
    def _draw_path(self, positions, color=[1, 0, 0], line_width=3):
        """Draw path as connected line segments"""
        for i in range(len(positions) - 1):
            line_id = self.client.addUserDebugLine(
                positions[i],
                positions[i + 1],
                lineColorRGB=color,
                lineWidth=line_width
            )
            self.trajectory_lines.append(line_id)
    
    def clear_trajectory_lines(self):
        """Clear all drawn trajectory lines"""
        for line_id in self.trajectory_lines:
            self.client.removeUserDebugItem(line_id)
        self.trajectory_lines = []
    
    def render_static_trajectories(self, trajectories, colors=None):
        """
        Render multiple trajectories as static paths (no animation)
        
        Args:
            trajectories: List of trajectories
            colors: List of RGB colors for each trajectory
        """
        if colors is None:
            colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1]]
        
        print(f"Rendering {len(trajectories)} static trajectory paths...")
        
        # Draw all trajectories as static paths
        for traj_idx, trajectory in enumerate(trajectories):
            color = colors[traj_idx % len(colors)]
            self._draw_trajectory_path(trajectory, color)
        
        # Set robot to final position of first trajectory
        if len(trajectories) > 0 and len(trajectories[0]) > 0:
            final_q = trajectories[0][-1]
            self.set_robot_configuration(final_q)
    
    def _draw_trajectory_path(self, trajectory, color):
        """Draw complete trajectory as a path"""
        if len(trajectory) < 2:
            return
            
        # Get end-effector positions for the entire trajectory
        ee_positions = []
        for q_pos in trajectory:
            try:
                if hasattr(q_pos, 'cpu'):
                    q_pos = to_numpy(q_pos)
                
                # Set robot to this configuration temporarily
                num_joints = min(len(q_pos), self.client.getNumJoints(self.robot_id))
                for joint_idx in range(num_joints):
                    self.client.resetJointState(self.robot_id, joint_idx, q_pos[joint_idx])
                
                # Get end-effector link state
                if self.robot_id is not None:
                    # Assume end-effector is the last link
                    num_joints = self.client.getNumJoints(self.robot_id)
                    if num_joints > 0:
                        link_state = self.client.getLinkState(self.robot_id, num_joints - 1)
                        ee_pos = link_state[0]  # World position
                        ee_positions.append(ee_pos)
                
            except Exception as e:
                if self.debug:
                    print(f"Warning: Could not get EE position: {e}")
                continue
        
        # Draw the path
        if len(ee_positions) > 1:
            self._draw_path(ee_positions, color)
    
    def set_robot_configuration(self, q_pos):
        """Set robot to specific joint configuration"""
        if self.robot_id is None:
            return
            
        if hasattr(q_pos, 'cpu'):
            q_pos = to_numpy(q_pos)
        
        num_joints = min(len(q_pos), self.client.getNumJoints(self.robot_id))
        for joint_idx in range(num_joints):
            self.client.resetJointState(
                self.robot_id,
                joint_idx,
                q_pos[joint_idx]
            )
        self.client.stepSimulation()
    
    def add_markers(self, positions, colors=None, sizes=None):
        """Add sphere markers at specified positions"""
        if colors is None:
            colors = [[1, 0, 0] for _ in positions]
        if sizes is None:
            sizes = [0.05 for _ in positions]
        
        marker_ids = []
        for i, pos in enumerate(positions):
            # Create visual shape
            visual_shape_id = self.client.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=sizes[i],
                rgbaColor=colors[i] + [1.0]
            )
            
            # Create multi-body
            marker_id = self.client.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual_shape_id,
                basePosition=pos
            )
            marker_ids.append(marker_id)
        
        return marker_ids
    
    def save_image(self, filename, width=1920, height=1080):
        """Save current view as image"""
        # Get camera image
        view_matrix = self.client.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0.5],
            distance=2.5,
            yaw=50,
            pitch=-35,
            roll=0,
            upAxisIndex=2
        )
        
        proj_matrix = self.client.computeProjectionMatrixFOV(
            fov=60,
            aspect=width/height,
            nearVal=0.1,
            farVal=100.0
        )
        
        (_, _, px, _, _) = self.client.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix
        )
        
        # Save image
        import PIL.Image as Image
        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (height, width, 4))
        rgb_array = rgb_array[:, :, :3]  # Remove alpha channel
        
        img = Image.fromarray(rgb_array)
        img.save(filename)
        print(f"Saved image: {filename}")
    
    def close(self):
        """Clean up and close PyBullet"""
        try:
            self.client.disconnect()
        except:
            pass