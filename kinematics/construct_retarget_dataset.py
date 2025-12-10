
"""
Actuator index -> Name mapping:
ctrl[0] -> fr3_joint1
ctrl[1] -> fr3_joint2
ctrl[2] -> fr3_joint3
ctrl[3] -> fr3_joint4
ctrl[4] -> fr3_joint5
ctrl[5] -> fr3_joint6
ctrl[6] -> fr3_joint7
ctrl[7] -> base_jointindex_mcp_actuator
ctrl[8] -> base_jointmiddle_mcp_actuator
ctrl[9] -> base_jointring_mcp_actuator
ctrl[10] -> base_jointpinky_mcp_actuator
ctrl[11] -> base_jointthumb_flexor_actuator
ctrl[12] -> base_jointthumb_rotator_actuator
"""

import pickle
import mink
import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
from mink.lie import SE3, SO3
from tqdm import tqdm
import matplotlib.pyplot as plt
import time
from PIL import Image
import io
import os
import subprocess
import hydra
from omegaconf import DictConfig
from scipy.signal import savgol_filter
from pathlib import Path

def right_ability_hand_to_canonical(hand_point):
    z_axis = hand_point[9] - hand_point[0]
    z_axis = z_axis / np.linalg.norm(z_axis)
    y_axis_aux = hand_point[5] - hand_point[13]
    y_axis_aux = y_axis_aux / np.linalg.norm(y_axis_aux)
    x_axis = np.cross(y_axis_aux, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    rotation_base = np.array([x_axis, y_axis, z_axis]).transpose()
    tranlation_base = hand_point[0]
    transform = np.eye(4)
    transform[:3, :3] = rotation_base
    transform[:3, 3] = tranlation_base
    transform_inv = np.linalg.inv(transform)
    hand_point = np.array(hand_point)
    hand_point = np.concatenate((np.array(hand_point), np.ones((21, 1))), axis=-1)
    hand_point = hand_point @ transform_inv.transpose()
    return hand_point[:, :3], rotation_base, tranlation_base


class PsyonicKinematics:
    def __init__(self, joints_dir, camera_config):

        self.joints_dir = joints_dir
        self.joint_cmds = []
        self.franka_ee_pos = []
        self.model_path = Path(__file__).parent.parent.resolve().joinpath("models/psyonic_right_franka_scene.xml")
        self.xml_model = mujoco.MjModel.from_xml_path(
            str(self.model_path)
        )
      
        self.fixed_world_frame_offset = np.array([0.0, 0.0, 0])

        self.X_BC = np.load(camera_config)
        print(self.X_BC)
        self.ctrl_iters = 25

        self.configuration = mink.Configuration(self.xml_model)

        self.mano_fingertip_indices = [
            8,
            12,
            16,
            20,
            4,
        ]  # index, middle, ring, pinky, thumb

        self.actuator_bodies = [
            "index_prox",
            "middle_prox",
            "ring_prox",
            "pinky_prox",
            "thumb_metacarp",
            "thumb_mcp",
        ]

        self.fingertip_sites = [  # franka + psyonic
            "base_jointfsr5",  # index
            "base_jointfsr11",  # middle
            "base_jointfsr17",  # ring
            "base_jointfsr23",  # pinky
            "base_jointfsr29",  # thumb
        ]

        self.mocap_body_targets = [
            "index_mcp_target",
            "middle_mcp_target",
            "ring_mcp_target",
            "pinky_mcp_target",
            "thumb_flexor_target",
            "thumb_rotator_target",
        ]

        self.mocap_site_targets = [
            "index_fingertip_target",
            "middle_fingertip_target",
            "ring_fingertip_target",
            "pinky_fingertip_target",
            "thumb_fingertip_target",
        ]
        self.wrist_site = "attachment_site"
        self.wrist_mocap = ["wrist_target"]

      
        self.finger_tasks = []
        self.posture_task = mink.PostureTask(self.xml_model, cost=1e-2)

        # Add fingertips to the solver tasks
        for fingertip in self.fingertip_sites:
            task = mink.FrameTask(
                frame_name=fingertip,
                frame_type="site",
                position_cost=0.5,
                orientation_cost=0.0, #no orientation cost for fingertips because they cannot rotate
                lm_damping=5.0, #default 1.0
            )
            self.finger_tasks.append(task)

        self.wrist_task = mink.FrameTask(
            frame_name=self.wrist_site,
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.1,
            lm_damping=5.0,
        )

        self.equality_task = mink.EqualityConstraintTask(
            model=self.xml_model, cost=500.0, gain=0.5, lm_damping=1e-3
        )
        self.tasks = [
            self.posture_task,
            *self.finger_tasks,
            self.equality_task,
            self.wrist_task,
        ]

        self.model = self.configuration.model
        self.data = self.configuration.data
        self.solver = "daqp"

        self.rate = RateLimiter(frequency=200.0, warn=False)
        self.dt = self.rate.dt
        self.t = 0
        
        # Add attributes for Cartesian filtering
        self.ee_position_history = []
        self.filtered_ee_positions = []



    def apply_camera_config(self, viewer, camera_config):
        """Apply camera configuration to viewer."""
        viewer.cam.azimuth = camera_config['azimuth']
        viewer.cam.elevation = camera_config['elevation'] 
        viewer.cam.distance = camera_config['distance']
        viewer.cam.lookat[:] = camera_config['lookat']
    

    def capture_frame_from_viewer(self, viewer, width=320, height=240):
        """Capture a frame from the MuJoCo viewer."""
        # Create a new context for offscreen rendering
        viewport = mujoco.MjrRect(0, 0, width, height)
        
        # Render the scene
        mujoco.mjr_render(viewport, viewer.scn, viewer.ctx)
        
        # Read pixels
        rgb_array = np.zeros((height, width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb_array, None, viewport, viewer.ctx)
        
        # Flip vertically (OpenGL has origin at bottom-left)
        rgb_array = np.flipud(rgb_array)
        
        return rgb_array
    
    def render_offscreen(self, width=640, height=480):
        """Render current scene offscreen and return RGB array."""
        # Create offscreen renderer
        ctx = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        
        # Create scene
        scn = mujoco.MjvScene(self.model, maxgeom=10000)
        
        # Create camera
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance = 2.0
        cam.azimuth = 45
        cam.elevation = -30
        cam.lookat[:] = [0, 0, 0.5]
        
        # Update scene
        mujoco.mjv_updateScene(
            self.model, self.data, mujoco.MjvOption(), None, cam, 
            mujoco.mjtCatBit.mjCAT_ALL, scn
        )
        
        # Create viewport
        viewport = mujoco.MjrRect(0, 0, width, height)
        
        # Render
        mujoco.mjr_render(viewport, scn, ctx)
        
        # Read pixels
        rgb_array = np.zeros((height, width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb_array, None, viewport, ctx)
        
        # Flip vertically (OpenGL has origin at bottom-left)
        rgb_array = np.flipud(rgb_array)
        
        return rgb_array

    def save_frame_as_png(self, rgb_array, filepath):
        """Save RGB array as PNG file."""
        img = Image.fromarray(rgb_array)
        img.save(filepath)

    def create_gif_from_frames(self, frames_folder, output_path, fps=30):
        """Create GIF from PNG frames using ffmpeg."""
        try:
            cmd = [
                'ffmpeg', '-y',  # -y to overwrite output file
                '-framerate', str(fps),
                '-pattern_type', 'glob',
                '-i', f'{frames_folder}/*.png',
                '-vf', 'palettegen=reserve_transparent=0',
                f'{frames_folder}/palette.png'
            ]
            subprocess.run(cmd, check=True, capture_output=True)
            
            cmd = [
                'ffmpeg', '-y',
                '-framerate', str(fps),
                '-pattern_type', 'glob',
                '-i', f'{frames_folder}/*.png',
                '-i', f'{frames_folder}/palette.png',
                '-lavfi', 'paletteuse',
                output_path
            ]
            subprocess.run(cmd, check=True, capture_output=True)
            
            # Clean up palette file
            os.remove(f'{frames_folder}/palette.png')
            
            print(f"Successfully created GIF: {output_path}")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Error creating GIF: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False


    def inverse_kinematics_fingertips_only(self, target_finger_pos, ik_iters=50, stability_weight=0.5, arm_stiffness=0.1):
        """
        Alternative IK solver using only fingertips as targets with whole-body control for stability.
        
        This uses hierarchical task control to:
        1. Primary: Achieve fingertip positions
        2. Secondary: Maintain arm stability (prevent drift)
        3. Tertiary: Hand posture regularization
        
        Args:
            target_finger_pos: (5, 3) array of fingertip positions [index, middle, ring, pinky, thumb]
            ik_iters: Number of IK iterations
            stability_weight: Weight for arm stability task (0-1)
            arm_stiffness: Stiffness for arm position holding (0-1)
            
        Returns:
            joint configuration
        """
        print(f"Starting fingertips-only IK with {len(target_finger_pos)} targets")
        print(f"Target shape: {target_finger_pos.shape}")
        
        # Store initial arm configuration for stability
        initial_arm_config = self.configuration.q[:7].copy()
        
        # Create separate task lists for hierarchical control
        fingertip_tasks = []
        stability_tasks = []
        
        # 1. Primary tasks: Fingertip positioning (highest priority)
        for i, fingertip in enumerate(self.fingertip_sites):
            task = mink.FrameTask(
                frame_name=fingertip,
                frame_type="site",
                position_cost=1.0,  # High priority for fingertips
                orientation_cost=0.0,
                lm_damping=1.0,
            )
            fingertip_tasks.append(task)
        
        # Set targets using mocap approach (similar to original method)
        if len(target_finger_pos) == 5:  # Ensure we have 5 fingertip targets
            offset_target_finger_pos = target_finger_pos + self.fixed_world_frame_offset
            
            # Move mocap bodies to desired positions first
            for i, fingertip in enumerate(self.fingertip_sites):
                mink.move_mocap_to_frame(
                    self.model,
                    self.data,
                    f"{self.mocap_site_targets[i]}",
                    fingertip,
                    "site",
                )
            
            # Set mocap positions
            self.data.mocap_pos[:] = offset_target_finger_pos
            
            # Recompute the simulation state
            mujoco.mj_forward(self.model, self.data)
            
            # Set the target positions for the finger tasks using mocap
            for i, finger_task in enumerate(fingertip_tasks):
                target = SE3.from_mocap_name(
                    self.model, self.data, f"{self.mocap_site_targets[i]}"
                )
                finger_task.set_target(target)
                print(f"Set target {i} for {self.mocap_site_targets[i]}")
        else:
            raise ValueError(f"Expected 5 fingertip targets, got {len(target_finger_pos)}")
        
        # 2. Secondary task: Arm stability (prevent drift)
        arm_stability_task = mink.PostureTask(
            self.xml_model, 
            cost=stability_weight,
            lm_damping=0.1
        )
        # Set target to maintain current arm configuration
        arm_stability_task.set_target(self.configuration.q)
        
        # 3. Tertiary task: Hand posture regularization
        hand_posture_task = mink.PostureTask(
            self.xml_model,
            cost=0.01,  # Low priority
            lm_damping=0.01
        )
        hand_posture_task.set_target_from_configuration(self.configuration)
        
        # 4. Equality constraints (joint limits, etc.)
        equality_task = mink.EqualityConstraintTask(
            model=self.xml_model, 
            cost=100.0, 
            gain=0.5, 
            lm_damping=1e-3
        )
        
        # Combine tasks with hierarchical prioritization
        all_tasks = fingertip_tasks + [arm_stability_task, hand_posture_task, equality_task]
        print(f"Created {len(all_tasks)} tasks total")
        
        # Iterative solving with adaptive arm constraint
        for iteration in range(ik_iters):
            # Compute current fingertip errors to adapt arm constraint
            fingertip_errors = []
            for i, task in enumerate(fingertip_tasks):
                current_pos = self.get_site_position(self.fingertip_sites[i])
                target_pos = target_finger_pos[i] + self.fixed_world_frame_offset
                error = np.linalg.norm(current_pos - target_pos)
                fingertip_errors.append(error)
            
            max_fingertip_error = max(fingertip_errors)
            
            # Adaptive arm constraint: relax when fingertips are far from targets
            if max_fingertip_error > 0.1:  # 10cm threshold
                # Allow more arm movement when targets are far
                arm_weight = stability_weight * 0.1
            elif max_fingertip_error > 0.05:  # 5cm threshold
                # Moderate arm constraint
                arm_weight = stability_weight * 0.5
            else:
                # Strong arm constraint when close to targets
                arm_weight = stability_weight * 1.0
            
            # Update arm stability task weight
            arm_stability_task.cost = arm_weight
            
            # Add arm position constraint based on current arm drift
            current_arm_drift = np.linalg.norm(self.configuration.q[:7] - initial_arm_config)
            if current_arm_drift > 0.5:  # 0.5 rad threshold
                # Increase arm stiffness if drifting too much
                arm_stability_task.cost = stability_weight * 2.0
                print(f"Iteration {iteration}: High arm drift ({current_arm_drift:.3f}), increasing stability")
            
            # Solve IK step
            try:
                vel = mink.solve_ik(
                    self.configuration, 
                    all_tasks, 
                    self.rate.dt, 
                    self.solver, 
                    1e-6
                )
                self.configuration.integrate_inplace(vel, self.rate.dt)
            except Exception as e:
                print(f"IK solver failed at iteration {iteration}: {e}")
                print(f"Configuration shape: {self.configuration.q.shape}")
                print(f"Number of tasks: {len(all_tasks)}")
                break
            
            # Check convergence
            if max_fingertip_error < 0.01 and current_arm_drift < 0.1:
                print(f"Converged at iteration {iteration}")
                break
                
            # Optional: Print progress every 10 iterations
            if iteration % 10 == 0:
                print(f"Iter {iteration}: Max fingertip error: {max_fingertip_error:.4f}, Arm drift: {current_arm_drift:.4f}")
        
        return self.configuration.q
    
    def get_site_position(self, site_name):
        """Helper function to get current position of a site."""
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        return self.data.site_xpos[site_id].copy()
    
    def inverse_kinematics_hierarchical(self, target_finger_pos, ik_iters=50, use_nullspace=True):
        """
        Advanced hierarchical IK solver with nullspace projection for whole-body control.
        
        This implements a proper hierarchical task control where:
        1. Fingertip tasks have absolute priority
        2. Arm stability is projected into the nullspace of fingertip tasks
        3. Hand posture is projected into the nullspace of both
        
        Args:
            target_finger_pos: (5, 3) array of fingertip positions
            ik_iters: Number of iterations
            use_nullspace: Whether to use nullspace projection
            
        Returns:
            joint configuration
        """
        from scipy.linalg import pinv
        
        # Store reference configurations
        initial_arm_config = self.configuration.q[:7].copy()
        reference_config = self.configuration.q.copy()
        
        for iteration in range(ik_iters):
            # Update forward kinematics
            mujoco.mj_forward(self.model, self.data)
            
            # 1. PRIORITY 1: Fingertip positioning
            fingertip_jacobian = []
            fingertip_errors = []
            
            for i, site_name in enumerate(self.fingertip_sites):
                # Get current position and Jacobian
                site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
                current_pos = self.data.site_xpos[site_id].copy()
                target_pos = target_finger_pos[i] + self.fixed_world_frame_offset
                
                # Position error
                pos_error = target_pos - current_pos
                fingertip_errors.extend(pos_error)
                
                # Compute Jacobian for this site
                jac_pos = np.zeros((3, self.model.nv))
                jac_rot = np.zeros((3, self.model.nv))
                mujoco.mj_jacSite(self.model, self.data, jac_pos, jac_rot, site_id)
                fingertip_jacobian.append(jac_pos)
            
            # Stack fingertip Jacobians and errors
            J_fingertips = np.vstack(fingertip_jacobian)  # (15, nv)
            e_fingertips = np.array(fingertip_errors)      # (15,)
            
            # 2. PRIORITY 2: Arm stability (only first 7 DOF)
            arm_error = initial_arm_config - self.configuration.q[:7]
            J_arm = np.zeros((7, self.model.nv))
            J_arm[:7, :7] = np.eye(7)  # Only affect arm joints
            
            # 3. PRIORITY 3: Hand posture
            hand_error = reference_config[7:] - self.configuration.q[7:]
            J_hand = np.zeros((len(hand_error), self.model.nv))
            J_hand[:, 7:] = np.eye(len(hand_error))  # Only affect hand joints
            
            if use_nullspace:
                # Hierarchical control with nullspace projection
                
                # Solve for fingertip task
                J_fing_pinv = pinv(J_fingertips, rcond=1e-6)
                dq_fingertips = J_fing_pinv @ e_fingertips
                
                # Nullspace of fingertip task
                N_fingertips = np.eye(self.model.nv) - J_fing_pinv @ J_fingertips
                
                # Project arm task into fingertip nullspace
                J_arm_projected = J_arm @ N_fingertips
                if np.linalg.matrix_rank(J_arm_projected) > 0:
                    J_arm_proj_pinv = pinv(J_arm_projected, rcond=1e-6)
                    dq_arm = J_arm_proj_pinv @ (arm_error * 0.1)  # Scale down arm correction
                else:
                    dq_arm = np.zeros(self.model.nv)
                
                # Combined nullspace for hand task
                N_combined = N_fingertips @ (np.eye(self.model.nv) - J_arm_proj_pinv @ J_arm_projected)
                J_hand_projected = J_hand @ N_combined
                if np.linalg.matrix_rank(J_hand_projected) > 0:
                    J_hand_proj_pinv = pinv(J_hand_projected, rcond=1e-6)
                    dq_hand = J_hand_proj_pinv @ (hand_error * 0.01)  # Small hand correction
                else:
                    dq_hand = np.zeros(self.model.nv)
                
                # Combine all corrections
                dq_total = dq_fingertips + dq_arm + dq_hand
                
            else:
                # Weighted least squares approach
                weights = [1.0] * 15 + [0.1] * 7 + [0.01] * len(hand_error)  # Fingertips, arm, hand
                J_combined = np.vstack([J_fingertips, J_arm, J_hand])
                e_combined = np.concatenate([e_fingertips, arm_error * 0.1, hand_error * 0.01])
                
                # Weighted pseudoinverse
                W = np.diag(weights)
                J_weighted = W @ J_combined
                e_weighted = W @ e_combined
                dq_total = pinv(J_weighted, rcond=1e-6) @ e_weighted
            
            # Apply joint velocity limits
            max_dq = 0.1  # rad per iteration
            dq_total = np.clip(dq_total, -max_dq, max_dq)
            
            # Update configuration
            self.configuration.q += dq_total * self.rate.dt
            
            # Check convergence
            fingertip_error_norm = np.linalg.norm(e_fingertips)
            arm_drift = np.linalg.norm(self.configuration.q[:7] - initial_arm_config)
            
            if fingertip_error_norm < 0.005 and arm_drift < 0.05:
                print(f"Hierarchical IK converged at iteration {iteration}")
                break
                
            if iteration % 10 == 0:
                print(f"Iter {iteration}: Fingertip error: {fingertip_error_norm:.4f}, Arm drift: {arm_drift:.4f}")
        
        return self.configuration.q

    def inverse_kinematics(self, target_finger_pos=None, target_wrist_pose=None, ik_iters=25):
        """
        Initialize at open hand configuration and move to keyframe "pinch" position
        Return joint angles for fingers
        """

        self.posture_task.set_target_from_configuration(self.configuration)

        # Move mocap bodies to desired positions
        for i, fingertip in enumerate(self.fingertip_sites):
            mink.move_mocap_to_frame(
                self.model,
                self.data,
                f"{self.mocap_site_targets[i]}",
                fingertip,
                "site",
            )

        if target_finger_pos is not None:
            offset_target_finger_pos = target_finger_pos + self.fixed_world_frame_offset
            self.data.mocap_pos[:] = (
                offset_target_finger_pos  # target_pos = (6,3) for all fingertip mocap bodies defined in scene xml
            )
        else:
            # Update mocap bodies to match the "pinch" keyframe (desired positions)
            pinch_keyframe_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_KEY, "pinch"
            )
            self.data.qpos[:] = self.model.key_qpos[pinch_keyframe_id]

        # Recompute the simulation state
        mujoco.mj_forward(self.model, self.data)

        # Set the target positions for the finger tasks
        for i, finger_task in enumerate(self.finger_tasks):
            target = SE3.from_mocap_name(
                self.model, self.data, f"{self.mocap_site_targets[i]}"
            )
            # print(f"Target {i}: {target}")
            finger_task.set_target(target)


        self.wrist_task.set_target(SE3.from_matrix(target_wrist_pose))

        for j in range(ik_iters):
            vel = mink.solve_ik(
                self.configuration, self.tasks, self.rate.dt, self.solver, 1e-5
            )
            self.configuration.integrate_inplace(vel, self.rate.dt)

        # return self.configuration.q[[7, 9, 11, 13, 16, 15]]
        return self.configuration.q
    
    def get_ee_pos(self):
        #in cartesian coordinates
        ee_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        ee_pos = self.data.site_xpos[ee_site].flatten()
        return ee_pos
    
    def causal_moving_average_cartesian(self, ee_positions, window_sizes):
        """
        Apply causal moving average filter to end effector positions with different window sizes per axis.
        
        Args:
            ee_positions: numpy array of shape (timesteps, 3) - x, y, z positions
            window_sizes: list or array of 3 integers [window_x, window_y, window_z]
        
        Returns:
            filtered_positions: numpy array of same shape as input
        """
        if len(window_sizes) != 3:
            raise ValueError("Window sizes must be a list/array of 3 elements for x, y, z axes")
        
        timesteps = ee_positions.shape[0]
        filtered_positions = np.zeros_like(ee_positions)
        
        for t in range(timesteps):
            for axis in range(3):
                window_size = window_sizes[axis]
                # Define the window start (causal - only look backwards)
                start_idx = max(0, t - window_size + 1)
                end_idx = t + 1
                
                # Calculate moving average for this axis
                window_data = ee_positions[start_idx:end_idx, axis]
                filtered_positions[t, axis] = np.mean(window_data)
        
        return filtered_positions
    
    def inverse_kinematics_cartesian_target(self, target_ee_pos, target_ee_rot=None, initial_guess=None, ik_iters=50):
        """
        Solve inverse kinematics for a target end effector position (and optionally orientation).
        
        Args:
            target_ee_pos: numpy array of shape (3,) - target end effector position
            target_ee_rot: numpy array of shape (3, 3) - target rotation matrix (optional)
            initial_guess: numpy array of shape (7,) - initial joint angles (optional)
            ik_iters: int - number of IK iterations
            
        Returns:
            joint_angles: numpy array of shape (7,) - solved joint angles
        """
        # Set initial guess if provided
        if initial_guess is not None:
            self.configuration.q[:7] = initial_guess
        
        # Create target transform
        if target_ee_rot is not None:
            target_transform = np.eye(4)
            target_transform[:3, :3] = target_ee_rot
            target_transform[:3, 3] = target_ee_pos
        else:
            # Use current orientation if none provided
            _, current_rot = self.forward_kinematics_franka(self.configuration.q[:7])
            target_transform = np.eye(4)
            target_transform[:3, :3] = current_rot
            target_transform[:3, 3] = target_ee_pos
        
        # Create a temporary task for the end effector
        ee_task = mink.FrameTask(
            frame_name="attachment_site",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.1 if target_ee_rot is not None else 0.0,
            lm_damping=1.0,
        )
        
        # Set target
        ee_task.set_target(SE3.from_matrix(target_transform))
        
        # Create temporary task list (only for Franka arm)
        temp_tasks = [ee_task]
        
        # Solve IK
        for _ in range(ik_iters):
            vel = mink.solve_ik(self.configuration, temp_tasks, self.rate.dt, self.solver, 1e-5)
            self.configuration.integrate_inplace(vel, self.rate.dt)
        
        return self.configuration.q[:7].copy()
    
    def apply_cartesian_filter_to_trajectory(self, joint_trajectory, window_sizes=[5, 5, 10]):
        """
        Apply Cartesian space filtering to a joint trajectory.
        
        Args:
            joint_trajectory: numpy array of shape (timesteps, n_joints) where first 7 are Franka joints
            window_sizes: list of 3 integers [window_x, window_y, window_z] for filtering
            
        Returns:
            filtered_joint_trajectory: numpy array of same shape as input
        """
        timesteps, n_joints = joint_trajectory.shape
        
        # Extract end effector positions from joint trajectory
        ee_positions = np.zeros((timesteps, 3))
        
        # Apply Cartesian filtering
        print(f"Applying Cartesian filter with window sizes: {window_sizes}")
        filtered_ee_positions = self.causal_moving_average_cartesian(ee_positions, window_sizes)
        import ipdb; ipdb.set_trace()
        
        # Solve IK to get filtered joint trajectory
        filtered_joint_trajectory = np.zeros_like(joint_trajectory)
        
        print("Solving inverse kinematics for filtered trajectory...")
        for t in tqdm(range(timesteps)):
            target_ee_pos = filtered_ee_positions[t]
            
            # Use previous solution as initial guess for stability
            if t > 0:
                initial_guess = filtered_joint_trajectory[t-1, :7]
            else:
                initial_guess = joint_trajectory[t, :7]
            
            # Solve IK for filtered end effector position
            filtered_franka_joints = self.inverse_kinematics_cartesian_target(
                target_ee_pos, 
                initial_guess=initial_guess,
                ik_iters=25
            )
            
            # Keep non-Franka joints unchanged
            filtered_joint_trajectory[t, :7] = filtered_franka_joints
            filtered_joint_trajectory[t, 7:] = joint_trajectory[t, 7:]
        
        return filtered_joint_trajectory
    
    def save_cartesian_filtered_data(self, folder_path, joint_trajectory, window_sizes=[5, 5, 10]):
        """
        Apply Cartesian filtering and save the results.
        
        Args:
            folder_path: str - path to save the filtered data
            joint_trajectory: numpy array - original joint trajectory
            window_sizes: list - filter window sizes for [x, y, z] axes
        """
        # Apply Cartesian filtering
        filtered_trajectory = self.apply_cartesian_filter_to_trajectory(joint_trajectory, window_sizes)
        
        # Save filtered data
        output_path = folder_path + "/robot_cmds_cartesian_filtered.pkl"
        with open(output_path, "wb") as f:
            pickle.dump(filtered_trajectory.tolist(), f)
        print(f"Saved Cartesian filtered robot commands to {output_path}")
        
        # Plot comparison
        self.plot_cartesian_filtering_comparison(folder_path, joint_trajectory, filtered_trajectory, window_sizes)
        
        return filtered_trajectory
    
    def plot_cartesian_filtering_comparison(self, folder_path, original_trajectory, filtered_trajectory, window_sizes):
        """
        Plot comparison between original and filtered trajectories in both joint and Cartesian space.
        """
        timesteps = original_trajectory.shape[0]
        
        # Compute end effector positions for both trajectories
        original_ee_pos = np.zeros((timesteps, 3))
        filtered_ee_pos = np.zeros((timesteps, 3))
        
        for t in range(timesteps):
            original_ee_pos[t], _ = self.forward_kinematics_franka(original_trajectory[t, :7])
            filtered_ee_pos[t], _ = self.forward_kinematics_franka(filtered_trajectory[t, :7])
        
        # Plot Cartesian space comparison
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Cartesian positions
        axes[0, 0].plot(original_ee_pos[:, 0], label='Original X', alpha=0.7)
        axes[0, 0].plot(filtered_ee_pos[:, 0], label=f'Filtered X (window={window_sizes[0]})', linewidth=2)
        axes[0, 0].set_title('End Effector X Position')
        axes[0, 0].legend()
        
        axes[0, 1].plot(original_ee_pos[:, 1], label='Original Y', alpha=0.7)
        axes[0, 1].plot(filtered_ee_pos[:, 1], label=f'Filtered Y (window={window_sizes[1]})', linewidth=2)
        axes[0, 1].set_title('End Effector Y Position')
        axes[0, 1].legend()
        
        axes[1, 0].plot(original_ee_pos[:, 2], label='Original Z', alpha=0.7)
        axes[1, 0].plot(filtered_ee_pos[:, 2], label=f'Filtered Z (window={window_sizes[2]})', linewidth=2)
        axes[1, 0].set_title('End Effector Z Position')
        axes[1, 0].legend()
        
        # 3D trajectory
        ax_3d = fig.add_subplot(2, 2, 4, projection='3d')
        ax_3d.plot(original_ee_pos[:, 0], original_ee_pos[:, 1], original_ee_pos[:, 2], 
                  label='Original', alpha=0.7, linewidth=1)
        ax_3d.plot(filtered_ee_pos[:, 0], filtered_ee_pos[:, 1], filtered_ee_pos[:, 2], 
                  label='Filtered', linewidth=2)
        ax_3d.set_title('3D End Effector Trajectory')
        ax_3d.legend()
        
        plt.tight_layout()
        plt.savefig(f"{folder_path}/cartesian_filtering_comparison.png", dpi=150)
        plt.close()
        print(f"Saved Cartesian filtering comparison plot to {folder_path}")
    
    def visualize_ik_trajectory(self, mano_keypoints, wrist_transforms, headless=False, save_frames=False):
        """
        Process IK trajectory with optional visualization and frame saving.
        
        Args:
            mano_keypoints: MANO keypoints array
            wrist_transforms: Wrist transformation matrices
            headless: If True, run without visualization. If False, show viewer (default: False)
            save_frames: If True, save frames as PNG and create GIF (default: False)
        """
        # Setup frame saving if requested
        self.violated_safety = []
        frames_folder = None
        if save_frames:
            frames_folder = os.path.join(self.joints_dir, "frames")
            os.makedirs(frames_folder, exist_ok=True)
            print(f"Saving frames to: {frames_folder}")
        
        def process_trajectory(viewer=None):
            """Core trajectory processing logic that works with or without viewer."""
            for i in tqdm(range(mano_keypoints.shape[0])):
                X_WKeypoints, X_WWrist = self.convert_mano_keypoints_to_mujoco(self.X_BC, mano_keypoints[i], wrist_transforms[i])

                hand_points, wrist_rot, wrist_translation = right_ability_hand_to_canonical(X_WKeypoints)
                X_WWrist[:3, :3] = wrist_rot
                X_WWrist[:3, 3] = wrist_translation
                X_WWrist[2, 3] -= 0.06 ### OFFSET FOR REAL WORLD ACCURACY

                mano_fingertips_i = X_WKeypoints[self.mano_fingertip_indices]
                mano_fingertips_i[:,2] -= 0.06 ### OFFSET FOR REAL WORLD ACCURACY
                # print(mano_fingertips_i)
                self.inverse_kinematics(mano_fingertips_i, X_WWrist, ik_iters=100)
                
                # Only plot keypoints if viewer is available
                if viewer is not None:
                    self.plot_mano_keypoints_in_mujoco(viewer, X_WKeypoints, X_WWrist)
                    time.sleep(0.01)  # wait for the viewer to update

                config_copy = self.configuration.q.copy()
                joint_qpos = np.concatenate(
                    (
                        config_copy[:7],  # franka
                        config_copy[8:9],
                        config_copy[10:11],
                        config_copy[12:13],
                        config_copy[14:15],
                        config_copy[16:18],
                    )
                )

                self.data.ctrl[:] = joint_qpos
                
                mujoco.mj_forward(self.model, self.data)
                mujoco.mj_step(self.model, self.data)

                #save ee_pos
                if i == 0:
                    self.franka_ee_pos.append(self.get_ee_pos())
                    self.joint_cmds.append(joint_qpos)
                    
                #check collision
                elif self.data.ncon > 6:
                    print(f"Collision detected at step {i}, repeat previous pose")
                    print(f"Number of contacts: {self.data.ncon}, trial {self.joints_dir}")
                    self.joint_cmds.append(self.joint_cmds[-1])
                    self.franka_ee_pos.append(self.franka_ee_pos[-1])
                    self.violated_safety.append(i)
                #check end effector distance from previous command
                else:
                    prev_ee_pos = self.franka_ee_pos[-1]
                    curr_ee_pos = self.get_ee_pos()
                    ee_dist = np.linalg.norm(curr_ee_pos - prev_ee_pos)
                    # print(f"End effector position: {ee_dist}")
                    if ee_dist > 0.2:
                        print(f"End effector distance {np.linalg.norm(curr_ee_pos - prev_ee_pos)} exceeds threshold, repeat previous pose")
                        self.joint_cmds.append(self.joint_cmds[-1])
                        self.franka_ee_pos.append(self.franka_ee_pos[-1])
                        self.violated_safety.append(i)

                    else:
                        self.joint_cmds.append(joint_qpos)
                        self.franka_ee_pos.append(self.get_ee_pos())
                        # print("valid command, saving joint commands")
                
                # Save frame if requested (works for both headless and viewer modes)
                if save_frames and frames_folder is not None:
                    if headless:
                        # Use offscreen rendering for headless mode
                        rgb_array = self.render_offscreen()
                    else:
                        # Capture from viewer if available
                        rgb_array = self.capture_frame_from_viewer(viewer)
                    
                    frame_path = os.path.join(frames_folder, f"frame_{i:06d}.png")
                    self.save_frame_as_png(rgb_array, frame_path)
                
                # Only sync viewer if available
                if viewer is not None:
                    viewer.sync()
                    
                self.rate.sleep()
                self.t += self.dt

        # Run with or without visualization based on headless parameter
        if headless:
            print("Running trajectory processing in headless mode...")
            process_trajectory(viewer=None)
        else:
            print("Running trajectory processing with visualization...")
            with mujoco.viewer.launch_passive(model=self.model, data=self.data) as viewer:
                mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
                while viewer.is_running():
                    process_trajectory(viewer=viewer)
                    break  # Exit after one pass through the trajectory
        
        # Create GIF if frames were saved
        if save_frames and frames_folder is not None:
            gif_path = os.path.join(self.joints_dir, "trajectory_animation.gif")
            self.create_gif_from_frames(frames_folder, gif_path, fps=30)
        
        # Save results regardless of visualization mode
        self.save_robot_cmds()
        with open(self.joints_dir + "/violated_safety.pkl", "wb") as f:
            print(f"Total safety violations: {len(self.violated_safety)} at steps {self.violated_safety}")
            pickle.dump(self.violated_safety, f)
        self.plot_franka(self.joints_dir, np.asarray(self.joint_cmds)[:,:7])
        self.plot_psyonic(self.joints_dir, np.asarray(self.joint_cmds)[:,7:])

    def visualize_ik_trajectory_fingertips_only(self, mano_keypoints, wrist_transforms, 
                                              headless=False, save_frames=False, 
                                              method='fingertips_only', stability_weight=0.3):
        """
        Alternative trajectory visualization using fingertips-only IK.
        
        Args:
            mano_keypoints: MANO keypoints array
            wrist_transforms: Wrist transformation matrices
            headless: If True, run without visualization
            save_frames: If True, save frames as PNG and create GIF
            method: 'fingertips_only' or 'hierarchical'
            stability_weight: Weight for arm stability (0-1)
        """
        print(f"Running trajectory with {method} IK method...")
        
        # Setup frame saving if requested
        frames_folder = None
        if save_frames:
            frames_folder = os.path.join(self.joints_dir, f"frames_{method}")
            os.makedirs(frames_folder, exist_ok=True)
            print(f"Saving frames to: {frames_folder}")
        
        # Reset joint commands and EE positions for this alternative method
        self.joint_cmds_alt = []
        self.franka_ee_pos_alt = []
        
        def process_trajectory_alt(viewer=None):
            """Core trajectory processing logic for fingertips-only methods."""
            for i in tqdm(range(mano_keypoints.shape[0])):
                X_WKeypoints, X_WWrist = self.convert_mano_keypoints_to_mujoco(
                    self.X_BC, mano_keypoints[i], wrist_transforms[i]
                )

                hand_points, wrist_rot, wrist_translation = right_ability_hand_to_canonical(X_WKeypoints)
                mano_fingertips_i = X_WKeypoints[self.mano_fingertip_indices]
                
                # Use alternative IK method
                if method == 'fingertips_only':
                    self.inverse_kinematics_fingertips_only(
                        mano_fingertips_i, ik_iters=50, stability_weight=stability_weight
                    )
                elif method == 'hierarchical':
                    self.inverse_kinematics_hierarchical(
                        mano_fingertips_i, ik_iters=50, use_nullspace=True
                    )
                else:
                    raise ValueError(f"Unknown method: {method}")
                
                # Only plot keypoints if viewer is available (no wrist arrow for fingertips-only)
                if viewer is not None:
                    self.plot_fingertips_only_in_mujoco(viewer, X_WKeypoints, mano_fingertips_i)
                    time.sleep(0.01)

                config_copy = self.configuration.q.copy()
                joint_qpos = np.concatenate(
                    (
                        config_copy[:7],  # franka
                        config_copy[8:9],
                        config_copy[10:11],
                        config_copy[12:13],
                        config_copy[14:15],
                        config_copy[16:18],
                    )
                )

                self.data.ctrl[:] = joint_qpos
                mujoco.mj_forward(self.model, self.data)
                mujoco.mj_step(self.model, self.data)

                # Save ee_pos
                if i == 0:
                    self.franka_ee_pos_alt.append(self.get_ee_pos())
                    self.joint_cmds_alt.append(joint_qpos)
                    
                # Check collision
                elif self.data.ncon > 0:
                    print(f"Collision detected at step {i}, repeat previous pose")
                    print(self)
                    self.joint_cmds_alt.append(self.joint_cmds_alt[-1])
                    self.franka_ee_pos_alt.append(self.franka_ee_pos_alt[-1])

                # Check end effector distance from previous command
                else:
                    prev_ee_pos = self.franka_ee_pos_alt[-1]
                    curr_ee_pos = self.get_ee_pos()
                    ee_dist = np.linalg.norm(curr_ee_pos - prev_ee_pos)
                    
                    if ee_dist > 0.2:
                        print(f"End effector distance {ee_dist:.3f} exceeds threshold, repeat previous pose")
                        self.joint_cmds_alt.append(self.joint_cmds_alt[-1])
                        self.franka_ee_pos_alt.append(self.franka_ee_pos_alt[-1])
                    else:
                        self.joint_cmds_alt.append(joint_qpos)
                        self.franka_ee_pos_alt.append(self.get_ee_pos())
                
                # Save frame if requested
                if save_frames and frames_folder is not None:
                    if headless:
                        rgb_array = self.render_offscreen()
                    else:
                        rgb_array = self.capture_frame_from_viewer(viewer)
                    
                    frame_path = os.path.join(frames_folder, f"frame_{i:06d}.png")
                    self.save_frame_as_png(rgb_array, frame_path)
                
                # Only sync viewer if available
                if viewer is not None:
                    viewer.sync()
                    
                self.rate.sleep()
                self.t += self.dt

        # Run with or without visualization
        if headless:
            print(f"Running {method} trajectory processing in headless mode...")
            process_trajectory_alt(viewer=None)
        else:
            print(f"Running {method} trajectory processing with visualization...")
            with mujoco.viewer.launch_passive(model=self.model, data=self.data) as viewer:
                mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
                while viewer.is_running():
                    process_trajectory_alt(viewer=viewer)
                    break
        
        # Create GIF if frames were saved
        if save_frames and frames_folder is not None:
            gif_path = os.path.join(self.joints_dir, f"trajectory_animation_{method}.gif")
            self.create_gif_from_frames(frames_folder, gif_path, fps=30)
        
        # Save results with method suffix
        self.save_robot_cmds_alt(method)
        self.plot_franka(self.joints_dir, np.asarray(self.joint_cmds_alt)[:,:7], suffix=f"_{method}")
        self.plot_psyonic(self.joints_dir, np.asarray(self.joint_cmds_alt)[:,7:], suffix=f"_{method}")
        
        print(f"Completed {method} trajectory generation!")

    def plot_fingertips_only_in_mujoco(self, viewer, mano_keypoints_world, target_fingertips):
        """
        Plot spheres for fingertips and target positions (no wrist arrow).
        """
        viewer.user_scn.ngeom = 0
        user_scn_geoms_index = 0

        # Add spheres for all MANO keypoints (small, blue)
        for i, keypoint in enumerate(mano_keypoints_world):
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[user_scn_geoms_index],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([0.003, 0.003, 0.003]),
                pos=keypoint,
                mat=np.eye(3).flatten(),
                rgba=np.array([0, 0, 1, 0.5], dtype=np.float32),  # Blue, semi-transparent
            )
            user_scn_geoms_index += 1
            viewer.user_scn.ngeom = user_scn_geoms_index

        # Add larger spheres for target fingertips (red)
        for i, target_pos in enumerate(target_fingertips):
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[user_scn_geoms_index],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([0.008, 0.008, 0.008]),
                pos=target_pos + self.fixed_world_frame_offset,
                mat=np.eye(3).flatten(),
                rgba=np.array([1, 0, 0, 1], dtype=np.float32),  # Red, opaque
            )
            user_scn_geoms_index += 1
            viewer.user_scn.ngeom = user_scn_geoms_index

        # Recompute the simulation state
        mujoco.mj_forward(self.model, self.data)

    def save_robot_cmds_alt(self, method):
        """Save alternative robot commands with method suffix."""
        filename = f"robot_cmds_{method}.pkl"
        with open(os.path.join(self.joints_dir, filename), "wb") as file:
            pickle.dump(self.joint_cmds_alt, file)
        print(f"Saved {method} joints to: {os.path.join(self.joints_dir, filename)}")

    def plot_franka(self, folder_path, franka, suffix=""):
        """Plot Franka joints with optional suffix."""
        for i in range(franka.shape[1]):
            plt.plot(franka[:,i],label=f"joint{i}")
        plt.legend()
        plt.title("Franka Joints" + suffix.replace("_", " ").title())
        plt.tight_layout()
        plt.savefig(f"{folder_path}/franka_joints{suffix}.png")
        plt.close()
        print(f"Saved Franka plot to {folder_path}/franka_joints{suffix}.png")

    def plot_psyonic(self, folder_path, psyonic, suffix=""):
        """Plot Psyonic joints with optional suffix."""
        for i in range(psyonic.shape[1]):
            plt.plot(psyonic[:,i], label=f"joint{i}")
        plt.legend()
        plt.title("Psyonic Joints" + suffix.replace("_", " ").title())
        plt.tight_layout()
        plt.savefig(f"{folder_path}/psyonic_joints{suffix}.png")
        plt.close()
        print(f"Saved Psyonic plot to {folder_path}/psyonic_joints{suffix}.png")


    def save_robot_cmds(self):
        with open(self.joints_dir+"/robot_cmds.pkl", "wb") as file:
            pickle.dump(self.joint_cmds, file)
        robot_arr = np.asarray(self.joint_cmds)
        np.save(self.joints_dir+"/robot_cmds.npy", robot_arr)
        print(f"Saved psyonic joints to:{self.joints_dir}/robot_cmds.pkl")
    

    def convert_mano_keypoints_to_mujoco(self, X_WC, p_CF, X_CWrist):
        """
        Convert MANO keypoints to Mujoco world frame using the wrist transform.
        mano_keypoints: (21, 3) array of MANO keypoints in the camera frame
        wrist_transform: (4, 4) transformation matrix from camera to Mujoco world frame
        """
        # Get finger keypoints in the world frame
        keypoints_homogenous = np.hstack((p_CF, np.ones((p_CF.shape[0], 1))))  # Convert to homogenous coordinates
        p_WF = np.array(
            [
                (X_WC @ keypoint)[:3]
                for keypoint in keypoints_homogenous
            ]
        )

        X_WWrist = X_WC @ X_CWrist  # World to wrist transform

        return p_WF, X_WWrist

    def plot_mano_keypoints_in_mujoco(self, viewer, mano_keypoints_world, wrist_transform):
        """
        Plot spheres at the positions of mano_keypoints in Mujoco.
        """
        viewer.user_scn.ngeom = 0
        user_scn_geoms_index = 0

        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[user_scn_geoms_index],
            type=mujoco.mjtGeom.mjGEOM_ARROW,
            size=np.array([0.005, 0.005, 0.1]),
            pos=wrist_transform[:3, 3],
            mat=(wrist_transform[:3, :3]).flatten(),
            rgba=np.array([1, 0, 0, 1], dtype=np.float32),
        )
        user_scn_geoms_index += 1
        viewer.user_scn.ngeom = user_scn_geoms_index

        # Add spheres to visualize the keypoints
        for i, keypoint in enumerate(mano_keypoints_world):
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[user_scn_geoms_index],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([0.005, 0.005, 0.005]),
                pos=keypoint,
                mat=np.eye(3).flatten(),
                rgba=np.array([1, 1, 0, 1], dtype=np.float32),
            )
            user_scn_geoms_index += 1
            viewer.user_scn.ngeom = user_scn_geoms_index

        # Recompute the simulation state to reflect the changes
        mujoco.mj_forward(self.model, self.data)



    def _config_q_to_joint(self):
        """
        Iterate through all joints and print their names and qpos indices. Use to find joint positions for control.
        Joint: base joint, q Index: 0
        Joint: index_mcp, q Index: 7
        Joint: index_pip, q Index: 8
        Joint: middle_mcp, q Index: 9
        Joint: middle_pip, q Index: 10
        Joint: ring_mcp, q Index: 11
        Joint: ring_pip, q Index: 12
        Joint: pinky_mcp, q Index: 13
        Joint: pinky_pip, q Index: 14
        Joint: thumb_cmc, q Index: 15
        Joint: thumb_mcp, q Index: 16

        NOTE: correct control order is [7, 9, 11, 13, 16, 15] = [index_mcp, middle_mcp, ring_mcp, pinky_mcp, thumb_mcp, thumb_cmc]
        """
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            q_index = self.model.jnt_qposadr[i]
            print(f"Joint: {joint_name}, q Index: {q_index}")

    def _check_mocap_ids(self):
        # Iterate through all bodies and print their mocap IDs
        for i in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            mocap_id = self.model.body_mocapid[i]
            print(f"Body: {body_name}, Mocap ID: {mocap_id}")

    def _debug_plot_mano_keypoints(self, keypoints):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        # Create a 3D scatter plot
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection="3d")

        # Plot keypoints
        ax.scatter(
            keypoints[:, 0],
            keypoints[:, 1],
            keypoints[:, 2],
            c="blue",
            label="Keypoints",
        )

        # Add labels and legend
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend()
        ax.set_title("3D Scatter Plot of Keypoints")

        # Show the plot
        plt.show()

    def _debug_plot_mano_mesh(self, mano_mesh):
        import pyrender
        import trimesh

        # Create a trimesh object from the mesh data
        mesh = trimesh.Trimesh(vertices=mano_mesh.vertices, faces=mano_mesh.faces)

        # Create a pyrender scene
        scene = pyrender.Scene()

        # Add the mesh to the scene
        mesh_node = pyrender.Mesh.from_trimesh(mesh)
        scene.add(mesh_node)

        # Create a viewer to visualize the scene
        viewer = pyrender.Viewer(scene, use_raymond_lighting=True)

    def _debug_single_pose(self):
        sample_data = pickle.load(
            open("./kinematics/sample_data/example_poses_001.pkl", "rb")
        )
        mano_keypoints = sample_data["pred_keypoints_3d"][0]  # just grab the first pose
        wrist_transform = sample_data["wrist"][0]
        mano_keypoints_world = self.psyonic_kinematics.convert_mano_to_mujoco(
            mano_keypoints, wrist_transform
        )
        mano_fingertips = mano_keypoints_world[
            [8, 12, 16, 20, 4]
        ]  # index, middle, ring, pinky, thumb
        self.psyonic_kinematics.inverse_kinematics(mano_fingertips)
        self.psyonic_kinematics.visualize_ik_results()

    def plot_psyonic(self, folder_path, psyonic):
        for i in range(psyonic.shape[1]):
            plt.plot(psyonic[:,i], label=f"joint{i}")
        plt.legend()
        plt.title("Psyonic Joints")
        plt.tight_layout()
        plt.savefig(f"{folder_path}/psyonic_joints.png")
        plt.close()
        print(f"Saved Psyonic plot to {folder_path}")

    def plot_franka(self, folder_path, franka):
        for i in range(franka.shape[1]):
            plt.plot(franka[:,i],label=f"joint{i}")
        plt.legend()
        plt.title("Franka Joints")
        plt.tight_layout()
        plt.savefig(f"{folder_path}/franka_joints.png")
        plt.close()
        print(f"Saved Franka plot to {folder_path}")
    
    def filter_mano_keypoints_savgol(self, keypoints, window_length=11, polyorder=3):
        """
        Apply Savitzky-Golay filter to smooth keypoints over time for each coordinate.
        
        Args:
            keypoints: numpy array of shape (T, 21, 3) - time, joints, coordinates
            window_length: int - length of the filter window (must be odd, >= polyorder+1)
            polyorder: int - order of the polynomial used for fitting
        
        Returns:
            filtered_keypoints: numpy array of same shape (T, 21, 3)
        """
        # Ensure window_length is odd and valid
        if window_length % 2 == 0:
            window_length += 1
            print(f"Window length adjusted to {window_length} (must be odd)")
        
        if window_length <= polyorder:
            polyorder = max(1, window_length - 2)
            print(f"Polynomial order adjusted to {polyorder} (must be < window_length)")
        
        # Handle case where we have fewer frames than window_length
        if keypoints.shape[0] < window_length:
            window_length = keypoints.shape[0] if keypoints.shape[0] % 2 == 1 else keypoints.shape[0] - 1
            polyorder = min(polyorder, window_length - 1)
            print(f"Adjusted window_length to {window_length} and polyorder to {polyorder} for short sequence")
        
        print(f"Applying Savitzky-Golay filter with window_length={window_length}, polyorder={polyorder}")
        
        # Initialize output array
        filtered_keypoints = np.zeros_like(keypoints)
        
        # Apply filter to each joint and coordinate separately
        for joint_idx in range(21):  # 21 MANO joints
            for coord_idx in range(3):  # x, y, z coordinates
                # Extract time series for this joint's coordinate
                time_series = keypoints[:, joint_idx, coord_idx]
                
                # Apply Savitzky-Golay filter along time axis
                filtered_time_series = savgol_filter(
                    time_series, 
                    window_length=window_length, 
                    polyorder=polyorder,
                    mode='nearest'  # Handle edges by extending with nearest values
                )
                
                # Store filtered result
                filtered_keypoints[:, joint_idx, coord_idx] = filtered_time_series
        
        return filtered_keypoints
    
    def filter_wrist_translation_savgol(self, wrist, window_length=11, polyorder=3):
        """
        Apply Savitzky-Golay filter to the z translation (element [2, 3]) of the wrist transform,
        returning the full wrist transform with only the z translation filtered.
        
        Args:
            wrist: numpy array of shape (T, 4, 4) - wrist transformation matrices
            window_length: int - length of the filter window (must be odd)
            polyorder: int - order of the polynomial used to fit the samples
            
        Returns:
            wrist_filtered: numpy array of same shape as input with filtered z translation
        """
        from scipy.signal import savgol_filter
        
        # Validate inputs
        if window_length % 2 == 0:
            window_length += 1
            print(f"Window length must be odd, adjusted to {window_length}")
        
        if window_length >= len(wrist):
            window_length = len(wrist) - 1 if len(wrist) % 2 == 0 else len(wrist) - 2
            window_length = max(3, window_length)  # Minimum window size
            print(f"Window length too large for data, adjusted to {window_length}")
            
        if polyorder >= window_length:
            polyorder = window_length - 1
            print(f"Polynomial order too high, adjusted to {polyorder}")
        
        # try:
        wrist_filtered = wrist.copy()
        z_vals = wrist[:, 2, 3]
        
        # Apply Savitzky-Golay filter to z translation
        z_filtered = savgol_filter(z_vals, window_length, polyorder)
        wrist_filtered[:, 2, 3] = z_filtered
        
        print(f"Applied Savitzky-Golay filter to wrist z translation (window={window_length}, poly={polyorder})")
        return wrist_filtered
    


@hydra.main(version_base=None, config_path="../labs/glove2robot/config", config_name="config_extract_hamer")
def main(cfg: DictConfig):
    """Main function with options to test different IK methods."""
    
    # Configuration
    test_ik_methods = False  # Set to True to test and compare IK methods
    run_full_trajectory = True  # Set to True to run full trajectory generation
    
    # Get data paths from configuration
    data_paths = cfg.paths.data_path
    camera_config = cfg.camera_calibration
    
    # Handle both single path and list of paths
    if isinstance(data_paths, str):
        data_paths = [data_paths]
    
    print(f"Processing {len(data_paths)} data directories from config:")
    for path in data_paths:
        print(f"  - {path}")
    
    for trial in data_paths:
        # Remove trailing slash if present
        trial = trial.rstrip('/')
        
        # Check if processed.pkl exists
        processed_file = trial + "/processed.pkl"
        if not os.path.exists(processed_file):
            print(f"Warning: {processed_file} does not exist, skipping {trial}")
            continue
            
        print(f"\nProcessing: {trial}")
        
        try:
            psyonic_kinematics = PsyonicKinematics(trial, camera_config)
            data = pickle.load(open(trial+"/processed.pkl", "rb"))
            mano_keypoints = data["pred_keypoints_3d"]
            wrist_transforms = data["wrist"]
            
            if test_ik_methods:
                print("\n" + "="*60)
                print("TESTING IK METHODS COMPARISON")
                print("="*60)
                
                # Test different IK methods on a subset of frames
                results = psyonic_kinematics.test_ik_methods_comparison(
                    mano_keypoints, wrist_transforms, test_frames=5
                )
                
                # Optionally visualize one of the methods
                print("\nDemonstrating fingertips-only IK...")
                psyonic_kinematics.visualize_ik_trajectory_fingertips_only(
                    mano_keypoints[:20], wrist_transforms[:20], headless=True
                )
            
            if run_full_trajectory:
                print("\nRunning full trajectory with original method...")
                
                # Choose filter type
                use_savgol_filter = True  # Set to True to use Savitzky-Golay, False for moving average
                
                if use_savgol_filter:
                    print("Using Savitzky-Golay filter for keypoints...")
                    
                    filtered_mano_keypoints = psyonic_kinematics.filter_mano_keypoints_savgol(
                        mano_keypoints, window_length=25, polyorder=3
                    )
                
                # Filter wrist transforms - now also using Savitzky-Golay filter
                    print("Using Savitzky-Golay filter for wrist translation...")
                    # Option 1: Standard Savitzky-Golay (best quality)
                    filtered_wrist_transforms = psyonic_kinematics.filter_wrist_translation_savgol(
                        wrist_transforms, window_length=25, polyorder=3
                    )
                    
                
                # Create comprehensive comparison plot
                plt.figure(figsize=(15, 10))
                
                # Plot wrist z translation
                plt.subplot(2, 3, 1)
                plt.plot(filtered_wrist_transforms[:,2,3], label="Filtered Wrist Z")
                plt.plot(wrist_transforms[:,2,3], alpha=0.7, label="Original Wrist Z")
                plt.legend()
                plt.title("Wrist Z Translation")
                plt.xlabel("Frame Index")
                plt.ylabel("Z Translation (m)")
                
                # Plot sample keypoint (e.g., index fingertip - joint 8)
                joint_idx = 8  # Index fingertip
                plt.subplot(2, 3, 2)
                plt.plot(filtered_mano_keypoints[:, joint_idx, 0], label="Filtered X")
                plt.plot(mano_keypoints[:, joint_idx, 0], alpha=0.7, label="Original X")
                plt.legend()
                plt.title(f"Joint {joint_idx} X Position")
                plt.xlabel("Frame Index")
                plt.ylabel("X Position (m)")
                
                plt.subplot(2, 3, 3)
                plt.plot(filtered_mano_keypoints[:, joint_idx, 1], label="Filtered Y")
                plt.plot(mano_keypoints[:, joint_idx, 1], alpha=0.7, label="Original Y")
                plt.legend()
                plt.title(f"Joint {joint_idx} Y Position")
                plt.xlabel("Frame Index")
                plt.ylabel("Y Position (m)")
                
                plt.subplot(2, 3, 4)
                plt.plot(filtered_mano_keypoints[:, joint_idx, 2], label="Filtered Z")
                plt.plot(mano_keypoints[:, joint_idx, 2], alpha=0.7, label="Original Z")
                plt.legend()
                plt.title(f"Joint {joint_idx} Z Position")
                plt.xlabel("Frame Index")
                plt.ylabel("Z Position (m)")
                
                # Plot filtering comparison for another joint (thumb tip - joint 4)
                joint_idx_2 = 4  # Thumb tip
                plt.subplot(2, 3, 5)
                plt.plot(filtered_mano_keypoints[:, joint_idx_2, 0], label="Filtered X")
                plt.plot(mano_keypoints[:, joint_idx_2, 0], alpha=0.7, label="Original X")
                plt.legend()
                plt.title(f"Joint {joint_idx_2} (Thumb) X Position")
                plt.xlabel("Frame Index")
                plt.ylabel("X Position (m)")
                
                # Plot RMS error between filtered and original
                plt.subplot(2, 3, 6)
                rms_error = np.sqrt(np.mean((filtered_mano_keypoints - mano_keypoints)**2, axis=(1, 2)))
                plt.plot(rms_error)
                plt.title("RMS Error (Filtered vs Original)")
                plt.xlabel("Frame Index")
                plt.ylabel("RMS Error (m)")
                
                plt.tight_layout()
                filter_type = "savgol" if use_savgol_filter else "moving_avg"
                plt.savefig(f"{trial}/filtering_comparison_{filter_type}.png", dpi=150)
                plt.close()
                
                # Run trajectory generation
                psyonic_kinematics.visualize_ik_trajectory(
                    filtered_mano_keypoints, filtered_wrist_transforms, headless=False, save_frames=False
                )
                
        except Exception as e:
            print(f"Error processing {trial}: {str(e)}")
            continue



if __name__ == "__main__":

    main()
