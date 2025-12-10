#!/usr/bin/env python3
"""
Open-loop replay script for Franka arm + Ability Hand system.
Replays joint commands from a .pkl file where:
- [:7] are Franka joint angles
- [7:] are Ability Hand joint commands
"""

import os
import sys
import time
import pickle
import numpy as np
import argparse
from pathlib import Path

# Add parent directories to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
sys.path.append("/home/gumdev/ability-hand-api/python")

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from ah_wrapper import AHSerialClient
from loop_rate_limiters import RateLimiter
from utils.folder_utils import select_folder_interactive
import threading
import threading
from collections import deque


class AbilityHandPDController:
    """
    PD controller for Ability Hand that accepts 20Hz commands and runs at 500Hz.
    Easy to enable/disable for debugging.
    """
    
    def __init__(self, ability_hand_client, kp=0.5, kd=0.1, control_rate_hz=500):
        """
        Initialize PD controller.
        
        Args:
            ability_hand_client: AHSerialClient instance
            kp: Proportional gain
            kd: Derivative gain  
            control_rate_hz: Internal control loop rate (should match ability hand rate)
        """
        self.ability_hand = ability_hand_client
        self.kp = kp
        self.kd = kd
        self.control_rate_hz = control_rate_hz
        self.dt = 1.0 / control_rate_hz
        
        # State variables
        self.target_positions = np.zeros(6)  # Target positions from high-level controller
        self.current_positions = np.zeros(6)  # Current positions (if available from hand)
        self.previous_error = np.zeros(6)
        self.enabled = True
        
        # Thread control
        self.running = False
        self.control_thread = None
        self.position_lock = threading.Lock()
        
        # Rate limiter for control loop
        self.rate_limiter = RateLimiter(control_rate_hz)
        
    def set_target_positions(self, positions):
        """
        Set target positions from high-level controller (called at ~20Hz).
        
        Args:
            positions: numpy array or list of 6 joint positions
        """
        with self.position_lock:
            self.target_positions = np.array(positions)
    
    def enable(self):
        """Enable PD control."""
        self.enabled = True
        
    def disable(self):
        """Disable PD control - useful for debugging."""
        self.enabled = False
    
    def start_control_loop(self):
        """Start the 500Hz control loop in a separate thread."""
        if self.running:
            return
            
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        print(f"PD controller started at {self.control_rate_hz}Hz")
    
    def stop_control_loop(self):
        """Stop the control loop."""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        print("PD controller stopped")
    
    def _control_loop(self):
        """Internal 500Hz control loop."""
        while self.running:
            try:
                if not self.enabled:
                    # If disabled, just pass through target positions
                    with self.position_lock:
                        command_positions = self.target_positions.copy()
                else:
                    # PD control
                    with self.position_lock:
                        target = self.target_positions.copy()
                    
                    # Get current positions (if feedback is available)
                    # For now, we'll use open-loop control since position feedback might not be available
                    current = self.current_positions.copy()
                    
                    # Calculate error
                    error = target - current
                    
                    # Calculate derivative
                    error_derivative = (error - self.previous_error) / self.dt
                    
                    # PD control law
                    command_positions = target + self.kp * error + self.kd * error_derivative
                    
                    # Update previous error
                    self.previous_error = error.copy()
                    
                    # Clamp to reasonable limits (0-100 for most joints, -100 to 100 for thumb rotator)
                    command_positions = np.clip(command_positions, -100, 100)
                
                # Send command to hand
                self.ability_hand.set_position(positions=command_positions.astype(int).tolist(), reply_mode=2)
                
                # Maintain control rate
                self.rate_limiter.sleep()
                
            except Exception as e:
                print(f"PD controller error: {e}")
                break
    
    def send_direct_command(self, positions):
        """
        Bypass PD controller and send direct command (for debugging).
        
        Args:
            positions: numpy array or list of 6 joint positions
        """
        self.ability_hand.set_position(positions=positions, reply_mode=2)


class FrankaAbilityReplay:
    def __init__(self, 
                 franka_config_path="/home/gumdev/gum_ws/src/deoxys_control/deoxys/config/charmander.yml",
                 replay_rate_hz=30,
                 ability_hand_rate_hz=500,
                 use_pd_controller=True):
        """
        Initialize the replay system for Franka arm and Ability Hand.
        
        Args:
            franka_config_path: Path to Franka configuration file
            replay_rate_hz: Rate at which to replay the trajectory (Hz)
            ability_hand_rate_hz: Internal rate for Ability Hand communication (Hz)
            use_pd_controller: Whether to use PD controller for Ability Hand
        """
        self.replay_rate_hz = replay_rate_hz
        self.use_pd_controller = use_pd_controller
        self.rate_limiter = RateLimiter(replay_rate_hz)
        
        # Initialize Franka interface
        print("Initializing Franka interface...")
        self.franka_interface = FrankaInterface(
            franka_config_path, use_visualizer=False
        )
        self.controller_type = "JOINT_IMPEDANCE"
        self.controller_cfg = YamlConfig(config_root + "/joint-impedance-controller.yml").as_easydict()
        
        # Initialize Ability Hand
        print("Initializing Ability Hand...")
        self.ability_hand = AHSerialClient(
            write_thread=True,
            read_thread=True,
            rate_hz=ability_hand_rate_hz,
            auto_start_threads=True
        )
        
        # Initialize PD controller if requested
        self.pd_controller = None
        if self.use_pd_controller:
            print("Initializing PD controller...")
            self.pd_controller = AbilityHandPDController(
                ability_hand_client=self.ability_hand,
                kp=0.5,  # Adjust these gains as needed
                kd=0.1,
                control_rate_hz=ability_hand_rate_hz
            )
            # Start the PD control loop
            self.pd_controller.start_control_loop()
        
        # Wait for connections to stabilize
        time.sleep(2.0)
        print("Initialization complete!")
    
    def load_trajectory(self, pkl_path):
        """
        Load trajectory from pickle file.
        
        Args:
            pkl_path: Path to pickle file containing joint commands
            
        Returns:
            trajectory: numpy array of shape (timesteps, n_joints)
        """
        print(f"Loading trajectory from: {pkl_path}")
        
        if pkl_path.endswith('.pkl'):
            with open(pkl_path, 'rb') as f:
                data = pickle.load(f)
            if isinstance(data, list):
                trajectory = np.array(data)
            else:
                trajectory = data
        else:
            raise ValueError("File must be a .pkl file")
        
        print(f"Loaded trajectory with shape: {trajectory.shape}")
        return trajectory

    def rad_to_pos(self, joints):
        """
        Remap radians in xml to hand joint commands
        fingers: 0-1.74 rad -> 0-100
        thumb_flexor: 0-1.91 rad -> 0-100
        thumb_rotator: 0-(-2.09) rad -> 0-(-100)
        """
        new_joints = np.zeros_like(joints)
        new_joints[0] = np.degrees(joints[0]).astype(int)
        new_joints[1] = np.degrees(joints[1]).astype(int)
        new_joints[2] = np.degrees(joints[2]).astype(int)
        new_joints[3] = np.degrees(joints[3]).astype(int)
        #swap thumb motor positions
        new_joints[4] = np.degrees(joints[5]).astype(int)
        new_joints[5] = np.degrees(joints[4]).astype(int)

        return new_joints

        
    def replay_trajectory(self, trajectory, start_idx=0, end_idx=None):
        """
        Replay the loaded trajectory.
        
        Args:
            trajectory: numpy array of joint commands
            start_idx: Starting index for replay
            end_idx: Ending index for replay (None for full trajectory)
        """
        if end_idx is None:
            end_idx = len(trajectory)
        
        print(f"Replaying trajectory from index {start_idx} to {end_idx}")
        print(f"Trajectory duration: {(end_idx - start_idx) / self.replay_rate_hz:.2f} seconds")
        print("Press Ctrl+C to stop replay")
        
        try:
            for i in range(start_idx, end_idx):
                joint_cmd = trajectory[i]
                
                # Split commands
                franka_joints = joint_cmd[:7]
                ability_joints = joint_cmd[7:]
                
                # Send Franka command
                self.franka_interface.control(
                    controller_type=self.controller_type,
                    action=list(franka_joints) + [-1],  # -1 for gripper (no change)
                    controller_cfg=self.controller_cfg,
                )
                
                # Send Ability Hand command
                ability_positions = self.rad_to_pos(ability_joints)
                
                if self.pd_controller is not None:
                    # Use PD controller - send target positions at 20Hz, controller handles 500Hz internally
                    self.pd_controller.set_target_positions(ability_positions)
                else:
                    # Direct control - send commands directly to hand
                    self.ability_hand.set_position(positions=ability_positions, reply_mode=2)
                
                # Debug print every 30 steps
                if i % 30 == 0:
                    print(f"Step {i}/{end_idx}: Franka: {franka_joints[:3]:.3f}..., "
                          f"Ability: {ability_positions[:3]:.1f}...")
                
                # Maintain replay rate
                self.rate_limiter.sleep()
                
        except KeyboardInterrupt:
            print("\nReplay interrupted by user")
        except Exception as e:
            print(f"Error during replay: {e}")
        finally:
            print("Replay finished")
    
    def move_to_start_position(self, trajectory):
        """
        Move to the starting position of the trajectory.
        
        Args:
            trajectory: numpy array of joint commands
        """
        print("Moving to start position...")
        
        start_joint_cmd = trajectory[0]
        franka_joints = start_joint_cmd[:7]
        ability_joints = start_joint_cmd[7:]
        
        # Move Franka to start position
        for _ in range(50):  # Send commands for 50 steps to ensure reaching position
            self.franka_interface.control(
                controller_type=self.controller_type,
                action=list(franka_joints) + [-1],
                controller_cfg=self.controller_cfg,
            )
            time.sleep(0.02)
        
        # Move Ability Hand to start position
        ability_positions = self.rad_to_pos(ability_joints)
        
        if self.pd_controller is not None:
            # Use PD controller
            self.pd_controller.set_target_positions(ability_positions)
        else:
            # Direct control
            self.ability_hand.set_position(positions=ability_positions, reply_mode=2)
        
        print("Moved to start position. Waiting 2 seconds...")
        time.sleep(2.0)
    
    def close(self):
        """Clean up resources."""
        print("Closing connections...")
        
        # Stop PD controller if running
        if self.pd_controller is not None:
            self.pd_controller.stop_control_loop()
        
        try:
            self.ability_hand.close()
        except:
            pass
        print("Cleanup complete")


def main():
    parser = argparse.ArgumentParser(description="Replay Franka + Ability Hand trajectory")
    parser.add_argument("--pkl_path", type=str, help="Path to pickle file with joint commands")
    parser.add_argument("--rate", type=int, default=30, help="Replay rate in Hz")
    parser.add_argument("--start_idx", type=int, default=0, help="Starting index for replay")
    parser.add_argument("--end_idx", type=int, default=None, help="Ending index for replay")
    parser.add_argument("--interactive", action="store_true", help="Use interactive folder selection")
    parser.add_argument("--no_pd_controller", action="store_true", help="Disable PD controller (for debugging)")
    
    args = parser.parse_args()
    
    # Get trajectory file path
    if args.interactive or args.pkl_path is None:
        # Use interactive selection
        print("Select trajectory file...")
        data_path, _ = select_folder_interactive(
            data_dir=".", 
            current_dir=".",
            file_extensions=[".pkl"]
        )
        
        if data_path is None:
            print("No file selected. Exiting...")
            return
        pkl_path = data_path
    else:
        pkl_path = args.pkl_path
    
    if not os.path.exists(pkl_path):
        print(f"Error: File {pkl_path} does not exist")
        return
    
    # Initialize replay system
    use_pd = not args.no_pd_controller
    if not use_pd:
        print("PD controller disabled for debugging")
    
    replay_system = FrankaAbilityReplay(
        replay_rate_hz=args.rate,
        use_pd_controller=use_pd
    )
    
    try:
        # Load trajectory
        trajectory = replay_system.load_trajectory(pkl_path)
        
        # Validate trajectory shape
        if trajectory.shape[1] < 13:  # At least 7 Franka + 6 Ability Hand
            print(f"Error: Trajectory has {trajectory.shape[1]} joints, expected at least 13")
            return
        
        # Move to start position
        replay_system.move_to_start_position(trajectory)
        
        # Start replay
        end_idx = args.end_idx if args.end_idx is not None else len(trajectory)
        replay_system.replay_trajectory(trajectory, args.start_idx, end_idx)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        replay_system.close()


if __name__ == "__main__":
    main()