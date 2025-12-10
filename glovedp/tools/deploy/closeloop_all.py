import sys
sys.path.insert(0, '/home/gumdev/human2robot/glovedp/')

# NumPy compatibility fix for older versions
try:
    import numpy._core
except ImportError:
    try:
        import numpy.core as _core
        sys.modules['numpy._core'] = _core
        sys.modules['numpy._core._exceptions'] = _core._exceptions
        sys.modules['numpy._core._internal'] = _core._internal
        sys.modules['numpy._core._dtype_ctypes'] = _core._dtype_ctypes
        sys.modules['numpy._core._multiarray_umath'] = _core._multiarray_umath
        sys.modules['numpy._core.multiarray'] = _core.multiarray
        sys.modules['numpy._core.umath'] = _core.umath
        sys.modules['numpy._core.arrayprint'] = _core.arrayprint
        sys.modules['numpy._core.defchararray'] = _core.defchararray
        sys.modules['numpy._core.einsumfunc'] = _core.einsumfunc
        sys.modules['numpy._core.fromnumeric'] = _core.fromnumeric
        sys.modules['numpy._core.function_base'] = _core.function_base
        sys.modules['numpy._core.getlimits'] = _core.getlimits
        sys.modules['numpy._core.memmap'] = _core.memmap
        sys.modules['numpy._core.numeric'] = _core.numeric
        sys.modules['numpy._core.numerictypes'] = _core.numerictypes
        sys.modules['numpy._core.overrides'] = _core.overrides
        sys.modules['numpy._core.records'] = _core.records
        sys.modules['numpy._core.shape_base'] = _core.shape_base
        print("Created NumPy compatibility layer for numpy._core")
    except Exception as e:
        print(f"Warning: Could not create NumPy compatibility layer: {e}")


import time
import tyro
import numpy as np
import os
import pickle

from glob import glob
from dataclasses import dataclass

from deoxys.experimental.motion_utils import joint_interpolation_traj
from agents.diffusion.diffusion_agent_sync import DiffusionAgent
from dataset.data_processing import iterate
import rclpy
from loop_rate_limiters import RateLimiter
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from tqdm import tqdm
from deoxys.utils.log_utils import get_deoxys_example_logger
from ah_wrapper import AHSerialClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
from std_msgs.msg import Float32MultiArray
import cv2
import signal
from pynput import keyboard


class RGBImageSubscriber:
    def __init__(self, node, topic_name):
        self.bridge = CvBridge()
        self.latest_rgb_image = None  # Shared variable to store the latest image
        self.node = node
        self._lock = threading.Lock()  # Add thread safety
        # Subscribe to the RGB topic
        self.node.create_subscription(Image, topic_name, self.rgb_callback, 10)

    def rgb_callback(self, msg):
        try:
            with self._lock:
                self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")

    def get_latest_image(self):
        """Return the latest RGB image."""
        # return self.latest_rgb_image
        with self._lock:  # Missing lock protection
            return self.latest_rgb_image.copy() if self.latest_rgb_image is not None else None
    


class BowieMagSubscriber:
    def __init__(self, node, topic_name='/bowie/synced'):
        self.node = node
        self.latest_mag = None
        self.lock = threading.Lock()
        self.sub = self.node.create_subscription(
            Float32MultiArray, topic_name, self.callback, 10
        )

    def callback(self, msg):
        mag_data = msg.data[0:8] + msg.data[13:21] + msg.data[26:34] + msg.data[39:47] + msg.data[52:60]
        mag_array = np.asarray(mag_data).reshape((10,4))[:,1:]  # shape (10, 3)
        with self.lock:
        # mag_array = mag_array.flatten()  # shape (30,)
        #import ipdb; ipdb.set_tract
            delta_index = mag_array[0] - mag_array[1]
            delta_middle = mag_array[2] - mag_array[3]
            delta_ring = mag_array[4] - mag_array[5]
            delta_pinky = mag_array[6] - mag_array[7]
            delta_thumb = mag_array[8] - mag_array[9]
            mag_array = np.concatenate([delta_index, delta_middle, delta_ring, delta_pinky, delta_thumb], axis=0)  # shape (15,)
            self.latest_mag = mag_array

    def get_latest_mag(self):
        with self.lock:
            return self.latest_mag.copy() if self.latest_mag is not None else None
    
def rad_to_pos(joints):
    """
    MODIFIED FOR ONE ACTION ONLY
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
    #swap joint commands here
    new_joints[4] = np.degrees(joints[5]).astype(int)
    new_joints[5] = np.degrees(joints[4]).astype(int)

    return new_joints


def pos_to_rad(pos_joints):
    """
    Inverse of rad_to_pos:
    Convert joint commands in degrees back to radians,
    and swap joints 4 and 5 back to their original order.

    Args:
        pos_joints: numpy array of shape (N, 6) with int degrees.

    Returns:
        numpy array of shape (N, 6) with float radians.
    """
    rad_joints = np.zeros_like(pos_joints, dtype=np.float32)
    rad_joints[0] = np.radians(pos_joints[0])
    rad_joints[1] = np.radians(pos_joints[1])
    rad_joints[2] = np.radians(pos_joints[2])
    rad_joints[3] = np.radians(pos_joints[3])
    # Undo the swap of joint 4 and 5
    rad_joints[4] = np.radians(pos_joints[5])
    rad_joints[5] = np.radians(pos_joints[4])

    return rad_joints

def go_to_reset_position(robot_interface, controller_type, controller_cfg):
    print("Going to reset position...")
    reset_called = True
    rate_limiter = RateLimiter(20)
    reset_q = [-0.11092332,  0.10882856, -0.0974085,  -2.41982981,  1.63709738,  1.81585244, 0.56755592, -1.0]
    reset_traj = joint_interpolation_traj(start_q=robot_interface.last_q, end_q=reset_q, traj_interpolator_type="linear")
    for i in range(len(reset_traj)):
        robot_interface.control(
            controller_type=controller_type,
            action=reset_traj[i].tolist(),
            controller_cfg=controller_cfg,
        )
        rate_limiter.sleep()
    print("Reset position reached. Exiting.")
    return


@dataclass
class EvalConfig:
    ckpt_path: str
    include_tactile: bool
    start_pos: int
    collect_logs: bool
    use_async: bool = False
    num_diffusion_iters: int = 5
    


# Declare global variables at the top of the script
robot_interface = None
controller_type = None
controller_cfg = None
log_folder = None
obs_dicts = []
pred_actions = []
collect_logs = False

# Define a flag to ensure the reset function is called only once
reset_called = False

def on_press(key):
    global reset_called
    try:
        if key.char == 'q' and not reset_called:
            reset_called = True
            print("\n'q' key detected. Going to reset position...")
            go_to_reset_position(robot_interface, controller_type, controller_cfg)
            if collect_logs:
                save_logs(log_folder, obs_dicts, pred_actions)
    except AttributeError:
        pass

# Start a keyboard listener
# listener = keyboard.Listener(on_press=on_press)
# listener.start()

def create_unique_log_folder(base_path):
    os.makedirs(base_path, exist_ok=True)
    existing_folders = [d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))]
    existing_folders.sort()
    folder_name = f"{int(existing_folders[-1]) + 1:03}" if existing_folders else "000"
    new_folder_path = os.path.join(base_path, folder_name)
    os.makedirs(new_folder_path)
    return new_folder_path

# Function to save logs as .pkl files
def save_logs(log_folder, obs_dicts, pred_actions):
    with open(os.path.join(log_folder, "obs_dicts.pkl"), "wb") as f:
        pickle.dump(obs_dicts, f)
    with open(os.path.join(log_folder, "pred_actions.pkl"), "wb") as f:
        pickle.dump(pred_actions, f)

def main(config):
    global robot_interface, controller_type, controller_cfg, log_folder, obs_dicts, pred_actions, collect_logs

    init_pos0 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-01_09_18/robot_cmds.npy")[60, :]
    init_pos1 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_13-23_13_34/robot_cmds.npy")[60, :]
    init_pos2 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-01_09_43/robot_cmds.npy")[60, :] 
    #test beginning trajectory 10/27
    # init_pos2 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-01_09_43/robot_cmds.npy")[0,:]
    init_pos3 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_13-23_31_59/robot_cmds.npy")[60, :]
    # init_pos4 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_13-23_23_26/robot_cmds.npy")[60, :]

    #test beginning trajectory 10/27
    init_pos4 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-01_06_17/robot_cmds.npy")[0, :]
    # init_pos5 = np.load("/home/gumdev/human2robot/data/wipe_sea3/rosbag2_2025_09_14-00_58_10/robot_cmds.npy")[60, :]

    #init position from different dataset
    init_pos5 = np.load("/home/gumdev/human2robot/data/wipe_sea1/wipe3/robot_cmds.npy")[0, :]
    
    
    init_positions = [init_pos0, init_pos1, init_pos2, init_pos3, init_pos4, init_pos5]
    start_pos_idx = config.start_pos
    act_arr = init_positions[start_pos_idx]
    print(f"Using start position index {start_pos_idx}")


    #FOR BOWIE EXPERIMENTS
    # training_data_norm = pickle.load(open("/home/gumdev/human2robot/glovedp/outputs/0919_state_dinov2_delta_touch_small_ddpm/norm.pkl", "rb"))
    # lower_touch = training_data_norm["delta_touch"]["lower"]
    # upper_touch = training_data_norm["delta_touch"]["upper"] 


    # Initialize robot interface and controller configuration
    robot_interface = FrankaInterface(
        "/home/gumdev/human2robot/trajdex/tools/deploy/charmander.yml",
        use_visualizer=False
    )
    controller_cfg = YamlConfig(
        "/home/gumdev/human2robot/trajdex/tools/deploy/joint-impedance-controller.yml"
    ).as_easydict()
    controller_type = "JOINT_IMPEDANCE"

    franka_actions = act_arr[:7]
    rate_limiter = RateLimiter(20)

    #### ABILITY HAND INTERFACE
    client = AHSerialClient(write_thread=True, read_thread=True)
    ability_sim_joints = act_arr[7:]  # ability actions only
    ability_actions = rad_to_pos(ability_sim_joints)
    print("Go to ability hand home position [30, 30, 30, 30, 30, -30]")
    
    ability_home = [30, 30, 30, 30, 30, -30]
    client.set_position(positions=ability_home, reply_mode=2)  # Update command
    client.send_command()  # Send command

    dp_agent = DiffusionAgent(ckpt_path=config.ckpt_path)
    include_tactile = config.include_tactile
    collect_logs = config.collect_logs
    print(f"Include tactile: {include_tactile}")
    print(f"Collect logs: {collect_logs}")

    # Use a single node for all subscriptions
    rclpy.init()
    main_node = rclpy.create_node('main_subscriber')
    rgb_subscriber = RGBImageSubscriber(main_node, "/realsense/rgb")
    if include_tactile:
        bowie_mag_subscriber = BowieMagSubscriber(main_node, topic_name="/bowie/synced")
    
    if collect_logs:
        base_log_path = config.ckpt_path.split('.')[0]
        log_folder = create_unique_log_folder(base_log_path)
        obs_dicts = []
        pred_actions = []

    #check robot states
    logger = get_deoxys_example_logger()
    while robot_interface.state_buffer_size == 0:
        logger.warn("Robot state not received")
        time.sleep(0.5)
    
    # check ability hand
    print("Current ability hand state: ", client.hand.get_position())

    # go to initial position
    interp_steps = 100
    franka_joint_traj = joint_interpolation_traj(start_q=robot_interface.last_q, end_q=franka_actions, num_steps = interp_steps)
    ability_joint_traj = joint_interpolation_traj(start_q=np.array(ability_home), end_q=ability_actions, num_steps = interp_steps)


    input("enter to go to start position")

    for i, joint in tqdm(enumerate(franka_joint_traj)):
        robot_interface.control(
                    controller_type=controller_type,
                    action=franka_joint_traj[i].tolist() + [-1.0],
                    controller_cfg=controller_cfg,
                )
        client.set_position(positions=ability_joint_traj[i].tolist(), reply_mode=2) 
        client.send_command()
        rate_limiter.sleep()
    
    input("enter to start closed-loop control")
    
    #wait for things to spin up
    time.sleep(2.0)
    policy_rollout_time = 90.0  # seconds
    start_time = time.time()
    bowie_flag = False
    i = 0
    while rclpy.ok() and (time.time() - start_time < policy_rollout_time):
        # Spin the node to process incoming messages for all subscriptions
        print("Time: ", time.time() - start_time)

        rclpy.spin_once(main_node, timeout_sec=0.01)

        # Get the most recent RGB image
        rgb_image = rgb_subscriber.get_latest_image()

        if include_tactile:
            bowie_mag = bowie_mag_subscriber.get_latest_mag()
            if bowie_mag is None:
                print("Waiting for Bowie mag data...")
                continue

        if rgb_image is None:
            print("Waiting for RGB image...")
            continue

        # Get robot and hand states
        franka_states = robot_interface.last_q
        hand_states = pos_to_rad(np.array(client.hand.get_position()))
        states = np.concatenate([franka_states, hand_states], axis=0)
        
        # rgb_input = cv2.resize(rgb_image, (640, 360))#commented out for ros2 node
        # rgb_input = rgb_input[50:290, 100:420] #0913 and before
        rgb_input = rgb_image[50:290, 100:420] #0916 and after
        # import ipdb; ipdb.set_trace()
        # print("RGB Input: ", rgb_input.shape)
        # print("Bowie Mag: ", bowie_mag)



        ### BOWIE EXPERIMENTS
        # print("ZEROING BOWIE MAGS")
        # bowie_mag = np.zeros(15,)

        # if bowie_flag is False:
        #     bowie_flag = True
        #     constant_bowie_mag = [-207., 2088.,   72. , 176. , -95. , -12. ,-559., 2523. ,  49., -134., 2688. ,  67. , 131. ,-729. , -71.]
        # print(constant_bowie_mag)

        # print("USING RANDOM BOWIE VALS")
        # random_bowie_mag = np.random.uniform(lower_touch, upper_touch)
        # print(random_bowie_mag)


        # Create the observation dictionary
        if include_tactile:
            obs_dict = {
                'states': states,
                'base_rgb': np.stack([rgb_input], axis=0),
                'delta_touch': bowie_mag,
            }
        else:
            obs_dict = {
                'states': states,
                'base_rgb': np.stack([rgb_input], axis=0), #commented for proprioception
            }
        # print(bowie_mag)
        # Get action from the diffusion agent
        action = dp_agent.act(obs_dict)
        franka_action = action[:7].tolist() + [-1.0]
        ability_action = action[7:]
        ability_cmd = rad_to_pos(ability_action).tolist()

        # Send commands to the robot and ability hand
        robot_interface.control(
            controller_type=controller_type,
            action=franka_action,
            controller_cfg=controller_cfg,
        )

        client.set_position(positions=ability_cmd, reply_mode=2) 
        client.send_command()


        if collect_logs:
            obs_dicts.append(obs_dict)
            pred_actions.append(action)
        

        i += 1
        # Sleep to maintain the loop rate
        rate_limiter.sleep()

    if collect_logs:
        print("Saving logs...")
        save_logs(log_folder, obs_dicts, pred_actions)
        print("Logs saved.")
    
    print("Timed out. Shutting down now.")

    if rclpy.ok():
        main_node.destroy_node()
        rclpy.shutdown()
    if 'client' in locals():
        print("Closing ability hand connection...")
        client.close()
    
    time.sleep(3.0)
    #exit script
    exit(0)


    # go_to_reset_position(robot_interface, controller_type, controller_cfg)



if __name__ == "__main__":
    main(tyro.cli(EvalConfig))
