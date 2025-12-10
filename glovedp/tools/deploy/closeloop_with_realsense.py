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
import cv2



class RGBImageSubscriber:
    def __init__(self, node, topic_name):
        self.bridge = CvBridge()
        self.latest_rgb_image = None  # Shared variable to store the latest image
        self.node = node

        # Subscribe to the RGB topic
        self.node.create_subscription(Image, topic_name, self.rgb_callback, 10)

    def rgb_callback(self, msg):
        try:
            # Convert ROS2 Image message to a NumPy array
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")

    def get_latest_image(self):
        """Return the latest RGB image."""
        return self.latest_rgb_image

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


@dataclass
class EvalConfig:
    ckpt_path: str
    use_async: bool = False
    num_diffusion_iters: int = 5

def main(config):
    frequency = 20  # Target frequency in Hz
    interval = 1 / frequency  # Time per cycle in seconds

    rclpy.init()

    # Create a ROS2 node
    rgb_node = rclpy.create_node('rgb_subscriber')

    # Initialize the RGB image subscriber
    rgb_topic = "/realsense/rgb"  # Replace with your topic name
    rgb_subscriber = RGBImageSubscriber(rgb_node, rgb_topic)

    robot_interface = FrankaInterface(
        "/home/gumdev/human2robot/trajdex/tools/deploy/charmander.yml",
        use_visualizer=False
    )
    controller_cfg = YamlConfig(
        "/home/gumdev/human2robot/trajdex/tools/deploy/joint-impedance-controller.yml"
    ).as_easydict()
    controller_type = "JOINT_IMPEDANCE"

    act_arr = np.load("/home/gumdev/human2robot/data/wipe_sea1/wipe5/robot_cmds.npy")
    start_index = 100
    print(len(act_arr))
    franka_actions = act_arr[:, :7]
    first_joint_pos = franka_actions[start_index]
    rate_limiter = RateLimiter(20)

    #### ABILITY HAND INTERFACE
    client = AHSerialClient(write_thread=True, read_thread=True)
    ability_sim_joints = act_arr[:, 7:]  # ability actions only
    ability_actions = rad_to_pos(ability_sim_joints)
    print("Go to ability hand home position [30, 30, 30, 30, 30, -30]")
        
    ability_home = [30, 30, 30, 30, 30, -30]
    client.set_position(positions=ability_home, reply_mode=2)  # Update command
    client.send_command()  # Send command

    dp_agent = DiffusionAgent(ckpt_path=config.ckpt_path)

    #check robot states
    logger = get_deoxys_example_logger()
    while robot_interface.state_buffer_size == 0:
        logger.warn("Robot state not received")
        time.sleep(0.5)
    import ipdb; ipdb.set_trace()
    print("Current robot joint state: ", robot_interface.last_q)
    # check ability hand
    print("Current ability hand state: ", client.hand.get_position())

    # for each position, interpolate for smooth trajectory
    franka_joint_traj = first_joint_pos.copy()
    ability_joint_traj = ability_actions[start_index].copy()

 

    interp_steps = 100
    for i in range(act_arr.shape[0]):
        if i == 0:
            franka_joint_traj = np.vstack(
                (franka_joint_traj, joint_interpolation_traj(start_q=robot_interface.last_q, end_q=franka_actions[start_index], num_steps=interp_steps)))
            ability_joint_traj = np.vstack((ability_joint_traj, joint_interpolation_traj(start_q=np.array(ability_home),
                                                                                        end_q=ability_actions[start_index], num_steps=interp_steps)))

    franka_joint_traj = np.asarray(franka_joint_traj)
    ability_joint_traj = np.asarray(ability_joint_traj)
    assert franka_joint_traj.shape[0] == ability_joint_traj.shape[0]
    print("Franka joint trajectory shape: ", franka_joint_traj.shape)
    print("Ability joint trajectory shape: ", ability_joint_traj.shape)


    input("enter to go to start position")

    for i, joint in tqdm(enumerate(franka_joint_traj[:100])):
        robot_interface.control(
                    controller_type=controller_type,
                    action=franka_joint_traj[i].tolist() + [-1.0],
                    controller_cfg=controller_cfg,
                )
        
    input("enter to start closed-loop control")
    
    #wait for things to spin up
    time.sleep(2.0)
    while rclpy.ok():
        # Spin the node to process incoming messages
        rclpy.spin_once(rgb_node, timeout_sec=0.1)

        # Get the most recent RGB image
        rgb_image = rgb_subscriber.get_latest_image()

        if rgb_image is None:
            print("Waiting for RGB image...")
            continue
        rgb_input = cv2.resize(rgb_image, (640, 360))
        rgb_input = rgb_input[50:290, 100:420]
        print("RGB Input: ", rgb_input.shape)

        # Get robot and hand states
        franka_states = robot_interface.last_q
        hand_states = pos_to_rad(np.array(client.hand.get_position()))
        states = np.concatenate([franka_states, hand_states], axis=0)

        # Create the observation dictionary
        obs_dict = {
            'states': states,
            'base_rgb': np.stack([rgb_image], axis=0),  # Add the RGB image to the observation dictionary
        }
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

        # Sleep to maintain the loop rate
        rate_limiter.sleep()


if __name__ == "__main__":
    main(tyro.cli(EvalConfig))
