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

from tqdm import tqdm
from glob import glob
from dataclasses import dataclass
from loop_rate_limiters import RateLimiter
from dataset.data_processing import iterate
from agents.diffusion.diffusion_agent_sync import DiffusionAgent


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


@dataclass
class EvalConfig:
    ckpt_path: str
    data_dir: str
    use_async: bool = False
    hardware_eval: bool = False
    num_diffusion_iters: int = 5


def main(config):
    frequency = 20  # Target frequency in Hz
    interval = 1 / frequency  # Time per cycle in seconds
    if config.hardware_eval:
        # env = FrankaAbility()
        import rclpy
        from deoxys.experimental.motion_utils import joint_interpolation_traj
        from deoxys.franka_interface import FrankaInterface
        from deoxys.utils import YamlConfig
        from deoxys.utils.log_utils import get_deoxys_example_logger
        from ah_wrapper import AHSerialClient
        rclpy.init()
        robot_interface = FrankaInterface(
            "/home/gumdev/human2robot/trajdex/tools/deploy/charmander.yml",
            use_visualizer=False
        )
        controller_cfg = YamlConfig(
            "/home/gumdev/human2robot/trajdex/tools/deploy/joint-impedance-controller.yml"
        ).as_easydict()
        controller_type = "JOINT_IMPEDANCE"

        act_arr = np.load("/home/gumdev/human2robot/data/wipe_sea1/wipe0/robot_cmds.npy")
        start_index = 15
        franka_actions = act_arr[:, :7]
        first_joint_pos = franka_actions[start_index]
        rate_limiter = RateLimiter(20)

        # #### ABILITY HAND INTERFACE
        client = AHSerialClient(write_thread=False)
        ability_sim_joints = act_arr[:, 7:]  # ability actions only
        ability_actions = rad_to_pos(ability_sim_joints)
        print("Go to ability hand home position [30, 30, 30, 30, 30, -30]")
        input("enter to continue")
        ability_home = [30, 30, 30, 30, 30, -30]
        client.set_position(positions=ability_home, reply_mode=2)  # Update command
        client.send_command()  # Send command

        logger = get_deoxys_example_logger()
        while robot_interface.state_buffer_size == 0:
            logger.warn("Robot state not received")
            time.sleep(0.5)

        # check ability hand
        # print("Current ability hand state: ", client.hand.get_position())

        # for each position, interpolate for smooth trajectory
        franka_joint_traj = first_joint_pos.copy()
        ability_joint_traj = ability_actions[start_index].copy()


        for i in range(act_arr.shape[0]):
            if i == 0:
                franka_joint_traj = np.vstack(
                    (franka_joint_traj, joint_interpolation_traj(start_q=robot_interface.last_q, end_q=franka_actions[start_index])))
                ability_joint_traj = np.vstack((ability_joint_traj, joint_interpolation_traj(start_q=np.array(ability_home),
                                                                                         end_q=ability_actions[start_index])))
        # franka_joint_traj = np.vstack((franka_joint_traj, franka_actions[1:]))
        # ability_joint_traj = np.vstack((ability_joint_traj, ability_actions[1:]))

        franka_joint_traj = np.asarray(franka_joint_traj)
        ability_joint_traj = np.asarray(ability_joint_traj)
        assert franka_joint_traj.shape[0] == ability_joint_traj.shape[0]
        print("Franka joint trajectory shape: ", franka_joint_traj.shape)
        print("Ability joint trajectory shape: ", ability_joint_traj.shape)

        for i, joint in tqdm(enumerate(franka_joint_traj[:100])):
            robot_interface.control(
                        controller_type=controller_type,
                        action=franka_joint_traj[i].tolist() + [-1.0],
                        controller_cfg=controller_cfg,
                    )
            client.set_position(positions=ability_joint_traj[i].tolist(), reply_mode=2)  # Update command
            client.send_command()  # Send command
            rate_limiter.sleep()

    dp_agent = DiffusionAgent(ckpt_path=config.ckpt_path)
    test_trajs = sorted(glob(config.data_dir + '*/'))

    for data_dir in test_trajs:
        data = iterate(data_dir, dp_agent.config, load_img=False)
        for i, obs in enumerate(data):
            action = dp_agent.act(obs)
            gt_action = obs["actions"]
            print(f'OpenLoop Error: {((action - gt_action) ** 2).sum():.4f}')
            if config.hardware_eval:
                franka_action = action[:7].tolist() + [-1.0]
                ability_action = action[7:]
                ability_cmd = rad_to_pos(ability_action).tolist()
                # gt_action_converted = gt_action.tolist() + [-1.0]
                # robot_traj = joint_interpolation_traj(start_q=robot_interface.last_q, end_q=franka_action[:-1], num_steps=5)
                # for j in range(robot_traj.shape[0]):
                #      robot_interface.control(
                #         controller_type=controller_type,
                #         action=robot_traj[j].tolist() + [-1.0],
                #         controller_cfg=controller_cfg,
                #     )
                robot_interface.control(
                    controller_type=controller_type,
                    action=franka_action,
                    controller_cfg=controller_cfg,
                )
                client.set_position(positions=ability_cmd, reply_mode=2)  # Update command
                client.send_command()  # Send command
                rate_limiter.sleep()


if __name__ == "__main__":
    main(tyro.cli(EvalConfig))
