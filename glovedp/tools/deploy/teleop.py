
import time
import numpy as np

from tqdm import tqdm
from glob import glob
from dataclasses import dataclass
from loop_rate_limiters import RateLimiter

from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.log_utils import get_deoxys_example_logger
from ah_wrapper import AHSerialClient
import time
import copy
from pydrake.all import (
    MultibodyPlant, 
    Parser, 
    RigidTransform, 
    RollPitchYaw, 
    DifferentialInverseKinematicsController, 
    DifferentialInverseKinematicsSystem,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    CollisionCheckerParams,
    DofMask,
    BusValue,
    Value,
    RotationMatrix,
    SpatialVelocity,
)
import pygame


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

INITIAL_NEUTRAL_POSITION = np.array([1.27, -1.35, -1.74, -2.36, 0.105, 2.03, 1.17])
TIMESTEP = 0.05
DELTA_RAD = 0.1
class DiffIKSolver:
    def __init__(self, initial_pos : np.ndarray):
        builder = RobotDiagramBuilder()
        self.plant = builder.plant()
        parser = Parser(self.plant)
        robot_index = parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm.urdf")[0]
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("panda_link0"))
        self.plant.Finalize()
        diagram = builder.Build()

        # self.plant_context = self.plant.CreateDefaultContext()
        # self.num_joints = self.plant.num_positions()
        params = CollisionCheckerParams(
            model=diagram, robot_model_instances=[robot_index],
            edge_step_size=0.1, env_collision_padding=0.05)
        self.collision_checker = SceneGraphCollisionChecker(params)
        Vd_TG_limit = SpatialVelocity([1, 1, 1], [1, 1, 1])
        self.recipe = DifferentialInverseKinematicsSystem.Recipe()
        self.recipe.AddIngredient(DifferentialInverseKinematicsSystem.LeastSquaresCost(DifferentialInverseKinematicsSystem.LeastSquaresCost.Config()))
        self.diffik_system = DifferentialInverseKinematicsSystem(recipe=self.recipe, task_frame="world", collision_checker=self.collision_checker, active_dof=DofMask(7, True), time_step=TIMESTEP, K_VX=1.0, Vd_TG_limit=Vd_TG_limit)
        # self.diffik_controller = DifferentialInverseKinematicsController(self.diffik_system, [])
        # self.diffik_controller_context = self.diffik_controller.CreateDefaultContext()
        # self.diffik_controller.set_initial_position(self.diffik_controller_context, initial_pos)
        # self.diffik_controller.GetInputPort("nominal_posture").FixValue(self.diffik_controller_context, np.zeros(7))
        self.diffik_system_context = self.diffik_system.CreateDefaultContext()
        self.diffik_system.GetInputPort("position").FixValue(self.diffik_system_context, initial_pos)
        self.diffik_system.GetInputPort("nominal_posture").FixValue(self.diffik_system_context, initial_pos)
        self.controller_time = 0.0
        self.bus_value = BusValue()

    
    def solve_diff_ik(self, current_pos : np.ndarray, target_pose: RigidTransform) -> np.ndarray:
        self.bus_value.Set("panda_link7", Value(target_pose))
        
        self.diffik_system.GetInputPort("position").FixValue(self.diffik_system_context, current_pos)
        self.diffik_system.GetInputPort("desired_cartesian_poses").FixValue(self.diffik_system_context, self.bus_value)
        return self.diffik_system.GetOutputPort("commanded_velocity").Eval(self.diffik_system_context) * TIMESTEP + current_pos
        # self.diffik_controller_context.SetTime(self.controller_time)
        # self.diffik_controller.GetInputPort("desired_poses").FixValue(self.diffik_controller_context, self.bus_value)
        # self.controller_time += 0.005
        # return self.diffik_controller.GetOutputPort("commanded_position").Eval(self.diffik_controller_context)


    def solve_fk(self, joint_pos: np.ndarray) -> RigidTransform:
        plant_context = self.plant.CreateDefaultContext()
        self.plant.SetPositions(plant_context, joint_pos)
        return self.plant.CalcRelativeTransform(plant_context, self.plant.world_frame(), self.plant.GetFrameByName("panda_link7"))


def test_ik():
    controller = DiffIKSolver(INITIAL_NEUTRAL_POSITION)
    forward_pose = controller.solve_fk(INITIAL_NEUTRAL_POSITION)
    # forward_pose.set_translation(forward_pose.translation() + np.array([0.1, 0.0, 0.0]))
    target = forward_pose
    next_joints = INITIAL_NEUTRAL_POSITION
    for _ in range(10):
        next_joints = controller.solve_diff_ik(next_joints, target)
        print(next_joints)
        # time.sleep(0.1)

def move_to_start(robot_interface, controller_type, controller_cfg, target_position):

    while np.linalg.norm(target_position - robot_interface.last_q) > 0.02:
        joint_position = robot_interface.last_q
        joint_delta = target_position - joint_position
        clipped_joint_delta = np.clip(joint_delta, -0.05, 0.05)
        next_joint_position = joint_position + clipped_joint_delta

        robot_interface.control(
                    controller_type=controller_type,
                    action=next_joint_position.tolist() + [-1.0],
                    controller_cfg=controller_cfg,
                )
        


def main():
    frequency = 1/TIMESTEP  # Target frequency in Hz
    interval = 1 / frequency  # Time per cycle in seconds
    pygame.init()
    pygame.joystick.init()
    for i in range(pygame.joystick.get_count()):
        game_controller = pygame.joystick.Joystick(i)
        game_controller.init()
    
    robot_interface = FrankaInterface(
        "/home/gumdev/human2robot/trajdex/tools/deploy/charmander.yml",
        use_visualizer=False
    )
    controller_cfg = YamlConfig(
        "/home/gumdev/human2robot/trajdex/tools/deploy/joint-impedance-controller.yml"
    ).as_easydict()
    controller_type = "JOINT_IMPEDANCE"

    rate_limiter = RateLimiter(frequency)

    # #### ABILITY HAND INTERFACE
    client = AHSerialClient(write_thread=False)
    print("Go to ability hand home position [30, 30, 30, 30, 30, -30]")
    ability_home = [30, 30, 30, 30, 30, -10]
    ability_grasp = [100, 100, 100, 100, 100, -10]
    client.set_position(positions=ability_home, reply_mode=2)  # Update command
    client.send_command()  # Send command
    logger = get_deoxys_example_logger()
    time.sleep(1)
    joint_position = robot_interface.last_q
    target_position = INITIAL_NEUTRAL_POSITION
    
    controller = DiffIKSolver(INITIAL_NEUTRAL_POSITION)
    initial_pose = controller.solve_fk(INITIAL_NEUTRAL_POSITION)
    start_time = time.time()
    move_to_start(robot_interface, controller_type, controller_cfg, target_position)
    print("At start position")
    delta_roll = 0
    delta_pitch = 0
    delta_yaw = 0
    grasp = False
    target_pose = copy.deepcopy(initial_pose)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                # You can perform actions based on the button number here
                if event.button == 1:
                    grasp = not grasp
                    
            elif event.type == pygame.JOYHATMOTION:
                hat_x, hat_y = event.value

                if hat_x == -1:
                    delta_yaw -= DELTA_RAD
                elif hat_x == 1:
                    delta_yaw += DELTA_RAD

                if hat_y == 1:
                    delta_pitch -= DELTA_RAD
                elif hat_y == -1:
                    delta_pitch += DELTA_RAD
            
        delta_translation = 0.4 * np.array([game_controller.get_axis(0), -game_controller.get_axis(1), -game_controller.get_axis(3)])
        delta_rotation = RollPitchYaw(delta_roll, delta_pitch, delta_yaw)
        current_time = time.time() - start_time
        target_pose.set_translation(initial_pose.translation() + delta_translation)
        target_pose.set_rotation(initial_pose.rotation() @ RotationMatrix(delta_rotation))
        next_position = controller.solve_diff_ik(robot_interface.last_q, target_pose)
        robot_interface.control(
                    controller_type=controller_type,
                    action=next_position.tolist() + [-1.0],
                    controller_cfg=controller_cfg,
                )
        if grasp:
            client.set_position(positions=ability_grasp, reply_mode=2)  # Update command
            client.send_command()  # Send commandQ
        else:
            client.set_position(positions=ability_home, reply_mode=2)  # Update command
            client.send_command()  # Send commandQ
        # joint_position = next_joint_position
        
        rate_limiter.sleep()


if __name__ == "__main__":
    main()
    # test_ik()
