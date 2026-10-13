from isaacsim import SimulationApp



simulation_app = SimulationApp({"headless": False})

import argparse

target_positions = None
parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

import carb
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation

import omni.appwindow  # Contains handle to keyboard
import carb.input 

first_step = True
reset_needed = False
robots = []

# initialize robot on first step, run robot advance
def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    global target_positions
    if first_step:
        for robot in robots:
            robot.initialize()

            robot.set_gains(
                kps=np.full(robot.num_dof, 200.0), 
                kds=np.full(robot.num_dof, 10.0)
            )  # Set gains for the robot
            
            dof_names = robot.dof_names
            print(f"DOF Names for {robot.name}: {dof_names}")
            target_positions = np.zeros(robot.num_dof)

            joint_config = {
                "hip_pitch": -0.15,
                "knee": 0.35,
                "ankle_pitch": -0.2,
                "shoulder_pitch": 0.2,
                "left_shoulder_roll": 0.25,
                "right_shoulder_roll": -0.25,
                "elbow": 0.6,
                "left_wrist_roll": 0.15,
                "right_wrist_roll": -0.15,
            }
            for i, name in enumerate(dof_names):
                for key, val in joint_config.items():
                    if key in name:
                        target_positions[i] = val
                        break
            robot.set_joint_position_targets(target_positions)

            if hasattr(robot, "set_joint_position_targets"):
                robot.set_joint_position_targets(target_positions)
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
    else:
        if target_positions is not None:
            for robot in robots:
                robot.set_joint_position_targets(target_positions)


my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=8 / 200)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    exit()




humanoid_asset_path = assets_root_path + "/Isaac/Robots/Unitree/G1/g1.usd"

add_reference_to_stage(usd_path=humanoid_asset_path, prim_path="/World/G1")  # add robot to stage
robot = Articulation(prim_paths_expr="/World/G1", name="g1_robot")  # create an articulation object
my_world.scene.add(robot)
robots.append(robot)

my_world.reset()

robot.set_world_poses(positions=np.array([[0.0, 0.0, 0.75]]), orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))  # Set the robot's

my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# robot command
base_command = np.zeros(3)


class RobotKeyboardController:
    def __init__(self):
        self._base_command = [0.0, 0.0,0.0]

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard_device =self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard_device, self._on_keyboard_event)
       
    @property
    def base_command(self):
        return self._base_command
    
    def _on_keyboard_event(self, event, *args, **kwargs):
       
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
           
            if event.input == carb.input.KeyboardInput.UP:
                self._base_command = np.array([0.5, 0, 0]) 
            elif event.input == carb.input.KeyboardInput.LEFT:
                self._base_command = np.array([0.0, 0, 0.5]) 
            elif event.input == carb.input.KeyboardInput.RIGHT:
                self._base_command = np.array([0, 0, -0.5])
       
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._base_command = np.array([0.0, 0.0,0.0])
        return True
    def shutdown(self):
        self._sub_keyboard = None

keyboard_controller = RobotKeyboardController()

step_size = 1.0 / 200.0

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped():
        reset_needed = True
    if my_world.is_playing():
        base_command = keyboard_controller.base_command


simulation_app.close()