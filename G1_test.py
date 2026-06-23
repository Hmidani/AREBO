from isaacsim import SimulationApp



simulation_app = SimulationApp({"headless": False})

import argparse


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
from g1_policy import G1FlatTerrainPolicy

import omni.appwindow  # Contains handle to keyboard
import carb.input 




my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=8 / 200)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    exit()




humanoid_asset_path = assets_root_path + "/Isaac/Robots/Unitree/G1/g1.usd"

robots = []
g1 = G1FlatTerrainPolicy(
    prim_path="/World/G1_",
    name="G1_",
    usd_path=assets_root_path + "/Isaac/Robots/Unitree/G1/g1.usd",
    position=np.array([0, 0, 1.05]),
)

robots.append(g1)

"""
add_reference_to_stage(usd_path=humanoid_asset_path, prim_path="/World/Humanoid")  # add robot to stage
humanoid = Articulation(prim_paths_expr="/World/Humanoid", name="my_humanoid")  # create an articulation object
humanoid.set_world_poses(positions=np.array([[0, 0, 1.05]]) / get_stage_units())
"""



my_world.reset()


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
        for robot in robots:
            robot.forward(step_size, base_command)


simulation_app.close()