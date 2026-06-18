
import argparse

from isaacsim import SimulationApp



parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import omni.appwindow  # Contains handle to keyboard
import carb.input 


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    exit()
    


humanoid_asset_path = assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd"
add_reference_to_stage(usd_path=humanoid_asset_path, prim_path="/World/Humanoid")  # add robot to stage
humanoid = Articulation(prim_paths_expr="/World/Humanoid", name="my_humanoid")  # create an articulation object

humanoid.set_world_poses(positions=np.array([[0, 0.0, 1.5]]) / get_stage_units())

my_world.reset()
for i in range(4):
    print("running cycle: ", i)
    if i == 1 or i == 3:
        print("moving")
        # move the arm

    if i == 2:
        print("stopping")
        # reset the arm

    for j in range(1000):
        # step the simulation, both rendering and physics
        my_world.step(render=True)
        # print the joint positions of the car at every physics step
        if i == 3:
            joint_positions = humanoid.get_joint_positions()
            print("humanoid positions:", joint_positions)

simulation_app.close()