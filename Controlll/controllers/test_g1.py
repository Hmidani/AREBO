from isaacsim import SimulationApp



simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.robots.robot import Robot
import numpy as np
import onnxruntime as ort
import yaml
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.rotations import quat_to_rot_matrix

import omni.appwindow  # Contains handle to keyboard
import carb.input 

ONNX_PATH = "C:\\isaacsim\\Areboprojekt\\config\\policy.onnx"
YAML_PATH = "C:\\isaacsim\\Areboprojekt\\config\\deploy.yaml"

ort_session = ort.InferenceSession(ONNX_PATH)
with open(YAML_PATH, "r") as f:
    yaml_data = yaml.safe_load(f)

decimation = yaml_data["policy"]["decimation"]
action_scale = yaml_data["policy"]["action_scale"]
num_dof = 29


my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=4 / 200)
my_world.scene.add_default_ground_plane()

g1_usd = get_assets_root_path() + "/Isaac/Robots/Unitree/G1/g1.usd"

g1_robot = my_world.scene.add(
    Robot(prim_path="/World/G1_", name="G1", usd_path=g1_usd, position=np.array([0.0, 0.0, 0.78]))

)

g1_robot.initialize()
my_world.reset()
default_joint_positions = g1_robot.get_joint_positions()

policy_counter = 0
current_action = np.zeros(num_dof)
previous_action = np.zeros(num_dof)

print("G1 robot initialized and ready to receive commands.")


while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
       
       command = np.array([0.0, 0.0, 0.0])  # Replace with your desired command values
       if keyboard.is_pressed(carb.input.KeyboardInput.UP):
           command[0] = 0.4  # Forward
         elif keyboard.is_pressed(carb.input.KeyboardInput.LEFT):
           command[2] = 0.8  # Backward
            elif keyboard.is_pressed(carb.input.KeyboardInput.RIGHT):
              command[2] = -0.8  # Turn left
           
        if policy_counter % decimation == 0:
            # Compute observation
            lin_vel_I = g1_robot.get_linear_velocity()
            ang_vel_I = g1_robot.get_angular_velocity()
            pos_IB, q_IB = g1_robot.get_world_pose()

            R_IB = quat_to_rot_matrix(q_IB)
            R_BI = R_IB.transpose()
            lin_vel_b = np.matmul(R_BI, lin_vel_I)
            ang_vel_b = np.matmul(R_BI, ang_vel_I)
            gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))
           
            joint_pos_error =g1_robot.get_joint_positions() - default_joint_positions
            joint_vel = g1_robot.get_joint_velocities()

            obs = np.zeros(81)
            obs[:3] = lin_vel_b
            obs[3:6] = ang_vel_b
            obs[6:9] = gravity_b
            obs[9:12] = command
            obs[12:41] = joint_pos_error
            obs[41:70] = joint_vel
            obs[70:81] = previous_action[:11]  # Only take the first 11 elements of previous_action
           
            obs_input
            current_joint_pos = g1_robot.get_joint_positions()
            current_joint_vel = g1_robot.get_joint_velocities()xcv
           


           