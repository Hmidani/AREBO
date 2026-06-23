from typing import Optional
import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from Controlll.controllers.policy_controller import PolicyController
from isaacsim.storage.native import get_assets_root_path

class G1FlatTerrainPolicy(PolicyController):
    """The G1 Humanoid Locomotion Policy customized for 23 DOFs"""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "g1",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        policy_file_path: Optional[str] = None, 
    ) -> None:
       
        assets_root_path = get_assets_root_path()
       
        
        if usd_path is None:
            usd_path = assets_root_path + "/Isaac/Robots/Unitree/G1/g1.usd"
           
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)
       
        
        if policy_file_path:
            self.load_policy(policy_file_path, env_yaml_path=None)
       
        self._action_scale = 0.5
       
      
        self.num_g1_dof = 23
        self._previous_action = np.zeros(self.num_g1_dof)
        self._policy_counter = 0

    def _compute_observation(self, command):
   
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(81)
       
        obs[:3] = lin_vel_b                     # Base lin vel
        obs[3:6] = ang_vel_b                    # Base ang vel
        obs[6:9] = gravity_b                    # Gravity
        obs[9:12] = command                     # Command (v_x, v_y, w_z)
       
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()

        obs[12:35] = current_joint_pos - self.default_pos
        obs[35:58] = current_joint_vel
        obs[58:81] = self._previous_action
       
        return obs

    def forward(self, dt, command):
        
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
           
        
            if hasattr(self, '_compute_action'):
                self.action = self._compute_action(obs)
            else:
        
                self.action = np.zeros(self.num_g1_dof)
               
            self._previous_action = self.action.copy()

        action = ArticulationAction(joint_positions=self.default_pos + (self.action * self._action_scale))
        self.robot.apply_action(action)

        self._policy_counter += 1

    def initialize(self):
        self.robot.initialize()
        return True
        #return super().initialize(set_articulation_props=False)