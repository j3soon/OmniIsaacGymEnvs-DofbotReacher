# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


from omniisaacgymenvs.utils.config_utils.sim_config import SimConfig
from omniisaacgymenvs.tasks.shared.reacher import ReacherTask
from omniisaacgymenvs.robots.articulations.views.dofbot_view import DofbotView
from omniisaacgymenvs.robots.articulations.dofbot import Dofbot

from omni.isaac.core.utils.torch import *
from omni.isaac.gym.vec_env import VecEnvBase

import numpy as np
import torch
import math


class DofbotReacherTask(ReacherTask):
    def __init__(
        self,
        name: str,
        sim_config: SimConfig,
        env: VecEnvBase,
        offset=None
    ) -> None:
        self._sim_config = sim_config
        self._cfg = sim_config.config
        self._task_cfg = sim_config.task_config

        self.obs_type = self._task_cfg["env"]["observationType"]
        if not (self.obs_type in ["full"]):
            raise Exception(
                "Unknown type of observations!\nobservationType should be one of: [full]")
        print("Obs type:", self.obs_type)
        self.num_obs_dict = {
            "full": 29,
            # 6: dofbot joints position (action space)
            # 6: dofbot joints velocity
            # 3: goal position
            # 4: goal rotation
            # 4: goal relative rotation
            # 6: previous action
        }

        self.object_scale = torch.tensor([0.1] * 3)
        self.goal_scale = torch.tensor([0.5] * 3)

        self._num_observations = self.num_obs_dict[self.obs_type]
        self._num_actions = 6
        self._num_states = 0

        pi = math.pi
        # For actions
        self._dof_limits = torch.tensor([[
            [-pi/2, pi/2],
            [-pi/4, pi/4],
            [-pi/4, pi/4],
            [-pi/4, pi/4],
            [-pi/2, pi/2],
            [-0.1, 0.1], # The gripper joint will be ignored, since it is not used in the Reacher task
        ]], dtype=torch.float32, device=self._cfg["sim_device"])
        # The last action space cannot be [0, 0]
        # It will introduce the following error:
        # ValueError: Expected parameter loc (Tensor of shape (2048, 6)) of distribution Normal(loc: torch.Size([2048, 6]), scale: torch.Size([2048, 6])) to satisfy the constraint Real(), but found invalid values

        ReacherTask.__init__(self, name=name, env=env)
        return

    def get_num_dof(self):
        assert self._arms.num_dof == 11
        return 6

    def get_arm(self):
        Dofbot(prim_path=self.default_zero_env_path + "/Dofbot", name="Dofbot")

    def get_arm_view(self, scene):
        arm_view = DofbotView(prim_paths_expr="/World/envs/.*/Dofbot", name="dofbot_view")
        scene.add(arm_view._end_effectors)
        return arm_view

    def get_observations(self):
        self.get_object_goal_observations()

        self.arm_dof_pos = self._arms.get_joint_positions()
        self.arm_dof_vel = self._arms.get_joint_velocities()

        if self.obs_type == "full_no_vel":
            self.compute_full_observations(True)
        elif self.obs_type == "full":
            self.compute_full_observations()
        else:
            print("Unkown observations type!")

        observations = {
            self._arms.name: {
                "obs_buf": self.obs_buf
            }
        }
        return observations

    def compute_full_observations(self, no_vel=False):
        if no_vel:
            raise NotImplementedError()
        else:
            # There are many redundant information for the simple Reacher task, but we'll keep them for now.
            self.obs_buf[:, 0:self.num_arm_dofs] = unscale(self.arm_dof_pos[:, :self.num_arm_dofs],
                self.arm_dof_lower_limits, self.arm_dof_upper_limits)
            self.obs_buf[:, self.num_arm_dofs:2*self.num_arm_dofs] = self.vel_obs_scale * self.arm_dof_vel[:, :self.num_arm_dofs]
            base = 2 * self.num_arm_dofs
            self.obs_buf[:, base+0:base+3] = self.goal_pos
            self.obs_buf[:, base+3:base+7] = self.goal_rot
            self.obs_buf[:, base+7:base+11] = quat_mul(self.object_rot, quat_conjugate(self.goal_rot))
            self.obs_buf[:, base+11:base+17] = self.actions
