# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from .actuators import DCMotorModel
from .backends import MujocoRobotInterface
from .common import Sim2SimCommand, Sim2SimRobotSpec


class Sim2SimAdapter(ABC):
    """Robot-specific policy adapter for MuJoCo playback."""

    def __init__(self, spec: Sim2SimRobotSpec):
        self.spec = spec
        self.motor = DCMotorModel(
            effort_limit=spec.effort_limit,
            velocity_limit=spec.velocity_limit,
            saturation_effort=spec.saturation_effort,
        )

    def clip_action(self, action: np.ndarray) -> np.ndarray:
        return np.clip(action, self.spec.action_clip_low, self.spec.action_clip_high)

    def action_to_target_pos(self, action: np.ndarray) -> np.ndarray:
        action = self.clip_action(action)
        target_pos = self.spec.default_joint_pos + action * self.spec.action_scale
        return np.clip(target_pos, self.spec.joint_limit_low, self.spec.joint_limit_high)

    def clip_torque(self, torque: np.ndarray, joint_vel: np.ndarray | None = None) -> np.ndarray:
        return self.motor.clip_torque(torque, joint_vel)

    @abstractmethod
    def build_actor_observation(
        self,
        interface: MujocoRobotInterface,
        command: Sim2SimCommand,
        last_action: np.ndarray,
    ) -> np.ndarray:
        """Build the actor observation expected by the Isaac Sim policy."""


class UnitreeGo2IsaacVelocityAdapter(Sim2SimAdapter):
    """Adapter for the current Isaac Sim Go2 velocity policy."""

    def build_actor_observation(
        self,
        interface: MujocoRobotInterface,
        command: Sim2SimCommand,
        last_action: np.ndarray,
    ) -> np.ndarray:
        joint_pos_rel = interface.get_joint_positions() - self.spec.default_joint_pos
        joint_vel = interface.get_joint_velocities()
        obs = np.concatenate(
            [
                interface.get_base_ang_vel_body() * 0.25,
                interface.get_projected_gravity(),
                command.as_array(),
                joint_pos_rel,
                joint_vel * 0.05,
                last_action,
            ],
            dtype=np.float32,
        )
        if obs.shape != (self.spec.actor_obs_dim,):
            raise ValueError(
                f"Unexpected observation shape for {self.spec.name}: {obs.shape}, expected {(self.spec.actor_obs_dim,)}"
            )
        return obs
