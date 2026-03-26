# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DCMotorModel:
    """Isaac-like DC motor saturation model used during playback."""

    effort_limit: np.ndarray
    velocity_limit: np.ndarray
    saturation_effort: np.ndarray

    def clip_torque(self, torque: np.ndarray, joint_vel: np.ndarray | None = None) -> np.ndarray:
        if joint_vel is None:
            return np.clip(torque, -self.effort_limit, self.effort_limit)

        vel_at_effort_lim = self.velocity_limit * (1.0 + self.effort_limit / self.saturation_effort)
        joint_vel = np.clip(joint_vel, -vel_at_effort_lim, vel_at_effort_lim)
        torque_speed_top = self.saturation_effort * (1.0 - joint_vel / self.velocity_limit)
        torque_speed_bottom = self.saturation_effort * (-1.0 - joint_vel / self.velocity_limit)
        max_effort = np.minimum(torque_speed_top, self.effort_limit)
        min_effort = np.maximum(torque_speed_bottom, -self.effort_limit)
        return np.clip(torque, min_effort, max_effort)
