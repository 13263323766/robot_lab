# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Sim2SimCommand:
    """Command vector expected by velocity-tracking policies."""

    lin_vel_x: float
    lin_vel_y: float
    ang_vel_z: float

    def as_array(self) -> np.ndarray:
        return np.asarray([self.lin_vel_x, self.lin_vel_y, self.ang_vel_z], dtype=np.float32)


@dataclass(frozen=True)
class Sim2SimRobotSpec:
    """Deployment-facing robot contract for sim2sim playback."""

    name: str
    root_body_name: str
    joint_names: tuple[str, ...]
    actuator_names: tuple[str, ...]
    actuator_name_candidates: tuple[tuple[str, ...], ...]
    default_joint_pos: np.ndarray
    action_scale: np.ndarray
    action_clip_low: np.ndarray
    action_clip_high: np.ndarray
    joint_limit_low: np.ndarray
    joint_limit_high: np.ndarray
    effort_limit: np.ndarray
    velocity_limit: np.ndarray
    saturation_effort: np.ndarray
    initial_base_pos: np.ndarray
    initial_base_quat_wxyz: np.ndarray
    control_dt: float
    actor_obs_dim: int

    def zero_action(self) -> np.ndarray:
        return np.zeros(len(self.joint_names), dtype=np.float32)

    def make_uniform_gains(self, kp: float, kd: float) -> tuple[np.ndarray, np.ndarray]:
        size = len(self.joint_names)
        return (
            np.full(size, kp, dtype=np.float64),
            np.full(size, kd, dtype=np.float64),
        )


@dataclass(frozen=True)
class PositionActuatorGroupCfg:
    """MuJoCo-native position actuator group description."""

    joint_name_patterns: tuple[str, ...]
    stiffness: float
    damping: float
    effort_limit: float
    armature: float | None = None


@dataclass(frozen=True)
class CollisionProfileCfg:
    """Collision profile metadata for a MuJoCo robot asset."""

    name: str
    geom_name_patterns: tuple[str, ...]
    foot_geom_names: tuple[str, ...]
    friction: tuple[float, ...]
    condim: int = 3


@dataclass(frozen=True)
class MujocoAssetPaths:
    """Resolved model paths for a sim2sim robot asset."""

    source_urdf: str
    mesh_root: str
    sanitized_urdf: str
    floating_xml: str
    feet_only_xml: str | None = None
    feet_only_position_xml: str | None = None
    feet_only_position_armature_xml: str | None = None
    preferred_playback_xml: str | None = None


@dataclass(frozen=True)
class MujocoRobotAssetCfg:
    """MuJoCo-native asset constants, inspired by mjlab asset_zoo configs."""

    name: str
    paths: MujocoAssetPaths
    default_base_pos: np.ndarray
    default_base_quat_wxyz: np.ndarray
    default_joint_pos: np.ndarray
    control_dt: float
    joint_names: tuple[str, ...]
    foot_geom_names: tuple[str, ...]
    position_actuators: tuple[PositionActuatorGroupCfg, ...]
    collision_profiles: tuple[CollisionProfileCfg, ...]
    preferred_actuator_type: str = "position"
    preferred_collision_profile: str = "feet_only_collision"
