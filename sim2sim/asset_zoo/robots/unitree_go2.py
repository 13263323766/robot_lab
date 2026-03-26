# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Go2 sim2sim adapter definitions.

This module mirrors the role of mjlab's asset_zoo robot constants, but keeps the
scope narrow: it only describes the deployment-facing robot contract needed for
policy playback and simulator-to-simulator validation.
"""

from __future__ import annotations

import numpy as np

from sim2sim.adapters import Sim2SimAdapter, UnitreeGo2IsaacVelocityAdapter
from sim2sim.common import Sim2SimRobotSpec
from sim2sim.asset_zoo.robots.unitree_go2_asset import get_go2_mujoco_asset_cfg


def make_unitree_go2_isaac_velocity_adapter() -> Sim2SimAdapter:
    asset_cfg = get_go2_mujoco_asset_cfg()
    joint_names = asset_cfg.joint_names
    actuator_names = tuple(f"{joint_name}_motor" for joint_name in joint_names)
    actuator_name_candidates = tuple(
        (
            joint_name,
            joint_name.replace("_joint", ""),
            f"{joint_name}_actuator",
            f"{joint_name.replace('_joint', '')}_motor",
            f"{joint_name.replace('_joint', '')}_actuator",
        )
        for joint_name in joint_names
    )
    default_joint_pos = asset_cfg.default_joint_pos
    action_scale = np.asarray(
        [0.125, 0.25, 0.25] * 4,
        dtype=np.float32,
    )
    clip_low = np.full(len(joint_names), -100.0, dtype=np.float32)
    clip_high = np.full(len(joint_names), 100.0, dtype=np.float32)
    joint_limit_low = np.asarray(
        [
            -1.0472,
            -1.5708,
            -2.7227,
            -1.0472,
            -1.5708,
            -2.7227,
            -1.0472,
            -0.5236,
            -2.7227,
            -1.0472,
            -0.5236,
            -2.7227,
        ],
        dtype=np.float32,
    )
    joint_limit_high = np.asarray(
        [
            1.0472,
            3.4907,
            -0.83776,
            1.0472,
            3.4907,
            -0.83776,
            1.0472,
            4.5379,
            -0.83776,
            1.0472,
            4.5379,
            -0.83776,
        ],
        dtype=np.float32,
    )
    effort_limit = np.full(len(joint_names), 23.5, dtype=np.float32)
    velocity_limit = np.full(len(joint_names), 30.0, dtype=np.float32)
    saturation_effort = np.full(len(joint_names), 23.5, dtype=np.float32)

    spec = Sim2SimRobotSpec(
        name="unitree_go2_isaac_velocity",
        root_body_name="base",
        joint_names=joint_names,
        actuator_names=actuator_names,
        actuator_name_candidates=actuator_name_candidates,
        default_joint_pos=default_joint_pos,
        action_scale=action_scale,
        action_clip_low=clip_low,
        action_clip_high=clip_high,
        joint_limit_low=joint_limit_low,
        joint_limit_high=joint_limit_high,
        effort_limit=effort_limit,
        velocity_limit=velocity_limit,
        saturation_effort=saturation_effort,
        initial_base_pos=asset_cfg.default_base_pos,
        initial_base_quat_wxyz=asset_cfg.default_base_quat_wxyz,
        control_dt=asset_cfg.control_dt,
        actor_obs_dim=45,
    )
    return UnitreeGo2IsaacVelocityAdapter(spec)
