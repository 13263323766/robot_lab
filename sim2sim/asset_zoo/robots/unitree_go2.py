# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Go2 sim2sim adapter definitions."""

from __future__ import annotations

import numpy as np

from sim2sim.adapters import Sim2SimAdapter, UnitreeGo2IsaacVelocityAdapter
from sim2sim.common import Sim2SimRobotSpec
from sim2sim.asset_zoo.robots.unitree_go2_asset import get_go2_unitree_mujoco_asset_cfg


def _make_go2_adapter(
    *,
    spec_name: str,
    root_body_name: str,
    actuator_names: tuple[str, ...],
    actuator_name_candidates: tuple[tuple[str, ...], ...],
    default_base_pos: np.ndarray,
    default_base_quat_wxyz: np.ndarray,
    default_joint_pos: np.ndarray,
    joint_names: tuple[str, ...],
    control_dt: float,
) -> Sim2SimAdapter:
    action_scale = np.asarray([0.125, 0.25, 0.25] * 4, dtype=np.float32)
    clip_low = np.full(len(joint_names), -100.0, dtype=np.float32)
    clip_high = np.full(len(joint_names), 100.0, dtype=np.float32)
    joint_limit_low = np.asarray(
        [-1.0472, -1.5708, -2.7227, -1.0472, -1.5708, -2.7227, -1.0472, -0.5236, -2.7227, -1.0472, -0.5236, -2.7227],
        dtype=np.float32,
    )
    joint_limit_high = np.asarray(
        [1.0472, 3.4907, -0.83776, 1.0472, 3.4907, -0.83776, 1.0472, 4.5379, -0.83776, 1.0472, 4.5379, -0.83776],
        dtype=np.float32,
    )
    effort_limit = np.full(len(joint_names), 23.5, dtype=np.float32)
    velocity_limit = np.full(len(joint_names), 30.0, dtype=np.float32)
    saturation_effort = np.full(len(joint_names), 23.5, dtype=np.float32)

    spec = Sim2SimRobotSpec(
        name=spec_name,
        root_body_name=root_body_name,
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
        initial_base_pos=default_base_pos,
        initial_base_quat_wxyz=default_base_quat_wxyz,
        control_dt=control_dt,
        actor_obs_dim=45,
    )
    return UnitreeGo2IsaacVelocityAdapter(spec)


def make_unitree_go2_unitree_mujoco_adapter() -> Sim2SimAdapter:
    asset_cfg = get_go2_unitree_mujoco_asset_cfg()
    joint_names = asset_cfg.joint_names
    actuator_names = (
        "FR_hip",
        "FR_thigh",
        "FR_calf",
        "FL_hip",
        "FL_thigh",
        "FL_calf",
        "RR_hip",
        "RR_thigh",
        "RR_calf",
        "RL_hip",
        "RL_thigh",
        "RL_calf",
    )
    actuator_name_candidates = tuple((name,) for name in actuator_names)
    return _make_go2_adapter(
        spec_name="unitree_go2_unitree_mujoco",
        root_body_name="base_link",
        actuator_names=actuator_names,
        actuator_name_candidates=actuator_name_candidates,
        default_base_pos=asset_cfg.default_base_pos,
        default_base_quat_wxyz=asset_cfg.default_base_quat_wxyz,
        default_joint_pos=asset_cfg.default_joint_pos,
        joint_names=joint_names,
        control_dt=asset_cfg.control_dt,
    )
