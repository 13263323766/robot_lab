# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Robot adapter and asset registry, inspired by mjlab's asset_zoo layout."""

from __future__ import annotations

from sim2sim.adapters import Sim2SimAdapter
from sim2sim.common import MujocoRobotAssetCfg

from .unitree_go2 import make_unitree_go2_isaac_velocity_adapter
from .unitree_go2_asset import GO2_MUJOCO_ASSET_CFG, get_go2_mujoco_asset_cfg


ROBOT_ADAPTER_FACTORIES: dict[str, callable] = {
    "unitree_go2_isaac_velocity": make_unitree_go2_isaac_velocity_adapter,
}
ROBOT_ASSET_FACTORIES: dict[str, callable] = {
    "unitree_go2_isaac_velocity": get_go2_mujoco_asset_cfg,
}


def make_robot_adapter(robot_name: str) -> Sim2SimAdapter:
    try:
        return ROBOT_ADAPTER_FACTORIES[robot_name]()
    except KeyError as exc:
        known = ", ".join(sorted(ROBOT_ADAPTER_FACTORIES))
        raise KeyError(f"Unknown robot adapter '{robot_name}'. Known adapters: {known}") from exc


def get_robot_asset_cfg(robot_name: str) -> MujocoRobotAssetCfg:
    try:
        return ROBOT_ASSET_FACTORIES[robot_name]()
    except KeyError as exc:
        known = ", ".join(sorted(ROBOT_ASSET_FACTORIES))
        raise KeyError(f"Unknown robot asset '{robot_name}'. Known assets: {known}") from exc


__all__ = [
    "GO2_MUJOCO_ASSET_CFG",
    "ROBOT_ADAPTER_FACTORIES",
    "ROBOT_ASSET_FACTORIES",
    "get_robot_asset_cfg",
    "get_go2_mujoco_asset_cfg",
    "make_robot_adapter",
    "make_unitree_go2_isaac_velocity_adapter",
]
