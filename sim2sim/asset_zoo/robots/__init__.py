# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Robot adapter and asset registry, inspired by mjlab's asset_zoo layout."""

from __future__ import annotations

from sim2sim.adapters import Sim2SimAdapter
from sim2sim.common import MujocoRobotAssetCfg

from .unitree_g1 import make_unitree_g1_unitree_mujoco_adapter
from .unitree_g1_asset import (
    UNITREE_MUJOCO_G1_ASSET_CFG,
    get_g1_unitree_mujoco_asset_cfg,
)
from .unitree_go2 import make_unitree_go2_unitree_mujoco_adapter
from .unitree_go2_asset import (
    UNITREE_MUJOCO_GO2_ASSET_CFG,
    get_go2_unitree_mujoco_asset_cfg,
)


ROBOT_ADAPTER_FACTORIES: dict[str, callable] = {
    "unitree_g1_unitree_mujoco": make_unitree_g1_unitree_mujoco_adapter,
    "unitree_go2_unitree_mujoco": make_unitree_go2_unitree_mujoco_adapter,
}
ROBOT_ASSET_FACTORIES: dict[str, callable] = {
    "unitree_g1_unitree_mujoco": get_g1_unitree_mujoco_asset_cfg,
    "unitree_go2_unitree_mujoco": get_go2_unitree_mujoco_asset_cfg,
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
    "UNITREE_MUJOCO_G1_ASSET_CFG",
    "UNITREE_MUJOCO_GO2_ASSET_CFG",
    "ROBOT_ADAPTER_FACTORIES",
    "ROBOT_ASSET_FACTORIES",
    "get_g1_unitree_mujoco_asset_cfg",
    "get_robot_asset_cfg",
    "get_go2_unitree_mujoco_asset_cfg",
    "make_robot_adapter",
    "make_unitree_g1_unitree_mujoco_adapter",
    "make_unitree_go2_unitree_mujoco_adapter",
]
