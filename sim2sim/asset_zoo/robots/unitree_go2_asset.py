# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Go2 asset constants for the current MuJoCo sim2sim target."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from sim2sim.common import (
    CollisionProfileCfg,
    MujocoAssetPaths,
    MujocoRobotAssetCfg,
    PositionActuatorGroupCfg,
)


UNITREE_MUJOCO_ROOT = Path("/data2/sdam/unitree_mujoco") / "unitree_robots" / "go2"
UNITREE_MUJOCO_SCENE_XML = UNITREE_MUJOCO_ROOT / "scene.xml"
UNITREE_MUJOCO_SCENE_TERRAIN_XML = UNITREE_MUJOCO_ROOT / "scene_terrain.xml"

GO2_JOINT_NAMES = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
)

GO2_DEFAULT_JOINT_POS = np.asarray([0.0, 0.8, -1.5] * 4, dtype=np.float32)
GO2_DEFAULT_BASE_POS = np.asarray([0.0, 0.0, 0.335], dtype=np.float32)
GO2_DEFAULT_BASE_QUAT_WXYZ = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

GO2_HIP_THIGH_ARMATURE = 0.004026312
GO2_CALF_ARMATURE = 0.009059202

GO2_POSITION_ACTUATORS = (
    PositionActuatorGroupCfg(
        joint_name_patterns=(".*_hip_joint", ".*_thigh_joint"),
        stiffness=25.0,
        damping=0.5,
        effort_limit=23.5,
        armature=GO2_HIP_THIGH_ARMATURE,
    ),
    PositionActuatorGroupCfg(
        joint_name_patterns=(".*_calf_joint",),
        stiffness=25.0,
        damping=0.5,
        effort_limit=23.5,
        armature=GO2_CALF_ARMATURE,
    ),
)

UNITREE_FULL_COLLISION = CollisionProfileCfg(
    name="unitree_full_collision",
    geom_name_patterns=("FL", "FR", "RL", "RR"),
    foot_geom_names=("FL", "FR", "RL", "RR"),
    friction=(0.4, 0.02, 0.01),
    condim=6,
)

UNITREE_MUJOCO_GO2_ASSET_CFG = MujocoRobotAssetCfg(
    name="unitree_go2_unitree_mujoco",
    paths=MujocoAssetPaths(
        mesh_root=str(UNITREE_MUJOCO_ROOT / "assets"),
        preferred_playback_xml=str(UNITREE_MUJOCO_SCENE_XML),
    ),
    default_base_pos=GO2_DEFAULT_BASE_POS,
    default_base_quat_wxyz=GO2_DEFAULT_BASE_QUAT_WXYZ,
    default_joint_pos=GO2_DEFAULT_JOINT_POS,
    control_dt=0.02,
    joint_names=GO2_JOINT_NAMES,
    foot_geom_names=("FL", "FR", "RL", "RR"),
    position_actuators=GO2_POSITION_ACTUATORS,
    collision_profiles=(UNITREE_FULL_COLLISION,),
    preferred_actuator_type="motor",
    preferred_collision_profile="unitree_full_collision",
)


def get_go2_unitree_mujoco_asset_cfg() -> MujocoRobotAssetCfg:
    """Return the Go2 asset config backed by Unitree's official MuJoCo model."""

    return UNITREE_MUJOCO_GO2_ASSET_CFG
