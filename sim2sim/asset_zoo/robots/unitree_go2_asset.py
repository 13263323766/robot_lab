# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""MuJoCo-native Go2 asset constants.

This module is intentionally closer to mjlab's robot constant files than to the
training-side adapter code. It defines model paths, initial state, collision
profiles, and actuator groups that future builders can consume.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from sim2sim.common import (
    CollisionProfileCfg,
    MujocoAssetPaths,
    MujocoRobotAssetCfg,
    PositionActuatorGroupCfg,
)


SIM2SIM_ROOT = Path(__file__).resolve().parents[2]
MODELS_ROOT = SIM2SIM_ROOT / "models" / "unitree_go2"
TRAINING_ROOT = SIM2SIM_ROOT.parent / "source" / "robot_lab" / "data" / "Robots" / "unitree" / "go2_description"
GO2_SOURCE_URDF = TRAINING_ROOT / "urdf" / "go2_description.urdf"
GO2_MESH_ROOT = TRAINING_ROOT / "meshes"

GO2_SANITIZED_URDF = MODELS_ROOT / "go2_description_mujoco.urdf"
GO2_FLOATING_XML = MODELS_ROOT / "go2_description_mujoco_floating.xml"
GO2_FEET_ONLY_XML = MODELS_ROOT / "go2_description_mujoco_feet_only.xml"
GO2_FEET_ONLY_POSITION_XML = MODELS_ROOT / "go2_description_mujoco_feet_only_position.xml"
GO2_FEET_ONLY_POSITION_ARMATURE_XML = MODELS_ROOT / "go2_description_mujoco_feet_only_position_armature.xml"


GO2_JOINT_NAMES = (
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
)

GO2_FOOT_GEOM_NAMES = (
    "FL_foot_collision",
    "FR_foot_collision",
    "RL_foot_collision",
    "RR_foot_collision",
)

GO2_DEFAULT_JOINT_POS = np.asarray(
    [0.0, 0.8, -1.5] * 4,
    dtype=np.float32,
)

GO2_DEFAULT_BASE_POS = np.asarray([0.0, 0.0, 0.335], dtype=np.float32)
GO2_DEFAULT_BASE_QUAT_WXYZ = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

# mjlab-inspired armature values, currently borrowed from the Unitree Go1 setup
# until a Go2-specific MuJoCo-native asset is authored.
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

GO2_FULL_COLLISION = CollisionProfileCfg(
    name="full_collision",
    geom_name_patterns=(".*_collision",),
    foot_geom_names=GO2_FOOT_GEOM_NAMES,
    friction=(1.0, 0.02, 0.01),
    condim=3,
)

GO2_FEET_ONLY_COLLISION = CollisionProfileCfg(
    name="feet_only_collision",
    geom_name_patterns=tuple(GO2_FOOT_GEOM_NAMES),
    foot_geom_names=GO2_FOOT_GEOM_NAMES,
    friction=(1.0, 0.02, 0.01),
    condim=3,
)

GO2_MUJOCO_ASSET_CFG = MujocoRobotAssetCfg(
    name="unitree_go2_mujoco",
    paths=MujocoAssetPaths(
        source_urdf=str(GO2_SOURCE_URDF),
        mesh_root=str(GO2_MESH_ROOT),
        sanitized_urdf=str(GO2_SANITIZED_URDF),
        floating_xml=str(GO2_FLOATING_XML),
        feet_only_xml=str(GO2_FEET_ONLY_XML),
        feet_only_position_xml=str(GO2_FEET_ONLY_POSITION_XML),
        feet_only_position_armature_xml=str(GO2_FEET_ONLY_POSITION_ARMATURE_XML),
        preferred_playback_xml=str(GO2_FLOATING_XML),
    ),
    default_base_pos=GO2_DEFAULT_BASE_POS,
    default_base_quat_wxyz=GO2_DEFAULT_BASE_QUAT_WXYZ,
    default_joint_pos=GO2_DEFAULT_JOINT_POS,
    control_dt=0.02,
    joint_names=GO2_JOINT_NAMES,
    foot_geom_names=GO2_FOOT_GEOM_NAMES,
    position_actuators=GO2_POSITION_ACTUATORS,
    collision_profiles=(GO2_FULL_COLLISION, GO2_FEET_ONLY_COLLISION),
    preferred_actuator_type="motor",
    preferred_collision_profile="full_collision",
)


def get_go2_mujoco_asset_cfg() -> MujocoRobotAssetCfg:
    """Return the reusable Go2 MuJoCo asset config."""

    return GO2_MUJOCO_ASSET_CFG
