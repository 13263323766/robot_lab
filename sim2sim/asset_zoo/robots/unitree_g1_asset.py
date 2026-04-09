# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""G1 asset constants for the current MuJoCo sim2sim target."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from sim2sim.common import CollisionProfileCfg, MujocoAssetPaths, MujocoRobotAssetCfg


UNITREE_MUJOCO_G1_ROOT = Path("/data2/sdam/unitree_mujoco") / "unitree_robots" / "g1"
UNITREE_MUJOCO_G1_SCENE_XML = UNITREE_MUJOCO_G1_ROOT / "scene_29dof.xml"

G1_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

G1_DEFAULT_JOINT_POS = np.asarray(
    [
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        0.0,
        0.0,
        0.0,
        0.2,
        0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
        0.2,
        -0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)
G1_DEFAULT_BASE_POS = np.asarray([0.0, 0.0, 0.76], dtype=np.float32)
G1_DEFAULT_BASE_QUAT_WXYZ = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

G1_CONTACT_PROFILE = CollisionProfileCfg(
    name="unitree_g1_default_collision",
    geom_name_patterns=("left_ankle_roll_link", "right_ankle_roll_link"),
    foot_geom_names=("left_ankle_roll_link", "right_ankle_roll_link"),
    friction=(0.8, 0.02, 0.01),
    condim=6,
)

UNITREE_MUJOCO_G1_ASSET_CFG = MujocoRobotAssetCfg(
    name="unitree_g1_unitree_mujoco",
    paths=MujocoAssetPaths(
        mesh_root=str(UNITREE_MUJOCO_G1_ROOT / "meshes"),
        preferred_playback_xml=str(UNITREE_MUJOCO_G1_SCENE_XML),
    ),
    default_base_pos=G1_DEFAULT_BASE_POS,
    default_base_quat_wxyz=G1_DEFAULT_BASE_QUAT_WXYZ,
    default_joint_pos=G1_DEFAULT_JOINT_POS,
    control_dt=0.02,
    joint_names=G1_JOINT_NAMES,
    foot_geom_names=("left_ankle_roll_link", "right_ankle_roll_link"),
    position_actuators=tuple(),
    collision_profiles=(G1_CONTACT_PROFILE,),
    preferred_actuator_type="motor",
    preferred_collision_profile="unitree_g1_default_collision",
)


def get_g1_unitree_mujoco_asset_cfg() -> MujocoRobotAssetCfg:
    """Return the G1 asset config backed by Unitree's official MuJoCo model."""

    return UNITREE_MUJOCO_G1_ASSET_CFG
