# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from robot_lab.assets.unitree import UNITREE_GO2_ARMATURE_CFG

from .rough_env_cfg import UnitreeGo2RoughEnvCfg


@configclass
class UnitreeGo2RoughArmatureEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Swap in the sim2sim-oriented training asset that adds reflected armature.
        self.scene.robot = UNITREE_GO2_ARMATURE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.disable_zero_weight_rewards()
