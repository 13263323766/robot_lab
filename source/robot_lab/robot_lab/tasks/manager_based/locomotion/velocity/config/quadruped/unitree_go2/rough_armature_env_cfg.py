# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from robot_lab.assets.unitree import UNITREE_GO2_ARMATURE_CFG, UNITREE_GO2_TARGET_CFG

from .rough_env_cfg import UnitreeGo2RoughEnvCfg


@configclass
class UnitreeGo2RoughArmatureEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Swap in the sim2sim-oriented training asset that adds reflected armature.
        self.scene.robot = UNITREE_GO2_ARMATURE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.disable_zero_weight_rewards()


@configclass
class UnitreeGo2RoughTargetEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Keep the current rough/stairs terrain curriculum behavior unchanged and only
        # align the robot-side dynamics toward the Unitree MuJoCo target model.
        self.scene.robot = UNITREE_GO2_TARGET_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.disable_zero_weight_rewards()


@configclass
class UnitreeGo2RoughTargetStairsHeavyEnvCfg(UnitreeGo2RoughTargetEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        terrain_generator = self.scene.terrain.terrain_generator
        if terrain_generator is None:
            return

        # Keep terrain curriculum active, but bias the terrain mix toward stairs so
        # the source policy sees stair climbing much more frequently during training.
        for name, sub_terrain in terrain_generator.sub_terrains.items():
            sub_terrain.proportion = 0.0

        terrain_generator.sub_terrains["pyramid_stairs"].proportion = 0.35
        terrain_generator.sub_terrains["pyramid_stairs_inv"].proportion = 0.35
        terrain_generator.sub_terrains["boxes"].proportion = 0.10
        terrain_generator.sub_terrains["random_rough"].proportion = 0.10
        terrain_generator.sub_terrains["hf_pyramid_slope"].proportion = 0.05
        terrain_generator.sub_terrains["hf_pyramid_slope_inv"].proportion = 0.05
