# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from .rough_env_cfg import UnitreeGo2RoughEnvCfg


@configclass
class UnitreeGo2StairsEnvCfg(UnitreeGo2RoughEnvCfg):
    """Go2 rough locomotion environment restricted to stair terrains only."""

    def __post_init__(self):
        super().__post_init__()

        terrain_generator = self.scene.terrain.terrain_generator
        if terrain_generator is None:
            return

        # The parent only disables zero-weight rewards for the exact rough config class name.
        # This derived config keeps the same reward table, so we need to drop unused zero-weight
        # terms again to avoid unresolved wheel-specific reward configs.
        self.disable_zero_weight_rewards()

        # Keep only ascending/descending stairs so we can isolate stair-climbing behavior.
        for name, sub_terrain in terrain_generator.sub_terrains.items():
            sub_terrain.proportion = 0.0

        terrain_generator.sub_terrains["pyramid_stairs"].proportion = 0.5
        terrain_generator.sub_terrains["pyramid_stairs_inv"].proportion = 0.5
