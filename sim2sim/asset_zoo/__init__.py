# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Robot asset registry for sim2sim validation."""

from .robots import ROBOT_ADAPTER_FACTORIES, ROBOT_ASSET_FACTORIES, get_robot_asset_cfg, make_robot_adapter

__all__ = ["ROBOT_ADAPTER_FACTORIES", "ROBOT_ASSET_FACTORIES", "get_robot_asset_cfg", "make_robot_adapter"]
