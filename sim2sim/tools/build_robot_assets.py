# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Build a robot's MuJoCo-facing assets from the registered sim2sim config."""

from __future__ import annotations

import argparse

from sim2sim import get_robot_asset_cfg
from sim2sim.tools.build_mjcf import build_mjcf_for_robot
from sim2sim.tools.prepare_urdf import prepare_urdf_for_robot


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare and build MuJoCo assets for a registered sim2sim robot.")
    parser.add_argument(
        "--robot",
        type=str,
        default="unitree_go2_isaac_velocity",
        help="Registered sim2sim robot name.",
    )
    parser.add_argument(
        "--compiled-xml",
        type=str,
        required=True,
        help="Path to the compiled MuJoCo XML produced from the raw URDF import.",
    )
    parser.add_argument(
        "--output-xml",
        type=str,
        default=None,
        help="Optional override for the generated floating-base MJCF output path.",
    )
    parser.add_argument(
        "--feet-only-collision",
        action="store_true",
        help="Generate a feet-only collision MJCF variant.",
    )
    parser.add_argument(
        "--actuator-type",
        type=str,
        choices=("motor", "position"),
        default="position",
        help="Actuator type for the generated MJCF.",
    )
    return parser


def main() -> None:
    args = build_argparser().parse_args()
    asset_cfg = get_robot_asset_cfg(args.robot)
    sanitized_urdf = prepare_urdf_for_robot(args.robot)
    output_xml = build_mjcf_for_robot(
        robot_name=args.robot,
        compiled_xml=args.compiled_xml,
        source_urdf=str(sanitized_urdf),
        output_xml=args.output_xml,
        actuator_type=args.actuator_type,
        feet_only_collision=args.feet_only_collision,
    )
    print(f"[sim2sim] robot={args.robot}")
    print(f"[sim2sim] sanitized_urdf={sanitized_urdf}")
    print(f"[sim2sim] output_xml={output_xml}")


if __name__ == "__main__":
    main()
