# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Validate a robot policy through the sim2sim asset pipeline."""

from __future__ import annotations

import argparse
import json

from sim2sim import get_robot_asset_cfg
from sim2sim.tools.build_mjcf import build_mjcf_for_robot
from sim2sim.tools.inspect_model import inspect_model
from sim2sim.tools.play_policy import run_policy
from sim2sim.tools.prepare_urdf import prepare_urdf_for_robot


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare, inspect, and play a policy for a registered sim2sim robot.")
    parser.add_argument("--policy", type=str, required=True, help="Path to exported TorchScript/ONNX policy.")
    parser.add_argument(
        "--robot",
        type=str,
        default="unitree_go2_isaac_velocity",
        help="Registered sim2sim robot name.",
    )
    parser.add_argument(
        "--xml-path",
        type=str,
        default=None,
        help="Optional ready-made MuJoCo XML. If omitted, the asset default or a rebuilt XML is used.",
    )
    parser.add_argument(
        "--compiled-xml",
        type=str,
        default=None,
        help="Optional compiled XML from a raw URDF import. If set, assets are rebuilt before validation.",
    )
    parser.add_argument("--sim-dt", type=float, default=0.005, help="MuJoCo simulation timestep.")
    parser.add_argument("--steps", type=int, default=2000, help="Number of control steps to simulate.")
    parser.add_argument("--cmd-vx", type=float, default=0.5, help="Forward velocity command.")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="Lateral velocity command.")
    parser.add_argument("--cmd-wz", type=float, default=0.0, help="Yaw-rate command.")
    parser.add_argument("--kp", type=float, default=25.0, help="Uniform proportional gain for all joints.")
    parser.add_argument("--kd", type=float, default=0.5, help="Uniform derivative gain for all joints.")
    parser.add_argument("--render", action="store_true", help="Launch passive MuJoCo viewer.")
    parser.add_argument("--real-time", action="store_true", help="Sleep to approximate real-time playback.")
    parser.add_argument(
        "--feet-only-collision",
        action="store_true",
        help="Use feet-only collision when rebuilding the MJCF from a compiled import.",
    )
    parser.add_argument(
        "--actuator-type",
        type=str,
        choices=("motor", "position"),
        default="position",
        help="Actuator type used when rebuilding the MJCF.",
    )
    return parser


def main() -> None:
    args = build_argparser().parse_args()
    asset_cfg = get_robot_asset_cfg(args.robot)

    xml_path = args.xml_path or asset_cfg.paths.preferred_playback_xml or asset_cfg.paths.floating_xml
    if args.compiled_xml is not None:
        sanitized_urdf = prepare_urdf_for_robot(args.robot)
        xml_path = str(
            build_mjcf_for_robot(
                robot_name=args.robot,
                compiled_xml=args.compiled_xml,
                source_urdf=str(sanitized_urdf),
                output_xml=args.xml_path,
                actuator_type=args.actuator_type,
                feet_only_collision=args.feet_only_collision,
            )
        )

    info = inspect_model(xml_path=xml_path, robot=args.robot, sim_dt=args.sim_dt)
    print("[sim2sim] inspection")
    print(json.dumps(info, indent=2))

    run_policy(
        policy_path=args.policy,
        xml_path=xml_path,
        robot=args.robot,
        sim_dt=args.sim_dt,
        steps=args.steps,
        cmd_vx=args.cmd_vx,
        cmd_vy=args.cmd_vy,
        cmd_wz=args.cmd_wz,
        kp=args.kp,
        kd=args.kd,
        render=args.render,
        real_time=args.real_time,
    )


if __name__ == "__main__":
    main()
