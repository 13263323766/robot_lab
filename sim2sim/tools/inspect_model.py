# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Inspect MuJoCo model names against a registered sim2sim robot adapter."""

from __future__ import annotations

import argparse
import json

from sim2sim import make_robot_adapter
from sim2sim.backends import MujocoRobotInterface


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a MuJoCo XML against a sim2sim robot adapter.")
    parser.add_argument("--xml-path", type=str, required=True, help="Path to MuJoCo XML model.")
    parser.add_argument(
        "--robot",
        type=str,
        default="unitree_go2_isaac_velocity",
        help="Registered sim2sim robot adapter name.",
    )
    parser.add_argument("--sim-dt", type=float, default=0.005, help="MuJoCo simulation timestep.")
    return parser


def inspect_model(xml_path: str, robot: str, sim_dt: float = 0.005) -> dict[str, object]:
    adapter = make_robot_adapter(robot)
    interface = MujocoRobotInterface(
        model_path=xml_path,
        root_body_name=adapter.spec.root_body_name,
        joint_names=adapter.spec.joint_names,
        actuator_names=adapter.spec.actuator_names,
        actuator_name_candidates=adapter.spec.actuator_name_candidates,
        sim_dt=sim_dt,
        control_dt=adapter.spec.control_dt,
    )
    return interface.describe()


def main() -> None:
    args = build_argparser().parse_args()
    print(json.dumps(inspect_model(args.xml_path, args.robot, args.sim_dt), indent=2))


__all__ = ["inspect_model", "main"]


if __name__ == "__main__":
    main()
