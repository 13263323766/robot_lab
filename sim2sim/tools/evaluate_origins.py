#!/usr/bin/env python3
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Evaluate a policy over multiple exported Isaac terrain origins one by one."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from sim2sim.tools.play_policy import run_policy


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a sim2sim policy on multiple exported terrain origins.")
    parser.add_argument("--policy", type=str, required=True)
    parser.add_argument("--xml-path", type=str, required=True)
    parser.add_argument("--origins-path", type=Path, required=True)
    parser.add_argument("--robot", type=str, default="unitree_go2_unitree_mujoco")
    parser.add_argument("--sim-dt", type=float, default=0.005)
    parser.add_argument("--control-dt", type=float, default=None)
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--cmd-vx", type=float, default=0.5)
    parser.add_argument("--cmd-vy", type=float, default=0.0)
    parser.add_argument("--cmd-wz", type=float, default=0.0)
    parser.add_argument("--kp", type=float, default=25.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--index-start", type=int, default=0)
    parser.add_argument("--index-stop", type=int, default=None, help="Exclusive stop index. Defaults to all origins.")
    parser.add_argument("--index-step", type=int, default=1)
    parser.add_argument("--record-dir", type=Path, default=None, help="Optional output directory for per-origin videos.")
    parser.add_argument("--track-camera", action="store_true")
    parser.add_argument("--camera-distance", type=float, default=2.6)
    parser.add_argument("--camera-elevation", type=float, default=-30.0)
    parser.add_argument("--camera-azimuth", type=float, default=0.0)
    return parser


def main() -> None:
    args = build_argparser().parse_args()
    origins = np.load(args.origins_path.expanduser().resolve()).reshape(-1, 3)
    stop = args.index_stop if args.index_stop is not None else len(origins)
    indices = range(args.index_start, min(stop, len(origins)), args.index_step)

    if args.record_dir is not None:
        args.record_dir = args.record_dir.expanduser().resolve()
        args.record_dir.mkdir(parents=True, exist_ok=True)

    for index in indices:
        record_path = None
        if args.record_dir is not None:
            record_path = str(args.record_dir / f"origin_{index:04d}.mp4")
        print(f"[sim2sim] evaluating origin index {index}")
        run_policy(
            policy_path=args.policy,
            xml_path=args.xml_path,
            robot=args.robot,
            sim_dt=args.sim_dt,
            control_dt=args.control_dt,
            steps=args.steps,
            cmd_vx=args.cmd_vx,
            cmd_vy=args.cmd_vy,
            cmd_wz=args.cmd_wz,
            kp=args.kp,
            kd=args.kd,
            render=False,
            real_time=False,
            record_video=record_path,
            track_camera=args.track_camera,
            camera_distance=args.camera_distance,
            camera_elevation=args.camera_elevation,
            camera_azimuth=args.camera_azimuth,
            origins_path=str(args.origins_path),
            spawn_origin_index=index,
        )


if __name__ == "__main__":
    main()
