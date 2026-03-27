# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Batch-record the current Go2 policy zoo for side-by-side comparison."""

from __future__ import annotations

import argparse
from pathlib import Path

from sim2sim.tools.play_policy import run_policy


GO2_MODEL_ZOO = (
    (
        "go2_flat_full",
        "/data2/sdam/robot_lab/logs/rsl_rl/unitree_go2_flat/2026-03-19_14-14-07_go2_flat_full/exported/policy.onnx",
    ),
    (
        "go2_rough_full",
        "/data2/sdam/robot_lab/logs/rsl_rl/unitree_go2_rough/2026-03-19_16-19-39_go2_rough_full/exported/policy.onnx",
    ),
    (
        "go2_rough_illegal_contact_on",
        "/data2/sdam/robot_lab/logs/rsl_rl/unitree_go2_rough/2026-03-23_14-48-35_go2_rough_illegal_contact_on/exported/policy.onnx",
    ),
    (
        "go2_stairs_full",
        "/data2/sdam/robot_lab/logs/rsl_rl/unitree_go2_rough/2026-03-24_17-27-57_go2_stairs_full/exported/policy.onnx",
    ),
    (
        "go2_rough_armature_full",
        "/data2/sdam/robot_lab/logs/rsl_rl/unitree_go2_rough_armature/2026-03-26_10-44-03_go2_rough_armature_full/exported/policy.onnx",
    ),
)

DEFAULT_XML_PATH = "/data2/sdam/unitree_mujoco/unitree_robots/go2/scene.xml"
DEFAULT_OUTPUT_DIR = "/data2/sdam/robot_lab/sim2sim/videos/model_zoo"


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Batch-record the current Go2 model zoo in MuJoCo.")
    parser.add_argument("--robot", type=str, default="unitree_go2_unitree_mujoco", help="Registered sim2sim robot.")
    parser.add_argument("--xml-path", type=str, default=DEFAULT_XML_PATH, help="MuJoCo XML scene path.")
    parser.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR, help="Directory for recorded mp4 files.")
    parser.add_argument("--steps", type=int, default=500, help="Control steps to record per model.")
    parser.add_argument("--cmd-vx", type=float, default=0.5, help="Forward velocity command.")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="Lateral velocity command.")
    parser.add_argument("--cmd-wz", type=float, default=0.0, help="Yaw velocity command.")
    parser.add_argument("--kp", type=float, default=25.0, help="Uniform PD kp.")
    parser.add_argument("--kd", type=float, default=0.5, help="Uniform PD kd.")
    parser.add_argument("--track-camera", action="store_true", help="Track the root body during recording.")
    parser.add_argument("--camera-distance", type=float, default=3.0, help="Tracking camera distance.")
    parser.add_argument("--camera-elevation", type=float, default=-30, help="Tracking camera elevation.")
    parser.add_argument("--camera-azimuth", type=float, default=0.0, help="Tracking camera azimuth.")
    parser.add_argument("--video-width", type=int, default=640, help="Video width.")
    parser.add_argument("--video-height", type=int, default=480, help="Video height.")
    parser.add_argument("--video-fps", type=int, default=None, help="Video fps.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing videos.")
    return parser


def main() -> None:
    args = build_argparser().parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    for run_name, policy_path in GO2_MODEL_ZOO:
        output_path = output_dir / f"{run_name}.mp4"
        if output_path.exists() and not args.overwrite:
            print(f"[sim2sim] skip existing {output_path}")
            continue
        print(f"[sim2sim] recording {run_name} -> {output_path}")
        run_policy(
            policy_path=policy_path,
            xml_path=args.xml_path,
            robot=args.robot,
            steps=args.steps,
            cmd_vx=args.cmd_vx,
            cmd_vy=args.cmd_vy,
            cmd_wz=args.cmd_wz,
            kp=args.kp,
            kd=args.kd,
            record_video=str(output_path),
            video_width=args.video_width,
            video_height=args.video_height,
            video_fps=args.video_fps,
            track_camera=args.track_camera,
            camera_distance=args.camera_distance,
            camera_elevation=args.camera_elevation,
            camera_azimuth=args.camera_azimuth,
        )


if __name__ == "__main__":
    main()
