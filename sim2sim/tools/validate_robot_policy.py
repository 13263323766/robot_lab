# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Validate a robot policy through the sim2sim asset pipeline."""

from __future__ import annotations

import argparse
import json

from sim2sim import get_robot_asset_cfg
from sim2sim.tools.inspect_model import inspect_model
from sim2sim.tools.play_policy import run_policy


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare, inspect, and play a policy for a registered sim2sim robot.")
    parser.add_argument("--policy", type=str, required=True, help="Path to exported TorchScript/ONNX policy.")
    parser.add_argument(
        "--robot",
        type=str,
        default="unitree_go2_unitree_mujoco",
        help="Registered sim2sim robot name.",
    )
    parser.add_argument(
        "--xml-path",
        type=str,
        default=None,
        help="Optional ready-made MuJoCo XML. If omitted, the asset default is used.",
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
    parser.add_argument("--record-video", type=str, default=None, help="Optional mp4 output path for offscreen recording.")
    parser.add_argument("--video-width", type=int, default=640, help="Recorded video width.")
    parser.add_argument("--video-height", type=int, default=480, help="Recorded video height.")
    parser.add_argument("--video-fps", type=int, default=None, help="Recorded video fps. Defaults to 1/control_dt.")
    parser.add_argument("--track-camera", action="store_true", help="Track the robot root body during recording.")
    parser.add_argument("--camera-distance", type=float, default=2.6, help="Tracking camera distance.")
    parser.add_argument("--camera-elevation", type=float, default=-12.0, help="Tracking camera elevation in degrees.")
    parser.add_argument("--camera-azimuth", type=float, default=0.0, help="Tracking camera azimuth in degrees.")
    parser.add_argument(
        "--origins-path",
        type=str,
        default=None,
        help="Optional .npy terrain origins exported from Isaac play. Used with --spawn-origin-index.",
    )
    parser.add_argument(
        "--spawn-origin-index",
        type=int,
        default=None,
        help="Flattened terrain-origin index to spawn the robot at on an exported Isaac terrain.",
    )
    parser.add_argument(
        "--spawn-points-path",
        type=str,
        default=None,
        help="Optional .npy root spawn patches exported from Isaac play. Preferred over --origins-path.",
    )
    parser.add_argument(
        "--spawn-point-index",
        type=int,
        default=None,
        help="Flattened root-spawn patch index to use when --spawn-points-path is provided.",
    )
    parser.add_argument(
        "--spawn-z-offset",
        type=float,
        default=0.0,
        help="Additional z offset applied to the initial base position after scene/origin spawning.",
    )
    parser.add_argument(
        "--spawn-clearance",
        type=float,
        default=0.05,
        help="Default base clearance above the local terrain height.",
    )
    return parser


def main() -> None:
    args = build_argparser().parse_args()
    asset_cfg = get_robot_asset_cfg(args.robot)

    xml_path = args.xml_path or asset_cfg.paths.preferred_playback_xml

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
        record_video=args.record_video,
        video_width=args.video_width,
        video_height=args.video_height,
        video_fps=args.video_fps,
        track_camera=args.track_camera,
        camera_distance=args.camera_distance,
        camera_elevation=args.camera_elevation,
        camera_azimuth=args.camera_azimuth,
        origins_path=args.origins_path,
        spawn_origin_index=args.spawn_origin_index,
        spawn_points_path=args.spawn_points_path,
        spawn_point_index=args.spawn_point_index,
        spawn_z_offset=args.spawn_z_offset,
        spawn_clearance=args.spawn_clearance,
    )


if __name__ == "__main__":
    main()
