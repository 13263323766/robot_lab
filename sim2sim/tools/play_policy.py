# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Run an exported Isaac Sim policy inside MuJoCo."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from sim2sim import Sim2SimCommand, make_robot_adapter
from sim2sim.backends import MujocoRobotInterface
from sim2sim.policies import load_policy


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Play an exported Isaac Sim policy inside MuJoCo.")
    parser.add_argument("--policy", type=str, required=True, help="Path to exported TorchScript actor policy.")
    parser.add_argument("--xml-path", type=str, required=True, help="Path to MuJoCo XML model.")
    parser.add_argument(
        "--robot",
        type=str,
        default="unitree_go2_unitree_mujoco",
        help="Registered sim2sim robot adapter name.",
    )
    parser.add_argument("--sim-dt", type=float, default=0.005, help="MuJoCo simulation timestep.")
    parser.add_argument("--control-dt", type=float, default=None, help="Policy/control timestep override.")
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
    parser.add_argument("--camera-elevation", type=float, default=-30, help="Tracking camera elevation in degrees.")
    parser.add_argument("--camera-azimuth", type=float, default=0.0, help="Tracking camera azimuth in degrees.")
    return parser


def run_policy(
    *,
    policy_path: str,
    xml_path: str,
    robot: str,
    sim_dt: float = 0.005,
    control_dt: float | None = None,
    steps: int = 2000,
    cmd_vx: float = 0.5,
    cmd_vy: float = 0.0,
    cmd_wz: float = 0.0,
    kp: float = 25.0,
    kd: float = 0.5,
    render: bool = False,
    real_time: bool = False,
    record_video: str | None = None,
    video_width: int = 640,
    video_height: int = 480,
    video_fps: int | None = None,
    track_camera: bool = False,
    camera_distance: float = 2.6,
    camera_elevation: float = -12.0,
    camera_azimuth: float = 0.0,
) -> None:
    import imageio.v2 as imageio
    import mujoco

    adapter = make_robot_adapter(robot)
    control_dt = control_dt if control_dt is not None else adapter.spec.control_dt

    policy = load_policy(policy_path)
    interface = MujocoRobotInterface(
        model_path=xml_path,
        root_body_name=adapter.spec.root_body_name,
        joint_names=adapter.spec.joint_names,
        actuator_names=adapter.spec.actuator_names,
        actuator_name_candidates=adapter.spec.actuator_name_candidates,
        sim_dt=sim_dt,
        control_dt=control_dt,
    )
    kp_gains, kd_gains = adapter.spec.make_uniform_gains(kp, kd)
    interface.set_pd_gains(kp_gains, kd_gains)
    interface.reset(
        initial_base_pos=adapter.spec.initial_base_pos,
        initial_base_quat_wxyz=adapter.spec.initial_base_quat_wxyz,
        default_joint_pos=adapter.spec.default_joint_pos,
    )

    command = Sim2SimCommand(cmd_vx, cmd_vy, cmd_wz)
    last_action = adapter.spec.zero_action()
    viewer = interface.render() if render else None
    renderer = None
    video_writer = None
    video_camera = None
    if record_video is not None:
        record_path = Path(record_video).expanduser().resolve()
        record_path.parent.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(interface.model, height=video_height, width=video_width)
        fps = video_fps if video_fps is not None else max(1, int(round(1.0 / control_dt)))
        video_writer = imageio.get_writer(record_path, fps=fps)
        if track_camera:
            video_camera = mujoco.MjvCamera()
            video_camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            video_camera.trackbodyid = interface.root_body_id
            video_camera.distance = camera_distance
            video_camera.elevation = camera_elevation
            video_camera.azimuth = camera_azimuth

    print(f"[sim2sim] robot={adapter.spec.name}")
    print(f"[sim2sim] obs_dim={adapter.spec.actor_obs_dim}, act_dim={interface.action_dim}")
    print(f"[sim2sim] sim_dt={sim_dt}, control_dt={control_dt}, frame_skip={interface.frame_skip}")
    print(f"[sim2sim] command={command.as_array().tolist()}")
    print(f"[sim2sim] actuators={interface.resolved_actuator_names}")
    print(f"[sim2sim] actuator_command_mode={interface.actuator_command_mode}")
    if record_video is not None:
        print(f"[sim2sim] recording={str(record_path)}")

    try:
        for _ in range(steps):
            start = time.time()
            obs = adapter.build_actor_observation(interface, command, last_action)
            action = policy(obs)
            target_pos = adapter.action_to_target_pos(action)
            if interface.control_mode == "actuator_ctrl" and interface.actuator_command_mode == "position_target":
                interface.set_position_target(target_pos)
            else:
                torque = adapter.clip_torque(interface.compute_pd_torque(target_pos), interface.get_joint_velocities())
                interface.set_motor_torque(torque)
            interface.step()
            last_action = action
            if viewer is not None:
                viewer.sync()
            if video_writer is not None and renderer is not None:
                renderer.update_scene(interface.data, camera=video_camera if video_camera is not None else -1)
                video_writer.append_data(renderer.render())
            if real_time:
                elapsed = time.time() - start
                time.sleep(max(0.0, control_dt - elapsed))
    finally:
        if video_writer is not None:
            video_writer.close()
        if renderer is not None:
            renderer.close()


def main() -> None:
    args = build_argparser().parse_args()
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
    )


__all__ = ["run_policy", "main"]


if __name__ == "__main__":
    main()
