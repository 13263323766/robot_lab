# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Run an exported Isaac Sim policy inside MuJoCo."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from sim2sim import Sim2SimCommand, make_robot_adapter
from sim2sim.backends import MujocoRobotInterface
from sim2sim.policies import load_policy


def _infer_spawn_points_path(origins_path: str | None, spawn_points_path: str | None) -> str | None:
    if spawn_points_path is not None or origins_path is None:
        return spawn_points_path
    origins = Path(origins_path).expanduser().resolve()
    if origins.name.endswith("_origins.npy"):
        candidate = origins.with_name(origins.name.replace("_origins.npy", "_root_spawn.npy"))
        if candidate.exists():
            return str(candidate)
    return None


def _cell_center(row: int, col: int, num_rows: int, num_cols: int, size_x: float, size_y: float) -> np.ndarray:
    total_x = num_rows * size_x
    total_y = num_cols * size_y
    cx = (row + 0.5) * size_x - 0.5 * total_x
    cy = (col + 0.5) * size_y - 0.5 * total_y
    return np.asarray([cx, cy, 0.0], dtype=np.float32)


def _quat_wxyz_to_yaw(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = [float(v) for v in quat_wxyz]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def _wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


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
    parser.add_argument("--kp", type=float, default=None, help="Optional uniform proportional gain override.")
    parser.add_argument("--kd", type=float, default=None, help="Optional uniform derivative gain override.")
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
        default=0.10,
        help="Default base clearance above the local terrain height.",
    )
    parser.add_argument("--spawn-row", type=int, default=None, help="Spawn the robot at a specific tiled terrain row.")
    parser.add_argument("--spawn-col", type=int, default=None, help="Spawn the robot at a specific tiled terrain col.")
    parser.add_argument("--terrain-num-rows", type=int, default=5, help="Number of terrain rows for tiled scene spawn.")
    parser.add_argument("--terrain-num-cols", type=int, default=5, help="Number of terrain cols for tiled scene spawn.")
    parser.add_argument("--terrain-size-x", type=float, default=8.0, help="Per-cell terrain size in x.")
    parser.add_argument("--terrain-size-y", type=float, default=8.0, help="Per-cell terrain size in y.")
    parser.add_argument(
        "--heading-hold",
        action="store_true",
        help="Emulate Isaac heading_command=True by converting heading error into cmd_wz online.",
    )
    parser.add_argument(
        "--heading-target",
        type=float,
        default=None,
        help="Optional world-frame yaw target in radians for --heading-hold. Defaults to the initial base yaw.",
    )
    parser.add_argument(
        "--heading-kp",
        type=float,
        default=0.5,
        help="Heading proportional gain used by --heading-hold.",
    )
    parser.add_argument(
        "--heading-max-wz",
        type=float,
        default=1.0,
        help="Absolute cmd_wz clip used by --heading-hold.",
    )
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
    kp: float | None = None,
    kd: float | None = None,
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
    origins_path: str | None = None,
    spawn_origin_index: int | None = None,
    spawn_points_path: str | None = None,
    spawn_point_index: int | None = None,
    spawn_z_offset: float = 0.0,
    spawn_clearance: float = 0.10,
    spawn_row: int | None = None,
    spawn_col: int | None = None,
    terrain_num_rows: int = 5,
    terrain_num_cols: int = 5,
    terrain_size_x: float = 8.0,
    terrain_size_y: float = 8.0,
    heading_hold: bool = False,
    heading_target: float | None = None,
    heading_kp: float = 0.5,
    heading_max_wz: float = 1.0,
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
    kp_gains, kd_gains = adapter.spec.resolve_gains(kp, kd)
    interface.set_pd_gains(kp_gains, kd_gains)

    initial_base_pos = adapter.spec.initial_base_pos.copy()
    if (spawn_row is None) != (spawn_col is None):
        raise ValueError("--spawn-row and --spawn-col must be provided together.")
    if spawn_row is not None and spawn_col is not None:
        if not (0 <= spawn_row < terrain_num_rows and 0 <= spawn_col < terrain_num_cols):
            raise IndexError(
                f"spawn cell ({spawn_row}, {spawn_col}) out of bounds for terrain grid "
                f"{terrain_num_rows}x{terrain_num_cols}"
            )
        initial_base_pos = initial_base_pos + _cell_center(
            spawn_row, spawn_col, terrain_num_rows, terrain_num_cols, terrain_size_x, terrain_size_y
        )
    resolved_spawn_points_path = _infer_spawn_points_path(origins_path, spawn_points_path)
    if resolved_spawn_points_path is not None:
        spawn_points = np.load(Path(resolved_spawn_points_path).expanduser().resolve())
        flat_spawn_points = spawn_points.reshape(-1, 3)
        if spawn_point_index is None:
            if spawn_origin_index is not None:
                spawn_point_index = spawn_origin_index
            else:
                raise ValueError("--spawn-point-index must be provided when root spawn points are used.")
        if spawn_point_index < 0 or spawn_point_index >= len(flat_spawn_points):
            raise IndexError(
                f"spawn point index {spawn_point_index} out of bounds for {len(flat_spawn_points)} root spawn points"
            )
        initial_base_pos = initial_base_pos + flat_spawn_points[spawn_point_index].astype(np.float32)
    elif origins_path is not None:
        origins = np.load(Path(origins_path).expanduser().resolve())
        flat_origins = origins.reshape(-1, 3)
        if spawn_origin_index is None:
            raise ValueError("--spawn-origin-index must be provided when --origins-path is used.")
        if spawn_origin_index < 0 or spawn_origin_index >= len(flat_origins):
            raise IndexError(
                f"spawn origin index {spawn_origin_index} out of bounds for {len(flat_origins)} terrain origins"
            )
        initial_base_pos = initial_base_pos + flat_origins[spawn_origin_index].astype(np.float32)

    base_height_reference = float(initial_base_pos[2])
    ground_height = interface.query_ground_height(
        float(initial_base_pos[0]),
        float(initial_base_pos[1]),
        z_start=max(base_height_reference + 5.0, 5.0),
    )
    initial_base_pos[2] = np.float32(ground_height + base_height_reference + spawn_clearance)

    if spawn_z_offset != 0.0:
        initial_base_pos[2] += np.float32(spawn_z_offset)

    interface.reset(
        initial_base_pos=initial_base_pos,
        initial_base_quat_wxyz=adapter.spec.initial_base_quat_wxyz,
        default_joint_pos=adapter.spec.default_joint_pos,
    )

    initial_yaw = _quat_wxyz_to_yaw(interface.get_base_quat_wxyz())
    resolved_heading_target = initial_yaw if heading_target is None else float(heading_target)
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
    print(f"[sim2sim] initial_base_pos={initial_base_pos.tolist()}")
    print(
        f"[sim2sim] ground_height={ground_height:.4f}, base_height_reference={base_height_reference:.4f}, "
        f"spawn_clearance={spawn_clearance:.4f}"
    )
    if spawn_z_offset != 0.0:
        print(f"[sim2sim] spawn_z_offset={spawn_z_offset}")
    if spawn_row is not None and spawn_col is not None:
        print(
            f"[sim2sim] spawn_cell=({spawn_row}, {spawn_col}) "
            f"terrain_grid={terrain_num_rows}x{terrain_num_cols} cell_size=({terrain_size_x}, {terrain_size_y})"
        )
    if resolved_spawn_points_path is not None:
        print(f"[sim2sim] spawn_points_path={resolved_spawn_points_path}, spawn_point_index={spawn_point_index}")
    elif origins_path is not None:
        print(f"[sim2sim] origins_path={origins_path}, spawn_origin_index={spawn_origin_index}")
    if record_video is not None:
        print(f"[sim2sim] recording={str(record_path)}")
    if heading_hold:
        print(
            f"[sim2sim] heading_hold=true target_yaw={resolved_heading_target:.4f} "
            f"heading_kp={heading_kp:.3f} heading_max_wz={heading_max_wz:.3f}"
        )

    try:
        for _ in range(steps):
            start = time.time()
            if heading_hold:
                current_yaw = _quat_wxyz_to_yaw(interface.get_base_quat_wxyz())
                heading_error = _wrap_to_pi(resolved_heading_target - current_yaw)
                commanded_wz = float(np.clip(heading_kp * heading_error, -heading_max_wz, heading_max_wz))
                command = Sim2SimCommand(cmd_vx, cmd_vy, commanded_wz)
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
        origins_path=args.origins_path,
        spawn_origin_index=args.spawn_origin_index,
        spawn_points_path=args.spawn_points_path,
        spawn_point_index=args.spawn_point_index,
        spawn_z_offset=args.spawn_z_offset,
        spawn_clearance=args.spawn_clearance,
        spawn_row=args.spawn_row,
        spawn_col=args.spawn_col,
        terrain_num_rows=args.terrain_num_rows,
        terrain_num_cols=args.terrain_num_cols,
        terrain_size_x=args.terrain_size_x,
        terrain_size_y=args.terrain_size_y,
        heading_hold=args.heading_hold,
        heading_target=args.heading_target,
        heading_kp=args.heading_kp,
        heading_max_wz=args.heading_max_wz,
    )


__all__ = ["run_policy", "main"]


if __name__ == "__main__":
    main()
