# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""MuJoCo backend implementation."""

from __future__ import annotations

from pathlib import Path

import numpy as np

try:
    import mujoco
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    mujoco = None


def _require_mujoco() -> None:
    if mujoco is None:
        raise ModuleNotFoundError(
            "MuJoCo is not installed in the current environment. Install `mujoco` before using sim2sim playback."
        )


def quat_wxyz_to_rotmat(quat: np.ndarray) -> np.ndarray:
    """Convert MuJoCo-style quaternion [w, x, y, z] to a 3x3 rotation matrix."""
    w, x, y, z = quat
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class MujocoRobotInterface:
    """Minimal MuJoCo robot interface for policy playback and sim2sim checks."""

    def __init__(
        self,
        model_path: str,
        root_body_name: str,
        joint_names: tuple[str, ...],
        actuator_names: tuple[str, ...],
        actuator_name_candidates: tuple[tuple[str, ...], ...] | None,
        sim_dt: float,
        control_dt: float,
    ):
        _require_mujoco()

        fullpath = Path(model_path).expanduser().resolve()
        if not fullpath.exists():
            raise FileNotFoundError(f"MuJoCo model file does not exist: {fullpath}")

        self.model = mujoco.MjModel.from_xml_path(str(fullpath))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = sim_dt

        self.root_body_name = root_body_name
        self.root_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, root_body_name)
        self.joint_names = joint_names
        self.actuator_names = actuator_names
        self.control_dt = control_dt
        self.has_floating_base = self.model.nq >= len(joint_names) + 7 and self.model.nv >= len(joint_names) + 6

        if round(control_dt / sim_dt) != control_dt / sim_dt:
            raise ValueError("control_dt must be an integer multiple of sim_dt")
        self.frame_skip = int(round(control_dt / sim_dt))

        self.joint_qpos_ids = []
        self.joint_qvel_ids = []
        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            self.joint_qpos_ids.append(self.model.jnt_qposadr[joint_id])
            self.joint_qvel_ids.append(self.model.jnt_dofadr[joint_id])
        self.joint_qpos_ids = np.asarray(self.joint_qpos_ids, dtype=np.int32)
        self.joint_qvel_ids = np.asarray(self.joint_qvel_ids, dtype=np.int32)

        self.control_mode = "actuator_ctrl"
        self.actuator_command_mode = "torque"
        self.actuator_ids = np.asarray([], dtype=np.int32)
        self.resolved_actuator_names: list[str] = []
        if self.model.nu > 0:
            self.actuator_ids = []
            available_actuators = {
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.model.nu)
            }
            actuator_name_candidates = actuator_name_candidates or tuple(() for _ in actuator_names)
            for index, actuator_name in enumerate(actuator_names):
                candidates = (actuator_name,) + tuple(actuator_name_candidates[index])
                resolved_name = None
                for candidate in candidates:
                    if candidate in available_actuators:
                        resolved_name = candidate
                        break
                if resolved_name is None:
                    known = ", ".join(sorted(available_actuators))
                    tried = ", ".join(candidates)
                    raise KeyError(
                        f"Unable to resolve actuator for index {index}. Tried: {tried}. Available: {known}"
                    )
                self.actuator_ids.append(available_actuators[resolved_name])
                self.resolved_actuator_names.append(resolved_name)
            self.actuator_ids = np.asarray(self.actuator_ids, dtype=np.int32)
            if np.all(self.model.actuator_biastype[self.actuator_ids] == mujoco.mjtBias.mjBIAS_AFFINE):
                self.actuator_command_mode = "position_target"
        else:
            self.control_mode = "joint_torque"
            self.resolved_actuator_names = list(joint_names)

        self.kp = np.zeros(len(actuator_names), dtype=np.float64)
        self.kd = np.zeros(len(actuator_names), dtype=np.float64)
        self.initial_qpos = self.data.qpos.copy()
        self.initial_qvel = self.data.qvel.copy()

    @property
    def action_dim(self) -> int:
        return len(self.joint_names) if self.control_mode == "joint_torque" else len(self.actuator_ids)

    def reset(self, initial_base_pos: np.ndarray, initial_base_quat_wxyz: np.ndarray, default_joint_pos: np.ndarray) -> None:
        mujoco.mj_resetData(self.model, self.data)
        np.copyto(self.data.qpos, self.initial_qpos)
        np.copyto(self.data.qvel, self.initial_qvel)

        if self.has_floating_base:
            self.data.qpos[:3] = initial_base_pos
            self.data.qpos[3:7] = initial_base_quat_wxyz

        self.data.qpos[self.joint_qpos_ids] = default_joint_pos
        mujoco.mj_forward(self.model, self.data)

    def query_ground_height(self, x: float, y: float, z_start: float = 5.0) -> float:
        geomgroup = np.ones(6, dtype=np.uint8)
        geomid = np.asarray([-1], dtype=np.int32)
        point = np.asarray([x, y, z_start], dtype=np.float64)
        direction = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
        distance = mujoco.mj_ray(
            self.model,
            self.data,
            point,
            direction,
            geomgroup,
            1,
            self.root_body_id,
            geomid,
        )
        if not np.isfinite(distance) or distance < 0.0:
            raise RuntimeError(f"Failed to query ground height at x={x}, y={y}, z_start={z_start}")
        return float(z_start - distance)

    def set_pd_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        if kp.shape != (self.action_dim,) or kd.shape != (self.action_dim,):
            raise ValueError("PD gain shapes must match action dimension")
        self.kp = kp.astype(np.float64).copy()
        self.kd = kd.astype(np.float64).copy()

    def get_joint_positions(self) -> np.ndarray:
        return self.data.qpos[self.joint_qpos_ids].astype(np.float32).copy()

    def get_joint_velocities(self) -> np.ndarray:
        return self.data.qvel[self.joint_qvel_ids].astype(np.float32).copy()

    def get_base_quat_wxyz(self) -> np.ndarray:
        return np.asarray(self.data.xquat[self.root_body_id], dtype=np.float32).copy()

    def get_projected_gravity(self) -> np.ndarray:
        rotmat = quat_wxyz_to_rotmat(self.get_base_quat_wxyz().astype(np.float64))
        gravity_world = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
        return (rotmat.T @ gravity_world).astype(np.float32)

    def get_base_ang_vel_body(self) -> np.ndarray:
        velocity = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, self.root_body_id, velocity, 1
        )
        return velocity[:3].astype(np.float32)

    def compute_pd_torque(self, target_pos: np.ndarray, target_vel: np.ndarray | None = None) -> np.ndarray:
        if target_vel is None:
            target_vel = np.zeros(self.action_dim, dtype=np.float64)
        current_pos = self.get_joint_positions().astype(np.float64)
        current_vel = self.get_joint_velocities().astype(np.float64)
        pos_err = target_pos.astype(np.float64) - current_pos
        vel_err = target_vel.astype(np.float64) - current_vel
        return self.kp * pos_err + self.kd * vel_err

    def set_motor_torque(self, torque: np.ndarray) -> None:
        if torque.shape != (self.action_dim,):
            raise ValueError("Torque shape must match action dimension")
        if self.control_mode == "joint_torque":
            self.data.qfrc_applied[:] = 0.0
            self.data.qfrc_applied[self.joint_qvel_ids] = torque.astype(np.float64)
        else:
            self.data.ctrl[:] = torque.astype(np.float64)

    def set_position_target(self, target: np.ndarray) -> None:
        if target.shape != (self.action_dim,):
            raise ValueError("Target shape must match action dimension")
        if self.control_mode != "actuator_ctrl" or self.actuator_command_mode != "position_target":
            raise RuntimeError("Position targets are only valid for actuator_ctrl models using position actuators.")
        self.data.ctrl[:] = target.astype(np.float64)

    def step(self, n_frames: int | None = None) -> None:
        mujoco.mj_step(self.model, self.data, nstep=self.frame_skip if n_frames is None else n_frames)

    def render(self):
        _require_mujoco()
        import mujoco.viewer

        return mujoco.viewer.launch_passive(self.model, self.data)

    def describe(self) -> dict[str, object]:
        available_joints = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)
        ]
        available_actuators = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(self.model.nu)
        ]
        return {
            "root_body_name": self.root_body_name,
            "control_mode": self.control_mode,
            "actuator_command_mode": self.actuator_command_mode,
            "has_floating_base": self.has_floating_base,
            "joint_names": list(self.joint_names),
            "resolved_actuator_names": list(self.resolved_actuator_names),
            "available_joints": available_joints,
            "available_actuators": available_actuators,
        }


__all__ = ["MujocoRobotInterface"]
