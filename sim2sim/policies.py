# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

try:
    import onnxruntime as ort
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    ort = None


class PolicyBase:
    def __call__(self, observation: np.ndarray) -> np.ndarray:  # pragma: no cover - interface only
        raise NotImplementedError


class TorchScriptPolicy(PolicyBase):
    """Lightweight policy wrapper for exported Isaac Sim TorchScript actors."""

    def __init__(self, policy_path: str):
        self.policy_path = Path(policy_path).expanduser().resolve()
        self.module = torch.jit.load(str(self.policy_path), map_location="cpu")
        self.module.eval()

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        obs_tensor = torch.as_tensor(observation, dtype=torch.float32)
        if obs_tensor.ndim == 1:
            obs_tensor = obs_tensor.unsqueeze(0)
        with torch.inference_mode():
            action = self.module(obs_tensor)
        return action.squeeze(0).cpu().numpy().astype(np.float32)


class OnnxPolicy(PolicyBase):
    """ONNX Runtime wrapper for exported Isaac Sim actor policies."""

    def __init__(self, policy_path: str):
        if ort is None:
            raise ModuleNotFoundError(
                "onnxruntime is not installed in the current environment. Install it to run ONNX policies."
            )
        self.policy_path = Path(policy_path).expanduser().resolve()
        self.session = ort.InferenceSession(str(self.policy_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32)
        if obs.ndim == 1:
            obs = obs[None, :]
        action = self.session.run([self.output_name], {self.input_name: obs})[0]
        return np.asarray(action, dtype=np.float32).squeeze(0)


def load_policy(policy_path: str) -> PolicyBase:
    path = Path(policy_path).expanduser().resolve()
    suffix = path.suffix.lower()
    if suffix == ".pt":
        return TorchScriptPolicy(str(path))
    if suffix == ".onnx":
        return OnnxPolicy(str(path))
    raise ValueError(f"Unsupported policy format '{suffix}' for {path}. Expected .pt or .onnx")
