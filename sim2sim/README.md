# sim2sim

This directory contains simulator-to-simulator validation utilities.

Current scope:

- run an exported Isaac Sim policy inside MuJoCo
- keep policy inference separate from training code
- support reusable robot adapters instead of a one-off Go2 script
- follow an mjlab-style layout without transitional compatibility layers

Current first target:

- Isaac Sim Go2 velocity policy
- MuJoCo as the target simulator

## Layout

- `asset_zoo/robots/`: robot-facing adapter definitions, inspired by `mjlab.asset_zoo`
  includes both deployment adapters and MuJoCo-native asset constants
- `registry.py`: top-level robot registry
- `actuators/`: reusable actuator semantics, inspired by `mjlab.actuator`
- `backends/`: simulator backends
- `tools/`: command-style entry points for prepare/build/inspect/play/validate
- `common.py`: shared robot and command spec dataclasses
- `adapters.py`: robot-specific observation and action adapters
- `policies.py`: `.pt` / `.onnx` policy loaders

There are no longer duplicate top-level shims for play/inspect/build/prepare.
The canonical entry points now all live under `tools/`.

## Asset Config

The first mjlab-style asset config now lives in:

- `asset_zoo/robots/unitree_go2_asset.py`

It records:

- model paths for the current Go2 MuJoCo prototypes
- preferred playback XML
- default base pose and default joint pose
- foot collision geom names
- MuJoCo-native position actuator groups
- collision profile metadata

The deployment adapter in `asset_zoo/robots/unitree_go2.py` now consumes this
asset config instead of hardcoding base pose and joint ordering in multiple
places.

There is also a paired asset registry path now:

- `make_robot_adapter(robot_name)`
- `get_robot_asset_cfg(robot_name)`

This lets tools follow a single pipeline:

`robot name -> asset cfg -> build/inspect/play`

Current asset build helpers:

- `tools/prepare_urdf.py`
- `tools/build_mjcf.py`
- `tools/build_robot_assets.py`
- `tools/inspect_model.py`
- `tools/play_policy.py`
- `tools/validate_robot_policy.py`

So the first-pass Go2 asset pipeline can now be driven from the registered robot
name instead of repeating source paths by hand.

Current default for Go2 playback is now intended to be the more mjlab-like
variant:

- feet-only collision
- position actuator
- armature-enabled XML

## Go2 Baseline Assumptions

Current Go2 adapter is aligned to the Isaac Sim velocity policy with:

- root body name: `base`
- 12 controlled joints in this order:
  - `FL_hip_joint`
  - `FL_thigh_joint`
  - `FL_calf_joint`
  - `FR_hip_joint`
  - `FR_thigh_joint`
  - `FR_calf_joint`
  - `RL_hip_joint`
  - `RL_thigh_joint`
  - `RL_calf_joint`
  - `RR_hip_joint`
  - `RR_thigh_joint`
  - `RR_calf_joint`
- default joint pose:
  - hip `0.0`
  - thigh `0.8`
  - calf `-1.5`
- action scales:
  - hip `0.125`
  - thigh/calf `0.25`
- actor observation dimension: `45`
- control dt: `0.02`
- default PD gains for first playback:
  - `kp = 25.0`
  - `kd = 0.5`
- effort limit:
  - `23.5`

## Usage

Inspect a MuJoCo model before playback:

```bash
PYTHONPATH=/data2/sdam/robot_lab python sim2sim/tools/inspect_model.py \
  --xml-path /path/to/go2.xml \
  --robot unitree_go2_isaac_velocity
```

Run a TorchScript policy in MuJoCo:

```bash
PYTHONPATH=/data2/sdam/robot_lab python sim2sim/tools/play_policy.py \
  --policy /path/to/policy.pt \
  --xml-path /path/to/go2.xml \
  --robot unitree_go2_isaac_velocity \
  --render
```

## Extension Rule

For a new robot:

1. Add a robot module under `asset_zoo/robots/`.
2. Define a `Sim2SimRobotSpec`.
3. Add an adapter subclass only if observation logic differs.
4. Register it in `asset_zoo/robots/__init__.py`.

The registry, backend, and playback script should stay reusable.
