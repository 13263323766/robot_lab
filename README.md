# robot_lab: `exp/g1-sim2sim`

This README is branch-specific.

## Branch Purpose

This branch starts from the completed Go2 sim2sim baseline and moves the same workflow onto humanoid robots.

Current priority:

- keep the Go2 work as a finished quadruped reference
- start a new humanoid transfer line on `Unitree G1`
- reuse the existing `sim2sim/` framework instead of building a second one

## Current Status

The Go2 line is intentionally treated as complete enough for now:

- Isaac-side Go2 training was aligned toward `unitree_mujoco`
- MuJoCo-side Go2 playback, recording, and heading-command alignment were completed
- the main sim2sim issues on joint order, action semantics, and command semantics were already resolved

That Go2 work remains in this branch as the validated baseline, but the active next phase is now `G1`.

## What Is Active Now

This branch is currently focused on:

- `Unitree G1` locomotion training in Isaac Lab
- bringing `G1` into the existing `sim2sim/` adapter framework
- preparing the first G1 Isaac-to-MuJoCo validation path

Current G1-side progress already in this branch:

- G1 MuJoCo robot registration has been added to [sim2sim](/data2/sdam/robot_lab/sim2sim)
- the official `unitree_mujoco` G1 29-DoF model is the target asset
- G1 joint ordering between training and MuJoCo has been checked and aligned
- G1 actor observation layout has been mapped into the sim2sim adapter path
- G1 uses the same heading-command semantics as Go2, so the existing `heading-hold` alignment path is expected to remain relevant

## Training Direction

Current source-side focus is:

- `RobotLab-Isaac-Velocity-Rough-Unitree-G1-v0`

Recent training-side adjustments on this branch:

- added a posture termination using `bad_orientation`
- re-enabled command curriculum on the G1 rough task

The current intent is to first make the G1 locomotion task healthier in Isaac, then validate transfer through MuJoCo.

## Sim2Sim Direction

The reusable framework remains under:

- [sim2sim](/data2/sdam/robot_lab/sim2sim)

That framework was first proven on Go2 and is now being reused for G1 rather than rewritten.

For the current simulator target, we still use:

- `unitree_mujoco`

## Go2 Baseline

The completed Go2 reference still lives in this branch as the previous stage result:

- Go2 MuJoCo playback examples
- heading-hold comparison videos
- Go2 terrain scene builders
- documented Go2 transfer findings

Those artifacts are retained as the quadruped baseline, but they are no longer the main active expansion target on this branch.

## Where To Look

- G1 training config:
  - [unitree_g1 rough env](/data2/sdam/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/humanoid/unitree_g1/rough_env_cfg.py)
- G1 PPO config:
  - [unitree_g1 rsl_rl cfg](/data2/sdam/robot_lab/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/humanoid/unitree_g1/agents/rsl_rl_ppo_cfg.py)
- sim2sim framework:
  - [sim2sim/README.md](/data2/sdam/robot_lab/sim2sim/README.md)

## Current Meaning

This branch is no longer “Go2-only sim2sim documentation”.

It is now:

- a finished Go2 quadruped baseline
- plus the active first humanoid migration line on `Unitree G1`
