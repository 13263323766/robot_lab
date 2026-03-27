# robot_lab: `exp/go2-sim2sim`

This README is branch-specific.

This branch is no longer just a planning branch. It now contains a working first-pass sim2sim path for Go2.

## Branch Purpose

The goal of `exp/go2-sim2sim` is:

- keep training reproduction work on `exp/go2-train`
- build a dedicated simulator-to-simulator validation path here
- make the trained Go2 policy runnable inside MuJoCo
- document the current transfer state, not just ideas

## What We Built

We built our own `sim2sim/` framework inside this repository.

This framework is:

- focused on policy playback and validation, not MuJoCo-side training
- separated from the Isaac training code
- organized around reusable robot adapters and simulator backends
- narrowed to a practical current target:
  - `Go2`
  - `MuJoCo`
  - exported Isaac policy playback

The design direction is intentionally inspired by `mjlab`:

- `asset_zoo/robots/`
- `actuators/`
- `backends/`
- `tools/`

But the implementation here is our own branch-local framework, built for our current Go2 sim2sim work.

## Current Reference

Training-side baseline and reproduction history remain on:

- `exp/go2-train`

This branch starts after training feasibility was already verified there.

## Current Progress

We are currently at this stage:

1. A dedicated MuJoCo sim2sim framework has been built inside [sim2sim](/data2/sdam/robot_lab/sim2sim).
2. The active target model has been switched to the official `unitree_mujoco` Go2 asset.
3. The Go2 playback adapter has been aligned to the Isaac training interface on:
   - joint order
   - action semantics
   - default joint pose
   - control period
   - DCMotor-style torque clipping
4. A transfer-oriented Go2 rough training variant with `armature` has been added on the Isaac side.
5. The exported Go2 rough+armature policy can now be played and recorded in MuJoCo through the current `sim2sim` path.

So the branch is no longer at the “design checklist” stage. It has already reached:

- `Isaac training -> exported policy -> MuJoCo playback -> recorded artifact`

## Current Status Summary

Current validated path:

- source policy:
  - `logs/rsl_rl/unitree_go2_rough_armature/2026-03-26_10-44-03_go2_rough_armature_full/exported/policy.onnx`
- target scene:
  - `/data2/sdam/unitree_mujoco/unitree_robots/go2/scene.xml`
- current sim2sim README:
  - [sim2sim/README.md](/data2/sdam/robot_lab/sim2sim/README.md)
- current playback preview:
  - [![go2_unitree_mujoco_rough_armature_track](sim2sim/videos/go2_unitree_mujoco_rough_armature_track.gif)](sim2sim/videos/go2_unitree_mujoco_rough_armature_track.mp4)

## What This Branch Contains

This branch should accumulate:

- sim2sim code
- transfer-oriented environment/config work
- MuJoCo target-side validation
- playback artifacts
- migration conclusions

This branch should not become a duplicate of the full training reproduction branch.

## Immediate Meaning Of The Current Result

The current result means:

- we now have a working MuJoCo-side policy validation path
- we are using the official `unitree_mujoco` Go2 model instead of the earlier self-converted prototype model
- we have already identified and corrected several critical transfer mismatches:
  - wrong joint/action order
  - mismatched DCMotor clipping values
  - unsuitable initial base height

The next phase is no longer “can we build a framework”.

The next phase is:

- continue improving controller/model alignment
- compare behavior quality between Isaac playback and MuJoCo playback
- decide what further transfer-oriented training changes are still necessary

## Where To Look Next

For the concrete current sim2sim workflow, use:

- [sim2sim/README.md](/data2/sdam/robot_lab/sim2sim/README.md)

For earlier training runs and baseline behavior, use:

- `exp/go2-train`
