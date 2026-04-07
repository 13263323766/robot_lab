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

1. A dedicated MuJoCo sim2sim framework has been built inside [sim2sim](sim2sim).
2. The active target model has been switched to the official `unitree_mujoco` Go2 asset.
3. The Go2 playback adapter has been aligned to the Isaac training interface on:
   - joint order
   - action semantics
   - default joint pose
   - control period
   - DCMotor-style torque clipping
4. A transfer-oriented Go2 rough training variant with `armature` has been added on the Isaac side.
5. A second target-aligned Go2 rough task has been added to better match `unitree_mujoco` joint dynamics.
6. The exported Go2 rough+armature policy can now be played and recorded in MuJoCo through the current `sim2sim` path.
7. The latest target-aligned Go2 policy has also been recorded in MuJoCo and is now the main current reference artifact.

So the branch is no longer at the “design checklist” stage. It has already reached:

- `Isaac training -> exported policy -> MuJoCo playback -> recorded artifact`

## Current Status Summary

Current validated path:

- source policy:
  - `logs/rsl_rl/unitree_go2_rough_target/2026-03-27_17-00-12_go2_rough_target_4096_50k/exported/policy.onnx`
- target scene:
  - `/data2/sdam/unitree_mujoco/unitree_robots/go2/scene.xml`
- current sim2sim README:
  - [sim2sim/README.md](sim2sim/README.md)
- current playback preview:
  - [![go2_unitree_mujoco_rough_target_track](sim2sim/videos/go2_unitree_mujoco_rough_target_track.gif)](sim2sim/videos/go2_unitree_mujoco_rough_target_track.mp4)
- current flat-ground heading-command comparison:
  - no heading hold:
    - [![go2_flat_no_heading_hold](sim2sim/videos/go2_flat_no_heading_hold.gif)](sim2sim/videos/go2_flat_no_heading_hold.mp4)
  - with heading hold:
    - [![go2_flat_with_heading_hold](sim2sim/videos/go2_flat_with_heading_hold.gif)](sim2sim/videos/go2_flat_with_heading_hold.mp4)

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

One concrete alignment issue identified in this branch is command semantics:

- Isaac-side locomotion playback uses `heading_command=True`
- so the third command component is not simply a fixed user-provided yaw-rate at all times
- a `heading-hold` option has now been added to the MuJoCo sim2sim path to emulate this behavior
- on flat ground, this noticeably improves straight-line walking compared with the earlier fixed-`cmd_wz=0` playback

Current interpretation of the stair-climbing limitation:

- the issue is not explained by sim2sim mismatch alone
- in Isaac-side playback, the current Go2 policy can also climb lower stairs but fail on higher ones
- this means the current policy already has a source-side stair-height capability boundary
- the current MuJoCo stair-height difference appears to stay within the Isaac training stair-height range, so height alone is unlikely to explain the whole gap
- stair tread width / step spacing is now considered another plausible factor and should be treated as one candidate cause in the next validation round
- the MuJoCo result should therefore be interpreted as:
  - an existing policy capability limit
  - further exposed by target-side geometry, contact, and dynamics differences

## Current Training Variants

The branch now contains two transfer-oriented Go2 rough variants on the Isaac side:

- `RobotLab-Isaac-Velocity-Rough-Unitree-Go2-Armature-v0`
  - adds reflected actuator armature for sim2sim-oriented training
- `RobotLab-Isaac-Velocity-Rough-Unitree-Go2-Target-v0`
  - keeps the same rough curriculum/task structure
  - aligns robot-side joint dynamics more closely to `unitree_mujoco`
  - currently matches the target side on:
    - `armature = 0.01`
    - `damping = 0.1`
    - actuator friction term used as an approximation of `frictionloss = 0.2`

This means the current branch is no longer only about playback. It now also contains a dedicated training path that explicitly moves the Isaac-side Go2 dynamics toward the MuJoCo target model.

The branch also contains a stairs-heavy target-aligned variant:

- `RobotLab-Isaac-Velocity-Rough-Unitree-Go2-Target-StairsHeavy-v0`

This keeps the rough terrain curriculum active, but changes the terrain proportions so that stairs dominate the source training distribution:

- `pyramid_stairs = 0.35`
- `pyramid_stairs_inv = 0.35`
- the remaining `0.30` is split across boxes, random rough, and slopes

This task exists because the current question is no longer only “can the policy transfer”, but also “does the source policy itself have enough stair capability before transfer”.

## Current Terrain-Reuse Workflow

The branch now also supports a terrain-reuse workflow for tighter source-to-target comparisons:

1. Export the exact terrain mesh used during Isaac-side `play`
2. Build a MuJoCo scene from that exported terrain mesh
3. Reuse the exported `terrain_origins`
4. Evaluate one robot at one origin at a time on the same terrain

This is intentionally different from trying to make MuJoCo immediately run many robots in parallel.

At the current stage, the preferred workflow is:

- keep MuJoCo evaluation single-robot
- use the same exported terrain instance from the source side
- sweep multiple exported origins one by one

This gives a much cleaner sim2sim comparison because:

- terrain geometry is no longer guessed
- spawn locations are no longer guessed
- we can compare multiple sub-scenes from the same training terrain without adding multi-robot MuJoCo complexity too early

## Where To Look Next

For the concrete current sim2sim workflow, use:

- [sim2sim/README.md](sim2sim/README.md)

For earlier training runs and baseline behavior, use:

- `exp/go2-train`
