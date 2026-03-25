# robot_lab: Branch Notes for `exp/go2-sim2sim`

This README is branch-specific.

- Branch: `exp/go2-sim2sim`
- Goal: prepare a Unitree Go2 policy and environment setup for simulator-to-simulator transfer
- Policy: keep upstream/original project docs on `main`; keep training reproduction notes on `exp/go2-train`; keep only sim2sim work on this branch

## Scope

This branch is for:

- sim2sim-oriented environment cleanup
- transfer interface definition
- transfer-oriented training configuration
- transfer checklists, risks, and conclusions
- artifacts that directly support migration work

This branch is not the place for general project introduction or full training history.

## Reference Branch

The training feasibility work stays on:

- `exp/go2-train`

That branch already records:

- flat / rough / stairs-only Go2 training runs
- playback artifacts
- environment comparison notes
- baseline behavior before sim2sim preparation

This branch starts after basic training feasibility has already been established.

## Current Objective

The current priority is to make the Go2 policy transferable, not to keep doing generic hyperparameter tuning.

The immediate question is:

- what must be fixed, frozen, or simplified before moving the policy into a second simulator

## Transfer Baseline

Current control and policy assumptions inherited from the training branch:

- robot: Unitree Go2
- policy type: locomotion velocity tracking
- action interface: joint position targets
- controlled joints: 12 leg joints
- low-level actuator model: `DCMotorCfg`
- control step: `0.02 s`
- policy observations are asymmetric with critic observations
- deployment should follow actor observations, not critic observations

Current transfer-relevant actor-side assumptions:

- actor does not use `base_lin_vel`
- actor does not use `height_scan`
- actor uses body angular velocity, projected gravity, commands, joint position, joint velocity, and previous action

This is useful for transfer because the deployed policy interface is already narrower than the full privileged critic state.

## What Must Be Frozen Before Transfer

Before running a dedicated sim2sim training cycle, we should freeze and document:

- actor observation term list and exact order
- action dimension and joint order
- default joint pose
- control frequency
- actuator parameters:
  - stiffness
  - damping
  - effort limit
  - velocity limit
- termination behavior
- reset behavior
- command ranges

If any of these keep drifting during experiments, it becomes much harder to tell whether transfer failure comes from training or interface mismatch.

## What Must Be Checked In The Target Simulator

Before migration, the target simulator must be checked against the current policy assumptions:

- can it accept joint position targets directly
- if not, can we reproduce the same position-control semantics with an equivalent PD layer
- does it use the same joint order
- can it reproduce the same default pose
- can it match the same control rate
- can it provide the same actor observations

The most important point is that this policy is not a direct torque policy. It depends on a position-target control interface backed by motor parameters.

## Training Strategy For Sim2sim

The next training cycle on this branch should be transfer-oriented.

That means:

- keep domain randomization that improves transfer:
  - friction randomization
  - mass randomization
  - COM randomization
  - actuator gain randomization
- keep explicit fall termination behavior
- reduce reset settings that are unrealistically extreme if they harm transfer learning
- avoid simulator-specific shortcuts in the observation or control interface
- produce checkpoints and playback artifacts labeled specifically for sim2sim preparation

The goal is not just a high reward inside Isaac Sim. The goal is a policy that survives interface shift.

## First Concrete Tasks On This Branch

The first work items here should be:

1. Record the actor-side transfer interface in a stable, deployment-facing format.
2. Create a sim2sim-prep environment config derived from the current Go2 baseline.
3. Review reset logic and reduce overly aggressive root randomization if needed.
4. Keep `illegal_contact` enabled so reset-on-fall behavior is explicit.
5. Train one dedicated sim2sim-prep checkpoint.
6. Define the target simulator adapter requirements.

## Branch Output Policy

This branch should only accumulate:

- sim2sim configs
- transfer notes
- transfer experiments
- migration conclusions

It should not become another full replay of the general training branch.

## Status

At the time of writing:

- basic training feasibility has already been verified on `exp/go2-train`
- this branch is the dedicated starting point for sim2sim work
- the next implementation step is to build the first transfer-oriented Go2 environment/config on this branch
