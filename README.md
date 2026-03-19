# robot_lab: Branch Notes for `exp/go2-train`

This README is branch-specific.

- Branch: `exp/go2-train`
- Goal: record only our own work on Unitree Go2 training
- Policy: keep the upstream/original README on `main`; use experiment branches to document what we actually ran, what worked, and what we plan to change next

## Scope

This branch is for:

- Go2 flat-terrain training experiments
- local reproduction notes
- training result snapshots
- follow-up improvement ideas
- migration notes for later rough-terrain or deployment work

This branch is not trying to be a full project introduction. For the original upstream project description, use `main`.

## Current Reproduction Status

Current branch conclusion:

- training is reproducible locally
- the Go2 flat policy improves during training
- playback is not yet clean enough to consider the reproduction "solved"

Current playback artifact:

- clickable gif preview: [![Go2 playback preview](docs/branch_artifacts/go2_flat_full_reproduction_2026-03-19.gif)](docs/branch_artifacts/go2_flat_full_reproduction_2026-03-19.mp4)
- video: `docs/branch_artifacts/go2_flat_full_reproduction_2026-03-19.mp4`

Observed behavior in the current playback:

- some Go2 instances fall very early
- after falling, several robots remain in a near-static state instead of recovering
- this means the current branch should be treated as a partially successful reproduction, not a final result

## Local Setup Used

The runs on this branch were reproduced with the following local setup:

- Isaac Sim binary: `5.1.0`
- Isaac Lab: `v2.3.2`
- Python: `3.11`
- Conda env: `env_isaaclab`
- GPU: `RTX 4090 D`

Useful shell entrypoints:

```bash
use_isaaclab
use_robot_lab
```

`use_isaaclab` activates the conda env and sources Isaac Sim.

`use_robot_lab` does the same and then changes into this repository root.

## What We Ran

### 1. Go2 flat baseline

Command:

```bash
python scripts/reinforcement_learning/rsl_rl/train.py \
  --task=RobotLab-Isaac-Velocity-Flat-Unitree-Go2-v0 \
  --headless \
  --num_envs=64 \
  --max_iterations=50 \
  --run_name=go2_flat_baseline
```

Artifacts:

- log dir: `logs/rsl_rl/unitree_go2_flat/2026-03-19_14-05-32_go2_flat_baseline`
- checkpoint: `model_49.pt`

Recovered config summary:

- task: `RobotLab-Isaac-Velocity-Flat-Unitree-Go2-v0`
- device: `cuda:0`
- num envs: `64`
- decimation: `4`
- episode length: `20.0 s`
- iterations: `50`

Final recorded metrics:

- `Train/mean_reward = -4.0438`
- `Train/mean_episode_length = 571.51`
- `Episode_Reward/track_lin_vel_xy_exp = 0.0158`
- `Episode_Reward/track_ang_vel_z_exp = 0.0186`
- `Episode_Termination/time_out = 1.0`

Interpretation:

- this was enough to confirm the training pipeline runs end-to-end
- it was not enough for the policy to become good
- reward was still negative at the end of this short run

### 2. Go2 flat full run

Command:

```bash
python scripts/reinforcement_learning/rsl_rl/train.py \
  --task=RobotLab-Isaac-Velocity-Flat-Unitree-Go2-v0 \
  --headless \
  --num_envs=1024 \
  --run_name=go2_flat_full
```

Artifacts:

- log dir: `logs/rsl_rl/unitree_go2_flat/2026-03-19_14-14-07_go2_flat_full`
- final checkpoint: `model_4999.pt`
- playback video: `docs/branch_artifacts/go2_flat_full_reproduction_2026-03-19.mp4`

Recovered config summary:

- task: `RobotLab-Isaac-Velocity-Flat-Unitree-Go2-v0`
- device: `cuda:0`
- num envs: `1024`
- decimation: `4`
- episode length: `20.0 s`
- iterations: `5000`

Final recorded metrics:

- `Train/mean_reward = 43.7053`
- `Train/mean_episode_length = 1000.0`
- `Episode_Reward/track_lin_vel_xy_exp = 1.1316`
- `Episode_Reward/track_ang_vel_z_exp = 0.5762`
- `Episode_Termination/time_out = 1.0`

Interpretation:

- the policy clearly learned compared with the short baseline run
- the episode length reached the task horizon
- linear velocity tracking became strong
- angular velocity tracking improved, but still looks like an area worth tuning
- successful training metrics do not yet guarantee robust playback quality
- in the recorded playback, some robots still fall at the beginning and then remain inactive

## How To Reproduce

From a new terminal:

```bash
use_robot_lab
git checkout exp/go2-train
```

Short sanity run:

```bash
python scripts/reinforcement_learning/rsl_rl/train.py \
  --task=RobotLab-Isaac-Velocity-Flat-Unitree-Go2-v0 \
  --headless \
  --num_envs=64 \
  --max_iterations=50 \
  --run_name=go2_flat_baseline
```

Longer training run:

```bash
python scripts/reinforcement_learning/rsl_rl/train.py \
  --task=RobotLab-Isaac-Velocity-Flat-Unitree-Go2-v0 \
  --headless \
  --num_envs=1024 \
  --run_name=go2_flat_full
```

If you want to evaluate a saved checkpoint later, the next step is to add a matching `play.py` command and record a short rollout video.

## What Looks Good Right Now

- Isaac Sim + Isaac Lab + `robot_lab` training pipeline is working locally
- Go2 flat training is reproducible on the current machine
- the 5000-iteration run is not just launching; it actually learns a usable policy trend
- experiment outputs are structured and saved cleanly under `logs/rsl_rl/unitree_go2_flat`

## What Still Needs Improvement

### Training-side improvements

- compare `num_envs=1024` with `2048` or `4096` to see whether throughput and stability improve
- inspect whether `track_ang_vel_z_exp` can be improved with reward or command tuning
- run multiple seeds instead of trusting a single run
- add `play.py` evaluation and a short video capture step after each major checkpoint
- compare playback quality using `num_envs=1` versus multi-env playback to separate policy issues from batch-play artifacts

### Environment-side improvements

- migrate from flat terrain to rough terrain after the flat baseline is stable
- compare command tracking and fall behavior between flat and rough Go2 tasks
- inspect contact penalties and joint penalties if motion quality looks too stiff or too conservative
- inspect why some robots fall immediately at reset/play time and do not recover afterward
- verify whether resets, initial state sampling, or action smoothing are contributing to the frozen-after-fall behavior

### Repo-side improvements

- separate branch-specific experiment notes from upstream project docs more systematically
- add a lightweight experiment index so each branch can point to the exact log/checkpoint paths it produced
- clean up dependency mismatches we found during local setup when they block other workflows

## Migration Notes

Likely next branch directions:

- `exp/go2-rough-train`: move from flat to rough terrain
- `exp/go2-eval`: add playback, checkpoint comparison, and video-based evaluation
- `fix/dependency-cleanup`: fix local setup issues that are independent of training results

Recommended branch policy:

- `main`: preserve upstream README and upstream-facing project structure
- `exp/*`: document only our own experiments, results, and local conclusions
- `fix/*`: isolate dependency or infrastructure fixes
- `docs/*`: repository notes that are ours but not tied to one experiment

## Current Branch Status

At the time of writing, this branch has:

- a branch-specific README
- one short Go2 flat verification run
- one 5000-iteration Go2 flat training run
- one recorded playback video capturing the current reproduced behavior
- saved checkpoints and TensorBoard logs for both runs

This branch should be treated as our working notebook for Go2 training, not as a generic upstream project mirror.
