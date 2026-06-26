# Flexiv Real Robot Motion Data Collection

This collector executes joint-space motion files on a Flexiv robot through the
Flexiv RDK Python API and writes SAGE-compatible real robot logs.

## References

The implementation follows Flexiv RDK's non-real-time joint impedance example by default:

- `flexiv_rdk/example_py/intermediate2_non_realtime_joint_impedance_control.py`
- `flexiv_rdk/example_py/basics1_display_robot_states.py`

Key RDK calls used:

```python
robot = flexivrdk.Robot(robot_sn)
robot.ClearFault()
robot.Enable()
robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_IMPEDANCE)
robot.SetJointImpedance(group, K_q, Z_q)
robot.SendJointPosition({group: flexivrdk.NrtJointPositionCmd(q_d, dq_d, dq_max, ddq_max)})
states = robot.states()[group]
```

Use `--flexiv-control-mode nrt_joint_position` to reproduce the older direct
non-real-time joint position collection path.

## Install

```bash
pip install flexivrdk
```

Install SAGE dependencies if needed:

```bash
pip install -r requirements.txt
```

## Motion Format

Motion files are CSV/TXT files with joint names in the header and radians in
each row:

```csv
joint1,joint2,joint3,joint4,joint5,joint6,joint7
0.0,0.0,0.0,0.0,0.0,0.0,0.0
0.02,0.0,0.0,0.0,0.0,0.0,0.0
```

The header names become `joint_list.txt` and must match the downstream robot
model / SysID config order.

## Dry Run

Validate output formatting without robot hardware:

```bash
cd /home/horde/sim2real-sysid/sage
python scripts/run_real.py \
  --robot-name flexiv \
  --motion-files custom/tiny_joint_sine.txt \
  --output-folder output \
  --auto-start \
  --flexiv-dry-run \
  --flexiv-control-freq 10
```

This writes:

```text
output/real/flexiv/custom/tiny_joint_sine/
├── command_response.csv
├── command_response_long.csv
├── control.csv
├── event.csv
├── joint_list.txt
├── state_motor.csv
└── timing.csv
```

## Hardware Run

Use very conservative limits for the first real collection:

```bash
cd /home/horde/sim2real-sysid/sage
python scripts/run_real.py \
  --robot-name flexiv \
  --motion-files custom/tiny_joint_sine.txt \
  --output-folder output \
  --flexiv-robot-sn Rizon4s-123456 \
  --flexiv-joint-group ARMS \
  --flexiv-control-mode nrt_joint_impedance \
  --flexiv-stiffness-scales 0.25 \
  --flexiv-damping-ratios 0.7 \
  --flexiv-control-freq 50 \
  --flexiv-start-move-duration 30 \
  --flexiv-start-max-velocity 0.02 \
  --flexiv-start-max-acceleration 0.03 \
  --flexiv-max-velocity 0.03 \
  --flexiv-max-acceleration 0.05 \
  --flexiv-motion-scale 0.1 \
  --flexiv-max-initial-diff-rad 0.5
```

Optional:

```bash
--flexiv-home-plan PLAN-Home
```

This executes the named Flexiv plan before switching to
`NRT_JOINT_IMPEDANCE`.

## Impedance Sweep

After one conservative impedance run is stable, collect the same motion across a
small grid of Flexiv joint impedance settings:

```bash
cd /home/horde/sim2real-sysid/sage
python scripts/run_real.py \
  --robot-name flexiv \
  --motion-files custom/sysid_zero_multisine_a0p24_60s_all_joints.txt \
  --output-folder output \
  --flexiv-robot-sn Rizon4s-123456 \
  --flexiv-joint-group ARMS \
  --flexiv-control-mode nrt_joint_impedance \
  --flexiv-stiffness-scales 0.25 0.5 0.75 1.0 \
  --flexiv-damping-ratios 0.5 0.7 \
  --flexiv-control-freq 50 \
  --flexiv-start-move-duration 30 \
  --flexiv-start-max-velocity 0.02 \
  --flexiv-start-max-acceleration 0.03 \
  --flexiv-max-velocity 0.03 \
  --flexiv-max-acceleration 0.05 \
  --flexiv-motion-scale 0.1 \
  --flexiv-max-initial-diff-rad 0.5
```

Each grid point is saved to a separate dataset folder with a suffix like:

```text
sysid_zero_multisine_a0p24_60s_all_joints_k0p25_z0p7/
```

where `k` is the scale applied to `robot.info().K_q_nom`, and `z` is the
joint damping ratio passed to `SetJointImpedance()`. Each run also writes a
`metadata.json` file with the applied controller settings.

## Safety Notes

- The script prompts before starting motion unless `--auto-start` is set.
- The script prompts if the first waypoint differs from the current pose by
  more than `--flexiv-max-initial-diff-rad`.
- `--flexiv-start-max-velocity` and `--flexiv-start-max-acceleration` apply
  only to the move from the current robot state to the first motion waypoint.
- `--flexiv-max-velocity` and `--flexiv-max-acceleration` apply during motion
  playback.
- `--flexiv-motion-scale` scales every motion row around the first waypoint.
  Use `0.1` for a 10 percent amplitude test, then increase as confidence grows.
- `--flexiv-stiffness-scales` must stay in `[0.0, 1.0]`, because RDK limits
  stiffness to `[0, robot.info().K_q_nom]`.
- `--flexiv-damping-ratios` must stay in `[0.3, 0.8]`. Start with `0.7`;
  only sweep wider after a small motion is stable.
- Sweep stiffness first at a fixed damping ratio, then expand damping. A good
  first stable ladder is `k=0.25, 0.5, 0.75, 1.0` at `z=0.7`.
- Keep first-pass motion amplitudes small and test each joint independently
  before multi-joint excitation.
- The RDK non-real-time example documents command frequencies from 1 to 100 Hz;
  the script enforces that range.

## Large Impedance Command-Response Dataset

For the hybrid GRU / command-response path, collect data through the same
Flexiv impedance interface used during deployment. The generator below creates
motion files with per-joint steps, per-joint chirps, all-joint multisines, and
randomized hold commands. The collector logs commanded joint targets in
`control.csv`, measured positions/velocities/torques in `state_motor.csv`,
aligned tracking error in `command_response.csv` and
`command_response_long.csv`, and the applied `stiffness_scale` /
`damping_ratio` in `metadata.json`.

Generate a tiny smoke dataset without touching hardware:

```bash
cd /home/agx_thor/workspaces/ashwinvk/sage
source .venv/bin/activate

python scripts/run_flexiv_impedance_dataset.py \
  --dataset-name flexiv_impedance_smoke \
  --profile smoke \
  --prepare-only \
  --overwrite
```

Run that smoke dataset on the robot:

```bash
python scripts/move_flexiv_to_home.py \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --max-velocity 0.02 \
  --max-acceleration 0.04 \
  --timeout-s 240 \
  --tolerance-deg 0.5

python scripts/run_flexiv_impedance_dataset.py \
  --dataset-name flexiv_impedance_smoke \
  --profile smoke \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --stiffness-scales 0.75 \
  --damping-ratios 0.7 \
  --overwrite
```

Prepare the larger dataset:

```bash
python scripts/run_flexiv_impedance_dataset.py \
  --dataset-name flexiv_impedance_hybrid_v1 \
  --profile large \
  --prepare-only \
  --overwrite
```

Collect the larger dataset for the primary deployment setting:

```bash
python scripts/run_flexiv_impedance_dataset.py \
  --dataset-name flexiv_impedance_hybrid_v1 \
  --profile large \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --home-zero-first \
  --stiffness-scales 0.75 \
  --damping-ratios 0.7
```

If you want the learned model to condition on different Flexiv impedance
settings, expand the controller sweep:

```bash
python scripts/run_flexiv_impedance_dataset.py \
  --dataset-name flexiv_impedance_hybrid_sweep_v1 \
  --profile large \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --home-zero-first \
  --stiffness-scales 0.5 0.75 1.0 \
  --damping-ratios 0.5 0.7
```

The generated motions live under:

```text
motion_files/flexiv/impedance_hybrid/<dataset-name>/
```

The collected real data is saved under:

```text
output/real/flexiv/impedance_hybrid/<dataset-name>/<motion>_k<scale>_z<ratio>/
```

By default, generated motions are zero-based and `run_real.py` moves slowly to
the first waypoint before each motion. On a Rizon4s where all-zero joint
position is the selected lab home pose, run `--home-zero-first` or
`scripts/move_flexiv_to_home.py` first. If you want to collect around the
current robot pose instead, add `--center-on-current-pose`.

### Tracking Error Files

`command_response.csv` is a vector-format derived file. For each feedback
sample, it picks the latest command whose send timestamp is at or before the
state-read timestamp and writes:

```text
feedback_timestamp
command_timestamp
command_age_s
command_positions
measured_positions
measured_velocities
measured_torques
position_error
```

where:

```text
position_error = command_positions - measured_positions
```

`command_response_long.csv` stores the same aligned data one joint per row,
which is usually easier for GRU training/debugging.

## Home / Reset Workflow Notes

Flexiv's public RDK examples commonly call:

```cpp
robot.SwitchMode(flexiv::rdk::Mode::NRT_PLAN_EXECUTION);
robot.ExecutePlan("PLAN-Home");
```

but `PLAN-Home` must exist on the robot controller. If that saved plan is not
present in Flexiv Elements, use a slow joint-space reset instead:

```bash
python scripts/move_flexiv_to_home.py \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --target-q 0,0,0,0,0,0,0 \
  --max-velocity 0.02 \
  --max-acceleration 0.04 \
  --timeout-s 240 \
  --tolerance-deg 0.5
```

The torque SysID collector also supports this directly:

```bash
python scripts/run_flexiv_torque_sysid.py \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --home-zero \
  --reset-dq-max 0.02 \
  --reset-ddq-max 0.04 \
  --reset-timeout-s 240 \
  --reset-tolerance-deg 0.5 \
  --name rt_torque_joint1_smoke \
  --joints joint1 \
  --max-torque-nm 0.2 \
  --offset-limit-deg 5 \
  --passive-offset-limit-deg 2 \
  --velocity-limit-rad-s 0.15 \
  --trial-timeout-s 4 \
  --ramp-duration-s 2
```

For RDK v1.8 we wait on measured joint-position error rather than relying only
on `robot.stopped()` after `SendJointPosition()`. This is the safer criterion
for torque SysID because it verifies the robot is actually within the desired
joint-space tolerance before switching into `RT_JOINT_TORQUE`.

Public RDK headers document `Robot::SetVelocityScale()` for plan and primitive
execution, but not for `SendJointPosition()` trajectories. If you use
`ExecutePlan()`, you can try `SetVelocityScale()` in
`NRT_PLAN_EXECUTION`; otherwise use explicit `dq_max`/`ddq_max` with
`NRT_JOINT_POSITION`.

## Direct RT Joint Torque SysID

Flexiv's Python RDK package may not expose `RT_JOINT_TORQUE`, but the C++ RDK
does. Use this path only for conservative dynamics-identification tests. It
streams direct joint torque commands at 1 kHz, ramps one joint at a time, stops
when a small offset is reached, then resets before the next sign/joint trial.

Build and print the command without touching hardware:

```bash
cd /home/agx_thor/workspaces/ashwinvk/sage
source .venv/bin/activate

python scripts/run_flexiv_torque_sysid.py \
  --robot-sn Rizon4s-123456 \
  --name rt_torque_joint1_smoke \
  --home-zero \
  --joints joint1 \
  --max-torque-nm 0.2 \
  --offset-limit-deg 5 \
  --passive-offset-limit-deg 2 \
  --velocity-limit-rad-s 0.15 \
  --trial-timeout-s 4 \
  --ramp-duration-s 2 \
  --dry-run
```

First hardware smoke test:

```bash
cd /home/agx_thor/workspaces/ashwinvk/sage
source .venv/bin/activate

python scripts/run_flexiv_torque_sysid.py \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --home-zero \
  --reset-dq-max 0.02 \
  --reset-ddq-max 0.04 \
  --reset-timeout-s 240 \
  --reset-tolerance-deg 0.5 \
  --name rt_torque_joint1_smoke \
  --joints joint1 \
  --max-torque-nm 0.2 \
  --offset-limit-deg 5 \
  --passive-offset-limit-deg 2 \
  --velocity-limit-rad-s 0.15 \
  --trial-timeout-s 4 \
  --ramp-duration-s 2
```

Only after that is stable, run all joints at the intended 10 degree stop:

```bash
python scripts/run_flexiv_torque_sysid.py \
  --robot-sn Rizon4s-123456 \
  --joint-group ARMS \
  --home-zero \
  --reset-dq-max 0.02 \
  --reset-ddq-max 0.04 \
  --reset-timeout-s 240 \
  --reset-tolerance-deg 0.5 \
  --name rt_torque_all_joints_pm10deg \
  --joints all \
  --max-torque-nm 0.3 \
  --offset-limit-deg 10 \
  --passive-offset-limit-deg 3 \
  --velocity-limit-rad-s 0.25 \
  --trial-timeout-s 8 \
  --ramp-duration-s 2
```

If Flexiv RDK C++ is installed in a non-system prefix, add:

```bash
--rdk-prefix /path/to/rdk_install
```

The resulting dataset is saved under:

```text
output/real/flexiv/torque_sysid/<name>/
```

Files written:

- `control_torque.csv`: commanded torque vector per RT sample.
- `state_motor.csv`: SAGE-style measured `positions`, `velocities`,
  `torques`.
- `state_motor_extended.csv`: measured `tau_des`, `tau_ext`, motor-side
  `theta`/`dtheta`, and commanded torque.
- `event.csv`: trial start/end markers.
- `trial_summary.csv`: stop reason and max offsets per trial.
- `metadata.json`: torque limits, home pose, reset policy, and timestamp
  policy.

The C++ collector keeps Flexiv gravity compensation and soft limits enabled by
default in `RtJointTorqueCmd`. Do not pass `--disable-soft-limits` unless you
have a specific safety-reviewed reason.
