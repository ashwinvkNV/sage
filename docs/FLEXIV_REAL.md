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
├── control.csv
├── event.csv
├── joint_list.txt
└── state_motor.csv
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
  --home-plan PLAN-Home \
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
  --home-plan PLAN-Home \
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
