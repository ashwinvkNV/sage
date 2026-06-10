# Flexiv Real Robot Motion Data Collection

This collector executes joint-space motion files on a Flexiv robot through the
Flexiv RDK Python API and writes SAGE-compatible real robot logs.

## References

The implementation follows Flexiv RDK's non-real-time joint position example:

- `flexiv_rdk/example_py/intermediate1_non_realtime_joint_position_control.py`
- `flexiv_rdk/example_py/basics1_display_robot_states.py`

Key RDK calls used:

```python
robot = flexivrdk.Robot(robot_sn)
robot.ClearFault()
robot.Enable()
robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
robot.SendJointPosition({group: flexivrdk.NrtJointPositionCmd(q_d, dq_d, dq_max, ddq_max)})
states = robot.states()[group]
```

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
`NRT_JOINT_POSITION`.

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
- Keep first-pass motion amplitudes small and test each joint independently
  before multi-joint excitation.
- The RDK non-real-time example documents command frequencies from 1 to 100 Hz;
  the script enforces that range.
