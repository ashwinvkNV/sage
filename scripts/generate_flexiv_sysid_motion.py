#!/usr/bin/env python3
"""Generate Flexiv motion files for SysID and joint characterization."""

import argparse
import csv
import math
from pathlib import Path

DEFAULT_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]


def read_base_state(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        joint_names = next(reader)
        row = next(reader)
    return joint_names, [float(value) for value in row]


def chirp_phase(t, duration, f_start, f_end):
    sweep_rate = (f_end - f_start) / duration
    return 2.0 * math.pi * (f_start * t + 0.5 * sweep_rate * t * t)


def envelope(t, duration):
    if duration <= 0.0:
        return 0.0
    return math.sin(math.pi * t / duration) ** 2


def hold_rows(base, control_freq, hold_seconds):
    hold_steps = max(1, int(round(hold_seconds * control_freq)))
    return [list(base) for _ in range(hold_steps)]


def generate_single_joint_steps(
    base,
    control_freq,
    step_radians,
    hold_seconds,
    settle_seconds,
):
    """Move each joint with instantaneous +step / -step setpoints."""
    rows = []
    for joint_idx in range(len(base)):
        plus_pose = list(base)
        plus_pose[joint_idx] += step_radians
        minus_pose = list(base)
        minus_pose[joint_idx] -= step_radians

        rows.extend(hold_rows(base, control_freq, settle_seconds))
        rows.extend(hold_rows(plus_pose, control_freq, hold_seconds))
        rows.extend(hold_rows(base, control_freq, hold_seconds))
        rows.extend(hold_rows(minus_pose, control_freq, hold_seconds))
        rows.extend(hold_rows(base, control_freq, settle_seconds))
    return rows


def generate_single_joint_chirps(
    base,
    control_freq,
    amplitude,
    duration_per_joint,
    f_start,
    f_end,
    settle_seconds,
):
    rows = []
    settle_steps = max(1, int(round(settle_seconds * control_freq)))
    for _ in range(settle_steps):
        rows.append(list(base))

    segment_steps = max(2, int(round(duration_per_joint * control_freq)))
    for joint_idx in range(len(base)):
        for step in range(segment_steps):
            t = step / control_freq
            row = list(base)
            row[joint_idx] += amplitude * envelope(t, duration_per_joint) * math.sin(
                chirp_phase(t, duration_per_joint, f_start, f_end)
            )
            rows.append(row)
        for _ in range(settle_steps):
            rows.append(list(base))

    return rows


def write_motion(path, joint_names, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(joint_names)
        for row in rows:
            writer.writerow([f"{value:.9f}" for value in row])


def add_common_args(parser):
    parser.add_argument("--output", required=True, help="Output motion .txt/.csv path")
    parser.add_argument("--control-freq", type=float, default=50.0, help="Motion file sample rate in Hz")
    parser.add_argument(
        "--base-state-csv",
        default=None,
        help="CSV with joint names and base pose. Defaults to all-zero 7-joint pose.",
    )
    parser.add_argument(
        "--joint-count",
        type=int,
        default=7,
        help="Number of joints when --base-state-csv is not provided.",
    )


def resolve_base_state(base_state_csv, joint_count):
    if base_state_csv is not None:
        return read_base_state(base_state_csv)
    joint_names = DEFAULT_JOINT_NAMES[:joint_count]
    if len(joint_names) != joint_count:
        raise ValueError(f"--joint-count {joint_count} is not supported without --base-state-csv")
    return joint_names, [0.0] * joint_count


def build_chirp_parser(subparsers):
    parser = subparsers.add_parser("chirp", help="Generate per-joint chirp excitation motions.")
    add_common_args(parser)
    parser.add_argument("--amplitude", type=float, default=0.02, help="Joint chirp amplitude in radians")
    parser.add_argument("--duration-per-joint", type=float, default=30.0, help="Seconds per joint chirp")
    parser.add_argument("--f-start", type=float, default=0.05, help="Start frequency in Hz")
    parser.add_argument("--f-end", type=float, default=0.30, help="End frequency in Hz")
    parser.add_argument("--settle-seconds", type=float, default=2.0, help="Base-pose settle time between joints")
    return parser


def build_joint_step_parser(subparsers):
    parser = subparsers.add_parser(
        "joint-step",
        help="Generate per-joint +/- step motions for gain characterization.",
    )
    add_common_args(parser)
    parser.add_argument(
        "--step-degrees",
        type=float,
        default=10.0,
        help="Joint step size in degrees for + and - moves.",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=3.0,
        help="Hold time at each step level before the next setpoint jump.",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=2.0,
        help="Hold time at base pose between joints.",
    )
    return parser


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="motion_type", required=True)
    chirp_parser = build_chirp_parser(subparsers)
    joint_step_parser = build_joint_step_parser(subparsers)
    args = parser.parse_args()

    if args.control_freq <= 0.0:
        parser.error("--control-freq must be positive")

    joint_names, base = resolve_base_state(args.base_state_csv, args.joint_count)

    if args.motion_type == "chirp":
        if args.amplitude <= 0.0:
            chirp_parser.error("--amplitude must be positive")
        if args.duration_per_joint <= 0.0:
            chirp_parser.error("--duration-per-joint must be positive")
        if args.f_start <= 0.0 or args.f_end <= 0.0:
            chirp_parser.error("--f-start and --f-end must be positive")
        if args.settle_seconds < 0.0:
            chirp_parser.error("--settle-seconds must be non-negative")
        rows = generate_single_joint_chirps(
            base=base,
            control_freq=args.control_freq,
            amplitude=args.amplitude,
            duration_per_joint=args.duration_per_joint,
            f_start=args.f_start,
            f_end=args.f_end,
            settle_seconds=args.settle_seconds,
        )
        duration = len(rows) / args.control_freq
        max_velocity = 2.0 * math.pi * args.f_end * args.amplitude
        max_acceleration = (2.0 * math.pi * args.f_end) ** 2 * args.amplitude
        print(f"Wrote {len(rows)} rows to {args.output}")
        print(f"Nominal duration at {args.control_freq:g} Hz: {duration:.1f}s")
        print(f"Peak sine velocity estimate: {max_velocity:.3f} rad/s")
        print(f"Peak sine acceleration estimate: {max_acceleration:.3f} rad/s^2")
    else:
        if args.step_degrees <= 0.0:
            joint_step_parser.error("--step-degrees must be positive")
        if args.hold_seconds < 0.0 or args.settle_seconds < 0.0:
            joint_step_parser.error("--hold-seconds and --settle-seconds must be non-negative")
        step_radians = math.radians(args.step_degrees)
        rows = generate_single_joint_steps(
            base=base,
            control_freq=args.control_freq,
            step_radians=step_radians,
            hold_seconds=args.hold_seconds,
            settle_seconds=args.settle_seconds,
        )
        duration = len(rows) / args.control_freq
        print(f"Wrote {len(rows)} rows to {args.output}")
        print(f"Nominal duration at {args.control_freq:g} Hz: {duration:.1f}s")
        print(f"Step size: +/-{args.step_degrees:g} deg ({step_radians:.4f} rad)")
        print("Setpoints are instantaneous steps (no interpolation ramps).")
        print(
            "Example gain sweep:\n"
            "  python scripts/run_real.py --robot-name flexiv \\\n"
            f"    --motion-files custom/{Path(args.output).name} \\\n"
            "    --output-folder output --flexiv-robot-sn Rizon4s-063459 \\\n"
            "    --flexiv-center-on-current-pose \\\n"
            "    --flexiv-control-mode nrt_joint_impedance \\\n"
            "    --flexiv-stiffness-scales 0.25 0.5 0.75 1.0 \\\n"
            "    --flexiv-damping-ratios 0.5 0.7"
        )

    write_motion(Path(args.output), joint_names, rows)


if __name__ == "__main__":
    main()
