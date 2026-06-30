#!/usr/bin/env python3
# Copyright (c) 2026, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Generate and optionally collect a Flexiv impedance command-response dataset.

The dataset is meant for learning the robot-side impedance command path:

    command q target, measured q, dq, stiffness_scale, damping_ratio -> response

It reuses scripts/run_real.py, so the data is saved in the same SAGE real-robot
CSV format as the smaller Flexiv collections.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import subprocess
import sys
from pathlib import Path


JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
DEFAULT_STIFFNESS_SCALES = [0.75]
DEFAULT_DAMPING_RATIOS = [0.7]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def format_scalar(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def read_base_state(path: Path) -> tuple[list[str], list[float]]:
    with path.open(newline="") as f:
        reader = csv.reader(f)
        joint_names = next(reader)
        row = next(reader)
    return joint_names, [float(value) for value in row]


def write_motion(path: Path, joint_names: list[str], rows: list[list[float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(joint_names)
        for row in rows:
            writer.writerow([f"{value:.9f}" for value in row])


def hold_rows(base: list[float], control_freq: float, seconds: float) -> list[list[float]]:
    steps = max(1, int(round(seconds * control_freq)))
    return [list(base) for _ in range(steps)]


def smooth_envelope(t: float, duration: float, fade_s: float = 1.0) -> float:
    if duration <= 0.0 or fade_s <= 0.0:
        return 1.0
    fade_s = min(fade_s, 0.5 * duration)
    if t < fade_s:
        return 0.5 - 0.5 * math.cos(math.pi * t / fade_s)
    if t > duration - fade_s:
        return 0.5 - 0.5 * math.cos(math.pi * (duration - t) / fade_s)
    return 1.0


def chirp_phase(t: float, duration: float, f_start: float, f_end: float) -> float:
    sweep_rate = (f_end - f_start) / duration
    return 2.0 * math.pi * (f_start * t + 0.5 * sweep_rate * t * t)


def generate_step_motion(
    base: list[float],
    control_freq: float,
    amplitude_rad: float,
    hold_seconds: float,
    settle_seconds: float,
) -> list[list[float]]:
    rows: list[list[float]] = []
    for joint_idx in range(len(base)):
        plus_pose = list(base)
        minus_pose = list(base)
        plus_pose[joint_idx] += amplitude_rad
        minus_pose[joint_idx] -= amplitude_rad

        rows.extend(hold_rows(base, control_freq, settle_seconds))
        rows.extend(hold_rows(plus_pose, control_freq, hold_seconds))
        rows.extend(hold_rows(base, control_freq, hold_seconds))
        rows.extend(hold_rows(minus_pose, control_freq, hold_seconds))
        rows.extend(hold_rows(base, control_freq, settle_seconds))
    return rows


def generate_chirp_motion(
    base: list[float],
    control_freq: float,
    amplitude_rad: float,
    duration_per_joint: float,
    f_start: float,
    f_end: float,
    settle_seconds: float,
) -> list[list[float]]:
    rows: list[list[float]] = hold_rows(base, control_freq, settle_seconds)
    segment_steps = max(2, int(round(duration_per_joint * control_freq)))

    for joint_idx in range(len(base)):
        for step in range(segment_steps):
            t = step / control_freq
            row = list(base)
            row[joint_idx] += (
                amplitude_rad
                * smooth_envelope(t, duration_per_joint)
                * math.sin(chirp_phase(t, duration_per_joint, f_start, f_end))
            )
            rows.append(row)
        rows.extend(hold_rows(base, control_freq, settle_seconds))
    return rows


def generate_multisine_motion(
    base: list[float],
    control_freq: float,
    amplitude_rad: float,
    duration: float,
    seed: int,
    frequencies: list[float],
    settle_seconds: float,
) -> list[list[float]]:
    rng = random.Random(seed)
    phases = [[rng.uniform(0.0, 2.0 * math.pi) for _ in frequencies] for _ in base]
    signs = [[rng.choice([-1.0, 1.0]) for _ in frequencies] for _ in base]
    rows: list[list[float]] = hold_rows(base, control_freq, settle_seconds)
    steps = max(2, int(round(duration * control_freq)))
    normalizer = max(1.0, math.sqrt(len(frequencies)))

    for step in range(steps):
        t = step / control_freq
        env = smooth_envelope(t, duration)
        row = list(base)
        for joint_idx in range(len(base)):
            value = 0.0
            for freq_idx, freq in enumerate(frequencies):
                value += signs[joint_idx][freq_idx] * math.sin(
                    2.0 * math.pi * freq * t + phases[joint_idx][freq_idx]
                )
            row[joint_idx] += amplitude_rad * env * value / normalizer
        rows.append(row)

    rows.extend(hold_rows(base, control_freq, settle_seconds))
    return rows


def generate_random_hold_motion(
    base: list[float],
    control_freq: float,
    amplitude_rad: float,
    duration: float,
    seed: int,
    hold_min_s: float,
    hold_max_s: float,
    settle_seconds: float,
    active_joint_probability: float,
) -> list[list[float]]:
    rng = random.Random(seed)
    rows: list[list[float]] = hold_rows(base, control_freq, settle_seconds)
    total_steps = max(2, int(round(duration * control_freq)))
    generated_steps = 0

    while generated_steps < total_steps:
        hold_s = rng.uniform(hold_min_s, hold_max_s)
        hold_steps = max(1, int(round(hold_s * control_freq)))
        hold_steps = min(hold_steps, total_steps - generated_steps)
        target = list(base)
        any_active = False
        for joint_idx in range(len(base)):
            if rng.random() < active_joint_probability:
                any_active = True
                target[joint_idx] += rng.uniform(-amplitude_rad, amplitude_rad)
        if not any_active:
            joint_idx = rng.randrange(len(base))
            target[joint_idx] += rng.uniform(-amplitude_rad, amplitude_rad)

        for _ in range(hold_steps):
            rows.append(list(target))
        generated_steps += hold_steps

    rows.extend(hold_rows(base, control_freq, settle_seconds))
    return rows


def resolve_profile(profile: str) -> dict[str, object]:
    if profile == "smoke":
        return {
            "step_degrees": [2.0],
            "chirp_degrees": [2.0],
            "multisine_degrees": [2.0],
            "random_hold_degrees": [2.0],
            "chirp_duration_per_joint": 4.0,
            "multisine_duration": 10.0,
            "random_hold_duration": 10.0,
            "seeds": [1],
        }
    if profile == "large":
        return {
            "step_degrees": [2.0, 5.0, 10.0],
            "chirp_degrees": [3.0, 6.0],
            "multisine_degrees": [4.0, 8.0],
            "random_hold_degrees": [4.0, 8.0],
            "chirp_duration_per_joint": 12.0,
            "multisine_duration": 60.0,
            "random_hold_duration": 60.0,
            "seeds": [1, 2, 3],
        }
    raise ValueError(f"Unknown profile: {profile}")


def add_motion(
    manifest: list[dict[str, object]],
    motion_dir: Path,
    joint_names: list[str],
    name: str,
    kind: str,
    rows: list[list[float]],
    control_freq: float,
    details: dict[str, object],
) -> None:
    file_name = f"{name}.txt"
    write_motion(motion_dir / file_name, joint_names, rows)
    manifest.append(
        {
            "file": file_name,
            "name": name,
            "kind": kind,
            "rows": len(rows),
            "duration_s": len(rows) / control_freq,
            **details,
        }
    )


def generate_dataset(
    motion_dir: Path,
    joint_names: list[str],
    base: list[float],
    control_freq: float,
    profile: str,
    overwrite: bool,
) -> list[dict[str, object]]:
    if motion_dir.exists() and any(motion_dir.glob("*.txt")) and not overwrite:
        raise FileExistsError(
            f"{motion_dir} already contains .txt files. Pass --overwrite to regenerate it."
        )
    motion_dir.mkdir(parents=True, exist_ok=True)
    if overwrite:
        for txt_file in motion_dir.glob("*.txt"):
            txt_file.unlink()

    cfg = resolve_profile(profile)
    manifest: list[dict[str, object]] = []
    settle_seconds = 1.0

    for amp_deg in cfg["step_degrees"]:
        rows = generate_step_motion(
            base=base,
            control_freq=control_freq,
            amplitude_rad=math.radians(float(amp_deg)),
            hold_seconds=2.0,
            settle_seconds=settle_seconds,
        )
        add_motion(
            manifest,
            motion_dir,
            joint_names,
            name=f"cmd_response_steps_pm{format_scalar(float(amp_deg))}deg",
            kind="per_joint_step",
            rows=rows,
            control_freq=control_freq,
            details={"amplitude_deg": amp_deg, "hold_seconds": 2.0},
        )

    for amp_deg in cfg["chirp_degrees"]:
        rows = generate_chirp_motion(
            base=base,
            control_freq=control_freq,
            amplitude_rad=math.radians(float(amp_deg)),
            duration_per_joint=float(cfg["chirp_duration_per_joint"]),
            f_start=0.05,
            f_end=0.45,
            settle_seconds=settle_seconds,
        )
        add_motion(
            manifest,
            motion_dir,
            joint_names,
            name=f"cmd_response_chirp_a{format_scalar(float(amp_deg))}deg",
            kind="per_joint_chirp",
            rows=rows,
            control_freq=control_freq,
            details={
                "amplitude_deg": amp_deg,
                "duration_per_joint_s": cfg["chirp_duration_per_joint"],
                "f_start_hz": 0.05,
                "f_end_hz": 0.45,
            },
        )

    frequencies = [0.07, 0.11, 0.17, 0.23, 0.31, 0.41]
    for amp_deg in cfg["multisine_degrees"]:
        for seed in cfg["seeds"]:
            rows = generate_multisine_motion(
                base=base,
                control_freq=control_freq,
                amplitude_rad=math.radians(float(amp_deg)),
                duration=float(cfg["multisine_duration"]),
                seed=int(seed),
                frequencies=frequencies,
                settle_seconds=settle_seconds,
            )
            add_motion(
                manifest,
                motion_dir,
                joint_names,
                name=f"cmd_response_multisine_a{format_scalar(float(amp_deg))}deg_seed{seed}",
                kind="all_joint_multisine",
                rows=rows,
                control_freq=control_freq,
                details={
                    "amplitude_deg": amp_deg,
                    "active_duration_s": cfg["multisine_duration"],
                    "seed": seed,
                    "frequencies_hz": frequencies,
                },
            )

    for amp_deg in cfg["random_hold_degrees"]:
        for seed in cfg["seeds"]:
            rows = generate_random_hold_motion(
                base=base,
                control_freq=control_freq,
                amplitude_rad=math.radians(float(amp_deg)),
                duration=float(cfg["random_hold_duration"]),
                seed=int(seed),
                hold_min_s=0.25,
                hold_max_s=0.9,
                settle_seconds=settle_seconds,
                active_joint_probability=0.35,
            )
            add_motion(
                manifest,
                motion_dir,
                joint_names,
                name=f"cmd_response_random_hold_a{format_scalar(float(amp_deg))}deg_seed{seed}",
                kind="random_hold",
                rows=rows,
                control_freq=control_freq,
                details={
                    "amplitude_deg": amp_deg,
                    "active_duration_s": cfg["random_hold_duration"],
                    "seed": seed,
                    "hold_min_s": 0.25,
                    "hold_max_s": 0.9,
                    "active_joint_probability": 0.35,
                },
            )

    return manifest


def build_run_real_command(args: argparse.Namespace, motion_folder: str) -> list[str]:
    robot_sn = args.robot_sn or "<REAL_ROBOT_SN>"
    cmd = [
        sys.executable,
        "scripts/run_real.py",
        "--robot-name",
        "flexiv",
        "--motion-folder",
        motion_folder,
        "--output-folder",
        args.output_folder,
        "--flexiv-robot-sn",
        robot_sn,
        "--flexiv-joint-group",
        args.joint_group,
        "--flexiv-control-mode",
        "nrt_joint_impedance",
        "--flexiv-control-freq",
        str(args.control_freq),
        "--flexiv-start-move-duration",
        str(args.start_move_duration),
        "--flexiv-start-max-velocity",
        str(args.start_max_velocity),
        "--flexiv-start-max-acceleration",
        str(args.start_max_acceleration),
        "--flexiv-max-velocity",
        str(args.max_velocity),
        "--flexiv-max-acceleration",
        str(args.max_acceleration),
        "--flexiv-max-initial-diff-rad",
        str(args.max_initial_diff_rad),
        "--flexiv-stiffness-scales",
        *[str(value) for value in args.stiffness_scales],
        "--flexiv-damping-ratios",
        *[str(value) for value in args.damping_ratios],
    ]
    if args.home_plan:
        cmd += ["--flexiv-home-plan", args.home_plan]
    if args.center_on_current_pose:
        cmd.append("--flexiv-center-on-current-pose")
    if args.auto_start:
        cmd.append("--auto-start")
    if args.flexiv_dry_run:
        cmd.append("--flexiv-dry-run")
    if args.repeats != 1:
        cmd += ["--repeats", str(args.repeats)]
    return cmd


def build_home_command(args: argparse.Namespace) -> list[str]:
    robot_sn = args.robot_sn or "<REAL_ROBOT_SN>"
    cmd = [
        sys.executable,
        "scripts/move_flexiv_to_home.py",
        "--robot-sn",
        robot_sn,
        "--joint-group",
        args.joint_group,
        "--max-velocity",
        str(args.home_max_velocity),
        "--max-acceleration",
        str(args.home_max_acceleration),
        "--timeout-s",
        str(args.home_timeout_s),
        "--tolerance-deg",
        str(args.home_tolerance_deg),
    ]
    if args.auto_start:
        cmd.append("--auto-start")
    return cmd


def write_manifest(
    motion_dir: Path,
    args: argparse.Namespace,
    motion_folder: str,
    joint_names: list[str],
    base: list[float],
    motions: list[dict[str, object]],
    run_command: list[str],
    home_command: list[str],
) -> None:
    duration_per_setting = sum(float(item["duration_s"]) for item in motions)
    settings_count = len(args.stiffness_scales) * len(args.damping_ratios) * args.repeats
    manifest = {
        "dataset_name": args.dataset_name,
        "profile": args.profile,
        "motion_folder": motion_folder,
        "control_freq_hz": args.control_freq,
        "joint_names": joint_names,
        "base": base,
        "intended_controller": "Flexiv NRT_JOINT_IMPEDANCE",
        "stiffness_scales": args.stiffness_scales,
        "damping_ratios": args.damping_ratios,
        "repeats": args.repeats,
        "duration_per_impedance_setting_s": duration_per_setting,
        "estimated_total_motion_duration_s": duration_per_setting * settings_count,
        "home_zero_first": args.home_zero_first,
        "home_command": home_command,
        "run_real_command": run_command,
        "motions": motions,
    }
    with (motion_dir / "dataset_manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--dataset-name", default="flexiv_impedance_hybrid_v1")
    parser.add_argument(
        "--motion-folder",
        default=None,
        help="Folder under motion_files/flexiv. Defaults to impedance_hybrid/<dataset-name>.",
    )
    parser.add_argument(
        "--motion-root",
        type=Path,
        default=None,
        help="Root folder for generated motions. Defaults to motion_files/flexiv.",
    )
    parser.add_argument("--profile", choices=["smoke", "large"], default="large")
    parser.add_argument("--control-freq", type=int, default=50)
    parser.add_argument("--base-state-csv", type=Path, default=None)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--prepare-only", action="store_true", help="Generate motion files but do not run the robot.")
    parser.add_argument("--print-command-only", action="store_true", help="Print the run_real.py command and exit.")

    parser.add_argument("--robot-sn", default=None, help="Flexiv robot serial number.")
    parser.add_argument("--joint-group", default="ARMS")
    parser.add_argument("--home-plan", default="", help="Optional saved Flexiv plan to run before each motion.")
    parser.add_argument(
        "--center-on-current-pose",
        action="store_true",
        help="Shift each generated zero-based motion to the robot's current pose.",
    )
    parser.add_argument(
        "--home-zero-first",
        action="store_true",
        help="Before collection, move once to all-zero joint home with scripts/move_flexiv_to_home.py.",
    )
    parser.add_argument("--home-max-velocity", type=float, default=0.02)
    parser.add_argument("--home-max-acceleration", type=float, default=0.04)
    parser.add_argument("--home-timeout-s", type=float, default=240.0)
    parser.add_argument("--home-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--output-folder", default="output")
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--stiffness-scales", type=float, nargs="+", default=DEFAULT_STIFFNESS_SCALES)
    parser.add_argument("--damping-ratios", type=float, nargs="+", default=DEFAULT_DAMPING_RATIOS)
    parser.add_argument("--start-move-duration", type=float, default=30.0)
    parser.add_argument("--start-max-velocity", type=float, default=0.03)
    parser.add_argument("--start-max-acceleration", type=float, default=0.05)
    parser.add_argument("--max-velocity", type=float, default=0.20)
    parser.add_argument("--max-acceleration", type=float, default=0.50)
    parser.add_argument("--max-initial-diff-rad", type=float, default=0.5)
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--flexiv-dry-run", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = repo_root()
    default_motion_root = root / "motion_files" / "flexiv"
    motion_root = args.motion_root or default_motion_root
    motion_folder = args.motion_folder or f"impedance_hybrid/{args.dataset_name}"
    motion_dir = motion_root / motion_folder

    if args.control_freq <= 0.0:
        raise ValueError("--control-freq must be positive")
    if args.repeats < 1:
        raise ValueError("--repeats must be >= 1")
    for value in args.stiffness_scales:
        if value < 0.0 or value > 1.0:
            raise ValueError("--stiffness-scales values must be in [0.0, 1.0]")
    for value in args.damping_ratios:
        if value < 0.3 or value > 0.8:
            raise ValueError("--damping-ratios values must be in [0.3, 0.8]")

    if args.base_state_csv:
        joint_names, base = read_base_state(args.base_state_csv)
    else:
        joint_names, base = JOINT_NAMES, [0.0] * len(JOINT_NAMES)

    motions = generate_dataset(
        motion_dir=motion_dir,
        joint_names=joint_names,
        base=base,
        control_freq=args.control_freq,
        profile=args.profile,
        overwrite=args.overwrite,
    )

    run_command = build_run_real_command(args, motion_folder)
    home_command = build_home_command(args)
    write_manifest(
        motion_dir=motion_dir,
        args=args,
        motion_folder=motion_folder,
        joint_names=joint_names,
        base=base,
        motions=motions,
        run_command=run_command,
        home_command=home_command,
    )

    duration_per_setting = sum(float(item["duration_s"]) for item in motions)
    total_duration = duration_per_setting * len(args.stiffness_scales) * len(args.damping_ratios) * args.repeats
    print(f"Generated {len(motions)} motion files in: {motion_dir}")
    print(f"Duration per impedance setting: {duration_per_setting / 60.0:.1f} min")
    print(f"Estimated total commanded-motion time: {total_duration / 60.0:.1f} min")
    print(f"Manifest: {motion_dir / 'dataset_manifest.json'}")
    print("\nOptional home command:")
    print(" ".join(home_command))
    print("\nCollection command:")
    print(" ".join(run_command))

    if motion_root != default_motion_root and not (args.prepare_only or args.print_command_only):
        raise RuntimeError(
            "--motion-root is only supported with --prepare-only/--print-command-only because run_real.py "
            "loads motions from motion_files/flexiv."
        )

    if args.prepare_only or args.print_command_only:
        return
    if not args.robot_sn and not args.flexiv_dry_run:
        raise ValueError("--robot-sn is required unless --flexiv-dry-run, --prepare-only, or --print-command-only is set")

    if args.home_zero_first and not args.flexiv_dry_run:
        subprocess.run(home_command, cwd=root, check=True)
    subprocess.run(run_command, cwd=root, check=True)


if __name__ == "__main__":
    main()
