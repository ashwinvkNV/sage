# Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Flexiv motion collector for SAGE-compatible real robot data.

The hardware path uses Flexiv RDK non-real-time joint position control:

    robot = flexivrdk.Robot(robot_sn)
    robot.ClearFault()
    robot.Enable()
    robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
    robot.SendJointPosition({group: flexivrdk.NrtJointPositionCmd(...)})

Collected data is written as control.csv, state_motor.csv, event.csv, and
joint_list.txt for the SAGE / IsaacLab-Newton SysID flow.
"""

from __future__ import annotations

import csv
import os
import time
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

try:
    import flexivrdk

    FLEXIV_AVAILABLE = True
except ImportError:
    flexivrdk = None
    FLEXIV_AVAILABLE = False


DEFAULT_FLEXIV_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]


def interpolate_motion(seq, original_freq, target_freq):
    """Interpolate a joint trajectory to the target control frequency."""
    n_frames, n_joints = seq.shape
    duration = (n_frames - 1) / original_freq
    t_src = np.linspace(0, duration, n_frames)
    new_n_frames = int(duration * target_freq) + 1
    t_dst = np.linspace(0, duration, new_n_frames)
    seq_interp = np.zeros((new_n_frames, n_joints))
    for j in range(n_joints):
        f = interp1d(t_src, seq[:, j], kind="linear")
        seq_interp[:, j] = f(t_dst)
    return seq_interp


def load_motion_from_txt(txt_file_path):
    """Load a SAGE-style motion CSV/TXT with joint names in the header."""
    data_df = pd.read_csv(txt_file_path)
    joint_names = data_df.columns.tolist()
    seq = data_df.values.astype(np.float64)
    motion_name = Path(txt_file_path).stem
    motion_freq = 50
    return seq, joint_names, motion_freq, motion_name


def _select_group(robot, requested_group_name):
    """Select a Flexiv joint group by name, or the first available group."""
    groups = list(robot.groups())
    if not groups:
        raise RuntimeError("Flexiv robot reported no joint groups")

    if requested_group_name is None:
        return groups[0]

    requested_group_name = requested_group_name.upper()
    for group in groups:
        group_name = flexivrdk.kJointGroupNames[group]
        if group_name.upper() == requested_group_name:
            return group

    available = [flexivrdk.kJointGroupNames[group] for group in groups]
    raise RuntimeError(f"Flexiv joint group '{requested_group_name}' not found. Available groups: {available}")


class FlexivCollector:
    """Execute joint trajectories on a Flexiv robot and log measured states."""

    def __init__(
        self,
        robot_sn,
        joint_names,
        joint_group=None,
        dry_run=False,
        max_velocity=0.05,
        max_acceleration=0.1,
        home_plan=None,
    ):
        self.robot_sn = robot_sn
        self.joint_names = list(joint_names)
        self.joint_group_name = joint_group
        self.dry_run = dry_run
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.home_plan = home_plan
        self.robot = None
        self.group = None
        self.start_monotonic = None
        self.last_command = np.zeros(len(self.joint_names), dtype=np.float64)
        self.collected_data = self._empty_data()

        if self.dry_run:
            print("[Flexiv Collector] Dry run enabled; no hardware commands will be sent.")
            return

        if not FLEXIV_AVAILABLE:
            raise RuntimeError("flexivrdk is not installed. Install with: pip install flexivrdk")
        if not robot_sn:
            raise RuntimeError("--flexiv-robot-sn is required unless --flexiv-dry-run is set")

        self.robot = flexivrdk.Robot(robot_sn)
        self._prepare_robot()

    def _empty_data(self):
        return {
            "time": [],
            "positions": [],
            "velocities": [],
            "torques": [],
        }

    def _prepare_robot(self):
        """Clear faults, enable, optionally home, and switch to NRT joint position."""
        if self.robot.fault():
            print("[Flexiv Collector] Fault detected; attempting ClearFault()")
            if not self.robot.ClearFault():
                raise RuntimeError("Flexiv fault cannot be cleared")

        print("[Flexiv Collector] Enabling robot")
        self.robot.Enable()
        while not self.robot.operational():
            time.sleep(1.0)

        self.group = _select_group(self.robot, self.joint_group_name)
        print(f"[Flexiv Collector] Using joint group: {flexivrdk.kJointGroupNames[self.group]}")

        if self.home_plan:
            print(f"[Flexiv Collector] Executing home plan: {self.home_plan}")
            self.robot.SwitchMode(flexivrdk.Mode.NRT_PLAN_EXECUTION)
            self.robot.ExecutePlan(self.home_plan)
            while self.robot.busy():
                time.sleep(0.5)

        self.robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
        self.last_command, _, _ = self.read_state()

    def read_state(self):
        """Return link-side positions, link-side velocities, and measured torques."""
        if self.dry_run:
            return (
                self.last_command.copy(),
                np.zeros(len(self.joint_names), dtype=np.float64),
                np.zeros(len(self.joint_names), dtype=np.float64),
            )

        states = self.robot.states()[self.group]
        positions = np.array(states.q, dtype=np.float64)
        velocities = np.array(states.dq, dtype=np.float64)
        torques = np.array(states.tau, dtype=np.float64)
        return positions, velocities, torques

    def write_positions(self, positions, max_velocity=None, max_acceleration=None):
        """Send a non-real-time joint position command."""
        positions = np.array(positions, dtype=np.float64)
        self.last_command = positions.copy()

        if self.dry_run:
            return

        zero_vel = [0.0] * len(positions)
        max_velocity = self.max_velocity if max_velocity is None else max_velocity
        max_acceleration = self.max_acceleration if max_acceleration is None else max_acceleration
        max_vel = [max_velocity] * len(positions)
        max_acc = [max_acceleration] * len(positions)
        cmd = flexivrdk.NrtJointPositionCmd(positions.tolist(), zero_vel, max_vel, max_acc)
        self.robot.SendJointPosition({self.group: cmd})

    def move_to_position(
        self,
        target_positions,
        duration=20.0,
        control_freq=50,
        max_velocity=0.03,
        max_acceleration=0.05,
    ):
        """Move smoothly from current measured position to target position."""
        start_positions, _, _ = self.read_state()
        target_positions = np.array(target_positions, dtype=np.float64)
        steps = max(2, int(duration * control_freq))
        period = 1.0 / control_freq

        for i in range(steps):
            alpha = (i + 1) / steps
            interp = (1.0 - alpha) * start_positions + alpha * target_positions
            loop_start = time.monotonic()
            self.write_positions(interp, max_velocity=max_velocity, max_acceleration=max_acceleration)
            sleep_time = max(0.0, period - (time.monotonic() - loop_start))
            time.sleep(sleep_time)

    def safety_check(self, target_positions, max_diff_rad):
        """Prompt before large initial moves."""
        if self.dry_run:
            return True

        current_pos, _, _ = self.read_state()
        target_positions = np.array(target_positions, dtype=np.float64)
        diff = np.abs(target_positions - current_pos)
        max_diff = float(np.max(diff))

        print("\n=== FLEXIV SAFETY CHECK ===")
        print(f"{'Joint':<12} {'Current(rad)':>14} {'Target(rad)':>14} {'Diff(rad)':>12}")
        print("-" * 58)
        for name, cur, tgt, delta in zip(self.joint_names, current_pos, target_positions, diff):
            warning = " WARN" if delta > max_diff_rad else ""
            print(f"{name:<12} {cur:>14.4f} {tgt:>14.4f} {delta:>12.4f}{warning}")
        print("-" * 58)

        if max_diff <= max_diff_rad:
            return True

        print(f"Max initial move is {max_diff:.4f} rad, threshold is {max_diff_rad:.4f} rad.")
        response = input("Proceed with this Flexiv motion? [y/N]: ").strip().lower()
        return response == "y"

    def collect_motion(
        self,
        motion_seq,
        control_freq=50,
        slowdown_factor=1.0,
        auto_start=False,
        start_move_duration=20.0,
        start_max_velocity=0.03,
        start_max_acceleration=0.05,
        max_initial_diff_rad=0.5,
    ):
        """Execute the motion sequence and collect SAGE-compatible data."""
        if motion_seq.shape[1] != len(self.joint_names):
            raise ValueError(f"Motion has {motion_seq.shape[1]} joints but collector expects {len(self.joint_names)}")

        if not self.safety_check(motion_seq[0], max_initial_diff_rad):
            print("[Flexiv Collector] Motion cancelled by user.")
            return [], []

        n_frames = motion_seq.shape[0]
        loop_dt = (1.0 / control_freq) * slowdown_factor
        command_times = []
        command_positions = []
        self.collected_data = self._empty_data()

        print(
            "[Flexiv Collector] Moving to start position "
            f"(duration={start_move_duration}s, vmax={start_max_velocity}rad/s, amax={start_max_acceleration}rad/s^2)"
        )
        self.move_to_position(
            motion_seq[0],
            duration=start_move_duration,
            control_freq=control_freq,
            max_velocity=start_max_velocity,
            max_acceleration=start_max_acceleration,
        )
        time.sleep(0.5)

        print("\n=== READY TO START FLEXIV MOTION ===")
        print(f"Motion: {n_frames} frames at {control_freq}Hz (slowdown: {slowdown_factor}x)")
        if not auto_start:
            response = input("Press ENTER to start motion, or 'q' to quit: ").strip().lower()
            if response == "q":
                print("[Flexiv Collector] Motion cancelled by user.")
                return [], []

        print("[Flexiv Collector] Starting motion execution")
        self.start_monotonic = time.monotonic()

        for i in range(n_frames):
            loop_start = time.monotonic()
            t = loop_start - self.start_monotonic

            if not self.dry_run and self.robot.fault():
                raise RuntimeError("Fault occurred on the connected Flexiv robot")

            target_pos = motion_seq[i]
            self.write_positions(target_pos)
            command_times.append(t)
            command_positions.append(target_pos.copy())

            positions, velocities, torques = self.read_state()
            self.collected_data["time"].append(t)
            self.collected_data["positions"].append(positions)
            self.collected_data["velocities"].append(velocities)
            self.collected_data["torques"].append(torques)

            loop_elapsed = time.monotonic() - loop_start
            sleep_time = max(0.0, loop_dt - loop_elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

            if i % max(1, control_freq) == 0:
                print(f"  Frame {i}/{n_frames} | Loop: {loop_elapsed*1000:.1f}ms")

        print(f"[Flexiv Collector] Motion completed in {time.monotonic() - self.start_monotonic:.2f}s")
        return command_times, command_positions

    def close(self):
        """Stop the robot motion mode."""
        if self.dry_run or self.robot is None:
            return
        try:
            self.robot.Stop()
        except Exception as exc:
            print(f"[Flexiv Collector] Warning: robot.Stop() failed: {exc}")


def save_sage_format(output_dir, motion_name, joint_names, command_times, command_positions, collected_data):
    """Save collected Flexiv data in SAGE-compatible real robot format."""
    motion_dir = os.path.join(output_dir, motion_name)
    os.makedirs(motion_dir, exist_ok=True)

    joint_list_file = os.path.join(motion_dir, "joint_list.txt")
    with open(joint_list_file, "w") as f:
        for joint_name in joint_names:
            f.write(f"{joint_name}\n")

    control_file = os.path.join(motion_dir, "control.csv")
    with open(control_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["type", "timestamp", "positions"])
        for t, pos in zip(command_times, command_positions):
            writer.writerow(["CONTROL", t * 1e6, pos.tolist()])

    event_file = os.path.join(motion_dir, "event.csv")
    with open(event_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["type", "timestamp", "event"])
        if len(command_times) > 0:
            writer.writerow(["EVENT", 0, "CONTROL_MODE_ENABLE"])
            writer.writerow(["EVENT", command_times[0] * 1e6, "MOTION_START"])
            writer.writerow(["EVENT", command_times[-1] * 1e6, "MOTION_END"])

    state_file = os.path.join(motion_dir, "state_motor.csv")
    with open(state_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["type", "timestamp", "positions", "velocities", "torques"])
        for i, t in enumerate(collected_data["time"]):
            writer.writerow(
                [
                    "STATE_MOTOR",
                    t * 1e6,
                    collected_data["positions"][i].tolist(),
                    collected_data["velocities"][i].tolist(),
                    collected_data["torques"][i].tolist(),
                ]
            )

    print(f"[Flexiv Collector] Data saved to: {motion_dir}")


def flexiv_collector_main(
    motion_file,
    output_dir,
    robot_sn=None,
    joint_group=None,
    control_freq=50,
    slowdown_factor=1.0,
    auto_start=False,
    motion_name=None,
    dry_run=False,
    max_velocity=0.05,
    max_acceleration=0.1,
    home_plan=None,
    start_move_duration=20.0,
    start_max_velocity=0.03,
    start_max_acceleration=0.05,
    motion_scale=1.0,
    max_initial_diff_rad=0.5,
):
    """Run a motion on Flexiv and collect SAGE-format data."""
    print(f"[Flexiv Collector] Loading motion: {motion_file}")
    seq, joint_names, motion_freq, file_motion_name = load_motion_from_txt(motion_file)
    motion_name = motion_name if motion_name else file_motion_name
    print(f"  Motion: {motion_name}, {seq.shape[0]} frames, {len(joint_names)} joints")

    if motion_freq != control_freq:
        print(f"  Resampling from {motion_freq}Hz to {control_freq}Hz")
        seq = interpolate_motion(seq, motion_freq, control_freq)

    if motion_scale != 1.0:
        if motion_scale < 0.0:
            raise ValueError("motion_scale must be non-negative")
        print(f"  Scaling motion deviations from first waypoint by {motion_scale}")
        seq = seq[0] + motion_scale * (seq - seq[0])

    collector = FlexivCollector(
        robot_sn=robot_sn,
        joint_names=joint_names or DEFAULT_FLEXIV_JOINT_NAMES,
        joint_group=joint_group,
        dry_run=dry_run,
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
        home_plan=home_plan,
    )

    try:
        command_times, command_positions = collector.collect_motion(
            seq,
            control_freq=control_freq,
            slowdown_factor=slowdown_factor,
            auto_start=auto_start,
            start_move_duration=start_move_duration,
            start_max_velocity=start_max_velocity,
            start_max_acceleration=start_max_acceleration,
            max_initial_diff_rad=max_initial_diff_rad,
        )
        save_sage_format(
            output_dir=output_dir,
            motion_name=motion_name,
            joint_names=joint_names,
            command_times=command_times,
            command_positions=command_positions,
            collected_data=collector.collected_data,
        )
        print("[Flexiv Collector] Collection complete")
    finally:
        collector.close()
