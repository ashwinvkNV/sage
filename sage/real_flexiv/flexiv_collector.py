# Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Flexiv motion collector for SAGE-compatible real robot data.

The hardware path uses Flexiv RDK non-real-time joint impedance control by default:

    robot = flexivrdk.Robot(robot_sn)
    robot.ClearFault()
    robot.Enable()
    robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_IMPEDANCE)
    robot.SetJointImpedance(K_q, Z_q)
    robot.SendJointPosition(q_d, dq_d, dq_max, ddq_max)

Collected data is written as control.csv, state_motor.csv, event.csv,
timing.csv, and joint_list.txt for the SAGE / IsaacLab-Newton SysID flow.
"""

from __future__ import annotations

import csv
import json
import math
import os
import time
from bisect import bisect_right
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


FLEXIV_CONTROL_MODES = {
    "nrt_joint_impedance": "NRT_JOINT_IMPEDANCE",
    "nrt_joint_position": "NRT_JOINT_POSITION",
}

FLEXIV_UNLIMITED_MOTION = float("inf")
DEFAULT_UNLIMITED_ACCEL_SCALE = 100.0


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


def _resolve_joint_group(robot, requested_group_name):
    """Resolve a Flexiv joint group handle, or None for single-group RDK APIs."""
    if hasattr(robot, "groups") and hasattr(flexivrdk, "kJointGroupNames"):
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
        raise RuntimeError(
            f"Flexiv joint group '{requested_group_name}' not found. Available groups: {available}"
        )

    if requested_group_name is not None:
        requested = requested_group_name.upper()
        if requested not in {"ARMS", "ARM"}:
            print(
                "[Flexiv Collector] Warning: "
                f"joint group '{requested_group_name}' is not supported by this RDK version; "
                "controlling all robot joints"
            )
    return None


def is_unlimited_motion_limit(value):
    return value is None or math.isinf(value)


class FlexivCollector:
    """Execute joint trajectories on a Flexiv robot and log measured states."""

    def __init__(
        self,
        robot_sn,
        joint_names,
        joint_group=None,
        dry_run=False,
        max_velocity=2.0,
        max_acceleration=3.0,
        home_plan=None,
        control_mode="nrt_joint_impedance",
        stiffness_scale=1.0,
        damping_ratio=0.7,
    ):
        self.robot_sn = robot_sn
        self.joint_names = list(joint_names)
        self.joint_group_name = joint_group
        self.dry_run = dry_run
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.home_plan = home_plan
        self.control_mode = control_mode
        self.stiffness_scale = stiffness_scale
        self.damping_ratio = damping_ratio
        self.robot = None
        self.group = None
        self.start_monotonic = None
        self.last_command = np.zeros(len(self.joint_names), dtype=np.float64)
        self.collected_data = self._empty_data()
        self.timing_data = self._empty_timing_data()
        self.nominal_stiffness = None
        self.applied_stiffness = None
        self.applied_damping_ratio = None
        self.robot_dq_max = None

        if self.control_mode not in FLEXIV_CONTROL_MODES:
            valid = ", ".join(sorted(FLEXIV_CONTROL_MODES))
            raise ValueError(f"Unsupported Flexiv control mode '{self.control_mode}'. Valid modes: {valid}")
        if self.stiffness_scale < 0.0 or self.stiffness_scale > 1.0:
            raise ValueError("--flexiv-stiffness-scale must be in [0.0, 1.0]")
        if self.damping_ratio < 0.3 or self.damping_ratio > 0.8:
            raise ValueError("--flexiv-damping-ratio must be in the RDK-supported [0.3, 0.8] range")

        if self.dry_run:
            print("[Flexiv Collector] Dry run enabled; no hardware commands will be sent.")
            if self.control_mode == "nrt_joint_impedance":
                self.applied_stiffness = [self.stiffness_scale] * len(self.joint_names)
                self.applied_damping_ratio = [self.damping_ratio] * len(self.joint_names)
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

    def _empty_timing_data(self):
        return {
            "frame": [],
            "loop_start_s": [],
            "command_send_start_s": [],
            "command_send_end_s": [],
            "state_read_start_s": [],
            "state_read_end_s": [],
            "loop_work_s": [],
        }

    def _prepare_robot(self):
        """Clear faults, enable, optionally home, and switch to the requested joint mode."""
        if self.robot.fault():
            print("[Flexiv Collector] Fault detected; attempting ClearFault()")
            if not self.robot.ClearFault():
                raise RuntimeError("Flexiv fault cannot be cleared")

        print("[Flexiv Collector] Enabling robot")
        self.robot.Enable()
        while not self.robot.operational():
            time.sleep(1.0)

        self.group = _resolve_joint_group(self.robot, self.joint_group_name)
        if self.group is not None:
            print(f"[Flexiv Collector] Using joint group: {flexivrdk.kJointGroupNames[self.group]}")
        else:
            print(f"[Flexiv Collector] Using all {self.robot.info().DoF} robot joints")

        if self.home_plan:
            print(f"[Flexiv Collector] Executing home plan: {self.home_plan}")
            self.robot.SwitchMode(flexivrdk.Mode.NRT_PLAN_EXECUTION)
            self.robot.ExecutePlan(self.home_plan)
            while self.robot.busy():
                time.sleep(0.5)

        rdk_mode = getattr(flexivrdk.Mode, FLEXIV_CONTROL_MODES[self.control_mode])
        self.robot.SwitchMode(rdk_mode)
        print(f"[Flexiv Collector] Switched to mode: {FLEXIV_CONTROL_MODES[self.control_mode]}")

        if self.control_mode == "nrt_joint_impedance":
            self._apply_joint_impedance()

        self.robot_dq_max = np.array(self.robot.info().dq_max, dtype=np.float64)
        if is_unlimited_motion_limit(self.max_velocity) or is_unlimited_motion_limit(self.max_acceleration):
            max_vel, max_acc = self._resolve_command_limits(self.max_velocity, self.max_acceleration, len(self.joint_names))
            print("[Flexiv Collector] Using robot software motion limits:")
            print(f"  dq_max (rad/s): {max_vel}")
            print(f"  ddq_max (rad/s^2): {max_acc}")
        self.last_command, _, _ = self.read_state()

    def _resolve_command_limits(self, max_velocity, max_acceleration, n_joints):
        """Map unlimited limits to per-joint robot software maximums."""
        if self.robot_dq_max is not None and len(self.robot_dq_max) >= n_joints:
            robot_dq_max = self.robot_dq_max[:n_joints]
        else:
            robot_dq_max = np.full(n_joints, 2.0, dtype=np.float64)

        if is_unlimited_motion_limit(max_velocity):
            max_vel = robot_dq_max.tolist()
        elif np.isscalar(max_velocity):
            max_vel = [float(max_velocity)] * n_joints
        else:
            max_vel = list(max_velocity)

        if is_unlimited_motion_limit(max_acceleration):
            max_acc = (robot_dq_max * DEFAULT_UNLIMITED_ACCEL_SCALE).tolist()
        elif np.isscalar(max_acceleration):
            max_acc = [float(max_acceleration)] * n_joints
        else:
            max_acc = list(max_acceleration)

        return max_vel, max_acc

    def _apply_joint_impedance(self):
        """Apply scaled nominal joint stiffness and uniform damping ratio."""
        nominal = np.array(self.robot.info().K_q_nom, dtype=np.float64)
        stiffness = (self.stiffness_scale * nominal).tolist()
        damping = [self.damping_ratio] * len(stiffness)
        if self.group is not None:
            self.robot.SetJointImpedance(self.group, stiffness, damping)
        else:
            self.robot.SetJointImpedance(stiffness, damping)
        self.nominal_stiffness = nominal.tolist()
        self.applied_stiffness = stiffness
        self.applied_damping_ratio = damping
        print(
            "[Flexiv Collector] Joint impedance set "
            f"(stiffness_scale={self.stiffness_scale:g}, damping_ratio={self.damping_ratio:g})"
        )
        print(f"[Flexiv Collector] K_q: {stiffness}")
        print(f"[Flexiv Collector] Z_q: {damping}")

    def read_state(self):
        """Return link-side positions, link-side velocities, and measured torques."""
        if self.dry_run:
            return (
                self.last_command.copy(),
                np.zeros(len(self.joint_names), dtype=np.float64),
                np.zeros(len(self.joint_names), dtype=np.float64),
            )

        states = self.robot.states()[self.group] if self.group is not None else self.robot.states()
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
        max_vel, max_acc = self._resolve_command_limits(max_velocity, max_acceleration, len(positions))
        if self.group is not None:
            cmd = flexivrdk.NrtJointPositionCmd(positions.tolist(), zero_vel, max_vel, max_acc)
            self.robot.SendJointPosition({self.group: cmd})
        else:
            self.robot.SendJointPosition(positions.tolist(), zero_vel, max_vel, max_acc)

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
        center_on_current_pose=False,
    ):
        """Execute the motion sequence and collect SAGE-compatible data."""
        if motion_seq.shape[1] != len(self.joint_names):
            raise ValueError(f"Motion has {motion_seq.shape[1]} joints but collector expects {len(self.joint_names)}")

        motion_seq = np.array(motion_seq, dtype=np.float64, copy=True)
        if center_on_current_pose:
            current_pos, _, _ = self.read_state()
            offset = current_pos - motion_seq[0]
            motion_seq = motion_seq + offset
            print("[Flexiv Collector] Centered motion on current pose")
            print(f"  Base offset (rad): {offset.tolist()}")

        if not self.safety_check(motion_seq[0], max_initial_diff_rad):
            print("[Flexiv Collector] Motion cancelled by user.")
            return [], []

        n_frames = motion_seq.shape[0]
        loop_dt = (1.0 / control_freq) * slowdown_factor
        command_times = []
        command_positions = []
        self.collected_data = self._empty_data()
        self.timing_data = self._empty_timing_data()

        current_pos, _, _ = self.read_state()
        start_diff = float(np.max(np.abs(motion_seq[0] - current_pos)))
        if start_diff > 1e-4:
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
        else:
            print("[Flexiv Collector] Already at start position, skipping initial move")

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
            loop_start_s = loop_start - self.start_monotonic

            if not self.dry_run and self.robot.fault():
                raise RuntimeError("Fault occurred on the connected Flexiv robot")

            target_pos = motion_seq[i]
            command_send_start_s = time.monotonic() - self.start_monotonic
            self.write_positions(target_pos)
            command_send_end_s = time.monotonic() - self.start_monotonic
            command_times.append(command_send_start_s)
            command_positions.append(target_pos.copy())

            state_read_start_s = time.monotonic() - self.start_monotonic
            positions, velocities, torques = self.read_state()
            state_read_end_s = time.monotonic() - self.start_monotonic
            self.collected_data["time"].append(state_read_end_s)
            self.collected_data["positions"].append(positions)
            self.collected_data["velocities"].append(velocities)
            self.collected_data["torques"].append(torques)

            loop_elapsed = time.monotonic() - loop_start
            self.timing_data["frame"].append(i)
            self.timing_data["loop_start_s"].append(loop_start_s)
            self.timing_data["command_send_start_s"].append(command_send_start_s)
            self.timing_data["command_send_end_s"].append(command_send_end_s)
            self.timing_data["state_read_start_s"].append(state_read_start_s)
            self.timing_data["state_read_end_s"].append(state_read_end_s)
            self.timing_data["loop_work_s"].append(loop_elapsed)
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


def save_sage_format(
    output_dir,
    motion_name,
    joint_names,
    command_times,
    command_positions,
    collected_data,
    timing_data=None,
    metadata=None,
):
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

    response_rows = []
    command_times_list = list(command_times)
    for i, state_time in enumerate(collected_data["time"]):
        if not command_times_list:
            break
        command_idx = max(0, bisect_right(command_times_list, state_time) - 1)
        command_time = command_times_list[command_idx]
        command_pos = np.array(command_positions[command_idx], dtype=np.float64)
        measured_pos = np.array(collected_data["positions"][i], dtype=np.float64)
        measured_vel = np.array(collected_data["velocities"][i], dtype=np.float64)
        measured_torque = np.array(collected_data["torques"][i], dtype=np.float64)
        position_error = command_pos - measured_pos
        response_rows.append(
            {
                "feedback_time": state_time,
                "command_time": command_time,
                "command_age_s": state_time - command_time,
                "command_positions": command_pos,
                "measured_positions": measured_pos,
                "measured_velocities": measured_vel,
                "measured_torques": measured_torque,
                "position_error": position_error,
            }
        )

    response_file = os.path.join(motion_dir, "command_response.csv")
    with open(response_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "type",
                "feedback_timestamp",
                "command_timestamp",
                "command_age_s",
                "command_positions",
                "measured_positions",
                "measured_velocities",
                "measured_torques",
                "position_error",
            ]
        )
        for row in response_rows:
            writer.writerow(
                [
                    "COMMAND_RESPONSE",
                    row["feedback_time"] * 1e6,
                    row["command_time"] * 1e6,
                    row["command_age_s"],
                    row["command_positions"].tolist(),
                    row["measured_positions"].tolist(),
                    row["measured_velocities"].tolist(),
                    row["measured_torques"].tolist(),
                    row["position_error"].tolist(),
                ]
            )

    response_long_file = os.path.join(motion_dir, "command_response_long.csv")
    with open(response_long_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "type",
                "feedback_timestamp",
                "command_timestamp",
                "command_age_s",
                "joint",
                "joint_index",
                "command_position",
                "measured_position",
                "measured_velocity",
                "measured_torque",
                "position_error",
            ]
        )
        for row in response_rows:
            for joint_idx, joint_name in enumerate(joint_names):
                writer.writerow(
                    [
                        "COMMAND_RESPONSE_JOINT",
                        row["feedback_time"] * 1e6,
                        row["command_time"] * 1e6,
                        row["command_age_s"],
                        joint_name,
                        joint_idx,
                        row["command_positions"][joint_idx],
                        row["measured_positions"][joint_idx],
                        row["measured_velocities"][joint_idx],
                        row["measured_torques"][joint_idx],
                        row["position_error"][joint_idx],
                    ]
                )

    if timing_data:
        timing_file = os.path.join(motion_dir, "timing.csv")
        timing_fields = [
            "frame",
            "loop_start_s",
            "command_send_start_s",
            "command_send_end_s",
            "state_read_start_s",
            "state_read_end_s",
            "loop_work_s",
        ]
        with open(timing_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(timing_fields)
            for i in range(len(timing_data["frame"])):
                writer.writerow([timing_data[field][i] for field in timing_fields])

    if metadata:
        metadata_file = os.path.join(motion_dir, "metadata.json")
        with open(metadata_file, "w") as f:
            json.dump(metadata, f, indent=2, sort_keys=True)

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
    max_velocity=2.0,
    max_acceleration=3.0,
    home_plan=None,
    start_move_duration=20.0,
    start_max_velocity=0.03,
    start_max_acceleration=0.05,
    motion_scale=1.0,
    max_initial_diff_rad=0.5,
    center_on_current_pose=False,
    control_mode="nrt_joint_impedance",
    stiffness_scale=1.0,
    damping_ratio=0.7,
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
        control_mode=control_mode,
        stiffness_scale=stiffness_scale,
        damping_ratio=damping_ratio,
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
            center_on_current_pose=center_on_current_pose,
        )
        save_sage_format(
            output_dir=output_dir,
            motion_name=motion_name,
            joint_names=joint_names,
            command_times=command_times,
            command_positions=command_positions,
            collected_data=collector.collected_data,
            timing_data=collector.timing_data,
            metadata={
                "robot": "flexiv",
                "control_mode": control_mode,
                "stiffness_scale": stiffness_scale,
                "damping_ratio": damping_ratio,
                "nominal_stiffness": collector.nominal_stiffness,
                "applied_stiffness": collector.applied_stiffness,
                "applied_damping_ratio": collector.applied_damping_ratio,
                "max_velocity": max_velocity,
                "max_acceleration": max_acceleration,
                "command_velocity_limit_rad_s": max_velocity,
                "command_acceleration_limit_rad_s2": max_acceleration,
                "start_move_velocity_limit_rad_s": start_max_velocity,
                "start_move_acceleration_limit_rad_s2": start_max_acceleration,
                "motion_scale": motion_scale,
                "center_on_current_pose": center_on_current_pose,
                "control_freq": control_freq,
                "slowdown_factor": slowdown_factor,
                "timestamp_policy": {
                    "control_csv_timestamp": "command_send_start_s",
                    "state_motor_csv_timestamp": "state_read_end_s",
                    "command_response_alignment": (
                        "latest command_send_start_s at or before state_read_end_s"
                    ),
                    "timing_csv_units": "seconds_from_motion_start",
                },
            },
        )
        print("[Flexiv Collector] Collection complete")
    finally:
        collector.close()
