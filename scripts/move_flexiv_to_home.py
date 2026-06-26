#!/usr/bin/env python3
"""Slowly move a Flexiv arm to an explicit joint-space home pose.

This is intentionally separate from torque SysID collection. Use it to put the
robot at a known pose, then run the torque collector with --no-home-plan so the
current pose is recorded as home.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Iterable

import flexivrdk


def parse_joint_vector(value: str) -> list[float]:
    parts = [item.strip() for item in value.split(",") if item.strip()]
    if not parts:
        raise argparse.ArgumentTypeError("expected comma-separated joint values")
    try:
        return [float(item) for item in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def max_abs(values: Iterable[float]) -> float:
    return max(abs(value) for value in values)


def format_row(values: Iterable[float]) -> str:
    return ",".join(f"{value:.9f}" for value in values)


def resolve_group(robot, requested: str | None):
    if hasattr(robot, "groups") and hasattr(flexivrdk, "kJointGroupNames"):
        groups = robot.groups()
        if not groups:
            return None, "FULL_SYSTEM"
        if requested is None:
            group = groups[0]
            return group, flexivrdk.kJointGroupNames[group]
        requested_upper = requested.upper()
        for group in groups:
            name = flexivrdk.kJointGroupNames[group]
            if name.upper() == requested_upper:
                return group, name
        available = ", ".join(flexivrdk.kJointGroupNames[group] for group in groups)
        raise RuntimeError(f"Joint group {requested!r} not found. Available: {available}")
    return None, requested or "FULL_SYSTEM"


def read_state(robot, group):
    states = robot.states()
    if group is not None:
        states = states[group]
    return list(states.q), list(states.dq)


def send_joint_position(robot, group, positions, max_velocity, max_acceleration):
    zero_vel = [0.0] * len(positions)
    max_vel = [max_velocity] * len(positions)
    max_acc = [max_acceleration] * len(positions)
    if group is not None and hasattr(flexivrdk, "NrtJointPositionCmd"):
        cmd = flexivrdk.NrtJointPositionCmd(positions, zero_vel, max_vel, max_acc)
        robot.SendJointPosition({group: cmd})
    else:
        robot.SendJointPosition(positions, zero_vel, max_vel, max_acc)


def switch_idle(robot):
    try:
        robot.SwitchMode(flexivrdk.Mode.IDLE)
    except Exception:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Slowly move Flexiv to a joint-space home pose.")
    parser.add_argument("--robot-sn", required=True, help="Flexiv robot serial number")
    parser.add_argument("--joint-group", default="ARMS", help="Joint group for RDK v2; ignored by RDK v1.8")
    parser.add_argument(
        "--target-q",
        type=parse_joint_vector,
        default=None,
        help="Comma-separated target joint positions in rad. Default: all zeros.",
    )
    parser.add_argument("--max-velocity", type=float, default=0.02, help="NRT max joint velocity in rad/s")
    parser.add_argument("--max-acceleration", type=float, default=0.04, help="NRT max joint acceleration in rad/s^2")
    parser.add_argument("--timeout-s", type=float, default=240.0, help="Move timeout")
    parser.add_argument("--tolerance-deg", type=float, default=2.0, help="Allowed max joint error at finish")
    parser.add_argument("--progress-period-s", type=float, default=1.0, help="Progress print period")
    parser.add_argument("--auto-start", action="store_true", help="Do not ask for confirmation")
    args = parser.parse_args()

    if args.max_velocity <= 0 or args.max_acceleration <= 0 or args.timeout_s <= 0:
        raise ValueError("velocity, acceleration, and timeout must be positive")

    print("[Flexiv Home] Connecting; no motion command sent yet.")
    robot = flexivrdk.Robot(args.robot_sn)
    group, group_name = resolve_group(robot, args.joint_group)
    current_q, current_dq = read_state(robot, group)
    target_q = args.target_q if args.target_q is not None else [0.0] * len(current_q)
    if len(target_q) != len(current_q):
        raise ValueError(f"target has {len(target_q)} joints, robot state has {len(current_q)}")

    diff = [target - current for target, current in zip(target_q, current_q)]
    print(f"[Flexiv Home] Joint group: {group_name}")
    print("joint1,joint2,joint3,joint4,joint5,joint6,joint7")
    print("current_q," + format_row(current_q))
    print("target_q," + format_row(target_q))
    print("diff_rad," + format_row(diff))
    print(f"[Flexiv Home] Max move: {max_abs(diff):.6f} rad ({math.degrees(max_abs(diff)):.2f} deg)")
    print(f"[Flexiv Home] Limits: vmax={args.max_velocity} rad/s, amax={args.max_acceleration} rad/s^2")
    print(f"[Flexiv Home] Timeout: {args.timeout_s}s, tolerance={args.tolerance_deg} deg")

    if not args.auto_start:
        response = input("Proceed with slow joint-space move? [y/N]: ").strip().lower()
        if response != "y":
            print("Cancelled.")
            return

    if robot.fault():
        print("[Flexiv Home] Fault detected; attempting ClearFault().")
        if not robot.ClearFault():
            raise RuntimeError("Flexiv fault cannot be cleared")

    print("[Flexiv Home] Enabling robot.")
    robot.Enable()
    while not robot.operational():
        time.sleep(1.0)

    print("[Flexiv Home] Switching to NRT_JOINT_POSITION and sending target.")
    robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
    send_joint_position(robot, group, target_q, args.max_velocity, args.max_acceleration)

    tolerance_rad = math.radians(args.tolerance_deg)
    start = time.monotonic()
    last_print = start - args.progress_period_s
    reached = False
    last_error = float("inf")
    try:
        while time.monotonic() - start < args.timeout_s:
            q, dq = read_state(robot, group)
            err = [target - value for target, value in zip(target_q, q)]
            last_error = max_abs(err)
            now = time.monotonic()
            if now - last_print >= args.progress_period_s:
                print(
                    f"[Flexiv Home] t={now - start:.1f}s "
                    f"max_err={last_error:.6f} rad ({math.degrees(last_error):.2f} deg) "
                    f"max_vel={max_abs(dq):.4f} rad/s"
                )
                last_print = now
            if last_error <= tolerance_rad:
                reached = True
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[Flexiv Home] Interrupted by user; switching to IDLE.")
        switch_idle(robot)
        raise

    if not reached:
        switch_idle(robot)
        raise RuntimeError(
            f"Timed out {args.timeout_s}s from target; last max error "
            f"{last_error:.6f} rad ({math.degrees(last_error):.2f} deg)"
        )

    print(f"[Flexiv Home] Reached target within {args.tolerance_deg} deg tolerance.")
    print("[Flexiv Home] Final q:")
    final_q, _ = read_state(robot, group)
    print(format_row(final_q))
    switch_idle(robot)


if __name__ == "__main__":
    main()
