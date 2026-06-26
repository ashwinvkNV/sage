#!/usr/bin/env python3
"""Build and run the Flexiv C++ RT joint-torque SysID collector."""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run(cmd: list[str], *, dry_run: bool = False, env: dict[str, str] | None = None) -> None:
    print("+ " + " ".join(cmd))
    if not dry_run:
        subprocess.run(cmd, check=True, env=env)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build and run Flexiv RT joint-torque SysID collection.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--robot-sn", required=True, help="Flexiv robot serial number, e.g. Rizon4s-123456")
    parser.add_argument("--output-folder", default="output", help="SAGE output root")
    parser.add_argument("--name", default="rt_joint_torque_sysid", help="Dataset folder name")
    parser.add_argument("--joint-group", default=None, help="Flexiv joint group, e.g. ARMS")
    parser.add_argument("--home-plan", default="", help="Optional saved plan used before each torque trial")
    parser.add_argument("--home-zero", action="store_true", help="Move slowly to all-zero joints and use exact zero as home_q")
    parser.add_argument("--no-home-plan", action="store_true", help="Reset with slow NRT joint position instead")
    parser.add_argument("--joints", default="all", help="all, or comma list like joint1,joint2 or 1,2")
    parser.add_argument("--max-torque-nm", type=float, default=0.5, help="Peak torque on the active joint")
    parser.add_argument("--offset-limit-deg", type=float, default=10.0, help="Active joint stop offset from home")
    parser.add_argument(
        "--passive-offset-limit-deg",
        type=float,
        default=3.0,
        help="Stop if any inactive joint moves this far from home",
    )
    parser.add_argument("--velocity-limit-rad-s", type=float, default=0.25, help="Stop if any joint exceeds this")
    parser.add_argument("--trial-timeout-s", type=float, default=8.0, help="Max duration for each sign/joint trial")
    parser.add_argument("--ramp-duration-s", type=float, default=2.0, help="Half-cosine torque ramp duration")
    parser.add_argument("--settle-s", type=float, default=1.0, help="Settle time after each reset")
    parser.add_argument("--reset-dq-max", type=float, default=0.02, help="NRT reset max velocity without home plan")
    parser.add_argument("--reset-ddq-max", type=float, default=0.04, help="NRT reset max acceleration without home plan")
    parser.add_argument("--reset-timeout-s", type=float, default=240.0, help="NRT reset timeout")
    parser.add_argument("--reset-tolerance-deg", type=float, default=0.5, help="Allowed reset error before torque trial")
    parser.add_argument("--rdk-prefix", default=None, help="CMAKE_PREFIX_PATH for a source-built Flexiv RDK install")
    parser.add_argument("--build-dir", default=None, help="CMake build directory")
    parser.add_argument("--no-build", action="store_true", help="Skip CMake configure/build")
    parser.add_argument("--auto-start", action="store_true", help="Do not ask for confirmation in C++ collector")
    parser.add_argument("--disable-gravity-comp", action="store_true", help="Disable gravity compensation in torque cmd")
    parser.add_argument("--disable-soft-limits", action="store_true", help="Disable soft limits in torque cmd")
    parser.add_argument("--dry-run", action="store_true", help="Print build/run commands without executing")
    args = parser.parse_args()

    root = repo_root()
    source_dir = root / "tools" / "flexiv_rt_torque_collector"
    build_dir = Path(args.build_dir) if args.build_dir else root / "build" / "flexiv_rt_torque_collector"
    binary = build_dir / "flexiv_rt_torque_sysid"
    output_dir = root / args.output_folder / "real" / "flexiv" / "torque_sysid" / args.name

    env = os.environ.copy()
    if args.rdk_prefix:
        env["CMAKE_PREFIX_PATH"] = args.rdk_prefix

    if not args.no_build:
        cmake_configure = ["cmake", "-S", str(source_dir), "-B", str(build_dir), "-DCMAKE_BUILD_TYPE=Release"]
        if args.rdk_prefix:
            cmake_configure.append(f"-DCMAKE_PREFIX_PATH={args.rdk_prefix}")
        run(cmake_configure, dry_run=args.dry_run, env=env)
        run(["cmake", "--build", str(build_dir), "--config", "Release", "-j", "4"], dry_run=args.dry_run, env=env)

    collect_cmd = [
        str(binary),
        "--robot-sn",
        args.robot_sn,
        "--output-dir",
        str(output_dir),
        "--joints",
        args.joints,
        "--max-torque-nm",
        str(args.max_torque_nm),
        "--offset-limit-deg",
        str(args.offset_limit_deg),
        "--passive-offset-limit-deg",
        str(args.passive_offset_limit_deg),
        "--velocity-limit-rad-s",
        str(args.velocity_limit_rad_s),
        "--trial-timeout-s",
        str(args.trial_timeout_s),
        "--ramp-duration-s",
        str(args.ramp_duration_s),
        "--settle-s",
        str(args.settle_s),
        "--reset-dq-max",
        str(args.reset_dq_max),
        "--reset-ddq-max",
        str(args.reset_ddq_max),
        "--reset-timeout-s",
        str(args.reset_timeout_s),
        "--reset-tolerance-deg",
        str(args.reset_tolerance_deg),
    ]
    if args.joint_group:
        collect_cmd += ["--joint-group", args.joint_group]
    if args.home_plan:
        collect_cmd += ["--home-plan", args.home_plan]
    if args.home_zero:
        collect_cmd.append("--home-zero")
    if args.no_home_plan:
        collect_cmd.append("--no-home-plan")
    if args.auto_start:
        collect_cmd.append("--auto-start")
    if args.disable_gravity_comp:
        collect_cmd.append("--disable-gravity-comp")
    if args.disable_soft_limits:
        collect_cmd.append("--disable-soft-limits")

    run(collect_cmd, dry_run=args.dry_run, env=env)
    print(f"\nOutput dataset: {output_dir}")


if __name__ == "__main__":
    main()
