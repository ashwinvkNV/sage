#!/usr/bin/env python3
"""Build and run the Flexiv C++ RT zero-torque drift collector."""

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
        description="Build and run Flexiv RT zero user-torque drift collection.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--robot-sn", required=True, help="Flexiv robot serial number")
    parser.add_argument("--output-folder", default="output", help="SAGE output root")
    parser.add_argument("--name", default="rt_zero_torque_drift", help="Dataset folder name")
    parser.add_argument("--joint-group", default="ARMS", help="Flexiv joint group label, e.g. ARMS")
    parser.add_argument("--home-zero", action="store_true", help="Move slowly to all-zero joints before drift test")
    parser.add_argument("--duration-s", type=float, default=10.0, help="Zero-torque stream duration")
    parser.add_argument("--sample-period-ms", type=float, default=1.0, help="Command/sample period")
    parser.add_argument("--drift-limit-deg", type=float, default=5.0, help="Stop if any joint drifts this far")
    parser.add_argument("--velocity-limit-rad-s", type=float, default=0.5, help="Stop if any joint exceeds this")
    parser.add_argument("--reset-dq-max", type=float, default=0.02, help="Home move max velocity")
    parser.add_argument("--reset-ddq-max", type=float, default=0.04, help="Home move max acceleration")
    parser.add_argument("--reset-timeout-s", type=float, default=240.0, help="Home move timeout")
    parser.add_argument("--reset-tolerance-deg", type=float, default=0.5, help="Home tolerance before test")
    parser.add_argument("--rdk-prefix", default=None, help="CMAKE_PREFIX_PATH for Flexiv RDK install")
    parser.add_argument("--build-dir", default=None, help="CMake build directory")
    parser.add_argument("--no-build", action="store_true", help="Skip CMake configure/build")
    parser.add_argument("--auto-start", action="store_true", help="Do not ask for confirmation")
    parser.add_argument("--disable-gravity-comp", action="store_true", help="Disable gravity compensation")
    parser.add_argument("--disable-soft-limits", action="store_true", help="Disable RDK soft limits")
    parser.add_argument("--dry-run", action="store_true", help="Print build/run commands without executing")
    args = parser.parse_args()

    root = repo_root()
    source_dir = root / "tools" / "flexiv_rt_torque_collector"
    build_dir = Path(args.build_dir) if args.build_dir else root / "build" / "flexiv_rt_torque_collector"
    binary = build_dir / "flexiv_zero_torque_drift"
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
        "--joint-group",
        args.joint_group,
        "--duration-s",
        str(args.duration_s),
        "--sample-period-ms",
        str(args.sample_period_ms),
        "--drift-limit-deg",
        str(args.drift_limit_deg),
        "--velocity-limit-rad-s",
        str(args.velocity_limit_rad_s),
        "--reset-dq-max",
        str(args.reset_dq_max),
        "--reset-ddq-max",
        str(args.reset_ddq_max),
        "--reset-timeout-s",
        str(args.reset_timeout_s),
        "--reset-tolerance-deg",
        str(args.reset_tolerance_deg),
    ]
    if args.home_zero:
        collect_cmd.append("--home-zero")
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
