# Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import argparse
import glob
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

REST_PERIOD_SECONDS = 2


def parse_motion_limit(value):
    if isinstance(value, str) and value.lower() in ("inf", "infinity"):
        return float("inf")
    return float(value)


def format_scalar_for_name(value):
    """Format a scalar for filesystem-safe Flexiv sweep suffixes."""
    return f"{value:g}".replace("-", "m").replace(".", "p")


def run_motion(
    robot_name,
    motion_file,
    output_dir,
    motion_name=None,
    auto_start=False,
    robot_port=None,
    robot_type=None,
    robot_id=None,
    flexiv_robot_sn=None,
    flexiv_joint_group=None,
    flexiv_dry_run=False,
    flexiv_control_freq=50,
    flexiv_slowdown_factor=1.0,
    flexiv_max_velocity=2.0,
    flexiv_max_acceleration=3.0,
    flexiv_home_plan=None,
    flexiv_start_move_duration=20.0,
    flexiv_start_max_velocity=0.03,
    flexiv_start_max_acceleration=0.05,
    flexiv_motion_scale=1.0,
    flexiv_max_initial_diff_rad=0.5,
    flexiv_center_on_current_pose=False,
    flexiv_control_mode="nrt_joint_impedance",
    flexiv_stiffness_scale=1.0,
    flexiv_damping_ratio=0.7,
):
    """Run a single motion file for the specified robot."""
    if robot_name == "h12" or robot_name == "g1":
        from sage.real_unitree.unitree_collector import unitree_collector_main

        unitree_collector_main(robot_name, motion_file, output_dir)
    elif robot_name == "realman":
        from sage.real_realman.realman_collector import realman_collector_main

        realman_collector_main(motion_file, output_dir)
    elif robot_name == "so101":
        from sage.real_so101.so101_lerobot_collector import so101_collector_main

        so101_collector_main(
            motion_file,
            output_dir,
            robot_port=robot_port,
            robot_type=robot_type,
            robot_id=robot_id,
            auto_start=auto_start,
            motion_name=motion_name,
        )
    elif robot_name == "flexiv":
        from sage.real_flexiv.flexiv_collector import flexiv_collector_main

        flexiv_collector_main(
            motion_file,
            output_dir,
            robot_sn=flexiv_robot_sn,
            joint_group=flexiv_joint_group,
            control_freq=flexiv_control_freq,
            slowdown_factor=flexiv_slowdown_factor,
            auto_start=auto_start,
            motion_name=motion_name,
            dry_run=flexiv_dry_run,
            max_velocity=flexiv_max_velocity,
            max_acceleration=flexiv_max_acceleration,
            home_plan=flexiv_home_plan,
            start_move_duration=flexiv_start_move_duration,
            start_max_velocity=flexiv_start_max_velocity,
            start_max_acceleration=flexiv_start_max_acceleration,
            motion_scale=flexiv_motion_scale,
            max_initial_diff_rad=flexiv_max_initial_diff_rad,
            center_on_current_pose=flexiv_center_on_current_pose,
            control_mode=flexiv_control_mode,
            stiffness_scale=flexiv_stiffness_scale,
            damping_ratio=flexiv_damping_ratio,
        )
    else:
        raise ValueError(f"Unknown robot name: {robot_name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run motion files on a robot and collect data.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run specific motion files:
  python scripts/run_real.py --robot-name so101 --motion-files custom/file1.txt custom/file2.txt --output-folder output

  # Run all motion files in a folder:
  python scripts/run_real.py --robot-name so101 --motion-folder custom --output-folder output --auto-start
        """,
    )
    parser.add_argument(
        "--robot-name",
        action="store",
        type=str,
        help="Robot name: realman, h12, g1, so101, or flexiv",
        required=True,
    )
    parser.add_argument(
        "--motion-files",
        action="store",
        type=str,
        nargs="+",
        help="One or more motion files (relative to motion_files/<robot>/)",
        default=[],
    )
    parser.add_argument(
        "--motion-folder",
        action="store",
        type=str,
        help="Run all .txt motion files in this folder (relative to motion_files/<robot>/)",
        default=None,
    )
    parser.add_argument(
        "--output-folder", action="store", type=str, help="Output folder for saving data", required=True
    )
    parser.add_argument(
        "--repeats", action="store", type=int, default=1, help="Number of times to repeat each motion (default: 1)"
    )
    parser.add_argument(
        "--auto-start",
        "-y",
        action="store_true",
        help="Automatically start motion without waiting for user confirmation",
    )
    # SO-101 specific arguments
    parser.add_argument(
        "--robot-port",
        type=str,
        default="/dev/ttyACM0",
        help="Serial port for SO-101 robot (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="so101_follower",
        help="Robot type for SO-101 (default: so101_follower)",
    )
    parser.add_argument(
        "--robot-id",
        type=str,
        default="my_awesome_follower_arm",
        help="Robot ID for SO-101 (default: my_awesome_follower_arm)",
    )
    # Flexiv specific arguments
    parser.add_argument(
        "--flexiv-robot-sn",
        type=str,
        default=None,
        help="Flexiv robot serial number, e.g. Rizon4s-123456",
    )
    parser.add_argument(
        "--flexiv-joint-group",
        type=str,
        default=None,
        help="Flexiv joint group to control (ARMS, ARM_1, or ARM_2). Defaults to the first available group.",
    )
    parser.add_argument(
        "--flexiv-dry-run",
        action="store_true",
        help="Generate Flexiv SAGE CSV output without connecting to hardware.",
    )
    parser.add_argument(
        "--flexiv-control-freq",
        type=int,
        default=50,
        help="Flexiv non-real-time joint command frequency in Hz, 1-100 recommended by RDK.",
    )
    parser.add_argument(
        "--flexiv-slowdown-factor",
        type=float,
        default=1.0,
        help="Slow down Flexiv playback by this factor.",
    )
    parser.add_argument(
        "--flexiv-max-velocity",
        type=parse_motion_limit,
        default=2.0,
        help="Flexiv SendJointPosition dq_max per joint in rad/s. Use 'inf' for robot dq_max.",
    )
    parser.add_argument(
        "--flexiv-max-acceleration",
        type=parse_motion_limit,
        default=3.0,
        help="Flexiv SendJointPosition ddq_max per joint in rad/s^2. Use 'inf' for aggressive limits.",
    )
    parser.add_argument(
        "--flexiv-home-plan",
        type=str,
        default=None,
        help="Optional Flexiv plan to execute before collection, e.g. PLAN-Home.",
    )
    parser.add_argument(
        "--flexiv-start-move-duration",
        type=float,
        default=20.0,
        help="Seconds used to move smoothly to the first Flexiv motion waypoint.",
    )
    parser.add_argument(
        "--flexiv-start-max-velocity",
        type=float,
        default=0.03,
        help="Flexiv dq_max used only while moving to the first waypoint, in rad/s.",
    )
    parser.add_argument(
        "--flexiv-start-max-acceleration",
        type=float,
        default=0.05,
        help="Flexiv ddq_max used only while moving to the first waypoint, in rad/s^2.",
    )
    parser.add_argument(
        "--flexiv-motion-scale",
        type=float,
        default=1.0,
        help="Scale motion deviations from the first waypoint. Use 0.1 for 10 percent amplitude.",
    )
    parser.add_argument(
        "--flexiv-max-initial-diff-rad",
        type=float,
        default=0.5,
        help="Prompt if any Flexiv joint must move farther than this to reach the first waypoint.",
    )
    parser.add_argument(
        "--flexiv-center-on-current-pose",
        action="store_true",
        help="Shift the motion so the first waypoint matches the robot's current pose.",
    )
    parser.add_argument(
        "--flexiv-control-mode",
        type=str,
        choices=["nrt_joint_impedance", "nrt_joint_position"],
        default="nrt_joint_impedance",
        help="Flexiv joint controller mode. Use nrt_joint_impedance to compare against joint impedance.",
    )
    parser.add_argument(
        "--flexiv-stiffness-scales",
        type=float,
        nargs="+",
        default=[1.0],
        help=(
            "One or more Flexiv joint impedance stiffness scales in [0, 1], applied to robot.info().K_q_nom. "
            "Multiple values create a sweep."
        ),
    )
    parser.add_argument(
        "--flexiv-damping-ratios",
        type=float,
        nargs="+",
        default=[0.7],
        help="One or more Flexiv joint impedance damping ratios in [0.3, 0.8]. Multiple values create a sweep.",
    )
    args = parser.parse_args()

    if args.robot_name == "flexiv":
        if not args.flexiv_dry_run and not args.flexiv_robot_sn:
            parser.error("--flexiv-robot-sn is required for Flexiv hardware collection unless --flexiv-dry-run is set")
        if args.flexiv_control_freq < 1 or args.flexiv_control_freq > 100:
            parser.error("--flexiv-control-freq must be in the RDK-supported 1-100 Hz range")
        for stiffness_scale in args.flexiv_stiffness_scales:
            if stiffness_scale < 0.0 or stiffness_scale > 1.0:
                parser.error("--flexiv-stiffness-scales values must be in [0.0, 1.0]")
        for damping_ratio in args.flexiv_damping_ratios:
            if damping_ratio < 0.3 or damping_ratio > 0.8:
                parser.error("--flexiv-damping-ratios values must be in the RDK-supported [0.3, 0.8] range")

    # Validate that at least one motion source is provided
    if not args.motion_files and not args.motion_folder:
        parser.error("You must provide either --motion-files or --motion-folder")

    home_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Collect motion files
    motion_files = list(args.motion_files)  # Start with explicitly listed files

    # Add files from folder if specified
    if args.motion_folder:
        folder_path = os.path.join(home_dir, "motion_files", args.robot_name, args.motion_folder)
        if not os.path.isdir(folder_path):
            print(f"Error: Motion folder not found: {folder_path}")
            exit(1)

        # Find all .txt files in the folder
        txt_files = sorted(glob.glob(os.path.join(folder_path, "*.txt")))

        if not txt_files:
            print(f"Warning: No .txt files found in {folder_path}")
        else:
            # Convert to relative paths (relative to motion_files/<robot>/)
            for txt_file in txt_files:
                rel_path = os.path.relpath(txt_file, os.path.join(home_dir, "motion_files", args.robot_name))
                if rel_path not in motion_files:  # Avoid duplicates
                    motion_files.append(rel_path)
            print(f"Found {len(txt_files)} motion files in {args.motion_folder}/")

    if not motion_files:
        print("Error: No motion files to run")
        exit(1)

    flexiv_impedance_settings = [(None, None)]
    if args.robot_name == "flexiv" and args.flexiv_control_mode == "nrt_joint_impedance":
        flexiv_impedance_settings = [
            (stiffness_scale, damping_ratio)
            for stiffness_scale in args.flexiv_stiffness_scales
            for damping_ratio in args.flexiv_damping_ratios
        ]

    total_runs = len(motion_files) * args.repeats * len(flexiv_impedance_settings)
    current_run = 0

    for motion_filename in motion_files:
        # Get motion name without extension for folder naming
        motion_name = os.path.splitext(os.path.basename(motion_filename))[0]

        # Extract output_subfolder from motion file path (e.g., "custom/motion.txt" -> "custom")
        output_subfolder = os.path.dirname(motion_filename) or "amass"

        # Build full motion file path
        motion_file = os.path.join(home_dir, "motion_files", args.robot_name, motion_filename)

        if not os.path.exists(motion_file):
            print(f"Warning: Motion file not found: {motion_file}")
            continue

        for stiffness_scale, damping_ratio in flexiv_impedance_settings:
            for run_idx in range(1, args.repeats + 1):
                current_run += 1

                # Output directory: output/real/<robot>/<source>/
                # (motion_name subdirectory created by collector's save function)
                output_dir = os.path.join(home_dir, args.output_folder, f"real/{args.robot_name}/{output_subfolder}")
                os.makedirs(output_dir, exist_ok=True)

                suffixes = []
                if args.repeats > 1:
                    suffixes.append(f"run_{run_idx}")
                if args.robot_name == "flexiv" and args.flexiv_control_mode == "nrt_joint_impedance":
                    k_name = format_scalar_for_name(stiffness_scale)
                    z_name = format_scalar_for_name(damping_ratio)
                    suffixes.append(f"k{k_name}_z{z_name}")

                effective_motion_name = motion_name
                if suffixes:
                    effective_motion_name = f"{motion_name}_{'_'.join(suffixes)}"

                print(f"\n{'='*60}")
                print(f"Running motion: {effective_motion_name} (run {run_idx}/{args.repeats})")
                if args.robot_name == "flexiv":
                    print(f"Flexiv mode: {args.flexiv_control_mode}")
                    if args.flexiv_control_mode == "nrt_joint_impedance":
                        print(f"Flexiv impedance: stiffness_scale={stiffness_scale:g}, damping_ratio={damping_ratio:g}")
                print(f"Progress: {current_run}/{total_runs} total runs")
                print(f"Output: {output_dir}/{effective_motion_name}/")
                print(f"{'='*60}\n")

                run_motion(
                    args.robot_name,
                    motion_file,
                    output_dir,
                    motion_name=effective_motion_name,
                    auto_start=args.auto_start,
                    robot_port=args.robot_port,
                    robot_type=args.robot_type,
                    robot_id=args.robot_id,
                    flexiv_robot_sn=args.flexiv_robot_sn,
                    flexiv_joint_group=args.flexiv_joint_group,
                    flexiv_dry_run=args.flexiv_dry_run,
                    flexiv_control_freq=args.flexiv_control_freq,
                    flexiv_slowdown_factor=args.flexiv_slowdown_factor,
                    flexiv_max_velocity=args.flexiv_max_velocity,
                    flexiv_max_acceleration=args.flexiv_max_acceleration,
                    flexiv_home_plan=args.flexiv_home_plan,
                    flexiv_start_move_duration=args.flexiv_start_move_duration,
                    flexiv_start_max_velocity=args.flexiv_start_max_velocity,
                    flexiv_start_max_acceleration=args.flexiv_start_max_acceleration,
                    flexiv_motion_scale=args.flexiv_motion_scale,
                    flexiv_max_initial_diff_rad=args.flexiv_max_initial_diff_rad,
                    flexiv_center_on_current_pose=args.flexiv_center_on_current_pose,
                    flexiv_control_mode=args.flexiv_control_mode,
                    flexiv_stiffness_scale=stiffness_scale if stiffness_scale is not None else 1.0,
                    flexiv_damping_ratio=damping_ratio if damping_ratio is not None else 0.7,
                )

                # Rest period between runs (skip after the last run)
                should_rest = current_run < total_runs and not (args.robot_name == "flexiv" and args.flexiv_dry_run)
                if should_rest:
                    print(f"\n*** Resting for {REST_PERIOD_SECONDS} seconds to let robot cool off ***\n")
                    time.sleep(REST_PERIOD_SECONDS)

    print(f"\n{'='*60}")
    print(f"All motions completed! Total runs: {total_runs}")
    print(f"{'='*60}\n")
