#!/usr/bin/env python3
from __future__ import annotations

"""Single-Joint Nudge & Motion Direction Verifier Tool.

Interactive micro-motion tester to safely verify joint sign conventions and movement directions
one joint at a time before deploying dynamic control policies.

Features:
  - Quiet, smooth damping (default Kp=25.0, Kd=0.2) to prevent derivative chatter/humming.
  - Commands only the selected motor while keeping unselected motors quiet and relaxed.
  - Preserves current resting pose for unselected joints so displaced joints don't reject commands.
  - Streams live Target, Calibrated Angle, Raw Angle, Torque, and E-STOP telemetry in real time.

Usage:
    # On physical robot (with can0 & can1 up):
    uv run python tools/joint_nudge_tester.py

    # Dry run with mock CAN buses:
    python tools/joint_nudge_tester.py --use-mock-bus
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
LEROBOT_SRC = REPO_ROOT / "lerobot" / "src"
if LEROBOT_SRC.exists() and str(LEROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(LEROBOT_SRC))

from robot.root_constant import (
    CAN0_MOTOR_IDS,
    CAN1_MOTOR_IDS,
    MOTOR_IDS,
    MOTORS,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Single-Joint Nudge & Motion Direction Verifier."
    )
    parser.add_argument("--channel-can0", default="can0", help="CAN interface for left leg (default: can0)")
    parser.add_argument("--channel-can1", default="can1", help="CAN interface for right leg (default: can1)")
    parser.add_argument("--use-mock-bus", action="store_true", help="Use mock CAN buses for testing")
    parser.add_argument("--kp", type=float, default=25.0, help="Kp stiffness gain for active joint (default: 25.0)")
    parser.add_argument("--kd", type=float, default=0.2, help="Kd damping gain for active joint (default: 0.2)")
    return parser.parse_args()


def init_controller(args: argparse.Namespace):
    from robot.bipedal_robot import BipedalRobotController

    if args.use_mock_bus or "--use-mock-bus" in sys.argv:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot_humanoid_lerobot_integration.lerobot_humanoid import HUMANOID_MOTOR_TYPE_BY_ID
        from lerobot_humanoid_lerobot_integration.robstride_mock_bus import RobstrideMockBus

        def _build_motors(mids):
            return {
                f"m{mid}": Motor(
                    id=int(mid),
                    model="robstride",
                    norm_mode=MotorNormMode.DEGREES,
                    motor_type_str=str(HUMANOID_MOTOR_TYPE_BY_ID.get(int(mid), "o0")),
                    recv_id=int(mid),
                )
                for mid in mids
            }

        bus_can0 = RobstrideMockBus(motors=_build_motors(CAN0_MOTOR_IDS))
        bus_can1 = RobstrideMockBus(motors=_build_motors(CAN1_MOTOR_IDS))
        robot = BipedalRobotController(bus_can0=bus_can0, bus_can1=bus_can1, control_hz=100.0)
    else:
        robot = BipedalRobotController(
            channel_can0=args.channel_can0,
            channel_can1=args.channel_can1,
            control_hz=100.0,
        )

    # Allow command deltas during interactive testing
    robot.set_max_command_delta(80.0)

    # Start background control loop in safe state_only mode initially
    robot.start(mode="state_only", auto_enable=False)
    time.sleep(0.1)
    missing = robot.request_state_once()
    if missing:
        print(f"\n[WARNING] Some motors did not respond on CAN: {missing}")
        print("  Make sure CAN buses are up and all motors are powered.\n")
    else:
        print("[INIT] All 12 motors responding on CAN.")

    return robot


def print_menu():
    print("=" * 65)
    print("           Single-Joint Micro-Nudge Testing Menu           ")
    print("=" * 65)
    for mid in sorted(MOTORS.keys()):
        m = MOTORS[mid]
        leg = "Right" if mid <= 6 else "Left "
        print(f"  [{mid:>2}] {leg} {m.name:<15} (Bus: {'can0' if mid <= 6 else 'can1'})")
    print("=" * 65)
    print("Commands: Select Motor ID (1-12), 'zero' for zero pose, 'gains' to change Kp/Kd, 'q' to exit")
    print("=" * 65)


def nudge_motor(robot, motor_id: int, nudge_deg: float, args: argparse.Namespace, *, relative: bool = False) -> None:
    mname = MOTORS[motor_id].name
    side = "right" if motor_id <= 6 else "left"

    # Map motor ID to joint key
    joint_map = {
        1: ("right", "hipz"),
        2: ("right", "hipx"),
        3: ("right", "hipy"),
        4: ("right", "knee"),
        5: ("right", "ankle_pitch"),
        6: ("right", "ankle_roll"),
        7: ("left", "hipz"),
        8: ("left", "hipx"),
        9: ("left", "hipy"),
        10: ("left", "knee"),
        11: ("left", "ankle_pitch"),
        12: ("left", "ankle_roll"),
    }
    target_side, joint_key = joint_map[motor_id]

    # Get current live joint positions
    snap = robot.get_combined_state_snapshot()
    cur_raw = {mid: snap.get("motors", {}).get(mid, {}).get("raw_position_deg", 0.0) for mid in MOTOR_IDS}
    q_cur = robot.motor_state_to_joint_state(cur_raw, output_radians=False, nq=12)

    # In model joint array: Left is 0..5, Right is 6..11
    joint_idx_map = {
        7: 0, 8: 1, 9: 2, 10: 3, 11: 4, 12: 5, # Left
        1: 6, 2: 7, 3: 8, 4: 9, 5: 10, 6: 11,   # Right
    }
    cur_joint_val = float(q_cur[joint_idx_map[motor_id]])
    target_val = (cur_joint_val + nudge_deg) if relative else nudge_deg

    print(f"\n[NUDGE] Commanding {mname} (m{motor_id}) -> {target_val:+.1f}° (Kp={args.kp}, Kd={args.kd})...")

    # Set soft quiet holding gains on unselected motors, and active gains on test motor
    for mid in MOTOR_IDS:
        if mid == motor_id or (motor_id in (5, 6) and mid in (5, 6)) or (motor_id in (11, 12) and mid in (11, 12)):
            robot.set_joint_gains(mid, kp=args.kp, kd=args.kd)
        else:
            robot.set_joint_gains(mid, kp=0.0, kd=0.05)  # Quiet relaxed state

    # Build action preserving current resting angles for other joints
    target_left = {
        "hipz": float(q_cur[0]), "hipx": float(q_cur[1]), "hipy": float(q_cur[2]),
        "knee": float(q_cur[3]), "ankle_pitch": float(q_cur[4]), "ankle_roll": float(q_cur[5])
    }
    target_right = {
        "hipz": float(q_cur[6]), "hipx": float(q_cur[7]), "hipy": float(q_cur[8]),
        "knee": float(q_cur[9]), "ankle_pitch": float(q_cur[10]), "ankle_roll": float(q_cur[11])
    }

    if target_side == "left":
        target_left[joint_key] = float(target_val)
    else:
        target_right[joint_key] = float(target_val)

    # Clear estop, switch to control mode, and enable
    robot.clear_estop()
    robot.set_mode("control")
    robot.enable_all()
    robot.set_action(left=target_left, right=target_right)

    # Stream live feedback for 2.0 seconds
    print("  Streaming live feedback:")
    t_end = time.time() + 2.0
    while time.time() < t_end:
        snap = robot.get_combined_state_snapshot()
        st = snap.get("motors", {}).get(motor_id, {})
        meas_deg = st.get("calibrated_position_deg", 0.0)
        raw_deg = st.get("raw_position_deg", 0.0)
        tau = st.get("torque_nm", 0.0)
        estop = snap.get("estop", False)
        estop_reason = snap.get("estop_reason", "")

        status_txt = "RUNNING" if not estop else f"E-STOP: {estop_reason}"
        sys.stdout.write(
            f"\r  --> Target: {target_val:+.1f}° | Calibrated: {meas_deg:+6.2f}° (Raw: {raw_deg:+6.2f}°) | Torque: {tau:+5.2f}Nm | [{status_txt}]"
        )
        sys.stdout.flush()
        time.sleep(0.05)
    print()


def main() -> int:
    args = parse_args()
    try:
        robot = init_controller(args)
    except Exception as exc:
        print(f"[ERROR] Could not initialize CAN controller: {exc}")
        return 1

    verified_joints: Dict[int, str] = {}

    try:
        while True:
            print_menu()
            choice = input(f"Select Motor ID or command [Current Kp={args.kp}, Kd={args.kd}]: ").strip().lower()

            if choice in ("q", "quit", "exit"):
                break

            if choice == "gains":
                try:
                    new_kp = float(input(f"Enter new Kp gain (current={args.kp}): "))
                    new_kd = float(input(f"Enter new Kd gain (current={args.kd}): "))
                    args.kp = new_kp
                    args.kd = new_kd
                    print(f"[GAINS] Updated test gains to Kp={args.kp}, Kd={args.kd}")
                except ValueError:
                    print("[ERROR] Invalid number.")
                continue

            if choice == "zero":
                print("[ACTION] Returning all joints to 0.0° zero pose...")
                zero = {"hipz": 0.0, "hipx": 0.0, "hipy": 0.0, "knee": 0.0, "ankle_pitch": 0.0, "ankle_roll": 0.0}
                robot.clear_estop()
                robot.set_mode("control")
                robot.enable_all()
                for mid in MOTOR_IDS:
                    robot.set_joint_gains(mid, kp=args.kp, kd=args.kd)
                robot.set_action(left=zero, right=zero)
                time.sleep(1.0)
                continue

            try:
                mid = int(choice)
                if mid not in MOTOR_IDS:
                    print("[ERROR] Invalid Motor ID. Choose 1 to 12.")
                    continue
            except ValueError:
                print("[ERROR] Invalid input.")
                continue

            # Sub-menu for selected motor
            mname = MOTORS[mid].name
            print(f"\n--- Testing Motor ID {mid} ({mname}) ---")
            print("Select Nudge: [1] +2.0°  [2] -2.0°  [3] +5.0°  [4] -5.0°  [5] Custom  [b] Back")
            action = input("Choice: ").strip().lower()

            if action == "1":
                nudge_motor(robot, mid, +2.0, args)
            elif action == "2":
                nudge_motor(robot, mid, -2.0, args)
            elif action == "3":
                nudge_motor(robot, mid, +5.0, args)
            elif action == "4":
                nudge_motor(robot, mid, -5.0, args)
            elif action == "5":
                try:
                    cdeg = float(input("Enter custom angle delta (deg): "))
                    nudge_motor(robot, mid, cdeg, args)
                except ValueError:
                    print("[ERROR] Invalid float.")
                    continue
            elif action == "b":
                continue

            confirm = input(f"\nDid {mname} move in expected physical direction? (y/n) [y]: ").strip().lower()
            if confirm in ("y", ""):
                verified_joints[mid] = "PASSED"
                print(f"[VERIFIED] {mname} direction confirmed correct!")
            else:
                verified_joints[mid] = "INVERTED/FAILED"
                print(f"[WARNING] {mname} direction mismatch flagged! Check sign table in root_constant.py.")

            input("\nPress [ENTER] to relax motors and continue...")
            robot.set_mode("state_only")
            robot.disable_all()

        print("\n=" * 65)
        print("                MOTION VERIFICATION SUMMARY                ")
        print("=" * 65)
        for mid in sorted(MOTOR_IDS):
            status = verified_joints.get(mid, "UNTESTED")
            print(f"  Motor ID {mid:>2} ({MOTORS[mid].name:<15}): {status}")
        print("=" * 65)

    finally:
        try:
            robot.stop(disable_motors=True)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
