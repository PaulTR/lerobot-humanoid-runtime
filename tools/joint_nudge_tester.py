#!/usr/bin/env python3
from __future__ import annotations

"""Single-Joint Nudge & Motion Direction Verifier Tool.

Interactive micro-motion tester to safely verify joint sign conventions and movement directions
one joint at a time before deploying dynamic control policies.

Applies soft test gains (default Kp=15.0, Kd=1.0) and small amplitude offsets (1 to 5 degrees).
Streams live feedback (Target vs Calibrated Angle vs Raw Angle vs Torque vs E-STOP state).

Usage:
    # On physical robot (with can0 & can1 up):
    uv run python tools/joint_nudge_tester.py
    # or with higher stiffness gain if needed:
    uv run python tools/joint_nudge_tester.py --kp 25.0

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
    parser.add_argument("--kp", type=float, default=15.0, help="Kp stiffness gain for testing (default: 15.0)")
    parser.add_argument("--kd", type=float, default=1.0, help="Kd damping gain for testing (default: 1.0)")
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
        leg = "Left " if mid <= 6 else "Right"
        print(f"  [{mid:>2}] {leg} {m.name:<15} (Bus: {'can0' if mid <= 6 else 'can1'})")
    print("=" * 65)
    print("Commands: Select Motor ID (1-12), 'zero' for zero pose, 'gains' to change Kp, 'q' to exit")
    print("=" * 65)


def nudge_motor(robot, motor_id: int, nudge_deg: float, args: argparse.Namespace) -> None:
    mname = MOTORS[motor_id].name
    print(f"\n[NUDGE] Commanding Motor ID {motor_id} ({mname}) by {nudge_deg:+.1f}° (Kp={args.kp}, Kd={args.kd})...")
    
    # Set gains on the target motor (and partner if coupled ankle)
    robot.set_joint_gains(motor_id, kp=args.kp, kd=args.kd)
    if motor_id in (5, 6):
        robot.set_joint_gains(5, kp=args.kp, kd=args.kd)
        robot.set_joint_gains(6, kp=args.kp, kd=args.kd)
    elif motor_id in (11, 12):
        robot.set_joint_gains(11, kp=args.kp, kd=args.kd)
        robot.set_joint_gains(12, kp=args.kp, kd=args.kd)

    # Clear any previous estop and switch to control mode
    robot.clear_estop()
    robot.set_mode("control")
    robot.enable_all()

    # Zero dicts
    zero_left = {"hipz": 0.0, "hipx": 0.0, "hipy": 0.0, "knee": 0.0, "ankle_pitch": 0.0, "ankle_roll": 0.0}
    zero_right = {"hipz": 0.0, "hipx": 0.0, "hipy": 0.0, "knee": 0.0, "ankle_pitch": 0.0, "ankle_roll": 0.0}

    # Map motor ID to joint key
    joint_map = {
        1: ("left", "hipz"),
        2: ("left", "hipx"),
        3: ("left", "hipy"),
        4: ("left", "knee"),
        5: ("left", "ankle_pitch"),
        6: ("left", "ankle_roll"),
        7: ("right", "hipz"),
        8: ("right", "hipx"),
        9: ("right", "hipy"),
        10: ("right", "knee"),
        11: ("right", "ankle_pitch"),
        12: ("right", "ankle_roll"),
    }
    side, joint_key = joint_map[motor_id]
    target_dict = zero_left if side == "left" else zero_right
    target_dict[joint_key] = float(nudge_deg)

    robot.set_action(left=zero_left, right=zero_right)

    # Stream live feedback for 1.5 seconds so user can see angles & torque
    print("  Streaming live feedback:")
    t_end = time.time() + 1.5
    last_meas = 0.0
    while time.time() < t_end:
        snap = robot.get_combined_state_snapshot()
        st = snap.get("motors", {}).get(motor_id, {})
        meas_deg = st.get("calibrated_position_deg", 0.0)
        raw_deg = st.get("raw_position_deg", 0.0)
        tau = st.get("torque_nm", 0.0)
        estop = snap.get("estop", False)
        estop_reason = snap.get("estop_reason", "")
        last_meas = meas_deg

        status_txt = "RUNNING" if not estop else f"E-STOP: {estop_reason}"
        sys.stdout.write(
            f"\r  --> Target: {nudge_deg:+.1f}° | Calibrated: {meas_deg:+6.2f}° (Raw: {raw_deg:+6.2f}°) | Torque: {tau:+5.2f}Nm | [{status_txt}]"
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
                robot.set_action(left=zero, right=zero)
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

            input("\nPress [ENTER] to return motor to zero pose and continue...")
            zero = {"hipz": 0.0, "hipx": 0.0, "hipy": 0.0, "knee": 0.0, "ankle_pitch": 0.0, "ankle_roll": 0.0}
            robot.set_action(left=zero, right=zero)

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
