#!/usr/bin/env python3
from __future__ import annotations

"""Interactive Manual Zeroing Wizard for LeRobot Humanoid.

This script guides the user step-by-step through manual joint zeroing.
For each single joint (Hips & Knees), it prompts the user to physically align the joint,
waits for confirmation (pressing Enter), and sends the motor zero command (CAN_CMD_ZERO = 0xFE).
For the feet (Ankles), it prompts once per leg to set the physical ankle alignment tool,
and zeroes the paired ankle motors simultaneously (IDs 5 & 6 for Left, IDs 11 & 12 for Right).

Usage:
    # On real hardware (Raspberry Pi 5 with can0 & can1 up):
    uv run python tools/interactive_zeroing.py

    # Dry-run test with mock CAN buses:
    uv run python tools/interactive_zeroing.py --use-mock-bus
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

# Ensure repo root is on sys.path
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from robot.root_constant import (
    CAN0_MOTOR_IDS,
    CAN1_MOTOR_IDS,
    MOTORS,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive manual joint zeroing wizard for LeRobot Humanoid."
    )
    parser.add_argument(
        "--channel-can0",
        default="can0",
        help="CAN interface for left leg / motors 1-6 (default: can0)",
    )
    parser.add_argument(
        "--channel-can1",
        default="can1",
        help="CAN interface for right leg / motors 7-12 (default: can1)",
    )
    parser.add_argument(
        "--use-mock-bus",
        action="store_true",
        help="Use mock CAN buses for testing without physical CAN hardware",
    )
    return parser.parse_args()


def get_joint_sequence() -> list[dict]:
    """Define the zeroing sequence step by step.

    Covers IDs 1-6 (Leg 1 / Left leg on can0) and IDs 7-12 (Leg 2 / Right leg on can1).
    For the feet/ankles, motors 5 & 6 (Left) and 11 & 12 (Right) are both mounted in the shin/tibia
    and are zeroed together using the printed ankle calibration tool attached between the tibia and foot.
    """
    return [
        # --- Leg 1 / Left (CAN0: IDs 1..6) ---
        {
            "step": 1,
            "side": "Leg 1 (CAN0 / Left)",
            "joint_name": "Hip Yaw (Z)",
            "motor_ids": [1],
            "instruction": (
                "Physically align Left Hip Z (Motor 1) to its zero position:\n"
                "  • Rotate pelvis block so it points straight forward parallel to forward axis.\n"
                "  • Reference images: docs/calibration_assets/zero_refs/hipz_zeros1.jpg & hipz_zero2.jpg"
            ),
        },
        {
            "step": 2,
            "side": "Leg 1 (CAN0 / Left)",
            "joint_name": "Hip Roll (X)",
            "motor_ids": [2],
            "instruction": (
                "Physically align Left Hip X (Motor 2) to its zero position:\n"
                "  • Align leg flush vertically with torso frame (0 deg abduction/adduction).\n"
                "  • Reference image: docs/calibration_assets/zero_refs/hipx_zero.jpg"
            ),
        },
        {
            "step": 3,
            "side": "Leg 1 (CAN0 / Left)",
            "joint_name": "Hip Pitch (Y)",
            "motor_ids": [3],
            "instruction": (
                "Physically align Left Hip Y (Motor 3) to its zero position:\n"
                "  • Align thigh member pointing straight down vertically.\n"
                "  • Reference image: docs/calibration_assets/zero_refs/hipy_zero.jpg"
            ),
        },
        {
            "step": 4,
            "side": "Leg 1 (CAN0 / Left)",
            "joint_name": "Knee",
            "motor_ids": [4],
            "instruction": (
                "Physically align Left Knee (Motor 4) to its zero position:\n"
                "  • Fully extend shin straight down with the thigh (0 deg knee flexion).\n"
                "  • Reference image: docs/calibration_assets/zero_refs/knee_zero.jpg"
            ),
        },
        {
            "step": 5,
            "side": "Leg 1 (CAN0 / Left)",
            "joint_name": "Tibia Motors 5 & 6 / Ankle Linkage",
            "motor_ids": [5, 6],
            "instruction": (
                "Physically align Left Ankle/Foot using the Printed Alignment Tool:\n"
                "  • Motors 5 & 6 are both mounted inside the Left Tibia (shin) driving the parallel rods.\n"
                "  • Mount the 3D-printed tool (docs/calibration_assets/zero_refs/ankle_calibration_tool.stl)\n"
                "    between the tibia (shin) and the foot to lock the ankle at 90 deg.\n"
                "  • Both Tibia Motors 5 and 6 will be zeroed together in this step.\n"
                "  • Reference image: docs/calibration_assets/zero_refs/ankle_zero2.jpg"
            ),
        },
        # --- Leg 2 / Right (CAN1: IDs 7..12) ---
        {
            "step": 6,
            "side": "Leg 2 (CAN1 / Right)",
            "joint_name": "Hip Yaw (Z)",
            "motor_ids": [7],
            "instruction": (
                "Physically align Right Hip Z (Motor 7) to its zero position:\n"
                "  • Rotate pelvis block so it points straight forward parallel to forward axis.\n"
                "  • Reference images: docs/calibration_assets/zero_refs/hipz_zeros1.jpg & hipz_zero2.jpg"
            ),
        },
        {
            "step": 7,
            "side": "Leg 2 (CAN1 / Right)",
            "joint_name": "Hip Roll (X)",
            "motor_ids": [8],
            "instruction": (
                "Physically align Right Hip X (Motor 8) to its zero position:\n"
                "  • Align leg flush vertically with torso frame (0 deg abduction/adduction).\n"
                "  • Reference image: docs/calibration_assets/zero_refs/hipx_zero.jpg"
            ),
        },
        {
            "step": 8,
            "side": "Leg 2 (CAN1 / Right)",
            "joint_name": "Hip Pitch (Y)",
            "motor_ids": [9],
            "instruction": (
                "Physically align Right Hip Y (Motor 9) to its zero position:\n"
                "  • Align thigh member pointing straight down vertically.\n"
                "  • Reference image: docs/calibration_assets/zero_refs/hipy_zero.jpg"
            ),
        },
        {
            "step": 9,
            "side": "Leg 2 (CAN1 / Right)",
            "joint_name": "Knee",
            "motor_ids": [10],
            "instruction": (
                "Physically align Right Knee (Motor 10) to its zero position:\n"
                "  • Fully extend shin straight down with the thigh (0 deg knee flexion).\n"
                "  • Reference image: docs/calibration_assets/zero_refs/knee_zero.jpg"
            ),
        },
        {
            "step": 10,
            "side": "Leg 2 (CAN1 / Right)",
            "joint_name": "Tibia Motors 11 & 12 / Ankle Linkage",
            "motor_ids": [11, 12],
            "instruction": (
                "Physically align Right Ankle/Foot using the Printed Alignment Tool:\n"
                "  • Motors 11 & 12 are both mounted inside the Right Tibia (shin) driving the parallel rods.\n"
                "  • Mount the 3D-printed tool (docs/calibration_assets/zero_refs/ankle_calibration_tool.stl)\n"
                "    between the tibia (shin) and the foot to lock the ankle at 90 deg.\n"
                "  • Both Tibia Motors 11 and 12 will be zeroed together in this step.\n"
                "  • Reference image: docs/calibration_assets/zero_refs/ankle_zero2.jpg"
            ),
        },
    ]


class DummyMockBus:
    """Lightweight dummy CAN bus for dry-run zeroing without hardware dependencies."""
    def send(self, msg: Any) -> None:
        pass

    def recv(self, timeout: float = 0.5) -> None:
        return None

    def shutdown(self) -> None:
        pass


def init_robot_controller(args: argparse.Namespace):
    """Initialize the robot controller for real hardware or dry-run mock mode."""
    from robot.bipedal_robot import BipedalRobotController

    if args.use_mock_bus or "--use-mock-bus" in sys.argv:
        print("[INIT] Operating in dry-run mode (--use-mock-bus)...")
        try:
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
        except Exception:
            bus_can0 = DummyMockBus()
            bus_can1 = DummyMockBus()
        robot = BipedalRobotController(bus_can0=bus_can0, bus_can1=bus_can1)
    else:
        print(f"[INIT] Opening real CAN interfaces ({args.channel_can0}, {args.channel_can1})...")
        try:
            robot = BipedalRobotController(
                channel_can0=args.channel_can0,
                channel_can1=args.channel_can1,
            )
        except Exception as exc:
            print(f"\n[ERROR] Could not open CAN interfaces: {exc}")
            print("  Make sure CAN interfaces are up (e.g., `sudo ip link set can0 up...`).")
            print("  Or run with `--use-mock-bus` for a dry run test.\n")
            sys.exit(1)

    return robot


def main() -> int:
    args = parse_args()

    print("=" * 70)
    print("      LeRobot Humanoid - Interactive Manual Zeroing Wizard      ")
    print("=" * 70)
    print("This wizard will guide you through setting zero positions for each joint.")
    print("For each step:")
    print("  1. Manually move the physical joint to its reference zero spot.")
    print("  2. Press [ENTER] to send the hardware zero command to the motor.")
    print("  3. Type 's' to skip a joint, or 'q' to quit at any time.")
    print("=" * 70)
    print()

    robot = init_robot_controller(args)
    # Ensure estop flag is cleared so set_zero commands are transmitted
    robot._estop = False
    robot._estop_reason = ""

    sequence = get_joint_sequence()
    zeroed_motors: list[int] = []

    try:
        for item in sequence:
            step_num = item["step"]
            side = item["side"]
            joint = item["joint_name"]
            motor_ids = item["motor_ids"]
            mids_str = ", ".join(f"ID {mid}" for mid in motor_ids)

            print("-" * 70)
            print(f"STEP {step_num}/10 | {side} - {joint} ({mids_str})")
            print("-" * 70)
            print(f"{item['instruction']}")
            print("-" * 70)

            while True:
                user_in = input(
                    f"--> Position {joint} ({mids_str}) and press [ENTER] to ZERO (or 's' to skip, 'q' to quit): "
                ).strip().lower()

                if user_in == "q":
                    print("\n[WIZARD] Exiting zeroing process upon user request.")
                    return 0

                if user_in == "s":
                    print(f"[SKIP] Skipped zeroing for {joint} ({mids_str}).\n")
                    break

                if user_in == "":
                    # Send zero command to each motor in this step
                    for mid in motor_ids:
                        print(f"  [CAN] Transmitting CAN_CMD_ZERO (0xFE) to Motor ID {mid}...", end="", flush=True)
                        robot.set_zero(mid)
                        print(" [OK] Zero set!")
                        if mid not in zeroed_motors:
                            zeroed_motors.append(mid)

                    print(f"--> {joint} ({mids_str}) zeroing COMPLETED!\n")
                    break

        print("=" * 70)
        print("                 ZEROING WIZARD SUMMARY                 ")
        print("=" * 70)
        print(f"Total motors zeroed: {len(zeroed_motors)} / 12")
        print(f"Zeroed Motor IDs: {sorted(zeroed_motors)}")
        print("Please verify joint calibration in `state_only` read-only mode!")
        print("=" * 70)

    finally:
        # Disable background threads or bus executors cleanly
        try:
            robot.disable_all()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
