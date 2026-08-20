#!/usr/bin/env python3
from __future__ import annotations

"""CAN Bus Motor Scanner & ID Diagnostic Tool for LeRobot Humanoid.

Scans `can0` (Left Leg: IDs 1..6) and `can1` (Right Leg: IDs 7..12) to verify
all 12 motors are connected, responding with valid state, and assigned to the correct bus.

Works with:
1) Direct system python on Linux (zero dependencies, native socketcan)
2) uv virtualenv (`uv run python tools/scan_motors.py`)
3) Offline mock testing (`--use-mock-bus`)

Usage:
    # Scan standard humanoid motor buses (can0 & can1):
    uv run python tools/scan_motors.py
    # or directly with python:
    python tools/scan_motors.py

    # Scan a wider ID range (0..127) to find unconfigured motors:
    python tools/scan_motors.py --full-scan

    # Dry-run test:
    python tools/scan_motors.py --use-mock-bus
"""

import argparse
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Ensure repo root is on sys.path
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from hardware.mit_codec import decode_state_frame
from robot.root_constant import (
    CAN0_MOTOR_IDS,
    CAN1_MOTOR_IDS,
    CAN_CMD_CLEAR_FAULT,
    MOTOR_IDS,
    MOTORS,
)

CAN_FRAME_STRUCT_FMT = "=IB3x8s"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CAN Bus Motor Scanner & ID Diagnostic Tool for LeRobot Humanoid."
    )
    parser.add_argument("--channel-can0", default="can0", help="CAN interface for left leg (default: can0)")
    parser.add_argument("--channel-can1", default="can1", help="CAN interface for right leg (default: can1)")
    parser.add_argument("--full-scan", action="store_true", help="Scan full ID range 0..127 on each bus")
    parser.add_argument("--timeout", type=float, default=0.08, help="Timeout in seconds per ping (default: 0.08s)")
    parser.add_argument("--use-mock-bus", action="store_true", help="Run in mock mode for testing without CAN hardware")
    return parser.parse_args()


class NativeSocketCANClient:
    """Lightweight SocketCAN client using Python standard library."""
    def __init__(self, channel: str):
        self.channel = channel
        self._sock = None
        if not hasattr(socket, "AF_CAN"):
            raise RuntimeError("AF_CAN is only supported on Linux kernel systems.")
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((channel,))
        self._sock.settimeout(0.05)

    def ping_motor(self, motor_id: int, timeout_s: float = 0.08) -> Optional[Tuple[int, bytes]]:
        # Flush stale input
        self._sock.settimeout(0.001)
        while True:
            try:
                self._sock.recv(16)
            except (socket.timeout, BlockingIOError, OSError):
                break

        # Send CLEAR_FAULT (0xFB) query frame
        payload = bytes([0xFF] * 7 + [CAN_CMD_CLEAR_FAULT])
        frame = struct.pack(CAN_FRAME_STRUCT_FMT, int(motor_id), 8, payload)
        self._sock.send(frame)

        # Wait for reply
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            self._sock.settimeout(remaining)
            try:
                raw = self._sock.recv(16)
                can_id, can_dlc, data = struct.unpack(CAN_FRAME_STRUCT_FMT, raw)
                if can_dlc >= 8:
                    resp_mid = int(data[0])
                    if resp_mid == int(motor_id) or can_id == int(motor_id):
                        return resp_mid, data[:can_dlc]
            except (socket.timeout, BlockingIOError, OSError):
                break
        return None

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None


def scan_bus_native(channel: str, scan_ids: List[int], timeout_s: float) -> Dict[int, Dict[str, Any]]:
    results: Dict[int, Dict[str, Any]] = {}
    try:
        client = NativeSocketCANClient(channel)
    except Exception as exc:
        print(f"  [WARN] Could not open {channel}: {exc}")
        return results

    try:
        for mid in scan_ids:
            reply = client.ping_motor(mid, timeout_s=timeout_s)
            if reply is not None:
                resp_id, data = reply
                res: Dict[str, Any] = {"raw_data": data, "detected": True}
                spec = MOTORS.get(resp_id)
                if spec is not None:
                    try:
                        _, st = decode_state_frame(data, pmax=spec.pmax_rad, vmax=spec.vmax_rad_s, tmax=spec.tmax_nm)
                        res["pos_deg"] = st.position_deg
                        res["temp_c"] = st.temp_mos_c
                    except Exception:
                        pass
                results[resp_id] = res
    finally:
        client.close()

    return results


def scan_bus_python_can(channel: str, scan_ids: List[int], timeout_s: float) -> Dict[int, Dict[str, Any]]:
    import can
    results: Dict[int, Dict[str, Any]] = {}
    try:
        bus = can.interface.Bus(interface="socketcan", channel=channel)
    except Exception as exc:
        print(f"  [WARN] Could not open {channel} via python-can: {exc}")
        return results

    try:
        for mid in scan_ids:
            # Send CLEAR_FAULT
            msg = can.Message(arbitration_id=mid, data=[0xFF] * 7 + [CAN_CMD_CLEAR_FAULT], is_extended_id=False)
            bus.send(msg)

            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                rx = bus.recv(timeout=0.01)
                if rx and len(rx.data) >= 8:
                    resp_id = int(rx.data[0])
                    if resp_id == int(mid) or rx.arbitration_id == int(mid):
                        res = {"raw_data": bytes(rx.data), "detected": True}
                        spec = MOTORS.get(resp_id)
                        if spec is not None:
                            try:
                                _, st = decode_state_frame(bytes(rx.data), pmax=spec.pmax_rad, vmax=spec.vmax_rad_s, tmax=spec.tmax_nm)
                                res["pos_deg"] = st.position_deg
                                res["temp_c"] = st.temp_mos_c
                            except Exception:
                                pass
                        results[resp_id] = res
                        break
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    return results


def scan_bus_mock(channel: str, scan_ids: List[int]) -> Dict[int, Dict[str, Any]]:
    results: Dict[int, Dict[str, Any]] = {}
    expected_ids = CAN0_MOTOR_IDS if channel == "can0" else CAN1_MOTOR_IDS
    for mid in scan_ids:
        if mid in expected_ids:
            results[mid] = {
                "detected": True,
                "pos_deg": 0.0,
                "temp_c": 30.0,
            }
    return results


def run_scanner(args: argparse.Namespace) -> int:
    print("=" * 76)
    print("           LeRobot Humanoid - CAN Bus Motor ID Scanner             ")
    print("=" * 76)

    scan_ids = list(range(0, 128)) if args.full_scan else list(range(1, 13))
    print(f"Scan Mode : {'Full Range (0..127)' if args.full_scan else 'Standard Humanoid (1..12)'}")
    print(f"Interfaces: Left Leg = {args.channel_can0} | Right Leg = {args.channel_can1}")
    if args.use_mock_bus:
        print("Backend   : MOCK (Simulation)")
    print("=" * 76)
    print()

    # Perform scans
    if args.use_mock_bus:
        can0_res = scan_bus_mock(args.channel_can0, scan_ids)
        can1_res = scan_bus_mock(args.channel_can1, scan_ids)
    else:
        # Try native SocketCAN first, fallback to python-can
        if hasattr(socket, "AF_CAN"):
            can0_res = scan_bus_native(args.channel_can0, scan_ids, args.timeout)
            can1_res = scan_bus_native(args.channel_can1, scan_ids, args.timeout)
        else:
            try:
                import can
                can0_res = scan_bus_python_can(args.channel_can0, scan_ids, args.timeout)
                can1_res = scan_bus_python_can(args.channel_can1, scan_ids, args.timeout)
            except Exception as exc:
                print(f"[ERROR] No CAN backend available: {exc}")
                print("  On Linux, verify interfaces with `ip link show can0`.")
                print("  Or run with `--use-mock-bus` for offline dry-run.\n")
                return 1

    # Format Results Table
    print("CAN0 (Left Leg - Expected IDs 1..6):")
    print("-" * 76)
    print(f"{'ID':>3} | {'Joint Name':<16} | {'Model':<5} | {'Status':<10} | {'Raw Pos':>9} | {'Temp':>6}")
    print("-" * 76)
    missing_left = []
    for mid in CAN0_MOTOR_IDS:
        m = MOTORS.get(mid)
        jname = m.name if m else f"ID {mid}"
        model = getattr(m, "motor_type", "o?") if m else "-"
        if mid in can0_res:
            st = can0_res[mid]
            pos_txt = f"{st.get('pos_deg', 0.0):+6.1f}°" if "pos_deg" in st else "n/a"
            temp_txt = f"{st.get('temp_c', 0.0):.0f}°C" if "temp_c" in st else "n/a"
            status = "✓ ONLINE"
        else:
            status = "✗ MISSING"
            pos_txt = "---"
            temp_txt = "---"
            missing_left.append(mid)
        print(f"{mid:>3} | {jname:<16} | {model:<5} | {status:<10} | {pos_txt:>9} | {temp_txt:>6}")

    # Check for unexpected IDs on can0
    extra_can0 = [mid for mid in can0_res.keys() if mid not in CAN0_MOTOR_IDS]
    if extra_can0:
        print("-" * 76)
        print(f"  [!] Unexpected motors found on {args.channel_can0}: {extra_can0}")

    print()
    print("CAN1 (Right Leg - Expected IDs 7..12):")
    print("-" * 76)
    print(f"{'ID':>3} | {'Joint Name':<16} | {'Model':<5} | {'Status':<10} | {'Raw Pos':>9} | {'Temp':>6}")
    print("-" * 76)
    missing_right = []
    for mid in CAN1_MOTOR_IDS:
        m = MOTORS.get(mid)
        jname = m.name if m else f"ID {mid}"
        model = getattr(m, "motor_type", "o?") if m else "-"
        if mid in can1_res:
            st = can1_res[mid]
            pos_txt = f"{st.get('pos_deg', 0.0):+6.1f}°" if "pos_deg" in st else "n/a"
            temp_txt = f"{st.get('temp_c', 0.0):.0f}°C" if "temp_c" in st else "n/a"
            status = "✓ ONLINE"
        else:
            status = "✗ MISSING"
            pos_txt = "---"
            temp_txt = "---"
            missing_right.append(mid)
        print(f"{mid:>3} | {jname:<16} | {model:<5} | {status:<10} | {pos_txt:>9} | {temp_txt:>6}")

    # Check for unexpected IDs on can1
    extra_can1 = [mid for mid in can1_res.keys() if mid not in CAN1_MOTOR_IDS]
    if extra_can1:
        print("-" * 76)
        print(f"  [!] Unexpected motors found on {args.channel_can1}: {extra_can1}")

    # Summary
    total_found = (6 - len(missing_left)) + (6 - len(missing_right))
    print("=" * 76)
    print(f"SCAN SUMMARY: {total_found} / 12 Expected Motors Responding")
    if not missing_left and not missing_right:
        print(">> STATUS: ALL 12 MOTORS READY ON CORRECT CAN BUSES! <<")
    else:
        if missing_left:
            print(f"  • Missing on {args.channel_can0} (Left) : IDs {missing_left}")
        if missing_right:
            print(f"  • Missing on {args.channel_can1} (Right): IDs {missing_right}")
        print("\nTroubleshooting tips:")
        print("  1. Verify bus interfaces: `ip link show can0` & `ip link show can1`")
        print("  2. Check motor 48V power and CAN bus termination resistors (120 Ohm).")
        print("  3. Run `--full-scan` to check if a missing motor has a factory ID (e.g. ID 0 or 127).")
    print("=" * 76)

    return 0 if (not missing_left and not missing_right) else 1


def main() -> int:
    args = parse_args()
    return run_scanner(args)


if __name__ == "__main__":
    sys.exit(main())
