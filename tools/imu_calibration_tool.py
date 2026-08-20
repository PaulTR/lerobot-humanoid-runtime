#!/usr/bin/env python3
from __future__ import annotations

"""IMU Diagnostic & Zero-Bias Calibration Tool.

Verifies IMU streaming, samples static gravity/gyro bias offsets while held level,
and interactive tilt-check to confirm roll/pitch coordinate conventions.

Saves calibration offsets to `imu_calibration.json`.

Usage:
    # Physical IMU on I2C bus 1 (BNO055 / BNO085):
    uv run python tools/imu_calibration_tool.py --sensor bno055

    # Mock IMU testing:
    uv run python tools/imu_calibration_tool.py --sensor mock
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Tuple
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from imu.IMU_integration import IMU


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IMU Diagnostic & Calibration Tool.")
    parser.add_argument("--sensor", default="mock", choices=["bno055", "bno085", "jy901", "mock"], help="IMU sensor backend")
    parser.add_argument("--i2c-bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--address", type=str, default="0x28", help="I2C hex address (default: 0x28)")
    parser.add_argument("--samples", type=int, default=300, help="Number of static calibration samples to average")
    parser.add_argument("--output-file", default="imu_calibration.json", help="Output calibration JSON path")
    return parser.parse_args()


def parse_addr(addr_str: str) -> int:
    try:
        return int(addr_str, 16) if addr_str.startswith("0x") else int(addr_str)
    except ValueError:
        return 0x28


def run_static_calibration(imu_dev: IMU, num_samples: int) -> dict:
    print(f"\n[CALIBRATION] Starting static bias acquisition ({num_samples} samples)...")
    print("  --> Please keep the robot completely STILL on level ground.")
    input("Press [ENTER] when ready to start sampling...")

    acc_samples = []
    gyro_samples = []

    for i in range(num_samples):
        snap = imu_dev.read_dict()
        if snap:
            acc = snap.get("acceleration_mps2") or (0.0, 0.0, 9.81)
            gyro = snap.get("gyro_rads") or (0.0, 0.0, 0.0)
            acc_samples.append([float(acc[0]), float(acc[1]), float(acc[2])])
            gyro_samples.append([float(gyro[0]), float(gyro[1]), float(gyro[2])])

        if (i + 1) % 50 == 0 or i == num_samples - 1:
            print(f"  Sampling progress: {i + 1}/{num_samples} frames...")
        time.sleep(0.01)

    if not acc_samples:
        print("[ERROR] No IMU data received during sampling!")
        return {}

    acc_arr = np.array(acc_samples)
    gyro_arr = np.array(gyro_samples)

    acc_mean = np.mean(acc_arr, axis=0)
    gyro_mean = np.mean(gyro_arr, axis=0)

    acc_std = np.std(acc_arr, axis=0)
    gyro_std = np.std(gyro_arr, axis=0)

    print("\n[RESULTS] Static Sampling Complete:")
    print(f"  Accel Mean (m/s²): X={acc_mean[0]:.4f}, Y={acc_mean[1]:.4f}, Z={acc_mean[2]:.4f}")
    print(f"  Gyro Bias (rad/s): X={gyro_mean[0]:.6f}, Y={gyro_mean[1]:.6f}, Z={gyro_mean[2]:.6f}")
    print(f"  Accel StdDev     : X={acc_std[0]:.4f}, Y={acc_std[1]:.4f}, Z={acc_std[2]:.4f}")

    return {
        "accel_bias": acc_mean.tolist(),
        "gyro_bias": gyro_mean.tolist(),
        "accel_std": acc_std.tolist(),
        "gyro_std": gyro_std.tolist(),
    }


def quat_to_roll_pitch_deg(q: Tuple[float, float, float, float]) -> Tuple[float, float]:
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def run_orientation_tilt_check(imu_dev: IMU) -> dict:
    print("\n" + "=" * 65)
    print("           Interactive IMU Orientation Frame Check           ")
    print("=" * 65)
    print("This test verifies that roll/pitch signs match body frame standards.")

    tests = [
        ("Pitch Forward", "tilt torso FORWARD", "pitch_deg", "positive"),
        ("Pitch Backward", "tilt torso BACKWARD", "pitch_deg", "negative"),
        ("Roll Left", "tilt torso LEFT", "roll_deg", "negative"),
        ("Roll Right", "tilt torso RIGHT", "roll_deg", "positive"),
    ]

    results = {}
    for name, instruction, param, expected in tests:
        input(f"\n--> Please {instruction} and press [ENTER] to read orientation...")
        snap = imu_dev.read_dict()
        q = snap.get("quaternion_xyzw") or (0.0, 0.0, 0.0, 1.0)
        r_deg, p_deg = quat_to_roll_pitch_deg(q)

        val = p_deg if "pitch" in param else r_deg
        print(f"  Observed Orientation: Roll={r_deg:+.1f}°, Pitch={p_deg:+.1f}°")

        is_ok = (expected == "positive" and val > 1.0) or (expected == "negative" and val < -1.0)
        status = "PASSED" if is_ok else "CHECK FRAME SIGN"
        print(f"  Result [{name}]: {status} (expected {expected}, got {val:+.1f}°)")
        results[name] = {"measured_deg": val, "status": status}

    return results


def main() -> int:
    args = parse_args()
    addr = parse_addr(args.address)

    print("=" * 65)
    print("            LeRobot Humanoid - IMU Calibration Tool            ")
    print("=" * 65)
    print(f"Sensor Backend: {args.sensor.upper()} | I2C Bus: {args.i2c_bus} | Addr: 0x{addr:02X}")

    try:
        imu_dev = IMU(sensor=args.sensor, i2c_bus=args.i2c_bus, address=addr)
    except Exception as exc:
        print(f"[ERROR] Failed to initialize IMU backend '{args.sensor}': {exc}")
        return 1

    calib_data = run_static_calibration(imu_dev, args.samples)
    if not calib_data:
        return 1

    tilt_data = run_orientation_tilt_check(imu_dev)

    output = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "sensor": args.sensor,
        "calibration": calib_data,
        "tilt_check": tilt_data,
    }

    out_path = Path(args.output_file)
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)

    print("\n" + "=" * 65)
    print(f"[SUCCESS] Calibration saved to: {out_path.resolve()}")
    print("=" * 65)
    return 0


if __name__ == "__main__":
    sys.exit(main())
