#!/usr/bin/env python3
from __future__ import annotations

"""Render Reference Zero-Pose Simulation Images for LeRobot Humanoid.

Renders high-resolution reference screenshots of the bipedal platform (no arms,
up to torso and Raspberry Pi) in the base zero configuration ($q = 0.0$ for all joints).
"""

import sys
from pathlib import Path
from PIL import Image

try:
    import mujoco
except ImportError as exc:
    print(f"[ERROR] mujoco is required to render simulation reference images: {exc}")
    sys.exit(1)

REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_XML = REPO_ROOT.parent / "lerobot-humanoid-model" / "models" / "bipedal_plateform_no_arms" / "mjcf" / "scene.xml"
OUT_DIR = REPO_ROOT / "docs" / "calibration_assets" / "zero_refs"


def main() -> int:
    if not MODEL_XML.exists():
        print(f"[ERROR] Model XML not found at {MODEL_XML}")
        return 1

    print(f"[RENDER] Loading MuJoCo model from: {MODEL_XML}")
    m = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    m.vis.global_.offwidth = 1600
    m.vis.global_.offheight = 1600
    d = mujoco.MjData(m)

    # Set base pose at standing height and all joints to 0.0
    d.qpos[:] = 0.0
    d.qpos[2] = 0.7  # Root Z height
    d.qpos[3] = 1.0  # Root quaternion w
    mujoco.mj_forward(m, d)

    renderer = mujoco.Renderer(m, height=1200, width=1200)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    views = {
        "simulation_zero_pose_front.png": {"lookat": [0.0, 0.0, 0.40], "dist": 1.6, "elevation": -5.0, "azimuth": 0.0},
        "simulation_zero_pose_side.png": {"lookat": [0.0, 0.0, 0.40], "dist": 1.6, "elevation": -5.0, "azimuth": 90.0},
        "simulation_zero_pose_iso.png": {"lookat": [0.0, 0.0, 0.40], "dist": 1.7, "elevation": -15.0, "azimuth": 45.0},
        "training_model_front.png": {"lookat": [0.0, 0.0, 0.40], "dist": 1.5, "elevation": -3.0, "azimuth": 0.0},
        "training_model_side.png": {"lookat": [0.0, 0.0, 0.40], "dist": 1.5, "elevation": -3.0, "azimuth": 90.0},
        "training_model_hip_knee_close.png": {"lookat": [0.0, 0.0, 0.50], "dist": 0.85, "elevation": -10.0, "azimuth": 35.0},
        "training_model_ankle_close.png": {"lookat": [-0.02, -0.11, 0.12], "dist": 0.55, "elevation": -15.0, "azimuth": 30.0},
    }

    cam = mujoco.MjvCamera()
    for fname, cfg in views.items():
        cam.lookat = cfg["lookat"]
        cam.distance = cfg["dist"]
        cam.elevation = cfg["elevation"]
        cam.azimuth = cfg["azimuth"]
        renderer.update_scene(d, camera=cam)
        rgb = renderer.render()
        img = Image.fromarray(rgb)
        target_path = OUT_DIR / fname
        img.save(target_path)
        print(f"  [SAVED] {fname} ({img.size[0]}x{img.size[1]}) -> {target_path}")

    # Generate combined 3-view strip
    front_img = Image.open(OUT_DIR / "simulation_zero_pose_front.png")
    side_img = Image.open(OUT_DIR / "simulation_zero_pose_side.png")
    iso_img = Image.open(OUT_DIR / "simulation_zero_pose_iso.png")

    w, h = front_img.size
    grid = Image.new("RGB", (w * 3, h))
    grid.paste(front_img, (0, 0))
    grid.paste(side_img, (w, 0))
    grid.paste(iso_img, (2 * w, 0))
    views_path = OUT_DIR / "simulation_zero_pose_views.png"
    grid.save(views_path)
    print(f"  [SAVED] simulation_zero_pose_views.png ({grid.size[0]}x{grid.size[1]}) -> {views_path}")

    print("\n[OK] All reference zero position simulation screenshots rendered successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
