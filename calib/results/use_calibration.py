"""Convert points from the Zivid One+ camera frame to the UR robot base frame.

Needs only numpy + pyyaml (no Zivid SDK):   pip install numpy pyyaml

    python3 use_calibration.py                        # demo with an example point
    python3 use_calibration.py 12.5 -40.0 1080.0      # your own camera point (x y z in mm)

In your own code:
    from use_calibration import load_hand_eye, camera_to_base
    T = load_hand_eye("handEyeTransform_oneplus_2026-11-18.yaml")
    p_base_mm = camera_to_base(T, [x_mm, y_mm, z_mm])
"""

import sys
from pathlib import Path

import numpy as np
import yaml

DEFAULT_FILE = Path(__file__).with_name("handEyeTransform_oneplus_2026-11-18.yaml")


def load_hand_eye(path=DEFAULT_FILE) -> np.ndarray:
    """4x4 matrix T_base_camera (translation in millimetres)."""
    with open(path) as f:
        data = yaml.safe_load(f)
    T = np.array(data["FloatMatrix"]["Data"], dtype=float)
    assert T.shape == (4, 4), "unexpected file format"
    return T


def camera_to_base(T: np.ndarray, points_mm) -> np.ndarray:
    """Point(s) in the camera frame [mm] -> robot base frame [mm]. Accepts (3,) or (N, 3)."""
    p = np.atleast_2d(np.asarray(points_mm, dtype=float))
    out = (T[:3, :3] @ p.T).T + T[:3, 3]
    return out[0] if np.ndim(points_mm) == 1 else out


def camera_pose_to_base(T: np.ndarray, T_cam_obj: np.ndarray) -> np.ndarray:
    """Object pose (4x4, mm) in the camera frame -> object pose in the robot base frame."""
    return T @ T_cam_obj


if __name__ == "__main__":
    T = load_hand_eye()
    np.set_printoptions(precision=4, suppress=True)
    print("T_base_camera (mm):\n", T)
    print("Camera position in robot base (mm):", T[:3, 3])
    p_cam = [float(v) for v in sys.argv[1:4]] if len(sys.argv) >= 4 else [0.0, 0.0, 1100.0]
    p_base = camera_to_base(T, p_cam)
    print(f"\nCamera point {p_cam} mm")
    print(f"-> robot base {p_base.round(2)} mm  = {(p_base / 1000).round(5)} m (UR pendant / URScript use metres)")
