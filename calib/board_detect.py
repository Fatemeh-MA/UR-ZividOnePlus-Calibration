"""Fallback calibration-board detection + hand-eye solver for the Zivid One+ (SDK 2.11).

Why: zivid.calibration.detect_feature_points() (SDK 2.11) fails when something (e.g. the robot
wrist) is close to the board border, even when all squares are clearly visible.

Method:
  1. OpenCV finds the inner checkerboard corners in the 2D color image (sub-pixel).
  2. The 3D position of every corner is read from the Zivid point cloud (bilinear, mm).
  3. Board pose in the camera = rigid fit (Kabsch) of the board model to those 3D points.
  4. Hand-eye: OpenCV calibrateHandEye for a first guess, then a least-squares refinement that
     minimises the 3D distance of all corners over all poses.

Board: Zivid ZVD-CB01, 7 x 8 squares of 30 mm  ->  6 x 7 inner corners.
"""

from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

SQUARE_MM = 30.0
INNER = (6, 7)  # inner corners (per row, rows) for a 7 x 8 square board


# --------------------------------------------------------------------------- detection
def _bilinear_xyz(xyz: np.ndarray, uv: np.ndarray) -> np.ndarray:
    h, w = xyz.shape[:2]
    out = np.full((len(uv), 3), np.nan)
    for k, (u, v) in enumerate(uv):
        x0, y0 = int(np.floor(u)), int(np.floor(v))
        if x0 < 0 or y0 < 0 or x0 + 1 >= w or y0 + 1 >= h:
            continue
        a, b = u - x0, v - y0
        p = xyz[y0 : y0 + 2, x0 : x0 + 2].reshape(4, 3)
        wts = np.array([(1 - a) * (1 - b), a * (1 - b), (1 - a) * b, a * b])
        ok = ~np.isnan(p[:, 2])
        if ok.sum() >= 3:
            out[k] = (p[ok] * wts[ok, None]).sum(0) / wts[ok].sum()
    return out


def _mean_gray(gray: np.ndarray, pt: np.ndarray, r: int) -> float:
    x, y = int(round(pt[0])), int(round(pt[1]))
    patch = gray[max(y - r, 0) : y + r + 1, max(x - r, 0) : x + r + 1]
    return float(patch.mean()) if patch.size else np.nan


def find_corners(rgba: np.ndarray) -> Optional[np.ndarray]:
    """Return (42, 2) sub-pixel corners in a canonical order, or None."""
    gray = cv2.cvtColor(np.ascontiguousarray(rgba[:, :, :3]), cv2.COLOR_RGB2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
    c = None
    # Try from most accurate to most tolerant; CALIB_CB_ACCURACY fails on some rotated/small views.
    for img in (gray, clahe):
        for flags in (
            cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
            cv2.CALIB_CB_EXHAUSTIVE,
            cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_NORMALIZE_IMAGE,
        ):
            ok, cc = cv2.findChessboardCornersSB(img, INNER, flags=flags)
            if ok:
                c = cc
                break
        if c is not None:
            break
    if c is None:
        return None
    c = c.reshape(-1, 2).astype(np.float32)
    # Sub-pixel refinement on the original image (window ~1/4 square)
    win = max(3, int(0.2 * np.linalg.norm(c[1] - c[0])))
    cv2.cornerSubPix(gray, c.reshape(-1, 1, 2), (win, win), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01))
    c = c.reshape(-1, 2).astype(np.float64)
    cols = INNER[0]
    # Canonical order (removes OpenCV's 180 deg ambiguity): the square between the first
    # four corners must be white. On a 7x8 board the opposite corner square is black.
    q = (c[0] + c[1] + c[cols] + c[cols + 1]) / 4.0
    q_last = (c[-1] + c[-2] + c[-1 - cols] + c[-2 - cols]) / 4.0
    r = max(2, int(0.15 * np.linalg.norm(c[1] - c[0])))
    if _mean_gray(gray, q, r) < _mean_gray(gray, q_last, r):
        c = c[::-1].copy()
    return c


def board_model() -> np.ndarray:
    cols, rows = INNER
    return np.array([[i * SQUARE_MM, j * SQUARE_MM, 0.0] for j in range(rows) for i in range(cols)])


def kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """4x4 T with dst ~ T @ src."""
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cd - R @ cs
    return T


def detect_board(rgba: np.ndarray, xyz: np.ndarray) -> dict:
    """Detect the board. Returns dict(valid, reason, corners2d, points3d, T_cam_board, fit_rms)."""
    res = {"valid": False, "reason": "", "corners2d": None, "points3d": None, "T_cam_board": None, "fit_rms": None}
    c = find_corners(rgba)
    if c is None:
        res["reason"] = "checkerboard grid not found in 2D image"
        return res
    P = _bilinear_xyz(xyz, c)
    ok = ~np.isnan(P[:, 2])
    if ok.sum() < 0.8 * len(P):
        res["reason"] = f"only {ok.sum()}/{len(P)} corners have 3D data"
        return res
    M = board_model()
    T = kabsch(M[ok], P[ok])
    err = np.linalg.norm((T[:3, :3] @ M[ok].T).T + T[:3, 3] - P[ok], axis=1)
    rms = float(np.sqrt((err**2).mean()))
    if rms > 2.0:
        res["reason"] = f"3D fit too poor ({rms:.2f} mm RMS) - wrong corner order or bad 3D data"
        return res
    res.update(valid=True, corners2d=c, points3d=P, T_cam_board=T, fit_rms=rms)
    return res


def detect_board_in_frame(frame) -> dict:
    pc = frame.point_cloud()
    return detect_board(pc.copy_data("rgba"), pc.copy_data("xyz"))


# --------------------------------------------------------------------------- hand-eye
def _to_T(x: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix()
    T[:3, 3] = x[3:]
    return T


def _from_T(T: np.ndarray) -> np.ndarray:
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def _inv(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def solve_hand_eye(
    robot_poses: List[np.ndarray], detections: List[dict], mode: str
) -> Tuple[np.ndarray, List[Tuple[float, float]], float]:
    """robot_poses: T_base_flange (mm). mode 'eye-to-hand' -> returns T_base_cam;
    'eye-in-hand' -> returns T_flange_cam. Also per-pose residuals (deg, mm) and corner RMS (mm)."""
    if len(robot_poses) < 3:
        raise RuntimeError(f"Need at least 3 valid poses, got {len(robot_poses)}")
    M = board_model()
    Tcb = [d["T_cam_board"] for d in detections]
    eth = mode == "eye-to-hand"
    # OpenCV initial guess (eye-to-hand: feed base->gripper, get cam->base)
    A = [_inv(T) if eth else T for T in robot_poses]
    pts = [d["points3d"] for d in detections]

    def _y_from_x(X):
        Ys = [(_inv(Tb) @ X @ Tc) if eth else (Tb @ X @ Tc) for Tb, Tc in zip(robot_poses, Tcb)]
        Y = np.eye(4)
        Y[:3, :3] = Rotation.from_matrix(np.array([Yi[:3, :3] for Yi in Ys])).mean().as_matrix()
        Y[:3, 3] = np.mean([Yi[:3, 3] for Yi in Ys], axis=0)
        return Y, Ys

    # Initial guess: try all OpenCV hand-eye methods, keep the most consistent finite one.
    best = None
    for method in (cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_HORAUD,
                   cv2.CALIB_HAND_EYE_DANIILIDIS, cv2.CALIB_HAND_EYE_ANDREFF):
        try:
            R_cv, t_cv = cv2.calibrateHandEye(
                [T[:3, :3] for T in A], [T[:3, 3] for T in A],
                [T[:3, :3] for T in Tcb], [T[:3, 3] for T in Tcb], method=method,
            )
        except cv2.error:
            continue
        if not (np.all(np.isfinite(R_cv)) and np.all(np.isfinite(t_cv))):
            continue
        X = np.eye(4)
        X[:3, :3], X[:3, 3] = R_cv, t_cv.ravel()
        try:
            Y, Ys = _y_from_x(X)
        except (ValueError, np.linalg.LinAlgError):
            continue
        spread = float(np.mean([np.linalg.norm(Yi[:3, 3] - Y[:3, 3]) for Yi in Ys]))
        if best is None or spread < best[0]:
            best = (spread, X, Y)
    if best is None:
        raise RuntimeError(
            "Hand-eye has no solution for these poses. Check that --eth/--eih is right and that the "
            "poses differ in rotation (tilt the board in several directions)."
        )
    _, X0, Y0 = best

    def resid(p):
        X, Y = _to_T(p[:6]), _to_T(p[6:])
        out = []
        for Tb, P in zip(robot_poses, pts):
            ok = ~np.isnan(P[:, 2])
            if eth:  # X @ P_cam = Tb @ Y @ M
                lhs = (X[:3, :3] @ P[ok].T).T + X[:3, 3]
                W = Tb @ Y
            else:  # Tb @ X @ P_cam = Y @ M
                W0 = Tb @ X
                lhs = (W0[:3, :3] @ P[ok].T).T + W0[:3, 3]
                W = Y
            rhs = (W[:3, :3] @ M[ok].T).T + W[:3, 3]
            out.append((lhs - rhs).ravel())
        return np.concatenate(out)

    sol = least_squares(resid, np.r_[_from_T(X0), _from_T(Y0)], loss="soft_l1", f_scale=1.0)
    X, Y = _to_T(sol.x[:6]), _to_T(sol.x[6:])
    r = resid(sol.x).reshape(-1, 3)
    rms = float(np.sqrt((np.linalg.norm(r, axis=1) ** 2).mean()))

    residuals = []
    for Tb, Tc in zip(robot_poses, Tcb):
        pred = _inv(Tb) @ X @ Tc if eth else Tb @ X @ Tc  # should equal Y
        dR = Rotation.from_matrix(Y[:3, :3].T @ pred[:3, :3]).magnitude()
        residuals.append((float(np.degrees(dR)), float(np.linalg.norm(pred[:3, 3] - Y[:3, 3]))))
    solve_hand_eye.last_board_transform = Y  # board in flange (eth) / board in base (eih), mm
    return X, residuals, rms


def load_pose_yaml(path: Path) -> np.ndarray:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    m = fs.getNode("FloatMatrix").mat()
    fs.release()
    if m is None:
        import zivid

        m = np.array(zivid.Matrix4x4(str(path)))
    return np.asarray(m, dtype=np.float64)
