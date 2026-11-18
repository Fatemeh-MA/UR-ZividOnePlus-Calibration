"""Verify a hand-eye calibration (eye-to-hand: camera fixed, board on the robot flange).

The robot is never moved by this script; it only READS the robot pose over RTDE.
Use the same TCP setting on the pendant as during calibration.

Test A  -  new pose (board on the flange):
    Move the robot (by hand / pendant) to a pose that was NOT used for calibration, board visible.
    python3 verify_calibration.py pose datasets/<date> --ip 192.168.163.11 --settings ~/zivid_oneplus_data/oneplus_3d.yml
    -> compares where the calibration predicts the board with where the camera sees it.
       Repeat for 3-5 different poses.

Test B  -  touch test (board lying flat on the table):
    python3 verify_calibration.py touch datasets/<date> --settings ~/zivid_oneplus_data/oneplus_3d.yml
    -> prints the 4 outer grid corners in ROBOT BASE coordinates. Jog a pointed tool (TCP set to the
       tip) to a corner and compare with the pendant (Base frame).
"""

import argparse
from pathlib import Path

import numpy as np
import zivid
from scipy.spatial.transform import Rotation

import board_detect as bd


def _load(path: Path) -> np.ndarray:
    return np.array(zivid.Matrix4x4(str(path)))


_APP = None  # created once in main(); needed before loading matrices and for the camera


def _capture(settings_path):
    app = _APP
    camera = app.connect_camera()
    settings = zivid.Settings.load(str(settings_path))
    frame = camera.capture(settings)
    det = bd.detect_board_in_frame(frame)
    if not det["valid"]:
        raise SystemExit(f"Board not detected: {det['reason']}")
    return app, frame, det


def _board_in_flange_for(X: np.ndarray, dataset: Path, transform_name: str) -> np.ndarray:
    """Board pose in the flange that is consistent with transform X (so every transform is tested fairly).
    Estimated from the calibration dataset (board detections + robot poses), cached next to it."""
    cache = dataset / f"boardInFlange_for_{Path(transform_name).stem}.yaml"
    if cache.is_file():
        return _load(cache)
    print(f"Estimating board-in-flange for {transform_name} from the dataset (once) ...")
    Ys = []
    i = 1
    while (dataset / f"img{i:02d}.zdf").is_file():
        det = bd.detect_board_in_frame(zivid.Frame(str(dataset / f"img{i:02d}.zdf")))
        if det["valid"]:
            Tb = _load(dataset / f"pos{i:02d}.yaml")
            Ys.append(bd._inv(Tb) @ X @ det["T_cam_board"])
        i += 1
    if not Ys:
        raise SystemExit("No usable poses in dataset")
    Y = np.eye(4)
    Y[:3, :3] = Rotation.from_matrix(np.array([y[:3, :3] for y in Ys])).mean().as_matrix()
    Y[:3, 3] = np.mean([y[:3, 3] for y in Ys], axis=0)
    zivid.Matrix4x4(Y).save(str(cache))
    return Y


def _robot_pose_mm(ip: str) -> np.ndarray:
    from rtde import rtde

    con = rtde.RTDE(ip, 30004)
    con.connect()
    con.negotiate_protocol_version()
    if not con.send_output_setup(["actual_TCP_pose"], ["VECTOR6D"], frequency=10):
        raise SystemExit("RTDE output setup failed")
    con.send_start()
    p = np.array(con.receive().actual_TCP_pose)
    con.send_pause()
    con.disconnect()
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(p[3:]).as_matrix()
    T[:3, 3] = p[:3] * 1000.0
    return T


def test_pose(args) -> None:
    X = _load(args.dataset / args.transform)  # T_base_cam
    Y = _board_in_flange_for(X, args.dataset, args.transform)  # T_flange_board consistent with X
    T_base_flange = _robot_pose_mm(args.ip)
    _app, _frame, det = _capture(args.settings)

    M = bd.board_model()
    measured = (X[:3, :3] @ det["points3d"].T).T + X[:3, 3]  # camera points -> base
    W = T_base_flange @ Y
    predicted = (W[:3, :3] @ M.T).T + W[:3, 3]
    ok = ~np.isnan(measured[:, 2])
    err = np.linalg.norm(measured[ok] - predicted[ok], axis=1)

    T_meas = X @ det["T_cam_board"]
    rot_err = np.degrees(Rotation.from_matrix(W[:3, :3].T @ T_meas[:3, :3]).magnitude())

    print(f"\nUsing {args.transform}")
    print(f"Robot TCP (base, mm): {T_base_flange[:3, 3].round(1)}")
    print(f"Board corner error, camera vs robot prediction:  mean {err.mean():.2f} mm   max {err.max():.2f} mm")
    print(f"Board orientation error: {rot_err:.3f} deg")
    verdict = "GOOD" if err.mean() < 2 else ("OK" if err.mean() < 5 else "POOR - recalibrate")
    print(f"=> {verdict}  (typical good result with a Zivid One+ at ~1 m: 1-2 mm)")


def test_touch(args) -> None:
    X = _load(args.dataset / args.transform)
    _app, _frame, det = _capture(args.settings)
    P = det["points3d"]
    cols, rows = bd.INNER
    names = {0: "first corner", cols - 1: "end of first row", len(P) - cols: "start of last row", len(P) - 1: "last corner"}
    print(f"\nUsing {args.transform}. Outer INNER corners of the grid, in ROBOT BASE coordinates:")
    print("(These are corners where 4 squares meet, one square in from the edge of the grid.)\n")
    for k, name in names.items():
        pb = X[:3, :3] @ P[k] + X[:3, 3]
        print(f"  {name:18s} x={pb[0]:8.1f}  y={pb[1]:8.1f}  z={pb[2]:8.1f} mm   (pendant: {pb[0]/1000:.4f}, {pb[1]/1000:.4f}, {pb[2]/1000:.4f} m)")
    print("\nThe 'first corner' is the one next to the WHITE square of the first row (see test_capture image).")
    print("Jog the tool tip to it: the difference to the pendant reading is your calibration error.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=["pose", "touch"])
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--settings", type=Path, required=True)
    ap.add_argument("--ip", default="192.168.163.11")
    ap.add_argument("--transform", default="handEyeTransform.yaml",
                    help="handEyeTransform.yaml (default) or handEyeTransform_fallback.yaml")
    args = ap.parse_args()
    global _APP
    _APP = zivid.Application()
    (test_pose if args.test == "pose" else test_touch)(args)


if __name__ == "__main__":
    main()
