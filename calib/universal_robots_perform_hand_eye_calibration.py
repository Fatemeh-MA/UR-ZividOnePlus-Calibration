"""Zivid One+ (SDK 2.11) adaptation of the Zivid UR hand-eye sample."""
"""
Script to generate a dataset and perform hand-eye calibration using a Universal Robot UR5e robot.
The script communicates with the robot through Real-Time Data Exchange (RTDE) interface.
More information about RTDE:
https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/

The entire sample consist of two additional files:
    - universal_robots_hand_eye_script.urp: Robot program script that moves between different poses.
    - robot_communication_file.xml: communication set-up file.

Running the sample requires that you have universal_robots_hand_eye_script.urp on your UR5e robot,
and robot_communication_file.xml in the same repo as this sample. Each robot pose
must be modified to your scene. This is done in universal_robots_hand_eye_script.urp on the robot.

Further explanation of this sample is found in our knowledge base:
https://support.zivid.com/latest/academy/applications/hand-eye/ur5-robot-%2B-python-generate-dataset-and-perform-hand-eye-calibration.html

"""

import argparse
import datetime
import time
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
import zivid
from rtde import rtde, rtde_config
from scipy.spatial.transform import Rotation
import board_detect
from zividsamples.save_load_matrix import assert_affine_matrix_and_save, load_and_assert_affine_matrix


def _options() -> argparse.Namespace:
    """Function for taking in arguments from user.

    Returns:
        Arguments from user

    """
    parser = argparse.ArgumentParser(description=__doc__)
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument("--eih", "--eye-in-hand", action="store_true", help="eye-in-hand calibration")
    mode_group.add_argument("--eth", "--eye-to-hand", action="store_true", help="eye-to-hand calibration")
    parser.add_argument("--ip", required=True, help="IP address to robot")

    subparsers = parser.add_subparsers(dest="calibration_object", required=True, help="Calibration object type")
    subparsers.add_parser("checkerboard", help="Use checkerboard for calibration")
    # Zivid One+ / SDK 2.11: ArUco marker calibration is not available in zivid-python 2.11.
    parser.add_argument(
        "--settings",
        type=Path,
        default=None,
        help="Zivid settings .yml made for the One+ (e.g. exported from Zivid Studio 2.11). "
        "If omitted, the Capture Assistant suggests settings once at start.",
    )

    return parser.parse_args()


def _write_robot_state(
    con: rtde.RTDE,
    input_data: rtde.serialize.DataObject,
    finish_capture: bool = False,
    camera_ready: bool = False,
) -> None:
    """Write to robot I/O registers.

    Args:
        con: Connection between computer and robot
        input_data: Input package containing the specific input data registers
        finish_capture: Boolean value to robot_state that q_r scene capture is finished
        camera_ready: Boolean value to robot_state that camera is ready to capture images

    """
    input_data.input_bit_register_64 = int(finish_capture)
    input_data.input_bit_register_65 = int(camera_ready)

    con.send(input_data)


def _initialize_robot_sync(host: str) -> Tuple[rtde.RTDE, rtde.serialize.DataObject]:
    """Set up communication with UR robot.

    Args:
        host: IP address

    Returns:
        con: Connection to robot
        robot_input_data: Package containing the specific input data registers

    Raises:
        RuntimeError: If protocol do not match
        RuntimeError: If script is unable to configure output
        RuntimeError: If synchronization is not possible

    """
    conf = rtde_config.ConfigFile(Path(Path.cwd() / "universal_robots_communication_file.xml"))
    output_names, output_types = conf.get_recipe("out")
    input_names, input_types = conf.get_recipe("in")

    # port 30004 is reserved for rtde
    con = rtde.RTDE(host, 30004)
    con.connect()

    # To ensure that the application is compatible with further versions of UR controller
    if not con.negotiate_protocol_version():
        raise RuntimeError("Protocol do not match")

    # One+ fix: 200 Hz overflowed this 10 Hz reader -> robot reset the connection
    if not con.send_output_setup(output_names, output_types, frequency=10):
        raise RuntimeError("Unable to configure output")

    robot_input_data = con.send_input_setup(input_names, input_types)

    if not con.send_start():
        raise RuntimeError("Unable to start synchronization")

    print("Communication initialization completed. \n")

    return con, robot_input_data


def _save_zdf_and_pose(save_dir: Path, image_num: int, frame: zivid.Frame, transform: np.ndarray) -> None:
    """Save data to folder.

    Args:
        save_dir: Directory to save data
        image_num: Image number
        frame: Point cloud stored as ZDF
        transform: Transformation matrix (4x4)

    """
    frame.save(save_dir / f"img{image_num:02d}.zdf")

    assert_affine_matrix_and_save(transform, save_dir / f"pos{image_num:02d}.yaml")


def _generate_folder() -> Path:
    """Generate folder where the dataset will be stored.

    Returns:
        Path: Location_dir to where the data will be saved

    """
    location_dir = Path.cwd() / "datasets" / datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    if not location_dir.is_dir():
        location_dir.mkdir(parents=True)

    return location_dir


def _get_frame_and_transform_matrix(
    con: rtde,
    camera: zivid.Camera,
    settings: zivid.Settings,
) -> Tuple[zivid.Frame, np.ndarray]:
    """Capture image with Zivid camera and read robot pose.

    Args:
        con: Connection between computer and robot
        camera: Zivid camera
        settings: Zivid settings

    Returns:
        frame: Zivid frame
        transform: Transformation matrix (4x4)

    """
    frame = camera.capture(settings)
    robot_pose = np.array(con.receive().actual_TCP_pose)

    translation = robot_pose[:3] * 1000
    rotation_vector = robot_pose[3:]
    rotation = Rotation.from_rotvec(rotation_vector)
    transform = np.eye(4)
    transform[:3, :3] = rotation.as_matrix()
    transform[:3, 3] = translation.T

    return frame, transform


_SETTINGS_CACHE = {}
_SETTINGS_PATH = None


def _camera_settings(camera: zivid.Camera) -> zivid.Settings:
    """Set camera settings.

    Args:
        camera: Zivid camera

    Returns:
        settings: Zivid Settings

    """
    # suggest_settings_parameters = zivid.capture_assistant.SuggestSettingsParameters(
    #     max_capture_time=datetime.timedelta(milliseconds=1200),
    #     ambient_light_frequency=zivid.capture_assistant.SuggestSettingsParameters.AmbientLightFrequency.none,
    # )
    #
    # settings = zivid.capture_assistant.suggest_settings(camera, suggest_settings_parameters)
    if "settings" not in _SETTINGS_CACHE:
        if _SETTINGS_PATH is not None:
            print(f"Loading camera settings from {_SETTINGS_PATH}")
            _SETTINGS_CACHE["settings"] = zivid.Settings.load(str(_SETTINGS_PATH))
        else:
            print("Capture Assistant: suggesting settings for the current scene ...")
            suggest_settings_parameters = zivid.capture_assistant.SuggestSettingsParameters(
                max_capture_time=datetime.timedelta(milliseconds=1200),
                ambient_light_frequency=zivid.capture_assistant.SuggestSettingsParameters.AmbientLightFrequency.none,
            )
            _SETTINGS_CACHE["settings"] = zivid.capture_assistant.suggest_settings(camera, suggest_settings_parameters)
    return _SETTINGS_CACHE["settings"]
    


def _read_robot_state(con: rtde.RTDE) -> rtde.serialize:
    """Receive robot output recipe.

    Args:
        con: Connection between computer and robot

    Returns:
        robot_state: Robot state

    """
    robot_state = con.receive()

    assert robot_state is not None, "Not able to receive robot_state"

    return robot_state


def _save_hand_eye_results(save_dir: Path, transform: np.ndarray, residuals: List) -> None:
    """Save transformation and residuals to folder.

    Args:
        save_dir: Path to where data will be saved
        transform: Transformation matrix (4x4)
        residuals: List of residuals

    """
    assert_affine_matrix_and_save(transform, save_dir / "handEyeTransform.yaml")

    _write_residuals(save_dir / "residuals.yaml", residuals)


def _write_residuals(path: Path, residuals: List) -> None:
    file_storage_residuals = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    file_storage_residuals.write(
        "Per pose residuals for rotation in deg and translation in mm",
        np.array([list(r) for r in residuals], dtype=np.float64),
    )
    file_storage_residuals.release()


def _image_count(robot_state: rtde.serialize.DataObject) -> int:
    """Read robot output register 24.

    Args:
        robot_state: Robot state

    Returns:
        Number of captured images

    """
    return robot_state.output_int_register_24


def _ready_for_capture(robot_state: rtde.serialize.DataObject) -> bool:
    """Read robot output register 64.

    Args:
        robot_state: Robot state

    Returns:
        Boolean value that states if camera is ready to capture

    """
    return robot_state.output_bit_register_64


def _verify_good_capture(frame: zivid.Frame, user_options: argparse.Namespace) -> None:
    """Verify that calibration object feature points are detected in the frame."""
    if user_options.calibration_object == "checkerboard":
        zivid_ok = zivid.calibration.detect_feature_points(frame.point_cloud()).valid()
        fb = board_detect.detect_board_in_frame(frame)
        if zivid_ok:
            print("Checkerboard detected (Zivid detector).")
        elif fb["valid"]:
            print(f"Checkerboard detected (OpenCV+3D fallback, fit {fb['fit_rms']:.2f} mm); Zivid detector failed.")
        else:
            print(f"WARNING: Checkerboard NOT detected for this pose ({fb['reason']}). Continuing anyway...")


def _capture_one_frame_and_robot_pose(
    *,
    con: rtde.RTDE,
    camera: zivid.Camera,
    save_dir: Path,
    input_data: rtde.serialize.DataObject,
    image_num: int,
    ready_to_capture: bool,
    user_options: argparse.Namespace,
) -> None:
    """Capture 3D image and robot pose for a given robot posture,
    then signals robot to move to next posture.

    Args:
        con: Connection between computer and robot
        camera: Zivid camera
        save_dir: Path to where data will be saved
        input_data: Input package containing the specific input data registers
        image_num: Image number
        ready_to_capture: Boolean value to robot_state that camera is ready to capture images
        user_options: Input arguments

    """
    settings = _camera_settings(camera)
    frame, transform = _get_frame_and_transform_matrix(con, camera, settings)
    _verify_good_capture(frame, user_options)

    # Signal robot to move to next position, then set signal to low again.
    _write_robot_state(con, input_data, finish_capture=True, camera_ready=ready_to_capture)
    time.sleep(0.1)
    _write_robot_state(con, input_data, finish_capture=False, camera_ready=ready_to_capture)
    _save_zdf_and_pose(save_dir, image_num, frame, transform)
    print("Image and pose saved")


def _generate_dataset(
    app: zivid.Application, con: rtde.RTDE, input_data: rtde.serialize.DataObject, user_options: argparse.Namespace
) -> Path:
    """Generate dataset based on predefined robot poses.

    Args:
        app: Zivid application instance
        con: Connection between computer and robot
        input_data: Input package containing the specific input data registers
        user_options: Input arguments

    Returns:
        Path: Save_dir to where dataset is saved

    """
    camera = app.connect_camera()
    save_dir = _generate_folder()

    # Signal robot that camera is ready
    ready_to_capture = True
    _write_robot_state(con, input_data, finish_capture=False, camera_ready=ready_to_capture)

    robot_state = _read_robot_state(con)

    print(
        "Initial output robot_states: \n"
        f"Image count: {_image_count(robot_state)} \n"
        f"Ready for capture: {_ready_for_capture(robot_state)}\n"
    )

    images_captured = 1
    while _image_count(robot_state) != -1:
        robot_state = _read_robot_state(con)

        if _ready_for_capture(robot_state) and images_captured == _image_count(robot_state):
            print(f"Capture image {_image_count(robot_state)}")
            _capture_one_frame_and_robot_pose(
                con=con,
                camera=camera,
                save_dir=save_dir,
                input_data=input_data,
                image_num=images_captured,
                ready_to_capture=ready_to_capture,
                user_options=user_options,
            )
            images_captured += 1
        # no sleep: con.receive() blocks until the next 10 Hz package, so we never fall behind

    _write_robot_state(con, input_data, finish_capture=False, camera_ready=False)
    time.sleep(1.0)
    con.send_pause()
    con.disconnect()

    print(f"\n Data saved to: {save_dir}")

    return save_dir


def perform_hand_eye_calibration(
    mode: str,
    data_dir: Path,
) -> Tuple[np.ndarray, List[zivid.calibration.HandEyeResidual]]:
    """Perform had-eye calibration based on mode.

    Args:
        mode: Calibration mode, eye-in-hand or eye-to-hand
        data_dir: Path to dataset

    Returns:
        transform: Transformation matrix (4x4)
        residuals: List of residuals

    Raises:
        RuntimeError: If no feature points are detected
        ValueError: If calibration mode is invalid

    """
    calibration_inputs = []
    fb_poses, fb_dets = [], []
    idata = 1
    while True:
        frame_file_path = data_dir / f"img{idata:02d}.zdf"
        pose_file_path = data_dir / f"pos{idata:02d}.yaml"
        if not (frame_file_path.is_file() and pose_file_path.is_file()):
            break

        frame = zivid.Frame(frame_file_path)
        pose_matrix = load_and_assert_affine_matrix(pose_file_path)

        detection_result = zivid.calibration.detect_feature_points(frame.point_cloud())
        fb = board_detect.detect_board_in_frame(frame)
        print(
            f"img{idata:02d}: Zivid detector {'OK' if detection_result.valid() else '--'}, "
            f"fallback {'OK (fit %.2f mm)' % fb['fit_rms'] if fb['valid'] else '-- ' + fb['reason']}"
        )
        if detection_result.valid():
            calibration_inputs.append(
                zivid.calibration.HandEyeInput(zivid.calibration.Pose(pose_matrix), detection_result)
            )
        if fb["valid"]:
            fb_poses.append(pose_matrix)
            fb_dets.append(fb)
        idata += 1

    np.set_printoptions(precision=5, suppress=True)
    transform, residuals = None, None
    print(f"\nZivid detector: {len(calibration_inputs)} valid poses; fallback: {len(fb_dets)} valid poses")

    if len(calibration_inputs) >= 3:
        try:
            print(f"\nPerform {mode} calibration (Zivid)")
            if mode == "eye-in-hand":
                calibration_result = zivid.calibration.calibrate_eye_in_hand(calibration_inputs)
            elif mode == "eye-to-hand":
                calibration_result = zivid.calibration.calibrate_eye_to_hand(calibration_inputs)
            else:
                raise ValueError(f"Invalid calibration mode: {mode}")
            if calibration_result.valid():
                transform = calibration_result.transform()
                residuals = [(r.rotation(), r.translation()) for r in calibration_result.residuals()]
                print("Zivid transform:\n", transform)
                for r in residuals:
                    print(f"  Rotation: {r[0]:.6f} deg   Translation: {r[1]:.6f} mm")
            else:
                print("Zivid calibration returned an invalid result.")
        except RuntimeError as ex:
            print(f"Zivid calibration failed: {ex}")

    if len(fb_dets) >= 3:
        print(f"\nPerform {mode} calibration (OpenCV+3D fallback)")
        fb_T, fb_res, fb_rms = board_detect.solve_hand_eye(fb_poses, fb_dets, mode)
        print("Fallback transform:\n", fb_T)
        print(f"  corner RMS over all poses: {fb_rms:.3f} mm")
        for r in fb_res:
            print(f"  Rotation: {r[0]:.6f} deg   Translation: {r[1]:.6f} mm")
        assert_affine_matrix_and_save(fb_T, data_dir / "handEyeTransform_fallback.yaml")
        assert_affine_matrix_and_save(
            board_detect.solve_hand_eye.last_board_transform,
            data_dir / ("boardInFlange_fallback.yaml" if mode == "eye-to-hand" else "boardInBase_fallback.yaml"),
        )
        _write_residuals(data_dir / "residuals_fallback.yaml", fb_res)
        if transform is None:
            print("\nUsing the fallback result as handEyeTransform.yaml (Zivid calibration not available).")
            transform, residuals = fb_T, fb_res
        else:
            d_t = np.linalg.norm(np.asarray(transform)[:3, 3] - fb_T[:3, 3])
            print(f"\nZivid vs fallback translation difference: {d_t:.2f} mm")

    if transform is None:
        raise RuntimeError("Not enough poses with a detected checkerboard (need at least 3).")
    return transform, residuals


def _main() -> None:
    app = zivid.Application()
    user_options = _options()
    global _SETTINGS_PATH
    _SETTINGS_PATH = user_options.settings

    robot_ip_address = user_options.ip
    con, input_data = _initialize_robot_sync(robot_ip_address)
    con.send_start()

    dataset_dir = _generate_dataset(app, con, input_data, user_options)

    if user_options.eih:
        transform, residuals = perform_hand_eye_calibration("eye-in-hand", dataset_dir)
    else:
        transform, residuals = perform_hand_eye_calibration("eye-to-hand", dataset_dir)

    _save_hand_eye_results(dataset_dir, transform, residuals)


if __name__ == "__main__":
    _main()
