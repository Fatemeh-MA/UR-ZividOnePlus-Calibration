"""Capture one frame with the Zivid One+ and check that the calibration checkerboard is detected.

Usage (inside the container, from /work):
    python3 test_capture.py                       # Capture Assistant settings
    python3 test_capture.py --settings my.yml     # settings exported from Zivid Studio
Saves test_capture.zdf (open it in Zivid Studio) in the current folder.
"""

import argparse
import datetime
from pathlib import Path

import zivid

import board_detect


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings", type=Path, default=None)
    args = parser.parse_args()

    app = zivid.Application()
    camera = app.connect_camera()
    print(f"Connected: {camera.info.model_name} (serial {camera.info.serial_number})")

    if args.settings:
        settings = zivid.Settings.load(str(args.settings))
    else:
        print("Capture Assistant: suggesting settings ...")
        settings = zivid.capture_assistant.suggest_settings(
            camera,
            zivid.capture_assistant.SuggestSettingsParameters(
                max_capture_time=datetime.timedelta(milliseconds=1200),
                ambient_light_frequency=zivid.capture_assistant.SuggestSettingsParameters.AmbientLightFrequency.none,
            ),
        )
        settings.save("suggested_settings.yml")
        print("Suggested settings saved to suggested_settings.yml")

    frame = camera.capture(settings)
    frame.save("test_capture.zdf")
    print("Saved test_capture.zdf")

    detection = zivid.calibration.detect_feature_points(frame.point_cloud())
    if detection.valid():
        print(f"Zivid detector:    DETECTED, centroid (mm): {detection.centroid()}")
    else:
        print("Zivid detector:    not detected")

    fb = board_detect.detect_board_in_frame(frame)
    if fb["valid"]:
        t = fb["T_cam_board"][:3, 3]
        print(f"OpenCV+3D fallback: DETECTED, board origin (mm): {t.round(1)}, 3D fit {fb['fit_rms']:.2f} mm")
    else:
        print(f"OpenCV+3D fallback: not detected ({fb['reason']})")

    if detection.valid() or fb["valid"]:
        print("=> This pose can be used for calibration.")
    else:
        print("=> NOT usable: the whole 7x8 grid must be visible, flat, well lit.")


if __name__ == "__main__":
    main()
