# Zivid One+ in Docker (SDK 2.11.1) + UR hand-eye calibration

Run the **Zivid One+** camera (here a One+ M) on a modern Ubuntu machine **without touching the host's own Zivid SDK**,
and calibrate it to a **Universal Robots** arm (eye-to-hand).

Why Docker: Zivid dropped One+ support after **SDK 2.11** (2.12+ cannot see the camera), while newer Zivid cameras
and the current `zivid-ros` need **SDK ≥ 2.15**. apt allows only one Zivid SDK version per machine and conda cannot
isolate system libraries, so the One+ SDK lives only inside Docker images. The host keeps its newer SDK untouched.

**Current calibration result:** [`calib/results/RESULT.md`](calib/results/RESULT.md)
(`T_base_camera`, mean error ~0.3–0.6 mm on new poses).

---

## Contents

```
.
├── Dockerfile                 image zivid-oneplus:2.11.1  (SDK 2.11.1, Teli USB driver, Zivid Studio, tools)
├── download_packages.sh       downloads the 4 Zivid .deb packages (not in git)
├── run.sh                     start Studio / tools in the container (USB + GPU + your home folder)
├── export_images.sh           save both images to one .tar.gz for offline transfer
├── host/
│   ├── 99-zivid.rules         udev rule: USB permissions for the One+
│   └── setup_host.sh          one-time host setup (sudo)
└── calib/
    ├── Dockerfile             image zivid-oneplus-calib:2.11.1 (adds zivid-python 2.11.1, OpenCV, rtde)
    ├── calib.sh               run any calibration script in the container (folder mounted as /work)
    ├── universal_robots_perform_hand_eye_calibration.py   Zivid UR sample, adapted to SDK 2.11 (see below)
    ├── universal_robots_robot_motion_script.urp           robot program (teach your waypoints in it)
    ├── universal_robots_communication_file.xml            RTDE recipe
    ├── board_detect.py        fallback board detection (OpenCV corners + Zivid 3D) + hand-eye solver
    ├── test_capture.py        capture once, check the board is detected
    ├── verify_calibration.py  check a calibration on new robot poses / touch test
    ├── compute_from_dataset.py  recompute a calibration from a saved dataset (no robot needed)
    ├── save_load_matrix.py    helper from Zivid's python samples
    ├── 3rdParty/rtde-2.3.6/   Universal Robots RTDE python client
    ├── settings/oneplus_3d.yml  camera settings used for the calibration
    ├── datasets/<date>/       pos*.yaml + results in git; img*.zdf captures NOT in git (45 MB each)
    └── results/               chosen calibration + RESULT.md + use_calibration.py example
```

---

## 1. Set up a computer (once)

Requirements: Ubuntu 22.04 (other recent Linux should work), NVIDIA GPU + driver (the Zivid SDK needs OpenCL),
Docker, NVIDIA Container Toolkit, internet for the first build. Tested on a ThinkPad P1 Gen 6 (RTX 4090 Laptop GPU,
driver 550), Docker with the `nvidia` runtime.

```bash
# Docker + NVIDIA Container Toolkit (skip what is already installed)
#   https://docs.docker.com/engine/install/ubuntu/
#   https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
sudo usermod -aG docker $USER

git clone <this repo> zivid-oneplus && cd zivid-oneplus
sudo ./host/setup_host.sh          # USB permissions + dialout group, then log out/in and replug the camera
```

Get the images, **either** build them (needs internet, ~10 min):
```bash
./download_packages.sh                                  # Zivid 2.11.1 .deb files
docker build -t zivid-oneplus:2.11.1 .
docker build -t zivid-oneplus-calib:2.11.1 calib/
```
**or** load the exported file (no internet needed):
```bash
gunzip -c zivid-oneplus-images.tar.gz | docker load
```

Check:
```bash
./run.sh ZividListCameras          # -> Found 1 camera(s) ... zividOnePlusMedium
```

## 2. Use the camera

```bash
./run.sh ZividStudio               # GUI (Live view, capture, export settings)
./run.sh                           # shell inside the container
```
- Your home folder is shared at the same path: anything saved under `/home/<you>/...` ends up on the computer.
  Default folder: `~/zivid_oneplus_data`. Do **not** save to `/root`, `/home` or system folders: lost on exit.
- **Only one program can use the camera at a time.** Close Studio before running any calibration script.
- The camera connects over **USB 3** (not Ethernet). `lsusb` must show `Zivid One Plus M`, `DLPC350`, `BU238MC`.

## 3. Hand-eye calibration (UR robot, eye-to-hand)

Camera fixed, **Zivid calibration board ZVD-CB01 (7×8, 30 mm)** rigidly mounted on the robot flange.
(Pickit plates / custom printed boards are not detected.)

**Network:** the robot here is `192.168.163.11`. Give the laptop's wired adapter an address in the same subnet,
e.g. `192.168.163.100/24` (on the lab laptop it is an extra address on NetworkManager profile "Profile 1").
Check: `ping 192.168.163.11`.

**Robot:** load `calib/universal_robots_robot_motion_script.urp`, teach 10–15 waypoints: whole board visible,
nothing (wrist, cables) over the board, different tilts (±10–25°) and distances. Use MoveJ.

```bash
cd calib
# 1) check the board is seen (Studio closed)
./calib.sh python3 test_capture.py --settings settings/oneplus_3d.yml
# 2) run; when "Ready for capture: False" is printed, press Play on the pendant
./calib.sh python3 universal_robots_perform_hand_eye_calibration.py --eth --ip 192.168.163.11 \
    --settings settings/oneplus_3d.yml checkerboard
```
Output in `calib/datasets/<UTC date>/`: `handEyeTransform.yaml` (Zivid), `handEyeTransform_fallback.yaml`
(OpenCV+3D), residuals, captures. Good residuals: < 1 mm, < 0.2°.

**Verify** on 3–5 new poses (robot moved by hand, board on flange, script only reads the pose):
```bash
./calib.sh python3 verify_calibration.py pose datasets/<date> --settings settings/oneplus_3d.yml [--transform handEyeTransform_fallback.yaml]
./calib.sh python3 verify_calibration.py touch datasets/<date> --settings settings/oneplus_3d.yml   # board on table, pointer tool
```
Mean error < 2 mm = good. Copy the chosen result to `calib/results/` and update `RESULT.md`.

### Changes vs. the original Zivid UR sample (needed for SDK 2.11 / this setup)
| Change | Why |
|---|---|
| `camera.capture()` instead of `capture_2d_3d()` | `capture_2d_3d` exists only from SDK 2.12 |
| `detect_feature_points(point_cloud)` instead of `detect_calibration_board` | API of SDK 2.11 |
| ArUco "marker" mode removed | not available in zivid-python 2.11 |
| `--settings` option (else Capture Assistant) | settings made for a newer camera do not fit the One+ |
| RTDE output **10 Hz** instead of 200 Hz, no sleep in loop | at 200 Hz the client fell behind and the robot reset the connection |
| OpenCV+3D fallback detector and solver (`board_detect.py`) | the SDK 2.11 detector fails when the robot wrist is near the board border |
| poses without a detected board are skipped | instead of aborting the whole run |

## 4. Using the calibration in your own code

```python
from use_calibration import load_hand_eye, camera_to_base     # calib/results/use_calibration.py
T = load_hand_eye("calib/results/handEyeTransform_oneplus_2026-11-18.yaml")
p_base_mm = camera_to_base(T, [x_mm, y_mm, z_mm])             # Zivid point cloud is in mm
```
UR uses metres (divide by 1000). Details and limits: [`calib/results/RESULT.md`](calib/results/RESULT.md).

## 5. Moving to another computer

| What | How | In git? |
|---|---|---|
| Code, Dockerfiles, settings, calibration results, pose files | this repository | yes |
| Zivid .deb packages | `./download_packages.sh` | no (proprietary) |
| Docker images (if no internet / exact same build) | `./export_images.sh` → copy `images/zivid-oneplus-images.tar.gz` (~1 GB) | no |
| Raw captures `calib/datasets/*/img*.zdf` (~460 MB per dataset) | copy the folder (USB drive / network) | no |
| Host setup (udev, dialout, network IP) | `sudo ./host/setup_host.sh` + network settings | — |

Keep the image export private: it contains the Zivid SDK.

## Troubleshooting
| Symptom | Fix |
|---|---|
| `No camera(s) found` | Studio (or another script) still open; or USB permissions (`host/setup_host.sh`, replug); `lsusb` |
| `OpenCL error ... Failed to get platforms` | container started without GPU: use `run.sh` / `calib.sh` (`--gpus all`), check `nvidia-smi` |
| Board "not detected" | whole 7×8 grid visible? nothing near the border? not over-exposed? look at `test_capture.zdf` in Studio |
| `ConnectionResetError` from RTDE | old script with 200 Hz; use this version |
| Studio window does not open | `xhost +local:docker`, `DISPLAY` set |

## Credits / licenses
- `universal_robots_perform_hand_eye_calibration.py`, `save_load_matrix.py`: based on
  [Zivid python samples](https://github.com/zivid/zivid-python-samples) (BSD-3-Clause).
- `3rdParty/rtde-2.3.6`: Universal Robots RTDE python client.
- Zivid SDK, Studio and Teli driver: downloaded from zivid.com, not redistributed here.
