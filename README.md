# Zivid One+ in Docker (SDK 2.11.1) + UR hand-eye calibration

Run the **Zivid One+** camera (here a One+ M) on a modern Ubuntu machine **without touching the host's own Zivid SDK**,
and calibrate it to a **Universal Robots** arm (eye-to-hand).

Why Docker: Zivid dropped One+ support after **SDK 2.11** (2.12+ cannot see the camera), while newer Zivid cameras
and the current `zivid-ros` need **SDK ≥ 2.15**. apt allows only one Zivid SDK version per machine and conda cannot
isolate system libraries, so the One+ SDK lives only inside Docker images. The host keeps its newer SDK untouched.

**Latest calibration (2026-11-18):** [`calib/results/handEyeTransform_oneplus_2026-11-18.yaml`](calib/results/handEyeTransform_oneplus_2026-11-18.yaml),
summary in [`calib/results/RESULT.md`](calib/results/RESULT.md) (`T_base_camera`, 0.3–0.55 mm mean error on new poses).

**Quick start** (Docker + NVIDIA Container Toolkit installed, details in section 1):
```bash
sudo ./host/setup_host.sh      # once: USB permissions (then log out/in, replug camera)
./build.sh                     # once: create the Docker images
./run.sh ZividListCameras      # camera found?
./run.sh ZividStudio           # live view
```

---

## Contents

```
.
├── Dockerfile                 image zivid-oneplus:2.11.1  (SDK 2.11.1, Teli USB driver, Zivid Studio, tools)
├── build.sh                   ONE command: download packages + build both images
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

Requirements: Ubuntu 22.04 (other recent Linux should work), an NVIDIA GPU with its driver (the Zivid SDK needs
OpenCL), internet for the first build. Tested on a ThinkPad P1 Gen 6 (RTX 4090 Laptop GPU, driver 550).

**Step 1: Docker and the NVIDIA Container Toolkit** (skip what is already installed)
- Docker: https://docs.docker.com/engine/install/ubuntu/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

```bash
sudo usermod -aG docker $USER      # then log out and back in
docker info | grep -i runtimes     # must list "nvidia"
```

**Step 2: get this repository and prepare the host**
```bash
git clone <this repo> zivid-oneplus && cd zivid-oneplus
sudo ./host/setup_host.sh          # USB permissions + dialout group
```
Log out and back in, then unplug and replug the camera.

**Step 3: create the Docker images**, one command (downloads the Zivid packages, builds both images, ~10–15 min):
```bash
./build.sh
```
No internet? Load the exported images instead: `gunzip -c zivid-oneplus-images.tar.gz | docker load`

**Step 4: check**
```bash
./run.sh ZividListCameras          # -> Found 1 camera(s) ... zividOnePlusMedium
```

## 2. Bring up the Docker / use the camera

There is nothing to start in the background. Each command below starts a container, runs, and removes the
container when it exits. Plug the camera into a **USB 3** port first.

| I want to … | Command |
|---|---|
| check the camera is found | `./run.sh ZividListCameras` |
| open Zivid Studio (live view, capture, save settings) | `./run.sh ZividStudio` |
| get a shell with the Zivid tools | `./run.sh` |
| run a calibration / Python script | `cd calib && ./calib.sh python3 <script.py> ...` |
| get a shell with Python + zivid | `cd calib && ./calib.sh` |

- **Only one program can use the camera at a time.** Close Studio before running any script (otherwise
  "No camera(s) found").
- Your home folder is shared with the container at the same path: anything saved under `/home/<you>/...`
  is on the computer. Studio starts in `~/zivid_oneplus_data`. Files saved elsewhere (`/root`, `/home`, `/tmp`)
  are lost when the container exits.
- `calib.sh` mounts the `calib/` folder as `/work`: datasets and results appear directly in `calib/`, and script
  edits take effect without rebuilding.
- `lsusb` on the host must show `Zivid One Plus M`, `DLPC350` and `BU238MC`.

## 3. Hand-eye calibration (UR robot, eye-to-hand)

**Latest calibration (2026-11-18):** [`calib/results/handEyeTransform_oneplus_2026-11-18.yaml`](calib/results/handEyeTransform_oneplus_2026-11-18.yaml)
(`T_base_camera`, in mm). It was verified at 0.3–0.55 mm mean error on new robot poses; details in
[`calib/results/RESULT.md`](calib/results/RESULT.md). It comes from dataset `calib/datasets/2026-11-18_15-44-14`.
**Redo the calibration only if the camera or the robot base has moved.**

### 3.1 Prepare
1. **Board:** Zivid calibration board **ZVD-CB01 (7×8, 30 mm)**, rigidly mounted on the robot flange. The camera
   stays fixed. (Pickit plates and self-printed boards are not detected.)
2. **Network:** the robot is `192.168.163.11`. Give the computer's wired adapter an address in the same subnet,
   e.g. `192.168.163.100/24` (Settings → Network → wired → IPv4 → Manual). Check: `ping 192.168.163.11`.
3. **Camera settings:** `calib/settings/oneplus_3d.yml`. If the light changed, make new ones in Studio
   (the white squares must not be over-exposed) and save them there.
4. **Robot program:** on the pendant load `universal_robots_robot_motion_script.urp` (from `calib/`) and teach
   **10–15 waypoints**: whole board visible, nothing (wrist, cables) covering it, different tilts (±10–25°)
   and distances. Use MoveJ and check the paths are free.

### 3.2 Calibrate
```bash
cd calib
# 1) Close Studio. At a few waypoints, check the board is detected:
./calib.sh python3 test_capture.py --settings settings/oneplus_3d.yml
#    -> "=> This pose can be used for calibration."

# 2) Start the calibration:
./calib.sh python3 universal_robots_perform_hand_eye_calibration.py --eth --ip 192.168.163.11 \
    --settings settings/oneplus_3d.yml checkerboard
#    When it prints "Ready for capture: False", press PLAY on the pendant.
```
The robot goes through the waypoints; the terminal prints `Capture image N` and the detection result for each.
At the end it prints and saves the result in `calib/datasets/<date>/` (the folder name is in UTC):

| File | Content |
|---|---|
| `handEyeTransform.yaml` | result of Zivid's calibration (poses where Zivid's detector found the board) |
| `handEyeTransform_fallback.yaml` | result of the OpenCV+3D method (usually all poses) |
| `residuals*.yaml` | error per pose. Good: under 1 mm and under 0.2° |
| `img*.zdf`, `pos*.yaml` | captures and robot poses (to recompute later with `compute_from_dataset.py`) |

Use the Zivid result if it used most of the poses; otherwise the fallback result. Section 3.3 decides.

### 3.3 Test the calibration
Board still on the flange. Move the robot with the pendant to a pose that was **not** a waypoint (the script only
reads the robot pose, it never moves the robot), then:
```bash
./calib.sh python3 verify_calibration.py pose datasets/<date> --settings settings/oneplus_3d.yml
./calib.sh python3 verify_calibration.py pose datasets/<date> --settings settings/oneplus_3d.yml \
    --transform handEyeTransform_fallback.yaml          # to test the other result
```
Repeat at 3–5 poses. It prints the distance between where the camera sees the board corners and where the
calibration predicts them: **mean under 2 mm = GOOD** (this setup reached 0.3–0.55 mm), over 5 mm = recalibrate.

Optional physical check, board flat on the table and a pointed tool on the robot:
```bash
./calib.sh python3 verify_calibration.py touch datasets/<date> --settings settings/oneplus_3d.yml
```
It prints four board corners in robot base coordinates; jog the tool tip to one and compare with the pendant.

### 3.4 Keep the result
Copy the chosen file to `calib/results/handEyeTransform_oneplus_<date>.yaml`, update `calib/results/RESULT.md`
and the "Latest calibration" line above, then commit.

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
| Zivid .deb packages + Docker images | `./build.sh` (downloads and builds) | no (proprietary) |
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
- Full notices: [`THIRD_PARTY_LICENSES.txt`](THIRD_PARTY_LICENSES.txt).
- `universal_robots_perform_hand_eye_calibration.py`, `save_load_matrix.py`: based on
  [Zivid python samples](https://github.com/zivid/zivid-python-samples) (BSD-3-Clause).
- `3rdParty/rtde-2.3.6`: Universal Robots RTDE python client.
- Zivid SDK, Studio and Teli driver: downloaded from zivid.com, not redistributed here.
