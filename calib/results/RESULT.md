# Zivid One+ M → UR robot: hand-eye calibration result (2026-11-18)

**File:** `handEyeTransform_oneplus_2026-11-18.yaml` (copy of `../datasets/2026-11-18_15-44-14/handEyeTransform.yaml`)

| | |
|---|---|
| Camera | Zivid One+ M, firmware 1.9.2, SDK 2.11.1 |
| Robot | Universal Robots e-series, Polyscope 5.13, IP 192.168.163.11 |
| Setup | eye-to-hand: camera fixed above the cell, Zivid board ZVD-CB01 (7×8, 30 mm) on the robot flange |
| Method | Zivid `calibrate_eye_to_hand`, 10 poses, camera settings `../settings/oneplus_3d.yml` |
| Meaning | `T_base_camera`: maps a point in the camera frame to the robot base frame |
| Units | translation in **millimetres**, rotation as a 3×3 matrix |

```
[[ 0.0433693, -0.976846,   0.2095017,   24.36271],
 [-0.9917114, -0.01670729, 0.1273946, -476.1615 ],
 [-0.1209446, -0.2132903, -0.9694739, 1381.802  ],
 [ 0,          0,          0,            1       ]]
```
Camera origin in the robot base: x = 24.4, y = −476.2, z = 1381.8 mm (about 1.38 m above the base, looking down).

## Quality

| Check | Result |
|---|---|
| Residuals, 10 calibration poses | 0.09–0.40 mm, 0.008–0.09° |
| Prediction on the other dataset (2026-11-18_15-17-58, not used) | mean 0.38 mm, max 0.56 mm |
| New robot poses (live test, `verify_calibration.py pose`) | mean 0.32 / 0.34 / 0.55 mm, max ≤ 0.99 mm, ≤ 0.11° |
| Agreement with the independent OpenCV+3D solution | 0.49 mm in camera position |

Valid workspace: the calibration poses covered roughly x 225–450, y −380 to −265, z 30–370 mm (TCP in robot base).
Accuracy is best inside and near this region.

## When the calibration is no longer valid
Redo it (see main README, "Hand-eye calibration") if the **camera or the robot base moves**, the camera mount is touched,
or the camera is re-mounted. The robot TCP setting does **not** matter for this result (eye-to-hand).
