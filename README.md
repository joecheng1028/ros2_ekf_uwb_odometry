# EKF Sensor Fusion — UWB and Wheel Odometry

Extended Kalman Filter in C++ (ROS 2 Humble, Eigen) fusing UWB position
measurements and wheel odometry for ground robot localisation.

This project extends the work from my M.Sc. thesis
(*Robot-Based UWB Localization Testbed and Simulation Environment*,
TU Chemnitz, 2025), which validated UWB indoor localisation accuracy
using a robot-based testbed. The thesis compared raw UWB positioning
against wheel odometry only. This repository implements an EKF to fuse
both sensors, producing a trajectory estimate that is smoother than raw
UWB and less drifted than raw odometry.

---

## What It Does

The filter runs two steps on every incoming message:

**Predict** — on every `/odometry/filtered` message, the differential-drive
motion model propagates the state estimate forward using linear velocity `v`
and angular velocity `ω`. The state transition Jacobian linearises the
nonlinear motion model for covariance propagation.

**Update** — on every `/uwb_pose` message, the UWB position measurement
corrects the predicted estimate via the standard EKF innovation equations.
Measurement noise covariance `R` is scaled by `PDOP²` to reduce trust in
UWB when anchor geometry is poor.

The fused position is published continuously on `/ekf_pose`.

---

## State Vector

```
x = [ px,  py,  θ ]ᵀ
```

- `px`, `py` — robot position in the map frame (metres)
- `θ` — robot heading / yaw angle (radians)

---

## Repository Structure

```
ekf_node/
├── src/
│   ├── ekf_node.cpp       — EKF node (predict + update + publish)
│   └── eigen_pdop.cpp     — PDOP computation via SVD pseudoinverse
│                            (reconstructed from thesis Method 3 in C++)
├── CMakeLists.txt
├── package.xml
└── README.md
```

`eigen_pdop.cpp` reconstructs the PDOP geometric reliability score from
the thesis simulation in C++ using Eigen. The PDOP value it computes
directly informs the `R` matrix in `ekf_node.cpp` — high PDOP means
poor anchor geometry, so the EKF trusts UWB less.

---

## Dependencies

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Eigen3 | 3.4.0 (system) |
| nav\_msgs | Humble |
| geometry\_msgs | Humble |

Install Eigen if not already present:

```bash
sudo apt install libeigen3-dev
```

---

## Build

```bash
# Place the package in your ROS 2 workspace src folder
cd ~/ros2_ws
colcon build --packages-select ekf_node
source install/setup.bash
```

---

## Run

```bash
# Terminal 1 — start the EKF node
ros2 run ekf_node ekf_node

# Terminal 2 — replay a bag file containing odometry and UWB data
ros2 bag play <path_to_bag_folder>

# Terminal 3 — verify fused output
ros2 topic echo /ekf_pose
```

---

## Expected Topics

| Topic | Type | Role |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | Drives predict step |
| `/uwb_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Drives update step |
| `/ekf_pose` | `geometry_msgs/msg/PoseStamped` | Fused position output |

---

## Tuning Parameters

Set in the constructor of `EkfNode`:

| Parameter | Default | Meaning |
|---|---|---|
| `sigma2_base_` | `0.01` | Baseline UWB variance at PDOP = 1.0 (m²) |
| `Q_` | `0.01 * I` | Process noise covariance |
| Initial `P_` diagonal | `[1.0, 1.0, 0.1]` | Initial position and heading uncertainty |

---

## Validation

Validated against ROS 2 bag files recorded during thesis experiments
(library building, TU Chemnitz, July 2025). Bag files are not included
in this repository.

To compare raw UWB, raw odometry, and EKF output side by side:

```bash
ros2 topic echo /uwb_pose
ros2 topic echo /odometry/filtered
ros2 topic echo /ekf_pose
```

Or visualise all three trajectories simultaneously in RViz2 by adding
three `Path` displays subscribed to each topic.

---

## Related Work

- M.Sc. Thesis: *Robot-Based UWB Localization Testbed and Simulation
  Environment*, TU Chemnitz, 2025 (Grade: 1.4)
- Thesis pipeline repository:
  [uwb-localization-pipeline](https://github.com/joectt/uwb-localization-pipeline)
  — 8-stage Python pipeline processing ROS 2 bag files for UWB
  validation experiments

---

## License

MIT License — see [LICENSE](LICENSE) for details.
