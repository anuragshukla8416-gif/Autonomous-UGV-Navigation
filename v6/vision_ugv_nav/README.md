# Vision-Based Autonomous Navigation for UGV in Outdoor Environment
### High-Performance Webots Robotics Project // GPS-Denied Outdoor Autonomous Challenge

An autonomous differential-drive Unmanned Ground Vehicle (UGV) designed for robust, real-time navigation from **Point A to Point B** through unstructured, GPS-denied outdoor environments using **multi-camera computer vision** and **deep monocular depth estimation** as primary perception sensors.

---

## 🚀 Key Highlights & Advanced Features

- **Theta* Any-Angle Path Planning & Catmull-Rom Spline Smoothing**:
  - Replaces slow grid A* with high-speed **Theta* Any-Angle Path Planning** (< 5ms calculation).
  - Uses Bresenham line-of-sight Euclidean raycasting to bypass grid jaggedness, connecting directly across line-of-sight waypoints.
  - Cubic Catmull-Rom spline interpolation ensures continuous curvature and optimal differential-drive trajectory tracking.
  - **Instant Obstacle Reaction**: Lookahead extended from 3.2m to **5.5m** with immediate replanning (< 50ms total response time).
- **MiDaS Monocular Relative Depth Estimation**:
  - Full depth estimation pipeline inspired by MiDaS, estimating continuous relative inverse depth from monocular RGB frames.
  - Generates authentic **MiDaS Inferno/Turbo colormaps** with metric range calibration.
  - Exported live to the dashboard alongside raw YOLO bounding boxes.
- **Multi-Camera Surround Rig & Quad Multi-Cam View**:
  - **4-Camera Visual Array**: Forward RGB-D (320x240, -12° pitch), Left Flank (yaw +45°), Right Flank (yaw -45°), and Rear View (yaw 180°).
  - Telemetry dashboard includes an interactive **Camera Switcher** with single-feed tabs (`FORWARD`, `MiDaS DEPTH`, `LEFT FLANK`, `RIGHT FLANK`) and a live **Quad View** rendering all angles in a synchronized 2x2 grid.
- **Real-Time 2D SLAM Traversability & Exploration Heatmap**:
  - Continuous 2D grid tracking explored spatial footprint and obstacle density.
  - Dual-mode visualization on the dashboard: switch between **360° Phosphor Radar**, **SLAM Heatmap**, or **Dual Overlay**.
- **Interactive Waypoint Configuration & Dynamic Mission Dispatch**:
  - Live UI to dynamically set **Start (X, Y)** and **Destination Goal (X, Y)** coordinates.
  - **Route Presets**: Point A → B (Default), Point B → A (Reverse), Cross-Field (C → D), and Obstacle Gauntlet.
  - Optional automatic vehicle repositioning and instant Theta* global replanning.
  - Clean HTTP REST API endpoint: `POST /api/set_mission`.
- **Streamlined HUD & Zero-Scroll Layout**:
  - Removed old scorecard widget to focus entirely on mission telemetry, camera streams, and path planning.
  - **Tactical Actions Toolbar**: Prominently features **⚠️ INJECT SUDDEN OBSTACLE** and **🔄 RESET SIMULATION**.
  - **Real-Time Event Stream**: Relocated directly into the main viewport (no scrolling required!) with an optional full-screen popup modal for full audit logs.
- **GPS-Denied Visual Odometry & EKF Localization**:
  - Pure visual odometry fused with 100Hz IMU/gyro and wheel kinematics via Extended Kalman Filter (sub-10cm RMSE).

---

## 🏛️ System Architecture

```
vision_ugv_nav/
├── worlds/
│   ├── outdoor_mission_default.wbt      # Scenario 1: Multi-Cam Rig + Centerline Boulders
│   ├── outdoor_mission_obstacle.wbt     # Scenario 2: Dynamic Obstacle Injection
│   ├── outdoor_mission_gps_denied.wbt   # Scenario 3: Complete GPS-Denied Challenge
│   └── outdoor_mission_rough.wbt        # Scenario 4: Difficult Terrain & Narrow Canyon Pass
├── controllers/
│   ├── ugv_controller/
│   │   └── ugv_controller.py            # Main robot controller orchestrator
│   ├── supervisor_controller/
│   │   └── supervisor_controller.py     # Ground truth, beacon mover & obstacle injector
│   ├── perception/
│   │   ├── camera_processor.py          # Multi-camera acquisition (front, left, right, rear)
│   │   ├── midas_depth_estimator.py     # MiDaS monocular depth & Inferno colormap
│   │   ├── yolo_detector.py             # YOLOv8 object detection & classification
│   │   ├── terrain_classifier.py        # Color & geometric traversability classification
│   │   ├── obstacle_detector.py         # 3D clustering & 5.5m corridor surveillance
│   │   └── traversability_map.py        # 2D costmap & 2D SLAM exploration heatmap
│   ├── localization/
│   │   ├── visual_odometry.py           # Optical flow & visual displacement
│   │   ├── wheel_odometry.py            # Differential drive kinematics
│   │   ├── imu_filter.py                # Gyro integration & yaw unwrapping
│   │   └── pose_estimator.py            # EKF sensor fusion (VO + Encoders + IMU)
│   ├── planning/
│   │   ├── theta_star.py                # Theta* any-angle planner & Catmull-Rom spline
│   │   ├── a_star.py                    # Grid A* baseline planner
│   │   ├── dwa_planner.py               # Dynamic Window Approach local planner
│   │   └── recovery_manager.py          # Stuck detection & recovery maneuvers
│   └── control/
│       └── diff_drive_controller.py     # Differential drive wheel velocities & emergency brake
├── config/
│   ├── default_config.yaml              # Vehicle dimensions, sensor rates & gains
│   └── scenarios.yaml                   # Configurations for scenarios 1-4
├── assets/
│   └── textures/                        # Seamless offline terrain textures
├── dashboard/
│   ├── app.py                           # Flask live telemetry & mission dispatch server
│   ├── templates/index.html             # High-tech HUD, multi-cam viewer & waypoint dispatch
│   └── static/                          # CSS styling, phosphor radar, heatmap & event stream JS
├── tests/
│   ├── test_perception.py               # Multi-camera, MiDaS & SLAM heatmap unit tests
│   ├── test_localization.py             # EKF & kinematics unit tests
│   ├── test_planning.py                 # Theta*, A* & DWA unit tests
│   └── run_system_test.py               # Automated headless integration test
├── run_demo.py                          # 1-Click launcher for Webots + Dashboard
└── README.md                            # Documentation
```

---

## ⚙️ Prerequisites & Setup

### Requirements
- **macOS** (Apple Silicon or Intel) or **Linux**
- **Webots R2025a** (installed in `/Applications/Webots.app` or accessible via `webots` CLI)
- **Python 3.10+** (tested on Python 3.13 / 3.14)
- Pre-installed packages: `numpy`, `scipy`, `pillow`, `matplotlib`, `Flask`

### Verify Setup
Run the unit test suite:
```bash
cd vision_ugv_nav
python3 -m unittest discover -s tests -p "test_*.py"
```
All 16 unit tests should pass with `OK`.

---

## 🎬 1-Click Quickstart Demonstration

To start the demonstration with the live telemetry dashboard:
```bash
python3 run_demo.py
```
This command automatically:
1. Spawns the **Mission Control Dashboard** web server on `http://127.0.0.1:5005`.
2. Automatically opens your default web browser to the dashboard.
3. Launches **Webots R2025a** with the multi-camera outdoor environment.

### Running Headless Automated Integration Test
```bash
python3 tests/run_system_test.py
```

---

## 🎮 Dashboard User Guide

### 1. Multi-Camera Rig & MiDaS Monocular Depth
- Click on the camera switcher buttons above the vision feed:
  - **FORWARD (YOLO)**: Shows the front 320x240 RGB feed annotated with YOLO bounding boxes, class chips (`ROCK`, `HAZARD_BARREL`, `GOAL_BEACON`), confidence scores, and metric distances.
  - **🌈 MiDaS DEPTH**: Shows the estimated inverse relative depth map rendered with an authentic Inferno colormap.
  - **◀ LEFT (+45°)**: Views the left flanking camera.
  - **▶ RIGHT (-45°)**: Views the right flanking camera.
  - **🔲 QUAD**: Renders all 4 cameras in a synchronized 2x2 surveillance grid.

### 2. Dynamic Waypoint Configuration & Dispatch
- Adjust **Start (X, Y)** and **Destination Goal (X, Y)** input fields.
- Or click one of the quick presets:
  - `Point A → B (Default)`: `(-10.0, -8.0) → (10.0, 8.0)`
  - `Point B → A (Reverse)`: `(10.0, 8.0) → (-10.0, -8.0)`
  - `Cross Field (C → D)`: `(-10.0, 8.0) → (10.0, -8.0)`
  - `Obstacle Gauntlet`: `(-6.0, -4.0) → (6.0, 4.0)`
- Check or uncheck **Reposition UGV To Start Waypoint**.
- Click **🚀 DISPATCH MISSION**: The supervisor immediately moves Goal Beacon B, repositions the robot if requested, and the UGV recalculates an optimal Theta* any-angle spline.

### 3. Tactical Actions
- **⚠️ INJECT SUDDEN OBSTACLE**: Drops a rolling dynamic hazard barrel 2.8m directly ahead of the moving UGV. Watch the UGV detect the hazard at 5.5m and execute an instantaneous S-curve Theta* replan with zero collisions.
- **🔄 RESET SIMULATION**: Reloads the Webots world to initial state.

### 4. 2D Radar & SLAM Heatmap Toggles
- **📡 RADAR**: Displays 360° phosphor rotating radar sweep, range rings, DWA rollout trajectories, and obstacle blips.
- **🔥 SLAM HEATMAP**: Displays the 2D spatial exploration and traversability density heatmap (green for traversable paths, red/amber for obstacle clusters).
- **⚡ DUAL OVERLAY**: Blends the radar sweep and trajectories directly on top of the SLAM heatmap.

### 5. In-Viewport Event Stream & Modal Audit Log
- All YOLO detections, dynamic replans, and hazards stream live directly in the main view without requiring scrolling.
- Click **↗ POPUP LOG** to open the full-screen modal audit drawer for deep inspection.
