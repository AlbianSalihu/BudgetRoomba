# Budget Roomba — Autonomous Duplo Brick Collector

EPFL Robotics Competition 2023 (Feb – Jun 2023), organized by BioRob lab (Prof. Auke Ijspeert).

The challenge: build a fully autonomous robot that navigates a 9×9 m arena, detects Duplo brick constructions, collects them, and deposits them in scoring zones — with zero human intervention during the 10-minute run.

**Result: completed all collection cycles in Zone 1 without a single navigation failure.**

> Original development happened across individual repositories at [github.com/STIRobotCompetition2](https://github.com/STIRobotCompetition2). This repo consolidates them into a single ROS 2 workspace for portfolio readability.

---

## Repository Structure

```
BudgetRoomba/               ← colcon workspace root
├── src/
│   ├── br_brain/           ← main state machine (C++)
│   ├── br_brick_detection/ ← YOLOv5 inference via torch.hub (Python)
│   ├── br_brick_management/← detection memory & cost map (C++)
│   ├── br_state_estimation/← Cartographer SLAM + arena localization + EKF odometry (C++)
│   ├── br_filtering/       ← LiDAR scan filtering (config)
│   ├── br_navigation/      ← Nav2 navigation stack (config/launch)
│   ├── br_drivers/         ← motor driver (pigpio), IMU driver (MPU9150/I2C), camera driver (libcamera)
│   ├── br_description/     ← robot URDF/xacro model
│   ├── br_simulation/      ← Gazebo simulation launch + ground-truth TF plugin
│   └── arena_gazebo/       ← 9×9 m arena SDF world + meshes + materials
└── tools/
    └── duplo_dataset_factory/ ← synthetic YOLO training data generator (Python, pyrender)
```

---

## System Architecture

```
┌─────────────┐    raw scan    ┌──────────────┐   filtered scan  ┌───────────────────┐
│   RPLidar   │───────────────▶│ br_filtering │─────────────────▶│ br_state_estimation│
└─────────────┘                └──────────────┘                   │                   │
                                                                   │  Cartographer SLAM │
┌─────────────┐  raw image     ┌──────────────────┐               │  Arena→Map TF      │
│  Pi Camera  │───────────────▶│ br_brick_detection│              │  EKF odometry      │
└─────────────┘  (torch.hub)   └─────────┬────────┘               └────────┬──────────┘
                                          │ /detected_markers               │ TF + /map
                                          ▼                                 ▼
                                ┌──────────────────┐              ┌────────────────┐
                                │br_brick_management│             │   Nav2 Stack   │
                                │  BrickMapper      │             │(br_navigation) │
                                │  decay grid map   │             └───────┬────────┘
                                └─────────┬─────────┘                     │
                                          │ /brick_detection (srv)         │ NavigateThroughPoses
                                          ▼                                │ (action)
                                ┌─────────────────────────────────────────▼────────┐
                                │                   br_brain                        │
                                │   IDLE → SEARCH_BRICKS_SAFE → COLLECT_BRICK → …  │
                                └──────────────────────────────────────────────────┘
```

---

## Package Descriptions

### `br_brain` — Main State Machine

The central coordinator. Implements a C++ ROS 2 node (`Brain`) that runs a 7-state FSM and wires together all subsystems at startup via timeout-guarded handshakes.

**States:**

| State | Description |
|---|---|
| `INIT` | Startup: connects to Nav2, search follower, brick management |
| `IDLE` | Ready, waiting |
| `SEARCH_BRICKS_SAFE` | Following the search grid in the safe (Zone 1) area |
| `SEARCH_BRICKS_CARPET` | Following the search grid in the carpet (Zone 2) area |
| `COLLECT_BRICK` | Executing a brick collection + drop-off sequence |
| `FINISHED` | Run complete |
| `ERROR` | Unrecoverable failure |

**Collection flow per detected brick:**
1. `brick_management` sends a `/brick_detection` service call with the brick pose
2. `Brain` pauses the search grid, calls `BrickCollectorSimple::processQuery()` — which path-plans through Nav2 (`ComputePathThroughPoses`) to validate feasibility before committing
3. If the plan succeeds, `Brain::collectBrick()` triggers `NavigateThroughPoses`: robot drives to the brick, then to the drop-off zone, then reverses clear
4. Search resumes from where it left off

**Zone validity masking:** a bitmask on `/valid_map/config` lets the brain dynamically restrict Nav2's costmap to safe zones only — if localization quality degrades, Zone 2 is disabled in real time.

**Key configs** (`config/`):
- `searchgrid_arena.csv` — waypoint grid covering the full arena
- `searchgrid_spot_small.csv` — compact spot-search grid
- `zones.csv` — zone boundary definitions

---

### `br_brick_detection` — YOLOv5 Inference

Custom-trained YOLOv5n model loaded via `torch.hub`. The detector runs a sliding window over the camera frame: the 640×480 image is divided into a 3×2 grid of overlapping 320×320 tiles, each tile inferred independently (confidence threshold: 0.4). Detections from all tiles are merged, morphologically dilated, and reduced to bounding-box contours. Each detected contour is back-projected to 3D using a fixed linear camera model (intrinsics: f≈772.5, cx=320) and published as a `visualization_msgs/MarkerArray` on `/detected_markers`.

**Training data:** 8 500 synthetic images generated by `tools/duplo_dataset_factory`.

---

### `br_brick_management` — Detection Memory & Cost Map

`BrickMapper` node maintains a 9×9 m `grid_map::GridMap` over the arena frame (2 cm resolution, 450×450 cells). Incoming detections from `/detected_markers` add probability mass to the map; the map decays at rate `DECAY = 0.1` every 5 seconds.

**Detection extraction:** every 5 s, the map is scanned for the global maximum. If it exceeds `DETECTION_THRESHOLD = 0.2`, a valid detection is reported at that position and the surrounding 0.3 m radius is zeroed (non-maximum suppression). The result is forwarded to `br_brain` as a `BrickDetection` service response.

The map is also published as a `nav_msgs/OccupancyGrid` on `/brick_cost_map` for visualization and as a `grid_map_msgs/GridMap` on `/brick_grid_map` for downstream consumers.

**Custom interfaces:**
- `msg/PolygonArrayStamped.msg` — array of stamped polygons (bounding footprints)
- `srv/BrickDetection.srv` — request/response for a single detected brick pose

---

### `br_state_estimation` — Localization Pipeline

Three components running in concert:

**1. Google Cartographer SLAM** (`config/cartographer.lua`)

2D SLAM fusing LiDAR scans with IMU and wheel odometry. Key tuning choices:
- Scan matcher: `translation_weight = 5.0`, `rotation_weight = 0.02` — aggressive on translation (arena walls are reliable), loose on rotation (avoids over-constraining)
- Pose graph optimization every 10 nodes
- 4 background threads for Ceres solver
- Range: 0.5–15 m (lower bound filters ground returns)

**2. Arena→Map TF Estimator** (`src/arena_to_map_estimator.cpp`)

The cleanest piece of engineering in the project. Rather than using ArUco markers or GPS for absolute localization in the arena frame, this node:

1. Takes the `/map` occupancy grid from Cartographer
2. Thresholds the wall cells (binary, 200/255)
3. Finds the minimum bounding rectangle of all wall points using `cv::minAreaRect`
4. Selects the corner closest to the robot's start pose as the arena origin
5. Publishes a static TF `arena → map`

The transform is updated continuously but gated by a hysteresis threshold (10 cm / 5°), preventing jitter. On subsequent updates it tracks the nearest corner to the previous estimate for consistency.

**3. Odometry Generator** (`src/odometry_generator.cpp`)

Converts `cmd_vel` (velocity commands) into a `nav_msgs/Odometry` message on `/arti_odom`, required as input to Cartographer and the EKF. Diagonal covariances: 0.05 for twist, 0.1 for pose.

**EKF** (`config/odom_ekf.yaml`): fuses IMU and wheel odometry via `robot_localization`.

---

### `br_filtering` — LiDAR Filter

`laser_filters` range filter: passes scans between 0.3 m and 12 m. Removes ground returns (below 0.3 m) and spurious long-range readings.

---

### `br_navigation` — Nav2 Stack

Configuration and launch for Nav2, tuned for the Budget Roomba's footprint (415×300 mm).

**Local planner:** Regulated Pure Pursuit Controller — desired linear velocity 0.5 m/s, lookahead distance 0.2–0.9 m with velocity scaling, rotate-to-heading enabled (min angle 0.5 rad), collision detection active up to 1 s horizon.

**Global planner:** NavFn with A* (`use_astar: true`), tolerance 0.1 m.

**Costmaps:**
- Global (9×9 m, 5 cm/cell, arena frame): obstacle layer (LiDAR + brick point cloud) + inflation layer (radius 0.6 m, scaling 15)
- Local (3×3 m rolling window, 2 cm/cell): obstacle layer + inflation layer (radius 0.5 m, scaling 5)
- Goal tolerance: xy = 0.03 m, yaw = 0.5 rad

---

### `br_drivers` — Hardware Drivers

Three ROS 2 nodes for the on-robot hardware:

**Motor driver** (`src/motors_driver.cpp`): subscribes to `/cmd_vel`, converts twist to per-wheel RPM using differential drive kinematics (wheel radius 57.5 mm, track 300 mm, gear ratio 60), drives two DC motors via pigpio PWM (GPIO 12/13, 100 Hz). PWM duty mapped linearly from 10–90% across 0–5000 RPM. Direction pins: GPIO 23 (right) / 24 (left).

**IMU driver** (`src/mpu9150_driver.cpp`): talks to MPU9150 over I2C (`/dev/i2c-1`, address 0x68). Configures: sample rate 1000 Hz, 94 Hz low-pass filter, ±250°/s gyro range, ±2g accelerometer range. Integrates quaternion on-device via the angular velocity ODE (`dq/dt = 0.5·Ω·q`). Calibrates gyro and accelerometer bias at startup by averaging 500 measurements (`/calibrate` service). Compensates 3 ms low-pass delay in message timestamps.

**Camera driver** (`src/raspberry_camera_v2_driver.cpp`): wraps Raspberry Pi Camera V2 via `lccv::PiCamera` (libcamera). Configurable resolution (default 1920×1080 @ 30 fps), publishes raw `sensor_msgs/Image` and JPEG-compressed `CompressedImage`.

---

### `br_description` — Robot Model

URDF/xacro description of the Budget Roomba:
- Differential drive base (415 mm wide, 300 mm long), ball caster at rear
- Navigation unit xacro: IMU at +53 mm forward offset, LiDAR at +45 mm height (both rotated 180° to face forward)
- Sensor meshes: RPLidar A1M8 and Raspberry Pi Camera V2 STLs
- Gazebo plugin: optional `BudgetRoombaROSPlugin` for ground-truth TF (`world → base_link`)
- Calibration file: `config/br_calibration.yaml`

---

### `br_simulation` — Gazebo Simulation

Launch file for simulating the full stack in Gazebo with the `arena_gazebo` world. Includes `BudgetRoombaROSPlugin` — a Gazebo model plugin that reads the robot's true pose from the physics engine and publishes a ground-truth TF (`world → base_link`) every simulation step. Used for debugging the navigation and state machine before hardware deployment.

### `arena_gazebo` — Arena World

SDF world definition of the 9×9 m competition arena: wall geometry, zone meshes (STL), zone 2 carpet (rug STL), and floor materials/textures. Matches the physical arena dimensions so simulation results transfer directly to hardware.

---

## `tools/duplo_dataset_factory` — Synthetic Dataset Generator

Standalone Python tool that generates synthetic YOLO-format training images for the brick detector.

**How it works:**
- Loads 6 Duplo brick shapes (2×2, 3×2, 4×2, 6×2, 8×2, 10×2) as STL meshes scaled to real Duplo proportions (stud pitch 3.18/2 mm, layer height 1.92 mm)
- Stacks bricks into random constructions using a collision grid (100×100 cells) to prevent overlaps, with exponentially decaying probability for adding more bricks per layer
- Renders to 640×640 using **pyrender** offscreen with randomized directional lighting and camera intrinsics (f = 772.5, cx = cy = 320)
- Adds configurable Gaussian noise
- Places constructions on random background textures (20% probability) or plain backgrounds
- Outputs YOLO-format label files and images; `empty_prob = 0.005` for negative samples
- Threaded generation via `N_THREADS` parameter

```bash
cd tools/duplo_dataset_factory
pip install -r requirements.txt
# Add background textures to data/textures/
python src/dataset_generator.py
# Output: generated/images/ and generated/labels/
```

8 500 images were generated for the final training run.

---

## Build & Run

### Prerequisites

| Dependency | Version |
|---|---|
| ROS 2 | Humble (Ubuntu 22.04) |
| Google Cartographer | ROS 2 port |
| Nav2 | Humble |
| grid_map | ROS 2 |
| robot_localization | ROS 2 |
| OpenCV | 4.x |
| laser_filters | ROS 2 |
| pigpio | latest |
| lccv | libcamera wrapper |
| PyTorch | ≥ 1.9 (for YOLOv5 inference) |
| pyrender + trimesh | (dataset generator only) |

### Build

```bash
cd BudgetRoomba
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Run (hardware)

```bash
# Hardware drivers (start first)
ros2 launch br_drivers robot.launch.xml

# State estimation (SLAM + localization + odometry)
ros2 launch br_state_estimation budget_roomba_state_estimation.launch.xml

# LiDAR filter
ros2 launch br_filtering lidar_filter_pipeline.launch.xml

# Navigation (Nav2)
ros2 launch br_navigation budget_roomba_navigation.launch.xml

# Brick detection
ros2 launch br_brick_detection budget_roomba_brick_detection.launch.xml

# Brain (start last — it waits for all subsystems to come online)
ros2 launch br_brain budget_roomba_brain.launch.xml arena_mode:=true
```

### Run (simulation)

```bash
ros2 launch br_simulation budget_roomba_sim.launch.xml
```

---

## Competition Result

The robot completed multiple collection cycles in Zone 1 without a navigation failure during the competition run. The greedy single-zone strategy proved its value: other teams targeting higher-value zones lost points to navigation failures, while Budget Roomba banked points consistently every cycle.

**Key metrics:**
- Robot footprint: 415×300×150 mm (final Gen 3)
- Obstacle clearance margin: ~85 mm (500 mm gap – 415 mm width / 2)
- Detection model: YOLOv5n, 8 500 synthetic training images (pyrender + trimesh)
- SLAM: Google Cartographer 2D, LiDAR + IMU + odometry

---

## Technical Stack

| Area | Details |
|---|---|
| Framework | ROS 2 Humble |
| Language | C++ (all ROS nodes), Python (detection + dataset generator) |
| SLAM | Google Cartographer 2D |
| Navigation | Nav2 (Regulated Pure Pursuit, BT navigator) |
| Localization | Custom arena→map TF via OpenCV minAreaRect |
| Detection | YOLOv5n (torch.hub), sliding-window 320×320 tiles |
| State estimation | robot_localization EKF (IMU + odometry) |
| IMU | MPU9150 I2C driver, on-device quaternion integration |
| Motors | pigpio PWM, differential drive kinematics |
| Camera | Raspberry Pi Camera V2 via libcamera (lccv) |
| Simulation | Gazebo + custom ground-truth TF plugin |
| Dataset | pyrender + trimesh, STL brick models |
| Competition | EPFL Robotics Competition, June 2023 |
| Team | Two-person team |

---

## Credits

Competition organized by Guillaume Bellegarda and Prof. Auke Ijspeert, EPFL BioRob lab.
