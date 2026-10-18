# RB3302 Real-World Navigation

ROS 2 Jazzy navigation for the physical Waveshare Wave Rover.

Supports:
- Robot initialization and manual control.
- Cartographer SLAM and map generation.
- Manual navigation using RViz goals.
- Automatic four-landmark graded run.
- EKF sensor fusion and AMCL localization.
- Navfn A* global planning and Regulated Pure Pursuit (RPP) control.

The same `task_manager.py` is shared between simulation and real-world navigation.

## 1. Project Structure

```text
rb3302_final_proj/
│
├── robot_ws/
│   └── src/
│       ├── robot_init/                  # Robot initialization
│       └── waverover_base/              # Wave Rover control
│
└── student_ws/
    ├── maps/
    │   ├── rb3302_map.yaml             # Real occupancy map
    │   └── rb3302_map.pgm
    │
    └── src/
        ├── rb3302_navigation/
        │   ├── launch/
        │   │   └── task.launch.py       # Automatic graded-run launcher
        │   │
        │   ├── config/
        │   │   ├── nav2_real.yaml       # Real navigation parameters
        │   │   ├── task.yaml            # Start and four targets
        │   │   └── landmarks.yaml       # Real landmark coordinates
        │   │
        │   └── rb3302_navigation/
        │       ├── task_manager.py
        │       ├── landmark_converter.py
        │       └── task_validation.py
        │
        ├── planning_rpp/                # Nav2 + RPP + EKF
        ├── planning_astar/              # Alternative planning package
        └── mapping_cartographer/        # Cartographer SLAM
```

## 2. Build

### Robot Workspace

```bash
cd /ssd/workspaces/rb3302_final_proj/robot_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

### Student Workspace

```bash
cd /ssd/workspaces/rb3302_final_proj/student_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

## 3. Robot Initialization

Start the physical robot before mapping or navigation.

### Terminal 1 — Physical Robot

```bash
source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/robot_ws/install/setup.bash

ros2 launch robot_init robot.launch.py
```

This initializes the robot-side nodes, including:
- LiDAR
- IMU
- Robot model and TF
- Wave Rover control interfaces

Keep this terminal running.

### Manual Driving

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/robot_ws/install/setup.bash

ros2 run waverover_base keyboard_teleop
```

This allows manual movement using the keyboard.

## 4. Mapping with Cartographer

Mapping is required when creating or updating the real-world occupancy map.

### Terminal 1 — Robot

Keep `robot_init robot.launch.py` running.

### Terminal 2 — Cartographer

```bash
source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/robot_ws/install/setup.bash
source /ssd/workspaces/rb3302_final_proj/student_ws/install/setup.bash

ros2 launch mapping_cartographer mapping.launch.py
```

### Mapping Flow

```text
robot_init/robot.launch.py
        │
        ├── LiDAR
        ├── Odometry
        └── IMU
             │
             ▼
mapping_cartographer/mapping.launch.py
             │
             ▼
       Cartographer SLAM
             │
             ▼
       Occupancy Grid Map
             │
             ▼
          Save Map
```

### Save the Map

Keep Cartographer running and open another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/student_ws/install/setup.bash

mkdir -p /ssd/workspaces/rb3302_final_proj/student_ws/maps

ros2 run nav2_map_server map_saver_cli \
  -f /ssd/workspaces/rb3302_final_proj/student_ws/maps/rb3302_map
```

Generated files:

```text
student_ws/maps/
├── rb3302_map.yaml
└── rb3302_map.pgm
```

The saved map is used by AMCL and Nav2 for real-world navigation.

## 5. Manual Real-World Navigation

Manual navigation uses the physical robot and Nav2 without TaskManager.

Use two terminals.

### Terminal 1 — Physical Robot

```bash
source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/robot_ws/install/setup.bash

ros2 launch robot_init robot.launch.py
```

### Terminal 2 — Nav2 + RViz

```bash
source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/robot_ws/install/setup.bash
source /ssd/workspaces/rb3302_final_proj/student_ws/install/setup.bash

ros2 launch planning_rpp planning.launch.py \
  params_file:=/ssd/workspaces/rb3302_final_proj/student_ws/src/rb3302_navigation/config/nav2_real.yaml \
  map:=/ssd/workspaces/rb3302_final_proj/student_ws/maps/rb3302_map.yaml \
  rviz:=true
```

### Manual Launch Flow

```text
Terminal 1                         Terminal 2
    │                                  │
    ▼                                  ▼
robot_init/robot.launch.py    planning_rpp/planning.launch.py
    │                                  │
    ├── Wave Rover                    ├── nav2_real.yaml
    ├── LiDAR                         ├── EKF
    ├── Odometry                      ├── Map Server
    ├── IMU                           ├── AMCL
    └── Robot TF                      ├── Navfn A*
                                      ├── RPP Controller
                                      ├── Costmaps
                                      ├── Velocity Smoother
                                      └── RViz
                                           │
                                           ▼
                                      2D Goal Pose
                                           │
                                           ▼
                                      NavigateToPose
                                           │
                                           ▼
                                        A* → RPP
                                           │
                                           ▼
                                        /cmd_vel
                                           │
                                           ▼
                                    Physical Wave Rover
```

### Send a Navigation Goal

In RViz:

1. Set **Fixed Frame** to `map`.
2. Use **2D Pose Estimate** to initialize AMCL.
3. Verify that LiDAR scans align with the occupancy map.
4. Use **2D Goal Pose** to select a destination.
5. Observe the physical robot following the planned path.

The robot's initial position must correspond to its actual location on the physical course.

## 6. Automatic Four-Landmark Graded Run

The automatic mission uses the same physical robot and Nav2 navigation stack.

Instead of manually clicking goals in RViz, `task_manager.py` sends the four landmark destinations automatically.

### Launch

Stop any previous Nav2 sessions before starting.

```bash
source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/robot_ws/install/setup.bash
source /ssd/workspaces/rb3302_final_proj/student_ws/install/setup.bash

ros2 launch rb3302_navigation task.launch.py \
  start_hardware:=true \
  params_file:=/ssd/workspaces/rb3302_final_proj/student_ws/src/rb3302_navigation/config/nav2_real.yaml \
  map:=/ssd/workspaces/rb3302_final_proj/student_ws/maps/rb3302_map.yaml
```

`start_hardware:=true` starts the physical robot drivers through the hardware launch.

If the physical drivers are already running, use `start_hardware:=false`.

**Note:** Verify that `task.launch.py` forwards `params_file` and `map` to `planning_rpp/planning.launch.py`.

### Automatic Launch Flow

```text
task.launch.py
        │
        ├── robot_init/robot.launch.py
        │       │
        │       ├── Wave Rover
        │       ├── LiDAR
        │       ├── Odometry
        │       └── IMU
        │
        ├── planning_rpp/planning.launch.py
        │       │
        │       ├── nav2_real.yaml
        │       ├── EKF
        │       ├── Map Server
        │       ├── AMCL
        │       ├── Navfn A*
        │       ├── RPP Controller
        │       ├── Costmaps
        │       └── Velocity Smoother
        │
        └── task_manager.py
                │
                ├── task.yaml
                └── landmarks.yaml
                │
                ▼
          Initialize AMCL
                │
                ▼
          Wait for Localization
                │
                ▼
          NavigateToPose
                │
                ▼
          G → M → T → X
                │
                ▼
          Stop at Each Landmark
          for at Least 3 Seconds
                │
                ▼
          Mission Completed
```

### Mission Configuration

Task file:

```text
config/task.yaml
```

Example:

```yaml
start:
  square: A
  bearing: N

targets: [G, M, T, X]
```

Landmark coordinates:

```text
config/landmarks.yaml
```

The landmark configuration converts labels A–X into real-world map coordinates.

The TaskManager:

1. Reads the task and landmark configurations.
2. Initializes AMCL using the starting landmark and bearing.
3. Waits for localization and Nav2 readiness.
4. Sends each target through `NavigateToPose`.
5. Verifies successful navigation and robot stopping.
6. Dwells for at least three seconds.
7. Proceeds to the next landmark.

The current TaskManager has an overall mission timeout of 180 seconds.

## 7. Real-World Navigation Parameters

### Navigation Configuration

```text
rb3302_navigation/config/nav2_real.yaml
```

| Component | Important Parameters | Effect |
|---|---|---|
| AMCL | `alpha1`–`alpha4` | Odometry motion uncertainty |
| AMCL | `update_min_d`, `update_min_a` | Localization update thresholds |
| RPP | `desired_linear_vel` | Forward speed |
| RPP | `rotate_to_heading_angular_vel` | In-place turning speed |
| RPP | `lookahead_dist` | Path tracking |
| RPP | `max_angular_accel` | Angular acceleration |
| Costmaps | `footprint` | Robot collision envelope |
| Costmaps | `inflation_radius` | Obstacle clearance costs |
| Goal Checker | `xy_goal_tolerance` | Accepted goal position error |
| Velocity Smoother | `max_velocity` | Speed limits |
| Velocity Smoother | `max_accel`, `max_decel` | Acceleration and braking |

### Other Configuration Files

| File | Purpose |
|---|---|
| `task.yaml` | Starting landmark and four targets |
| `landmarks.yaml` | Real-world landmark calibration |
| `nav2_real.yaml` | Real navigation parameters |
| `planning_rpp/config/nav2_params.yaml` | Official RPP baseline |
| `planning_astar/config/nav2_params.yaml` | Alternative planning configuration |
| `mapping_cartographer/config/cartographer.lua` | Cartographer SLAM parameters |
| `student_ws/maps/rb3302_map.yaml` | Saved physical occupancy map |

## 8. Notes

- The physical robot uses EKF sensor fusion for odometry and IMU data.
- AMCL estimates robot pose using LiDAR and the real occupancy map.
- Navfn A* calculates global paths; RPP follows them.
- The measured Wave Rover dimensions are approximately 0.19 × 0.17 m.
- The same `task_manager.py` is used in simulation and real-world navigation.
- Simulation and real-world Nav2 parameters are maintained separately.
- Real-world landmark calibration is required before the graded run.
- Manual physical navigation should be tested before automatic mission execution.
- The real-world graded run has not yet been fully verified.