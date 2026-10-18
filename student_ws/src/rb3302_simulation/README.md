# RB3302 Gazebo Simulation

Gazebo Harmonic simulation of the RB3302 obstacle course and Waveshare Wave Rover.

Supports:
- Manual navigation using RViz goals.
- Automatic four-landmark graded-run simulation.
- AMCL localization using simulated LiDAR and odometry.
- Navfn A* global planning and Regulated Pure Pursuit (RPP) control.

The simulation does not launch physical robot drivers.

## 1. Package Structure

```text
rb3302_simulation/
├── launch/
│   ├── simulation.launch.py       # Gazebo + robot + sensors
│   ├── sim_navigation.launch.py   # AMCL + A* + RPP + RViz
│   ├── task_sim.launch.py         # Automatic graded-run launcher
│   ├── task_nav_gate.py           # Gazebo readiness check
│   └── task_readiness.py          # Nav2 readiness check
│
├── config/
│   ├── nav2_sim.yaml              # Simulation navigation parameters
│   ├── landmarks_sim.yaml         # Simulation landmark coordinates
│   ├── course_geometry.yaml       # Course wall geometry
│   └── bridge.yaml                # ROS-Gazebo communication
│
├── maps/
│   ├── rb3302_sim_map.yaml
│   └── rb3302_sim_map.pgm
│
├── models/
│   ├── course_walls/
│   └── simulated_waverover/
│
└── worlds/
    └── rb3302.world.sdf
```

## 2. Build

```bash
cd /ssd/workspaces/rb3302_final_proj/student_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-select rb3302_navigation rb3302_simulation

source install/setup.bash
```

## 3. Manual Navigation

Use two terminals.

### Terminal 1 — Gazebo

```bash
export ROS_DOMAIN_ID=72
export GZ_PARTITION=rb3302_sim

source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/student_ws/install/setup.bash

ros2 launch rb3302_simulation simulation.launch.py gui:=true
```

### Terminal 2 — Nav2 + RViz

```bash
export ROS_DOMAIN_ID=72
export GZ_PARTITION=rb3302_sim

source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/student_ws/install/setup.bash

ros2 launch rb3302_simulation sim_navigation.launch.py rviz:=true
```

### Manual Launch Flow

```text
Terminal 1                         Terminal 2
    │                                  │
    ▼                                  ▼
simulation.launch.py           sim_navigation.launch.py
    │                                  │
    ├── Gazebo                         ├── nav2_sim.yaml
    ├── Wave Rover                     ├── Map Server
    ├── LiDAR                          ├── AMCL
    ├── Odometry                       ├── Navfn A*
    └── ROS-Gazebo Bridge              ├── RPP Controller
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
                                       Gazebo Robot
```

In RViz:

1. Set **Fixed Frame** to `map`.
2. Verify LiDAR scans align with the occupancy map.
3. Click **2D Goal Pose** to send a destination.

Initial Gazebo robot pose:

```text
x = 0.3 m
y = 2.1 m
yaw = 0 rad
```

AMCL is initialized using this pose.

## 4. Automatic Four-Landmark Simulation

Stop existing Gazebo/Nav2 sessions before starting.

### Launch

```bash
export ROS_DOMAIN_ID=72
export GZ_PARTITION=rb3302_sim

source /opt/ros/jazzy/setup.bash
source /ssd/workspaces/rb3302_final_proj/student_ws/install/setup.bash

ros2 launch rb3302_simulation task_sim.launch.py
```

### Automatic Launch Flow

```text
task_sim.launch.py
        │
        ▼
simulation.launch.py
        │
        ▼
Gazebo + Robot + Sensors
        │
        ▼
task_nav_gate.py
        │
        ├── Check /clock
        ├── Check /scan
        └── Check /odom
        │
        ▼
sim_navigation.launch.py
        │
        ├── nav2_sim.yaml
        ├── Map Server + AMCL
        ├── Navfn A*
        ├── RPP
        └── RViz
        │
        ▼
task_readiness.py
        │
        ├── Check Nav2 lifecycle
        ├── Check TF
        └── Check sensor readiness
        │
        ▼
task_manager.py
        │
        ├── task.yaml
        └── landmarks_sim.yaml
        │
        ▼
NavigateToPose
        │
        ▼
G → M → T → X
        │
        ▼
Stop at each landmark for 3 seconds
        │
        ▼
Mission Completed
```

The automatic run uses the **same Gazebo and Nav2 configuration as manual navigation**.

The difference is that `task_manager.py` sends navigation goals automatically instead of requiring RViz clicks.

## 5. Simulation Parameters

### Navigation

```text
config/nav2_sim.yaml
```

| Component | Important Parameters | Effect |
|---|---|---|
| AMCL | `alpha1`–`alpha4` | Odometry uncertainty |
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
| `landmarks_sim.yaml` | Simulation landmark coordinates |
| `course_geometry.yaml` | Obstacle-course wall geometry |
| `rb3302.world.sdf` | Gazebo physics, wheels and sensors |
| `bridge.yaml` | ROS-Gazebo topic connections |
| `rb3302_sim_map.yaml` | Simulation occupancy map metadata |

## 6. Notes

- The simulation uses a simplified two-driven-wheel robot with passive caster support.
- Gazebo provides `odom → base_link`; AMCL provides `map → odom`.
- No physical robot EKF or hardware drivers are launched.
- The generated occupancy map matches the idealized simulation course.
- Dynamic obstacles are not automatically inserted.
- Do not launch manual and automatic simulation workflows simultaneously.
- The complete four-landmark mission still requires final verification.