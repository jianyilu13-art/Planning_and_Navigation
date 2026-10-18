# RB3302 Final Project — Autonomous Mobile Robot Navigation

Autonomous navigation project using the **Waveshare Wave Rover**, ROS 2 Jazzy, Nav2, and Gazebo Harmonic.

The project supports both **Gazebo simulation** and **real-world navigation**, sharing the same mission controller and navigation algorithms.

## Project Architecture

```text
RB3302 Final Project
│
├── Simulation
│   ├── Gazebo Harmonic
│   ├── Simulated Wave Rover + LiDAR
│   ├── AMCL Localization
│   ├── Navfn A* Global Planner
│   ├── RPP Local Controller
│   └── Automatic Four-Landmark Mission
│
└── Real World
    ├── Physical Wave Rover
    ├── LiDAR + Odometry + IMU
    ├── EKF Sensor Fusion
    ├── AMCL Localization
    ├── Navfn A* Global Planner
    ├── RPP Local Controller
    └── Automatic Four-Landmark Mission
```

## Repository Structure

```text
rb3302_final_proj/
├── README.md
│
├── student_ws/src/
│   ├── rb3302_simulation/      # Gazebo simulation
│   ├── rb3302_navigation/      # Mission controller
│   ├── planning_rpp/           # RPP navigation stack
│   ├── planning_astar/         # Alternative planning package
│   └── mapping_cartographer/   # SLAM and map generation
│
└── robot_ws/src/               # Physical robot drivers
```

## Documentation

| Documentation | Description |
|---|---|
| [Gazebo Simulation](student_ws/src/rb3302_simulation/README.md) | Simulation setup, manual navigation, automatic simulation, parameters |
| [Real-World Navigation](student_ws/src/rb3302_navigation/README.md) | Physical robot setup, graded-run mission, navigation parameters |

## Navigation Algorithms

| Component | Algorithm |
|---|---|
| Mapping | Cartographer SLAM |
| Localization | AMCL |
| Global Planning | Navfn A* |
| Local Control | Regulated Pure Pursuit (RPP) |
| Obstacle Avoidance | Nav2 Costmaps + RPP |
| Mission Execution | Custom TaskManager |

## Simulation vs Real World

| Component | Simulation | Real World |
|---|---|---|
| Robot | Gazebo Wave Rover | Physical Wave Rover |
| Navigation YAML | `nav2_sim.yaml` | `nav2_real.yaml` |
| Map | Generated simulation map | Cartographer-scanned map |
| Odometry | Gazebo DiffDrive | EKF + physical sensors |
| Localization | AMCL | AMCL |
| Global Planner | A* | A* |
| Local Controller | RPP | RPP |
| Mission Controller | Shared TaskManager | Shared TaskManager |

The simulation and real robot use separate navigation configurations while sharing the same mission logic.

## Quick Start

### Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- ROS 2 Nav2
- Python 3
- Colcon and rosdep

### 1. Clone Repository

```bash
git clone <YOUR_REPOSITORY_URL>
cd rb3302_final_proj
```

### 2. Install Dependencies

```bash
source /opt/ros/jazzy/setup.bash

cd student_ws

rosdep install --from-paths src --ignore-src -r -y
```

Ensure Gazebo Harmonic, Nav2, and any dependencies not declared in `package.xml` are installed.

### 3. Build Workspace

```bash
colcon build --symlink-install

source install/setup.bash
```

### 4. Run Gazebo Simulation

```bash
export ROS_DOMAIN_ID=72
export GZ_PARTITION=rb3302_sim
```

**Manual navigation (two terminals):**

Terminal 1 — Gazebo:

```bash
ros2 launch rb3302_simulation simulation.launch.py gui:=true
```

Terminal 2 — Nav2 + RViz:

```bash
ros2 launch rb3302_simulation sim_navigation.launch.py rviz:=true
```

Source ROS 2 and `student_ws/install/setup.bash` in both terminals.

**Automatic four-landmark mission:**

```bash
ros2 launch rb3302_simulation task_sim.launch.py
```

Stop any existing manual simulation sessions before starting the automatic mission.

### 5. Real-World Navigation

The physical Wave Rover requires the `robot_ws` drivers, a calibrated map, and real-world navigation parameters.

See [Real-World Navigation](student_ws/src/rb3302_navigation/README.md) for hardware initialization, mapping, manual navigation, and the graded-run workflow.

> **Note:** The simulation has been tested on the development machine. A clean installation on another computer has not yet been verified.