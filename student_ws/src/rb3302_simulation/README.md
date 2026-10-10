# RB3302 Gazebo simulation

This Gazebo Harmonic package never launches physical drivers. Gazebo owns
`odom -> base_link`; AMCL owns `map -> odom`. The simulation Nav2 adapter
uses the project Navfn (A*) and Regulated Pure Pursuit settings without the
physical `/odom/raw` and IMU EKF.

## Build and launch

```bash
cd /ssd/workspaces/rb3302_final_proj/student_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rb3302_navigation rb3302_simulation --symlink-install
source install/setup.bash
ros2 launch rb3302_simulation simulation.launch.py
```

In a second terminal (after sourcing the same setup files), start manual
navigation and RViz:

```bash
ros2 launch rb3302_simulation manual_navigation.launch.py
```

Set RViz's fixed frame to `map`, click **2D Pose Estimate**, and set the pose
to the robot's actual Gazebo pose (initially `(0, 0, 0)` in this world). Then
use **Nav2 Goal** to send a reachable map-frame target. For a headless Gazebo
server use `simulation.launch.py gui:=false`.

To run the automatic wrapper (which starts Gazebo, the same Nav2 adapter, and
the task manager), first calibrate the physical landmark file and then run:

```bash
ros2 launch rb3302_simulation navigation_sim.launch.py
```

The task manager intentionally refuses to navigate while
`rb3302_navigation/config/landmarks.yaml` has `calibrated: false`; no physical
calibration values are fabricated for simulation. Stop launch processes with
Ctrl-C in each terminal.

## Temporary obstacles and limitations

The bridge exposes `/scan`, so a spawned Gazebo obstacle should be observed by
the local obstacle layer; inspect `/local_costmap/costmap` in RViz before
claiming avoidance or replanning. The checked-in world contains map-derived
walls but no scripted runtime obstacle insertion. Physical hardware and the
four-waypoint run remain unverified until real landmark calibration is
provided.
