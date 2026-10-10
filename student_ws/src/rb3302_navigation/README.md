# RB3302 autonomous navigation

`rb3302_navigation` is the final-project orchestration package. It retains the university hardware packages untouched and reuses `planning_rpp`'s Jazzy Nav2 launch: wheel odometry from `odom/raw` and Yesense IMU `/imu_data_ros` are fused by EKF into `/odom`; AMCL aligns `odom` to the static map using `/scan`; Navfn is configured with `use_astar: true`; Regulated Pure Pursuit produces conservative velocity commands through Nav2's velocity smoother to `/cmd_vel`.

## Contents

- `config/landmarks.yaml`: board-to-map calibration and per-square survey overrides.
- `config/task.yaml`: S0/B0 and exactly four ordered targets.
- `config/navigation.yaml`: AMCL, A*-mode Navfn, RPP, LiDAR obstacle/inflation layers, and conservative speed limits.
- `maps/map.yaml`: intentionally disabled metadata template.
- `task_manager.py`: validates input, waits for Nav2, publishes `/initialpose`, submits sequential `NavigateToPose` actions, holds zero velocity for at least three seconds after each success, and halts on rejection/failure/timeout.
- `initial_pose.py`: optional standalone initial-pose diagnostic publisher.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /ssd/workspaces/rb3302_final_proj_code/student_ws
colcon build --packages-select rb3302_navigation
source install/setup.bash
pytest src/rb3302_navigation/test
```

The runtime also needs the already-used Jazzy packages: `nav2_*`, `robot_localization`, `rclpy`, `geometry_msgs`, `nav2_msgs`, `PyYAML`, and the workspace's `planning_rpp`, `robot_init`, model, LiDAR and IMU packages. Build/source `robot_ws` and the planning workspace in the normal overlay order.

## REQUIRED MANUAL CALIBRATION

Do **not** set `map_ready:=true` or `calibrated: true` before all of these are measured and checked:

1. Make a real map YAML for `/home/lujianyi/Downloads/rb3302_map.pgm` using Cartographer's original resolution, origin and thresholds. The PGM alone does not contain sufficient Nav2 map metadata. Do not replace the original PGM.
2. Measure the map-frame centre of A, the actual centre pitch, and the board angle versus map +x; enter them in `landmarks.yaml`. Add `overrides` for surveyed non-regular squares.
3. Verify the physical chassis footprint. The current `0.30 x 0.25 m` polygon comes from the supplied URDF and must include protrusions and a safety margin; tune inflation after a static collision test.
4. Verify TF (`map -> odom -> base_link -> laser`), scan orientation/range, `/odom` direction/scale, IMU yaw convention, and that the supplied LiDAR is really `/scan`.
5. Tune AMCL particle count/noise, goal tolerance, RPP speed/lookahead, obstacle ranges and inflation for the actual surface and battery condition.

The static map layer cannot safely infer that a mapped wall has been removed. It does clear **LiDAR obstacle-layer** observations using ray tracing, and it replans around newly observed obstacles. Treat removed mapped obstacles as still blocked unless the static map is deliberately remapped/updated after a safety assessment.

## Configure and launch

Edit `config/task.yaml`; repeated targets are valid. First make a measured YAML (for example `~/maps/rb3302_map.yaml`) which references the original PGM. Then set the calibration flag only after completing the checks above.

```bash
ros2 launch rb3302_navigation task.launch.py \
  map:=/absolute/path/to/measured_rb3302_map.yaml map_ready:=true \
  start_hardware:=true rviz:=true
```

`start_hardware:=false` is the safe default: it starts navigation processes but never starts physical driver nodes. It is useful only with simulated/replayed interfaces. The task manager remains locked by default because both map and landmark defaults are intentionally uncalibrated.

For localization-only testing, start hardware and the launch with a valid map, then in RViz inspect `/map`, `/scan`, `/amcl_pose`, TF and costmaps. For one-goal testing, temporarily use the same label four times in `task.yaml`; the manager will execute four safe arrivals with dwell periods. For dynamic replanning, place a non-contact obstacle on a clear planned path, observe it in local/global costmaps, and verify Nav2 replans before proceeding. Test at low speed with a spotter and emergency stop available.

Useful diagnostics:

```bash
ros2 topic echo /amcl_pose
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /scan
ros2 topic echo /cmd_vel
ros2 lifecycle nodes
```

Hardware navigation has not been validated by these source-level tests. Before the graded run, validate map metadata, calibration, AMCL convergence from each permitted starting bearing, stopping distance, LiDAR clearing/marking, blocked-path recovery, goal timeouts, and the complete sequence within three minutes.
