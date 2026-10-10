# RB3302 navigation

`task_manager` is a one-shot Nav2 `NavigateToPose` client. It validates four ordered targets (including duplicates), refuses uncalibrated landmarks, republishes map-frame initial pose while waiting, verifies AMCL covariance plus `map -> base_link`, handles acceptance/navigation/cancellation/overall deadlines, confirms position after Nav2 success, verifies odometry is stopped, then dwells for three seconds.

Build: `cd /ssd/workspaces/rb3302_final_proj/student_ws && colcon build --packages-select rb3302_navigation rb3302_simulation && source install/setup.bash`.

Measure map-frame A, pitch, and rotation in RViz; set `calibrated: true` only after review, configure `config/task.yaml`, then run `ros2 launch rb3302_navigation task.launch.py start_hardware:=true`. The original map remains at `/ssd/workspaces/rb3302_final_proj/student_ws/maps/rb3302_map.yaml`.

The footprint is conservatively based on the supplied 0.30 x 0.25 m chassis. Hardware LiDAR/TF, obstacle replanning, label coverage, and calibration need real-robot verification.
