# TurtleBot3 SLAM + Gap Drive Next Steps

Reference: https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/

## Current Understanding

- Use `cartographer.launch.py` for SLAM/map generation.
- Use `gap_drive.launch.py` for the custom reactive autonomous driving node.
- Do not run `slam_gap_drive.launch.py` together with Cartographer, because that launch starts `slam_toolbox`, and both SLAM systems can publish `/map` and `map -> odom`.
- RViz config cannot save TF publishers. The SLAMTEC LiDAR static TF must be launched as a ROS node or added to a launch file.
- Current LiDAR static transform that worked:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.172 0 0 0 base_link laser
```

## Recommended Bringup Order

Run these in separate terminals, after sourcing the workspace where needed.

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.172 0 0 0 base_link laser
```

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

```bash
ros2 launch turtlebot_autonomous_gap_drive gap_drive.launch.py
```

Start driving only when ready:

```bash
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
```

Stop driving:

```bash
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## RViz Notes

- If Cartographer launches RViz with the TurtleBot3 Cartographer config, use that RViz. There is no need to also run `ros2 launch turtlebot3_bringup rviz2.launch.py`.
- For map view:
  - Fixed Frame: `map`
  - Map topic: `/map`
  - LaserScan topic: `/scan`
  - TF: enabled
  - Odometry topic: `/odom`
- `No map received` means no valid `/map` message has arrived yet. It is not fixed by saving RViz config.
- If `/scan` is OK but `/map` is missing, check Cartographer and TF instead of RViz styling.

## Debug Checklist

Check Cartographer nodes:

```bash
ros2 node list
```

Expected:

```text
/cartographer_node
/occupancy_grid_node
```

Check map publisher:

```bash
ros2 topic info /map
```

Check TF chain:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
```

Expected TF chain:

```text
map -> odom -> base_footprint -> base_link -> laser
```

Check scan frame:

```bash
ros2 topic echo /scan
```

Expected:

```text
header.frame_id: laser
```

Check odom sanity:

```bash
ros2 topic echo /odom
```

`pose.pose.position.x` and `y` should be normal meter-scale values. Values like `e+40` are invalid and can crash RViz or break SLAM.

## Performance Tuning Plan

Cartographer is mainly for visualization and map diagnosis in the current setup. The custom `gap_drive_node` still drives from `/scan`, `/imu`, and `/odom`, not from the saved map.

Use RViz to classify failures:

- Narrow gap is ignored:
  - lower `safe_gap_distance_m`
  - lower `front_stop_distance_m`
  - reduce `gap_window_deg`
  - tune `narrow_passage_*`
- Hits wall at corners:
  - lower `max_linear_speed`
  - increase `front_slow_distance_m`
  - increase `turn_slowdown_gain`
  - increase `front_stop_distance_m`
- Oscillates left/right:
  - lower `heading_gain`
  - lower `centering_gain`
  - lower `max_angular_speed`
- Always turns one direction:
  - inspect `/cmd_vel`
  - test asymmetric obstacles
  - check `prefer_left_recovery`
  - confirm LaserScan orientation in RViz

Record data for repeatable tuning:

```bash
ros2 bag record /scan /imu /odom /tf /tf_static /cmd_vel
```

## Later Cleanup

- Add the static TF publisher to a launch file so it does not need to be typed manually.
- Optionally save the RViz display setup with `File -> Save Config As`, then run it with `rviz2 -d <config.rviz>`.
- If moving from reactive gap driving to map-based navigation, next major step is saved map + AMCL + Nav2.
