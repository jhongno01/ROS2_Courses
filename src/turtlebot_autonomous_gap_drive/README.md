# turtlebot_autonomous_gap_drive

`turtlebot_autonomous_gap_drive`는 "터틀봇 자율주행" 의미의 `turtlebot_autonomous`와, 메인 구동 방식인 라이다 기반 빈 공간 통과 주행 `gap_drive`를 합친 패키지입니다.

목표는 빠른 기록보다 완주입니다. 사진처럼 칸막이 벽, 좁은 틈, 장애물이 섞인 경로에서 SLAM/Nav2 없이도 라이다 `/scan`으로 천천히 안전한 방향을 고르는 reactive 주행을 기본 전략으로 합니다. 여기에 IMU 기울기 정지, gyro 기준 역방향 복구, odom 기반 stuck 복구를 안전장치로 더했습니다.

## 추천 방향

- 1순위: `gap_drive.launch.py`로 라이다 기반 저속 완주 주행
- 2순위: 시간이 남으면 `slam_gap_drive.launch.py`로 `slam_toolbox` 지도 생성만 함께 실행
- 비추천: 처음부터 Realsense + SLAM + Nav2 전체 구성

Jetson Nano에서 ROS2 Foxy를 맞춰야 한다면 이 패키지는 C++14와 Foxy 기본 API 위주로 작성되어 있습니다. 다만 Ubuntu 22.04의 공식 ROS2 조합은 Humble이므로, Foxy를 22.04에서 쓸 경우에는 소스 빌드나 컨테이너 환경이 필요할 수 있습니다.

## 동작 개요

노드는 `/scan`의 `sensor_msgs/msg/LaserScan`, `/imu`의 `sensor_msgs/msg/Imu`, `/odom`의 `nav_msgs/msg/Odometry`를 구독합니다. 전방 거리가 충분하면 가장 넓고 안전한 빈 공간을 향해 전진하고, 전방이 막히면 좌우 중 더 여유 있는 방향으로 제자리 회전합니다. 기본 속도는 TurtleBot3 Burger 최대 속도보다 낮은 `0.10 m/s`로 설정했습니다.

기본값은 `auto_start: false`라서 launch 직후에는 0속도를 publish하며 대기합니다. start 서비스가 호출되면 gyro yaw 적분값을 0도로 reset하고 주행을 시작합니다. 주행 중 기울기가 `tilt_stop_deg`를 넘으면 즉시 정지하고, 기준 yaw의 반대 방향으로 일정 시간 이상 전진하면 제자리 복구 회전을 수행합니다. 전진 명령이 있는데 `/odom`상 움직임이 거의 없으면 짧게 후진한 뒤 회전해서 다시 탐색합니다.

기본 출력은 TurtleBot3 Foxy에서 흔히 쓰는 `geometry_msgs/msg/Twist` 타입의 `/cmd_vel`입니다. 기존 수업 코드처럼 `/cmd_vel`이 `geometry_msgs/msg/TwistStamped` 타입이어야 하는 환경이면 YAML에서 `cmd_vel_stamped: true`로 바꾸세요.

## 빌드

워크스페이스 루트에서 실행합니다.

```bash
colcon build --packages-select turtlebot_autonomous_gap_drive
source install/setup.bash
```

## 실행 방법

### Terminal 4 (사전 세팅1)

터틀봇3 OpenCR 보드 시작

```bash
source install/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
```

### Terminal 5 (사전 세팅2)

SLAMTEC RPLIDAR C1 시작 (반드시 OpenCR 노드 실행 후 시작!)

```bash
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```
라이다 드라이버가 먼저 `/scan`을 publish하고 있어야 합니다.

```bash
ros2 launch turtlebot_autonomous_gap_drive gap_drive.launch.py
```

노드는 켜진 상태에서 다음 서비스로 주행을 시작하고 멈춥니다.

```bash
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: false}"
```

SLAM 지도 생성을 같이 보고 싶고 `slam_toolbox`가 설치되어 있다면 다음을 사용할 수 있습니다.

```bash
sudo apt install ros-foxy-slam-toolbox
ros2 launch turtlebot_autonomous_gap_drive slam_gap_drive.launch.py
```

## 실행 전 확인

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
ros2 topic info /cmd_vel
```

`/cmd_vel` 타입이 `geometry_msgs/msg/TwistStamped`로 떠야 하는 구동 환경이면 [config/slow_finish_first.yaml](config/slow_finish_first.yaml)의 `cmd_vel_stamped`를 `true`로 바꿉니다.

## 파라미터 튜닝 가이드

튜닝 파일: [config/slow_finish_first.yaml](config/slow_finish_first.yaml)

### 토픽과 시작 방식

- `scan_topic`: LiDAR `LaserScan` 입력 토픽입니다. 기본은 `/scan`입니다.
- `imu_topic`: IMU 입력 토픽입니다. gyro yaw 적분과 전복 정지에 씁니다. 기본은 `/imu`입니다.
- `odom_topic`: odom 입력 토픽입니다. stuck 복구 판단에 씁니다. 기본은 `/odom`입니다.
- `cmd_vel_topic`: 속도 명령 출력 토픽입니다. 기본은 `/cmd_vel`입니다.
- `cmd_vel_stamped`: `/cmd_vel` 타입이 `geometry_msgs/msg/TwistStamped`이면 `true`, TurtleBot3 기본 `geometry_msgs/msg/Twist`이면 `false`로 둡니다.
- `auto_start`: launch 직후 바로 주행하려면 `true`, 서비스 호출 전까지 정지하려면 `false`를 권장합니다.
- `reset_heading_on_start`: start 서비스가 들어올 때 gyro yaw 기준을 0도로 다시 잡을지 정합니다. 보통 `true`로 둡니다.

### 속도와 기본 안전거리

- `control_period_ms`: 제어 주기입니다. 기본 `100 ms`가 무난하고, 너무 키우면 반응이 늦습니다.
- `report_period_ms`: 상태 로그 출력 주기입니다. 로그가 너무 많으면 키우세요.
- `max_linear_speed`: 최대 전진 속도입니다. 완주 우선이면 `0.08 ~ 0.12`를 추천합니다.
- `min_linear_speed`: gap 주행 중 최소 전진 속도입니다. 너무 낮으면 멈칫거리고, 너무 높으면 좁은 곳에서 밀고 들어갑니다.
- `max_angular_speed`: 회전 명령 상한입니다. 회전이 과격하면 낮추고, 코너를 못 돌면 올립니다.
- `emergency_stop_distance_m`: 전방이 이 거리 이하이면 즉시 전진을 멈추고 회전합니다.
- `front_stop_distance_m`: 전방이 이 거리 이하이면 막힌 것으로 보고 회전합니다. 좁은 미션에서 자주 멈추면 낮추고, 충돌하면 올립니다.
- `front_slow_distance_m`: 전방 거리가 이 값 안으로 들어오면 점진적으로 감속합니다.
- `side_stop_distance_m`: 좌우 벽이 너무 가까울 때 강제로 반대 방향으로 회전시키는 기본 거리입니다.
- `safe_gap_distance_m`: 빈 공간 후보로 인정할 최소 거리입니다. 좁은 통로를 너무 못 들어가면 낮추고, 위험한 틈까지 들어가면 올립니다.
- `target_distance_cap_m`: LiDAR 거리를 점수 계산에 반영할 최대값입니다. 먼 거리 하나가 과하게 유리해지는 것을 막습니다.

### LiDAR 후보 탐색

- `search_angle_deg`: 전방 기준 좌우 몇 도까지 후보 gap을 찾을지 정합니다. 넓히면 옆길 후보를 더 많이 보고, 좁히면 전방 진행성이 커집니다.
- `gap_window_deg`: 후보 방향 주변 몇 도를 함께 보고 안전성을 판단할지 정합니다. 키우면 보수적이고, 줄이면 좁은 틈을 더 잘 후보로 잡습니다.
- `sample_step_deg`: LiDAR 각도를 몇 도 간격으로 샘플링할지 정합니다. 줄이면 촘촘하지만 계산이 늘고, 키우면 빠르지만 듬성해집니다.
- `front_window_deg`: 전방 거리 판단에 사용할 반각입니다. 정면 장애물에 민감하게 반응하려면 키웁니다.
- `side_angle_deg`: 좌우 거리 판단을 몇 도 방향에서 할지 정합니다. 벽과 나란히 달릴 때 기준이 안 맞으면 조정합니다.
- `side_window_deg`: 좌우 거리 판단에 사용할 반각입니다.
- `narrow_passage_enabled`: 작은 문틈 뒤에 대각선 벽이 있어 `gap_window_deg`의 최소 거리가 낮게 보이는 경우에도, 폭과 중심 ray 조건이 맞으면 천천히 진입하는 좁은 통로 모드를 켭니다.
- `narrow_passage_min_center_distance_m`: 좁은 통로 후보의 중심 ray가 최소 이 거리 이상 보여야 합니다. 대각 벽 때문에 갭을 못 들어가면 낮추고, 앞벽에 너무 붙으면 올립니다.
- `narrow_passage_min_sector_distance_m`: 좁은 통로 후보 주변 섹터 안에서 허용할 최소 LiDAR 거리입니다. 갭 모서리 때문에 막히면 낮추고, 모서리를 치면 올립니다.
- `narrow_passage_max_angle_deg`: 전방 기준 이 각도 안의 좁은 통로만 특수 후보로 인정합니다. 옆의 작은 틈까지 따라가면 낮춥니다.
- `narrow_passage_max_width_m`: 이 폭 이하의 measured gap만 좁은 통로로 봅니다. 넓은 공간까지 좁은 통로 모드로 느려지면 낮춥니다.
- `narrow_passage_min_depth_gain_m`: 중심 ray가 양쪽 경계 평균 거리보다 최소 이만큼 더 깊어야 합니다. 평평한 벽을 갭으로 오인하면 올리고, 실제 작은 갭을 놓치면 낮춥니다.
- `narrow_passage_bonus`: 좁은 통로 후보 점수 가산점입니다. 작은 갭 대신 옆의 먼 공간을 자꾸 고르면 올리고, 작은 갭에 과하게 집착하면 낮춥니다.
- `narrow_passage_speed_m_s`: 좁은 통로 모드에서 사용할 전진 속도 상한입니다.

### 후보 점수와 조향

- `heading_gain`: 선택된 gap 방향으로 회전하는 강도입니다. 목표 gap을 못 따라가면 올리고, 좌우로 흔들리면 낮춥니다.
- `centering_gain`: 좌우 벽 사이 중앙으로 가려는 강도입니다. 벽에서 너무 멀어져 출구를 놓치면 낮추고, 한쪽 벽에 자주 붙으면 올립니다.
- `forward_bias_weight`: 전방에서 많이 벗어난 후보에 주는 감점입니다. 직진성을 높이고 싶으면 올리고, 옆 출구를 더 잘 찾게 하려면 낮춥니다.
- `heading_alignment_weight`: 후보 gap 중 start yaw 기준 heading 오차가 작은 방향에 주는 가산점입니다. 역방향/옆길 선택을 줄이고 싶으면 올리고, 필요한 우회까지 막으면 낮춥니다.
- `turn_slowdown_gain`: 회전량이 클수록 전진 속도를 줄이는 정도입니다. 코너에서 벽을 치면 올리고, 너무 느리면 낮춥니다.
- `recovery_turn_speed`: 막혔을 때 제자리 회전 속도입니다.
- `recovery_flip_seconds`: 같은 방향으로 이 시간 이상 회전해도 못 빠져나오면 회전 방향을 뒤집습니다.
- `lost_scan_timeout_seconds`: 이 시간 이상 `/scan`이 갱신되지 않으면 정지합니다.
- `prefer_left_recovery`: 좌우 여유가 비슷할 때 복구 회전 기본 방향입니다.

### Dynamic Wall Assist

- `wall_assist_enabled`: 좁은 양벽 구간에서 중앙 정렬 대신 가까운 벽과 목표 거리를 유지하는 보조 제어를 켭니다. 좌/우수법 금지 기준이 엄격하면 `false`로 끄세요.
- `wall_assist_side_detect_m`: 좌우 벽이 이 거리 안에 있으면 좁은 구간으로 보고 wall assist 후보가 됩니다.
- `wall_assist_target_distance_m`: 따라갈 벽과 유지하려는 목표 거리입니다. 벽에 더 붙어야 하면 낮추고, 충돌하면 올립니다.
- `wall_assist_gain`: 목표 거리 오차를 회전 명령으로 바꾸는 강도입니다. 벽을 못 따라가면 올리고, 흔들리면 낮춥니다.
- `wall_assist_max_angular_speed`: wall assist가 낼 수 있는 최대 회전 속도입니다.
- `wall_assist_hard_stop_distance_m`: wall assist 중에도 이 거리보다 벽이 가까우면 강제로 멀어지게 합니다.
- `wall_assist_gap_width_max_m`: 측정된 gap 폭이 이 값 이하이면 좁은 통로로 보고 wall assist를 켤 수 있습니다.

### IMU, Gyro, 역방향 복구

- `tilt_stop_deg`: IMU roll/pitch 중 큰 값이 이 각도를 넘으면 정지합니다. 넘어짐 감지가 너무 민감하면 올리고, 늦으면 낮춥니다.
- `imu_timeout_seconds`: 이 시간 이상 IMU가 갱신되지 않으면 정지합니다.
- `gyro_yaw_sign`: gyro yaw 방향이 실제 회전 방향과 반대일 때 `-1.0`으로 바꿉니다.
- `gyro_angular_deadband_rad_s`: 이 값보다 작은 z축 각속도는 노이즈로 보고 적분하지 않습니다. 정지 중 yaw가 흐르면 올립니다.
- `gyro_integration_max_dt_seconds`: IMU 메시지 간격이 이 값보다 크면 그 구간은 적분하지 않습니다.
- `odom_timeout_seconds`: stuck 판단에 사용할 `/odom` timeout입니다.
- `reverse_heading_threshold_deg`: start 시 reset한 gyro yaw에서 이 각도 이상 벗어난 채 전진하면 역방향 후보로 봅니다.
- `reverse_hold_seconds`: 역방향 후보 상태가 이 시간 이상 지속되면 복구 회전에 들어갑니다.
- `reverse_linear_threshold`: 전진 명령이 이 값보다 클 때만 역방향 주행으로 판단합니다.
- `heading_recovery_turn_speed`: 역방향 복구 때 기준 yaw 쪽으로 제자리 회전하는 속도입니다.
- `reverse_resume_threshold_deg`: gyro yaw 오차가 이 각도 이하로 줄면 역방향 복구를 끝냅니다.

### Stuck 복구

- `stuck_recovery_enabled`: 전진 명령이 있는데 `/odom` 움직임이 거의 없을 때 후진/회전 복구를 켭니다.
- `stuck_command_linear_threshold`: 전진 명령이 이 값보다 클 때만 stuck 판단을 시작합니다.
- `stuck_odom_linear_threshold`: `/odom` 속도가 이 값보다 작으면 거의 못 움직이는 상태로 봅니다.
- `stuck_min_progress_m`: stuck 판단 시간 동안 이 거리보다 덜 움직이면 stuck 후보로 유지합니다.
- `stuck_hold_seconds`: stuck 후보 상태가 이 시간 이상 지속되면 복구에 들어갑니다.
- `stuck_backup_speed`: stuck 복구 때 후진 속도입니다. YAML 값은 양수로 두면 코드에서 후진 방향으로 씁니다.
- `stuck_backup_seconds`: 후진 지속 시간입니다.
- `stuck_turn_speed`: 후진 후 제자리 회전 속도입니다.
- `stuck_turn_seconds`: 후진 후 제자리 회전 시간입니다.
- `stuck_cooldown_seconds`: stuck 복구 직후 다시 stuck 복구에 들어가기 전 대기 시간입니다.

### Gap 폭 필터

- `enforce_gap_width`: 후보 방향 양쪽 장애물 사이 폭을 계산해서 좁은 틈을 거를지 정합니다.
- `min_passage_width_m`: 코사인법칙으로 계산한 양쪽 장애물 사이 폭이 이 값 이상일 때만 gap 후보로 인정합니다. 로봇 폭과 여유를 합친 값으로 잡습니다.
- `gap_width_search_deg`: 후보 방향 기준 좌우 몇 도 안에서 gap의 양쪽 장애물을 찾을지 정합니다.
- `gap_width_obstacle_max_range_m`: 이 거리 안에 있는 LiDAR 점만 gap 경계 장애물로 봅니다. 너무 멀리 있는 벽까지 경계로 잡으면 낮추고, 경계를 못 찾으면 올립니다.
- `gap_width_boundary_drop_m`: gap 중심 ray보다 이 거리 이상 가까운 LiDAR 점만 좌우 경계로 봅니다. 갭 뒤 대각 벽 표면을 경계로 오인하면 올리고, 실제 갭 경계를 못 잡으면 낮춥니다.

## 실전 운용 팁

처음 테스트할 때는 로봇을 들어 올릴 수 있는 상태에서 `/cmd_vel` 방향이 맞는지 먼저 확인하세요. 전방 장애물에 가까워졌을 때 `BLOCKED_TURN` 또는 `EMERGENCY_TURN` 로그가 나오며 제자리 회전하면 정상입니다.

경로 폭이 좁아 로봇이 자주 멈추면 `front_stop_distance_m`을 `0.30` 정도로 낮추거나 `safe_gap_distance_m`을 `0.40`으로 낮춥니다. 반대로 충돌 가능성이 보이면 `front_stop_distance_m`을 `0.38 ~ 0.42`로 올리고 `max_linear_speed`를 `0.08`로 낮추세요.

## 센서 구성

현재 안전 주행 기본 설정은 라이다, IMU, odom이 필요합니다. RPLIDAR C1 또는 TurtleBot 기본 라이다 중 하나가 `/scan`을 publish하고, IMU는 `/imu`, odom은 `/odom`을 publish해야 합니다. `/odom`이 없어도 기본 주행과 기울기/역방향 정지는 가능하지만 stuck 복구는 동작하지 않습니다.

Realsense D435i는 필수로 쓰지 않습니다. 깊이 카메라까지 넣으면 복잡도가 크게 올라가므로, 이번 과제의 완주 목표에는 라이다 reactive 주행을 먼저 안정화하는 쪽을 추천합니다.

노드를 종료할 때는 여러 번 0속도를 publish해서 바퀴를 멈추도록 했습니다. 다만 이것은 모터 torque-off가 아니라 `/cmd_vel=0` 정지 명령입니다. 실제 모터 전원/토크 해제는 TurtleBot3 OpenCR 또는 bringup 드라이버가 별도로 지원해야 합니다.
