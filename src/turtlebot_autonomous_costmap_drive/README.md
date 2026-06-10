# turtlebot_autonomous_costmap_drive

`turtlebot_autonomous_costmap_drive`는 기존 `turtlebot_autonomous_gap_drive`를 바탕으로 만든 local costmap 기반 자율주행 패키지입니다. 목표는 빠른 기록보다 안정적인 완주입니다. SLAM/Nav2 전체를 붙이지 않고도 `/scan`, `/imu`, `/odom`만으로 짧은 시간 누적 costmap을 만들고, 그 위에서 후보 궤적을 평가해 `/cmd_vel`을 publish합니다.

기존 gap-drive는 LaserScan 한 프레임에서 가장 좋은 gap 방향을 고르는 reactive 방식입니다. 이 패키지는 같은 센서와 안전장치를 쓰지만, 작은 장애물 끝점, 대각 벽 뒤 gap, 좁은 문틈, C/U자 구조처럼 한 프레임 ray만으로 애매한 상황을 local costmap과 footprint collision check로 판단합니다.

상세한 코드 흐름은 [Flow.md](Flow.md)에, gap-drive 대비 상황별 동작 차이는 [SCENARIO_COMPARISON.md](SCENARIO_COMPARISON.md)에 정리되어 있습니다.

## 추천 방향

- 1순위: `costmap_drive.launch.py`로 저속 완주 테스트
- 2순위: RViz에서 `/costmap_drive/local_costmap`과 `/costmap_drive/trajectory_markers`를 보며 튜닝
- 3순위: rosbag replay로 기존 `gap_drive`와 같은 상황의 `/cmd_vel`과 mode 로그 비교

이 패키지는 ROS2 Foxy 호환을 목표로 C++14와 기본 message 패키지 위주로 작성했습니다. Nav2, AMCL, saved map, RealSense depth는 v1 범위에 넣지 않았습니다.

## 동작 개요

노드는 `/scan`의 `sensor_msgs/msg/LaserScan`, `/imu`의 `sensor_msgs/msg/Imu`, `/odom`의 `nav_msgs/msg/Odometry`를 구독합니다. `/scan`의 유효 hit point를 `/odom` pose 기준으로 짧게 저장하고, 제어 주기마다 `base_link` 기준 rolling local costmap으로 다시 변환합니다. 현재 scan ray는 free space로 표시하고, 저장된 obstacle point는 occupied cell로 표시한 뒤 `inflation_radius_m`만큼 추가 안전 cost를 퍼뜨립니다. 실제 로봇 크기인 `robot_radius_m`은 후보 궤적의 원형 footprint 검사에서 적용합니다.

이후 여러 `angular_z` 후보를 샘플링하고, unicycle 모델로 `trajectory_horizon_s`만큼 미래 궤적을 예측합니다. 각 궤적의 원형 footprint가 costmap에서 장애물 또는 높은 inflation cost를 밟으면 reject합니다. 추가로, LiDAR에서 관측된 gap의 boundary 폭이 `narrow_gap_min_width_m`보다 작으면 그 방향을 통과 불가능 sector로 기록하고 해당 sector를 향하는 궤적도 hard reject합니다. 살아남은 후보 중 전진성, 장애물 cost, unknown cost, 회전량, 직전 명령 변화량, 기준 heading 유지 점수, long-range target 보상, narrow gap target 보상을 합쳐 가장 좋은 궤적을 선택합니다. 전방이 가까울 때는 일반 전진 후보의 `min_linear_speed`는 유지하되, 선속도 0의 제자리 회전 후보도 추가로 평가합니다. 전방이 가까워질수록 start heading 유지와 long-range target 보상은 자동으로 약해져 C/U자 구조에서 local escape가 우선됩니다.

기본 출력은 TurtleBot3 Foxy에서 흔히 쓰는 `geometry_msgs/msg/Twist` 타입의 `/cmd_vel`입니다. 기존 수업 코드처럼 `/cmd_vel`이 `geometry_msgs/msg/TwistStamped` 타입이어야 하는 환경이면 YAML에서 `cmd_vel_stamped: true`로 바꾸세요.

## 빌드

워크스페이스 루트에서 실행합니다.
.yaml의 사항을 반영하게 하려면, 꼭 심볼릭 링크 빌드 진행

```bash
colcon build --symlink-install --packages-select turtlebot_autonomous_costmap_drive
source install/setup.bash
```

## 실행 방법

터틀봇 bringup과 LiDAR driver가 먼저 실행되어 있어야 합니다.

```bash
source install/setup.bash
ros2 launch turtlebot_autonomous_costmap_drive costmap_drive.launch.py
```

노드는 기본값 `auto_start: false`라서 launch 직후에는 정지 상태입니다. 다음 서비스로 주행을 시작하고 멈춥니다.

```bash
ros2 service call /turtlebot_autonomous_costmap_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /turtlebot_autonomous_costmap_drive/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 실행 전 확인

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
ros2 topic info /cmd_vel
```

RViz에서 함께 보면 좋은 토픽입니다.

```text
/costmap_drive/local_costmap
/costmap_drive/trajectory_markers
```

## 주행 모드

- `STOPPED`: 서비스 start 전 또는 stop 요청 후 0속도입니다.
- `WAIT_SENSORS`: IMU 또는 odom이 없거나 timeout입니다.
- `WAIT_SCAN`: 아직 LiDAR scan을 받지 못했습니다.
- `SCAN_TIMEOUT`: 마지막 scan이 너무 오래되었습니다.
- `TILT_STOP`: IMU roll/pitch가 `tilt_stop_deg`를 넘었습니다.
- `COSTMAP_DRIVE`: local costmap에서 가장 낮은 cost의 전진 궤적을 따라갑니다.
- `NARROW_COSTMAP_DRIVE`: 양벽이 가까운 통로를 저속으로 통과합니다.
- `NARROW_GAP_COSTMAP_DRIVE`: 코사인법칙으로 폭이 검증된 좁은 gate를 저속으로 향합니다.
- `BLOCKED_TURN`: 안전한 전진 궤적이 없어 제자리 회전합니다.
- `EMERGENCY_TURN`: 전방 장애물이 매우 가까워 즉시 전진을 멈추고 회전합니다.
- `REVERSE_WARN`: 기준 heading 반대 방향으로 전진 중이지만 아직 hold 시간 전입니다.
- `REVERSE_RECOVERY`: 기준 heading 쪽으로 제자리 회전합니다.
- `STUCK_BACKUP`: 전진 명령이 있는데 odom상 진행이 없어 짧게 후진합니다.
- `STUCK_TURN`: 후진 후 더 여유 있는 방향으로 제자리 회전합니다.
- `BLOCKED_BACKUP`: 막힘 회전 상태가 오래 지속되어 LiDAR 시야 확보를 위해 짧게 후진합니다.
- `BLOCKED_ESCAPE_TURN`: blocked backup 후 더 여유 있는 방향으로 짧게 회전합니다.

## 튜닝 기본 방향

완주 우선 기본 전략은 아래 순서입니다.

1. 충돌이 있으면 속도보다 footprint와 inflation을 먼저 봅니다.
2. 좁은 통로를 못 지나가면 `robot_radius_m`, `inflation_radius_m`, `lethal_cost_threshold`를 한 번에 크게 바꾸지 말고 하나씩 낮춥니다.
3. 코너에서 벽을 치면 `front_slow_distance_m`, `cost_obstacle_weight`, `turn_slowdown_gain`을 올립니다.
4. 열린 공간에서 너무 느리면 `max_linear_speed`와 `progress_reward_weight`를 조금 올립니다.
5. 좌우 흔들림은 `cost_smooth_weight`, `cost_turn_weight`, `max_angular_speed`로 잡습니다.
6. C/U자 구조에서 늦게 빠져나오면 `trajectory_horizon_s`, `grid_forward_m`, `obstacle_memory_seconds`를 조금 올립니다.
7. 대각선 벽 뒤의 긴 출구를 늦게 잡으면 `long_range_clearance_weight`, `long_range_clearance_max_range_m`, `long_range_clearance_search_deg`를 조정합니다.
8. 좁은 문틈이나 S자 gate를 지나치면 `narrow_gap_bonus_weight`, `narrow_gap_min_width_m`, `narrow_gap_hold_seconds`를 조정합니다.
9. `BLOCKED_TURN`에서 좌우로 계속 반전하면 `blocked_recovery_hold_seconds`, `stuck_backup_seconds`, `stuck_turn_seconds`를 조정합니다.

튜닝은 한 번에 하나씩 바꾸고 같은 Gazebo world 또는 같은 rosbag replay에서 비교하세요. costmap 방식은 파라미터끼리 상호작용이 크기 때문에 여러 값을 동시에 바꾸면 원인 파악이 어려워집니다.

## 파라미터 상세 설명

튜닝 파일: [config/complete_first.yaml](config/complete_first.yaml)

### 토픽과 시작 방식

- `scan_topic`: LiDAR `LaserScan` 입력 토픽입니다. 기본은 `/scan`입니다.
- `imu_topic`: IMU 입력 토픽입니다. 기울기 정지와 gyro yaw 적분에 씁니다.
- `odom_topic`: odom 입력 토픽입니다. obstacle memory의 좌표 기준과 stuck 복구에 씁니다.
- `cmd_vel_topic`: 속도 명령 출력 토픽입니다. 기본은 `/cmd_vel`입니다.
- `cmd_vel_stamped`: 출력 타입을 `TwistStamped`로 바꿀지 정합니다. TurtleBot3 기본은 `false`입니다.
- `local_costmap_topic`: RViz 확인용 local occupancy grid 출력 토픽입니다.
- `trajectory_marker_topic`: 선택된 후보 궤적 marker 출력 토픽입니다.
- `costmap_frame_id`: local costmap과 marker의 frame입니다. 기본은 `base_link`입니다.
- `auto_start`: launch 직후 바로 주행할지 정합니다. 실제 로봇에서는 `false`를 권장합니다.
- `reset_heading_on_start`: start 서비스가 들어올 때 gyro yaw 기준을 0도로 다시 잡을지 정합니다.
- `require_odom`: odom이 준비되지 않으면 주행을 막을지 정합니다. costmap memory와 stuck 복구를 위해 `true`를 권장합니다.

### 속도와 복구 회전

- `control_period_ms`: 제어 주기입니다. 기본 100ms가 무난합니다.
- `report_period_ms`: 상태 로그 출력 주기입니다.
- `max_linear_speed`: 일반 costmap 주행 최대 선속도입니다. 완주 우선이면 `0.08 ~ 0.12`를 추천합니다.
- `min_linear_speed`: 일반 전진 후보의 최소 선속도입니다. 너무 낮으면 stuck/block 상태가 아닌데도 거의 멈출 수 있고, 너무 높으면 좁은 곳에서 밀고 들어갑니다. 전방이 가까울 때 필요한 정지는 이 값을 0으로 낮추지 않고 별도 제자리 회전 후보로 처리합니다.
- `max_angular_speed`: 후보 궤적 샘플링과 최종 회전 속도 상한입니다.
- `recovery_turn_speed`: `BLOCKED_TURN` 또는 `EMERGENCY_TURN`에서 쓰는 제자리 회전 속도입니다.
- `recovery_flip_seconds`: 같은 방향으로 이 시간 이상 회전해도 못 빠져나오면 회전 방향을 뒤집습니다.
- `prefer_left_recovery`: 좌우 여유가 비슷할 때 기본 회전 방향입니다.

### Scan 해석과 기본 거리

- `emergency_stop_distance_m`: 전방 최단거리가 이 값보다 가까우면 즉시 전진 후보 평가를 건너뛰고 회전합니다.
- `front_stop_distance_m`: 이 거리 이하에서는 전진 속도 계산이 가장 보수적으로 내려갑니다.
- `front_slow_distance_m`: 전방 거리가 이 값 안에 들어오면 점진적으로 감속합니다.
- `side_angle_deg`: 좌우 거리 판단에 사용할 중심 각도입니다. 정면을 `0 deg`, 왼쪽을 `+`, 오른쪽을 `-`로 봅니다.
- `side_window_deg`: 좌우 거리 판단 섹터 반각입니다. 예를 들어 `side_angle_deg: 75`, `side_window_deg: 18`이면 왼쪽은 `57~93 deg`, 오른쪽은 `-93~-57 deg` 범위의 최소 거리를 봅니다.
- `front_window_deg`: 전방 거리 판단 섹터 반각입니다. 예를 들어 `front_window_deg: 18`이면 정면 `-18~+18 deg` 범위의 최소 거리를 봅니다. 즉 전체 전방 섹터 폭은 `36 deg`입니다.
- `scan_sample_step_deg`: scan ray를 몇 도 간격으로 costmap에 반영할지 정합니다. 낮추면 촘촘하지만 계산량이 늘어납니다.
- `scan_obstacle_min_range_m`: 이 거리 이하의 LiDAR 값은 0, 본체 반사, 근거리 튐값으로 보고 obstacle에도 free clearing에도 쓰지 않습니다.
- `scan_obstacle_max_range_m`: obstacle memory에 넣을 최대 LiDAR 거리입니다. 너무 크면 먼 벽까지 로컬 판단을 방해할 수 있습니다.
- `raytrace_max_range_m`: 현재 scan ray로 free space를 표시할 최대 거리입니다. 보통 `scan_obstacle_max_range_m`과 같거나 더 짧게 둡니다. 더 길어도 코드 오류는 없지만, `scan_obstacle_max_range_m` 밖의 hit를 장애물로 저장하지 않으면서 그 뒤쪽을 free로 칠 수 있어 해석이 공격적으로 변합니다.
- `lost_scan_timeout_seconds`: 마지막 scan이 이 시간 이상 갱신되지 않으면 정지합니다.

`scan_callback`은 들어온 LaserScan을 먼저 정리합니다. `Inf`, `NaN`, `0.0` 값은 장애물이나 열린 공간으로 직접 쓰지 않고, 같은 scan 안에서 바로 이전 각도의 유효 range 값으로 대체합니다. 맨 앞처럼 이전 유효 값이 없으면 `NaN`으로 남겨 이후 costmap/free-space/long-range 판단에서 무시합니다. `scan_obstacle_min_range_m` 이하 값과 `range_max`보다 큰 값도 costmap 판단에는 쓰지 않습니다.

### Local Costmap

- `grid_resolution_m`: costmap 한 cell의 크기입니다. 작을수록 정밀하지만 계산량이 늘어납니다.
- `grid_forward_m`: 로봇 앞쪽으로 볼 costmap 거리입니다. 코너와 함정을 더 일찍 보려면 올립니다.
- `grid_back_m`: 로봇 뒤쪽으로 유지할 costmap 거리입니다. 후진 복구나 회전 중 뒤쪽 장애물을 보려면 올립니다.
- `grid_half_width_m`: 좌우 costmap 반폭입니다. 예를 들어 `0.95`이면 로봇 중심선 기준 왼쪽 `0.95 m`, 오른쪽 `0.95 m`, 총 `1.90 m` 폭의 local grid를 만듭니다. 좁으면 회전 후보가 map 밖으로 나가 reject될 수 있습니다.
- `obstacle_memory_seconds`: scan hit point를 기억할 시간입니다. 짧으면 반응성은 좋지만 구조 기억이 약하고, 길면 지나간 장애물이 오래 남습니다.
- `obstacle_memory_max_points`: obstacle memory 최대 point 수입니다. 오래 주행할 때 메모리와 계산량을 제한합니다.
- `robot_radius_m`: 후보 궤적 충돌 검사에 쓰는 원형 footprint 반지름입니다. 실제 TurtleBot3 Burger 폭과 최소 여유를 합쳐 잡습니다.
- `inflation_radius_m`: obstacle 주변에 추가로 cost를 퍼뜨릴 거리입니다. `robot_radius_m`에 더해지는 추가 margin으로 생각하면 됩니다. 벽을 치면 올리고, 좁은 문틈을 못 지나가면 낮춥니다.
- `lethal_cost_threshold`: 후보 궤적이 이 cost 이상인 cell을 밟으면 collision으로 reject합니다. 낮추면 보수적이고, 높이면 좁은 곳을 더 통과합니다.
- `unknown_cell_cost`: unknown cell을 지나갈 때 부여할 cost입니다.
- `allow_unknown_trajectory`: unknown cell 통과 후보를 허용할지 정합니다. 실제 완주 우선에서는 `true`가 덜 답답합니다.

### 후보 궤적 점수

- `trajectory_horizon_s`: 후보 궤적을 몇 초 앞까지 예측할지 정합니다. 길면 코너와 함정을 일찍 보지만 좁은 공간에서 보수적입니다.
- `trajectory_dt_s`: 궤적 예측 샘플 간격입니다. 낮추면 정밀하지만 계산량이 늘어납니다.
- `angular_sample_count`: 최종 회전 각도가 아니라 `/cmd_vel.angular.z` 회전 속도 후보를 몇 개 볼지 정합니다. 예를 들어 `max_angular_speed: 0.85`, `angular_sample_count: 15`이면 `-0.85 ~ +0.85 rad/s` 사이의 회전 속도 명령 15개를 만들고, 각 명령을 `trajectory_horizon_s` 동안 시뮬레이션합니다. 홀수로 두면 `0.0 rad/s` 직진 후보가 포함됩니다. 전방 거리가 가까우면 멈춘 상태에서 돌 수 있는 pivot 후보가 추가되어 로그의 `traj=evaluated/rejected`가 `angular_sample_count`보다 커질 수 있습니다.
- `turn_slowdown_gain`: 회전량이 클수록 선속도를 낮추는 정도입니다.
- `progress_reward_weight`: 앞으로 나아간 거리에 주는 보상입니다. 올리면 더 적극적으로 전진합니다.
- `cost_obstacle_weight`: obstacle/inflation cost 감점입니다. 벽을 스치면 올리고, 너무 소극적이면 낮춥니다.
- `cost_unknown_weight`: unknown cell 비율 감점입니다. 올리면 관측된 free space를 더 선호합니다.
- `cost_turn_weight`: 큰 회전량 감점입니다. 올리면 직진성이 강해지고, 낮추면 코너를 더 적극적으로 돕니다.
- `cost_smooth_weight`: 직전 `angular_z`와의 차이 감점입니다. 올리면 좌우 흔들림이 줄지만 반응이 느려질 수 있습니다.
- `cost_lateral_weight`: 최종 궤적이 좌우로 벗어나는 정도의 감점입니다. 올리면 중앙 진행성이 강해집니다.
- `heading_alignment_weight`: start 시 reset한 gyro heading과 후보 최종 heading 차이 감점입니다. 역방향 진행을 줄이는 데 도움됩니다. 전방 거리가 `front_slow_distance_m` 안으로 들어오면 이 감점은 front clearance ratio만큼 자동으로 약해집니다.

### Long-range clearance 보상

- `long_range_clearance_enabled`: 긴 열린 ray 방향 보상을 켤지 정합니다. gap-drive의 장거리 출구 감각을 costmap 점수에 섞는 기능입니다.
- `long_range_clearance_weight`: 후보 최종 heading이 긴 열린 ray 방향과 가까울 때 주는 보상입니다. 대각선 벽 뒤 출구를 늦게 찾으면 올리고, 옆방으로 빨려 들어가면 낮춥니다. 전방이 가까워질수록 이 보상도 front clearance ratio만큼 약해집니다. 폭 부족으로 hard reject된 small gap 각도 범위는 long-range 후보에서도 제외됩니다.
- `long_range_clearance_max_range_m`: 긴 ray 판단에 사용할 최대 거리입니다. 이 값을 넘는 유효 range는 같은 최대 열린 거리로 봅니다.
- `long_range_clearance_min_range_m`: 이 거리보다 짧은 ray는 긴 출구 후보로 보지 않습니다.
- `long_range_clearance_search_deg`: 현재 로봇 기준에서 start heading 방향 주변 몇 도까지 긴 출구 후보를 찾을지 정합니다. 너무 넓으면 옆방까지 목표로 잡고, 너무 좁으면 회전 후 출구를 놓칩니다.
- `long_range_clearance_window_deg`: 단일 ray가 아니라 해당 중심 주변 반각 안의 최소 거리를 함께 봅니다. 올리면 로봇 폭보다 좁은 틈에 덜 속지만, 좁은 문을 놓칠 수 있습니다.
- `long_range_clearance_alignment_deg`: 후보 궤적 최종 heading과 긴 열린 ray 방향의 오차가 이 각도 안에 들어올 때 보상을 줍니다.
- `long_range_clearance_start_bias`: 긴 ray 후보를 고를 때 start heading에 가까운 방향을 얼마나 선호할지 정합니다.

RViz의 `/costmap_drive/trajectory_markers`에는 파란색 최종 궤적과 함께 초록색 `long_range_target` 선이 표시됩니다. 터미널 로그의 `long=yes 15deg/1.50m`는 현재 선택된 장거리 열린 방향과 거리를 뜻합니다. 빨간색 rejected gap 폭과 같은 각도에 초록선이 겹치면 안 됩니다.

### Narrow gap/gate target 보상

- `narrow_gap_target_enabled`: gap-drive의 좁은 통로 폭 측정 기능을 costmap 점수에 섞을지 정합니다.
- `narrow_gap_bonus_weight`: 후보 최종 heading이 검증된 narrow gap 중심과 가까울 때 주는 보상입니다.
- `narrow_gap_min_width_m`: 코사인법칙으로 계산한 좌우 boundary 폭이 이 값 이상이어야 통과 가능한 좁은 gate로 봅니다. 이 값보다 작은 관측 gap 방향은 일반 costmap 궤적에서도 hard reject됩니다. 현재 `robot_radius_m: 0.105`에서는 `0.23 m` 정도가 시작점입니다.
- `narrow_gap_max_width_m`: 이 값보다 넓은 opening은 narrow gate가 아니라 일반 열린 공간/방으로 보고 gate target에서 제외합니다.
- `narrow_gap_search_deg`: 로봇 정면 기준 좌우 몇 도까지 gate 후보를 찾을지 정합니다. S자 구조에서는 long-range search보다 넓게 둘 수 있습니다.
- `narrow_gap_sector_half_width_deg`: 중심 ray 주변 최소 clearance를 확인할 반각입니다.
- `narrow_gap_boundary_search_deg`: 중심 ray 좌우로 boundary를 찾을 최대 각도입니다.
- `narrow_gap_boundary_obstacle_max_range_m`: boundary로 인정할 장애물 최대 거리입니다. 올리면 먼 벽도 boundary로 잡고, 낮추면 가까운 구조물만 boundary로 봅니다.
- `narrow_gap_boundary_drop_m`: 중심 ray에서 좌우로 boundary를 찾을 때, boundary ray가 중심 ray보다 최소 이 거리 이상 가까워야 경계로 인정합니다. 작은 range 흔들림이나 완만한 V자 벽을 gate 경계로 잡지 않기 위한 1차 필터입니다. 값을 올리면 가짜 경계가 줄지만 실제 완만한 문턱을 놓칠 수 있습니다.
- `narrow_gap_min_center_distance_m`: gate 중심 ray가 최소 이 거리 이상 깊어야 합니다.
- `narrow_gap_min_sector_distance_m`: 중심 주변 섹터의 최소 clearance가 이 값 이상이어야 합니다.
- `narrow_gap_min_depth_gain_m`: 중심 ray가 좌우 boundary 중 더 먼 쪽보다도 이 거리 이상 깊어야 합니다. V자 코너가 gate로 잡히면 이 값을 올리고, 실제 230~300mm 문틈을 놓치면 낮춥니다.
- `narrow_gap_alignment_deg`: 후보 trajectory 최종 heading이 gate 중심과 이 각도 안에 들어오면 보상을 받습니다.
- `narrow_gap_hold_seconds`: 같은 gate가 비슷한 각도에서 계속 보이면 이 시간 동안 target 선택을 유지해 S자/방 구조에서 target 튐을 줄입니다.
- `narrow_gap_hold_max_angle_error_deg`: hysteresis로 같은 gate라고 볼 최대 각도 차이입니다.
- `narrow_gap_speed_m_s`: `NARROW_GAP_COSTMAP_DRIVE`에서 사용할 선속도 상한입니다.

RViz의 `/costmap_drive/trajectory_markers`에서 주황색 선은 narrow gap 중심 target, 노란색 짧은 선은 계산된 gate 폭, 빨간색 짧은 선은 폭이 부족해서 hard reject된 gap 폭입니다. V자 코너에서도 주황/노랑 marker가 자주 뜨면 `narrow_gap_min_depth_gain_m`을 먼저 올리고, 그래도 gate 보상이 너무 강하면 `narrow_gap_bonus_weight`를 낮춥니다. 터미널 로그의 `gate=yes* 12deg/260mm/0.80m`는 gate target이 있고, `*`는 hysteresis로 이전 gate를 이어 잡았다는 뜻입니다. `small_gap=1/8`은 통과 불가능 gap sector 1개가 관측됐고, 후보 궤적 8개가 그 방향이라 reject됐다는 뜻입니다. `pivot=yes`는 전방이 가까워져 선속도 0의 제자리 회전 후보가 최종 선택됐다는 뜻입니다.

### 좁은 통로 모드

- `narrow_corridor_enabled`: 양쪽 벽이 가까울 때 저속 좁은 통로 모드를 쓸지 정합니다.
- `narrow_corridor_width_m`: 좌우 거리 합이 이 값 이하이면 좁은 통로 후보로 봅니다.
- `narrow_corridor_side_detect_m`: 양쪽 벽이 각각 이 거리 안에 있어야 좁은 통로로 봅니다.
- `narrow_corridor_speed_m_s`: `NARROW_COSTMAP_DRIVE`에서 사용할 전진 속도 상한입니다.

### IMU, Gyro, 역방향 복구

- `tilt_stop_deg`: IMU roll/pitch 중 큰 값이 이 각도를 넘으면 정지합니다.
- `imu_timeout_seconds`: IMU가 이 시간 이상 갱신되지 않으면 정지합니다.
- `gyro_yaw_sign`: gyro yaw 방향이 실제와 반대이면 `-1.0`으로 바꿉니다.
- `gyro_angular_deadband_rad_s`: 작은 z축 각속도를 노이즈로 무시하는 deadband입니다.
- `gyro_integration_max_dt_seconds`: IMU 메시지 간격이 이 값보다 크면 그 구간은 yaw 적분하지 않습니다.
- `odom_timeout_seconds`: odom timeout 기준입니다.
- `reverse_heading_threshold_deg`: start heading 반대 방향으로 이 각도 이상 벗어나 전진하면 역방향 후보로 봅니다.
- `reverse_hold_seconds`: 역방향 후보가 이 시간 이상 지속되면 복구 회전에 들어갑니다.
- `reverse_linear_threshold`: 전진 명령이 이 값보다 클 때만 역방향 주행으로 판단합니다.
- `heading_recovery_turn_speed`: 역방향 복구 때 기준 heading 쪽으로 도는 속도입니다.
- `reverse_resume_threshold_deg`: heading 오차가 이 값 이하가 되면 역방향 복구를 끝냅니다.

### Stuck 복구

- `stuck_recovery_enabled`: 전진 명령이 있는데 odom상 진행이 없을 때 후진/회전 복구를 켭니다.
- `stuck_command_linear_threshold`: 전진 명령이 이 값보다 클 때만 stuck 판단을 시작합니다.
- `stuck_odom_linear_threshold`: odom 속도가 이 값보다 낮으면 거의 못 움직이는 상태로 봅니다.
- `stuck_min_progress_m`: stuck 판단 시간 동안 이 거리보다 덜 움직이면 stuck 후보로 유지합니다.
- `stuck_hold_seconds`: stuck 후보가 이 시간 이상 지속되면 복구를 시작합니다.
- `stuck_backup_speed`: stuck 복구 때 후진 속도입니다. YAML 값은 양수로 두면 코드에서 후진 방향으로 씁니다.
- `stuck_backup_seconds`: 후진 지속 시간입니다.
- `stuck_turn_speed`: 후진 후 제자리 회전 속도입니다.
- `stuck_turn_seconds`: 후진 후 제자리 회전 시간입니다.
- `stuck_cooldown_seconds`: stuck 복구 직후 다시 stuck 복구에 들어가기 전 대기 시간입니다.
- `blocked_recovery_enabled`: `BLOCKED_TURN` 또는 `EMERGENCY_TURN`이 일정 시간 지속될 때 후진/회전 복구를 켭니다.
- `blocked_recovery_hold_seconds`: 막힘 회전 상태가 이 시간 이상 지속되면 `BLOCKED_BACKUP`을 시도합니다. 후진 직전 local costmap에서 뒤쪽 짧은 경로와 footprint가 clear인지 확인하며, 뒤가 장애물/unknown이면 후진을 건너뛰고 `BLOCKED_ESCAPE_TURN`으로 넘어갑니다. 너무 낮으면 불필요하게 복구가 걸리고, 너무 높으면 제자리 회전이 길어집니다.

## gap_drive 대비 핵심 차이

- `gap_drive`: 한 프레임 scan에서 후보 angle의 섹터 최소거리와 gap 폭을 계산합니다.
- `costmap_drive`: 짧게 누적한 obstacle memory와 현재 free ray로 local grid를 만들고, 로봇 footprint가 지나갈 수 있는 후보 궤적을 평가합니다.
- `gap_drive`: 작은 틈 뒤 대각 벽처럼 섹터 최소거리 하나가 낮으면 후보가 막힐 수 있습니다.
- `costmap_drive`: inflated obstacle 사이에 corridor가 남으면 저속으로 통과하고, 폭이 검증된 narrow gap target 방향 후보에는 추가 보상을 줍니다.
- `gap_drive`: 후보 중심 ray가 통과 가능해도 로봇 몸체가 장애물 끝에 걸릴 수 있습니다.
- `costmap_drive`: 원형 footprint collision check로 edge clipping을 줄입니다.

## rosbag 비교 팁

기존 bag을 replay하면서 새 노드의 출력이 실제 `/cmd_vel`과 섞이지 않게 하려면 별도 YAML을 만들어 `cmd_vel_topic: /cmd_vel_costmap_test`로 바꿔 실행하세요.

```bash
ros2 bag play ~/tb3_bags/candidate_run_01 --loop
ros2 launch turtlebot_autonomous_costmap_drive costmap_drive.launch.py params_file:=/path/to/replay_costmap.yaml
ros2 service call /turtlebot_autonomous_costmap_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
```

비교할 값은 mode, `cmd=(linear, angular)`, `traj=evaluated/rejected`, `score`, `avg_cost`, `unknown`, RViz의 selected trajectory입니다.
