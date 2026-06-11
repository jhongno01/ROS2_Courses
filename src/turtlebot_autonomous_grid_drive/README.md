# turtlebot_autonomous_grid_drive

`turtlebot_autonomous_grid_drive`는 기존 `costmap_drive`를 대체하지 않고 병렬로 추가한 실험용 주행 패키지입니다. trajectory rollout으로 후보 경로를 고르는 대신, 로봇 주변 360도 rolling grid를 만들고 vector field로 진행 방향을 정합니다.

핵심 의도는 다음입니다.

- 로봇 주변 반경 `grid_radius_m` 안을 `FREE=0`, `UNKNOWN=1`, `BLOCKED=2`로 분류한다.
- 통과 불가능한 작은 gap은 점수에서만 불리하게 보는 것이 아니라 실제 `BLOCKED`처럼 grid에 반영한다.
- `lowest_cost_vector + obstacle_repulsion_vector + heading_bias + reverse_bias`를 합쳐 `/cmd_vel`을 만든다.
- dead-end memory를 켜면 emergency/stuck이 발생한 방향의 odom corridor를 잠시 `1.5` cost로 기억한다.
- pivot은 메인 주행 전략이 아니라 emergency/stuck 상황의 fallback으로만 사용한다.

## Build

Ubuntu/ROS2 환경에서 빌드합니다.

```bash
colcon build --symlink-install --packages-select turtlebot_autonomous_grid_drive
source install/setup.bash
```

이 Mac 작업 환경에서는 `colcon`을 실행하지 않는 전제로 작성했습니다.

## Run

```bash
ros2 launch turtlebot_autonomous_grid_drive grid_drive.launch.py
```

다른 파라미터 파일을 쓰려면:

```bash
ros2 launch turtlebot_autonomous_grid_drive grid_drive.launch.py params_file:=/path/to/your.yaml
```

기본값은 `auto_start: false`라서 서비스로 켭니다.

```bash
ros2 service call /turtlebot_autonomous_grid_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /turtlebot_autonomous_grid_drive/set_enabled std_srvs/srv/SetBool "{data: false}"
```

Dead-end memory는 기본 YAML에서 꺼져 있습니다. 필요할 때 서비스로 켤 수 있습니다.

```bash
ros2 service call /turtlebot_autonomous_grid_drive/set_dead_end_memory std_srvs/srv/SetBool "{data: true}"
ros2 service call /turtlebot_autonomous_grid_drive/set_dead_end_memory std_srvs/srv/SetBool "{data: false}"
```

`false`를 호출하면 dead-end memory가 비활성화되고 기존 memory도 지워집니다.

## Topics

Subscribe:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `/imu` (`sensor_msgs/msg/Imu`)

Publish:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/grid_drive/local_grid` (`nav_msgs/msg/OccupancyGrid`)
- `/grid_drive/vector_markers` (`visualization_msgs/msg/MarkerArray`)

Service:

- `/turtlebot_autonomous_grid_drive/set_enabled` (`std_srvs/srv/SetBool`)
- `/turtlebot_autonomous_grid_drive/set_dead_end_memory` (`std_srvs/srv/SetBool`)

## RViz

Fixed Frame은 보통 `base_link`로 둡니다.

- Map: `/grid_drive/local_grid`
- MarkerArray: `/grid_drive/vector_markers`

Grid cell 의미:

- `FREE=0`: scan ray가 hit 전까지 확인한 빈 공간
- `UNKNOWN=1`: hit 뒤쪽이거나 scan이 확인하지 못한 공간
- `DEAD_END=1.5`: emergency/stuck으로 실패한 odom corridor의 임시 회피 비용
- `BLOCKED=2`: lidar hit, memory obstacle, 또는 통과 불가능한 small gap

OccupancyGrid 표시값은 RViz 호환을 위해 `FREE=0`, `DEAD_END=75`, `BLOCKED=100`, `UNKNOWN=-1`로 발행합니다.

Vector marker 색상:

- 초록: lowest cost 방향
- 자홍: obstacle repulsion 방향
- 파랑: 최초 heading bias
- 하늘색: reverse return bias
- 주황: 최종 합산 vector

## Algorithm

### 1. Scan sanitize

`/scan`의 `inf`, `nan`, `0.0` 또는 너무 작은 값은 유효 관측으로 쓰지 않습니다. 유효한 finite range만 grid와 gap 판단에 사용합니다.

### 2. Rolling grid

최근 scan hit를 odom 좌표계에 memory로 저장합니다. 매 control tick마다 현재 odom pose 기준으로 memory point를 다시 `base_link` local grid에 재투영합니다.

Grid는 기본 반경 `2.0m`, 해상도 `0.02m`입니다. 현재 scan은 memory 재투영 뒤에 다시 반영되므로, 최신 free ray가 오래된 obstacle memory를 지울 수 있습니다.

### 3. Small gap blocking

전방 `gap_search_deg` 범위에서 gap 후보를 찾고, 양쪽 boundary range와 각도 차이로 코사인법칙 기반 width를 계산합니다. `min_passable_gap_m`보다 작은 gap은 그 방향 sector를 `BLOCKED`로 채웁니다.

이 패키지에서 small gap은 "낮은 점수 후보"가 아니라 "장애물과 동일한 hard block"입니다.

### 4. Vector field

Grid와 obstacle repulsion은 360도를 그대로 사용합니다. 다만 lowest-cost 방향 후보는 `low_cost_search_half_angle_deg`로 제한할 수 있습니다. 기본 YAML은 로봇 전방 기준 `-90deg ~ +90deg`만 lowest-cost 후보로 봅니다.

각 후보 방향마다 로봇 폭만큼의 corridor를 훑으며 `FREE`, `UNKNOWN`, `BLOCKED` cell cost를 거리 가중으로 누적합니다.

가장 낮은 cost 방향 하나만 쓰지 않고, 비슷하게 좋은 방향들을 평균해서 `lowest_cost_vector`를 만듭니다. 그 뒤 가까운 BLOCKED cell에서 멀어지는 `obstacle_repulsion_vector`를 더합니다.

초기 heading bias는 시작 직후 뒤쪽으로 많이 가는 것을 막는 용도입니다. `heading_decay_distance_m`까지 odom 누적 주행거리가 늘어나면 `heading_forward_weight_initial`에서 `heading_forward_weight_min`까지 선형으로 줄어듭니다.

Reverse bias는 로봇이 최초 heading 반대 방향으로 계속 진행하려 할 때만 켜집니다. 반대 방향으로 이동한 거리에 비례해 원 heading 쪽 복귀 vector를 키웁니다.

### 5. Dead-end memory

Dead-end memory는 SLAM map이 아닙니다. `/map`을 만들지 않고 odom 좌표계에 "이 위치 근처에서 이 방향으로 가면 막혔다"는 실패 corridor만 저장합니다.

Emergency stop 또는 stuck recovery가 발생하면 현재 odom pose와 실패 방향을 저장합니다. 이후 local grid를 만들 때, 그 실패 지점에서 뒤쪽으로 `dead_end_backtrack_distance_m`만큼 이어지는 corridor를 `dead_end_memory_cell_cost`로 칠합니다. 기본 cost는 `1.5`라서 `UNKNOWN=1`과 `BLOCKED=2` 사이입니다.

이 방식은 방 전체를 막지 않습니다. 방 중앙에 점 하나를 hard obstacle로 두는 대신, 실패한 branch 방향만 약하게 비싸게 만들어 같은 막다른 길을 다시 고르는 확률을 낮춥니다.

### 6. Control

최종 vector 각도:

```text
final_vector = lowest_cost + repulsion + heading_bias + reverse_bias
target_angle = atan2(final.y, final.x)
```

`angular_z`는 `steering_kp * target_angle`을 `max_angular_speed`로 제한합니다. `linear_x`는 전방 clearance, target angle, vector 크기를 기준으로 줄입니다. 전방이 `front_stop_distance_m`보다 가까우면 전진을 멈춥니다.

## Parameters

### Speed / Steering

- `max_linear_speed`: 정상 전진 최고 속도
- `min_linear_speed`: 주행 vector가 살아있을 때 유지할 최소 전진 속도
- `max_angular_speed`: 회전 속도 제한
- `steering_kp`: target angle을 angular velocity로 바꾸는 비례 gain
- `recovery_turn_speed`: emergency pivot에 쓰는 회전 속도
- `stuck_turn_speed`: stuck recovery turn에 쓰는 회전 속도

### Safety

- `emergency_stop_distance_m`: 이 거리보다 전방 obstacle이 가까우면 즉시 emergency 처리
- `front_stop_distance_m`: 일반 주행에서 전진 속도를 0으로 만드는 전방 거리
- `front_slow_distance_m`: 이 거리부터 전방 clearance 기반 감속 시작
- `front_window_deg`: 전방 거리 계산용 sector 반각

### Grid

- `grid_radius_m`: 로봇 중심 local grid 반경. 기본 `2.0m`
- `grid_resolution_m`: grid cell 크기. 기본 `0.02m`
- `grid_memory_seconds`: odom memory를 유지할 시간
- `grid_memory_max_points`: memory point 상한
- `obstacle_thickness_m`: lidar hit를 BLOCKED로 칠할 두께
- `robot_radius_m`: 로봇 반경
- `clearance_margin_m`: corridor 평가 때 로봇 반경에 더하는 안전 여유

### Direction Score

- `direction_sample_step_deg`: 360도 후보 방향 샘플 간격
- `low_cost_search_half_angle_deg`: lowest-cost 후보를 로봇 전방 기준 반각 안으로 제한. `90.0`이면 `-90deg ~ +90deg`, `180.0`이면 360도 전체
- `corridor_sample_step_m`: corridor 내부 샘플 간격
- `unknown_direction_penalty`: UNKNOWN cell 비용
- `blocked_direction_penalty`: BLOCKED cell 비용
- `direction_score_tolerance_ratio`, `direction_score_tolerance_abs`: 최저 cost와 비슷한 방향을 평균에 포함하는 허용 범위
- `lowest_cost_weight`: lowest cost vector 크기
- `repulsion_range_m`: obstacle repulsion이 작동하는 거리
- `repulsion_weight`: obstacle repulsion 최대 크기

### Heading / Reverse Bias

- `heading_forward_weight_initial`: 시작 heading 방향 초기 bias
- `heading_forward_weight_min`: decay가 끝난 뒤에도 남길 heading bias 최소값
- `heading_decay_distance_m`: 이 odom 누적 주행거리에서 heading bias가 최소값에 도달함. 기본 `4.0m`
- `reverse_heading_threshold_deg`: 이 각도 이상 반대 방향을 보려 할 때 reverse bias 후보로 봄
- `reverse_bias_start_m`: 반대 방향 진행 후 이 거리까지는 복귀 bias를 거의 주지 않음
- `reverse_bias_full_m`: 이 거리에서 reverse bias가 최대치가 됨
- `reverse_bias_max_weight`: reverse bias 최대 크기

### Small Gap

- `min_passable_gap_m`: 이보다 작은 gap은 BLOCKED로 반영
- `gap_search_deg`: small gap 탐색 전방 반각
- `gap_sector_half_width_deg`: gap 중심 주변 clearance 확인 sector 반각
- `gap_boundary_search_deg`: gap 양쪽 boundary를 찾는 각도 범위
- `gap_boundary_obstacle_max_range_m`: boundary로 인정할 가까운 obstacle 최대 거리
- `gap_boundary_drop_m`: center range보다 boundary가 이만큼 가까워야 gap으로 봄
- `gap_min_center_distance_m`: center가 너무 가까우면 gap 판정하지 않음
- `gap_min_depth_gain_m`: center가 boundary보다 이만큼 깊어야 열린 틈으로 봄

### Recovery

- `stuck_recovery_enabled`: odom stuck 감지 사용 여부
- `stuck_command_linear_threshold`: 이보다 큰 전진 명령이 있는데 진행이 없으면 stuck watch 시작
- `stuck_min_progress_m`: stuck watch 중 최소 이동거리
- `stuck_hold_seconds`: 이 시간 동안 진행이 부족하면 stuck
- `stuck_backup_speed`, `stuck_backup_seconds`: 뒤가 clear할 때 stuck 후진
- `stuck_cooldown_seconds`: stuck recovery 반복 방지 시간
- `emergency_backup_speed`, `emergency_backup_seconds`: emergency 후진
- `pivot_hold_seconds`: pivot 방향이 매 tick 좌우로 바뀌지 않도록 유지하는 시간

### Dead-end Memory

- `dead_end_memory_enabled`: dead-end memory 기본 활성화 여부. 기본값은 `false`
- `dead_end_memory_seconds`: 실패 corridor를 유지할 시간
- `dead_end_memory_max_entries`: 저장할 실패 corridor 최대 개수
- `dead_end_memory_cell_cost`: local grid에 얹는 virtual cost. 기본 `1.5`
- `dead_end_record_cooldown_seconds`: 같은 상황에서 memory가 과도하게 찍히지 않도록 하는 기록 간격
- `dead_end_merge_distance_m`: 가까운 실패 기록을 하나로 합칠 거리
- `dead_end_merge_angle_deg`: 방향이 비슷하면 같은 실패 branch로 볼 각도
- `dead_end_backtrack_distance_m`: 실패 지점에서 뒤쪽으로 corridor를 칠할 길이
- `dead_end_forward_extension_m`: 실패 방향 앞쪽으로 조금 더 칠할 길이
- `dead_end_corridor_half_width_m`: 실패 corridor 반폭

## Tuning Direction

- 벽에 가까이 붙으면 `repulsion_weight`를 올리거나 `repulsion_range_m`을 키웁니다.
- 넓은 구간에서도 너무 느리면 `front_slow_distance_m`을 줄이거나 `max_linear_speed`를 올립니다.
- heading 영향이 너무 빨리 사라지면 `heading_decay_distance_m`을 키우거나 `heading_forward_weight_min`을 올립니다.
- 출발 직후 heading을 너무 고집하면 `heading_forward_weight_initial`을 낮추거나 `heading_decay_distance_m`을 줄입니다.
- 방 안에서 출구보다 열린 실내 쪽을 오래 맴돌면 `unknown_direction_penalty`를 조금 낮추고, `repulsion_weight`가 과도하지 않은지 봅니다.
- 통과 가능한 gap을 막는다면 `min_passable_gap_m`을 낮추거나 `gap_boundary_drop_m`, `gap_min_depth_gain_m`을 완화합니다.
- 너무 얇은 틈을 가려 하면 `min_passable_gap_m`을 올리고 `gap_boundary_drop_m`, `gap_min_depth_gain_m`을 키웁니다.
- Y자에서 넓은 막다른 길을 반복해서 고르면 `set_dead_end_memory`를 켜고 `dead_end_memory_cell_cost`, `dead_end_backtrack_distance_m`을 조금씩 올립니다.
- Dead-end memory를 켠 뒤 탈출로까지 싫어하면 `dead_end_corridor_half_width_m`을 줄이거나 `dead_end_memory_cell_cost`를 낮춥니다.
