# Code Flow

이 문서는 [src/costmap_drive_node.cpp](src/costmap_drive_node.cpp)의 동작 흐름을 설명합니다. 이 노드는 기존 `gap_drive`처럼 `/scan`, `/imu`, `/odom`으로 직접 `/cmd_vel`을 만들지만, 핵심 판단은 LaserScan 한 프레임의 gap 후보가 아니라 짧게 누적한 local costmap과 후보 궤적 점수화로 수행합니다.

## 전체 구조

```mermaid
flowchart TD
    A["ROS2 node start"] --> B["Load YAML parameters"]
    B --> C["Create publishers / subscribers / service"]
    C --> D["Wait for /set_enabled"]
    D --> E{"Enabled?"}
    E -- "No" --> F["STOPPED: publish 0 velocity"]
    E -- "Yes" --> G{"IMU and odom current?"}
    G -- "No" --> H["WAIT_SENSORS: publish 0 velocity"]
    G -- "Yes" --> I{"Heading reference valid?"}
    I -- "No" --> J["Reset gyro yaw reference"]
    I -- "Yes" --> K["Use latest LaserScan"]
    J --> K
    K --> L{"Scan valid and fresh?"}
    L -- "No" --> M["WAIT_SCAN or SCAN_TIMEOUT"]
    L -- "Yes" --> N{"Tilt over limit?"}
    N -- "Yes" --> O["TILT_STOP: publish 0 velocity"]
    N -- "No" --> P["Integrate scan into obstacle memory"]
    P --> Q["Build rolling local costmap"]
    Q --> R["Find long-range open target"]
    R --> R2["Find width-valid narrow gap target"]
    R2 --> S["Score forward trajectory candidates"]
    S --> T["Apply reverse heading guard"]
    T --> U["Apply blocked/stuck recovery"]
    U --> V["Publish /cmd_vel, /local_costmap, /trajectory_markers"]
    F --> V
    H --> V
    M --> V
    O --> V
```

## 입력과 상태 저장

1. `scan_callback`

   `/scan`의 최신 `LaserScan`을 복사한 뒤 `Inf`, `NaN`, `0.0` 값을 같은 scan 안의 바로 이전 각도 유효 range로 대체해 저장합니다. 이전 유효 값이 없으면 `NaN`으로 남겨 이후 판단에서 무시합니다. 제어 timer에서는 `lost_scan_timeout_seconds`보다 오래된 scan을 사용하지 않습니다.

2. `imu_callback`

   `/imu` orientation quaternion으로 roll/pitch를 계산해 전복 정지에 사용합니다. `angular_velocity.z`는 gyro yaw로 적분하며, start 시점에 기준 yaw를 0도로 reset합니다.

3. `odom_callback`

   `/odom` pose에서 로봇의 `x`, `y`, `yaw`를 저장합니다. scan obstacle point를 odom frame에 짧게 누적하고, 전진 명령이 있는데 위치 변화가 거의 없을 때 stuck 복구를 판단합니다.

4. `set_enabled_callback`

   `/turtlebot_autonomous_costmap_drive/set_enabled` 서비스로 주행을 켜고 끕니다. stop 요청이 들어오면 obstacle memory와 recovery 상태를 비우고 0속도를 publish합니다.

## Costmap 생성 흐름

```mermaid
flowchart TD
    A["Latest scan + odom pose"] --> B["Ignore stale scan"]
    B --> C["Project valid scan hits into odom frame"]
    C --> D["Append hits to obstacle memory"]
    D --> E["Prune old points by obstacle_memory_seconds"]
    E --> F["Create base_link rolling grid"]
    F --> G["Transform remembered obstacle points back to base_link"]
    G --> H["Mark remembered occupied cells"]
    H --> I["Raytrace current scan as free space"]
    I --> J["Mark current scan hits again"]
    J --> K["Inflate by extra inflation_radius margin"]
    K --> L["Publish nav_msgs/OccupancyGrid"]
```

Costmap은 `base_link` 기준 rolling grid입니다. obstacle memory는 짧은 시간 동안 장애물 끝점과 벽 구조를 유지합니다. 이후 현재 scan ray가 free space를 다시 칠해 오래된 obstacle memory를 지우고, 현재 scan hit를 마지막에 다시 obstacle로 찍습니다. 그래서 작은 장애물 섬, 대각 벽, C자형 구조처럼 한 프레임 scan만으로 애매한 상황에서 더 안정적인 판단 근거를 가지면서도 LiDAR 튐값이 오래 남는 문제를 줄입니다.

## 후보 궤적 선택

```mermaid
flowchart TD
    A["Build costmap summary"] --> B["front / left / right range"]
    B --> B2["Find long-range target near start heading"]
    B2 --> B3["Find narrow gap target by boundary width"]
    B3 --> C{"Front inside emergency distance?"}
    C -- "Yes" --> D["EMERGENCY_TURN"]
    C -- "No" --> E["Sample angular_z candidates"]
    E --> E2{"Front close?"}
    E2 -- "Yes" --> E3["Add in-place pivot candidates"]
    E2 -- "No" --> F["Predict unicycle trajectory"]
    E3 --> F
    F --> G["Check circular footprint on costmap"]
    G -- "Collision or outside map" --> H["Reject candidate"]
    G -- "Safe" --> I["Score candidate"]
    I --> J["Keep best score"]
    H --> E
    J --> E
    J --> K{"Any safe trajectory?"}
    K -- "No" --> L["BLOCKED_TURN"]
    K -- "Yes" --> M["COSTMAP_DRIVE or NARROW_COSTMAP_DRIVE"]
```

각 후보는 `max_angular_speed` 범위 안에서 샘플링한 `angular_z`와 감속된 `linear_x`를 갖습니다. 일반 전진 후보는 `min_linear_speed`를 유지합니다. 단, 전방 거리가 `front_slow_distance_m` 안에서 충분히 가까워지면 선속도 0의 제자리 회전 후보를 추가로 평가해, stuck/block으로 판정되기 전에 먼저 몸을 틀 수 있게 합니다. 후보 궤적은 `trajectory_horizon_s` 동안 `trajectory_dt_s` 간격으로 예측하고, 로봇 반지름 원형 footprint가 lethal 또는 높은 inflation cost를 밟으면 reject합니다. 또한 관측된 gap의 boundary 폭이 `narrow_gap_min_width_m`보다 작고 중심 ray가 충분히 깊으면 통과 불가능 gap sector로 기록하며, 그 sector를 향하는 후보 궤적도 hard reject합니다.

점수는 아래 항목을 합쳐 계산합니다.

- 앞으로 진행한 거리 보상
- obstacle/inflation cost 감점
- unknown cell 비율 감점
- 큰 회전량 감점
- 직전 명령과의 급격한 변화 감점
- 좌우로 크게 벗어나는 lateral drift 감점
- start heading 기준에서 크게 벗어나는 heading 감점. 단, 전방이 가까워질수록 front clearance ratio만큼 약해짐
- start heading에 가까우면서 멀리 열린 ray 방향과 후보 최종 heading이 가까울 때 주는 long-range clearance 보상. 단, 전방이 가까워질수록 front clearance ratio만큼 약해짐
- 코사인법칙으로 폭이 검증된 narrow gap 중심과 후보 최종 heading이 가까울 때 주는 gate 보상

Long-range target은 sanitize된 현재 scan에서 `long_range_clearance_search_deg` 범위 안을 훑어 찾습니다. 단일 ray만 믿지 않고 `long_range_clearance_window_deg` 반각 안의 최소 거리를 함께 보며, 후보 궤적의 최종 heading이 이 target에 가까우면 `long_range_clearance_weight`만큼 보상을 받습니다. 다만 전방 장애물이 가까우면 이 보상과 start heading 유지 감점이 front clearance ratio만큼 줄어 C/U자 구조에서 local escape가 우선됩니다. RViz에서는 초록색 `long_range_target` 선으로 표시됩니다.

Narrow gap target은 후보 중심 ray 좌우에서 가까운 boundary를 찾고, 두 boundary 사이 폭을 코사인법칙으로 계산합니다. 폭이 `narrow_gap_min_width_m ~ narrow_gap_max_width_m` 안에 있고 중심 ray가 좌우 boundary 중 더 먼 쪽보다 `narrow_gap_min_depth_gain_m` 이상 깊으면 gate로 인정합니다. 반대로 boundary 폭이 `narrow_gap_min_width_m`보다 작으면 passable target이 아니라 rejected gap sector가 되어 일반 trajectory도 그쪽으로 가지 못합니다. 같은 gate가 비슷한 각도에 계속 보이면 `narrow_gap_hold_seconds` 동안 hysteresis를 적용해 S자/방 구조에서 target이 매 프레임 튀는 것을 줄입니다.

## 주행 모드

- `STOPPED`: 서비스 start 전 또는 stop 요청 후 0속도입니다.
- `WAIT_SENSORS`: IMU 또는 odom이 없거나 timeout입니다.
- `WAIT_SCAN`: 아직 LiDAR scan을 받지 못했습니다.
- `SCAN_TIMEOUT`: 마지막 scan이 너무 오래되었습니다.
- `TILT_STOP`: IMU roll/pitch가 `tilt_stop_deg`를 넘었습니다.
- `COSTMAP_DRIVE`: local costmap에서 가장 낮은 cost의 전진 궤적을 따라갑니다.
- `NARROW_COSTMAP_DRIVE`: 양쪽 벽이 가깝지만 inflation 사이 중앙 corridor가 통과 가능해서 저속으로 진행합니다.
- `NARROW_GAP_COSTMAP_DRIVE`: 폭이 검증된 narrow gap target이 있어 해당 방향 후보에 보상을 주고 저속으로 진행합니다.
- `BLOCKED_TURN`: 안전한 전진 궤적이 없어 좌우 중 더 여유 있는 방향으로 제자리 회전합니다.
- `EMERGENCY_TURN`: 전방 장애물이 매우 가까워 즉시 전진을 멈추고 회전합니다.
- `REVERSE_WARN`: 기준 heading 반대 방향으로 전진 중이지만 아직 hold 시간 전입니다.
- `REVERSE_RECOVERY`: 기준 heading 쪽으로 제자리 회전합니다.
- `STUCK_BACKUP`: 전진 명령이 있는데 odom상 진행이 없어 짧게 후진합니다.
- `STUCK_TURN`: 후진 후 더 여유 있는 방향으로 제자리 회전합니다.
- `BLOCKED_BACKUP`: `BLOCKED_TURN` 또는 `EMERGENCY_TURN`이 일정 시간 지속되어 LiDAR 시야 확보를 위해 짧게 후진합니다.
- `BLOCKED_ESCAPE_TURN`: blocked backup 후 더 여유 있는 방향으로 짧게 회전합니다.

## 덮어쓰기 우선순위

`compute_costmap_drive_output`이 만든 명령은 최종 명령이 아닙니다. 기존 `gap_drive`처럼 안전장치가 뒤에서 명령을 덮어씁니다.

1. Costmap 기본 로직이 `COSTMAP_DRIVE`, `NARROW_COSTMAP_DRIVE`, `BLOCKED_TURN`, `EMERGENCY_TURN` 중 하나를 만듭니다.
2. `apply_reverse_guard`가 필요하면 `REVERSE_RECOVERY`로 선속도 0, 회전 명령을 덮어씁니다.
3. `apply_stuck_recovery`가 막힘 회전 지속 또는 odom stuck을 감지하면 `BLOCKED_BACKUP`, `BLOCKED_ESCAPE_TURN`, `STUCK_BACKUP`, `STUCK_TURN`으로 다시 덮어씁니다. 후진은 local costmap에서 뒤쪽 짧은 경로가 clear할 때만 수행하고, 뒤가 장애물/unknown이면 바로 turn 단계로 넘어갑니다.

따라서 로그의 최종 mode는 마지막 안전장치 기준입니다. 예를 들어 costmap상 안전한 궤적이 있어도 odom이 움직이지 않으면 최종 mode는 `STUCK_BACKUP`이 됩니다.

## RViz와 로그 확인 포인트

- `/costmap_drive/local_costmap`: local occupancy grid입니다. 회색/검은 장애물과 inflation이 너무 커서 좁은 통로를 막으면 `inflation_radius_m` 또는 `robot_radius_m`을 낮춥니다.
- `/costmap_drive/trajectory_markers`: 파란색은 최종 선택된 궤적, 초록색은 long-range target, 주황색은 narrow gap target, 노란색은 계산된 gate 폭, 빨간색은 폭 부족으로 hard reject된 gap 폭입니다. 제자리 pivot 후보가 선택되면 파란 궤적은 거의 점처럼 보일 수 있습니다. 궤적이 벽 쪽으로 붙으면 `cost_obstacle_weight`, `cost_lateral_weight`, `cost_turn_weight`를 조정합니다. 초록색 target이 옆방으로 자주 튀면 `long_range_clearance_search_deg` 또는 `long_range_clearance_weight`를 낮춥니다. 주황색 target이 V자 코너에 자주 뜨면 `narrow_gap_min_depth_gain_m`을 올리고, 실제 문틈을 놓치면 `narrow_gap_min_width_m`, `narrow_gap_boundary_obstacle_max_range_m`, `narrow_gap_bonus_weight`를 확인합니다.
- 터미널 로그의 `traj=evaluated/rejected`: RViz marker가 아니라 `report_timer_callback`에서 찍는 숫자입니다. 전방이 가까울 때는 pivot 후보가 추가되어 `evaluated`가 `angular_sample_count`보다 커질 수 있습니다. `pivot=yes`는 최종 선택 후보가 선속도 0의 제자리 회전 후보라는 뜻입니다. 후보 대부분이 reject되면 costmap이 너무 보수적이거나 grid 범위가 너무 좁은 상태입니다.

즉 RViz에서는 map, 최종 선택 궤적, long-range target, narrow gap target을 보고, 후보 개수와 reject 개수는 노드 로그에서 확인합니다. 현재 코드는 모든 후보 궤적을 RViz로 그리지 않고, 최종 선택된 궤적과 target marker만 `/costmap_drive/trajectory_markers`로 발행합니다.
