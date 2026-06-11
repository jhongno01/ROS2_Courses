# Flow

`turtlebot_autonomous_grid_drive`는 기존 `costmap_drive`처럼 여러 arc trajectory를 rollout해서 고르는 방식이 아닙니다. 매 tick마다 로봇 주변 360도 local grid를 만들고, 그 grid에서 vector field를 계산해 바로 `/cmd_vel`을 냅니다.

## Runtime Flow

```text
/scan
  -> sanitize inf/nan/0
  -> latest scan 저장

/odom
  -> 현재 pose 저장
  -> enabled 상태에서 누적 주행거리 갱신

/imu
  -> roll/pitch tilt 계산

control tick
  -> enabled/sensor/tilt/scan timeout 확인
  -> 최신 scan hit를 odom memory에 추가
  -> 오래된 memory point pruning
  -> odom memory를 현재 base_link local grid로 재투영
  -> 현재 scan free ray + hit obstacle 반영
  -> small gap hard block 반영
  -> dead-end memory cost overlay 반영
  -> 360도 direction score 계산
  -> lowest-cost vector 계산
  -> obstacle repulsion vector 계산
  -> heading bias 계산
  -> reverse return bias 계산
  -> final vector 합산
  -> emergency/stuck recovery override 확인
  -> /grid_drive/local_grid publish
  -> /grid_drive/vector_markers publish
  -> /cmd_vel publish
```

## Scan Sanitize

Lidar의 `inf`, `nan`, `0.0`은 모두 유효하지 않은 값으로 처리합니다. 이 값들은 free-space 판단이나 gap 판단에 쓰지 않습니다. 유효한 finite range만 grid raytrace와 obstacle hit에 사용합니다.

## Odom Memory

현재 scan에서 관측한 obstacle hit는 odom 좌표계 point로 저장됩니다. `grid_memory_seconds`가 지나거나 `grid_memory_max_points`를 넘으면 오래된 point부터 삭제합니다.

매 tick current odom pose를 기준으로 memory point를 `base_link` local 좌표로 다시 투영합니다. 그래서 로봇이 움직여도 최근 obstacle 기억이 로봇 주변 grid 안에 남습니다.

## Local Grid

Grid는 로봇 중심 원형 범위로 봅니다.

- 반경: `grid_radius_m`
- 해상도: `grid_resolution_m`
- 초기값: `UNKNOWN`

반영 순서:

1. memory obstacle을 `BLOCKED`로 칠함
2. 현재 scan의 hit 전 ray를 `FREE`로 칠함
3. 현재 scan hit를 `BLOCKED`로 칠함
4. small gap sector를 `BLOCKED`로 칠함
5. dead-end memory corridor를 `1.5` cost로 칠함

현재 scan이 memory 뒤에 적용되기 때문에 새 free ray가 오래된 obstacle memory를 덮어쓸 수 있습니다.

## Small Gap Hard Block

전방 `gap_search_deg` 안에서 center ray가 깊고 양쪽 boundary가 가까운 후보를 찾습니다. 양쪽 boundary 거리와 각도 차이를 이용해 코사인법칙으로 width를 계산합니다.

`width < min_passable_gap_m`이면 그 gap 방향 sector는 실제 obstacle처럼 `BLOCKED`로 채웁니다. 이 로직은 "후보 점수 감점"이 아니라 "grid 수정"입니다. 그래서 이후 lowest-cost 방향 계산도 작은 gap을 통과 가능 공간으로 보지 않습니다.

## Direction Score

Grid와 obstacle repulsion은 360도 전체를 사용합니다. 하지만 lowest-cost 방향 후보는 `low_cost_search_half_angle_deg` 안에서만 샘플링합니다. 기본 YAML의 `90.0`은 로봇 전방 기준 `-90deg ~ +90deg`만 lowest-cost 후보로 본다는 뜻입니다.

각 방향은 로봇 폭 corridor입니다.

각 corridor 내부 cell을 보며 비용을 누적합니다.

```text
FREE    -> 0
UNKNOWN -> unknown_direction_penalty
DEADEND -> dead_end_memory_cell_cost
BLOCKED -> blocked_direction_penalty
```

각 cell cost는 거리와 곱해서 누적합니다. score가 낮을수록 더 안전하고 열린 방향입니다. 가까운 `BLOCKED`가 `front_stop_distance_m` 안에 있으면 해당 방향은 driveable에서 제외합니다.

최저 score 방향 하나만 쓰면 흔들리기 쉬우므로, 최저 score와 비슷한 방향들을 평균해 `lowest_cost_vector`를 만듭니다.

## Dead-end Memory

Dead-end memory는 `/map`을 발행하는 SLAM이 아니라 odom 기준 실패 corridor 기억입니다.

기록 조건:

- `front_m <= emergency_stop_distance_m`
- 전진 명령이 있는데 odom 진행이 부족해 stuck recovery가 시작됨

기록 내용:

- 실패 지점의 odom `(x, y)`
- 실패 방향 yaw
- 기록 시각
- 같은 위치/방향에서 반복 실패한 hit count

Local grid에 반영할 때는 실패 지점 하나만 찍지 않습니다. 실패 yaw 기준으로 뒤쪽 `dead_end_backtrack_distance_m`, 앞쪽 `dead_end_forward_extension_m` 구간을 corridor로 칠합니다. cost는 기본 `1.5`라서 `UNKNOWN`보다 비싸고 `BLOCKED`보다 낮습니다.

이 corridor는 physical obstacle repulsion에는 쓰지 않습니다. 방향 score에서만 비싸게 보이므로, 필요한 경우 통과할 수 있고 방 중앙에 찍힌 한 점 때문에 탈출이 완전히 막히는 상황을 줄입니다.

런타임 토글:

```bash
ros2 service call /turtlebot_autonomous_grid_drive/set_dead_end_memory std_srvs/srv/SetBool "{data: true}"
ros2 service call /turtlebot_autonomous_grid_drive/set_dead_end_memory std_srvs/srv/SetBool "{data: false}"
```

`false`를 호출하면 dead-end memory가 비활성화되고 저장된 corridor도 지워집니다.

## Vector Field

최종 방향은 네 vector의 합입니다.

```text
final_vector =
  lowest_cost_vector
  + obstacle_repulsion_vector
  + heading_bias
  + reverse_bias
```

- `lowest_cost_vector`: grid에서 가장 비용이 낮은 방향
- `obstacle_repulsion_vector`: 가까운 BLOCKED cell에서 멀어지는 방향
- `heading_bias`: 시작 heading을 초반에만 약하게 유지
- `reverse_bias`: 최초 heading 반대 방향으로 오래 진행하려 할 때 원 heading 쪽으로 복귀

초기 heading bias는 코스 전체 목표가 아닙니다. 시작하자마자 뒤쪽의 넓은 공간으로 돌아가는 현상을 줄이는 보조항입니다. `heading_decay_distance_m`만큼 odom 누적 주행거리가 늘어나면 `heading_forward_weight_min`까지 줄어듭니다.

여기서 누적 주행거리는 시작점과 현재 위치의 직선거리가 아니라, odom pose 사이의 translation 변화량을 계속 더한 값입니다. 제자리 회전은 원칙적으로 거의 더해지지 않지만, odom translation 노이즈나 미끄러짐이 있으면 조금씩 누적될 수 있습니다.

## Control

```text
target_angle = atan2(final_vector.y, final_vector.x)
angular_z = clamp(steering_kp * target_angle, -max_angular_speed, max_angular_speed)
```

`linear_x`는 다음 조건으로 줄어듭니다.

- 전방 clearance가 `front_slow_distance_m` 안으로 들어옴
- target angle이 커져 회전이 더 필요함
- 전방 obstacle이 `front_stop_distance_m`보다 가까움

정상 주행 중 pivot은 선택하지 않습니다. drive vector가 없거나 recovery 상황일 때만 pivot을 씁니다.

## Recovery Override

정상 vector 계산 뒤 recovery 조건을 마지막에 확인합니다.

Emergency:

- `front_m <= emergency_stop_distance_m`
- 뒤 corridor가 `FREE`이면 짧게 후진
- 뒤가 `UNKNOWN/BLOCKED`이면 grid 기준 좌우 cost가 낮은 쪽으로 pivot

Stuck:

- 전진 명령이 있는데 odom 진행이 `stuck_min_progress_m`보다 작음
- 그 상태가 `stuck_hold_seconds` 이상 지속됨
- 뒤 corridor가 `FREE`이면 후진
- 뒤가 `UNKNOWN/BLOCKED`이면 recovery turn

Emergency backup과 stuck backup은 같은 `rear_corridor_is_clear()` 판단을 공유합니다.

## Debug Reading

로그 예시:

```text
[GRID_DRIVE] enabled=yes imu=yes odom=yes ref=yes tilt=1.2deg path=0.80m front=0.64m best=28deg score=0.42 blocked=12/180 rev_bias=no dead=yes/1 cmd=(0.10, 0.32) vector field steering
```

- `front`: 전방 sector 최단 거리
- `best`: lowest-cost vector 방향
- `score`: 최저 direction score
- `blocked`: driveable에서 제외된 방향 수 / 전체 샘플 수
- `rev_bias`: reverse return bias 활성 여부
- `dead`: dead-end memory 활성 여부 / 기록 개수
- `cmd`: 최종 `/cmd_vel`

RViz에서는 `/grid_drive/local_grid`와 `/grid_drive/vector_markers`를 같이 봅니다. 주황색 final vector가 로봇이 실제로 가려는 방향이고, 자홍색 repulsion이 벽에서 밀어내는 방향입니다.
