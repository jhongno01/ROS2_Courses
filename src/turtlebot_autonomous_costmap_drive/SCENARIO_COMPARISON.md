# Gap Drive vs Costmap Drive Scenario Comparison

이 문서는 기존 `turtlebot_autonomous_gap_drive`가 어려워했던 상황을 기준으로, 새 `turtlebot_autonomous_costmap_drive`가 같은 상황에서 어떤 판단 근거와 동작을 선택하는지 정의합니다.

## 1. 좁은 문틈

### Gap drive 동작

`gap_drive`는 후보 방향 주변 `gap_window_deg` 섹터의 최소 거리를 봅니다. 문틈 양쪽 모서리 중 하나가 후보 섹터 안에 들어오면 실제 중심은 비어 있어도 `safe_gap_distance_m` 미만으로 판단해 `BLOCKED_TURN`에 들어갈 수 있습니다. 이를 보완하려고 `NARROW_GAP_DRIVE`가 있지만, 폭 경계를 ray 하나하나로 찾기 때문에 대각 벽이나 노이즈에 민감합니다.

### Costmap drive 동작

`costmap_drive`는 장애물 cell을 inflate한 뒤 후보 궤적의 원형 footprint가 지나갈 수 있는지 검사합니다. 문틈 중심에 inflated corridor가 남아 있으면 `NARROW_COSTMAP_DRIVE`로 속도를 `narrow_corridor_speed_m_s` 근처까지 낮추고 중앙 궤적을 선택합니다. 문틈이 실제로 로봇 반지름과 inflation보다 좁으면 모든 전진 궤적이 reject되고 `BLOCKED_TURN`으로 바뀝니다.

## 2. 대각선 벽 뒤 작은 gap

### Gap drive 동작

대각선 벽 표면이 gap 뒤쪽에 보이면 섹터 최소거리 방식은 gap 내부에 장애물이 있는 것처럼 해석할 수 있습니다. 중심 ray는 깊어도 주변 ray 중 하나가 가까워 `GAP_DRIVE` 후보 점수가 낮아지고, 옆의 더 먼 공간을 따라가거나 멈춰서 회전할 수 있습니다.

### Costmap drive 동작

대각선 벽은 costmap에서 연속된 obstacle band로 표시되고, 문틈 opening은 free ray와 obstacle memory의 빈 corridor로 남습니다. 후보 궤적이 대각 벽과 충돌하지 않고 opening을 통과하면 `COSTMAP_DRIVE` 또는 `NARROW_COSTMAP_DRIVE`를 유지합니다. 궤적이 벽 끝을 긁는 경우에는 footprint collision으로 reject되어 더 완만한 arc를 선택합니다.

## 3. 긴 복도 끝 90도 코너

### Gap drive 동작

복도 끝에 가까워질 때 정면 거리가 `front_stop_distance_m` 아래로 내려가기 전까지 직진성이 강할 수 있습니다. stop 판단이 늦으면 코너 모서리에 가까이 붙고, 이후 `BLOCKED_TURN`에서 회전 반경이 커져 벽을 칠 수 있습니다.

### Costmap drive 동작

코너 모서리와 측면 벽이 inflation cost로 미리 보입니다. 직진 후보는 전방 obstacle cost가 커져 점수가 낮아지고, 코너 안쪽을 긁는 후보는 reject됩니다. 살아남은 완만한 회전 궤적을 선택하므로 코너 진입 전에 선속도가 줄고 `angular_z`가 먼저 생깁니다.

## 4. 좌우 비대칭 장애물

### Gap drive 동작

가장 열린 ray 또는 gap 중심이 장애물의 끝점 옆을 지나가면 로봇 footprint 전체가 아니라 ray 중심 기준으로 판단합니다. 그래서 중심은 통과 가능해 보여도 실제 바퀴나 몸체가 장애물 끝에 걸릴 수 있습니다.

### Costmap drive 동작

모든 후보 궤적은 원형 footprint 샘플을 costmap 위에 올려 검사합니다. 중심선만 비어 있고 측면이 inflated obstacle을 밟는 후보는 reject됩니다. 결과적으로 장애물 끝점에서 더 멀리 도는 arc가 선택되고, 필요하면 선속도가 줄어듭니다.

## 5. 작은 장애물 섬

### Gap drive 동작

한 프레임 scan에서 작은 장애물은 ray 몇 개로만 보입니다. gap 선택이 그 양옆의 먼 거리 ray에 끌리면 장애물 바로 옆으로 지나가고, scan 노이즈에 따라 좌우로 흔들릴 수 있습니다.

### Costmap drive 동작

작은 장애물도 obstacle memory에 잠시 남고 inflation이 적용됩니다. 장애물 주변에 원형 금지 영역이 생기므로 가까이 스치는 후보의 cost가 올라갑니다. 안정적으로 한쪽 우회 궤적이 선택되며, 직전 명령과의 변화량 감점 때문에 좌우 흔들림도 줄어듭니다.

## 6. 막다른 길

### Gap drive 동작

전방이 막히면 좌우 거리 비교로 회전 방향을 정합니다. 좌우가 비슷하면 `prefer_left_recovery`에 의존하고, 같은 방향으로 오래 돌면 `recovery_flip_seconds` 후 방향을 뒤집습니다.

### Costmap drive 동작

전방 후보 궤적이 모두 collision이면 `BLOCKED_TURN`으로 들어갑니다. 회전 방향 선택은 gap-drive와 비슷하게 좌우 여유를 보지만, 회전 후 새 scan과 obstacle memory가 rolling costmap에 반영되어 열린 쪽 전진 궤적이 생기는 순간 `COSTMAP_DRIVE`로 복귀합니다.

## 7. C자형 또는 U자형 함정

### Gap drive 동작

순간적으로 보이는 열린 ray를 따라 안쪽으로 더 들어갈 수 있습니다. 막힌 뒤에는 stuck 복구와 recovery turn에 의존합니다.

### Costmap drive 동작

local costmap만 쓰기 때문에 전역적으로 함정을 완벽히 피한다고 보장하지는 않습니다. 다만 obstacle memory 때문에 방금 지나온 벽과 막힌 전방이 함께 표현되어, 전진 후보가 빨리 reject되고 회전/후진 복구로 넘어가는 시점이 빨라집니다. 같은 구조가 반복되면 `obstacle_memory_seconds`, `trajectory_horizon_s`, `grid_forward_m`을 키우는 쪽이 도움이 됩니다.

## 8. 좁은 양벽 구간

### Gap drive 동작

`WALL_ASSIST`가 켜지면 가까운 벽을 따라가며 목표 거리를 유지합니다. 하지만 한쪽 벽 추종이 강하면 출구 중앙을 놓치거나 벽 쪽으로 붙을 수 있습니다.

### Costmap drive 동작

양쪽 벽이 모두 inflation band로 표시되고, 그 사이 중앙에 낮은 cost corridor가 남습니다. 좌우 중 한쪽 벽만 따라가는 대신 corridor 중앙을 통과하는 후보가 가장 낮은 cost를 얻습니다. `narrow_corridor_width_m` 조건이 맞으면 `NARROW_COSTMAP_DRIVE`로 속도 상한을 낮춥니다.

## 9. T자/Y자 갈림길

### Gap drive 동작

더 먼 ray나 forward bias, start heading 보너스의 조합으로 한쪽을 고릅니다. 구조를 기억하지 않으므로 scan 한 프레임에서 더 넓어 보이는 쪽으로 급격히 바뀔 수 있습니다.

### Costmap drive 동작

각 갈림길 후보가 실제 footprint로 몇 초 동안 안전한지 평가됩니다. 장애물에 가까운 급회전 후보는 cost가 높고, 직전 명령과 크게 다른 후보도 감점됩니다. 그래서 같은 갈림길 안에서 선택이 덜 튀며, 안전한 arc가 없다면 먼저 감속하거나 회전합니다.

## 튜닝 판단 기준

- 통과 가능한 좁은 문틈을 막힌 곳으로 보면 `robot_radius_m`, `inflation_radius_m`, `lethal_cost_threshold`를 순서대로 확인합니다.
- 벽을 너무 스치면 `inflation_radius_m`, `cost_obstacle_weight`, `front_slow_distance_m`을 올립니다.
- 열린 공간에서도 너무 느리면 `max_linear_speed`, `front_slow_distance_m`, `progress_reward_weight`를 조정합니다.
- 좌우로 흔들리면 `cost_smooth_weight`, `cost_turn_weight`, `angular_sample_count`를 조정합니다.
- C/U자 구조에서 너무 늦게 막힘을 깨달으면 `trajectory_horizon_s`, `grid_forward_m`, `obstacle_memory_seconds`를 조금 올립니다.
