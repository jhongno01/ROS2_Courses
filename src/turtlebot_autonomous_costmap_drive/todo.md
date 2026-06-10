# TODO - costmap_drive next test

## 오늘 추가한 흐름

- LiDAR invalid 값 처리: `/scan` 콜백에서 `Inf`, `NaN`, `0.0`을 이전 유효 각도 range로 대체.
- Local costmap 기반 trajectory score에 `long_range_target` 보상 추가.
- `BLOCKED_TURN`/`EMERGENCY_TURN` 장기 지속 시 `BLOCKED_BACKUP -> BLOCKED_ESCAPE_TURN` 복구 추가.
- Gap-drive의 코사인법칙 gap width 측정을 costmap 쪽 `narrow_gap_target`으로 이식.
- RViz marker:
  - 파란색: 최종 선택 trajectory
  - 초록색: long-range target
  - 주황색: narrow gap target
  - 노란색: 계산된 gate width

## 내일 먼저 볼 것

- S자/방 구조에서 로그의 `gate=yes`가 실제 지나가야 할 틈에서 뜨는지 확인.
- `gate=yes*`가 너무 오래 붙으면 `narrow_gap_hold_seconds` 또는 `narrow_gap_hold_max_angle_error_deg` 낮추기.
- gate가 안 잡히면 `narrow_gap_boundary_obstacle_max_range_m` 올리거나 `narrow_gap_min_width_m` 낮추기.
- 넓은 방 입구를 gate로 오인하면 `narrow_gap_max_width_m` 낮추기.
- 좁은 틈에서 후보가 살아 있는데도 안 들어가면 `narrow_gap_bonus_weight` 올리고 `cost_lateral_weight`, `heading_alignment_weight`를 조금 낮춰보기.

## 현재 폭 기준

- 현재 `robot_radius_m: 0.105`, `narrow_gap_min_width_m: 0.23`.
- 이론상 로봇 지름은 약 210mm, gate 인정 시작은 230mm.
- 실제 안정 통과 폭은 LiDAR/grid 오차까지 고려해서 250~290mm 정도로 보고 테스트.
