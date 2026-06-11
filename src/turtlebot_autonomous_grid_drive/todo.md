# TODO

내일 실제 주행에서 확인할 항목만 남긴다.

- Ubuntu/ROS2 환경에서 `colcon build --symlink-install --packages-select turtlebot_autonomous_grid_drive` 확인
- RViz에서 `/grid_drive/local_grid`, `/grid_drive/vector_markers` 표시 확인
- 직선 통로 50cm 전방 장애물에서 10cm gap은 BLOCKED, 26cm gap은 통과 후보로 남는지 확인
- ㄱ자 벽/대각선 벽 앞에서 emergency 거리까지 붙기 전에 repulsion으로 미리 비켜가는지 확인
- 큰 방+출구 구조에서 heading bias가 후반에 약해지고 lowest-cost/vector 중심으로 출구를 찾는지 확인
- emergency 상황에서 뒤 corridor가 FREE일 때만 짧게 후진하는지 확인
- stuck 상황에서 odom 진행 없음이 감지되고 backup 또는 recovery turn으로 넘어가는지 확인
- Y자 갈림길에서 넓은 막다른 길 진입 후 `set_dead_end_memory`를 켰을 때 같은 branch 재진입이 줄어드는지 확인
- Dead-end memory RViz 표시값 75가 방 전체가 아니라 실패 corridor 쪽에만 생기는지 확인
- 벽에 여전히 붙으면 `repulsion_weight`, `repulsion_range_m`, `front_slow_distance_m`부터 튜닝
- 너무 얇은 gap을 보려 하면 `min_passable_gap_m`, `gap_boundary_drop_m`, `gap_min_depth_gain_m` 조정
- Dead-end memory가 탈출까지 방해하면 `dead_end_corridor_half_width_m` 또는 `dead_end_memory_cell_cost`를 낮춰보기
