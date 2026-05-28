# turtlebot_autonomous_gap_drive

`turtlebot_autonomous_gap_drive`는 "터틀봇 자율주행" 의미의 `turtlebot_autonomous`와, 메인 구동 방식인 라이다 기반 빈 공간 통과 주행 `gap_drive`를 합친 패키지입니다.

목표는 빠른 기록보다 완주입니다. 사진처럼 칸막이 벽, 좁은 틈, 장애물이 섞인 경로에서 SLAM/Nav2 없이도 라이다 `/scan`으로 천천히 안전한 방향을 고르는 reactive 주행을 기본 전략으로 합니다. 여기에 IMU 기울기 정지와 magnetometer 기준 역방향 복구를 안전장치로 더했습니다.

## 추천 방향

- 1순위: `gap_drive.launch.py`로 라이다 기반 저속 완주 주행
- 2순위: 시간이 남으면 `slam_gap_drive.launch.py`로 `slam_toolbox` 지도 생성만 함께 실행
- 비추천: 처음부터 Realsense + SLAM + Nav2 전체 구성

Jetson Nano에서 ROS2 Foxy를 맞춰야 한다면 이 패키지는 C++14와 Foxy 기본 API 위주로 작성되어 있습니다. 다만 Ubuntu 22.04의 공식 ROS2 조합은 Humble이므로, Foxy를 22.04에서 쓸 경우에는 소스 빌드나 컨테이너 환경이 필요할 수 있습니다.

## 동작 개요

노드는 `/scan`의 `sensor_msgs/msg/LaserScan`, `/imu`의 `sensor_msgs/msg/Imu`, `/mag`의 `sensor_msgs/msg/MagneticField`를 구독합니다. 전방 거리가 충분하면 가장 넓고 안전한 빈 공간을 향해 전진하고, 전방이 막히면 좌우 중 더 여유 있는 방향으로 제자리 회전합니다. 기본 속도는 TurtleBot3 Burger 최대 속도보다 낮은 `0.10 m/s`로 설정했습니다.

기본값은 `auto_start: false`라서 launch 직후에는 0속도를 publish하며 대기합니다. start 서비스가 호출되면 그 순간 magnetometer heading을 기준 방향으로 저장하고 주행을 시작합니다. 주행 중 기울기가 `tilt_stop_deg`를 넘으면 즉시 정지하고, 기준 heading의 반대 방향으로 일정 시간 이상 전진하면 제자리 복구 회전을 수행합니다.

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
ros2 topic echo /mag --once
ros2 topic info /cmd_vel
```

`/cmd_vel` 타입이 `geometry_msgs/msg/TwistStamped`로 떠야 하는 구동 환경이면 [config/slow_finish_first.yaml](config/slow_finish_first.yaml)의 `cmd_vel_stamped`를 `true`로 바꿉니다.

## 주요 튜닝값

튜닝 파일: [config/slow_finish_first.yaml](config/slow_finish_first.yaml)

- `max_linear_speed`: 최대 전진 속도입니다. 완주 우선이면 `0.08 ~ 0.12`를 추천합니다.
- `front_stop_distance_m`: 이 거리보다 전방이 가까우면 전진을 멈추고 회전합니다.
- `front_slow_distance_m`: 이 거리 안에서는 속도를 줄입니다.
- `safe_gap_distance_m`: 빈 공간 후보로 인정할 최소 거리입니다.
- `search_angle_deg`: 전방 기준 좌우 몇 도까지 빈 공간을 찾을지 정합니다.
- `heading_gain`: 선택한 빈 공간 쪽으로 회전하는 강도입니다.
- `centering_gain`: 좌우 벽 사이 중앙을 맞추는 강도입니다.
- `recovery_turn_speed`: 막혔을 때 제자리 회전 속도입니다.
- `tilt_stop_deg`: IMU roll/pitch 중 큰 값이 이 각도를 넘으면 정지합니다.
- `reverse_heading_threshold_deg`: 기준 heading에서 이 각도 이상 벗어난 채 전진하면 역방향 후보로 봅니다.
- `reverse_hold_seconds`: 역방향 후보 상태가 이 시간 이상 지속되면 복구 회전에 들어갑니다.
- `mag_yaw_sign`, `mag_yaw_offset_deg`: magnetometer 축 방향이 실제 로봇 기준과 다를 때 보정합니다.

## 실전 운용 팁

처음 테스트할 때는 로봇을 들어 올릴 수 있는 상태에서 `/cmd_vel` 방향이 맞는지 먼저 확인하세요. 전방 장애물에 가까워졌을 때 `BLOCKED_TURN` 또는 `EMERGENCY_TURN` 로그가 나오며 제자리 회전하면 정상입니다.

경로 폭이 좁아 로봇이 자주 멈추면 `front_stop_distance_m`을 `0.30` 정도로 낮추거나 `safe_gap_distance_m`을 `0.40`으로 낮춥니다. 반대로 충돌 가능성이 보이면 `front_stop_distance_m`을 `0.38 ~ 0.42`로 올리고 `max_linear_speed`를 `0.08`로 낮추세요.

## 센서 구성

현재 안전 주행 기본 설정은 라이다, IMU, magnetometer가 모두 필요합니다. RPLIDAR C1 또는 TurtleBot 기본 라이다 중 하나가 `/scan`을 publish하고, IMU는 `/imu`, magnetometer는 `/mag`를 publish해야 start 후 실제 주행으로 넘어갑니다.

Realsense D435i는 필수로 쓰지 않습니다. 깊이 카메라까지 넣으면 복잡도가 크게 올라가므로, 이번 과제의 완주 목표에는 라이다 reactive 주행을 먼저 안정화하는 쪽을 추천합니다.

노드를 종료할 때는 여러 번 0속도를 publish해서 바퀴를 멈추도록 했습니다. 다만 이것은 모터 torque-off가 아니라 `/cmd_vel=0` 정지 명령입니다. 실제 모터 전원/토크 해제는 TurtleBot3 OpenCR 또는 bringup 드라이버가 별도로 지원해야 합니다.
