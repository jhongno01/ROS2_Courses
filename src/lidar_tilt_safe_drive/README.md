# lidar_tilt_safe_drive

## 개요

`lidar_tilt_safe_drive` 패키지는 다음 세 가지 역할을 나눠서 제공합니다.

- `speed_input_publisher`: 터미널에서 `0.0 ~ 1.0` 속도를 입력받아 발행
- `angle_range_input_publisher`: 터미널에서 감시할 LiDAR 각도 두 개를 입력받아 발행
- `lidar_tilt_safe_drive`: LiDAR, IMU, 입력 토픽을 종합해서 `/cmd_vel`을 제어하고 1초마다 최소 거리 정보를 출력

기존 패키지에서 재사용한 코드

- `hw05_lidar_fb_motor`: `/cmd_vel` 퍼블리시 구조와 안전 정지 방식
- `hw02_lidar_subscriber`: 터미널 기반 각도 입력 퍼블리셔 패턴
- `hw04_read_lidar_with_imu`: IMU + LiDAR 동시 구독 구조

## Window 별 기능

1. 속도 입력 창
   - `speed_input_publisher`가 `0.0 ~ 1.0` 범위의 속도를 반복 입력받습니다.
2. LiDAR 감시 각도 입력 창
   - `angle_range_input_publisher`가 두 개 각도를 반복 입력받습니다.
   - 새 값을 넣으면 즉시 감시 구간이 바뀝니다.
3. 안전 주행 제어
   - IMU에서 계산한 `roll`, `pitch` 중 절댓값이 더 큰 값을 기울기 기준으로 사용합니다.
   - 기울기 60도 초과 시 즉시 정지합니다.
   - 입력한 각도 구간에서 최소 거리 물체가 20cm 이내면 정지합니다.
   - 그 외에는 가장 가까운 장애물의 반대 방향으로 `angular.z`를 줘서 회피하도록 했습니다.
   - 회전 속도는 요청 속도에 비례하도록 연동했고, `speed_to_turn_gain` 파라미터로 보정할 수 있게 했습니다.
4. 1초 주기 출력
   - `lidar_tilt_safe_drive` 노드가 1초마다 가장 가까운 물체의 각도와 거리를 출력합니다.

## 토픽과 메시지

### 입력 토픽

- `drive_speed`
  - 타입: `std_msgs/msg/Float32`
- `monitor_angle_range`
  - 타입: `more_interfaces/msg/LidarAngleRange`

```text
int32 start_deg
int32 end_deg
```

### 출력 토픽

- `/cmd_vel`
  - 타입: `geometry_msgs/msg/TwistStamped`
- `drive_safety_status`
  - 타입: `more_interfaces/msg/LidarSafetyStatus`

```text
int32 window_start_deg
int32 window_end_deg
int32 min_angle_deg
float32 min_distance_m
float32 tilt_deg
float32 requested_speed
float32 output_linear_x
float32 output_angular_z
bool imu_ready
bool scan_ready
bool window_measurement_valid
bool tilt_safe
bool stop_for_distance
```

## 각도 입력 규칙

- 입력 각도는 `0 ~ 359` 범위로 받습니다.
- 내부에서는 `[-180, 180)` 기준으로 정규화해서 사용합니다.
- 감시 구간은 `start_deg -> end_deg` 방향으로 해석하며, `start_deg`부터 + 방향 각도로 계산합니다.
  - 예: `330 40`이면 내부적으로 `-30도 -> 40도`의 구간을 감시합니다.
  - 예: `40 330`이면 내부적으로 `40도 -> 330도`처럼 `180도` 부근을 가로지르는 구간을 감시할 수 있습니다.

## 빌드

ROS2 워크스페이스 루트에서 실행합니다.

```bash
colcon build --packages-select more_interfaces lidar_tilt_safe_drive
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

### Terminal 1

안전 주행 노드 실행

```bash
source install/setup.bash
ros2 launch lidar_tilt_safe_drive lidar_tilt_safe_drive.launch.py
```

### Terminal 2

속도 입력 퍼블리셔 실행

```bash
source install/setup.bash
ros2 run lidar_tilt_safe_drive speed_input_publisher
```

예:

```text
Please type target speed (0.0 ~ 1.0): 0.5
```

### Terminal 3

LiDAR 각도 구간 입력 퍼블리셔 실행

```bash
source install/setup.bash
ros2 run lidar_tilt_safe_drive angle_range_input_publisher
```

예:

```text
Please type two lidar angles : 340 20
```

## 기본 파라미터

- `stop_distance_m`: `0.20`
- `slow_distance_m`: `0.60`
- `steer_distance_m`: `0.80`
- `tilt_stop_deg`: `60.0`
- `max_turn_rate`: `2.84`
- `speed_to_turn_gain`: `1.0`
- `control_period_ms`: `100`
- `report_period_ms`: `1000`

필요하면 launch 파일이나 `ros2 run` 명령에서 파라미터를 덮어쓸 수 있습니다.

## 참고

- IMU 또는 LiDAR 데이터가 아직 들어오지 않은 동안에는 안전하게 정지 상태를 유지합니다.
- 요청 속도가 `0.0`이면 회피 회전도 하지 않고 정지합니다.
- `drive_speed` 입력값은 코드 내부의 `max_linear_speed_ = 0.22 m/s`를 기준으로 실제 선속도 명령으로 변환됩니다.
- 회전 속도는 `requested_speed`, 장애물 근접도, 회피 방향을 함께 사용해서 계산하고 `speed_to_turn_gain`으로 추가 보정할 수 있습니다.
- 최종 회전 명령은 `max_angular_speed_ = 2.84 rad/s`를 넘지 않도록 제한합니다.
- LiDAR 거리값이 `0`, `inf`, `NaN`, 혹은 센서 유효 범위를 벗어나면 `99.9 m`로 처리합니다.
- 상태를 다른 노드에서 재사용하고 싶으면 `drive_safety_status` 토픽을 구독하면 됩니다.
