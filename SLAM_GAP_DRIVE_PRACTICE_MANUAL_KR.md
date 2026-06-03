# TurtleBot3 + SLAMTEC LiDAR 시험 대비 매뉴얼

목표는 빠른 주행이 아니라, 낯선 후보 코스에서 최대한 많은 케이스를 안정적으로 완주하는 것이다. 실제 후보 맵이 있는 연습장 사용 기회가 한 번뿐이라면, 그 한 번은 파라미터를 즉석에서 맞추는 시간이 아니라 데이터 수집과 최종 검증 시간으로 써야 한다.

참고 문서:

- ROBOTIS TurtleBot3 SLAM Simulation: https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/
- ROBOTIS TurtleBot3 SLAM: https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/
- ROS 2 Foxy ros2 bag: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html
- Nav2 Map Saver: https://docs.nav2.org/configuration/packages/map_server/configuring-map-saver.html

## 1. 현재 전략 요약

현재 패키지의 `gap_drive_node`는 `/map`을 사용하지 않는다. 실제 주행 판단은 `/scan`, `/imu`, `/odom` 기반의 reactive gap driving이다.

따라서 SLAM과 RViz는 다음 용도로 쓴다.

- 라이다 방향, TF, odom이 정상인지 확인
- 실패 지점을 지도 위에서 관찰
- 실제 연습장 1회에서 맵과 rosbag을 확보
- 나중에 rosbag replay로 같은 센서 데이터를 반복 분석

시험 주행에서는 안정성이 제일 중요하다. Cartographer나 SLAM Toolbox는 주행 판단에 직접 필요하지 않으므로, 시험 직전 최종 주행에서는 가능하면 실행 노드를 줄이고 `gap_drive`만 단순하게 돌리는 쪽이 안전하다.

## 2. 실제 로봇 권장 실행 조합

`robot.launch.py`는 실제 TurtleBot3/OpenCR가 붙은 쪽에서 실행한다. 나머지 LiDAR, Cartographer, RViz, `gap_drive`는 Mac VM에서 자원 할당을 높게 잡고 실행해도 된다. 단, 모든 터미널은 같은 ROS 환경과 네트워크를 공유해야 한다.

확인할 것:

```bash
echo $ROS_DOMAIN_ID
echo $TURTLEBOT3_MODEL
```

권장 모델 설정:

```bash
export TURTLEBOT3_MODEL=burger
```

실행 순서:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

현재 SLAMTEC LiDAR 위치 기준 static TF:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.172 0 0 0 base_link laser
```

지도와 RViz가 필요할 때:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

자율주행 노드:

```bash
ros2 launch turtlebot_autonomous_gap_drive gap_drive.launch.py
```

주행 시작:

```bash
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
```

주행 정지:

```bash
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 3. Cartographer와 slam_gap_drive 사용 기준

`slam_gap_drive.launch.py`는 `slam_toolbox`와 `gap_drive_node`를 같이 실행하는 launch다.

Cartographer를 사용할 때는 `slam_gap_drive.launch.py`를 쓰지 않는다. Cartographer와 SLAM Toolbox는 둘 다 `/map`과 `map -> odom`을 만들 수 있어서 동시에 켜면 TF와 지도 토픽이 꼬일 수 있다.

사용 기준:

- Cartographer 사용: `cartographer.launch.py` + `gap_drive.launch.py`
- SLAM Toolbox 비교 테스트: `slam_gap_drive.launch.py`
- 실제 시험 주행 안정 우선: `gap_drive.launch.py`만 사용, 필요하면 RViz/Cartographer는 관찰용으로만 짧게 사용

추천:

```text
평소 분석/지도 확인: Cartographer + gap_drive
SLAM Toolbox 비교: slam_gap_drive
최종 완주 목표: gap_drive 단독 또는 Cartographer 관찰용 병행
```

## 4. RViz와 TF 체크리스트

RViz config는 화면 표시 설정만 저장한다. static TF publisher는 RViz에 저장되지 않는다. 매번 실행하거나 launch 파일에 넣어야 한다.

정상 TF 체인:

```text
map -> odom -> base_footprint -> base_link -> laser
```

확인 명령:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
```

라이다 확인:

```bash
ros2 topic echo /scan
```

확인 포인트:

```text
header.frame_id: laser
range_min: 정상 값
range_max: 16.0 근처
angle_increment: 0이 아니어야 함
```

odom 확인:

```bash
ros2 topic echo /odom
```

`x`, `y`는 보통 m 단위의 작은 실수여야 한다. `e+40` 같은 값이 나오면 비정상이고, RViz가 튕기거나 SLAM이 깨질 수 있다.

Cartographer RViz에서 `No map received`가 뜨면 RViz 설정 문제가 아니라 `/map` 메시지가 아직 안 온 것이다.

확인:

```bash
ros2 node list
ros2 topic info /map
```

기대 노드:

```text
/cartographer_node
/occupancy_grid_node
```

## 5. Gazebo 시뮬레이션으로 할 수 있는 것

ROBOTIS e-Manual의 SLAM Simulation 흐름은 다음과 같다.

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 run turtlebot3_teleop teleop_keyboard
```

여기에 `gap_drive`를 붙이면 teleop 대신 자율주행 명령을 테스트할 수 있다.

```bash
ros2 launch turtlebot_autonomous_gap_drive gap_drive.launch.py
```

주의할 점:

- Gazebo에서 `/scan`, `/odom`, `/tf`는 대체로 나온다.
- 현재 `gap_drive_node`는 `/imu`가 없으면 `waiting for imu`로 멈춘다.
- 따라서 먼저 Gazebo에서 `/imu`가 있는지 확인해야 한다.

```bash
ros2 topic list | grep imu
```

없으면 시뮬레이션 전용 개선이 필요하다.

추천 코드 개선:

```text
require_imu: false
require_odom_for_stuck: false
```

같은 파라미터를 추가해서 Gazebo 또는 rosbag replay에서 IMU 없이도 gap 판단을 테스트할 수 있게 만든다.

## 6. 맵과 주행 정보가 있으면 반복 시뮬레이션이 가능한가

가능하지만, 두 종류를 구분해야 한다.

### 6.1 rosbag replay 반복 실험

실제 연습장에서 기록한 `/scan`, `/imu`, `/odom`, `/tf`를 그대로 다시 흘려보낸다. 이 방식은 `gap_drive`의 판단 로그와 `/cmd_vel` 출력을 반복 비교하는 데 좋다.

장점:

- 실제 센서 데이터로 반복 실험 가능
- YAML 파라미터를 바꾸고 같은 입력에서 판단이 어떻게 달라지는지 비교 가능
- 실제 연습장 1회를 여러 번 되살릴 수 있음

한계:

- replay된 센서는 이미 기록된 데이터라서, 새 `/cmd_vel`이 로봇 위치를 바꾸지 않는다.
- 즉 "닫힌 루프 주행 시뮬레이션"은 아니다.
- 충돌 후 다른 위치로 갔을 때의 새 scan은 만들어지지 않는다.

결론:

```text
rosbag replay = 판단 알고리즘 반복 분석용
Gazebo world = 실제 움직임까지 포함한 반복 주행 실험용
```

### 6.2 Gazebo world 반복 주행

Gazebo에서 실제 후보 코스와 비슷한 벽/장애물 world를 만들면, YAML을 바꿔가며 진짜로 반복 주행시킬 수 있다.

장점:

- `/cmd_vel`에 따라 로봇이 실제로 움직임
- 코너, 막다른 길, 좁은 통로 등 실패 유형을 반복 생성 가능
- 완주율 중심 튜닝에 적합

한계:

- 저장된 `map.pgm`/`map.yaml`이 곧바로 Gazebo 물리 world가 되는 것은 아니다.
- 실제 맵을 Gazebo world로 쓰려면 벽/장애물 모델을 수동 또는 스크립트로 만들어야 한다.
- 라이다 높이, 충돌 모델, 마찰, 바퀴 특성이 실제와 다를 수 있다.

실전적으로는 시험 후보 맵을 그대로 복원하려 하지 말고, 실패 유형별 world를 여러 개 만드는 쪽이 좋다.

## 7. 만들어야 할 Gazebo 연습 코스

완주율을 높이려면 빠른 맵 하나보다, 실패 패턴 여러 개가 낫다.

우선순위:

1. 좁은 문틈
2. 대각선 벽 뒤 작은 gap
3. T자 갈림길
4. Y자 갈림길
5. 막다른 길 후 제자리 회전
6. 긴 복도 끝 90도 코너
7. 좌우 비대칭 장애물
8. 작은 장애물 섬
9. U자형 또는 C자형 함정
10. 좁은 양벽 구간

각 코스에서 기록할 값:

```text
완주 여부
걸린 위치
충돌 여부
최종 mode 로그
/cmd_vel angular.z 부호 변화
front/left/right/gap/width 값
```

## 8. rosbag 저장 방법

실제 연습장 1회에서는 반드시 기록한다.

저장 폴더 예시:

```bash
mkdir -p ~/tb3_bags
cd ~/tb3_bags
```

최소 기록:

```bash
ros2 bag record -o candidate_run_01 /scan /imu /odom /tf /tf_static /cmd_vel
```

Cartographer 지도까지 분석하려면 추가:

```bash
ros2 bag record -o candidate_run_01_full /scan /imu /odom /tf /tf_static /cmd_vel /map /map_updates
```

노드 로그도 같이 남기고 싶으면 launch 터미널 출력을 파일로 저장한다.

```bash
ros2 launch turtlebot_autonomous_gap_drive gap_drive.launch.py 2>&1 | tee ~/tb3_bags/gap_drive_run_01.log
```

기록 확인:

```bash
ros2 bag info ~/tb3_bags/candidate_run_01
```

## 9. rosbag 재생 방법

실제 로봇을 움직이지 않고 분석할 때는 bringup과 LiDAR driver를 끄고 replay한다. 기존 실제 토픽 publisher와 bag publisher가 동시에 같은 토픽을 내면 분석이 꼬인다.

기본 재생:

```bash
ros2 bag play ~/tb3_bags/candidate_run_01
```

느리게 재생:

```bash
ros2 bag play ~/tb3_bags/candidate_run_01 --rate 0.5
```

반복 재생:

```bash
ros2 bag play ~/tb3_bags/candidate_run_01 --loop
```

Foxy에서 옵션 지원이 환경마다 다를 수 있으니 도움말로 확인한다.

```bash
ros2 bag play -h
```

replay 중 다른 터미널에서 `gap_drive` 실행:

```bash
ros2 launch turtlebot_autonomous_gap_drive gap_drive.launch.py
```

주행 판단 시작:

```bash
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
```

이때 `/cmd_vel`을 새로 기록해서 파라미터별 결과를 비교한다.

```bash
ros2 bag record -o replay_tune_a /cmd_vel
```

주의:

- bag에 `/cmd_vel`이 들어있으면 replay가 예전 `/cmd_vel`도 publish한다.
- 새 `gap_drive`의 `/cmd_vel`과 섞이지 않게 하려면, 분석용 bag은 센서/TF만 기록하거나 replay 시 `/cmd_vel` 포함 여부를 주의한다.
- 더 깔끔한 비교를 위해 `cmd_vel_topic: /cmd_vel_test` 같은 파라미터 파일을 따로 만들어도 좋다.

## 10. 맵 저장 방법

Cartographer로 맵이 보이면 저장한다.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/candidate_map
```

결과:

```text
~/candidate_map.pgm
~/candidate_map.yaml
```

맵은 나중에 Nav2/AMCL 테스트나 코스 구조 분석에 쓸 수 있다.

## 11. YAML 튜닝 전략

목표는 "느리더라도 많은 코스 완주"다.

기본 방향:

```text
속도 낮게
회전 과격하지 않게
좁은 통로는 허용
막혔을 때 복구는 확실하게
센서 timeout은 보수적으로
```

실패별 조정:

좁은 gap을 못 들어감:

```text
safe_gap_distance_m 낮추기
front_stop_distance_m 낮추기
gap_window_deg 줄이기
narrow_passage_bonus 올리기
narrow_passage_min_sector_distance_m 낮추기
```

코너에서 벽을 침:

```text
max_linear_speed 낮추기
front_slow_distance_m 올리기
turn_slowdown_gain 올리기
front_stop_distance_m 올리기
```

좌우로 흔들림:

```text
heading_gain 낮추기
centering_gain 낮추기
max_angular_speed 낮추기
```

막다른 길에서 못 빠져나옴:

```text
recovery_turn_speed 조정
recovery_flip_seconds 낮추기
stuck_recovery_enabled 유지
stuck_hold_seconds 낮추기
```

한 방향으로만 돈다:

```text
/cmd_vel angular.z 확인
prefer_left_recovery 영향 확인
좌우 장애물을 확실히 다르게 놓고 테스트
LaserScan 방향이 RViz에서 실제와 맞는지 확인
```

추천 튜닝 방식:

1. 파라미터 하나만 바꾼다.
2. 같은 rosbag 또는 같은 Gazebo world에서 재실행한다.
3. 완주 여부와 실패 위치를 기록한다.
4. 더 안정적인 값만 남긴다.

## 12. 실제 연습장 1회 운영 계획

연습장 들어가기 전:

```text
정적 TF launch 또는 명령 준비
rosbag 저장 폴더 준비
RViz config 준비
gap_drive YAML 후보 2~3개 준비
배터리/네트워크/ROS_DOMAIN_ID 확인
```

연습장 첫 2분:

```text
/scan 확인
/odom 확인
TF 확인
RViz에서 LaserScan 방향 확인
```

기록 시작:

```bash
ros2 bag record -o candidate_run_01 /scan /imu /odom /tf /tf_static /cmd_vel /map /map_updates
```

가능하면 Cartographer로 맵 생성:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

주행:

```bash
ros2 launch turtlebot_autonomous_gap_drive gap_drive.launch.py
ros2 service call /turtlebot_autonomous_gap_drive/set_enabled std_srvs/srv/SetBool "{data: true}"
```

종료 후:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/candidate_map
ros2 bag info ~/tb3_bags/candidate_run_01
```

## 13. Nav2 활용 플랜

Nav2는 지금 당장 시험용 주력 전략이라기보다, 시간이 남을 때 확장하는 2단계 전략이다.

필요한 흐름:

```text
Cartographer 또는 SLAM Toolbox로 map 생성
map_saver_cli로 map 저장
AMCL로 현재 위치 추정
Nav2 bringup
목표점 또는 waypoint 전송
local costmap / global costmap 튜닝
```

장점:

- 저장된 맵을 기반으로 경로 계획 가능
- goal 기반 주행이 가능
- 반복 위치 이동에는 reactive gap drive보다 체계적

위험:

- 초기 pose 설정 실패 시 바로 틀어질 수 있음
- 좁은 시험 코스에서는 costmap inflation 때문에 통로를 막힌 곳으로 볼 수 있음
- 튜닝 시간이 많이 듦
- 실제 후보 맵 1회 상황에서는 검증 부담이 큼

추천 전략:

```text
1순위: gap_drive 완주율 개선
2순위: Cartographer/RViz/rosbag으로 진단 체계 구축
3순위: 저장된 map + AMCL + Nav2를 별도 실험으로 준비
시험 직전에는 가장 단순하고 검증된 조합 사용
```

Nav2를 쓰게 된다면 완주 목표에 맞춰 속도를 낮추고 inflation, obstacle layer, robot footprint를 보수적으로 조정해야 한다.

## 14. 다음 코드 개선 후보

가장 먼저 할 만한 것:

- static TF를 launch 파일에 포함
- 시뮬레이션용 `require_imu` 파라미터 추가
- replay 분석용 `cmd_vel_topic`을 `/cmd_vel_test`로 쉽게 바꾸는 YAML 추가
- gap_drive 로그를 CSV처럼 남기는 옵션 추가
- Gazebo용 launch와 params 파일 분리

추천 파일 구성:

```text
config/slow_finish_first.yaml          # 실제 로봇 안정 주행
config/sim_no_imu.yaml                 # Gazebo/rosbag replay용
config/replay_cmd_vel_test.yaml        # replay 분석용
launch/gap_drive.launch.py             # 기본
launch/gap_drive_with_laser_tf.launch.py # static TF 포함
```

최종 목표는 "빠른 1회 성공"이 아니라 "여러 낯선 코스에서 실패 확률을 줄이는 구성"이다.
