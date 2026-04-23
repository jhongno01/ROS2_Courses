# hw01_topic_srvcli

## 한국어

### 개요

`hw01_topic_srvcli`는 ROS 2에서 토픽 통신과 서비스 통신을 함께 연습할 수 있도록 구성된 C++ 패키지입니다.

이 패키지는 3개의 노드로 구성됩니다.

- `hw01_pub`: 터미널에서 두 정수를 입력받아 토픽으로 발행합니다.
- `hw01_sub_cli`: 토픽 메시지를 구독하고, 로컬 덧셈 결과를 출력한 뒤 서비스 요청을 보냅니다.
- `hw01_srv`: 시작 시 정수 하나를 입력받아 저장하고, 서비스 요청이 오면 `a * b + 저장된 값`을 계산해 응답합니다.

### 패키지 구성

- Publisher node: `src/hw01_pub.cpp`
- Subscriber + Service Client node: `src/hw01_sub_cli.cpp`
- Service Server node: `src/hw01_srv.cpp`

### 사용 인터페이스

#### Topic message

토픽 메시지는 `more_interfaces/msg/TwoIntsIterationCount.msg`를 사용합니다.

```text
int64 a
int64 b
int64 count
```

- `a`: 퍼블리셔가 입력받은 첫 번째 정수
- `b`: 퍼블리셔가 입력받은 두 번째 정수
- `count`: 입력 반복 횟수. 첫 입력은 `1`, 다음 입력은 `2`처럼 증가합니다.

#### Service definition

서비스는 `more_interfaces/srv/TimesAddTwoInts.srv`를 사용합니다.

```text
int64 a
int64 b
---
int64 times_sum
```

- Request: 두 정수 `a`, `b`
- Response: 현재 구현에서는 `a * b + 서버 시작 시 입력한 값`

### 통신 흐름

1. `hw01_srv`를 실행하면 터미널에서 정수 하나를 입력받아 내부 변수로 저장합니다.
2. `hw01_sub_cli`는 토픽 `topic_pub_hw01`을 구독하고 서비스 `times_add_two_ints`를 기다립니다.
3. `hw01_pub`는 터미널에서 두 정수를 입력받아 `topic_pub_hw01` 토픽으로 발행합니다.
4. `hw01_sub_cli`는 수신한 메시지로 `iteration N: a + b = result` 형식의 로컬 계산 결과를 출력합니다.
5. 이어서 같은 `a`, `b`를 서비스 서버에 요청합니다.
6. `hw01_srv`는 `a * b + 저장된 값`을 계산해 터미널에 출력하고, 결과를 응답으로 반환합니다.
7. `hw01_sub_cli`는 서비스 응답을 받아 터미널에 출력합니다.

### 노드별 동작

#### `hw01_pub`

- 노드 이름: `hw01_pub`
- 발행 토픽: `topic_pub_hw01`
- 메시지 타입: `more_interfaces::msg::TwoIntsIterationCount`
- 주요 동작:
  - `scanf("%d %d", ...)`로 두 정수를 입력받음
  - `count`를 1씩 증가시켜 메시지에 포함
  - `Publishing: a b count` 형식으로 로그 출력
  - EOF 입력 전까지 반복 가능

#### `hw01_sub_cli`

- 노드 이름: `hw01_sub_cli`
- 구독 토픽: `topic_pub_hw01`
- 서비스 클라이언트 이름: `times_add_two_ints`
- 주요 동작:
  - 토픽 수신 시 `a + b`를 먼저 계산하여 출력
  - 같은 `a`, `b`를 서비스 서버에 요청
  - 서비스 응답을 받아 출력
  - 서비스 응답 대기와 토픽 콜백 처리를 위해 `MultiThreadedExecutor`를 사용

#### `hw01_srv`

- 노드 이름: `hw01_srv`
- 서비스 이름: `times_add_two_ints`
- 주요 동작:
  - 시작할 때 `scanf("%ld", ...)`로 정수 하나 입력
  - 이후 모든 요청에 대해 `a * b + 저장된 값` 계산
  - 계산식과 결과를 터미널에 출력
  - 결과를 서비스 응답으로 반환

### 빌드

워크스페이스 루트에서 실행합니다.

```bash
colcon build --packages-select more_interfaces hw01_topic_srvcli
source install/setup.bash
```

### 실행 순서

터미널 3개를 사용하는 것이 가장 간단합니다.

#### Terminal 1

```bash
source install/setup.bash
ros2 run hw01_topic_srvcli hw01_srv
```

실행 직후 서비스 서버 입력값 하나를 넣습니다.

예:

```text
Please type one int for service server: 10
```

#### Terminal 2

```bash
source install/setup.bash
ros2 run hw01_topic_srvcli hw01_sub_cli
```

#### Terminal 3

```bash
source install/setup.bash
ros2 run hw01_topic_srvcli hw01_pub
```

퍼블리셔에서 두 정수를 반복 입력합니다.

예:

```text
Please type two int: 3 4
Please type two int: 2 5
```

### 실행 예시

서비스 서버 초기 입력값이 `10`이고, 퍼블리셔에서 `3 4`를 입력한 경우:

#### `hw01_pub`

```text
Please type two int: 3 4
[INFO] [...] Publishing: 3 4 1
```

#### `hw01_sub_cli`

```text
[INFO] [...] iteration 1: 3 + 4 = 7
[INFO] [...] service response: 22
```

#### `hw01_srv`

```text
[INFO] [...] 3 * 4 + 10 = 22
```

### 의존성

- `rclcpp`
- `more_interfaces`
- `ament_cmake`

### 참고

- 퍼블리셔와 서브스크라이버가 연결되려면 토픽 이름이 정확히 같아야 합니다.
- 서비스 클라이언트와 서버가 연결되려면 서비스 이름이 정확히 같아야 합니다.
- `source install/setup.bash`를 하지 않으면 새로 빌드한 실행 파일과 인터페이스를 찾지 못할 수 있습니다.

---

## English

### Overview

`hw01_topic_srvcli` is a ROS 2 C++ package for studying topic communication and service communication together in a small, traceable example.

The package contains three nodes:

- `hw01_pub`: reads two integers from the terminal and publishes them as a topic message.
- `hw01_sub_cli`: subscribes to the topic, prints a local addition result, and then sends a service request.
- `hw01_srv`: reads one integer once at startup and uses it as an extra value in every service response.

### Package Layout

- Publisher node: `src/hw01_pub.cpp`
- Subscriber + Service Client node: `src/hw01_sub_cli.cpp`
- Service Server node: `src/hw01_srv.cpp`

### Interfaces

#### Topic message

The topic message type is `more_interfaces/msg/TwoIntsIterationCount.msg`.

```text
int64 a
int64 b
int64 count
```

- `a`: first integer entered in the publisher
- `b`: second integer entered in the publisher
- `count`: input iteration count, increasing as messages are published

#### Service definition

The service type is `more_interfaces/srv/TimesAddTwoInts.srv`.

```text
int64 a
int64 b
---
int64 times_sum
```

- Request: two integers `a`, `b`
- Response: in the current implementation, `a * b + startup_input`

### Communication Flow

1. `hw01_srv` starts and reads one integer from the terminal.
2. `hw01_sub_cli` subscribes to `topic_pub_hw01` and waits for the `times_add_two_ints` service.
3. `hw01_pub` reads two integers and publishes them to `topic_pub_hw01`.
4. `hw01_sub_cli` receives the message and prints `iteration N: a + b = result`.
5. `hw01_sub_cli` sends the same `a`, `b` values to the service server.
6. `hw01_srv` computes `a * b + startup_input`, prints the expression, and returns the result.
7. `hw01_sub_cli` prints the service response.

### Node Behavior

#### `hw01_pub`

- Node name: `hw01_pub`
- Publish topic: `topic_pub_hw01`
- Message type: `more_interfaces::msg::TwoIntsIterationCount`
- Behavior:
  - reads two integers using `scanf("%d %d", ...)`
  - increments `count` for each successful input
  - prints `Publishing: a b count`
  - repeats until EOF

#### `hw01_sub_cli`

- Node name: `hw01_sub_cli`
- Subscribe topic: `topic_pub_hw01`
- Service client name: `times_add_two_ints`
- Behavior:
  - computes and prints `a + b` when a topic message arrives
  - sends the same `a`, `b` to the service server
  - prints the service response
  - uses a `MultiThreadedExecutor` so the node can wait for a service response without blocking all callback handling

#### `hw01_srv`

- Node name: `hw01_srv`
- Service name: `times_add_two_ints`
- Behavior:
  - reads one integer once at startup using `scanf("%ld", ...)`
  - computes `a * b + stored_value` for each request
  - prints the expression and result
  - returns the result through the service response

### Build

Run the following commands from the workspace root:

```bash
colcon build --packages-select more_interfaces hw01_topic_srvcli
source install/setup.bash
```

### Run

Using three terminals is the simplest setup.

#### Terminal 1

```bash
source install/setup.bash
ros2 run hw01_topic_srvcli hw01_srv
```

Enter the server-side constant when prompted.

Example:

```text
Please type one int for service server: 10
```

#### Terminal 2

```bash
source install/setup.bash
ros2 run hw01_topic_srvcli hw01_sub_cli
```

#### Terminal 3

```bash
source install/setup.bash
ros2 run hw01_topic_srvcli hw01_pub
```

Enter two integers repeatedly in the publisher terminal.

Example:

```text
Please type two int: 3 4
Please type two int: 2 5
```

### Example Session

If the server startup input is `10` and the publisher input is `3 4`:

#### `hw01_pub`

```text
Please type two int: 3 4
[INFO] [...] Publishing: 3 4 1
```

#### `hw01_sub_cli`

```text
[INFO] [...] iteration 1: 3 + 4 = 7
[INFO] [...] service response: 22
```

#### `hw01_srv`

```text
[INFO] [...] 3 * 4 + 10 = 22
```

### Dependencies

- `ament_cmake`
- `rclcpp`
- `more_interfaces`

### Notes

- Topic names must match exactly for the publisher and subscriber to communicate.
- Service names must match exactly for the client and server to connect.
- If `install/setup.bash` is not sourced after building, ROS 2 may not find the new executables or interfaces.
