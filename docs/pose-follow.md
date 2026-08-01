# RViz에서 `robotctl pose follow` 사용하기

`robotctl pose follow`는 RViz의 TCP 인터랙티브 마커를 계속 읽어 실제 팔이
마커를 따라가게 하는 GUI 제어 모드입니다. 오른팔은
`openarm_right_hand_tcp`, 왼팔은 `openarm_left_hand_tcp`를 추종합니다.

이 문서는 ROS 2 Jazzy, 실물 OpenArm, `can0`/`can1` 두 버스를 기준으로
설명합니다. 실제 인터페이스 이름이 다르면 해당 이름으로 바꾸십시오.

## 기능과 제어 경로

RViz에서 마커를 드래그하면 `pose follow`가 100 Hz를 목표로 feedback을 읽고,
현재 관절 자세에서 가까운 해를 계산해 trajectory controller에 위치 명령을
스트리밍합니다. 이 모드는 마커의 **위치 변위만** 추종하고, TCP 방향은 시작
자세로 유지합니다.

- `pose follow`: 시작 시 현재 TCP를 영점으로 잡고 마커의 **변위**를 실시간 추종
- `pose ee --from-marker`: 마커의 `world` 절대 자세를 한 번 읽어 한 번 이동
- RViz의 **Plan & Execute**: 이 프로젝트의 운영자 제어 경로가 아님

모든 스트리밍 명령은 프로파일의 위치·속도·effort 한계와 command gate를
통과합니다. `--execute`가 없으면 실물에는 아무것도 발행하지 않습니다.

## 시작 전 안전 확인

실물에서 `--execute`를 붙이기 전에 다음을 확인하십시오.

1. E-stop을 즉시 누를 수 있는 위치에 둡니다.
2. 사람, 케이블, 공구를 팔의 작업 공간 밖으로 치웁니다.
3. CAN 좌우 배정이 맞는지 작은 움직임으로 먼저 확인합니다.
4. 첫 실행은 `--seconds 60`과 중력 보상 없이 시험합니다.
5. 마커를 팔의 현재 TCP 가까이에서 조금씩 움직입니다.
6. 무기한 모드를 실행한 채 자리를 비우지 않습니다.

## 1. 터미널 환경 준비

브링업 터미널과 명령 터미널 모두 작업 트리에서 ROS 환경을 준비합니다.

```bash
cd ~/rl_ws/robot_control
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"
alias robotctl='python3 -m robot_control.cli'
```

`src:.`을 기존 `PYTHONPATH` 앞에 추가하되 기존 값은 반드시 보존해야 합니다.
다음처럼 대입해 버리면 ROS 설정이 추가한 Python 경로가 사라질 수 있습니다.

```bash
# 사용하지 마십시오.
PYTHONPATH=src:.
```

`.venv`에 프로젝트가 설치되어 있다면 다음 방법도 사용할 수 있습니다.

```bash
source .venv/bin/activate
robotctl pose follow --help
```

다만 가상환경이 ROS의 `rclpy`를 찾지 못한다면 위의 모듈 실행 방식으로
돌아가십시오.

## 2. CAN FD와 실물 브링업

먼저 두 CAN 인터페이스를 확인합니다.

```bash
ip -br link show type can
```

두 인터페이스는 CAN FD, arbitration 1 Mbit/s, data 5 Mbit/s로 올라와야
합니다.

```bash
for iface in can0 can1; do
  sudo ip link set "$iface" down
  sudo ip link set "$iface" type can bitrate 1000000 dbitrate 5000000 fd on
  sudo ip link set "$iface" up
done

ip -details link show can0 | grep -E "state|fd on"
ip -details link show can1 | grep -E "state|fd on"

# RX/TX packet과 CAN 오류 상태를 함께 확인합니다.
ip -s -details link show can0
ip -s -details link show can1
```

두 인터페이스 모두 `UP`이고 `fd on`이어야 합니다. `BUS-OFF`이거나 error,
dropped counter가 계속 증가하면 브링업하지 말고 배선, 종단저항, 전원과
bitrate를 먼저 확인하십시오. 여기까지는 SocketCAN 설정 확인이며, 모터와 실제
통신되는지는 브링업 뒤의 `/joint_states` 확인으로 판정합니다.

첫 번째 터미널에서 실물 스택을 실행합니다.

```bash
cd ~/rl_ws/robot_control
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash

./ros_ws/pose_bringup.sh \
  --real \
  --right-can can0 \
  --left-can can1
```

브링업이 계속 실행 중인 상태에서 두 번째 터미널을 준비하고 controller와
관절 상태를 확인합니다.

```bash
cd ~/rl_ws/robot_control
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"
alias robotctl='python3 -m robot_control.cli'

ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo --once /joint_states
```

`joint_state_broadcaster`와 좌우 trajectory controller가 `active`여야 하고,
hardware interface가 `available` 또는 `claimed` 상태여야 합니다.
`/joint_states`에는 `openarm_right_joint1`부터 `openarm_right_joint7`,
`openarm_left_joint1`부터 `openarm_left_joint7`까지 보여야 합니다. timeout,
빈 joint 목록, `unconfigured` controller가 나오면 CAN 통신 성공으로 간주하지
마십시오.

명령 발행 없이 CLI까지 연결되는지 마지막으로 확인합니다.

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds 60
```

`DRY RUN: nothing is published`가 출력되면 profile, ROS adapter와
`/joint_states` 입력 경로까지 연결된 것입니다. marker feedback은 실제 추종을
시작한 뒤 RViz에서 마커를 드래그할 때 확인됩니다. `--execute`가 없으므로 팔은
움직이지 않습니다.

> **실물 동작 주의:** CAN 번호만으로 좌우를 검출할 수 없습니다. 오른팔
> 명령에 왼팔이 반응하면 즉시 중단한 뒤
> `--right-can can1 --left-can can0`으로 바꿔 다시 브링업하십시오.

## 3. RViz 설정

오른팔을 제어하려면 RViz에서 다음과 같이 설정합니다.

1. **MotionPlanning** 패널을 엽니다.
2. Planning Group을 `right_arm`으로 선택합니다.
   `openarm_right_arm`이 아닙니다.
3. RViz 툴바에서 **Interact**를 선택합니다.
4. `openarm_right_hand_tcp` 위의 축 화살표를 드래그합니다.

현재 `pose follow`는 위치 변위만 추종하며 회전 링은 추종하지 않습니다. 추종
중에는 시작 시점의 TCP 방향을 유지합니다. 회전까지 적용하려면 마커 자세를
정한 뒤 `robotctl pose ee --from-marker`로 한 번 이동하십시오.

왼팔은 Planning Group을 `left_arm`으로 선택합니다. MotionPlanning은 현재
선택한 그룹의 TCP 마커만 게시합니다.

RViz 고스트는 목표 운동학 결과이고 실제 모터값은 `/joint_states`입니다.
`pose follow`는 현재 실측 관절을 seed로 MoveIt IK를 계산하고, 성공한 목표
관절을 실제 모터가 수렴할 때까지 피드백 추종합니다.

## 4. 60초 시험 운전

두 번째 터미널에서 환경 준비를 다시 실행한 뒤 오른팔 시험 운전을 시작합니다.

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds 60 \
  --execute
```

> **이 명령은 실물을 움직입니다.**

다음 메시지가 나오면 RViz 마커를 천천히 드래그합니다.

```text
following openarm_right_hand_tcp at 100 Hz for 60 s, gravity off
drag the marker in RViz; the arm tracks it until the time runs out
```

시작 순간 RViz 마커가 실제 TCP와 다른 곳에 남아 있어도 그 위치로 바로
이동하지 않습니다. 첫 마커 좌표를 현재 TCP에 앵커링한 뒤, 그 지점에서 마커를
움직인 거리와 방향만큼 상대적으로 따라갑니다.

`--seconds`를 생략해도 기본값은 60초입니다. 생략한다고 무기한 실행되지는
않습니다.

60초가 지나거나 터미널에서 `Ctrl+C`를 누르면 추종이 끝납니다. trajectory
controller는 마지막 명령 위치를 유지하므로 팔은 마지막 위치를 계속 잡습니다.

## 5. 무기한 운전

60초 시험에서 좌우, 방향, 속도, 종료가 모두 정상임을 확인한 뒤에만 무기한
모드를 사용합니다.

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds inf \
  --execute
```

> **이 명령은 실물을 움직이며 자동 종료되지 않습니다.**

종료하려면 명령을 실행한 터미널에서 `Ctrl+C`를 누릅니다. `pose follow`가
정상 종료되면서 위치 스트리밍을 멈추고 마지막 자세를 유지합니다.

## 6. 중력 보상

`--gravity`는 목표 위치로 움직이는 명령이 아닙니다. URDF로 계산한 중력 보상
토크를 위치 제어에 추가해 팔의 처짐을 줄입니다. 따라서 gravity만 켰을 때
눈에 띄는 이동이 없을 수 있으며, 그것만으로 마커 추종이 시작되지는 않습니다.

중력 보상용 effort controller는 기본 브링업에 포함되지 않습니다. 브링업이
실행 중인 상태에서 별도 터미널을 준비한 뒤 먼저 로드합니다.

```bash
cd ~/rl_ws/robot_control
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash

./ros_ws/load_effort_controllers.sh right
```

양팔에 필요하면 `right` 대신 `both`를 사용합니다.

60초 시험이 성공한 뒤 중력 보상을 추종에 함께 적용합니다.

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds inf \
  --gravity 0.75 \
  --execute
```

> **이 명령은 위치 명령과 effort 명령을 함께 발행해 실물을 움직입니다.**

`0.75`는 모델 중력 토크의 75%라는 뜻이며 모든 로봇에 정확한 값은 아닙니다.
관절마다 튜닝된 값이 있다면 7개를 쉼표로 전달할 수도 있습니다.

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds inf \
  --gravity 1.1,1.15,1.1,1.1,1.1,1.1,1.1 \
  --execute
```

`Ctrl+C` 또는 시간 만료로 종료할 때 gravity effort는 0으로 돌아가고,
trajectory controller가 마지막 위치를 유지합니다.

## 7. 왼팔 제어

RViz Planning Group을 `left_arm`으로 바꾸고 다음 명령을 실행합니다.

```bash
robotctl pose follow \
  --group openarm_left_arm \
  --seconds 60 \
  --execute
```

> **이 명령은 실물 왼팔을 움직입니다.**

중력 보상도 사용할 경우 먼저 왼팔 effort controller를 로드합니다.

```bash
./ros_ws/load_effort_controllers.sh left
```

그런 다음 `--gravity`를 추가합니다.

## 8. 종료 보고서 읽기

종료되면 다음과 비슷한 보고서가 나옵니다.

```text
followed 4193 samples; the arm holds its last commanded pose
  actual control rate: 76.8 Hz; joint-state wait 12.9 ms/sample
  IK requests 91: 72 succeeded, 3 failed, 16 superseded
  tool centre point trailed the marker by 8.4 mm on average, 61.2 mm at worst
  last TCP position error: 1.7 mm
  last maximum joint error: 0.0041 rad
  velocity limit clamped on 271 of 4193 samples
```

| 출력 | 의미 | 대응 |
| --- | --- | --- |
| `trailed the marker by` | 실제 TCP와 마커 사이 평균·최대 거리 | 값이 크면 천천히 드래그하고 중력 보상을 검토 |
| `actual control rate` | 실물에서 달성한 루프 주파수와 joint-state 평균 대기시간 | 100 Hz 표시는 목표값이며 이 값이 실제값 |
| `IK requests` | MoveIt IK 요청의 성공·실패·최신 목표 대체 횟수 | 실패가 많으면 작업공간·특이점·충돌을 확인 |
| `last maximum joint error` | 마지막 IK 목표 대비 가장 큰 실측 관절 오차 | 값이 계속 크면 부하·장애물·중력 보상을 확인 |
| `velocity limit` | 마커가 프로파일 허용 속도보다 빠르게 움직임 | 정상적인 안전 제한이며 더 천천히 드래그 |
| `lead limit` | 실제 팔이 명령을 따라가지 못해 명령 선행량이 제한됨 | 장애물·부하를 확인하고 중력 보상을 검토 |
| `position limit` | 요청한 관절 자세가 프로파일 위치 한계를 넘음 | 마커를 작업 공간 안쪽으로 이동 |

clamp 횟수는 명령이 제한된 횟수입니다. 실제 추종 품질은
`trailed the marker by` 값을 기준으로 판단하십시오.

## 9. 문제 해결

### `robotctl: command not found`

현재 터미널에 실행 파일이나 alias가 없습니다. 작업 트리에서 다음을 다시
실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"
alias robotctl='python3 -m robot_control.cli'
```

### 오른팔 TCP 마커가 보이지 않음

MotionPlanning의 Planning Group을 `right_arm`으로 설정합니다.
CLI 그룹명 `openarm_right_arm`과 RViz Planning Group 이름은 다릅니다.

### 마커가 보이지만 드래그되지 않음

RViz 툴바에서 **Interact** 도구를 선택하고 TCP의 축 화살표를 잡습니다.
`pose follow`는 회전 링을 추종하지 않습니다. 로봇 링크 자체를 드래그하는
것도 아닙니다.

### 마커는 움직이지만 실제 팔이 움직이지 않음

다음 순서로 확인합니다.

1. 실행 명령에 `--execute`가 있는지 확인합니다.
2. 터미널에 `DRY RUN`이 출력되지 않았는지 확인합니다.
3. RViz 그룹과 CLI 그룹이 같은 팔인지 확인합니다.
4. 브링업 터미널에서 controller 또는 CAN 오류를 확인합니다.
5. 팔이 정확한 영점 자세라면 아래 특이점 회피를 수행합니다.

정확히 모든 관절이 0인 home 자세에서는 Jacobian rank가 부족해 MoveIt IK가
특정 방향의 해를 찾지 못할 수 있습니다. 종료 보고에서 IK 실패가 반복되면
팔꿈치를 조금 굽힌 뒤 추종을 다시 시작합니다.

```bash
robotctl pose joints \
  --group openarm_right_arm \
  --values 0,0,0,0.6,0,0,0 \
  --execute
```

> **이 명령은 실물 오른팔을 움직입니다.**

그 뒤 `pose follow`를 다시 시작합니다.

### `--gravity`를 추가하자 subscriber/controller 오류가 남

effort controller가 로드되지 않은 상태입니다. 브링업을 유지한 채 별도
터미널에서 다음을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
./ros_ws/load_effort_controllers.sh right
```

로드 상태는 다음 명령으로 확인할 수 있습니다.

```bash
ros2 control list_controllers | grep effort
```

### gravity를 실행했지만 팔이 이동하지 않음

gravity feedforward는 이동 목표가 아니라 현재 자세를 떠받치는 토크입니다.
GUI 이동에는 반드시 `pose follow ... --gravity ... --execute`를 실행한 상태에서
RViz TCP 마커를 드래그해야 합니다.

### 오른팔 명령에 왼팔이 움직임

즉시 추종을 `Ctrl+C`로 중단하고 필요하면 E-stop을 누릅니다. 브링업을 종료한
뒤 CAN 배정을 바꿉니다.

```bash
./ros_ws/pose_bringup.sh \
  --real \
  --right-can can1 \
  --left-can can0
```

소프트웨어는 두 버스가 뒤바뀐 것을 자동으로 검출하지 못합니다.

### 손가락 링크의 `unrealistic inertia` 오류

RViz가 inertia box를 그리지 않겠다는 시각화 경고입니다. TCP 마커 추종이
안 되는 직접 원인은 아닙니다.

## 명령 요약

```bash
# 오른팔: 먼저 60초 시험 — 실물 이동
robotctl pose follow --group openarm_right_arm --seconds 60 --execute

# 오른팔: 무기한 추종 — 실물 이동, Ctrl+C로 종료
robotctl pose follow --group openarm_right_arm --seconds inf --execute

# 오른팔: 중력 보상 controller 로드
./ros_ws/load_effort_controllers.sh right

# 오른팔: 무기한 추종 + 중력 보상 — 실물 이동
robotctl pose follow \
  --group openarm_right_arm \
  --seconds inf \
  --gravity 0.75 \
  --execute

# 왼팔: 60초 시험 — 실물 이동
robotctl pose follow --group openarm_left_arm --seconds 60 --execute
```

전체 옵션과 내부 동작은 [CLI 명령 레퍼런스](cli.md#robotctl-pose-follow)를
참조하십시오.
