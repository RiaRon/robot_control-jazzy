# Pose Follow post-crossing 실물 검증 인계

이 문서는 개발 PC 검증 뒤 사용자가 OpenArm 컴퓨터에서 수행할 실물 시험 절차다.
Codex는 이 명령을 실행하지 않는다. 두 시나리오 모두 오른팔, 중력보상 scale 1.0,
기존 limiter와 Follow 경로를 그대로 사용한다.

## 목표 자세

실험 시작과 목표 B는 검증된 표준 A′ 자세이고, 목표 A는 검증된 legacy D 자세다.
둘 다 7관절 값이 모두 있으며 기존 joint limit 안, collision-free, Jacobian rank 6인
ready 자세다.

| 용도 | J1--J7 (rad) | TCP xyz (m, world) | TCP quaternion xyzw |
| --- | --- | --- | --- |
| 시작/B: A′ v2 | `0, 0.2, 0, 0.6, 0, 0, 0` | `0.223654884, -0.262153355, 0.161985610` | `0.950563961, 0.029502252, 0.294043891, 0.095372760` |
| A: legacy D v1 | `0.15, 0.55, 0.15, 0.8, -0.1, 0.15, 0.1` | `0.350641420, -0.419278006, 0.342782577` | `0.839572618, 0.105062881, 0.479262721, 0.233209892` |

수치는 현재 vendored bimanual OpenArm URDF를 FK한 참고값이다. A′→A의 TCP 변화는
약 0.2711 m와 0.5358 rad(30.70도)로 위치와 방향이 모두 달라진다. A→B는 같은
경로의 원점 방향 reversal이다. 실험 PC에서 RViz가 live robot description으로
표시하는 TCP marker가 기준이며, 위 수치와 크게 다르면 실행하지 않는다.

7관절 값은 RViz MotionPlanning의 Goal State에 넣어 TCP marker를 FK로 배치하는
용도로만 쓴다. `robotctl pose joints --values ... --execute`로 A나 B를 직접
명령하지 않는다. RViz의 **Plan** 또는 **Execute** 버튼도 누르지 않는다. 실제
이동 명령은 아래 `robotctl pose follow`만 발행한다.

## 배포와 시작 확인

OpenArm 컴퓨터에서 PR branch와 commit을 기록하고 빌드한다.

```bash
cd /home/user/robot_control-jazzy
git fetch origin
git switch feature/follow-post-crossing-hold
git pull --ff-only origin feature/follow-post-crossing-hold
git status --short --branch
git log -1 --oneline

source /opt/ros/jazzy/setup.bash
./ros_ws/build.sh
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"
alias robotctl='python3 -m robot_control.cli'
```

E-stop, 작업공간, 케이블, 오른팔 `can0`/왼팔 `can1` 매핑과 두 CAN FD link를 먼저
확인한다. 브링업과 effort controller는 별도 터미널에서 실행한다.

```bash
./ros_ws/pose_bringup.sh --real --right-can can0 --left-can can1
```

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/robot_control-jazzy/ros_ws/install/setup.bash
cd /home/user/robot_control-jazzy
./ros_ws/load_effort_controllers.sh right
```

명령 터미널에서 controller와 joint state를 확인하고 결과 폴더를 만든다.

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo --once /joint_states

RUN_DIR=/tmp/pose-follow-post-crossing
mkdir -p "$RUN_DIR"
git status --short --branch | tee "$RUN_DIR/git-status.txt"
git log -1 --oneline | tee "$RUN_DIR/git-head.txt"
robotctl pose show --group openarm_right_arm \
  --output "$RUN_DIR/before-all.json"
```

RViz MotionPlanning group은 `right_arm`으로 둔다. marker를 **Current**로 옮긴 뒤
다음 dry run이 ROS를 열거나 command를 발행하지 않는지 확인한다.

```bash
robotctl pose follow --group openarm_right_arm --seconds 60 --gravity 1.0
```

## 시나리오 1: A 추종 후 target 유지

먼저 표준 A′ 시작 자세를 dry run으로 검토한다. 실물 이동 승인을 받은 뒤에만 두
번째 명령을 실행한다.

```bash
robotctl pose ready --group openarm_right_arm
robotctl pose ready --group openarm_right_arm \
  --before-output "$RUN_DIR/scenario-1-ready-before.json" \
  --after-output "$RUN_DIR/scenario-1-ready-after.json" \
  --execute
```

RViz marker를 다시 **Current**로 맞춘 뒤 Follow를 시작한다.

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds 60 \
  --gravity 1.0 \
  --output "$RUN_DIR/scenario-1-a-hold.json" \
  --execute 2>&1 | tee "$RUN_DIR/scenario-1-a-hold.log"
```

`startup alignment complete` 뒤 RViz Goal State의 J1--J7을 목표 A의 7개 값으로
설정해 TCP marker를 이동한다. Plan/Execute는 누르지 않는다. 팔이 A를 추종한 뒤
marker를 바꾸지 않고 실행이 끝날 때까지 유지한다. 진동, limiter/refusal 출력과
E-stop 사용 여부를 현장 메모에 기록한다.

```bash
robotctl pose show --group openarm_right_arm \
  --output "$RUN_DIR/scenario-1-after.json"
```

## 시나리오 2: 같은 시작→A→B reversal

다시 A′로 준비하고 marker를 Current에 둔다. 실물 이동 승인을 받은 뒤에만
`--execute` 명령을 실행한다.

```bash
robotctl pose ready --group openarm_right_arm
robotctl pose ready --group openarm_right_arm \
  --before-output "$RUN_DIR/scenario-2-ready-before.json" \
  --after-output "$RUN_DIR/scenario-2-ready-after.json" \
  --execute

robotctl pose follow \
  --group openarm_right_arm \
  --seconds 120 \
  --gravity 1.0 \
  --output "$RUN_DIR/scenario-2-a-to-b.json" \
  --execute 2>&1 | tee "$RUN_DIR/scenario-2-a-to-b.log"
```

`startup alignment complete` 뒤 RViz Goal State를 목표 A의 7개 값으로 바꾼다.
팔이 A에서 멈추고 marker를 유지하는 구간을 관찰한 다음, Follow 프로세스를
종료하지 않은 채 Goal State를 목표 B=A′의 7개 값으로 바꾼다. 이 변경은 A로 향한
관절 방향의 reversal이며 새 accepted IK에서 hold가 풀리고 B 방향으로 다시
움직여야 한다.

```bash
robotctl pose show --group openarm_right_arm \
  --output "$RUN_DIR/scenario-2-after.json"
```

## 돌려보낼 파일과 판정

`$RUN_DIR`의 `git-status.txt`, `git-head.txt`, ready before/after JSON, 두 Follow
JSON과 log, 두 after pose JSON, 현장 메모를 개발 PC로 전달한다. 원시 JSON은 Git에
커밋하지 않는다.

JSON에서 다음을 확인한다.

- `outer_post_crossing_hold.command_crossing_events`의 crossing command가 같은
  trace sample의 `joint_positions_rad.next_command`와 일치한다.
- crossing 다음 같은 IK sample의 `used_ik_target_mask`와 `target_hold_mask`가
  해당 관절에서 true다.
- `measured_crossing_mask`가 false여도 command crossing과 hold 전이가 동작한다.
- 시나리오 1의 고정 IK 구간에서 raw candidate가 바깥을 향해도
  `pre_limiter_target_rad`는 IK target이다.
- 시나리오 2의 B accepted IK에서 `target_changed_release_events`가 발생하고 이후
  command가 reversal 방향으로 진행한다.
- 두 시나리오 모두 기존 Cartesian/joint limiter와 safety refusal 기록을 함께
  보며, fake hardware 결과를 진동 개선 근거로 사용하지 않는다.
