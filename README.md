# robot_control

This directory owns canonical lower-level robot contracts and the Real2Sim
artifact pipeline. `sim2real` remains responsible for policy execution and
task orchestration; `hdgp` remains responsible for robot assets and RL.

This Git branch targets Ubuntu 24.04 and ROS 2 Jazzy only. It contains
validated OpenArm and Tesollo Jazzy driver snapshots directly under
`ros_ws/src`. Humble is maintained on a separate long-lived branch; do not
merge the two distribution branches wholesale.

The first complete profile is `openarm_tesollo`. RH56F1 and the simple gripper
currently provide static component contracts only.

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -e '.[test,hdf5]'
robotctl r2s preflight
robotctl r2s collect --dry-run
```

Install the Jazzy dependencies and build the supported OpenArm/DG5F graph with:

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/install_dependencies_jazzy.sh
./ros_ws/build.sh
source ros_ws/install/setup.bash
```

`install_dependencies_jazzy.sh` deliberately prompts through `sudo`; run it
only when you are ready to grant the operator-controlled package changes. The
build wrapper rejects non-Jazzy environments, builds only the supported
OpenArm/DG5F leaves, and keeps all generated products inside
`ros_ws/{build,install,log}`.

No command is published unless `--execute` is explicit. Execution also needs
the ROS adapter, which requires `rclpy`; without it the CLI fails with a named
error rather than crashing on import.

Calibration JSON v1 is read-only compatibility input. All newly exported
bundles are schema v2, checksum protected, and tied to a profile and asset
manifest hash.

---

# 자세 설정 사용법

RViz에서 TCP 마커를 끌어다 놓고, 그 자세로 팔을 보내는 흐름입니다. 전체
명령 레퍼런스는 [docs/cli.md](docs/cli.md)에 있습니다.

## 0. 명령 준비

`robotctl`은 패키지를 설치해야 생기는 실행 파일입니다. 설치하지 않고 소스에서
바로 쓰는 편이 간단하고, 가상환경이 ROS의 `rclpy`를 가리는 문제도 없습니다.

```bash
cd ~/rl_ws/robot_control/.worktrees/jazzy
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"        # 대입(=)이 아니라 추가
alias robotctl='python3 -m robot_control.cli'
```

`PYTHONPATH`는 **반드시 추가**하십시오. `PYTHONPATH=src:.` 처럼 대입하면 방금
source한 ROS 경로가 지워져서, `rclpy`가 설치돼 있는데도 없다고 나옵니다.

굳이 진짜 `robotctl` 명령이 필요하면 가상환경이 시스템 패키지를 상속하도록
만들어야 합니다:

```bash
python3 -m venv --system-site-packages .venv
. .venv/bin/activate && pip install -e .
```

## 1. 가짜 하드웨어로 먼저

```bash
./ros_ws/pose_bringup.sh                     # CAN을 건드리지 않습니다
```

`pose_bringup.sh`는 벤더 기본값을 뒤집습니다. `demo.launch.py`는
`use_fake_hardware`가 false여서 인자를 빼면 실물 CAN을 엽니다. 이 래퍼는
반대로, `--real`과 버스 이름을 **명시하지 않으면** 가짜 하드웨어로 뜹니다.

## 2. 실물 연결

### 2-1. CAN 버스 올리기

오픈암 한 대는 팔당 버스 하나입니다. 듀얼 채널 어댑터 한 대면 `can0`, `can1`로
잡힙니다. 먼저 실제로 무엇이 있는지 확인하십시오:

```bash
ip -br link show type can
```

**반드시 CAN FD 모드**여야 합니다. `openarm_description`이 `can_fd:=true`로
렌더링하고 런치 인자로 바꿀 수 없어서, CAN 2.0으로 올리면 링크는 UP이 되지만
모터와 프레임을 한 개도 주고받지 못합니다.

```bash
for iface in can0 can1; do
  sudo ip link set "$iface" down
  sudo ip link set "$iface" type can bitrate 1000000 dbitrate 5000000 fd on
  sudo ip link set "$iface" up
done

ip -details link show can0 | grep -E "state|fd on"   # state UP + fd on 둘 다
```

배선은 오픈암 문서 기준 **빨강 → CANH, 검정 → CANL**, GND 선은 없습니다.
어댑터 쪽 GND 핀은 비워두면 됩니다.

벤더의 `openarm-can-configure-socketcan-4-arms` 스크립트는 `can0`~`can3`이 전부
있어야 실행되므로 로봇 한 대 구성에서는 쓸 수 없습니다.

### 2-2. 브링업

```bash
./ros_ws/pose_bringup.sh --real --right-can can0 --left-can can1
```

두 버스 다 이름을 줘야 하고, 래퍼는 추측하지 않습니다. **좌우를 바꿔 넣어도
소프트웨어는 알아채지 못합니다** — `openarm_hardware`가 두 팔을 같은 모터
ID(송신 `0x01`~`0x07`, 수신 `0x11`~`0x17`, 그리퍼 `0x08`/`0x18`)로 주소 지정해서,
어느 팔이 붙어 있든 그럴듯한 관절값이 올라옵니다. 오른팔에 명령했는데 왼팔이
움직이면 그때 알게 됩니다. 그러면 `--right-can can1 --left-can can0`으로 바꿔
다시 띄우십시오.

## 3. 자세 읽기

두 번째 터미널에서 0단계를 다시 실행한 뒤:

```bash
robotctl pose show --group openarm_right_arm
```

```text
openarm_right_arm: controller=right_joint_trajectory_controller planning_group=right_arm
openarm_right_arm: +0.0082 -0.0139 -0.0181 +0.0147 +0.0158 +0.0071 +0.0143
openarm_right_arm: openarm_right_hand_tcp xyz [+0.0134 -0.1435 +0.0822] rpy [-3.1204 -0.0372 +0.0018]
```

읽기는 발행이 아니므로 `--execute`가 필요 없습니다. 관절값이 정확히 0이 아니라
미세하게 떨리면 실물이고, 딱 `0.0000`이면 가짜 하드웨어입니다.

## 4. 마커를 끌어서 그 자세로 보내기

RViz **MotionPlanning** 패널에서 `openarm_right_hand_tcp`(공구 중심점) 위의
6축 마커를 원하는 곳으로 끕니다. 마커는 목표 상태만 움직이며, 명령을 실행하기
전까지 로봇은 따라가지 않습니다.

```bash
robotctl pose ee --group openarm_right_arm --from-marker              # 드라이런
robotctl pose ee --group openarm_right_arm --from-marker --execute    # 실제로 감
```

드라이런이 목표 좌표와 7개 관절 해를 전부 출력합니다. 보고 나서 `--execute`를
붙이십시오.

`--from-marker`는 RViz의 마커 서버에 `get_interactive_markers`로 물어서 이
그룹의 tip link에 해당하는 마커(`EE:goal_openarm_right_hand_tcp`)를 찾습니다.
마커의 `feedback` 토픽을 듣지 않는 이유는, feedback이 **드래그하는 동안에만**
나와서 명령이 마우스와 경주를 해야 하기 때문입니다. 서버는 드래그가 끝난 뒤에도
자세를 들고 있습니다.

마커는 `world` 기준 절대 자세라 `--from-marker`는 `--rpy`와 `--relative`를
거부합니다. 거기에 오프셋을 더하면 화면에 없던 곳으로 팔이 갑니다.

RViz는 패널에 선택된 플래닝 그룹의 마커만 내보냅니다. 다른 그룹을 요청하면
어느 마커가 없는지 알려줍니다:

```text
unavailable: RViz is running but holds no marker named
'EE:goal_openarm_right_hand_tcp'; set the MotionPlanning panel's planning group
to 'right_arm' so it publishes one
```

### RViz 자체의 Plan & Execute 버튼은 동작하지 않습니다

벤더 `joint_limits.yaml`이 17개 관절 전부 `has_acceleration_limits: false`인데,
Jazzy 기본 플래닝 파이프라인의 `AddTimeOptimalParameterization`이 가속도 한계를
요구합니다. 가짜 하드웨어·Gazebo·실물에서 똑같이 실패합니다.

고칠 수는 있지만 그러면 로봇으로 가는 **두 번째 경로**가 열립니다. RViz의
Execute는 `move_group` → `/execute_trajectory` → 컨트롤러로 바로 가서
`CommandGate`(프로파일 위치·속도 한계, 워치독, `--execute` 명시 요구)를 전부
건너뜁니다. 이 프로젝트는 운영자 명령도 학습 정책 명령과 같은 게이트를 지나는
것을 전제로 설계돼 있습니다. 그래서 RViz는 **자세를 고르는 도구**, `robotctl`은
**거기 도달하는 도구**로 나눠져 있습니다.

## 5. 오차가 남을 때 — `--settle`

`--execute`는 항상 도달 잔차를 출력합니다.

```text
EXECUTED: openarm_right_arm over 3 s
residual: 58.4 mm from the commanded pose
```

이건 IK 오차가 아닙니다. 계산된 관절 해의 정기구학은 목표 위에 정확히
떨어집니다(측정값 0.00 mm). 전부 **추종 오차**입니다.

원인은 제어 설정입니다. 컨트롤러가 위치만 보내고(`command_interfaces: [position]`),
`openarm_hardware`가 그것을 DM 모터의 MIT 명령으로 넘깁니다. 모터 토크는
`kp * (명령 − 실제)`이고 중력 피드포워드가 없으므로, **관절은 명령보다 뒤처져
있어야만** 중력을 버티는 토크를 냅니다. 정상상태 오차 ≈ 유지토크 / kp:

| 관절 | 오차 (rad) | kp |
|---|---:|---:|
| r_aj_1 (어깨) | 0.067 | 70 |
| r_aj_4 (팔꿈치) | 0.081 | 60 |
| r_aj_5 (손목) | 0.004 | 10 |

같은 해를 다시 보내도 소용없습니다. 똑같은 처짐이 그대로 재현됩니다. `--settle`은
매 회차의 실측 부족분을 명령에 **더해서**, 지난번에 못 간 만큼 목표를 지나쳐
가도록 명령합니다:

```bash
robotctl pose ee --group openarm_right_arm --from-marker --execute --settle
```

```text
settle 1: 58.4 -> 6.1 mm
settle 2: 6.1 -> 1.4 mm
settled: 1.4 mm after 2 corrections
```

`--tolerance`(기본 0.005 m)로 목표 잔차를 정합니다. 매 회차는 새로 만든 안전
게이트의 승인을 받으므로, 누적된 명령이 프로파일 위치 한계를 넘으면 거부됩니다.
최대 4회까지 시도하고, 한 회차가 잔차를 10% 이상 줄이지 못하면 중단합니다 —
스토퍼에 닿았거나 물체를 쥐고 있으면 수렴하지 않고, 더 밀어봐야 도달할 수 없는
자세로 명령만 감기기 때문입니다.

`--settle`은 `--execute` 없이 못 씁니다. 드라이런은 아무것도 보내지 않으므로
부족분이라는 게 존재하지 않습니다.

## 6. 처짐을 근본적으로 없애기 — 중력 보상

`--settle`은 도착점만 고칩니다. 움직이는 내내 정확하려면 처짐 자체를 없애야
하고, 그건 피드포워드 토크로 합니다.

### 왜 배율을 실험으로 정하는가

중력 토크는 URDF의 질량과 무게중심에서 계산하는데, 이게 맞서는 게인은
**벤더 하드웨어에 하드코딩**돼 있습니다:

```
control_gains.yaml:  kp = 70 70 70 60 10 10 10   ← 아무도 읽지 않음
DEFAULT_KP (헤더):   kp = 20 20 20 20  5  5  5   ← write()가 쓰는 값
```

`v10_simple_hardware.cpp`가 읽는 하드웨어 파라미터는 `can_interface`,
`arm_prefix`, `hand`, `can_fd` 넷뿐입니다. **`control_gains.yaml`을 고쳐도
아무 일도 일어나지 않습니다.**

그래서 올바른 배율은 계산으로 나오는 값이 아니라 **측정하는 값**입니다.

### 절차

effort 컨트롤러는 벤더 브링업에 없으니 따로 올립니다. 트래젝토리 컨트롤러가
위치를 계속 잡은 채로 effort만 추가되는 구조입니다:

```bash
./ros_ws/pose_bringup.sh --real --right-can can0 --left-can can1   # 터미널 1
./ros_ws/load_effort_controllers.sh                                # 터미널 2
```

먼저 **부하가 걸리는 자세**로 팔을 보내십시오. 팔을 접고 있으면 중력 토크가
작아서 배율 차이가 안 보입니다.

```bash
robotctl pose ee --group openarm_right_arm --from-marker --execute --settle
```

그 자세에서 배율을 훑습니다. 각 배율마다 유지하고, 트래젝토리 컨트롤러가
발행하는 `error.positions`(= 관절별 처짐, IK도 FK도 거치지 않은 값)를 읽어
표로 찍습니다:

```bash
# --execute는 배율마다 토크를 발행합니다. 보상이 바뀔 때마다 팔이 조금씩
# 움직입니다. E-stop 준비하십시오.
robotctl pose gravity --group openarm_right_arm --execute --sweep 0,0.25,0.5,0.75,1.0
```

```text
  scale      r_aj_1    r_aj_2    r_aj_3    r_aj_4    r_aj_5    r_aj_6    r_aj_7
   0.00     +0.0694   -0.0225   -0.0137   +0.1790   +0.0112   +0.0193   -0.0705
   ...
best measured scale: 0.75 (worst joint +0.0121 rad)
```

찾은 배율로 유지합니다:

```bash
# --execute가 토크를 발행합니다. 명령을 멈출 때까지 팔이 스스로 버팁니다.
robotctl pose gravity --group openarm_right_arm --scale 0.75 --execute
```

### 안전장치

- 배율 **1.5 초과는 거부**합니다. 과보상은 자세가 틀리는 게 아니라 버티던
  자리에서 팔을 밀어냅니다
- 토크가 프로파일의 관절별 `effort` 한계를 넘으면 **거부**합니다. 서보 스텝과
  달리 클램프하지 않습니다 — "허용된 만큼"으로 줄인 힘도 여전히 잘못된 크기의
  힘입니다
- 종료할 때 **반드시 0을 발행**합니다. 프로세스가 죽은 뒤 토크가 남아 있으면
  계속 밀게 됩니다

### 모델의 한계

제 중력 모델을 실측 처짐과 비교하면 어깨 1.39배, 팔꿈치 0.87배입니다. 부호와
순위는 전부 맞습니다. 남은 차이는 마찰·스틱션과 URDF 질량 오차이고, 그래서
배율이 조작자 손에 있습니다. 최적값은 1.0 근처지만 1.0은 아닐 것이며, 그건
위 sweep이 답할 문제입니다.

## 7. 마커를 실시간으로 추종 — `pose follow`

한 번에 한 자세씩 커밋하는 대신, 끄는 대로 따라옵니다.

```bash
# --execute가 위치 명령을 스트리밍합니다. 끄는 동안 팔이 움직입니다.
robotctl pose follow --group openarm_right_arm --execute --gravity 0.75
```

```text
following openarm_right_hand_tcp at 100 Hz for 60 s, gravity scale 0.75
drag the marker in RViz; the arm tracks it until the time runs out
followed 634 samples; the arm holds its last commanded pose
  tool centre point trailed the marker by 8.4 mm on average, 61.2 mm at worst
  velocity limit clamped on 74 of 634 samples
```

### `pose ee --from-marker`와 다른 점

`pose ee`는 마커를 **한 번** 읽고 트래젝토리 하나를 보냅니다. `pose follow`는
마커의 `feedback` 스트림(드래그 중 계속 나오는 토픽)을 읽어 컨트롤러 주기로
명령합니다.

역기구학이 `/compute_ik`가 아니라 **자코비안 기반 미분 IK**입니다. 100 Hz에
서비스 왕복은 못 따라가고, 자코비안 스텝은 국소적이라 팔이 가까운 해로
쓸려갑니다 — 매번 새로 IK를 풀면 해 분기 사이로 튈 수 있습니다. 역행렬은
감쇠 최소자승이라 특이점을 지나가도 스텝이 유한합니다.

모든 샘플이 `CommandGate.follow`를 지나며 **거부가 아니라 클램프**됩니다.
팔보다 빨리 끄는 건 정상 조작이므로, 프로파일 속도 한계로 제한하고 몇 개가
제한됐는지 끝에 보고합니다.

레이트 리밋의 기준이 **이전 명령**이라는 점이 팔이 움직이느냐를 결정합니다.
이 관절들은 버티는 토크를 내기 위해 명령보다 처짐만큼(실측 0.02~0.05 rad) 뒤에
있는데, 100 Hz에서 샘플당 예산은 0.02 rad입니다. 실측 기준으로 예산을 잡으면
명령이 실측보다 한 주기치만 앞설 수 있어 처짐보다 작고, 결과적으로 명령이
평형점 뒤에 놓여 아무것도 전진하지 않습니다. 가짜 하드웨어는 처짐이 없어 어느
쪽이든 완벽히 추종하므로, 이 문제는 실물에서만 드러났습니다.

그래서 세 번째 한계가 필요합니다. `max_lead`가 명령이 실측을 앞지르는 양을
묶습니다 — 없으면 장애물에 막힌 관절이 명령을, 따라서 토크를 무한히 감습니다.
클램프 보고에 `lead limit`으로 나옵니다.

### 멈출 때

`--seconds`가 끝나거나 Ctrl-C로 끝납니다. 어느 쪽이든 마지막에 명령을 멈추고
피드포워드 토크를 0으로 되돌립니다. 트래젝토리 컨트롤러가 마지막 명령 위치를
잡고 있고 그게 팔이 이미 있는 곳이므로, 힘이 빠지거나 어디로 튀지 않고 그
자리에 섭니다.

**스스로 끝나는 게 설계입니다.** 서보 루프를 켜둔 채로 두면, 몇 시간 뒤 누가
마커를 건드릴 때 움직이는 로봇이 됩니다.

### 전 관절 0 자세는 특이점입니다

정확히 `q = 0`에서 자코비안의 **z 행 전체가 0**이고 rank가 5입니다. 그
자세에서는 어떤 관절도 TCP를 수직으로 움직이지 못합니다. 마커를 위로 끌어도
움직이지 않는 게 **정상**입니다.

```
q = 0      특이값: [1.8674 1.5266 1.4142 0.286 0.2856 0.0000]   rank 5
팔꿈치 0.6 특이값: [1.8601 1.5187 1.4142 0.275 0.2743 0.0524]   rank 6
```

추종 전에 팔을 살짝 굽히십시오. 정확한 home만 아니면 됩니다:

```bash
# --execute가 트래젝토리를 보내 팔꿈치를 특이점에서 빼냅니다.
robotctl pose joints --group openarm_right_arm --values 0,0,0,0.6,0,0,0 --execute
```

`--gravity`는 여기서 특히 의미가 있습니다. `pose ee --settle`은 이동이 **끝나는
지점**을 고치는데, 움직이는 팔에는 끝나는 지점이 없습니다. 배율은 앞의
`pose gravity --sweep`으로 먼저 정하십시오.

## 그 밖의 명령

```bash
robotctl pose joints --group openarm_right_arm --named home --execute
robotctl pose joints --group openarm_right_arm --values 0,0,0,0.3,0,0,0 --execute
robotctl pose ee     --group openarm_right_arm --relative --xyz 0,0,0.03 --execute
```

`--named`는 SRDF의 group state를 읽습니다. `home`은 전 관절을 0으로 보내므로,
현재 자세를 모르는 상태에서 실행하면 큰 이동이 됩니다. `pose show`로 먼저
확인하십시오.

## 종료 코드

| 코드 | 뜻 |
|---|---|
| `0` | 정상 |
| `2` | **할 수 없음** — 알 수 없는 그룹, 인자 오류, ROS 미실행, 마커 없음 |
| `3` | **하지 않음** — IK 해 없음, 안전 게이트 거부 |

`3`은 시스템이 제대로 동작한 결과입니다.

## 문제 해결

| 증상 | 원인과 조치 |
|---|---|
| `unavailable: ... needs rclpy` | ROS 미source, 또는 `PYTHONPATH`를 대입해서 ROS 경로가 지워짐 |
| `unavailable: no /joint_states within 10.0 s` | 브링업이 안 떠 있음. `ros2 control list_controllers`로 확인 |
| `unavailable: /compute_ik is not available` | 컨트롤러는 떴는데 `move_group`이 안 뜸. 브링업 로그 확인 |
| `refused: no IK solution` | 도달 범위 밖이거나 해가 전부 충돌. 더 작은 `--relative` 단계로 |
| `refused: velocity limit exceeded` | `--duration`에 비해 이동이 큼. 프로파일이 아니라 `--duration`을 올리십시오 |
| 브링업은 뜨는데 관절값이 안 옴 | `candump can0`으로 프레임 확인. `state STOPPED`면 링크 미기동, `BUS-OFF`면 비트레이트/종단저항 |
| 다른 로봇 관절이 섞여 보임 | 이전 런치가 살아 있음. `pkill -f "[m]ove_group"` (첫 글자 대괄호는 pkill이 자기 자신을 죽이지 않게 함) |
