# ChatGPT Work 인계 — OpenArm 오른팔 `pose follow`

작성일: 2026-08-18 (Asia/Seoul)

이 문서는 ChatGPT 계정의 `OpenArm 연구진행` Work가 OpenArm 컴퓨터의
배포·CAN·실물 검증을 이어가기 위한 현재 인계이다. Codex는 개발 PC에서 코드
수정, 테스트, ROS 빌드와 가짜 하드웨어 검증만 담당한다.

## 범위와 안전

- 현재 실물 연구 대상은 오른팔 `openarm_right_arm`만이다.
- 오른팔 CAN은 `can0`, 왼팔 CAN은 `can1`로 물리 확인됐다.
- 실물 움직임은 매 작업에서 사용자가 명시 승인한 뒤에만 수행한다.
- 오른팔 명령에는 항상 `--group openarm_right_arm`을 쓴다.
- effort controller와 중력 보상은 오른팔만 사용한다.
- 비상정지를 준비하고 작업 공간에서 사람·케이블·공구를 치운다.
- rosbag, HDF5, MCAP, JSON trace와 영상 같은 실험 데이터는 Git에 올리지 않는다.

## OpenArm 컴퓨터 환경

```text
호스트: user-NUC14SRK-B
저장소: /home/user/robot_control-jazzy
HDGP: /home/user/rl_ws/hdgp
ROS: ROS 2 Jazzy
```

각 터미널에서 다음 환경을 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/robot_control-jazzy/ros_ws/install/setup.bash
export PYTHONPATH="/home/user/robot_control-jazzy/src:/home/user/robot_control-jazzy:${PYTHONPATH:-}"
export HDGP_ROOT=/home/user/rl_ws/hdgp
```

## 확인 완료 상태

- CAN `can0`과 `can1`: `UP`, `ERROR-ACTIVE`, CAN FD 사용
- bitrate 1 Mbit/s, data bitrate 5 Mbit/s, tx/rx error counter 0
- 오른팔 실물 관절 상태, canonical 변환, FK와 자세 JSON 저장·파싱 정상
- 오른팔 `pose follow` 실제 제어주기 약 99.1 Hz
- 두 실물 시험의 MoveIt IK 실패 0회
- 중력 보상 기준값: 오른팔 전 관절 `1.0`

두 번째 30초 시험은 다음 기준선이다.

```text
TCP position error: mean 12.0 mm, worst 63.3 mm, last 1.4 mm
TCP orientation error: mean 1.1 deg, worst 3.6 deg, last 0.4 deg
Cartesian speed limit: 1025 / 2866 samples (35.8%)
last maximum joint error: 0.0063 rad
all joint velocity/lead/position clamps: 0
```

J4가 이동 중 가장 크게 뒤처졌다.

```text
r_aj_4: mean 0.0178 rad, worst 0.1921 rad, last 0.0063 rad
```

첫 시험의 J3/J5 약 0.98 rad 순간 오차는 두 번째 시험에서 재현되지 않았다.
현재는 결함으로 단정하지 않고 trace에서 IK 해와 명령 경계를 확인한다.

## Codex 분석과 변경

기존 `trailed the marker by` 값은 실시간 마커가 아니라 현재 채택된 IK 요청에
연결된 마커 스냅샷과 실측 TCP의 거리였다. 빠르게 드래그하면 IK 요청이 처리되는
동안 최신 마커와 차이가 날 수 있으므로 이동 중 전체 지연을 완전히 분리하지
못했다.

또한 외부 루프 기본값은 `kp=2.0 s^-1`이다. 20 mm/s로 일정하게 움직이는
목표를 이상적인 1차 추종기로 따라가도 `속도 / kp`에 해당하는 약 10 mm의
정상상태 지연이 생길 수 있다. 이는 현재 12 mm의 원인에 대한 코드 기반
추론이며, 실제 기여도는 새 로그로 확인해야 한다.

새 `pose follow --output <파일.json>`은 제어 한계나 명령을 바꾸지 않고 다음
계층을 100 Hz trace와 요약으로 기록한다.

1. `live_marker_to_measured`: 최신 마커에서 실측 TCP까지의 전체 오차
2. `marker_update_staleness`: 최신 마커와 채택된 IK 요청의 마커 차이
3. `accepted_marker_to_ik_target`: 최종 마커와 제한된 IK 중간목표 차이
4. `ik_target_to_command`: IK 목표와 상태 샘플 시점의 활성 명령 차이
5. `command_to_measured`: 활성 명령과 실측 TCP의 물리 추종 차이

관절별로도 `IK target → command`와 `command → measured`를 분리한다.
따라서 J4가 제어 계산에서 많이 요구된 것인지, 실제 모터가 보낸 명령을
뒤따르는 것인지 구분할 수 있다. `--output`은 측정값이 없는 dry run에서는
거부되고, 정상 종료된 실행만 원자적으로 저장한다.

## 다음 Work 절차

먼저 최신 `jazzy`를 받고 빌드한다. 미커밋 변경이 있으면 중단하고 검토한다.

```bash
cd /home/user/robot_control-jazzy
git status --short --branch
git remote -v
git fetch origin
git switch jazzy
git pull --ff-only origin jazzy
source /opt/ros/jazzy/setup.bash
./ros_ws/build.sh
```

실물 시험은 사용자의 해당 작업 승인을 받은 뒤에만 수행한다. 기존 기준선을
그대로 재현하고 새 로그만 추가한다. 첫 비교에서는 `kp`, `max_lead`,
`max-tcp-speed`를 바꾸지 않는다.

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --gravity 1.0 \
  --seconds 30 \
  --max-tcp-speed 0.02 \
  --max-tcp-angular-speed 0.10 \
  --output /tmp/right-follow-kp2.json \
  --execute
```

마커를 움직인 뒤 마지막 수 초 동안 고정해 최종 수렴도 함께 기록한다.
종료 후 JSON 문법만 먼저 확인한다.

```bash
python3 -m json.tool /tmp/right-follow-kp2.json >/dev/null
```

Work가 Codex로 돌려보낼 것은 다음과 같다.

- 터미널의 전체 종료 요약
- `/tmp/right-follow-kp2.json` 파일 자체 또는 별도 실험 저장소 경로
- 마커를 움직인 대략적인 구간과 고정한 구간
- E-stop 사용 여부, 비정상 소음·진동·발열·충돌 여부
- OpenArm 컴퓨터의 최종 Git 커밋

## 다음 조정 판단

새 기준선 한 번을 받은 뒤 한 항목만 바꾼다.

- `command_to_measured`와 J4의 `command → measured`가 크면 `kp`를 먼저
  올리지 않는다. 부하, 중력 보상, 모터 추종과 J4 상태를 확인한다.
- `ik_target_to_command`가 지배적이고 Cartesian speed limit가 적으면
  `kp=3.0`의 작은 비교 시험을 검토한다.
- Cartesian speed limit가 계속 지배적이면 `kp`를 올려도 제한 구간은 빨라지지
  않는다. 속도 한계 상향은 별도 안전 승인과 한 항목 비교가 필요하다.
- `marker_update_staleness`가 크면 IK 요청 처리율과 superseded 비율을 먼저
  분석한다.
- `accepted_marker_to_ik_target`가 크면 `--max-ik-step` 영향이 크다.
- 첫 시험 같은 J3/J5 순간 오차가 다시 나오면 해당 trace의 IK sequence,
  target, command와 measured 관절값으로 해 전환 여부를 확인한다.

현재 결론은 정적 도달 실패가 아니다. 오른팔은 마커 정지 후 1.4 mm까지
수렴했다. 다음 목표는 기존 안전 제한을 유지하면서 이동 중 지연의 원인을
분리한 뒤 가장 큰 한 계층만 조정하는 것이다.
## 최신 인계 — deterministic 진단 배치

이 절차는 위의 수동 기준선 수집 절차를 대체한다. 배포 대상은 이 기능이 병합된
최신 `jazzy@2b2ba14`이다. 이후 문서 커밋이 있으면 기능 기준은 `2b2ba14`이고
`docs/CURRENT_STATUS.md`에서 확인한다.

2026-08-18의 slow, fast, rotation 실물 JSON을 비교한 결과:

- 평균 live-marker 오차는 slow 6.6 mm, fast 13.6 mm, rotation 12.3 mm였다.
- fast의 live worst 154.9 mm는 이전 IK marker snapshot 기준 58.4 mm와 달리,
  최신 marker가 118.2 mm 앞선 순간까지 포함한 값이다.
- signed projection상 평균의 주 양의 기여는 `command_to_measured`였고,
  `ik_target_to_command`는 상당 부분 반대 방향으로 상쇄됐다.
- J4 peak는 모터 추종만의 문제가 아니라 J1/J4/J7이 함께 바뀌는 IK target
  posture 불연속 정황이 강했다.
- rotation 로그에는 alignment 완료 문장이 있으나 첫 trace가 10.4초부터이고,
  79 요청 중 49개가 superseded됐으며 위치도 약 206 mm 움직였다. 정상
  rotation 기준선으로 사용하지 않는다.

새 JSON은 기존 schema v1과 필드를 유지하면서 startup 완료 시각, IK sequence별
request/complete/accepted 시각과 latency, signed projection, 관절 target jump
이벤트, deterministic profile phase를 추가한다. Jump는 진단만 하며 제어를
바꾸지 않는다.

### OpenArm 배포와 무동작 확인

```bash
cd /home/user/robot_control-jazzy
git status --short --branch
git fetch origin
git switch jazzy
git pull --ff-only origin jazzy
git log -1 --oneline

source /opt/ros/jazzy/setup.bash
./ros_ws/build.sh
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:${PYTHONPATH:-}"
export HDGP_ROOT=/home/user/rl_ws/hdgp
```

실물 브링업 후 RViz planning group을 `right_arm`으로 선택하고 marker를
Current로 맞춘다. 다음 두 명령은 `--execute`가 없으므로 profile을 출력할
뿐 위치 명령을 보내지 않는다.

```bash
robotctl pose follow --group openarm_right_arm \
  --diagnostic-profile translation \
  --max-tcp-speed 0.02 --max-tcp-angular-speed 0.10

robotctl pose follow --group openarm_right_arm \
  --diagnostic-profile rotation \
  --max-tcp-speed 0.02 --max-tcp-angular-speed 0.10
```

### 최소 실물 배치

아래 명령은 실물을 움직인다. 당일 사용자의 명시 승인, E-stop, 빈 작업공간,
오른팔 `can0` 확인 후에만 실행한다. 각 실행에서 alignment 완료 문장을 본
뒤에는 marker를 만지지 않는다.

1. 10 mm world-x translation 왕복, 5 mm/s, 양 끝 3초 hold:

```bash
# --execute publishes commands and moves the real right arm.
robotctl pose follow --group openarm_right_arm \
  --gravity 1.0 --seconds 20 \
  --max-tcp-speed 0.02 --max-tcp-angular-speed 0.10 \
  --diagnostic-profile translation \
  --diagnostic-distance 0.01 \
  --diagnostic-linear-speed 0.005 \
  --diagnostic-hold-sec 3 \
  --output /tmp/right-follow-diagnostic-translation.json \
  --execute
```

2. startup TCP local-z 기준 5도 회전 왕복, 0.05 rad/s, 양 끝 3초 hold:

```bash
# --execute publishes commands and moves the real right arm.
robotctl pose follow --group openarm_right_arm \
  --gravity 1.0 --seconds 20 \
  --max-tcp-speed 0.02 --max-tcp-angular-speed 0.10 \
  --diagnostic-profile rotation \
  --diagnostic-angle 0.0872665 \
  --diagnostic-angular-speed 0.05 \
  --diagnostic-hold-sec 3 \
  --output /tmp/right-follow-diagnostic-rotation.json \
  --execute
```

비정상 소음·진동·충돌, 30 mm 초과 live 위치 오차, 10도 초과 live 방향 오차,
0.30 rad 초과 단일 관절 target jump가 보이면 즉시 중단하고 두 JSON과 전체
터미널 로그를 Codex로 전달한다. 이 두 clean 기준선을 확보하기 전에는 kp나
속도 한계를 비교하지 않는다.
