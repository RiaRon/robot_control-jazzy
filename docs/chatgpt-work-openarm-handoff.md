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
이벤트, deterministic profile phase를 추가한다. 이 문단의 `8a700c0`에서는
jump가 진단 전용이었지만, 아래 2026-08-20 최신 인계의 `0.30 rad` 하드 차단이
이를 대체한다.

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

## 2026-08-20 안전 중단 이후 최신 인계

이 절은 위의 최소 실물 배치를 대체한다. 2026-08-20 translation 최소 배치는
startup alignment로 보이는 소폭 움직임 직후 중단됐다. rotation은 실행하지
않았다. 당시 터미널에는 1133 samples, J3/J5 worst 약 `0.7646/0.7480 rad`,
J4 position clamp `516/1133`, live TCP worst `18.9 mm/3.9 deg`, IK accepted 7,
superseded 3이 출력됐으나 follow JSON과 전체 명령줄은 저장되지 않았다.

코드 조사와 제한 계산은
[안전 중단 조사](pose-follow-safety-incident-2026-08-20.md)에 기록했다. 핵심은
다음과 같다.

- `--execute` 없는 dry-run은 이제 ROS 연결 자체를 열지 않으며 startup
  alignment를 포함해 어떤 command도 publish할 수 없다.
- `--execute --output`은 output 저장 가능성을 ROS 연결 전에 검사한다.
- accepted IK target의 단일 관절 변화가 `0.30 rad` 이상이면 그 target을
  publish하기 전에 자동 거부한다.
- deterministic profile에서 position clamp가 발생하면 clamp된 command를
  publish하기 전에 lower/upper 방향과 관절을 출력하고 자동 거부한다.
- 수동 marker follow의 기존 clamp 정책은 바뀌지 않았다.
- 회수한 초기 pose의 J4는 `-0.0055313954 rad`로 URDF/profile lower `0 rad`보다
  `5.53 mrad` 낮았다. exact-pose replay는 인위적 IK offset 없이 startup lower
  clamp를 재현했다.
- fake MoveIt에서 같은 seed의 world-x 2 mm 목표를 50회 계산하자 39회가
  `0.30 rad` 이상 branch jump였고, 그중 terminal 크기에 가까운 해는 J3
  `-0.753756 rad`, J5 `+0.753498 rad`였다.
- 이 snapshot은 encoder/ROS state와 모델 한계의 불일치를 증명하지만 motor zero
  calibration 오차와 제어 settling을 구분하지는 못한다. 이 자료만으로 motor
  zero를 다시 쓰거나 URDF lower를 완화하지 않는다.

### 재시험 전 파일 경로 준비

아래 실물 절차는 이 변경의 Python 테스트, ROS 빌드, fake stack과 dry-run
무발행 검증이 모두 통과한 커밋을 배포한 뒤, 사용자가 그 작업에서 실물 이동을
명시 승인한 경우에만 수행한다. 먼저 빈 환경변수와 `tee` 실패를 제어 시작 전에
차단한다.

```bash
set -euo pipefail

export RUN_DIR=/home/user/openarm_follow_data/2026-08-20
: "${RUN_DIR:?RUN_DIR must be an explicit non-empty directory}"
mkdir -p -- "$RUN_DIR"
test -d "$RUN_DIR"
test -w "$RUN_DIR"

RUN_STEM=right-follow-diagnostic-translation
POSE_JSON="$RUN_DIR/right-pose-before.json"
FOLLOW_JSON="$RUN_DIR/$RUN_STEM.json"
FOLLOW_LOG="$RUN_DIR/$RUN_STEM.log"

# tee가 로봇 프로세스와 동시에 실패하지 않도록 로그 파일도 먼저 연다.
: >"$FOLLOW_LOG"

# 읽기 전용 초기 자세 기록. 실물 command를 보내지 않는다.
robotctl pose show --group openarm_right_arm --output "$POSE_JSON"
python3 -m json.tool "$POSE_JSON" >/dev/null
python3 - "$POSE_JSON" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    payload = json.load(stream)
group = next(
    item for item in payload["groups"]
    if item["name"] == "openarm_right_arm"
)
q4 = group["joint_positions_rad"][3]
print(f"pre-run J4: {q4:+.9f} rad (required: >= 0 rad)")
if q4 < 0.0:
    raise SystemExit("STOP: J4 is below the URDF/profile lower limit")
PY
```

원시 pose/follow JSON과 terminal log는 실험 데이터 저장소에 두고 Git에 추가하지
않는다. J4 검사에서 `STOP`이면 follow `--execute`로 넘어가지 않는다. 이 상태에서
영점 calibration이나 URDF limit 변경도 자동으로 수행하지 않고 별도 기구·엔코더
점검으로 넘긴다.

### 무동작 확인

다음 명령에는 `--execute`와 `--output`이 없다. 출력 마지막 줄이 정확히
`no ROS connection is opened`인지 확인하고, 팔이 움직이면 즉시 E-stop 후 실제
실행 argv, shell history와 실행 중인 `robot_control.cli` process를 보존한다.

```bash
robotctl pose follow --group openarm_right_arm \
  --gravity 1.0 --seconds 20 \
  --max-tcp-speed 0.02 --max-tcp-angular-speed 0.10 \
  --diagnostic-profile translation \
  --diagnostic-distance 0.01 \
  --diagnostic-linear-speed 0.005 \
  --diagnostic-hold-sec 3
```

### 승인 후 translation 한 번만 재시험

```bash
# 아래 명령만 --execute가 있으며 실물 오른팔을 움직인다.
robotctl pose follow --group openarm_right_arm \
  --gravity 1.0 --seconds 20 \
  --max-tcp-speed 0.02 --max-tcp-angular-speed 0.10 \
  --diagnostic-profile translation \
  --diagnostic-distance 0.01 \
  --diagnostic-linear-speed 0.005 \
  --diagnostic-hold-sec 3 \
  --output "$FOLLOW_JSON" \
  --execute 2>&1 | tee -a "$FOLLOW_LOG"
```

다음 중 하나면 즉시 `Ctrl+C`, 필요하면 E-stop하고 rotation으로 넘어가지 않는다.

- 비정상 소음·진동·발열·충돌 또는 예상 밖 방향의 움직임
- CAN error counter 증가, controller 비활성 또는 joint-state 중단
- `refused: IK target jump refused before publish` (단일 관절 `>= 0.30 rad`)
- 실행 전 pose의 J4가 `0 rad`보다 작음
- `refused: deterministic profile position clamp refused before publish`
  (`lower:` 또는 `upper:` 방향 포함)
- IK failed, superseded 또는 J3/J5의 새 target 불연속
- live TCP 위치 `> 30 mm` 또는 방향 `> 10 deg`
- JSON/로그 파일이 없거나 JSON 문법 검사가 실패함

정상 종료 뒤 `python3 -m json.tool "$FOLLOW_JSON" >/dev/null`을 실행하고 초기
pose JSON, follow JSON, 전체 log와 Git 커밋을 함께 전달한다. clean translation
한 번을 검토하기 전에는 rotation, kp 또는 속도 한계 변경을 실행하지 않는다.

## 2026-08-20 sequence-6 IK refusal 이후 개발 인계

이 절은 위 재시험 절차보다 최신이며, 이 개발 배치에서는 추가 실물 재시험이나
rotation 실행을 요구하지 않는다. `jazzy@0673903`에서 수행한 오른팔 translation
1회는 startup alignment 후 351 samples, 99.0 Hz를 기록했다. 안전 구간은 IK
6/6, failed/superseded 0, live TCP 위치 mean/worst `2.5/8.3 mm`, 방향
mean/worst `0.2/0.2 deg`, Cartesian limit와 joint clamp 0이었다.

sequence 6에서 직전 accepted target 대비 J1 `+3.1559`, J2 `+3.1269`, J3
`+1.5527`, J5 `+1.5889 rad`인 다른 IK branch가 나왔다. 기존 `>= 0.30 rad`
경계가 첫 publish 전에 차단했다. 실행 전후 J4는 `+0.021553368/+0.0216 rad`,
현장 기록상 CAN FD는 ERROR-ACTIVE이고 tx/rx error counter는 0이었으며 이상
소음·진동은 없었다.

회수한 `openarm-ik-refusal-2026-08-20.tar.gz`는 3,027 bytes이며 SHA-256은
`a57951e8f2ba52fcc98adf2fe99c3d976e18b29e12e6c3d2b6fd02a1f59c0178`이다.
pose JSON 4개와 log 5개가 위 수치를 확인한다. follow JSON은 생성되지 않아
351-sample phase time-series 자체는 재계산할 수 없다. 거부 전후 snapshot의 최대
관절 변화는 J5 `0.000762951 rad`, TCP 변화는 `0.271920 mm/0.081805 deg`이고
J4는 정확히 `+0.02155336842908362 rad`로 동일했다. 실제 pre-retest 7개 관절값은
synthetic replay seed에 반영했다.

첨부 CAN log는 기존 오른팔 매핑 `can0`이 아닌 `can1`을 기록하므로, 그 파일만으로
오른팔 CAN 상태를 독립적으로 입증하지 않는다. 원시 JSON·log, 압축파일과 생성
분석물은 Git에 넣지 않는다.
상세 근거는
[sequence-6 IK continuity 사건 문서](pose-follow-ik-continuity-incident-2026-08-20.md)에
있다.

새 정책은 기존 hard boundary를 완화하지 않는다. 첫 request는 startup measured
state, 이후 request는 직전 연속 accepted target을 seed이자 비교 기준으로 삼는다.
불연속 해는 target으로 승격하거나 publish하지 않고, 이전 accepted target을
유지하면서 동일 Cartesian 목표를 동일 seed로 최대 4회 푼다. 모든 관절 delta가
`0.30 rad` 미만인 해만 수락하며, 연속 해를 찾지 못하면 안전 종료한다.

`--output`이 지정된 safety refusal은 exit 3을 반환하기 전에 partial JSON을
원자 저장한다. `result.termination`, `is_partial`, `refusal`에 종료 원인·sequence·
phase·7개 delta를, `result.ik.continuity_*`와 `continuity_events`에 retry 이력을
남긴다. 거부된 sequence는 trace에 들어가지 않는다. MATLAB 분석기는 legacy,
full deterministic, partial/refused JSON을 한 bundle에서 함께 비교한다.

개발 PC에서 실물/CAN 없이 확인한 결과:

- 실제 pre-retest 7개 관절값을 seed로 한 합성 replay의 sequence 6에서 위
  J1/J2/J3/J5 delta를 4회 모두 차단하고, 이전 target을 유지한 채 partial JSON
  저장 및 exit 3
- 실제 ROS MoveIt + `mock_components/GenericSystem`의 비특이 seed에서 10 mm,
  5 mm/s translation 왕복 완료: 408 samples, 99.0 Hz, IK 9/9,
  failed/superseded/continuity refusal/clamp 모두 0
- MATLAB R2026a에서 2026-08-18 legacy real, full deterministic fake,
  partial/refused fake를 함께 분석해 CSV/JSON/MAT, PNG 6개와 7-page PDF 생성
- 집중 회귀 `19 passed`, pose/pose-follow CLI `67 passed, 8 deselected`, 전체
  Python `647 passed, 4 skipped`, ROS 2 Jazzy 11개 패키지 빌드 성공

현재 인계는 코드 커밋과 PR 및 fake 검증 결과까지다. 새 실물 명령을 실행하거나
재시험을 요청하지 말고, 별도 명시 승인과 다음 작업 지시를 기다린다.


## 최신 인계 — 표준 ready와 closest-IK 배치

오른팔 표준 자세는 `openarm_right_ready_v1 =
[0.15,0.55,0.15,0.8,-0.1,0.15,0.1] rad`, 관절별 허용 오차는 0.020 rad다.
`pose ready`는 오른팔만 지원하고 joint-space minimum-jerk trajectory를 사용한다.
ready 이동과 deterministic follow는 반드시 별도 명령으로 유지하며 자동 연결하지
않는다. closest-IK는 직전 accepted target seed에서 최대 4개 후보를 만든 뒤 기존
`>=0.30 rad` 경계를 먼저 적용하고 weighted joint distance 최소 해를 선택한다.

이 배치에서는 실물/CAN을 사용하지 않았고 실물 재시험을 요청하지 않는다. 다음
현장 단계가 별도로 승인되면 먼저 아래 dry-run만 검토한다.

```bash
robotctl pose ready --group openarm_right_arm
```

실물 이동 승인을 별도로 받은 경우에만 before/after 경로의 쓰기 가능성,
controller active, joint-state freshness와 E-stop을 확인하고 다음 ready 명령만
실행한다. 완료 뒤 즉시 follow를 시작하지 않고 after JSON과 도달 오차를 검토한다.

```bash
robotctl pose ready --group openarm_right_arm \
  --before-output /tmp/right-before-ready.json \
  --after-output /tmp/right-after-ready.json \
  --execute
```
