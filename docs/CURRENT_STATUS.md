# OpenArm 현재 진행 상태

마지막 갱신: 2026-08-20 (Asia/Seoul)

이 문서는 새 세션이 중단 지점부터 안전하게 이어가기 위한 스냅샷이다. 작업을
시작할 때 실제 Git 상태와 원격 PR 상태를 다시 확인한다.

## 저장소와 역할

- 저장소: `RiaRon/robot_control-jazzy`
- 개발 PC: `/home/cbj4/robot_control-jazzy`
- 안정 브랜치: `jazzy`
- PR #12 기능 병합 기준 `jazzy`: `2b2ba14`
- 이전 PR #9 기능 병합 기준: `6b4b51c`
- 최신 병합 기능 커밋: `2b2ba14`
- 이전 상태 커밋: `6b4b51c`
- Pull Request:
  [#12](https://github.com/RiaRon/robot_control-jazzy/pull/12) (merged),
  [#9](https://github.com/RiaRon/robot_control-jazzy/pull/9) (merged)
- OpenArm 컴퓨터: `user-NUC14SRK-B`
- OpenArm 저장소: `/home/user/robot_control-jazzy`
- HDGP: `/home/user/rl_ws/hdgp`

Codex는 개발 PC의 코드 수정, 테스트, ROS 빌드와 가짜 하드웨어 검증을
담당한다. ChatGPT 계정의 `OpenArm 연구진행` Work는 OpenArm 컴퓨터의 배포,
CAN, 실물 검증과 현장 안전 기록을 담당한다. 실물 움직임은 해당 작업에서
사용자가 명시 승인한 경우에만 수행한다.

## 이전 완료 기능

- `robotctl pose show --output <파일.json>`이 `jazzy`에 병합됐다.
- 화면 출력과 읽기 전용 동작을 유지하면서 canonical 관절과 TCP
  XYZ·Quaternion·RPY를 원자적 JSON으로 저장한다.
- 개발 PC와 OpenArm 컴퓨터의 가짜 하드웨어에서 오른팔 JSON 저장·파싱을
  확인했다.
- OpenArm 컴퓨터에서는 각 터미널에
  `HDGP_ROOT=/home/user/rl_ws/hdgp`가 필요하다.

## 실물 오른팔 확인 결과

현재 실물 연구 범위는 오른팔 `openarm_right_arm`만이다.

- 오른팔 CAN `can0`, 왼팔 CAN `can1`을 물리 확인했다.
- 두 링크는 `UP`, `ERROR-ACTIVE`, CAN FD, 1/5 Mbit/s이며 오류 counter는
  0이었다.
- 실물 자세 JSON, canonical 관절 변환과 FK가 정상이다.
- 중력 보상 기준은 오른팔 전 관절 `1.0`이다.
- `pose follow` 실제 주기는 약 99.1 Hz였고 두 시험 모두 IK 실패 0회였다.

두 번째 30초 시험을 기준선으로 보존한다.

```text
TCP position: mean 12.0 mm, worst 63.3 mm, last 1.4 mm
TCP orientation: mean 1.1 deg, worst 3.6 deg, last 0.4 deg
Cartesian speed limit: 1025 / 2866 samples (35.8%)
last maximum joint error: 0.0063 rad
all joint velocity/lead/position clamps: 0
r_aj_4: mean 0.0178 rad, worst 0.1921 rad, last 0.0063 rad
```

오른팔은 마커 정지 후 1.4 mm까지 수렴했다. 현재 문제는 정적 도달 실패가
아니라 안전 제한을 유지하면서 이동 중 지연의 원인을 분리하는 것이다.

## PR #9 변경

`pose follow --output <파일.json>`을 추가했다. 제어 게인, Cartesian 속도,
관절 lead와 safety gate는 바꾸지 않았다.

JSON은 다음 위치·방향 오차 계층을 요약과 100 Hz trace로 기록한다.

1. 최신 마커 → 실측 TCP
2. 최신 마커 → 채택된 IK 요청의 마커
3. 채택된 마커 → IK 중간목표
4. IK 목표 → 상태 샘플 시점의 활성 명령
5. 활성 명령 → 실측 상태

관절별로도 `IK target → command`와 `command → measured`를 분리하므로
J4 지연의 소프트웨어·실물 기여를 구분할 수 있다. 기존
`trailed the marker by`는 이전 기준선 비교를 위해 유지하지만, 채택된 IK
요청의 마커 스냅샷 기준임을 문서화했다.

코드상 `kp=2.0 s^-1`인 1차 추종은 20 mm/s 목표에서 이상적인 경우에도
`속도 / kp ≈ 10 mm`의 지연을 만들 수 있다. 이는 원인 후보에 대한 추론이며
실제 기여도는 새 로그로 확인한다. 첫 비교 전에는 `kp`나 속도 한계를
변경하지 않는다.

## 개발 PC 검증

- pose-follow 코드·문서 회귀군: `30 passed, 46 deselected`
- 전체 Python: `619 passed, 4 skipped`
- ROS 2 Jazzy 빌드: 11개 패키지 성공
- fake adapter 완전 추종: `command → measured = 0` 검출
- fake adapter 처짐 모델: `command → measured > 0` 검출
- `mock_components/GenericSystem`과 좌우 trajectory controller 브링업 성공
- RViz가 `left_arm` marker만 게시해 오른팔 통합 실행은 명령 발행 전에
  안전 거부됐다. 현재 범위 밖인 왼팔로 우회하지 않았다.
- CAN과 실물 모터는 개발 PC 검증에 사용하지 않았다.

## 현재 중단 지점

- PR #9의 코드, 테스트, CLI 문서, Work 인계와 상태 문서가 `jazzy`에
  rebase 병합됐다.
- 개발 PC의 로컬·원격 `jazzy`는 동기화했다. 문서 후속 커밋이 있으므로
  재개할 때 실제 HEAD를 다시 확인한다.
- 다음 실물 단계는 Work에서 최신 `jazzy` 배포 후 동일한 `kp=2.0`
  기준선을 새 JSON으로 한 번 수집하는 것이다.

## Work의 다음 실물 명령

아래 명령은 실물을 움직인다. Work에서 당일 사용자의 명시 승인을 받고
E-stop과 작업 공간을 확인한 뒤에만 실행한다.

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

마커를 움직인 뒤 마지막 수 초 동안 고정한다. 터미널 전체 요약과
`/tmp/right-follow-kp2.json`, 마커 이동·고정 구간 및 안전 관찰을 Codex로
돌려보낸다. 대용량 trace는 GitHub에 커밋하지 않는다.

상세 판단 기준과 OpenArm 환경은
[`docs/chatgpt-work-openarm-handoff.md`](chatgpt-work-openarm-handoff.md)를
따른다.

## 최신 완료 — deterministic follow diagnostics

- PR [#12](https://github.com/RiaRon/robot_control-jazzy/pull/12)가 `jazzy`에
  rebase 병합됐다.
- 기능 기준 커밋: `2b2ba14`
- 기존 pose-follow 제어, gain, safety gate와 schema v1 필드는 유지했다.
- startup alignment 완료 여부·시각, IK request/complete/accepted 시각·latency,
  live-error 방향 signed projection, 관절별 IK target jump 이벤트를 추가했다.
- jump 이벤트는 관측 전용이며 target을 거부·클램프하지 않는다.
- deterministic `translation`, `rotation`, `translation-rotation` 왕복
  profile을 추가했다. 기본은 dry run이고 `--execute` 없이는 발행하지 않는다.
- profile hard cap은 거리 30 mm, 회전 10도, 20 mm/s, 0.10 rad/s, 3회이다.

검증:

- 전체 Python: `632 passed, 4 skipped`
- 관련 회귀: `46 passed, 46 deselected`
- ROS 2 Jazzy 빌드: 11개 패키지 성공
- 실제 ROS mock stack: `mock_components/GenericSystem`, 167 samples,
  97.4 Hz, IK 3/3 성공, 실패·superseded 0, JSON 저장 성공
- 실제 OpenArm/CAN/모터는 이 개발 배치에서 사용하지 않았다.

다음 실물 단계는 파라미터 tuning이 아니라 clean deterministic 기준선 2개다.

1. 오른팔 10 mm world-x translation 왕복, 5 mm/s, 양 끝 3초 hold
2. 오른팔 local-z 5도 rotation 왕복, 0.05 rad/s, 양 끝 3초 hold

정확한 배포·dry-run·실물 명령과 중단 조건은
[`docs/chatgpt-work-openarm-handoff.md`](chatgpt-work-openarm-handoff.md)의
'최신 인계 — deterministic 진단 배치'를 따른다. 두 clean JSON을 확보하기
전에는 kp나 속도 한계를 바꾸지 않는다. IK continuity 보호는 다음 별도 개발
배치이며 이번 커밋에는 포함되지 않았다.

## 최신 안전 수정 — 2026-08-20 중단 사건

오른팔 deterministic translation 최소 배치는 startup alignment로 보이는 소폭
움직임 직후 중단됐다. rotation은 실행하지 않았다. 당시 terminal 요약은 1133
samples, J3/J5 worst `0.7646/0.7480 rad`, J4 position clamp `516/1133`,
live TCP worst `18.9 mm/3.9 deg`, IK accepted 7, superseded 3이었다.

조사 결론:

- 기준 `jazzy@8a700c0`에서 `--execute` 없는 경로는 `_follow_loop`에 들어가지
  않아 startup alignment command를 publish할 수 없다. 1133-sample 요약은
  `--execute` 경로에서만 생성된다. 실제 argv나 shell history가 없어 어떻게
  `--execute`가 포함됐는지는 확정하지 못했다.
- output 쓰기는 제어 종료 후에만 시도하므로 쓰기 불가 경로가 움직임을 막지
  못한 것이 확인된 결함이다.
- J4 제한은 `[0, 2.44346] rad`, `2 rad/s`다. 기존 경로는 velocity, `0.2 rad`
  lead, position 순으로 clamp하고 clamp된 command를 publish했다. 초기 pose와
  trace가 없어 516회가 lower인지 upper인지는 확정하지 못했다.

`feature/pose-follow-safety-abort` 변경:

- dry-run은 ROS adapter 생성 전 반환하며 startup alignment를 포함한 publish,
  marker/joint-state/URDF 읽기를 모두 생략한다.
- `--execute --output`은 ROS 연결 전에 부모 디렉터리와 실제 sibling temporary
  file write/flush/fsync로 저장 가능성을 검사한다.
- 첫 accepted IK target은 startup measured state와 비교한다. 이후 target은
  직전 accepted target과 비교하며 단일 관절 변화 `>= 0.30 rad`를 해당 target의
  첫 publish 전에 거부한다. 이 하드 경계는 CLI로 완화할 수 없다.
- deterministic profile의 position clamp는 첫 publish 전에 거부한다. 수동
  marker follow의 기존 clamp 정책은 유지한다.
- deterministic startup 안내에서 일반 `drag the marker` 문장을 제거했다.

개발 PC 검증(실물/CAN 사용 안 함):

- 안전 회귀 + 기존 pose-follow: `36 passed, 46 deselected`
- 전체 Python: `639 passed, 4 skipped`
- ROS 2 Jazzy 빌드: 11개 패키지 성공
- OpenArm fake smoke: 오른팔 TCP z `+30.0 mm`, residual `0.0 mm`, 성공
- fake 오른팔 command topic 감시: dry-run 8초간 0건(exit 124 timeout)
- fake 오른팔 command topic 감시: 쓰기 불가 `/proc/...` output도 8초간 0건,
  CLI는 ROS 연결 전 exit 2
- fake/replay: 첫 IK에서 J3 `+0.7646 rad`, J5 `-0.7480 rad`를 재현하고
  publish 0건으로 exit 3; J4 lower clamp도 publish 0건으로 exit 3

남은 자료 문제: 사용자가 지정한
`openarm_follow_data/2024-08-20/right-pose-before.json`은 개발 PC에 없고,
전송된 `openarm-diagnostic-aborted-2026-08-20.tar.gz`는 0바이트다. 현재 replay는
terminal jump 수치를 사용하지만 정확한 초기 7개 관절값은 임시 0 자세다. 파일이
다시 전달되면 fixture를 실제 값으로 교체하고 전체 검증을 다시 실행한다.

실물 재시험은 아직 실행하지 않았다. 최신 경로 준비, dry-run, 승인 후 translation
한 번과 중단 조건은
[`docs/chatgpt-work-openarm-handoff.md`](chatgpt-work-openarm-handoff.md)의
'2026-08-20 안전 중단 이후 최신 인계'를 따른다. clean translation 검토 전에는
rotation, kp 또는 속도 한계를 변경하지 않는다.
