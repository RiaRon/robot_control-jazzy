# OpenArm 현재 진행 상태

마지막 갱신: 2026-08-18 (Asia/Seoul)

이 문서는 새 세션이 중단 지점부터 안전하게 이어가기 위한 스냅샷이다. 작업을
시작할 때 실제 Git 상태와 원격 PR 상태를 다시 확인한다.

## 저장소와 역할

- 저장소: `RiaRon/robot_control-jazzy`
- 개발 PC: `/home/cbj4/robot_control-jazzy`
- 안정 브랜치: `jazzy`
- 이번 기준 `jazzy`: `9262b0f`
- 진행 브랜치: `feature/pose-follow-lag-diagnostics`
- 기능 커밋: `01cd476`
- Pull Request:
  [#9](https://github.com/RiaRon/robot_control-jazzy/pull/9)
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

- PR #9가 `jazzy` 대상으로 열려 있다.
- 코드, 테스트, CLI 문서, Work 인계와 이 상태 갱신을 PR #9에 포함한다.
- 검증 후 PR #9를 병합하고 로컬 `jazzy`를 원격과 동기화한다.
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
