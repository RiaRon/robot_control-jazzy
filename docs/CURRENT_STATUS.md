# OpenArm 현재 진행 상태

마지막 갱신: 2026-08-18 (Asia/Seoul)

이 문서는 새 Codex 세션이 이전 대화를 기억하지 못해도 작업을 안전하게 이어가기
위한 짧은 인계 문서이다. 상세 연구 기록은 아래 Notion 연구일지를 참고한다.

- [2026-08-14 GitHub 정리 및 가짜 하드웨어 검증](https://app.notion.com/p/3bcca70aae2a8107b4d9f1ee7509b72c?pvs=204)

## 기준 저장소와 컴퓨터

- 개발 기준 저장소: `RiaRon/robot_control-jazzy`
- 개발 PC 작업 폴더: `/home/cbj4/robot_control-jazzy`
- 기준 브랜치: `jazzy`
- 2026-08-18 `jazzy` 기준 커밋: `93a4c1f`
- 개발 PC의 `origin`: `https://github.com/RiaRon/robot_control-jazzy.git`
- OpenArm 컴퓨터는 배포, 실물 실행, 측정 전용이다.
- `RiaRon/robot_control`은 선배 저장소를 fork한 별도 저장소이며 현재 작업 대상이
  아니다.

위 값은 스냅샷이므로 작업을 시작할 때 실제 Git 상태를 다시 확인한다.

## 2026-08-18 완료 내용

- `feature/pose-show-json`에서 `robotctl pose show --output <파일.json>`을
  구현했다.
- 기능 브랜치의 원래 커밋은 `46c4645`이고, rebase 병합된 `jazzy`의 기능
  커밋은 `e168919`이다.
- `jazzy` 대상 Pull Request는
  [#5](https://github.com/RiaRon/robot_control-jazzy/pull/5)이며 `93a4c1f`까지
  병합됐다.
- JSON에는 schema version, 프로필, 선택한 그룹, canonical 관절 이름·위치와
  TCP frame·tip link·XYZ·XYZW Quaternion·RPY가 기록된다.
- 기존 화면 출력과 읽기 전용 동작은 유지된다. 모든 ROS 읽기가 성공한 뒤에만
  임시 파일을 목적 파일로 원자적으로 교체하며, 읽기 실패 시 기존 파일을
  보존하는 단위 테스트를 추가했다.
- 관련 테스트는 `3 passed`, 전체 테스트는 `615 passed, 4 skipped`였다.
- ROS 2 Jazzy 워크스페이스의 11개 패키지 빌드에 성공했다.
- `mock_components/GenericSystem`에서 오른팔 JSON을 `/tmp`에 실제 생성했고,
  관절 7개와 TCP XYZ·Quaternion·RPY를 확인했다. CAN과 실물 모터는 사용하지
  않았다.
- OpenArm 컴퓨터 `user@user-NUC14SRK-B`의 `/home/user/robot_control-jazzy`에서도
  가짜 하드웨어 JSON 저장을 확인했다. 이 컴퓨터의 HDGP asset은
  `/home/user/rl_ws/hdgp`에 있으므로 각 터미널에서
  `HDGP_ROOT=/home/user/rl_ws/hdgp`를 export해야 한다.
- OpenArm 컴퓨터의 가짜 오른팔 결과는 관절 7개가 모두 `0.0`이고 TCP XYZ가
  `[0.0, -0.1534977369405261, 0.08189955003653106] m`였으며 JSON 구조와
  Quaternion·RPY를 `python3 -m json.tool`로 확인했다.

## 2026-08-14 완료 내용

- Codex의 프로젝트·하드웨어 안전 규칙을 루트 `AGENTS.md`에 기록했다.
- 선배 GitHub는 읽기 전용으로만 사용하고 본인 저장소에만 push하도록 규칙을
  추가했다.
- `divingyoon/hdgp`의 `pour` 브랜치 커밋 `f285c17`에서 필요한
  `openarm_tesollo_sensor_rl` asset만 `/home/cbj4/hdgp`에 sparse clone했다.
- `/home/cbj4/hdgp`의 fetch URL은 선배 저장소이지만 push URL은 `DISABLED`다.
- asset manifest SHA-256은 프로젝트가 기대한
  `5f0e422ae33230592f97732448c7018ee853b40a03e689d8f884c9a3d6719798`과
  일치했다.
- ROS 2 Jazzy 시스템 의존성 확인과 11개 ROS 패키지 빌드에 성공했다.
- 전체 Python 테스트 결과는 `613 passed, 4 skipped`였다.

## 가짜 하드웨어 기준 실험

- `./ros_ws/pose_bringup.sh`를 `--real` 없이 실행했다.
- 하드웨어가 `mock_components/GenericSystem`인지 확인했으며 CAN과 실제 모터는
  사용하지 않았다.
- 오른팔 TCP 초기 위치는 `xyz [0.0000, -0.1535, 0.0819] m`였다.
- 오른팔 TCP를 Z축으로 `+0.03 m` 이동했다. 최종 Z는 `0.1119 m`, 잔차는
  `0.0 mm`였고 controller가 `Goal reached`를 보고했다.
- 다시 Z축으로 `-0.03 m` 이동해 초기 높이로 복귀했다. 복귀 잔차도 `0.0 mm`였다.
- 종료 시 MoveIt이 SIGINT 뒤 늦게 닫혀 SIGTERM 경고가 있었지만 두 이동 명령과
  mock controller 종료는 성공했다.
- 실물 OpenArm 테스트는 수행하지 않았다.

## 현재 중단 지점

- 기능 구현, 로컬 테스트, 가짜 하드웨어 검증과 PR #5의 `jazzy` 병합까지
  완료했다.
- 이 상태 문서 갱신 직전 개발 PC의 로컬·원격 `jazzy`는 `93a4c1f`로
  동기화됐다.
- OpenArm 컴퓨터에서 가짜 하드웨어 JSON 읽기는 성공했지만 실물 OpenArm의
  JSON 자세는 아직 읽지 않았다.
- OpenArm 컴퓨터의 로컬 호스트명과 저장소 경로는 확인됐지만 원격 SSH 주소는
  아직 확인되지 않았다. 과거 5070ti 후보 주소는 연결이 시간 초과됐다.
- 실물 OpenArm을 움직이는 시험은 수행하지 않았고 현재 승인도 없다.

## 다음 재개 절차

1. 실제 상태를 확인한다.

   ```bash
   cd /home/cbj4/robot_control-jazzy
   git status --short --branch
   git remote -v
   git log -1 --oneline --decorate
   ```

2. OpenArm 컴퓨터의 저장소는 `/home/user/robot_control-jazzy`, HDGP 경로는
   `/home/user/rl_ws/hdgp`로 확인됐다. 실제 Git 상태와 최종 빌드 결과는 그
   컴퓨터에서 다시 기록한다.
3. 실물 브링업 전 `ip -br link show type can`과 각 인터페이스의
   `ip -details link show` 결과로 CAN FD의 `state UP`, `fd on`을 확인한다.
4. 케이블을 따라 오른팔·왼팔의 실제 CAN 인터페이스 매핑을 확인한다. 두 팔은
   같은 모터 ID를 사용하므로 소프트웨어 출력만 보고 매핑을 추측하지 않는다.
5. 정확한 매핑과 비상정지 준비가 확인된 뒤에만 실물 브링업을 검토한다. 그 뒤
   `pose show`에는 `--execute`를 붙이지 않고 JSON을 저장한다.
6. 화면 결과와 JSON의 프로필, 그룹, 7개 관절, TCP XYZ·Quaternion·RPY를
   대조하고 작은 JSON을 개발 PC로 가져온다.
7. `pose show` 자체는 읽기 전용이지만 실물 브링업은 컨트롤러를 활성화하므로
   정확한 CAN 매핑과 장비 안전 준비 없이 실행하지 않는다.
8. 이후 움직임 시험이 필요해지면 먼저 `README.md`와
   `docs/openarm-test-workflow.md`를 전부 읽고 사용자의 명시적 승인을 받는다.

## 재개할 때 사용자에게 확인할 내용

- OpenArm 컴퓨터의 SSH 주소 또는 같은 네트워크에서 접근 가능한 방법
- OpenArm 컴퓨터의 실제 Git 상태와 최종 빌드 결과
- 움직임 시험까지 진행할 경우에만 실제 오른팔·왼팔 CAN 인터페이스 매핑
- 움직임 시험까지 진행할 경우에만 당일 실물 움직임에 대한 명시적 승인
