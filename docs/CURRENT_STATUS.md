# OpenArm 현재 진행 상태

마지막 갱신: 2026-08-14 (Asia/Seoul)

이 문서는 새 Codex 세션이 이전 대화를 기억하지 못해도 작업을 안전하게 이어가기
위한 짧은 인계 문서이다. 상세 연구 기록은 아래 Notion 연구일지를 참고한다.

- [2026-08-14 GitHub 정리 및 가짜 하드웨어 검증](https://app.notion.com/p/3bcca70aae2a8107b4d9f1ee7509b72c?pvs=204)

## 기준 저장소와 컴퓨터

- 개발 기준 저장소: `RiaRon/robot_control-jazzy`
- 개발 PC 작업 폴더: `/home/cbj4/robot_control-jazzy`
- 기준 브랜치: `jazzy`
- 2026-08-14 기준 `jazzy` 커밋: `6fb9911`
- 개발 PC의 `origin`: `https://github.com/RiaRon/robot_control-jazzy.git`
- OpenArm 컴퓨터는 배포, 실물 실행, 측정 전용이다.
- `RiaRon/robot_control`은 선배 저장소를 fork한 별도 저장소이며 현재 작업 대상이
  아니다.

위 값은 스냅샷이므로 작업을 시작할 때 실제 Git 상태를 다시 확인한다.

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

- 기능 코드는 아직 수정하지 않았다.
- 로컬 `feature/pose-show-json` 브랜치는 `6fb9911`에서 만들기만 했고 변경이나
  원격 push는 없다.
- 다음 기능 후보는 `robotctl pose show --output <파일.json>`이다.
- 목적은 OpenArm 컴퓨터에서 읽은 프로필, 그룹, 관절 위치, TCP XYZ·Quaternion·
  RPY를 작은 JSON 파일로 저장해 개발 PC로 가져오는 것이다.
- 기존 `pose show`의 화면 출력과 읽기 전용 동작은 유지해야 한다.
- ROS 읽기가 실패하면 불완전한 JSON 파일을 만들지 않아야 한다.

## 다음 재개 절차

다음 작업 예정일은 2026-08-18 화요일이다.

1. 실제 상태를 확인한다.

   ```bash
   cd /home/cbj4/robot_control-jazzy
   git status --short --branch
   git remote -v
   git log -1 --oneline --decorate
   ```

2. `jazzy`를 본인 GitHub와 동기화한다.

   ```bash
   git switch jazzy
   git pull --ff-only origin jazzy
   ```

3. 준비된 기능 브랜치로 이동한다. `jazzy`가 그 사이 변경됐다면 작업 전에 최신
   기준에서 브랜치를 갱신할 방법을 검토한다.

   ```bash
   git switch feature/pose-show-json
   ```

4. JSON 저장 기능과 단위 테스트, CLI 문서를 최소 범위로 수정한다.
5. 관련 테스트 후 전체 `pytest`와 ROS 빌드를 위험에 비례해 다시 실행한다.
6. remote가 본인 저장소인지 확인한 뒤 기능 브랜치를 push하고 `jazzy` 대상 Pull
   Request를 만든다.
7. OpenArm 컴퓨터에서 기능 브랜치를 fetch하고 그 컴퓨터에서 다시 빌드한다.
8. 실물 시험 전 `README.md`와 `docs/openarm-test-workflow.md`를 읽고 정확한 좌우
   CAN FD 매핑, 비상정지 준비, 현재 브랜치와 커밋을 확인한다.
9. 드라이런과 자세 읽기부터 수행하고, 사용자가 현재 작업에서 명시적으로 승인한
   경우에만 저속 실물 명령을 실행한다.
10. 작은 JSON 결과는 `test/2026-08-18-*` 브랜치나 검토된 별도 파일로, rosbag·
    HDF5·MCAP 등 큰 데이터는 USB로 개발 PC에 가져온다.

## 재개할 때 사용자에게 확인할 내용

- OpenArm 컴퓨터에서 이 저장소를 이미 clone했는지와 실제 경로
- OpenArm 컴퓨터의 네트워크 또는 GitHub 접근 가능 여부
- 실제로 확인된 오른팔·왼팔 CAN 인터페이스 매핑
- 당일 실물 움직임을 실행해도 되는지에 대한 명시적 승인
