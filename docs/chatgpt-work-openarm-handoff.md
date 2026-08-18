# ChatGPT Work 인계 — OpenArm 연구진행

아래 내용은 ChatGPT 계정의 `OpenArm 연구진행` Work가 OpenArm 컴퓨터 작업을
이어가기 위한 인계 문서이다. Codex는 개발 PC에서 코드 수정·테스트·가짜
하드웨어 검증을 담당하고, 이 Work는 OpenArm 컴퓨터의 배포·CAN·실물 검증과
현장 안전 기록을 담당한다.

## 저장소와 현재 코드

- 기준 저장소: `RiaRon/robot_control-jazzy`
- 기준 브랜치: `jazzy`
- 인계문 작성 직전 `jazzy`: `d55d2b3`
- pose JSON 기능 커밋: `e168919`
- 기능 PR: <https://github.com/RiaRon/robot_control-jazzy/pull/5>
- OpenArm 가짜 검증 기록 PR: <https://github.com/RiaRon/robot_control-jazzy/pull/7>
- OpenArm 컴퓨터 저장소: `/home/user/robot_control-jazzy`
- OpenArm 컴퓨터 HDGP: `/home/user/rl_ws/hdgp`

작업 시작 시 문서의 커밋을 그대로 가정하지 말고 실제 Git 상태를 확인한다.

```bash
cd /home/user/robot_control-jazzy
git status --short --branch
git remote -v
git fetch origin
git switch jazzy
git pull --ff-only origin jazzy
git log -1 --oneline --decorate
```

미커밋 변경이 있으면 pull이나 switch를 계속하지 말고 먼저 내용을 검토한다.

## 완료된 개발·검증

- `robotctl pose show --output <파일.json>`이 구현되어 `jazzy`에 병합됐다.
- JSON에는 schema version, 프로필, 선택 그룹, canonical 관절 이름·위치와
  TCP frame·tip link·XYZ·XYZW Quaternion·RPY가 기록된다.
- 모든 ROS 읽기가 성공한 뒤에만 목적 파일을 원자적으로 교체한다. 읽기 실패 시
  기존 JSON을 보존한다.
- 개발 PC 전체 Python 테스트: `615 passed, 4 skipped`.
- 개발 PC ROS 2 Jazzy 빌드: 11개 패키지 성공.
- 개발 PC 가짜 하드웨어 JSON 통합 검증 성공.
- OpenArm 컴퓨터에서도 가짜 하드웨어 JSON 생성과 파싱에 성공했다.

## OpenArm 컴퓨터에서 확인된 환경

기본 asset 경로와 실제 HDGP 위치가 다르므로 ROS/CLI를 사용하는 각 터미널에서
다음을 먼저 설정한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/robot_control-jazzy/ros_ws/install/setup.bash
export PYTHONPATH="/home/user/robot_control-jazzy/src:/home/user/robot_control-jazzy:${PYTHONPATH:-}"
export HDGP_ROOT=/home/user/rl_ws/hdgp
```

가짜 하드웨어 오른팔 결과:

- 관절 `r_aj_1`~`r_aj_7`: 모두 `0.0 rad`
- TCP XYZ: `[0.0, -0.1534977369405261, 0.08189955003653106] m`
- JSON: `/tmp/right-pose-fake.json`
- `python3 -m json.tool /tmp/right-pose-fake.json` 파싱 성공

CAN과 실물 모터는 이 검증에 사용하지 않았다.

## 아직 하지 않은 일

- 실물 OpenArm의 JSON 자세 읽기
- 현재 CAN FD 링크 상태 확인
- 오른팔·왼팔의 물리적 CAN 인터페이스 매핑 확인
- 실물 움직임 명령

`pose show` 자체는 읽기 전용이지만 `--real` 브링업은 하드웨어와 컨트롤러를
활성화한다. 따라서 읽기 작업이라도 CAN 매핑과 현장 안전 준비 없이 실물
브링업을 실행하지 않는다.

## Work에서 다음에 할 일

1. 가짜 브링업이 남아 있으면 `Ctrl+C`로 종료하고 ROS 노드가 남지 않았는지
   확인한다.
2. 다음 읽기 전용 명령의 전체 출력을 기록한다.

   ```bash
   ip -br link show type can
   ip -details link show can0
   ip -details link show can1
   ```

3. 케이블을 물리적으로 따라 오른팔과 왼팔이 각각 어느 인터페이스인지 확인한다.
   두 팔은 같은 모터 ID를 사용하므로 소프트웨어 출력만으로 좌우를 추측할 수
   없다.
4. 두 링크의 `state UP`, `fd on`, 정확한 좌우 매핑과 비상정지 준비를 기록한다.
5. 위 조건을 확인한 뒤에만 실물 브링업 명령을 구성한다. `--right-can`과
   `--left-can` 값은 확인된 실제 매핑을 사용하고 추측하지 않는다.
6. 실물 자세 읽기에는 `pose show`만 사용하며 `--execute`를 붙이지 않는다.
7. 실물 움직임은 사용자가 그 작업에서 별도로 명시 승인하기 전에는 실행하지
   않는다.

## Codex로 돌려보낼 결과

- OpenArm 컴퓨터의 `git status --short --branch`와 최종 커밋
- ROS 빌드 결과 요약
- CAN FD 상태와 물리적으로 확인한 좌우 매핑
- 실물 브링업 여부와 사용한 정확한 명령
- `pose show` 화면 출력과 생성한 작은 JSON
- 실물 움직임이 있었는지 여부

rosbag, HDF5, MCAP, 영상과 대용량 센서 데이터는 GitHub에 올리지 않고 USB나
별도 실험 데이터 저장소로 전달한다.
