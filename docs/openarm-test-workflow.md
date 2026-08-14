# OpenArm 개발·실물 테스트 워크플로

## 저장소 역할

- `RiaRon/robot_control-jazzy`: 이 프로젝트의 개발·백업 저장소
- `RiaRon/robot_control`: 선배의 `divingyoon/robot_control`을 fork한 별도 저장소
- 개발 PC: 코드 수정, 검토, 커밋의 기준 장소
- OpenArm 컴퓨터: 빌드, 실물 실행, 측정만 하는 테스트 장소
- GitHub: 두 컴퓨터 사이의 코드와 작은 테스트 결과를 전달하는 중간 저장소
- USB: rosbag, HDF5, MCAP처럼 크거나 민감한 실험 데이터를 전달하는 수단

현재 개발 PC의 `origin`은 `RiaRon/robot_control-jazzy`만 가리킨다. 따라서 아래
명령으로 선배 저장소가 변경되지는 않는다. 선배 저장소에 변경을 제안하려면 별도로
fork 저장소에서 Pull Request를 만들어야 한다.

## 브랜치 규칙

- `jazzy`: 검토와 테스트가 끝난 기준 코드
- `humble`: ROS 2 Humble 기준 코드. `jazzy`와 통째로 합치지 않는다.
- `feature/이름`: 개발 PC에서 만드는 기능 또는 수정 브랜치
- `test/날짜-이름`: OpenArm 컴퓨터에서 작은 테스트 결과나 설정을 돌려보낼 때 사용

`jazzy`에서 바로 수정하지 말고 짧은 `feature/` 브랜치를 만든다. 실물 확인이 끝난
뒤 그 브랜치를 `jazzy`에 합친다.

## 1. 개발 PC에서 수정하기

아래 예시의 `can-tuning`은 작업에 맞는 짧은 영문 이름으로 바꾼다.

```bash
cd /home/cbj4/robot_control-jazzy
git switch jazzy
git pull --ff-only origin jazzy
git switch -c feature/can-tuning

# 코드 수정과 로컬 테스트 후
git status
git add 수정한_파일_경로
git commit -m "fix: describe the change"
git push -u origin feature/can-tuning
```

`git add .`보다 실제로 확인한 파일 경로만 지정하는 편이 안전하다.

## 2. OpenArm 컴퓨터로 보내기

처음 한 번은 다음처럼 복제한다.

```bash
git clone https://github.com/RiaRon/robot_control-jazzy.git
cd robot_control-jazzy
git switch feature/can-tuning
```

이미 복제돼 있다면 최신 커밋만 가져온다.

```bash
cd ~/robot_control-jazzy
git fetch origin
git switch feature/can-tuning
git pull --ff-only origin feature/can-tuning
```

OpenArm 컴퓨터에서는 `build/`, `install/`, `log/` 폴더를 복사해 오지 않고 그
컴퓨터의 ROS 2 Jazzy 환경에서 다시 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/install_dependencies_jazzy.sh   # 최초 설치나 의존성 변경 때만
./ros_ws/build.sh
source ros_ws/install/setup.bash
```

실물 명령은 README의 CAN 확인과 드라이런 절차를 먼저 수행한 뒤 실행한다.

## 3. 테스트 결과를 개발 PC로 돌려보내기

### 작은 파일: GitHub

수정된 YAML/JSON 설정, 테스트 코드, 짧은 Markdown 보고서는 OpenArm 컴퓨터에서
별도 테스트 브랜치에 커밋한다. 개발 PC와 OpenArm 컴퓨터에서 같은 브랜치를
동시에 수정하지 않는다.

```bash
git switch -c test/2026-08-14-can-tuning
git add 확인한_작은_파일_경로
git commit -m "test: record OpenArm CAN tuning result"
git push -u origin test/2026-08-14-can-tuning
```

개발 PC에서는 결과를 바로 합치기 전에 먼저 비교한다.

```bash
cd /home/cbj4/robot_control-jazzy
git fetch origin
git log --oneline --decorate origin/test/2026-08-14-can-tuning
git diff feature/can-tuning..origin/test/2026-08-14-can-tuning
```

내용을 확인한 뒤 필요한 커밋을 기능 브랜치에 합치거나 다시 수정한다.

### 큰 파일: USB

다음 파일은 GitHub에 올리지 않고 날짜와 테스트 이름을 붙인 폴더로 USB에
복사한다.

- rosbag (`*.db3`, `*.mcap`)
- HDF5 (`*.h5`, `*.hdf5`)
- `artifacts/`, `bags/`, ROS 빌드 로그
- 카메라 영상과 대용량 센서 원본

이 항목들은 저장소의 `.gitignore`에 등록돼 있다. 꼭 필요한 결과 요약과 재현에
필요한 작은 설정만 Git에 남긴다. 비밀번호, 토큰, 장비 고유 비밀값은 크기와
상관없이 커밋하지 않는다.

## 4. 테스트가 끝난 뒤

개발 PC에서 결과를 반영하고 다시 로컬 테스트한 뒤 기능 브랜치를 push한다.
GitHub에서 `feature/can-tuning`을 `jazzy`로 합치는 Pull Request를 만들고,
변경 파일을 확인한 다음 병합한다. 병합 후 OpenArm 컴퓨터는 다음 명령으로 기준
코드로 돌아온다.

```bash
git fetch origin
git switch jazzy
git pull --ff-only origin jazzy
```

## 사고 방지 원칙

1. 개발 PC를 수정의 기준으로 삼는다.
2. OpenArm 컴퓨터에서는 실물 테스트에 필요한 최소 변경만 `test/` 브랜치에 남긴다.
3. 실물 테스트 전에 현재 브랜치와 커밋을 `git status --short --branch`와
   `git log -1 --oneline`으로 기록한다.
4. 생성물과 대용량 데이터는 Git에 넣지 않는다.
5. `git push --force`, 브랜치 삭제, 선배 저장소 대상 PR은 목적을 확인하기 전에는
   실행하지 않는다.
