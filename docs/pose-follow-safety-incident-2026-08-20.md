# `pose follow` 안전 중단 조사 — 2026-08-20

## 결론

2026-08-20 오른팔 최소 배치는 중단 상태를 유지한다. 당시 JSON과 전체 명령줄은
보존되지 않았으므로 아래 결론은 `8a700c0` 코드 경로, 터미널 요약 수치와 운영자
관찰을 구분해 기록한다.

- `8a700c0`에서 `--execute` 없는 `pose follow`는 follow loop에 진입하지 않아
  startup alignment 위치 명령을 발행할 수 없다. 다만 ROS adapter를 열고 marker,
  관절 상태와 URDF를 읽는 불필요한 경로는 실행했다.
- 1133 samples와 IK accepted/failed/superseded, 관절 clamp 요약은 follow loop의
  `--execute` 경로에서만 출력된다. 따라서 그 요약을 만든 프로세스에는 실제로
  `--execute`가 파싱됐다고 보는 것이 코드상 유일하게 일관된다. shell alias,
  wrapper, 붙여넣은 실제 명령 또는 다른 터미널 로그는 원본이 없어 특정할 수 없다.
- `--output`은 실행이 모두 끝난 뒤 처음 쓰였다. 쓰기 불가 경로도 제어 시작 전에
  검사하지 않아, 로봇은 움직인 뒤 JSON 저장만 실패할 수 있었다. 이 부분은 확인된
  결함이다.

## 당시 확보된 증거

```text
samples: 1133
J3 worst tracking error: 0.7646 rad
J5 worst tracking error: 0.7480 rad
J3 target -> command worst: 0.7579 rad
J5 target -> command worst: 0.7424 rad
J4 position clamp: 516 / 1133 samples
live TCP position worst: 18.9 mm
live TCP orientation worst: 3.9 deg
IK: 7 accepted, 3 superseded
rotation profile: not run
```

이후 `right-pose-before.json`을 Downloads에서 확인했다. 원본은 읽기만 했고
Git에는 포함하지 않았다. SHA-256은
`2c6b96b0518c71619f4b80b01a71860c9db98f60a30982688474863f2788a164`다.
fixture에는 다음 canonical 관절값만 복사했다.

```text
r_aj_1 -0.002861066605630569
r_aj_2 +0.005149919890135024
r_aj_3 +0.008964675364309116
r_aj_4 -0.005531395437552433
r_aj_5 -0.026131074998092530
r_aj_6 -0.021553368429083620
r_aj_7 -0.000572213321126114
```

J4 실측값은 모델 lower `0 rad`보다 `0.0055313954 rad` 낮다. 원시 follow
trace는 여전히 없으므로 516개 clamp 각각의 방향과 당시 J3/J5 jump 부호는
복구할 수 없지만, startup 상태의 J4 lower 위반은 snapshot으로 확정됐다.

## J4 clamp 계산 경로

오른팔 프로파일의 J4(`r_aj_4`) 제한은 lower `0 rad`, upper `2.44346 rad`,
velocity `2 rad/s`다. 한 follow 주기의 후보 명령은 다음 순서를 거친다.

1. `command + kp * (ik_target - measured) * elapsed`로 관절 후보를 만든다.
2. TCP 선속도·각속도 제한 비율을 후보 전체에 적용한다.
3. `CommandGate.follow()`가 이전 command 기준 관절 속도 제한을 적용한다.
4. measured 기준 최대 lead(`velocity * 0.1 s`, J4는 `0.2 rad`)를 적용한다.
5. 최종 command를 관절 위치 범위로 clamp한다.
6. 기존 코드는 clamp된 command를 controller에 발행하고 횟수를 기록했다.

정확한 pose를 쓰고 IK offset을 전혀 추가하지 않은 replay는 첫 accepted target이
measured state와 같아도 J4를 즉시 `lower: r_aj_4`로 재현했다. 따라서 startup
clamp 방향은 lower다. 원시 trace가 없어 516회 전부가 lower였다고 단정할 수는
없지만, upper 방향 증거는 없고 초기 lower 위반만으로 기존 clamp 경로가 시작될
조건은 충분하다.

URDF 생성 원본과 robot-control profile은 J4 lower를 모두 정확히 `0 rad`로
정의하며 MoveIt 설정은 위치 한계를 override하지 않는다. 실물 hardware
interface는 motor `get_position()`을 offset 없이 ROS state로 내보내고, 활성화 때
`0 rad`를 명령하지만 1 ms 뒤 한 번만 상태를 받는다. 따라서 snapshot은
엔코더/ROS 관절 좌표와 모델 한계가 `5.53 mrad` 어긋나 있음을 증명한다. 다만 이
한 장의 snapshot만으로 motor에 저장된 영점 calibration 오차와 impedance
settling·제어 오버슈트를 구분할 수는 없다. 실물 mechanical-stop calibration
없이는 URDF lower를 완화하거나 motor zero를 다시 쓰지 않는다.

## 반영한 안전 경계

- dry-run은 ROS adapter 생성 전에 끝난다. startup alignment를 포함해 publish
  경로가 열리지 않으며 marker, joint state, URDF도 읽지 않는다.
- `--execute --output FILE`은 adapter 생성 전에 부모 디렉터리를 만들고 FILE과
  같은 디렉터리에 임시 파일을 실제 생성·flush·`fsync`·삭제한다. 실패하면 종료
  코드 2로 끝나고 로봇 명령을 보내지 않는다.
- 첫 accepted IK target은 startup measured state와, 이후 target은 직전 accepted
  target과 비교한다. 단일 관절 절댓값 변화가 `0.30 rad` 이상이면 해당 target을
  한 번도 publish하지 않고 종료 코드 3으로 거부한다.
- `--ik-jump-threshold`는 분석 이벤트 임계값으로만 유지된다. 하드 안전 경계
  `0.30 rad`는 CLI로 완화할 수 없다.
- deterministic profile에서 position clamp가 하나라도 생기면 clamp된 목표를
  publish하지 않고 종료 코드 3으로 거부한다. 수동 marker follow의 기존
  속도·lead·position clamp 동작은 유지한다.
- deterministic alignment 완료 문장은 marker를 움직이라는 일반 안내 대신
  marker를 Current에 두고 움직이지 말라는 profile 전용 안내를 출력한다.

## 검증 결과

실물과 CAN은 사용하지 않았다.

- 안전 회귀 + 기존 pose-follow: `36 passed, 46 deselected`
- 전체 Python (최신 `jazzy` 병합 후): `643 passed, 4 skipped`
- ROS 2 Jazzy 빌드: 11개 패키지 성공
- OpenArm fake smoke: 오른팔 TCP z `+30.0 mm`, residual `0.0 mm`
- fake 오른팔 command topic 감시 중 dry-run 실행: 8초간 message 0건
- 같은 감시 중 쓰기 불가 `/proc/...` output과 `--execute`: ROS 연결 전 exit 2,
  8초간 message 0건
- 정확한 7관절 fixture + terminal magnitude replay의 J3/J5
  `+0.7646/-0.7480 rad` branch jump: 첫 publish 전 exit 3, stream 0건
- 정확한 7관절 fixture, IK offset 0의 J4 clamp: `lower: r_aj_4`, 첫 publish 전
  exit 3, stream 0건
- fake MoveIt에서 snapshot TCP를 snapshot seed로 풀면 delta는 전 관절 정확히
  `0 rad`였다. 즉 현재 TCP 자체는 branch jump를 만들지 않았다.
- 같은 seed의 world-x `2 mm` 목표를 50회 풀면 50회 accepted, 39회가 한 관절
  `0.30 rad` 이상이었다. J3 절댓값 범위는 `0.00157–1.57976 rad`, J5는
  `0.00128–1.59693 rad`였고, terminal 크기에 가장 가까운 해는
  J3 `-0.753756 rad`, J5 `+0.753498 rad`였다.

따라서 약 `0.75 rad` 현상은 정확한 초기 pose에서 작은 translation 목표가
들어올 때 KDL이 여유자유도의 다른 branch를 선택하는 것으로 재현된다. 선택은
호출마다 달라 exact terminal 값이 결정적이지 않지만, 새 `0.30 rad` 경계는
그러한 해를 controller로 처음 보내기 전에 차단한다.
