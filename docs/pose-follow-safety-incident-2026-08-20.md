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

`right-pose-before.json`과 원시 follow JSON은 현재 개발 PC에 없다.
`/home/cbj4/openarm_follow_data/openarm-diagnostic-aborted-2026-08-20.tar.gz`도
0바이트다. 따라서 정확한 초기 관절값, J3/J5 jump 부호, J4 lower/upper clamp
방향은 아직 원시 자료로 확정할 수 없다.

## J4 clamp 계산 경로

오른팔 프로파일의 J4(`r_aj_4`) 제한은 lower `0 rad`, upper `2.44346 rad`,
velocity `2 rad/s`다. 한 follow 주기의 후보 명령은 다음 순서를 거친다.

1. `command + kp * (ik_target - measured) * elapsed`로 관절 후보를 만든다.
2. TCP 선속도·각속도 제한 비율을 후보 전체에 적용한다.
3. `CommandGate.follow()`가 이전 command 기준 관절 속도 제한을 적용한다.
4. measured 기준 최대 lead(`velocity * 0.1 s`, J4는 `0.2 rad`)를 적용한다.
5. 최종 command를 관절 위치 범위로 clamp한다.
6. 기존 코드는 clamp된 command를 controller에 발행하고 횟수를 기록했다.

따라서 J4 516회는 516개 샘플에서 5단계 직전의 J4 command가 `[0,
2.44346]` 밖이었다는 뜻이다. 초기 pose와 trace가 없으므로 lower 또는 upper 중
어느 방향인지는 터미널 합계만으로 결정할 수 없다. 작은 deterministic 왕복에서
position clamp는 예상 동작이 아니므로 수정본은 첫 발생을 발행 전에 거부한다.

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
- 전체 Python: `639 passed, 4 skipped`
- ROS 2 Jazzy 빌드: 11개 패키지 성공
- OpenArm fake smoke: 오른팔 TCP z `+30.0 mm`, residual `0.0 mm`
- fake 오른팔 command topic 감시 중 dry-run 실행: 8초간 message 0건
- 같은 감시 중 쓰기 불가 `/proc/...` output과 `--execute`: ROS 연결 전 exit 2,
  8초간 message 0건
- fake replay의 J3/J5 `+0.7646/-0.7480 rad` branch jump: 첫 publish 전 exit 3,
  stream 0건
- fake replay의 J4 lower position clamp: 첫 publish 전 exit 3, stream 0건

정확한 초기 pose replay는 `right-pose-before.json`이 다시 전달된 뒤 그 7개
관절값으로 fixture를 교체하고 전체 검증을 재실행해야 한다.
