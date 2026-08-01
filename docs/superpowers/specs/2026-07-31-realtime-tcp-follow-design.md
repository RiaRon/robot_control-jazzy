# MoveIt IK 기반 TCP 마커 추종 설계

## 목적

`robotctl pose follow`가 RViz 마커의 위치를 현재 TCP 기준으로 실시간 추종하되,
RViz/MoveIt이 선택하는 목표 관절 자세와 일관된 해를 사용하고 실제
`/joint_states`가 그 목표에 수렴할 때까지 명령을 유지한다.

## 범위와 사용자 계약

- TCP 위치 `x/y/z`만 추종한다.
- 시작 시 실측 TCP orientation을 고정하고 마커 회전은 무시한다.
- 첫 marker feedback은 현재 TCP에 앵커링한다. 남아 있던 RViz goal 때문에
  시작 직후 팔이 움직이지 않는다.
- 이후 목표는 `시작 TCP + (현재 마커 - 시작 마커)`의 `world` 위치다.
- 오른팔과 왼팔에 같은 경로를 사용한다.
- `--execute`가 없으면 어떤 명령도 발행하지 않는다.
- 기존 `--gravity`, `--seconds`, `--max-tcp-speed`, `--tolerance` 인터페이스를
  유지한다. `--kp`와 `--ki`는 호환성을 위해 파싱하되 IK 래치 모드에서는
  Cartesian PI gain으로 사용하지 않음을 도움말과 문서에 명시한다.

## 선택한 구조

마커 목표 생성, MoveIt IK 목표 계산, 관절 목표 추종을 분리한다.

1. 빠른 제어 루프가 marker feedback과 최신 `/joint_states`를 처리한다.
2. 새 마커 위치가 들어오면 앵커 기준 TCP 목표를 만든다.
3. 목표가 마지막 제출 위치보다 tolerance 이상 변했을 때 현재 실측 관절을
   seed로 MoveIt `/compute_ik`에 제출한다.
4. IK 요청은 제어 루프를 막지 않는 latest-wins 작업자로 처리한다. 처리 중 새
   목표가 오면 대기 중인 이전 목표를 버리고 최신 목표만 계산한다.
5. 성공한 IK 결과를 새로운 래치 관절 목표로 원자적으로 교체한다.
6. 제어 루프는 IK 계산 중에도 마지막 유효 관절 목표를 계속 추종한다.
7. 매 주기 래치 목표 방향으로 움직이되, 목표 TCP 이동 속도
   `--max-tcp-speed`에 대응하는 작은 Cartesian 보간 목표를 만들고 현재 래치
   IK 해로 향하는 관절 증분을 제한한다.
8. 모든 관절 명령은 기존 `CommandGate.follow`의 위치·속도·최대 lead 제한을
   통과한다.

MoveIt 서비스 호출과 ROS subscription 처리는 같은 노드의 동시 `spin_once`로
경쟁시키지 않는다. IK 작업자는 독립적인 ROS adapter/node를 사용하고 종료 시
반드시 join/close한다.

## IK 요청과 실패 처리

- IK pose frame은 `world`, link는 group의 TCP tip, group은 profile의 MoveIt
  planning group을 사용한다.
- orientation은 follow 시작 시 실측 FK에서 저장한 quaternion이다.
- seed는 요청을 만들 때의 최신 실측 관절값이다.
- MoveIt collision checking을 활성화한다.
- 아직 처리되지 않은 요청은 한 개만 보관한다.
- IK 실패, timeout 또는 유한하지 않은 결과는 래치 목표를 바꾸지 않는다.
- 연속 실패 중에도 마지막 유효 목표를 유지하며 실패 횟수를 보고한다.
- 첫 IK 성공 전에는 현재 자세만 유지한다.

## 피드백과 속도

MoveIt IK 결과는 목표일 뿐 실제 모터 상태가 아니다. 빠른 루프는 매 샘플
`/joint_states`를 읽고 래치 목표와 비교한다. 명령은 이전 명령에 무한 누적하지
않고, 현재 실측 상태와 래치 목표 사이에서 안전 gate가 허용하는 다음 위치로
갱신한다. 따라서 모터 처짐이 있어도 같은 목표가 유지되며 실제 관절이 계속
수렴한다.

표시 주파수와 실제 주파수를 구분한다. 종료 시 전체 경과시간으로 계산한 실제
제어 주파수와 joint-state 대기시간을 보고한다. 프로파일의 100 Hz는 목표값이며
실제 hardware/joint-state 주기가 더 느리면 그 사실을 숨기지 않는다.

## 안전

- 첫 marker feedback으로 이동하지 않는다.
- 유한하지 않은 marker, joint state, IK 결과는 발행하지 않는다.
- 기존 joint position, velocity, effort 및 maximum-lead 제한이 최종 권한이다.
- 마커 점프는 `--max-tcp-speed`와 관절 gate로 제한한다.
- IK가 서로 다른 분기로 크게 점프하면 관절 gate가 한 주기 이동량을 제한한다.
- `Ctrl+C`, 시간 만료 또는 예외 시 작업자를 종료하고 gravity effort를 0으로
  되돌린다. trajectory controller는 마지막 승인 위치를 유지한다.
- 개발 도구와 자동 테스트는 실물에 `--execute`를 실행하지 않는다.

## 보고

종료 출력에 다음을 포함한다.

- 실제 제어 샘플 수와 평균 주파수
- IK 요청, 성공, 실패, 최신 목표로 대체된 요청 수
- 마지막/평균 TCP 위치 오차
- tolerance 안에 든 샘플 비율
- 마지막 목표 관절과 실측 관절의 최대 오차
- Cartesian 및 joint safety clamp 횟수

## 코드 경계

- `ros_adapter.py`: 독립 IK worker가 사용할 수 있는 blocking IK 호출과 명확한
  adapter 생명주기 제공
- 새 순수 모듈: latest-wins 요청 상태, 성공한 관절 목표 래치, 통계
- `cli.py`: marker 앵커, 작업자 시작/종료, 빠른 관절 피드백 루프와 보고
- `tests/`: stale marker 무이동, 최신 요청 우선, IK 실패 시 hold, 현재 joint
  seed 사용, 래치 목표 수렴, 속도/gate 제한, 정상 종료를 검증

## 실물 승인 기준

1. 시작 시 stale marker가 있어도 이동하지 않는다.
2. 마커를 10 mm 이동한 뒤 놓으면 실제 TCP가 2 mm 이내로 수렴한다.
3. 같은 시험에서 마지막 관절 오차가 gate의 허용 lead보다 작다.
4. 50 mm/s 이하의 연속 이동에서 목표가 멈춘 뒤 수렴한다.
5. 종료 보고의 실제 루프 주기와 IK 성공/실패가 관측 가능하다.
6. E-stop을 잡은 운영자가 오른팔부터 검증하며 자동 실행하지 않는다.
