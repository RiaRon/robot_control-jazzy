# Ready–deterministic follow 중력보상 인계 (2026-08-24)

## 범위와 입력

이 변경은 ROS 2 Jazzy `jazzy@e1903ce`를 기준으로 코드, Python fake, ROS
GenericSystem, MoveIt fake와 MATLAB만 사용한다. 실물 OpenArm/CAN 명령은
실행하지 않았다.

원본 archive는 `openarm-ready-v2-gravity-2026-08-24.tar.gz`, SHA-256은
`f4fefa54db78e2f64ef5919f45c385bd47ac5c026b7fbd03053e6f9b10a4bd1e`다.
원시 JSON과 분석 생성물은 저장소에 넣지 않는다.

## 실물 JSON 분석

표준 A-prime은 `openarm_right_ready_v2 = [0, 0.2, 0, 0.6, 0, 0, 0] rad`다.
gravity scale 1.0으로 987 samples를 기록했다. ready 성능은 cleanup 직전
feedback으로 계산하며 worst target error는 J4 `0.0470511940 rad`다.

zero-effort cleanup 뒤 pose JSON은 별도 drift로 해석한다. J4는
`0.5529488060 -> 0.4800869764 rad`, 즉 `-0.0728618296 rad` 처졌고 post-cleanup
target error는 `0.1199130236 rad`다. 따라서 후속 pose show의 큰 오차는 ready
도달 오차가 아니라 post-cleanup drift다. MATLAB 함수
`analyze_ready_cleanup_drift.m`이 이 두 구간을 분리해 6-file bundle을 만든다.

## 구현 흐름

1. dry-run과 output preflight를 마친 뒤에만 ROS adapter를 연다.
2. deterministic `--execute`는 검증된 gravity scale 1.0과 right arm만 허용한다.
3. position/effort controller가 active이고 command interface가 분리됐는지 확인한다.
4. 첫 ready 검사보다 먼저 measured-state gravity effort를 발행한다.
5. A-prime에서 0.050 rad를 벗어나면 J4-first 두 단계 minimum-jerk trajectory로
   재획득하며 매 제어주기 gravity effort를 갱신한다.
6. 재획득 settle 성공 뒤 기존 TCP startup position/orientation alignment를 한다.
7. alignment 성공 뒤에만 deterministic diagnostic target을 생성·publish한다.
8. ready 재획득, alignment와 profile 동안 gravity를 끊지 않는다.
9. 정상 종료, safety refusal, 예외에서 zero effort를 세 번 발행한다.

재획득 실패는 마지막 measured position을 safe hold하고 profile position publish
0건, partial JSON과 명확한 termination reason을 남긴다. startup alignment 실패,
IK continuity refusal과 deterministic position clamp도 profile 시작 전이면 같은
0-publish 계약을 유지한다.

## 0.050 rad follow 전용 tolerance 근거

기존 `pose ready` acceptance `0.020 rad`는 변경하지 않았다. follow handoff에만
`0.050 rad`를 적용한다. 값은 cleanup 직전 실물 worst `0.047051 rad`를 포함하면서
기존 IK single-joint hard boundary `>= 0.30 rad`보다 6배 작다.

`tools/analyze_follow_reacquisition.py`로 A-prime 주변 각 관절
`{-0.05, 0, +0.05}`의 3^7=2,187 상태를 평가했다.

- joint-limit failure 0; 최소 limit margin `0.324533 rad`
- Jacobian minimum rank 6
- minimum singular value `0.0477313`; maximum condition number `38.9763`
- MoveIt state-validity 143 samples(중심, single-axis extrema, 128 corners):
  collision failure 0
- 15 seeds x 6 Cartesian goals x 2 repeats = 180 IK requests: failure 0
- solution joint-limit failure 0; repeated-solve delta `0 rad`
- worst seed-to-solution joint delta `0.175789 rad`, 기존 `0.30 rad` 경계 이내

closest bounded IK 후보 선택, `>=0.30 rad` jump 거부와 deterministic
position-clamp 거부 정책은 그대로다.

## JSON 관측성

`result.gravity_compensation`은 activation, scale, torque sample 수와 cleanup
시점을 기록한다. `result.ready_reacquisition`은 시작/완료/필요 여부, 초기·최종
오차, worst error, motion sample, settle/duration과 safe-hold 결과를 기록한다.
`startup_alignment`와 `diagnostic_execution`은 각각 시작·완료 시점과 profile
position publish 수를 기록한다. 실패 JSON은 `is_partial`, refusal 단계와
`termination_reason`을 보존한다.

## 검증 결과

- 관련 handoff/adapter/documentation 회귀: 59 passed, 1 skipped
- 전체 Python: 669 passed, 4 skipped
- ROS 2 Jazzy build: 11 packages 성공
- GenericSystem: sagged start에서 gravity 680 samples, A-prime 재획득 549
  position samples, final error 0, alignment 후 translation profile 119 publish
- GenericSystem profile: 120 samples, 95.6 Hz, IK 5/5, continuity rejection 0,
  joint position clamp 0, 정상 zero-effort cleanup
- MATLAB R2026a: cleanup 전/후 분리 및 6-file bundle 검증 성공

## 남은 위험

- 143개 state-validity 표본은 A-prime 주변 연속 공간 전체의 충돌 증명이 아니다.
- fake planning scene에는 실물 주변 장애물과 동적 케이블 형상이 없다.
- GenericSystem은 gravity sag와 실제 controller tracking error를 물리적으로
  재현하지 않으므로 실물 `0.047051 rad`는 분석 입력으로만 사용했다.
- cleanup 뒤 gravity가 사라지면 다시 처질 수 있다. JSON timing으로 구간은
  분리하지만 hardware brake/hold 정책 자체를 바꾸지는 않는다.
- 이번 범위에서는 실물 재시험을 하지 않았다.
