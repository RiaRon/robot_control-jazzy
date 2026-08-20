# Pose-follow sequence-6 IK continuity refusal — 2026-08-20

## 실물 관측

기준은 `jazzy@0673903`, 오른팔 deterministic world-x translation 1회다.
rotation은 실행하지 않았다.

```text
pre-run J4: +0.021553368 rad (PASS)
startup alignment: completed
samples/rate: 351 / 99.0 Hz
IK: 6/6 solver success, failed 0, superseded 0
safe trace live TCP position: mean 2.5 mm, worst 8.3 mm
safe trace orientation: mean/worst 0.2 deg
Cartesian linear/angular limits: 0
joint clamps: 0
refused accepted sequence: 6
joint delta:
  J1 +3.1559 rad
  J2 +3.1269 rad
  J3 +1.5527 rad
  J5 +1.5889 rad
post-run J4: +0.0216 rad
CAN FD: ERROR-ACTIVE, tx/rx error counters 0
abnormal noise/vibration: none
```

sequence 6의 MoveIt solve는 성공했지만 직전 관절해와 연속적이지 않았다. J1/J2는
약 pi, J3/J5는 약 pi/2인 다른 7-DOF branch다. `0673903`의 `>= 0.30 rad`
하드 경계가 해당 target을 첫 publish 전에 차단했으며, 그 이전 command tracking과
limit 결과는 정상이다.

## 전달 파일 상태

`/home/cbj4/openarm_follow_data/2026-08-20/openarm-ik-refusal-2026-08-20/`
아래 pose JSON과 log 9개는 모두 0바이트였고 새 압축파일은 전달되지 않았다.
따라서 351-sample 원시 trace를 다시 계산할 수 없으며 위 값은 사용자가 보존한
terminal 결과를 근거로 한다. 0바이트 파일이나 생성 분석물은 Git에 포함하지
않는다.

## 변경한 continuity 정책

- `0.30 rad` 하드 경계와 `>=` 비교는 변경하지 않고 CLI로 완화하지 않는다.
- 첫 request는 startup measured state, 이후 request는 직전 연속 accepted target을
  seed와 비교 기준으로 사용한다.
- 불연속 solution은 worker target으로 승격하지 않고 controller에 publish하지
  않는다. retry 중에는 직전 accepted target을 유지한다.
- 같은 Cartesian 목표를 같은 연속 seed로 최대 4회(최초 1 + retry 3) 푼다.
- 제한 횟수 안에 모든 관절 delta가 `0.30 rad` 미만인 해만 accepted 처리한다.
- 연속 해를 찾지 못하면 마지막 승인 자세를 유지한 채 safety refusal로 종료한다.
- main loop에도 기존 hard-boundary 검사를 방어 계층으로 남긴다.

## Partial/refused JSON

`--output`을 지정한 safety refusal은 종료 전에 지금까지의 승인 trace를 같은
schema v1 JSON으로 원자 저장한다.

```text
result.termination = "safety_refused"
result.is_partial = true
result.refusal = {
  reason, message, refused_sequence, profile_phase,
  attempts, max_attempts, reference_sequence,
  joint_delta_rad[7], triggered_joints[]
}
result.ik = {
  submitted, succeeded, failed, superseded, solve_attempts,
  continuity_rejected, continuity_retries, continuity_exhausted,
  continuity_events[]
}
```

거부된 sequence는 trace에 추가되지 않는다. output 저장 후에도 CLI는 기존처럼
exit 3과 `refused:` 메시지를 반환한다.

## 개발 PC 검증

실물/CAN은 사용하지 않았다.

- 합성 sequence-6 replay: J1/J2 `+3.1559/+3.1269 rad`,
  J3/J5 `+1.5527/+1.5889 rad`; 5개 연속 target 뒤 4개 bad solve를 모두 차단,
  sequence 6 publish 0건, partial JSON 저장, exit 3
- worker 단위 검증: bad branch 다음 retry에서 연속 해가 나오면 수락; 4회
  소진하면 이전 target/sequence 유지; retry solver error도 bounded exhaustion
- MATLAB R2026a: 2026-08-18 legacy real, full deterministic fake,
  partial/refused fake를 한 bundle로 분석; CSV/JSON/MAT, PNG 6개, 7-page PDF 성공
- ROS MoveIt + `mock_components/GenericSystem`: 비특이 mock seed
  `[0.2, 0.4, 0.1, 0.5, -0.1, 0.2, 0.1]`에서 10 mm, 5 mm/s translation 왕복
  완료; 408 samples, 99.0 Hz, IK 9/9, failed/superseded 0,
  continuity rejection 0, joint clamp 0
- mock target transition 8개의 관절별 worst jump는 최대 J4
  `0.01815 rad`로 하드 경계보다 충분히 작았다.
- 집중 회귀: `19 passed`; pose/pose-follow CLI: `67 passed, 8 deselected`
- 전체 Python: `647 passed, 4 skipped`
- ROS 2 Jazzy 빌드: 11개 패키지 성공

이 개발 배치에서는 추가 실물 재시험이나 rotation 실행을 요구하지 않는다.
