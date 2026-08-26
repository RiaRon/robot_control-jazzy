# Pose Follow handoff observability

이 디렉터리는 outer joint command law를 바꾸기 전 기준선이다. 실행 코드는
Ready A′ 완료 직후 관절을 다시 읽어 measured TCP, marker, command, 최초 IK seed,
continuity reference와 deterministic profile 원점을 한 시점에 맞춘다. 그 뒤
bounded convergence gate를 통과한 sample만 profile 성능 비교에 포함한다.

현재 outer law는 그대로다.

```text
q_cmd[k+1] = q_cmd[k] + Kp * (q_IK[k] - q_measured[k]) * dt
```

대체 law는 이 배치의 결론이 아니며 `Future work`이다. Kp/Kd, gravity scale 1.0,
TCP 선속도·각속도 한계, IK continuity 0.30 rad, closest-candidate 정책,
Cartesian intermediate target과 joint/controller 설정도 변경하지 않았다.

## 빠른 위치 안내

- [기존 handoff](diagrams/handoff_before.md)
- [measured-state handoff](diagrams/handoff_after.md)
- [측정 계층·통계 분리·profile 검증](diagrams/measurement_pipeline.md)
- [발표 figure metadata](figures/README.md)
- [2026-08-24 기준선 표](tables/README.md)
- [2026-08-26 Translation baseline 분석](2026-08-26-translation-baseline.md)
- MATLAB: [`matlab/pose_follow`](../../../matlab/pose_follow)

## Schema v2

`pose_follow_diagnostics` schema v2는 v1 필드를 보존하고 다음을 추가한다.

- `stage_trace`와 `result.timeline`: run-relative monotonic clock, sample index,
  Ready reacquisition부터 cleanup/termination까지의 event 경계
- `result.handoff_sync`, `result.convergence_gate`: 재동기화 근거, gate 임계값,
  stable sample, timeout과 safe-hold 결과
- trace마다 live/accepted/IK/command/measured TCP의 xyz와 xyzw, J1-J7의
  IK/command/next-command/measured, command/measured velocity, limiter와 gravity 상태
- `result.statistics_by_window`: 전체, Ready, handoff/alignment, gate,
  profile-only, ramp/hold/return/origin-hold 통계

자세오차는 quaternion 부호를 동일 자세로 취급하고 상대 회전각을 사용한다.
RPY 단순 차분은 사용하지 않는다. effort/current는 현 interface에서 읽을 수 없어
`unavailable`이며, 실제 값에는 joint-state/controller interface의 별도 계측이 필요하다.

## 재현 분석

```bash
/usr/local/MATLAB/R2026a/bin/matlab -batch \
  "addpath('matlab/pose_follow'); analyze_pose_follow( ...
  '/path/to/run.json','/tmp/pose-follow-analysis');"
```

분석기는 legacy schema v1과 measured-handoff schema v2를 함께 읽고 12종 figure를
각각 PNG 300 dpi, PDF, SVG, FIG로 저장한다. `summary.csv`,
`joint_summary.csv`, `analysis_summary.json`, `analysis.mat`의 수치도 함께 저장한다.
기본 비교 window는 `profile-only`이다. 원시 JSON, rosbag, HDF5와 archive는 Git에
넣지 않는다.

## 2026-08-24 기준선 해석

저장된 real translation JSON은 legacy v1이므로 새 handoff gate를 소급해
검증하는 자료가 아니다. 다만 기존 전체-run worst `48.466 mm`가 startup에서
발생했고 profile-only accepted-target worst는 `16.842 mm`(live-target worst
`17.685 mm`)였음을 분리해, startup peak를 profile 성능으로 다시 보고하지 않도록
하는 기준선이다. 새 rotation/combined 실물 수치,
실제 effort/current와 controller delay, outer-law 변경 효과는 아직 미확인이다.
