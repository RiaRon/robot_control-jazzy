# Figure metadata

생성 figure는 원시 데이터가 아니라 발표·검토용 파생물이다. 원시 JSON은 Git에
추가하지 않는다.

| 항목 | 2026-08-24 baseline |
| --- | --- |
| source JSON | `right-follow-ready-handoff-translation.json` |
| date | 2026-08-24 |
| Git commit | runtime `b119ff2`; MATLAB analyzer `9fbd2e8`; source run `34ddb17` 계열 |
| profile | translation-only |
| gravity | enabled, scale 1.0 |
| analysis window | startup 0–9.566 s; profile-only 9.576–19.456 s |
| mode | real stored-data replay; 분석 중 robot/CAN 미접근 |
| MATLAB | R2026a |
| command | `analyze_pose_follow(source, output, 'ExperimentNames','real-2026-08-24-translation')` |

새 schema fake replay는 translation, rotation, translation-rotation을 같은 gate와
phase 구조로 생성하며 source filename, profile과 전체 run-relative window를 각
figure title에 표시한다. 각 figure의 수직 점선은 phase 경계이고, 선 스타일과
marker를 함께 사용해 색상만으로 상태를 구분하지 않는다.

## 2026-08-24 tracked SVG

1. [TCP translation layers](01_tcp_translation_layers.svg)
2. [TCP orientation layers](02_tcp_orientation_layers.svg)
3. [Translation decomposition](03_translation_error_decomposition.svg)
4. [Orientation decomposition](04_orientation_error_decomposition.svg)
5. [J1-J7 positions](05_joint_positions.svg)
6. [J1-J7 velocities](06_joint_velocities.svg)
7. [Handoff/startup zoom](07_handoff_convergence_zoom.svg)
8. [Hold lead and overshoot](08_hold_command_lead_overshoot.svg)
9. [Limiter overlay](09_limiter_activation.svg)
10. [Phase statistics](10_phase_statistics.svg)
11. [Joint maximum heatmap](11_joint_max_error_heatmap.svg)
12. [Profile comparison view](12_profile_comparison.svg)

이 실물 legacy 파일에는 새 convergence stage가 없으므로 7번은 기존 startup 구간을
확대한 기준선이다. 새 gate를 소급해 존재한 것처럼 표시하지 않는다. 전체 네 형식
bundle은 Git 제외 `artifacts/pose_follow_matlab_validation/real_final_v2/`에 생성해
검증했다.
