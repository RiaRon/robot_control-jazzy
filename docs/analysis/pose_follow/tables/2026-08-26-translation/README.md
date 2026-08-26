# 2026-08-26 Translation baseline tables

MATLAB R2026a로 계산한 schema v2 실물 Translation 두 run의 파생 요약이다.
원시 JSON과 archive는 포함하지 않는다.

- [profile-only run comparison](run_comparison.csv)
- [profile-only absolute and percentage difference](run_difference.csv)
- [phase comparison](phase_comparison.csv)
- [phase absolute and percentage difference](phase_difference.csv)
- [accepted-target Cartesian layer statistics](accepted_layer_statistics.csv)
- [J1-J7 layer statistics](joint_layer_statistics.csv)
- [Cartesian and joint limiter statistics](limiter_statistics.csv)
- [maximum-error layer decomposition](maximum_error_decomposition.csv)
- [IK and decomposition checks](ik_statistics.csv)

Profile-only가 주 비교 window다. Signed Cartesian projection은 JSON에 저장된
live-marker 방향이 아니라 accepted-marker-to-measured 방향으로 재계산했다.
