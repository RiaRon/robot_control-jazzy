# Pose-follow MATLAB analyzer

사용법과 출력 계약은
[`../../docs/matlab-pose-follow-analysis.md`](../../docs/matlab-pose-follow-analysis.md)에
정리되어 있습니다.

진입점은 `analyze_pose_follow.m`, 독립 parser는
`read_pose_follow_json.m`입니다. 두 함수 모두 로봇 제어 코드나 ROS를 호출하지
않습니다.

현재 parser/analyzer는 deterministic JSON의 ready posture 이름·통과 여부·7관절
시작 오차와 closest-IK candidate/rejection/selection 통계, continuity cost 시계열을
정규화합니다. 분석 요약의 `comparison_metadata`로 같은 ready posture에서 시작한
실험만 묶어 비교할 수 있습니다.
