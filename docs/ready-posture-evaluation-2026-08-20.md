# 오른팔 ready posture 재평가 — 2026-08-20

기준은 최신 `jazzy@df06a9d`의 runtime URDF, SRDF allowed-collision matrix,
MoveIt KDL IK와 `mock_components/GenericSystem`이다. 외부 collision object가 없는
fake planning scene 결과이며 실물 OpenArm/CAN은 사용하지 않았다. 첨부 원본
`openarm-ready-comparison-2026-08-20.tar.gz`의 SHA-256은
`722095bb0040a30b38a68d72a63a4f04541fd138dadccde4b3254944d02b6a3e`이고 원시
JSON·로그·생성 분석물은 Git에 넣지 않았다.

비교 시작값은 A′ 실물 시험 직전 measured 7관절이다. 이동 경로는 기존과 같은
J4-first minimum-jerk 두 구간이며 gravity는 runtime URDF에서 매 sample 계산했다.

| 후보 | target (rad) | 시작 이동 max/L2/L1 (rad) | self/world collision | 최소 limit 여유 (rad) | rank | sigma min | cond | manipulability | target gravity norm (N·m) | path gravity mean/max (N·m) | TCP xyz (m) |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A | `[0,0,0,0.6,0,0,0]` | `0.610/0.612/0.675` | 없음/없음 | 0.175 | 6 | 0.05245 | 35.47 | 0.01581 | 3.716 | `2.797/3.716` | `[0.2237,-0.1535,0.1511]` |
| A′ | `[0,0.2,0,0.6,0,0,0]` | `0.610/0.640/0.854` | 없음/없음 | 0.375 | 6 | 0.05218 | 35.58 | 0.01549 | 4.371 | `2.921/4.371` | `[0.2237,-0.2622,0.1620]` |
| D | `[0.15,0.55,0.15,0.8,-0.1,0.15,0.1]` | `0.810/1.012/1.953` | 없음/없음 | 0.635 | 6 | 0.06517 | 28.54 | 0.01611 | 8.437 | `4.454/8.437` | `[0.3506,-0.4193,0.3428]` |

관절별 lower/upper 여유(rad)는 A
`[1.396/3.491,0.175/3.316,1.571/1.571,0.600/1.843,1.571/1.571,0.785/0.785,1.571/1.571]`,
A′ `[1.396/3.491,0.375/3.116,1.571/1.571,0.600/1.843,1.571/1.571,0.785/0.785,1.571/1.571]`,
D `[1.546/3.341,0.725/2.766,1.721/1.421,0.800/1.643,1.471/1.671,0.935/0.635,1.671/1.471]`이다.

주변 작업공간은 직전 accepted seed에 대한 single-step `<0.30 rad`를 유지하며
방향별 translation 50 mm와 rotation 15도까지 탐색했다. A와 A′는
world `x-/x+/y-/y+/z-/z+ = 50/50/50/50/20/50 mm`, local
`15/15/15/12.5/15/15 deg`이고 D는 world `50/50/50/50/45/50 mm`, local
여섯 방향 모두 15도다.

world x/y/z 10 mm 및 local x/y/z 5도 왕복을 각 10회 다시 계산한 결과는 모두
`60/60` 성공했고 축마다 동일한 관절 궤적 1종만 나왔다.

| 후보 | world x/y/z 최대 single-step (rad) | local x/y/z 최대 single-step (rad) | 전체 최대 |
| --- | --- | --- | ---: |
| A | `0.0227 / 0.0115 / 0.0743` | `0.0597 / 0.0801 / 0.0238` | 0.0801 |
| A′ | `0.0232 / 0.0147 / 0.0729` | `0.0597 / 0.0801 / 0.0238` | 0.0801 |
| D | `0.0296 / 0.0313 / 0.0407` | `0.0571 / 0.0835 / 0.0253` | 0.0835 |

중력보상 없이 수행한 실물 비교에서 D는 target-reference 전달은 정확했지만
5초 뒤 worst feedback error가 `0.2309 rad`, A′는 `0.1443 rad`로 둘 다
0.020 rad settle 기준을 통과하지 못했다. 이상 소음·진동은 없었다. A′는 D보다
실측 오차가 37.5% 작고, 시작 이동 L1은 56.3%, 경로 peak gravity는 48.2% 작다.
D가 Jacobian과 작업공간에서는 우수하지만 A′도 rank 6, 60/60 continuity 및
0.375 rad 최소 여유를 확보한다.

따라서 운영자 결정에 따라 A′를 표준
`openarm_right_ready_v2 = [0,0.2,0,0.6,0,0,0] rad`로 승격한다. D는 비교용
legacy `openarm_right_ready_v1`으로 보존한다. GenericSystem에서 v2와 controller-rate
gravity scale 1.0 조합은 worst `0.0000 rad`, settle `0.52 s`였으며, 첫 실물
중력보상 결과는 아직 없는 상태임을 데이터 해석에서 구분한다.
