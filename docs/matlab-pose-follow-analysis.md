# pose-follow MATLAB 분석 번들

이 도구는 ChatGPT Work가 연구일지를 작성할 때 근거로 사용할 정확한 그래프,
표, 통계와 구조화된 분석 결과를 만든다. 연구일지를 직접 작성하거나
`robot_control`/ROS/실물 로봇을 호출하지 않는다. 입력 JSON은
`jsondecode(fileread(...))`로 읽기만 하며 수정하지 않는다.

## 지원 입력

- 2026-08-18 real batch의 legacy schema v1 trace
- Jazzy `8a700c0`의 deterministic diagnostics가 추가된 extended schema v1 trace
- safety refusal까지 승인된 trace와 종료 원인을 담은 partial/refused schema v1
- 같은 필드를 유지하는 이후 JSON

두 형식이 모두 `schema_version: 1`이므로 parser는 버전 숫자만 추측하지 않는다.
`diagnostic_profile`, signed projection, IK timing, IK target jump 필드의 존재 여부로
기능을 판별한다.

partial/refused 파일은 정상 파일과 같은 trace 배열을 사용하므로 refusal 전까지의
phase 통계와 그림을 그대로 생성한다. `summary.csv`,
`analysis_summary.json`, `analysis.mat`에는 `termination`, `is_partial`,
refusal reason/sequence/phase와 IK continuity rejected/retry/exhausted 수를
추가한다. `ik_events.png`에도 refused outcome을 별도 색으로 표시한다. 거부가
첫 sample 전에 발생해 trace가 비어 있어도 refusal metadata는 읽는다.

legacy JSON에는 profile phase, IK event별 latency와 jump event가 없다. 이 경우:

- 모든 sample의 phase를 `unlabeled`로 보존한다.
- signed projection은 저장된 live marker, accepted marker, IK target, command,
  measured TCP 좌표에서 현재 serializer와 같은 식으로 재계산한다.
- IK accepted/failed/superseded 총계는 읽되 event latency와 jump 통계는
  `NaN`(CSV) 또는 `null`(JSON)로 표시한다. 기록되지 않은 값을 0으로 꾸미지 않는다.

deterministic profile 이름은 다음 네 canonical phase로 정규화한다.

| JSON phase suffix | 분석 phase |
| --- | --- |
| `*_ramp_out` | `ramp` |
| `*_hold` | `hold` |
| `*_ramp_back` | `return` |
| `origin_hold` | `origin-hold` |

## MATLAB 요구사항

- MATLAB R2021b 이상(검증 버전: R2026a)
- 추가 Toolbox 없음

`prctile`이나 Statistics and Machine Learning Toolbox에 의존하지 않고 p95를
선형 보간으로 계산한다. PDF도 MATLAB Report Generator가 아니라 base MATLAB의
`exportgraphics`로 만든다. MATLAB이 설치되지 않은 환경에서는 이 폴더가
Python package와 ROS/colcon 대상에 들어가지 않으므로 기존 테스트와 빌드에
영향을 주지 않는다.

## 실행

저장소 루트에서 MATLAB을 실행한다. 결과 디렉터리는 원본과 분리된 `/tmp` 또는
실험 데이터 저장소 아래를 권장한다.

```matlab
addpath('matlab/pose_follow');

files = [
    "/data/2026-08-18/right-follow-kp2-slow.json"
    "/data/new/right-translation.json"
    "/data/new/right-rotation.json"
];
names = ["manual-slow"; "translation-10mm"; "rotation-5deg"];

analysis = analyze_pose_follow( ...
    files, "/tmp/openarm-follow-analysis/baseline", ...
    'ExperimentNames', names);
```

GUI 없이 실행하려면:

```bash
matlab -batch "addpath('matlab/pose_follow'); analyze_pose_follow( ...
  [\"/data/run-a.json\";\"/data/run-b.json\"], ...
  \"/tmp/openarm-follow-analysis/compare-a-b\", ...
  'ExperimentNames',[\"run-a\";\"run-b\"]);"
```

그래프 창도 보고 싶으면 `'Visible','on'`, PDF가 필요 없는 빠른 반복 분석은
`'CreatePDF',false`를 추가한다. 기본값은 headless(`Visible='off'`)이며 PDF를
생성한다.

## analysis bundle

지정한 output 폴더에 다음 파일을 고정 이름으로 생성한다.

| 파일 | 내용 |
| --- | --- |
| `summary.csv` | 실험×phase 비교표. TCP 위치/자세, ready 이름·통과·시작 오차, IK candidate/rejection/selection cost·latency, partial/refusal 통계 |
| `analysis_summary.json` | Work가 읽기 쉬운 metadata, layer 통계, summary row와 색상 규칙 |
| `analysis.mat` | 정규화 time series, summary table과 구조화 분석 전체 |
| `tcp_error_timeseries.png` | TCP 위치(mm)·자세(deg) 오차와 phase band |
| `error_layers.png` | 레이어별 거리와 live-error 방향 signed projection(mm) |
| `joint_tracking.png` | J1-J7 중 최대 target-command와 command-measured 오차(rad) |
| `j1_j4_j7_detail.png` | J1/J4/J7 target-command-measured 및 IK target jump 시각 |
| `ik_events.png` | request latency(ms), weighted continuity cost 시계열, accepted/failed/superseded/refused 수 |
| `phase_comparison.png` | canonical phase별 TCP RMS/p95 실험 비교 |
| `research_report.pdf` | 요약 표지와 위 6개 그림을 묶은 7-page 연구 보고서 |

각 time-series 그림은 실험명, elapsed-time 축, 물리 단위, 범례와 profile phase
색상 band를 표시한다. 한 bundle 안에서는 모든 실험이 같은 time/y 축 범위를
사용한다. 실험 색은 입력 순서, layer/phase/state 색은 고정 규칙이다. 서로 다른
bundle도 같은 실험 순서로 호출하면 실험 색이 유지된다.

`summary.csv`의 `phase=all`은 전체 실험, 나머지는 phase별 행이다. legacy
실험은 canonical phase 행의 sample 수가 0이고 실제 데이터는 `unlabeled` 행에
있다. `analysis.mat`은 다음처럼 읽는다.

```matlab
loaded = load('/tmp/openarm-follow-analysis/baseline/analysis.mat');
summary = loaded.summaryTable;
run = loaded.analysis.experiments(1);
```

## parser 검증

원본 데이터는 저장소 밖에 둔 채 다음 검증 함수를 사용할 수 있다.

```matlab
addpath('matlab/pose_follow/tests');
validate_pose_follow_parser( ...
    '/data/2026-08-18/right-follow-kp2-slow.json', ...
    '/data/current/fake-diagnostic.json', ...
    '/tmp/openarm-follow-analysis/parser-validation', ...
    '/data/current/partial-refused.json');
```

네 번째 인자는 선택사항이다. 이 검증은 legacy signed projection 재구성,
extended phase 네 종류, partial refusal sequence/reason/phase, 통합 parsing과
전체 bundle 파일을 확인한다.

## Git 및 데이터 보존

- 원시 pose-follow JSON을 저장소로 복사하거나 커밋하지 않는다.
- output 폴더도 저장소 밖에 만든다. 부득이하게 이 도구 폴더 아래에서 실행해도
  로컬 `.gitignore`가 JSON/CSV/MAT/PNG/PDF와 일반 output 폴더를 제외한다.
- `git status --short`로 원시 데이터와 생성물이 staging되지 않았는지 확인한다.
- 원본 JSON을 보존하고 실험명·원본 경로·생성 시각은 bundle metadata로 추적한다.


## ready 기준 비교

새 schema v1 확장 파일은 `run.ready_posture`에 이름, target 7관절, actual start,
관절별 start error와 pass를 정규화한다. `analysis_summary.json`의
`comparison_metadata.ready_posture_name`으로 같은 ready 자세에서 수행한 실험만
묶어 비교한다. `run.ik.selection_events`와 `run.continuity_cost`는 후보 수,
거부 수, 선택 candidate/cost, solve/batch latency와 cost 시계열을 보존한다.
legacy 파일에는 해당 값이 `NaN`/빈 값으로 남는다.

## ready 도달·중력 비교 분석

`read_ready_json.m`과 `analyze_ready_comparison.m`은 pose-ready before/after JSON을
읽어 target/reference/feedback, 7관절 오차, max/RMS error, posture 이름,
gravity 활성/scale/torque와 settle 종료를 정규화한다. legacy snapshot에 새 필드가
없으면 호출자가 target·posture·gravity 여부를 제공하며 기록되지 않은 reference나
torque는 `NaN`으로 보존한다.

```matlab
addpath('matlab/pose_follow');
analysis = analyze_ready_comparison( ...
    ["/data/d-no-gravity.json"; "/data/aprime-no-gravity.json"; ...
     "/data/aprime-gravity.json"], ...
    "/data/analysis/ready-compare", ...
    'ExperimentNames', ["D-no-gravity"; "Aprime-no-gravity"; ...
                        "Aprime-gravity"]);
```

출력은 `ready_summary.csv`, `ready_joint_errors.csv`,
`ready_group_comparison.csv`, `ready_analysis_summary.json`,
`ready_analysis.mat`, `ready_target_error.png`이다. group key는
`posture_name|gravity=<0|1>`이므로 같은 target에서 중력보상 유무에 따른 도달
오차를 직접 비교한다. 원시 JSON과 이 생성물은 저장소 밖, 예를 들어
`/home/user/openarm_follow_data/.../analysis`에 둔다.
