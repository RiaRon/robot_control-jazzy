# 2026-08-26 OpenArm Translation baseline

이 문서는 OpenArm 오른팔의 schema v2 Translation diagnostic profile 실물 데이터
2개를 MATLAB R2026a Update 4로 재생 분석한 결과다. 제어기, outer command law,
gain, gravity compensation과 원본 JSON은 변경하지 않았다. 분석 중 robot/CAN에는
접근하지 않았다.

## 입력과 재현 범위

두 run의 조건은 translation 0.01 m, 0.005 m/s, hold 3 s, 왕복 1회,
100 Hz, kp_per_sec 2.0, gravity scale 1.0, 최대 TCP 선속도 0.02 m/s,
최대 TCP 각속도 0.10 rad/s다.

| run | source | SHA-256 |
| --- | --- | --- |
| Run 1 | right-follow-baseline-translation-run1.json | bfefc4b64f7651f90e468e10baec420d63c67e98cde41dd77d82b0e96e38e274 |
| Run 2 | right-follow-baseline-translation-run2.json | 667780d285f243b53417d8aefb3fa78536053a7af478b391ab099abed045e596 |

원시 JSON과 archive는 저장소 정책에 따라 Git에 포함하지 않는다. 저장된 표와
figure는 원본을 수정하지 않고 read_pose_follow_json과 analyze_pose_follow을
우선 실행한 뒤, accepted-target 방향 projection을 MATLAB에서 별도로 재계산해
만들었다.

~~~matlab
addpath('matlab/pose_follow');
files = {
    '/path/to/right-follow-baseline-translation-run1.json'
    '/path/to/right-follow-baseline-translation-run2.json'
};
analysis = analyze_pose_follow( ...
    files, '/path/to/output', ...
    'ExperimentNames', ["Run 1"; "Run 2"], ...
    'Visible', 'off', ...
    'CreatePDF', true);
~~~

기본 비교 window는 startup과 convergence gate를 제외한 profile-only다.

| phase | Run 1 | Run 2 |
| --- | ---: | ---: |
| profile-only | 991 samples, 9.991 s | 992 samples, 9.997 s |
| ramp | 198, 1.989 s | 198, 1.984 s |
| hold | 297, 2.984 s | 298, 2.993 s |
| return | 198, 1.990 s | 198, 1.990 s |
| origin-hold | 298, 2.998 s | 298, 2.999 s |

## Accepted-target 계산

Deterministic profile의 목표는 live marker가 아니라 accepted marker다.

~~~matlab
e = accepted_marker_position - measured_position;
if norm(e) > 1e-12
    u = e / norm(e);
else
    u = zeros(1, 3);
end

p1 = dot(accepted_marker_position - ik_target_position, u);
p2 = dot(ik_target_position - command_position, u);
p3 = dot(command_position - measured_position, u);
~~~

각 샘플에서 norm(e) = p1 + p2 + p3를 확인했다. Profile-only decomposition의
최대 절대 수치 잔차는 Run 1 1.65e-11 um, Run 2 6.94e-12 um였다.
JSON에 기록된 live-marker 방향 signed projection은 이 계산에 사용하지 않았다.

## Profile-only 결과

### TCP 추종

| metric | Run 1 | Run 2 | Run 2 - Run 1 |
| --- | ---: | ---: | ---: |
| position mean | 9.679 mm | 5.917 mm | -3.762 mm (-38.9%) |
| position RMS | 12.059 mm | 7.229 mm | -4.830 mm (-40.1%) |
| position P95 | 20.666 mm | 13.356 mm | -7.311 mm (-35.4%) |
| position maximum | 21.403 mm | 13.647 mm | -7.756 mm (-36.2%) |
| position final | 17.904 mm | 12.535 mm | -5.368 mm (-30.0%) |
| samples within 2 mm | 7.770% | 5.141% | -2.629 percentage points |
| orientation mean | 1.169 deg | 0.731 deg | -37.4% |
| orientation RMS | 1.523 deg | 0.873 deg | -42.6% |
| orientation maximum | 2.710 deg | 1.664 deg | -38.6% |
| orientation final | 2.297 deg | 1.315 deg | -42.8% |

Run 2는 큰 오차 분포는 줄었지만 2 mm 이내 비율은 더 낮다. 따라서 mean/RMS
개선을 정밀 수렴으로 해석하지 않는다. 보조 지표인 live-marker 기준
mean/RMS/max/final은 Run 1 8.876/11.031/19.630/16.147 mm, Run 2
6.076/7.085/13.073/12.535 mm다.

### Phase 차이

| run/phase | mean | RMS | maximum | final | linear limiter |
| --- | ---: | ---: | ---: | ---: | ---: |
| R1 ramp | 3.476 | 4.175 | 9.067 | 8.146 mm | 0/198 |
| R1 hold | 5.560 | 5.594 | 8.495 | 5.100 mm | 0/297 |
| R1 return | 6.329 | 7.268 | 20.889 | 20.889 mm | 19/198 |
| R1 origin-hold | 20.131 | 20.144 | 21.403 | 17.904 mm | 298/298 |
| R2 ramp | 3.922 | 4.701 | 8.552 | 1.802 mm | 0/198 |
| R2 hold | 6.609 | 8.368 | 13.647 | 2.319 mm | 110/298 |
| R2 return | 3.706 | 3.952 | 7.965 | 3.430 mm | 0/198 |
| R2 origin-hold | 8.018 | 8.881 | 13.073 | 12.535 mm | 119/298 |

Run 2의 hold mean과 maximum은 각각 18.9%, 60.7% 크지만 hold final은
54.5% 작다. 가장 큰 run 간 차이는 return과 origin-hold다. Run 1은 return
끝에서 오차가 급증한 뒤 origin-hold 전체에서 큰 오차를 유지했다.

## Cartesian additive layers

Profile-only vector norm의 mean/RMS/max/final은 다음과 같다.

| run | layer | norm, mm |
| --- | --- | ---: |
| R1 | accepted marker to IK | 0.004/0.006/0.010/0.010 |
| R1 | IK to command | 20.469/25.174/47.578/19.988 |
| R1 | command to measured | 22.554/27.075/53.807/36.979 |
| R2 | accepted marker to IK | 0.009/0.009/0.010/0.009 |
| R2 | IK to command | 12.123/13.603/23.860/8.330 |
| R2 | command to measured | 14.293/15.615/27.416/17.549 |

Accepted marker to IK 오차는 두 run 모두 최대 약 0.01 mm다. Run 1의 signed
mean은 accepted-to-IK -0.002, IK-to-command -7.673,
command-to-measured +17.353 mm이며 두 큰 계층이 부분 상쇄된다. Run 2는
-0.001/-1.228/+7.145 mm다.

Run 1 hold와 return의 IK-to-command/command-to-measured signed mean은 각각
-18.412/+23.972 mm, -28.205/+34.535 mm다. Origin-hold 평균에서는
+11.622/+8.513 mm로 부호 관계가 바뀐다. 고정 방향의 단순 추종 지연만으로는
이 phase 전환 동작을 설명하기 어렵다.

## Joint layers

Profile-only mean absolute error는 다음과 같다. 단위는 mrad다.

| run/layer | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| R1 IK to command | 17.77 | 3.72 | 13.03 | 31.80 | 15.36 | 3.46 | 16.49 |
| R1 command to measured | 19.83 | 3.84 | 13.32 | 38.71 | 15.84 | 3.50 | 17.67 |
| R1 IK to measured | 12.11 | 0.49 | 1.98 | 16.07 | 1.84 | 0.58 | 1.46 |
| R2 IK to command | 23.98 | 2.75 | 12.53 | 26.00 | 12.44 | 7.10 | 0.47 |
| R2 command to measured | 27.43 | 2.61 | 13.15 | 31.07 | 13.96 | 7.64 | 0.51 |
| R2 IK to measured | 12.88 | 0.70 | 1.73 | 12.18 | 4.01 | 0.79 | 0.15 |

J4가 두 run의 IK-to-command와 command-to-measured에서 가장 크고 J1이
반복적으로 그 다음이다. J7은 Run 1과 Run 2 사이에서 약 97% 달라져 반복적인
기여로 볼 수 없다. J1과 J4의 두 내부 계층은 ramp, hold, return,
origin-hold 사이에서 반복적으로 부호가 바뀐다.

## IK와 limiter

| metric | Run 1 | Run 2 |
| --- | ---: | ---: |
| IK submitted/succeeded | 9/9 | 10/10 |
| failed/superseded | 0/0 | 0/0 |
| candidates/rejected | 36/0 | 40/0 |
| continuity reject/retry/exhausted | 0/0/0 | 0/0/0 |
| recorded target jump events | 0 | 0 |
| normal sample-to-sample target maximum | 0.01438 rad | 0.01368 rad |
| Cartesian linear limiter | 317/991 (31.99%) | 229/992 (23.08%) |
| Cartesian angular limiter | 4/991 | 0 |
| joint velocity/lead/position limiter | 0/0/0 | 0/0/0 |

두 최대오차 시점 모두 Cartesian linear limiter가 활성화돼 있었다. Run 1은
profile-relative 7.0128 s origin-hold에서 21.403 mm였고 layer는
-0.004/+40.902/-19.495 mm였다. Run 2는 2.2862 s hold에서 13.647 mm였고
+0.004/+3.721/+9.922 mm였다.

## 해석과 한계

IK는 모두 성공했고 continuity reject와 기록된 jump가 없으며 accepted-to-IK
오차도 0.01 mm 이하이므로 IK 또는 deterministic 목표 생성은 주원인으로
지지되지 않는다.

현재 outer law는 measured error를 command에 누적한다.

~~~text
q_cmd[k+1] = q_cmd[k] + Kp * (q_IK[k] - q_measured[k]) * dt
~~~

큰 IK-to-command layer, phase별 부호 반전과 command가 IK target을 지나간
상태는 이 구조에서 생길 수 있는 command backlog와 windup 유사 동작에
부합한다. 동시에 command-to-measured norm은 두 run 모두 평균적으로 가장 큰
직접 계층이며 J4와 J1의 관절 오차도 반복적으로 크다. 따라서 profile 평균은
실제 command-to-measured 추종 오차가 직접 지배하고, outer backlog가 이를
상쇄하거나 phase 전환에서 증폭하는 상호작용으로 해석한다.

표본이 두 개뿐이고 최대오차 phase와 J7 기여가 run 사이에서 다르다. Limiter와
오차의 동시성도 인과관계를 증명하지 않는다. 물리 추종 오차를 motor impedance,
마찰, calibration 또는 다른 하위 요인으로 분해할 수 없으며, 이 결과만으로
일반적인 제어 성능을 확정하지 않는다.

## Tracked outputs

- [MATLAB comparison tables](tables/2026-08-26-translation/README.md)
- [seven review figures](figures/2026-08-26-translation/README.md)

전체 PNG/PDF/FIG/MAT bundle과 원시 데이터는 Git 외부 분석 artifact에 보존한다.
