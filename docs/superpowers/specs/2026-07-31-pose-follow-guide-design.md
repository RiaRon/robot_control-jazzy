# `pose follow` 전용 가이드 설계

## 목적

`robotctl pose follow`를 처음 사용하는 운영자가 다른 문서를 오가지 않고
RViz에서 실물 OpenArm의 TCP를 안전하게 조작할 수 있는 한국어 실행 가이드를
제공한다.

## 산출물

- 경로: `docs/pose-follow.md`
- 언어: 한국어
- 형식: 설치·설정 레퍼런스가 아니라 실제 작업 순서 중심의 독립 실행 가이드
- 대상: Jazzy 작업 트리에서 RViz와 실물 양팔 브링업까지 완료하려는 운영자

## 구성

1. `pose follow`가 RViz TCP 마커를 100 Hz로 추종한다는 기능 설명
2. 터미널 환경 준비와 `robotctl` 실행 준비
3. CAN FD 및 실물 브링업
4. RViz의 MotionPlanning 그룹과 Interact 도구 설정
5. 안전한 60초 시험 운전
6. `--seconds inf`를 사용한 무기한 운전과 `Ctrl+C` 종료
7. 선택적인 `--gravity 0.75` 사용법과 gravity가 이동 명령이 아니라는 설명
8. 왼팔 명령 및 주요 옵션 표
9. 실행 종료 보고서 해석
10. 마커가 안 움직이거나 팔이 안 따라오는 경우의 진단 순서
11. 영점 자세 Jacobian 특이점과 팔꿈치를 먼저 굽히는 회피 절차
12. E-stop, 작은 이동, CAN 좌우 확인 등 실물 안전 체크리스트

## 안전 기본값

- 첫 실행 예제는 `--seconds 60`과 gravity 미사용으로 한다.
- `--seconds inf`는 정상 추종을 확인한 뒤 사용하는 고급 운용으로 구분한다.
- 움직이는 모든 명령에는 `--execute`의 효과를 명확히 표시한다.
- 무기한 운전은 운영자가 자리를 비운 상태로 두지 않도록 경고한다.
- 종료 시 trajectory controller가 마지막 명령 위치를 유지하고 gravity
  feedforward는 0으로 돌아간다는 동작을 설명한다.

## 기술적 정확성

- 명령과 기본값은 현재 `robot_control.cli` 구현을 기준으로 한다.
- `pose ee --from-marker`와 `pose follow`의 차이를 짧게 구분한다.
- RViz의 기본 Plan & Execute가 이 브랜치의 제어 경로가 아님을 명시한다.
- gravity controller 미로딩, 잘못된 planning group, 특이점, 위치·속도·lead
  clamp를 서로 다른 증상으로 구분한다.

## 검증

- 문서의 모든 셸 명령을 현재 CLI의 `--help` 및 구현과 대조한다.
- 문서 내 링크와 파일 경로가 작업 트리에서 유효한지 확인한다.
- `TBD`나 구현되지 않은 옵션을 포함하지 않는다.
