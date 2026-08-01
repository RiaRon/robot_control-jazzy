# Pose Follow Operator Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a standalone Korean operator guide for controlling a real OpenArm TCP from the RViz marker with `robotctl pose follow`.

**Architecture:** Add one task-oriented Markdown file under `docs/`. Derive every command, default, and behavior from the current CLI implementation and link to the comprehensive CLI reference for deeper details.

**Tech Stack:** Markdown, Bash command examples, ROS 2 Jazzy, MoveIt RViz MotionPlanning, `robotctl`

## Global Constraints

- Create `docs/pose-follow.md` in Korean.
- Make the guide independently usable from environment preparation through shutdown and troubleshooting.
- Use a 60-second run without gravity as the safe first example.
- Present `--seconds inf` only after the first bounded run succeeds.
- Explain that gravity feedforward does not itself command motion.
- Clearly mark every command that can move physical hardware.
- Do not document options or behavior absent from the current implementation.

---

### Task 1: Write and verify the standalone operator guide

**Files:**
- Create: `docs/pose-follow.md`
- Reference: `README.md:50`
- Reference: `docs/cli.md:515`
- Reference: `src/robot_control/cli.py:365`
- Reference: `src/robot_control/cli.py:905`
- Reference: `src/robot_control/ros_adapter.py:726`

**Interfaces:**
- Consumes: CLI flags `--profile`, `--group`, `--gravity`, `--seconds`, and `--execute`
- Produces: A standalone Korean procedure for bounded and unbounded RViz TCP following

- [ ] **Step 1: Create the guide with the approved operating sequence**

Create `docs/pose-follow.md` with these concrete sections:

```markdown
# RViz에서 `robotctl pose follow` 사용하기

## 기능과 제어 경로
## 시작 전 안전 확인
## 1. 터미널 환경 준비
## 2. CAN FD와 실물 브링업
## 3. RViz 설정
## 4. 60초 시험 운전
## 5. 무기한 운전
## 6. 중력 보상
## 7. 왼팔 제어
## 8. 종료 보고서 읽기
## 9. 문제 해결
## 명령 요약
```

Include the exact bounded first-run command:

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds 60 \
  --execute
```

Include the exact indefinite command, with explicit `Ctrl+C` shutdown guidance:

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds inf \
  --execute
```

Include gravity only as an optional combined mode:

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --seconds inf \
  --gravity 0.75 \
  --execute
```

Explain the following failure cases separately:

- `robotctl: command not found`: source ROS and overlay, append to `PYTHONPATH`, and define the module alias.
- No right-arm marker: set MotionPlanning to `right_arm`, not `openarm_right_arm`.
- Marker cannot be dragged: select RViz `Interact`.
- Marker moves but the arm does not: require `--execute`; move away from the all-zero singular pose.
- Gravity mode reports no subscriber: load the effort controllers.
- Wrong physical arm moves: stop and swap `--right-can`/`--left-can` assignments.
- `velocity limit`, `lead limit`, and `position limit`: explain the corresponding clamp condition.

- [ ] **Step 2: Check all documented CLI flags against the executable**

Run:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"
python3 -m robot_control.cli pose follow --help
```

Expected: help lists `--profile`, `--group`, `--gravity`, `--seconds`, and `--execute`; every flag used in the guide appears.

- [ ] **Step 3: Check Markdown paths, placeholders, and whitespace**

Run:

```bash
test -f docs/pose-follow.md
rg -n 'TBD|TODO|FIXME' docs/pose-follow.md
git diff --check -- docs/pose-follow.md
```

Expected: the file exists, `rg` prints no matches, and `git diff --check` exits successfully.

- [ ] **Step 4: Review safety-critical statements against implementation**

Verify in `src/robot_control/cli.py` that:

- `--seconds` defaults to 60 seconds.
- Non-positive values are rejected while `inf` produces an infinite deadline.
- `Ctrl+C` reaches the `finally` block.
- The `finally` block zeros gravity effort and leaves the last position command held.
- `--execute` is required before `_follow_loop` runs.

Expected: every claim in the guide matches those code paths.

- [ ] **Step 5: Commit the guide**

```bash
git add docs/pose-follow.md
git commit -m "docs: add pose follow operator guide"
```
