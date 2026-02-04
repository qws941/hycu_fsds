# Centerline Generation Fix - Single Side Offset

## TL;DR

> **Quick Summary**: FSDS 자율주행 드라이버의 centerline 생성 실패율(30-40%)을 개선. 한쪽 콘만 감지될 때 사용하는 `single_side_offset`을 하드코딩된 1.0m에서 `safe_track_width/2`로 변경하여 트랙 중앙으로 정확히 경로 생성.
> 
> **Deliverables**:
> - `competition_driver.py` build_centerline() 함수 수정 (2줄 변경)
> - 시뮬레이터 1랩 완주 테스트로 검증
> 
> **Estimated Effort**: Quick (15분 수정 + 10분 테스트)
> **Parallel Execution**: NO - sequential
> **Critical Path**: Task 1 (수정) → Task 2 (검증)

---

## Context

### Original Request
FSDS 자율주행 프로젝트에서 centerline 생성 실패율이 30-40%로 높음. 특히 한쪽 콘만 감지될 때 (L:0 R:5 또는 L:5 R:0) 문제 발생. "No current centerline, using last valid at min speed" 메시지가 빈번하게 출력됨.

### Interview Summary
**Key Discussions**:
- 수정 범위: **최소 변경** 선택 (offset만 수정)
- 검증 방법: **시뮬레이터 테스트**로 검증
- 구조 유지: cone_detector.py 통합하지 않음

**Research Findings**:
- `single_side_offset = 1.0` 하드코딩이 근본 원인
- `safe_track_width` 변수가 선언되어 있으나 사용되지 않음
- 트랙 폭 3-4m에서 1.0m offset은 중앙에서 벗어난 경로 생성

### Root Cause Analysis
| 이슈 | 현재 | 수정 후 |
|------|------|---------|
| Line 475 | `single_side_offset = 1.0` | 삭제 (미사용) |
| Line 494 | `left_y + single_side_offset` | `left_y - (safe_track_width / 2.0)` |
| Line 498 | `right_y - single_side_offset` | `right_y + (safe_track_width / 2.0)` |

**Note**: 왼쪽 콘(y>0)에서 centerline은 -Y 방향(안쪽), 오른쪽 콘(y<0)에서는 +Y 방향(안쪽)으로 이동해야 함.

---

## Work Objectives

### Core Objective
한쪽 콘만 감지될 때 centerline이 트랙 중앙에 생성되도록 offset 계산 로직 수정

### Concrete Deliverables
- `src/autonomous/driver/competition_driver.py` 수정 (lines 494, 498)

### Definition of Done
- [ ] 시뮬레이터에서 1랩 완주 시 "No current centerline" 메시지 빈도 50% 이상 감소

### Must Have
- 한쪽 콘만 있을 때 `safe_track_width / 2.0` 사용
- 기존 양쪽 콘 로직은 변경하지 않음

### Must NOT Have (Guardrails)
- cone_detector.py 모듈 수정하지 않음
- 다른 파라미터 튜닝하지 않음
- 상태 머신 로직 변경하지 않음

---

## Verification Strategy

> **UNIVERSAL RULE: ZERO HUMAN INTERVENTION**
>
> ALL tasks MUST be verifiable WITHOUT any human action.

### Test Decision
- **Infrastructure exists**: YES (Docker + FSDS)
- **Automated tests**: NO (시뮬레이터 실시간 검증)
- **Framework**: ROS + FSDS 시뮬레이터

### Agent-Executed QA Scenarios (MANDATORY)

**Verification Tool**: PTY + Docker exec

---

## Execution Strategy

### Sequential Execution
```
Task 1 (수정) → Task 2 (검증)
```

No parallel execution - 수정 완료 후 검증 필요.

---

## TODOs

- [ ] 1. Fix single_side_offset calculation in build_centerline()

  **What to do**:
  1. `competition_driver.py` 라인 494 수정:
     - 변경 전: `target_y = left_y + single_side_offset`
     - 변경 후: `target_y = left_y - (safe_track_width / 2.0)`
  2. `competition_driver.py` 라인 498 수정:
     - 변경 전: `target_y = right_y - single_side_offset`
     - 변경 후: `target_y = right_y + (safe_track_width / 2.0)`
  3. 라인 475의 `single_side_offset = 1.0` 삭제 (더 이상 사용 안 함)

  **Must NOT do**:
  - 라인 503-507의 fallback 로직 수정하지 않음
  - safety_margin 값 변경하지 않음
  - safe_track_width 계산 로직 변경하지 않음

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 단순 2줄 수정, 복잡한 로직 변경 없음
  - **Skills**: []
    - 기본 Python 편집만 필요

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Sequential
  - **Blocks**: Task 2 (검증)
  - **Blocked By**: None (시작 가능)

  **References**:
  - `src/autonomous/driver/competition_driver.py:469-531` - build_centerline() 함수 전체
  - `src/autonomous/driver/competition_driver.py:476` - safe_track_width 계산 (이미 존재)
  - `src/autonomous/modules/perception/cone_detector.py:168-173` - 올바른 offset 사용 예시

  **Why Each Reference Matters**:
  - build_centerline() 전체 컨텍스트 이해 필요
  - safe_track_width 변수가 이미 선언되어 있음을 확인
  - cone_detector.py에서 `last_valid_track_width / 2` 패턴 참조

  **Acceptance Criteria**:
  - [ ] 라인 494: `left_y - (safe_track_width / 2.0)` 로 변경됨
  - [ ] 라인 498: `right_y + (safe_track_width / 2.0)` 로 변경됨
  - [ ] `single_side_offset` 변수 삭제 또는 미사용 상태 확인
  - [ ] Python 문법 오류 없음: `python3 -m py_compile competition_driver.py`

  **Agent-Executed QA Scenarios**:

  ```
  Scenario: Python syntax validation
    Tool: Bash
    Preconditions: competition_driver.py 수정 완료
    Steps:
      1. docker exec fsds_autonomous python3 -m py_compile /root/catkin_ws/src/autonomous/driver/competition_driver.py
      2. Assert: exit code 0 (no syntax errors)
    Expected Result: 문법 오류 없이 컴파일 성공
    Evidence: Exit code 0

  Scenario: Verify code changes applied
    Tool: Bash (grep)
    Preconditions: 파일 수정 완료
    Steps:
      1. grep -n "safe_track_width / 2.0" src/autonomous/driver/competition_driver.py
      2. Assert: 2 matches found (lines ~494, ~498)
      3. grep -n "single_side_offset" src/autonomous/driver/competition_driver.py
      4. Assert: 0 matches in lines 494-498 (제거됨)
    Expected Result: safe_track_width 사용, single_side_offset 미사용
    Evidence: grep 출력 결과
  ```

  **Commit**: YES
  - Message: `fix(driver): use track width for single-side centerline offset`
  - Files: `src/autonomous/driver/competition_driver.py`
  - Pre-commit: `python3 -m py_compile`

---

- [ ] 2. Verify fix in FSDS simulator

  **What to do**:
  1. Docker 컨테이너에서 드라이버 재시작
  2. 1랩 완주 동안 로그 모니터링
  3. "No current centerline" 메시지 빈도 계산
  4. 이전 대비 50% 이상 감소 확인

  **Must NOT do**:
  - 파라미터 튜닝하지 않음
  - 시뮬레이터 설정 변경하지 않음

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 시뮬레이터 실행 및 로그 모니터링
  - **Skills**: []
    - PTY 사용만 필요

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Sequential
  - **Blocks**: None (최종 태스크)
  - **Blocked By**: Task 1

  **References**:
  - `src/autonomous/start.sh` - 환경 시작 스크립트
  - `src/autonomous/docker-compose.yml` - Docker 구성
  - 이전 세션 기록: "No current centerline" 메시지 빈도 ~30-40%

  **Why Each Reference Matters**:
  - start.sh로 시뮬레이터 + Docker 시작
  - docker-compose.yml로 컨테이너 상태 확인
  - 이전 빈도와 비교하여 개선 측정

  **Acceptance Criteria**:
  - [ ] 드라이버가 TRACKING 모드로 정상 주행
  - [ ] 1랩 완주 성공
  - [ ] "No current centerline" 메시지 빈도 50% 이상 감소

  **Agent-Executed QA Scenarios**:

  ```
  Scenario: Start driver and monitor centerline messages
    Tool: interactive_bash (tmux) + PTY
    Preconditions: FSDS 시뮬레이터 + Docker 실행 중
    Steps:
      1. docker exec -it fsds_autonomous bash
      2. source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
      3. python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py _target_laps:=1 2>&1 | tee /tmp/driver.log &
      4. Wait for: "TRACKING" in output (timeout: 30s)
      5. Wait for: 1랩 완주 또는 5분 경과
      6. grep -c "No current centerline" /tmp/driver.log
      7. Assert: count < 이전 빈도의 50% (예: 이전 100회 → 50회 미만)
    Expected Result: centerline 실패 메시지 감소
    Evidence: /tmp/driver.log 파일

  Scenario: Verify lap completion
    Tool: Bash
    Preconditions: 드라이버 실행 중
    Steps:
      1. docker exec fsds_autonomous bash -c "source /opt/ros/noetic/setup.bash && rostopic echo /lap/count -n 1"
      2. Assert: lap count >= 1
    Expected Result: 최소 1랩 완주
    Evidence: ROS topic 출력
  ```

  **Commit**: NO (검증만)

---

## Commit Strategy

| After Task | Message | Files | Verification |
|------------|---------|-------|--------------|
| 1 | `fix(driver): use track width for single-side centerline offset` | competition_driver.py | py_compile |

---

## Success Criteria

### Verification Commands
```bash
# 문법 검증
docker exec fsds_autonomous python3 -m py_compile /root/catkin_ws/src/autonomous/driver/competition_driver.py

# 코드 변경 확인
grep -n "safe_track_width / 2.0" src/autonomous/driver/competition_driver.py  # 2 matches expected

# 시뮬레이터 테스트
docker exec -it fsds_autonomous bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py _target_laps:=1"
```

### Final Checklist
- [ ] `safe_track_width / 2.0` 사용하여 centerline offset 계산
- [ ] 시뮬레이터에서 1랩 완주
- [ ] "No current centerline" 메시지 50% 이상 감소
