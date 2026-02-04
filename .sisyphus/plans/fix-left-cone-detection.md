# Fix Left Cone Detection Bug

## TL;DR

> **Quick Summary**: 왼쪽 콘이 감지되지 않는 버그 수정. 디버그 로깅으로 원인 확인 후 좌표계 또는 필터 수정.
> 
> **Deliverables**:
> - 디버그 로깅 추가로 원인 진단
> - cone_detector.py 수정 (Y좌표 분류 로직)
> - 양쪽 콘 정상 감지 확인
> 
> **Estimated Effort**: Quick (~30분)
> **Parallel Execution**: NO - sequential
> **Critical Path**: Task 1 → Task 2 → Task 3

---

## Context

### Original Request
왼쪽 콘이 감지되지 않음 (L:0, R:1-6). "No centerline" 경고 및 콘 충돌 발생.

### Interview Summary
**Key Discussions**:
- 증상: `L:0 R:1-6` 반복, 왼쪽 항상 0
- 가능한 원인: LiDAR 좌표계 반전, front_mask 필터 문제

**Research Findings**:
- `cone_detector.py:122-124`: `y > 0`을 왼쪽으로 분류
- FSDS/AirSim은 ROS 표준과 다른 좌표계 사용 가능성 (Y+ = 오른쪽)
- `front_mask`: `x > 0.3`만 통과 - 측면 콘 필터링 가능성

---

## Work Objectives

### Core Objective
왼쪽/오른쪽 콘 모두 정상 감지되도록 수정

### Concrete Deliverables
- `cone_detector.py` 수정본
- 양쪽 콘 감지 확인 로그

### Definition of Done
- [ ] 드라이버 실행 시 `L:N R:M` (N, M 모두 1 이상)
- [ ] "No centerline" 경고 해결

### Must Have
- 실제 Y값 확인 후 정확한 원인 진단
- 최소한의 코드 변경

### Must NOT Have (Guardrails)
- 다른 파라미터 임의 수정 금지
- 기존 클러스터링 로직 변경 금지
- 테스트 없는 배포 금지

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (`tests/` 디렉토리 존재)
- **Automated tests**: NO (빠른 수정이므로 실행 테스트로 확인)
- **Agent-Executed QA**: 실제 시뮬레이터 실행으로 확인

### Agent-Executed QA Scenarios

```
Scenario: 디버그 로그로 Y값 확인
  Tool: interactive_bash (tmux)
  Preconditions: Docker 컨테이너 실행 중, FSDS 시뮬레이터 연결됨
  Steps:
    1. tmux new-session: docker exec -it fsds_autonomous bash
    2. Run: python3 driver/competition_driver.py
    3. Wait for: 콘 감지 로그 출력 (timeout: 30s)
    4. Capture: "[DEBUG] All cones Y values:" 라인
    5. Assert: Y값들의 부호 확인 (양수/음수 분포)
  Expected Result: Y값 분포에서 좌/우 구분 패턴 파악
  Evidence: 터미널 출력 캡처

Scenario: 수정 후 양쪽 콘 감지 확인
  Tool: interactive_bash (tmux)
  Preconditions: cone_detector.py 수정 완료
  Steps:
    1. Run: python3 driver/competition_driver.py
    2. Wait for: 콘 감지 로그 (timeout: 30s)
    3. Assert: "L:N" 출력에서 N >= 1
    4. Assert: "R:M" 출력에서 M >= 1
    5. Assert: "No centerline" 경고 없음
  Expected Result: L과 R 모두 1개 이상 감지
  Evidence: 터미널 출력 캡처
```

---

## Execution Strategy

### Dependency Matrix

| Task | Depends On | Blocks | Can Parallelize With |
|------|------------|--------|---------------------|
| 1 | None | 2 | None |
| 2 | 1 | 3 | None |
| 3 | 2 | None | None |

---

## TODOs

- [ ] 1. 디버그 로깅 추가

  **What to do**:
  - `cone_detector.py`의 `find_cones_filtered()` 함수에 디버그 출력 추가
  - 클러스터링 후 모든 콘의 Y값 출력
  - left/right 분류 결과 출력

  **Must NOT do**:
  - 분류 로직 자체를 아직 변경하지 말 것
  - 기존 return 값 변경 금지

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: []
    - 단순 로깅 추가, 특수 스킬 불필요

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Sequential
  - **Blocks**: Task 2
  - **Blocked By**: None

  **References**:
  - `src/autonomous/modules/perception/cone_detector.py:116-125` - 콘 리스트 생성 및 분류 부분
  - `src/autonomous/driver/competition_driver.py` - 현재 로깅 패턴 참고

  **Changes**:
  ```python
  # cone_detector.py line 120 이후, return 전에 추가:
  print(f"[DEBUG] All cones: {[(c['x']:.2f, c['y']:.2f) for c in cones]}")
  print(f"[DEBUG] Y values: {[c['y'] for c in cones]}")
  print(f"[DEBUG] Left (y>0): {len(left)}, Right (y<=0): {len(right)}")
  ```

  **Acceptance Criteria**:
  - [ ] 드라이버 실행 시 `[DEBUG]` 로그가 출력됨
  - [ ] 모든 콘의 Y값을 확인할 수 있음
  - [ ] left/right 카운트가 로그에 표시됨

  **Commit**: YES
  - Message: `debug(perception): add cone Y-value logging for diagnosis`
  - Files: `src/autonomous/modules/perception/cone_detector.py`

---

- [ ] 2. 원인 분석 및 수정

  **What to do**:
  - Task 1의 로그를 분석하여 원인 확정
  - **Case A**: Y좌표 부호 반전 필요 시 → 분류 조건 swap
  - **Case B**: 필터 문제 시 → front_mask 조건 조정
  
  **예상 수정 (Case A - Y좌표 반전)**:
  ```python
  # line 122-124 변경
  # FSDS는 Y+ = 오른쪽이므로 조건 반전
  left = sorted([c for c in cones if c['y'] < 0], key=lambda c: c['x'])
  right = sorted([c for c in cones if c['y'] >= 0], key=lambda c: c['x'])
  ```

  **예상 수정 (Case B - front_mask 완화)**:
  ```python
  # line 60 변경
  front_mask = points[:, 0] > 0.0  # 또는 -0.3 등으로 완화
  ```

  **Must NOT do**:
  - 로그 분석 없이 임의 수정 금지
  - 양쪽 다 수정하지 말고, 원인 하나만 수정

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Blocks**: Task 3
  - **Blocked By**: Task 1

  **References**:
  - Task 1에서 캡처한 디버그 로그
  - `cone_detector.py:60` - front_mask 정의
  - `cone_detector.py:122-124` - left/right 분류

  **Acceptance Criteria**:
  - [ ] 원인 명확히 기록됨 (Case A 또는 B)
  - [ ] 해당 원인에 맞는 수정 적용됨
  - [ ] 디버그 로깅은 유지 (검증용)

  **Commit**: YES
  - Message: `fix(perception): correct cone left/right classification for FSDS coordinate system`
  - Files: `src/autonomous/modules/perception/cone_detector.py`

---

- [ ] 3. 검증 및 정리

  **What to do**:
  - 수정된 드라이버 실행
  - 양쪽 콘 감지 확인 (L:N, R:M, N>=1, M>=1)
  - 디버그 로깅 제거 또는 주석 처리
  - 최종 커밋

  **Must NOT do**:
  - 검증 전 디버그 로그 제거 금지

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Sequential (final)
  - **Blocks**: None
  - **Blocked By**: Task 2

  **References**:
  - 이전 증상: `L:0 R:1-6`
  - 기대 결과: `L:1-6 R:1-6`

  **Acceptance Criteria**:
  - [ ] `L:N R:M` 로그에서 N >= 1, M >= 1
  - [ ] "No centerline" 경고 없음
  - [ ] 차량이 트랙 중앙을 따라 주행
  - [ ] 디버그 로깅 제거됨

  **Commit**: YES
  - Message: `chore(perception): remove debug logging after cone detection fix verified`
  - Files: `src/autonomous/modules/perception/cone_detector.py`

---

## Commit Strategy

| After Task | Message | Files |
|------------|---------|-------|
| 1 | `debug(perception): add cone Y-value logging` | cone_detector.py |
| 2 | `fix(perception): correct left/right classification` | cone_detector.py |
| 3 | `chore(perception): remove debug logging` | cone_detector.py |

---

## Success Criteria

### Verification Commands
```bash
# Container 접속
docker exec -it fsds_autonomous bash

# 드라이버 실행
python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py

# 기대 출력
# L:3 R:4 (양쪽 모두 1 이상)
# centerline 정상 생성
```

### Final Checklist
- [ ] 왼쪽 콘 감지됨 (L >= 1)
- [ ] 오른쪽 콘 감지됨 (R >= 1)
- [ ] 중앙선 정상 생성
- [ ] 콘 충돌 없음
- [ ] 디버그 로그 제거됨
