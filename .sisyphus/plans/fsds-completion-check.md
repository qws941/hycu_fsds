# FSDS 자율주행 완주점검 계획

## TL;DR

> **Quick Summary**: 한양사이버대학교 FSDS 자율주행 프로젝트 제출 전 최종 점검. params.yaml 동기화, Docker healthcheck 수정, Git 커밋 정리.
> 
> **Deliverables**:
> - params.yaml 동기화 완료 (docker → src)
> - Docker healthcheck 경고 제거
> - Git 커밋 완료 ("feat: FSDS 자율주행 완주 최적화")
> 
> **Estimated Effort**: Quick (~15분)
> **Parallel Execution**: NO - sequential (의존성 있음)
> **Critical Path**: Task 1 → Task 2 → Task 3

---

## Context

### Original Request
FSDS 자율주행 시뮬레이션 제출물 완주점검. 코드 동기화 확인, params.yaml 불일치 해결, Docker healthcheck 수정, Git 커밋.

### Interview Summary
**Key Discussions**:
- params.yaml 버전: **docker 버전 (안정적)을 src로 동기화**
- Git 전략: **단일 커밋**으로 모든 변경사항 정리
- 추가 테스트: **불필요** (기존 10분 녹화로 충분)

**Research Findings**:
- 세션 메모리: `max_lateral_accel: 4.5`가 실제 테스트되어 안정적 TRACKING 확인됨
- Docker healthcheck: ROS 환경 미소싱이 원인

### Metis Review
**Identified Gaps** (addressed):
- 코드 동기화 검증: diff 명령어로 확인 추가
- Healthcheck 테스트: `docker-compose config` 검증 추가
- Git 커밋 전 staging 확인: `git status` 검증 추가

---

## Work Objectives

### Core Objective
FSDS 자율주행 프로젝트 제출 전 설정 파일 동기화 및 정리

### Concrete Deliverables
- `src/autonomous/config/params.yaml` - docker 버전과 동기화
- `fsds_docker/docker-compose.yml` - healthcheck 명령어 수정
- Git commit: "feat: FSDS 자율주행 완주 최적화"

### Definition of Done
- [ ] `diff src/autonomous/config/params.yaml fsds_docker/config/params.yaml` → 핵심 파라미터 동일
- [ ] `docker-compose -f fsds_docker/docker-compose.yml config` → 에러 없음
- [ ] `git log -1 --oneline` → 커밋 메시지 확인

### Must Have
- lookahead_speed_gain=0.4, max_lateral_accel=4.5 동기화
- ROS 환경 소싱 추가된 healthcheck
- 모든 변경사항 커밋

### Must NOT Have (Guardrails)
- 테스트된 파라미터 값 변경 금지
- 드라이버 코드 로직 수정 금지
- 새로운 기능 추가 금지

---

## Verification Strategy

> **UNIVERSAL RULE: ZERO HUMAN INTERVENTION**
> ALL tasks MUST be verifiable by running a command.

### Test Decision
- **Infrastructure exists**: YES (ROS, Docker)
- **Automated tests**: None (설정 파일 변경만)
- **Framework**: N/A

### Agent-Executed QA Scenarios (MANDATORY)

모든 Task는 Bash 명령어로 검증됨.

---

## Execution Strategy

### Sequential Execution (의존성 있음)

```
Task 1: params.yaml 동기화
    ↓
Task 2: Docker healthcheck 수정
    ↓
Task 3: Git 커밋
```

**Why Sequential**:
- Task 3 (Git)은 Task 1, 2 완료 후 실행해야 모든 변경 포함

### Dependency Matrix

| Task | Depends On | Blocks | Parallel |
|------|------------|--------|----------|
| 1 | None | 3 | No |
| 2 | None | 3 | Yes (1과) |
| 3 | 1, 2 | None | No |

**Note**: Task 1과 2는 기술적으로 병렬 가능하나, 단순함을 위해 순차 실행 권장.

---

## TODOs

- [ ] 1. params.yaml 동기화 (docker → src)

  **What to do**:
  - `src/autonomous/config/params.yaml` 파일에서 2개 파라미터 수정
  - `lookahead_speed_gain`: 0.3 → 0.4
  - `max_lateral_accel`: 6.0 → 4.5

  **Must NOT do**:
  - 다른 파라미터 변경 금지
  - 주석 수정 금지

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 단순 설정 파일 2줄 수정, 복잡한 로직 없음
  - **Skills**: `[]` (없음)
    - 단순 Edit 작업, 특수 스킬 불필요
  - **Skills Evaluated but Omitted**:
    - `git-master`: 이 Task는 커밋 아님, Task 3에서 사용

  **Parallelization**:
  - **Can Run In Parallel**: YES (Task 2와)
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 3
  - **Blocked By**: None

  **References**:

  **Pattern References**:
  - `fsds_docker/config/params.yaml:25` - lookahead_speed_gain: 0.4 (정답 값)
  - `fsds_docker/config/params.yaml:51` - max_lateral_accel: 4.5 (정답 값)

  **Target File**:
  - `src/autonomous/config/params.yaml:25` - 현재 lookahead_speed_gain: 0.3
  - `src/autonomous/config/params.yaml:51` - 현재 max_lateral_accel: 6.0

  **Acceptance Criteria**:

  **Agent-Executed QA Scenarios:**

  ```
  Scenario: params.yaml 동기화 검증
    Tool: Bash (diff)
    Preconditions: 두 params.yaml 파일 존재
    Steps:
      1. diff src/autonomous/config/params.yaml fsds_docker/config/params.yaml
      2. Assert: 출력이 비어있거나, 핵심 4개 파라미터 동일 확인
         - lookahead_base: 3.5
         - max_speed: 6.0
         - lookahead_speed_gain: 0.4
         - max_lateral_accel: 4.5
      3. grep "lookahead_speed_gain" src/autonomous/config/params.yaml
      4. Assert: 출력에 "0.4" 포함
      5. grep "max_lateral_accel" src/autonomous/config/params.yaml
      6. Assert: 출력에 "4.5" 포함
    Expected Result: 두 파라미터가 docker 버전과 동일
    Evidence: grep 출력 캡처
  ```

  **Commit**: NO (Task 3에서 통합)

---

- [ ] 2. Docker healthcheck 수정

  **What to do**:
  - `fsds_docker/docker-compose.yml` 파일에서 healthcheck 명령어 2개 수정
  - 라인 20: ROS 환경 소싱 추가
  - 라인 42: ROS 환경 소싱 추가

  **Exact Changes**:
  
  **라인 20 (roscore healthcheck)**:
  ```yaml
  # Before:
  test: ["CMD-SHELL", "rostopic list || exit 1"]
  
  # After:
  test: ["CMD-SHELL", "source /opt/ros/noetic/setup.bash && rostopic list || exit 1"]
  ```
  
  **라인 42 (fsds_bridge healthcheck)**:
  ```yaml
  # Before:
  test: ["CMD-SHELL", "rostopic list | grep -q '/fsds/' || exit 1"]
  
  # After:
  test: ["CMD-SHELL", "source /opt/ros/noetic/setup.bash && rostopic list | grep -q '/fsds/' || exit 1"]
  ```

  **Must NOT do**:
  - 다른 docker-compose 설정 변경 금지
  - 서비스 구성 변경 금지

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Docker 설정 파일 2줄 수정, 단순 작업
  - **Skills**: `[]`
    - 단순 Edit 작업
  - **Skills Evaluated but Omitted**:
    - `playwright`: Docker 설정이므로 브라우저 불필요

  **Parallelization**:
  - **Can Run In Parallel**: YES (Task 1과)
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 3
  - **Blocked By**: None

  **References**:

  **Pattern References**:
  - `fsds_docker/docker-compose.yml:64-68` - fsds_bridge command에서 ROS 소싱 패턴:
    ```yaml
    command: >
      bash -c "
        source /opt/ros/noetic/setup.bash &&
        ...
      "
    ```

  **Target File**:
  - `fsds_docker/docker-compose.yml:20` - roscore healthcheck
  - `fsds_docker/docker-compose.yml:42` - fsds_bridge healthcheck

  **Acceptance Criteria**:

  **Agent-Executed QA Scenarios:**

  ```
  Scenario: Docker healthcheck 구문 검증
    Tool: Bash (docker-compose, grep)
    Preconditions: docker-compose 설치됨
    Steps:
      1. docker-compose -f fsds_docker/docker-compose.yml config > /dev/null 2>&1
      2. Assert: exit code 0 (YAML 구문 유효)
      3. grep -n "source /opt/ros/noetic/setup.bash" fsds_docker/docker-compose.yml
      4. Assert: 최소 2개 라인 출력 (healthcheck 2개)
      5. grep "rostopic list" fsds_docker/docker-compose.yml | grep "source"
      6. Assert: 모든 rostopic 명령어에 source 포함
    Expected Result: healthcheck 명령어에 ROS 환경 소싱 포함
    Evidence: grep 출력 캡처
  
  Scenario: Healthcheck 명령어 실행 가능성 검증
    Tool: Bash (grep)
    Preconditions: 파일 수정 완료
    Steps:
      1. grep -A1 'test:.*CMD-SHELL' fsds_docker/docker-compose.yml
      2. Assert: 모든 healthcheck에 "source /opt/ros" 포함
    Expected Result: 모든 healthcheck가 ROS 환경 소싱 후 실행
    Evidence: grep 출력
  ```

  **Commit**: NO (Task 3에서 통합)

---

- [ ] 3. Git 커밋

  **What to do**:
  - 모든 변경사항 스테이징
  - 단일 커밋 생성: "feat: FSDS 자율주행 완주 최적화"

  **Commit Details**:
  - **Message**: `feat: FSDS 자율주행 완주 최적화`
  - **Description** (optional):
    ```
    - params.yaml: lookahead_speed_gain=0.4, max_lateral_accel=4.5 동기화
    - docker-compose: healthcheck ROS 환경 소싱 추가
    - 드라이버: build_centerline 로직 개선, ANTI-UTURN 필터
    - AGENTS.md: 프로젝트 문서화
    ```
  - **Files**: 18개 수정 파일 + 신규 파일들

  **Must NOT do**:
  - `.env` 파일 커밋 (이미 .gitignore에 있을 수 있음 - 확인 필요)
  - 녹화 파일 커밋 금지 (`recordings/` 디렉토리)
  - 시뮬레이터 바이너리 커밋 금지 (`src/simulator/fsds-linux/`)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 단순 git 명령어 실행
  - **Skills**: `['git-master']`
    - `git-master`: Git 커밋 전문 스킬, 안전한 커밋 워크플로우
  - **Skills Evaluated but Omitted**:
    - N/A

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 2 (final)
  - **Blocks**: None
  - **Blocked By**: Task 1, Task 2

  **References**:

  **Git Status Reference**:
  - 18개 수정 파일 (725+ / 472- 라인)
  - 주요: competition_driver.py, params.yaml, docker-compose.yml, AGENTS.md

  **Acceptance Criteria**:

  **Agent-Executed QA Scenarios:**

  ```
  Scenario: Git 커밋 전 상태 확인
    Tool: Bash (git)
    Preconditions: Task 1, 2 완료
    Steps:
      1. git status --porcelain
      2. Assert: 수정된 파일 목록 확인 (params.yaml, docker-compose.yml 포함)
      3. git diff --stat
      4. Assert: 변경 라인 수 확인
    Expected Result: 예상 파일들이 modified 상태
    Evidence: git status 출력

  Scenario: Git 커밋 실행 및 검증
    Tool: Bash (git)
    Preconditions: 파일 수정 완료
    Steps:
      1. git add -A
      2. git status --short
      3. Assert: staged 파일 확인
      4. git commit -m "feat: FSDS 자율주행 완주 최적화"
      5. Assert: exit code 0
      6. git log -1 --oneline
      7. Assert: 출력에 "feat: FSDS 자율주행 완주 최적화" 포함
    Expected Result: 커밋 성공, 메시지 확인
    Evidence: git log 출력

  Scenario: 민감 파일 제외 확인
    Tool: Bash (git)
    Preconditions: 커밋 완료
    Steps:
      1. git show --stat HEAD
      2. Assert: recordings/ 디렉토리 파일 없음
      3. Assert: src/simulator/fsds-linux/ 바이너리 없음
    Expected Result: 민감/대용량 파일 제외됨
    Evidence: git show 출력
  ```

  **Commit**: YES (이 Task 자체가 커밋)
  - Message: `feat: FSDS 자율주행 완주 최적화`
  - Files: All staged files
  - Pre-commit: N/A

---

## Commit Strategy

| After Task | Message | Files | Verification |
|------------|---------|-------|--------------|
| 3 | `feat: FSDS 자율주행 완주 최적화` | 18+ files | `git log -1` |

---

## Success Criteria

### Verification Commands
```bash
# params.yaml 동기화 확인
grep "lookahead_speed_gain" src/autonomous/config/params.yaml
# Expected: lookahead_speed_gain: 0.4

grep "max_lateral_accel" src/autonomous/config/params.yaml
# Expected: max_lateral_accel: 4.5

# Docker healthcheck 확인
grep -c "source /opt/ros/noetic/setup.bash" fsds_docker/docker-compose.yml
# Expected: 2 이상

# Git 커밋 확인
git log -1 --oneline
# Expected: feat: FSDS 자율주행 완주 최적화
```

### Final Checklist
- [ ] lookahead_speed_gain = 0.4 (src/autonomous)
- [ ] max_lateral_accel = 4.5 (src/autonomous)
- [ ] Docker healthcheck에 ROS 소싱 포함
- [ ] Git 커밋 완료
- [ ] 녹화 파일/바이너리 커밋에서 제외

---

## Notes

- **예상 소요 시간**: ~15분
- **난이도**: Easy (설정 파일 수정 + Git)
- **위험도**: Low (드라이버 로직 변경 없음)
- **롤백**: `git reset --soft HEAD~1`로 커밋 취소 가능
