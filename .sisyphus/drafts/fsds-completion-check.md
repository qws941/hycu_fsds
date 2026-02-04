# Draft: FSDS 완주점검 작업 계획

## 요청 요약 (Request Summary)
한양사이버대학교 포뮬러 경진대회 FSDS 자율주행 시뮬레이션 제출 전 완주점검

## 현재 상태 분석

### 1. 코드 동기화: ✅ 완료
- `src/autonomous/driver/competition_driver.py` ↔ `fsds_docker/scripts/competition_driver.py`
- build_centerline() 함수 동기화됨
- ANTI-UTURN 필터 ✅, single_side_offset=1.0 ✅, x매칭 3.0m ✅

### 2. params.yaml: ⚠️ 결정 필요

| 파라미터 | src/autonomous | fsds_docker | 차이 |
|----------|----------------|-------------|------|
| lookahead_base | 3.5 | 3.5 | 동일 |
| max_speed | 6.0 | 6.0 | 동일 |
| cone_min_z | -0.3 | -0.3 | 동일 |
| cone_min_points | 3 | 3 | 동일 |
| **lookahead_speed_gain** | **0.3** | **0.4** | 다름 |
| **max_lateral_accel** | **6.0** | **4.5** | 다름 |

**분석:**
- `lookahead_speed_gain`: 0.3 (src) = 더 짧은 lookahead, 0.4 (docker) = 더 긴 lookahead
- `max_lateral_accel`: 6.0 (src) = 더 공격적 코너링, 4.5 (docker) = 더 안정적 코너링

### 3. Docker healthcheck: ❌ 수정 필요
**문제 위치:** `fsds_docker/docker-compose.yml`
- 라인 20: `test: ["CMD-SHELL", "rostopic list || exit 1"]`
- 라인 42: `test: ["CMD-SHELL", "rostopic list | grep -q '/fsds/' || exit 1"]`

**원인:** ROS 환경 미소싱 (`source /opt/ros/noetic/setup.bash` 누락)

### 4. Git 상태
- 18개 파일 수정됨 (725+ / 472- 라인)
- 미커밋 상태
- 주요 변경: competition_driver.py, params.yaml, AGENTS.md

### 5. 제출 파일
- `/mnt/hycu/fsds_clean_lap.mp4` (302MB, 10분) ✅ 제출됨

## 결정 필요 사항

### Q1: params.yaml 어느 버전이 정답?
- **옵션 A (src 버전):** lookahead_speed_gain=0.3, max_lateral_accel=6.0 (공격적)
- **옵션 B (docker 버전):** lookahead_speed_gain=0.4, max_lateral_accel=4.5 (안정적)
- **옵션 C:** docker 버전을 src로 동기화 (안정적 버전이 실제 테스트됨)

### Q2: Git 커밋 범위
- 모든 변경사항 한번에 커밋?
- 의미 단위로 분리 커밋?

### Q3: 실시간 테스트 필요 여부
- 기존 녹화 영상 충분?
- 추가 완주 테스트 필요?

## 작업 목록 (초안)

1. params.yaml 동기화
2. Docker healthcheck 수정
3. Git 커밋
4. (선택) 실시간 완주 테스트
5. (선택) 최종 녹화

## Open Questions
- [ ] params.yaml 정답 버전 결정
- [ ] Git 커밋 전략 결정
- [ ] 추가 테스트 필요 여부

---
*Draft recorded: 2026-02-05*
