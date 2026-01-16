# FSDS 자율주행 시뮬레이션 (Docker 환경)

한양사이버대학교 자율주행 시뮬레이션 포뮬러 경진대회

## 빠른 시작

```bash
# 1. Windows에서 FSDS 시뮬레이터 실행
#    다운로드: https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases

# 2. .env 파일에서 Windows IP 설정
#    ipconfig로 확인 후 FSDS_HOST_IP 수정

# 3. Docker 환경 시작
./start.sh

# 4. 개발 컨테이너 접속
docker exec -it fsds_dev bash

# 5. 자율주행 실행 (경진대회용)
python3 /root/catkin_ws/src/fsds_scripts/scripts/competition_driver.py
```

## 프로젝트 구조

```
fsds_docker/
├── Dockerfile          # ROS Noetic + FSDS 이미지
├── docker-compose.yml  # 멀티 컨테이너 구성
├── .env                # 환경 변수 (Windows IP)
├── entrypoint.sh       # 컨테이너 진입점
├── start.sh            # 원클릭 시작 스크립트
├── scripts/            # 자율주행 코드
│   ├── basic_driver.py       # 기본 주행 (장애물 정지)
│   ├── autonomous_driver.py  # 콘 탐지 자율주행
│   ├── advanced_driver.py    # PID + 중앙선 추종
│   ├── competition_driver.py # 경진대회용 (Pure Pursuit + 곡률 속도)
│   └── simple_slam.py        # SLAM 맵 생성
├── data/               # 공유 데이터
└── rviz/               # RViz 설정
```

## 드라이버 비교

| 드라이버 | 알고리즘 | 특징 |
|----------|----------|------|
| `basic_driver.py` | LiDAR 장애물 감지 | 3m 내 장애물 시 정지 |
| `autonomous_driver.py` | 콘 그룹핑 + 단순 조향 | ±max_steering 이진 조향 |
| `advanced_driver.py` | PID + 중앙선 | PID 조향 제어 |
| `competition_driver.py` | **Pure Pursuit + 곡률 속도** | **경진대회 제출용** |
| `simple_slam.py` | Occupancy Grid SLAM | 맵 생성 + 경로 기록 |

## 경진대회 드라이버 (competition_driver.py)

### 핵심 기능

1. **콘 검출 품질 필터**: 높이(z) + 클러스터 반경/분산 검증
2. **Pure Pursuit 조향**: Lookahead 기반 기하학적 경로 추종
3. **곡률 기반 속도 제어**: 커브 감속, 직선 가속
4. **실패 복구 상태머신**: TRACKING → DEGRADED → STOPPING

### 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `max_throttle` | 0.25 | 최대 가속 |
| `min_speed` / `max_speed` | 2.0 / 6.0 m/s | 속도 범위 |
| `max_steering` | 0.4 | 최대 조향 |
| `lookahead_base` | 4.0m | 기본 전방 주시 거리 |
| `cones_range_cutoff` | 12.0m | 콘 탐지 범위 |
| `cone_min_z` / `cone_max_z` | -0.3 / 0.5m | 콘 높이 필터 |

### 상태머신

| 상태 | 조건 | 동작 |
|------|------|------|
| TRACKING | 좌/우 콘 모두 탐지 | 정상 Pure Pursuit |
| DEGRADED | 1초 이상 콘 미탐지 | 마지막 centerline 유지 + 최저 속도 |
| STOPPING | 3초 이상 콘 미탐지 | 완전 정지 |

## SLAM (simple_slam.py)

별도 터미널에서 실행:
```bash
python3 /root/catkin_ws/src/fsds_scripts/scripts/simple_slam.py
```

### 출력 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/slam/map` | OccupancyGrid | 점유 격자 맵 |
| `/slam/path` | Path | 주행 경로 기록 |
| `/slam/pose` | PoseStamped | 현재 위치/자세 |

## 주요 명령어

| 작업 | 명령어 |
|------|--------|
| 전체 시작 | `./start.sh` |
| 개발 접속 | `docker exec -it fsds_dev bash` |
| 경진대회 실행 | `python3 .../competition_driver.py` |
| SLAM 실행 | `python3 .../simple_slam.py` |
| RViz 시각화 | `docker-compose --profile viz up -d rviz` |
| 로그 확인 | `docker-compose logs -f` |
| 전체 종료 | `docker-compose down` |

## ROS 토픽

| 토픽 | 타입 | 방향 |
|------|------|------|
| `/fsds/control_command` | ControlCommand | Publish |
| `/fsds/lidar/Lidar1` | PointCloud2 | Subscribe |
| `/fsds/testing_only/odom` | Odometry | Subscribe |
