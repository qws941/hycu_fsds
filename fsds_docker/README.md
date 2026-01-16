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

# 5. 자율주행 실행
python3 /root/catkin_ws/src/fsds_scripts/scripts/autonomous_driver.py
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
│   └── autonomous_driver.py  # 콘 탐지 자율주행
├── data/               # 공유 데이터
└── rviz/               # RViz 설정
```

## 주요 명령어

| 작업 | 명령어 |
|------|--------|
| 전체 시작 | `./start.sh` |
| 개발 접속 | `docker exec -it fsds_dev bash` |
| RViz 시각화 | `docker-compose --profile viz up -d rviz` |
| 로그 확인 | `docker-compose logs -f` |
| 전체 종료 | `docker-compose down` |

## ROS 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/fsds/control_command` | ControlCommand | 차량 제어 (throttle, steering, brake) |
| `/fsds/lidar/Lidar1` | PointCloud2 | LiDAR 포인트 클라우드 |
| `/fsds/testing_only/odom` | Odometry | 차량 위치/속도 |

## 알고리즘 파라미터

### autonomous_driver.py

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `max_throttle` | 0.2 | 최대 가속 |
| `target_speed` | 4.0 m/s | 목표 속도 |
| `max_steering` | 0.3 | 최대 조향 |
| `cones_range_cutoff` | 7.0m | 콘 탐지 범위 |

## 차별화 아이디어

1. **PID 조향 제어** - 부드러운 경로 추종
2. **콘 색상 분류** - 파랑/노랑 구분으로 트랙 경계 인식
3. **중앙선 추종** - 양쪽 콘 사이 최적 경로 계산
4. **SLAM 통합** - 맵 생성 + 위치 추정
5. **속도 최적화** - 곡선 구간 감속, 직선 가속
