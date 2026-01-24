# HYCU FSDS 자율주행 경진대회

## 구조

```
submission/
├── 001_simulator/          # Windows FSDS 시뮬레이터
│   ├── settings.json       # 시뮬레이터 설정
│   └── README.md
│
└── 002_autonomous/         # Linux Docker 자율주행
    ├── driver/             # 메인 드라이버
    │   └── competition_driver.py
    ├── modules/            # 모듈
    │   ├── control/        # Pure Pursuit, 속도 제어
    │   ├── perception/     # 콘 검출, SLAM
    │   └── utils/          # Watchdog, 타이머
    ├── config/params.yaml  # 파라미터
    └── tests/              # 테스트
```

## 실행 방법

### 1. Windows (시뮬레이터)
```
1. 001_simulator/settings.json → C:\Users\<사용자>\Documents\AirSim\
2. FSDS 시뮬레이터 실행
3. 방화벽: 41451 포트 허용
```

### 2. Linux (자율주행)
```bash
cd 002_autonomous
# .env 파일에서 FSDS_HOST_IP를 Windows IP로 변경
./start.sh
```

### 3. 드라이버 실행
```bash
docker exec -it fsds_autonomous bash
python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py
```

## 핵심 알고리즘

| 모듈 | 알고리즘 |
|------|----------|
| 조향 | Pure Pursuit (Coulter, 1992) |
| 속도 | 곡률 기반 v = √(a_lat_max / κ) |
| 인지 | Grid BFS 클러스터링 O(N) |
| 안전 | 3-tier Watchdog 상태머신 |

## 테스트
```bash
docker exec -it fsds_autonomous bash
python3 -m pytest /root/catkin_ws/src/autonomous/tests/ -v
```
