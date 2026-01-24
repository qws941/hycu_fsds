# HYCU FSDS 자율주행

Formula Student Driverless Simulator 기반 자율주행 시스템

## 구조

```
hycu_fsds/
├── src/
│   ├── simulator/      # Windows FSDS 설정
│   └── autonomous/     # Docker/ROS 자율주행
├── dist/               # 배포 패키지
└── docs/               # 문서
```

## 빠른 시작

```bash
# 1. Windows: src/simulator/settings.json → C:\Users\<user>\Documents\AirSim\
# 2. Windows: FSDS 시뮬레이터 실행
# 3. Linux:
cd src/autonomous
sed -i 's/YOUR_WINDOWS_IP_HERE/<Windows IP>/' .env
./start.sh
```

## 드라이버 실행

```bash
docker exec -it fsds_autonomous bash
python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py
```
