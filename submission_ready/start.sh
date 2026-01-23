#!/bin/bash
set -e

echo "=== FSDS Docker 환경 시작 ==="

if [ ! -f .env ]; then
    echo "Error: .env 파일이 없습니다. .env 파일을 생성하세요."
    exit 1
fi

source .env

check_fsds_host() {
    if [ "$FSDS_HOST_IP" = "host.docker.internal" ]; then
        echo "   Using Docker internal networking"
        return 0
    fi
    
    echo "   Checking FSDS host connectivity: $FSDS_HOST_IP:41451"
    
    if ! command -v nc &> /dev/null; then
        if ! timeout 3 bash -c "echo >/dev/tcp/$FSDS_HOST_IP/41451" 2>/dev/null; then
            echo "Error: Cannot connect to FSDS simulator at $FSDS_HOST_IP:41451"
            echo "Please ensure:"
            echo "  1. Windows FSDS simulator is running"
            echo "  2. FSDS_HOST_IP in .env is correct (current: $FSDS_HOST_IP)"
            echo "  3. Firewall allows port 41451"
            return 1
        fi
    else
        if ! nc -z -w 3 "$FSDS_HOST_IP" 41451 2>/dev/null; then
            echo "Error: Cannot connect to FSDS simulator at $FSDS_HOST_IP:41451"
            echo "Please ensure:"
            echo "  1. Windows FSDS simulator is running"
            echo "  2. FSDS_HOST_IP in .env is correct (current: $FSDS_HOST_IP)"
            echo "  3. Firewall allows port 41451"
            return 1
        fi
    fi
    
    echo "   FSDS host reachable"
    return 0
}

if ! check_fsds_host; then
    exit 1
fi

echo "1. Docker 이미지 빌드..."
docker-compose build

echo "2. ROS Core 시작..."
docker-compose up -d roscore
sleep 3

echo "3. FSDS Bridge 시작..."
echo "   Windows FSDS 시뮬레이터가 실행 중인지 확인하세요!"
echo "   Host IP: $FSDS_HOST_IP"
docker-compose up -d fsds_bridge
sleep 5

echo "4. 개발 컨테이너 시작..."
docker-compose up -d fsds_dev

echo ""
echo "=== 시작 완료 ==="
echo ""
echo "사용 방법:"
echo "  개발 컨테이너 접속:  docker exec -it fsds_dev bash"
echo "  기본 주행 테스트:    python3 /root/catkin_ws/src/fsds_scripts/scripts/basic_driver.py"
echo "  자율주행 테스트:     python3 /root/catkin_ws/src/fsds_scripts/scripts/autonomous_driver.py"
echo "  RViz 시작:          docker-compose --profile viz up -d rviz"
echo "  로그 확인:          docker-compose logs -f"
echo "  전체 종료:          docker-compose down"
