#!/bin/bash
# 개발 컨테이너 빠른 접속
# 사용법: ./dev.sh [command]
#   ./dev.sh          → 컨테이너 쉘 접속
#   ./dev.sh run      → 드라이버 실행
#   ./dev.sh test     → 테스트 실행
#   ./dev.sh slam     → SLAM 실행
#   ./dev.sh stop     → 전체 종료

cd "$(dirname "$0")"

case "${1:-shell}" in
    shell|sh|"")
        docker exec -it fsds_dev bash
        ;;
    run|drive)
        docker exec -it fsds_dev bash -c "
            export PYTHONPATH=/root/catkin_ws/src/fsds_scripts:\$PYTHONPATH
            python3 /root/catkin_ws/src/fsds_scripts/src/drivers/competition.py
        "
        ;;
    test)
        docker exec -it fsds_dev bash -c "
            pip3 install -q pytest 2>/dev/null
            python3 -m pytest /root/catkin_ws/src/fsds_scripts/tests/ -v
        "
        ;;
    slam)
        docker exec -it fsds_dev bash -c "
            export PYTHONPATH=/root/catkin_ws/src/fsds_scripts:\$PYTHONPATH
            python3 /root/catkin_ws/src/fsds_scripts/src/perception/slam.py
        "
        ;;
    stop|down)
        docker-compose down
        ;;
    logs)
        docker-compose logs -f
        ;;
    up|start)
        ./start.sh
        ;;
    *)
        echo "Usage: ./dev.sh [command]"
        echo ""
        echo "Commands:"
        echo "  (none), shell  컨테이너 쉘 접속"
        echo "  run, drive     드라이버 실행"
        echo "  test           테스트 실행"
        echo "  slam           SLAM 실행"
        echo "  up, start      환경 시작"
        echo "  stop, down     전체 종료"
        echo "  logs           로그 확인"
        ;;
esac
