#!/bin/bash
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass() { echo -e "${GREEN}[PASS]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
info() { echo -e "       $1"; }

echo "============================================"
echo "  FSDS Autonomous Driving - Preflight Check"
echo "============================================"
echo ""

CHECKS_PASSED=0
CHECKS_FAILED=0

check_docker_services() {
    echo "1. Docker Services"
    echo "-------------------"
    
    for svc in fsds_roscore fsds_bridge fsds_dev; do
        if docker ps --format '{{.Names}}' | grep -q "^${svc}$"; then
            pass "$svc is running"
            ((CHECKS_PASSED++))
        else
            fail "$svc is NOT running"
            ((CHECKS_FAILED++))
        fi
    done
    echo ""
}

check_ros_master() {
    echo "2. ROS Master"
    echo "--------------"
    
    if docker exec fsds_dev rostopic list >/dev/null 2>&1; then
        pass "ROS Master is reachable"
        ((CHECKS_PASSED++))
    else
        fail "ROS Master is NOT reachable"
        ((CHECKS_FAILED++))
        return
    fi
    echo ""
}

check_fsds_topics() {
    echo "3. FSDS Simulator Topics"
    echo "-------------------------"
    
    REQUIRED_TOPICS=(
        "/fsds/lidar/Lidar1"
        "/fsds/testing_only/odom"
    )
    
    TOPICS=$(docker exec fsds_dev rostopic list 2>/dev/null || echo "")
    
    for topic in "${REQUIRED_TOPICS[@]}"; do
        if echo "$TOPICS" | grep -q "^${topic}$"; then
            pass "$topic available"
            ((CHECKS_PASSED++))
        else
            fail "$topic NOT found - Is FSDS simulator running?"
            ((CHECKS_FAILED++))
        fi
    done
    echo ""
}

check_driver_topics() {
    echo "4. Driver Debug Topics"
    echo "-----------------------"
    
    DEBUG_TOPICS=(
        "/debug/state"
        "/debug/speed"
        "/debug/steering"
    )
    
    TOPICS=$(docker exec fsds_dev rostopic list 2>/dev/null || echo "")
    
    for topic in "${DEBUG_TOPICS[@]}"; do
        if echo "$TOPICS" | grep -q "^${topic}$"; then
            pass "$topic publishing"
            ((CHECKS_PASSED++))
        else
            warn "$topic not found - Driver may not be running"
        fi
    done
    echo ""
}

check_v2x_topics() {
    echo "5. V2X Topics"
    echo "--------------"
    
    V2X_TOPICS=(
        "/v2x/speed_limit"
        "/v2x/hazard"
        "/v2x/stop_zone"
    )
    
    TOPICS=$(docker exec fsds_dev rostopic list 2>/dev/null || echo "")
    
    for topic in "${V2X_TOPICS[@]}"; do
        if echo "$TOPICS" | grep -q "^${topic}$"; then
            pass "$topic available"
            ((CHECKS_PASSED++))
        else
            warn "$topic not found - V2X RSU may not be running"
        fi
    done
    echo ""
}

check_slam_topics() {
    echo "6. SLAM Topics"
    echo "---------------"
    
    SLAM_TOPICS=(
        "/slam/map"
        "/slam/path"
    )
    
    TOPICS=$(docker exec fsds_dev rostopic list 2>/dev/null || echo "")
    
    for topic in "${SLAM_TOPICS[@]}"; do
        if echo "$TOPICS" | grep -q "^${topic}$"; then
            pass "$topic available"
            ((CHECKS_PASSED++))
        else
            warn "$topic not found - SLAM may not be running"
        fi
    done
    echo ""
}

sample_lidar() {
    echo "7. LiDAR Data Sample"
    echo "---------------------"
    
    LIDAR_HZ=$(docker exec fsds_dev timeout 3 rostopic hz /fsds/lidar/Lidar1 2>&1 | grep "average rate" | head -1 || echo "")
    
    if [ -n "$LIDAR_HZ" ]; then
        pass "LiDAR publishing: $LIDAR_HZ"
        ((CHECKS_PASSED++))
    else
        fail "LiDAR data not received within 3 seconds"
        ((CHECKS_FAILED++))
    fi
    echo ""
}

sample_odom() {
    echo "8. Odometry Data Sample"
    echo "------------------------"
    
    ODOM_HZ=$(docker exec fsds_dev timeout 3 rostopic hz /fsds/testing_only/odom 2>&1 | grep "average rate" | head -1 || echo "")
    
    if [ -n "$ODOM_HZ" ]; then
        pass "Odometry publishing: $ODOM_HZ"
        ((CHECKS_PASSED++))
    else
        fail "Odometry data not received within 3 seconds"
        ((CHECKS_FAILED++))
    fi
    echo ""
}

print_summary() {
    echo "============================================"
    echo "                  SUMMARY"
    echo "============================================"
    echo -e "Passed: ${GREEN}${CHECKS_PASSED}${NC}"
    echo -e "Failed: ${RED}${CHECKS_FAILED}${NC}"
    echo ""
    
    if [ $CHECKS_FAILED -eq 0 ]; then
        echo -e "${GREEN}All critical checks passed! Ready for demo.${NC}"
        return 0
    else
        echo -e "${RED}Some checks failed. Please fix before demo.${NC}"
        return 1
    fi
}

print_quick_commands() {
    echo ""
    echo "============================================"
    echo "              QUICK COMMANDS"
    echo "============================================"
    echo "Enter dev container:"
    echo "  docker exec -it fsds_dev bash"
    echo ""
    echo "Run competition driver:"
    echo "  python3 /root/catkin_ws/src/fsds_scripts/scripts/competition_driver.py"
    echo ""
    echo "Run V2X RSU (interactive):"
    echo "  python3 /root/catkin_ws/src/fsds_scripts/scripts/v2x_rsu.py -i"
    echo ""
    echo "Run V2X RSU (demo sequence):"
    echo "  python3 /root/catkin_ws/src/fsds_scripts/scripts/v2x_rsu.py -d"
    echo ""
    echo "Run SLAM:"
    echo "  python3 /root/catkin_ws/src/fsds_scripts/scripts/simple_slam.py"
    echo ""
    echo "Monitor driver state:"
    echo "  rostopic echo /debug/commentary"
    echo ""
}

check_docker_services
check_ros_master
check_fsds_topics
check_driver_topics
check_v2x_topics
check_slam_topics
sample_lidar
sample_odom
print_summary
print_quick_commands
