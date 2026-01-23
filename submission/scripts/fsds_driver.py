#!/usr/bin/env python3
"""
FSDS Autonomous Driver - Standalone Python Version
===================================================
ROS/Docker 없이 FSDS 시뮬레이터에 직접 연결하여 자율주행 수행.

사용법:
    pip install fsds numpy
    python fsds_driver.py

FSDS.exe 실행 후 이 스크립트 실행하면 자동 연결됩니다.
"""

import sys
import time
import math
import threading
from enum import Enum
from collections import deque
from typing import List, Dict, Tuple, Optional

try:
    import numpy as np
except ImportError:
    print("numpy 설치 필요: pip install numpy")
    sys.exit(1)

try:
    import fsds
except ImportError:
    print("fsds 설치 필요: pip install fsds")
    sys.exit(1)


# =============================================================================
# 상태 정의
# =============================================================================

class DriveState(Enum):
    TRACKING = 1      # 정상 주행
    DEGRADED = 2      # 저하 모드 (콘 일부 손실)
    STOPPING = 3      # 정지 중


class StopReason(Enum):
    NONE = 0
    CONES_LOST = 1
    LIDAR_STALE = 2
    EMERGENCY_STOP = 3


# =============================================================================
# Pure Pursuit 알고리즘
# =============================================================================

def pure_pursuit_steering(path: List[Tuple[float, float]], 
                          lookahead_distance: float,
                          wheelbase: float = 1.5) -> float:
    """
    Pure Pursuit 조향 계산.
    
    Args:
        path: [(x, y), ...] 경로 점들 (차량 좌표계)
        lookahead_distance: 전방 주시 거리 (m)
        wheelbase: 휠베이스 (m)
    
    Returns:
        steering: 조향각 (-1.0 ~ 1.0)
    """
    if not path or len(path) < 2:
        return 0.0
    
    # Lookahead point 찾기
    target = None
    for point in path:
        dist = math.sqrt(point[0]**2 + point[1]**2)
        if dist >= lookahead_distance:
            target = point
            break
    
    if target is None:
        target = path[-1]
    
    # Pure Pursuit 공식: δ = atan2(2 * L * sin(α), ld)
    x, y = target
    ld = math.sqrt(x**2 + y**2)
    if ld < 0.1:
        return 0.0
    
    alpha = math.atan2(y, x)
    steering = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
    
    # 정규화 (-1 ~ 1)
    max_steer = 0.5  # rad
    return np.clip(steering / max_steer, -1.0, 1.0)


# =============================================================================
# 콘 탐지 및 경로 생성
# =============================================================================

def find_cones_from_lidar(points: np.ndarray, 
                          max_range: float = 20.0,
                          min_height: float = -0.3,
                          max_height: float = 0.5) -> Tuple[List[Dict], List[Dict]]:
    """
    LiDAR 포인트에서 콘 추출.
    
    Returns:
        (left_cones, right_cones): 좌/우 콘 리스트
    """
    if points is None or len(points) == 0:
        return [], []
    
    # 포인트 필터링 (높이, 거리)
    points = np.array(points).reshape(-1, 3)
    
    mask = (
        (points[:, 2] > min_height) & 
        (points[:, 2] < max_height) &
        (np.linalg.norm(points[:, :2], axis=1) < max_range) &
        (points[:, 0] > 0.5)  # 전방만
    )
    filtered = points[mask]
    
    if len(filtered) < 3:
        return [], []
    
    # 간단한 Grid 기반 클러스터링
    grid_size = 0.5
    grid = {}
    
    for p in filtered:
        key = (int(p[0] / grid_size), int(p[1] / grid_size))
        if key not in grid:
            grid[key] = []
        grid[key].append(p)
    
    # 클러스터 중심 추출
    cones = []
    for key, pts in grid.items():
        if len(pts) >= 3:  # 최소 포인트 수
            center = np.mean(pts, axis=0)
            cones.append({
                'x': float(center[0]),
                'y': float(center[1]),
                'z': float(center[2]),
                'points': len(pts)
            })
    
    # 좌/우 분리
    left_cones = [c for c in cones if c['y'] > 0]
    right_cones = [c for c in cones if c['y'] <= 0]
    
    # 거리순 정렬
    left_cones.sort(key=lambda c: c['x'])
    right_cones.sort(key=lambda c: c['x'])
    
    return left_cones, right_cones


def build_centerline(left_cones: List[Dict], 
                     right_cones: List[Dict],
                     default_width: float = 3.0) -> List[Tuple[float, float]]:
    """좌/우 콘에서 중앙선 생성."""
    path = []
    
    if left_cones and right_cones:
        # 양쪽 콘 있음 - 중앙 계산
        for i in range(min(len(left_cones), len(right_cones))):
            cx = (left_cones[i]['x'] + right_cones[i]['x']) / 2
            cy = (left_cones[i]['y'] + right_cones[i]['y']) / 2
            path.append((cx, cy))
    elif left_cones:
        # 왼쪽만 - 오프셋
        for c in left_cones:
            path.append((c['x'], c['y'] - default_width / 2))
    elif right_cones:
        # 오른쪽만 - 오프셋
        for c in right_cones:
            path.append((c['x'], c['y'] + default_width / 2))
    
    return path


# =============================================================================
# 속도 제어
# =============================================================================

def calculate_target_speed(path: List[Tuple[float, float]],
                           min_speed: float = 2.0,
                           max_speed: float = 6.0,
                           max_lateral_accel: float = 4.0) -> float:
    """곡률 기반 목표 속도 계산."""
    if len(path) < 3:
        return min_speed
    
    # 곡률 추정 (3점 원)
    p1, p2, p3 = np.array(path[0]), np.array(path[1]), np.array(path[2])
    
    a = np.linalg.norm(p2 - p1)
    b = np.linalg.norm(p3 - p2)
    c = np.linalg.norm(p3 - p1)
    
    if a < 0.1 or b < 0.1 or c < 0.1:
        return max_speed
    
    # 삼각형 넓이
    s = (a + b + c) / 2
    area_sq = s * (s - a) * (s - b) * (s - c)
    
    if area_sq <= 0:
        return max_speed
    
    area = math.sqrt(area_sq)
    
    # 곡률 = 4 * area / (a * b * c)
    curvature = 4 * area / (a * b * c)
    
    if curvature < 0.01:
        return max_speed
    
    # v = sqrt(a_lat / curvature)
    target = math.sqrt(max_lateral_accel / curvature)
    return np.clip(target, min_speed, max_speed)


def speed_to_throttle(current_speed: float, 
                      target_speed: float,
                      max_throttle: float = 0.5) -> float:
    """속도 차이 기반 스로틀 계산."""
    error = target_speed - current_speed
    
    if error > 1.0:
        return max_throttle
    elif error > 0:
        return max_throttle * error
    else:
        return 0.0


# =============================================================================
# 메인 드라이버
# =============================================================================

class FSDSDriver:
    """FSDS 자율주행 드라이버."""
    
    def __init__(self, host: str = "localhost"):
        self.host = host
        self.client: Optional[fsds.FSDSClient] = None
        self.running = False
        
        # 상태
        self.state = DriveState.TRACKING
        self.stop_reason = StopReason.NONE
        
        # 파라미터
        self.max_throttle = 0.3
        self.max_steering = 0.5
        self.min_speed = 2.0
        self.max_speed = 6.0
        self.lookahead_base = 4.0
        self.control_rate = 20  # Hz
        
        # 타이밍
        self.last_lidar_time = 0.0
        self.last_valid_time = 0.0
        self.lidar_timeout = 1.0
        self.cone_timeout = 3.0
        
        # 락
        self.lock = threading.Lock()
        
        # 텔레메트리
        self.telemetry = deque(maxlen=100)
    
    def connect(self, max_retries: int = 10, retry_delay: float = 2.0) -> bool:
        """FSDS에 연결 (자동 재시도)."""
        print(f"FSDS 연결 중... ({self.host})")
        
        for attempt in range(max_retries):
            try:
                self.client = fsds.FSDSClient()
                self.client.confirmConnection()
                self.client.enableApiControl(True)
                print(f"✓ FSDS 연결 성공! (시도 {attempt + 1}/{max_retries})")
                return True
            except Exception as e:
                print(f"  연결 실패 ({attempt + 1}/{max_retries}): {e}")
                time.sleep(retry_delay)
        
        print("✗ FSDS 연결 실패. FSDS.exe가 실행 중인지 확인하세요.")
        return False
    
    def get_lidar_points(self) -> Optional[np.ndarray]:
        """LiDAR 데이터 획득."""
        try:
            lidar_data = self.client.getLidarData(lidar_name='Lidar')
            if len(lidar_data.point_cloud) < 3:
                return None
            
            points = np.array(lidar_data.point_cloud).reshape(-1, 3)
            self.last_lidar_time = time.time()
            return points
        except Exception as e:
            print(f"LiDAR 오류: {e}")
            return None
    
    def get_car_state(self) -> Tuple[float, float, float]:
        """차량 상태 획득 (speed, x, y)."""
        try:
            state = self.client.getCarState()
            return state.speed, state.kinematics_estimated.position.x_val, \
                   state.kinematics_estimated.position.y_val
        except Exception as e:
            print(f"상태 오류: {e}")
            return 0.0, 0.0, 0.0
    
    def set_controls(self, throttle: float, steering: float, brake: float = 0.0):
        """차량 제어 명령 전송."""
        try:
            controls = fsds.CarControls()
            controls.throttle = float(np.clip(throttle, 0, self.max_throttle))
            controls.steering = float(np.clip(steering, -self.max_steering, self.max_steering))
            controls.brake = float(np.clip(brake, 0, 1))
            self.client.setCarControls(controls)
        except Exception as e:
            print(f"제어 오류: {e}")
    
    def check_watchdog(self) -> bool:
        """센서 상태 확인."""
        now = time.time()
        
        # LiDAR 타임아웃
        if now - self.last_lidar_time > self.lidar_timeout:
            if self.state != DriveState.STOPPING:
                print("⚠ LiDAR 타임아웃 - 정지")
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.LIDAR_STALE
            return False
        
        # 콘 손실 타임아웃
        if now - self.last_valid_time > self.cone_timeout:
            if self.state != DriveState.STOPPING:
                print("⚠ 콘 손실 - 정지")
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.CONES_LOST
            return False
        
        return True
    
    def control_loop(self):
        """메인 제어 루프."""
        period = 1.0 / self.control_rate
        
        while self.running:
            loop_start = time.time()
            
            try:
                # 1. 센서 데이터 획득
                points = self.get_lidar_points()
                speed, x, y = self.get_car_state()
                
                # 2. 콘 탐지
                if points is not None:
                    left_cones, right_cones = find_cones_from_lidar(points)
                else:
                    left_cones, right_cones = [], []
                
                # 3. 상태 업데이트
                total_cones = len(left_cones) + len(right_cones)
                has_both = len(left_cones) >= 1 and len(right_cones) >= 1
                has_strong = has_both or total_cones >= 3
                
                if has_strong:
                    self.last_valid_time = time.time()
                    if self.state == DriveState.STOPPING and self.stop_reason == StopReason.CONES_LOST:
                        self.state = DriveState.DEGRADED
                        self.stop_reason = StopReason.NONE
                        print("✓ 콘 복구 - 재개")
                    if has_both:
                        self.state = DriveState.TRACKING
                
                # 4. Watchdog 체크
                if not self.check_watchdog():
                    self.set_controls(0, 0, 1.0)  # 정지
                    time.sleep(period)
                    continue
                
                # 5. 경로 생성
                path = build_centerline(left_cones, right_cones)
                
                if not path:
                    self.set_controls(0, 0, 0.5)
                    time.sleep(period)
                    continue
                
                # 6. 제어 계산
                lookahead = self.lookahead_base + speed * 0.3
                steering = pure_pursuit_steering(path, lookahead)
                
                target_speed = calculate_target_speed(path, self.min_speed, self.max_speed)
                throttle = speed_to_throttle(speed, target_speed, self.max_throttle)
                
                # 7. 제어 명령 전송
                self.set_controls(throttle, steering)
                
                # 텔레메트리
                self.telemetry.append({
                    'time': time.time(),
                    'speed': speed,
                    'throttle': throttle,
                    'steering': steering,
                    'cones': total_cones,
                    'state': self.state.name
                })
                
            except Exception as e:
                print(f"루프 오류: {e}")
                self.set_controls(0, 0, 1.0)
            
            # 주기 유지
            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
    
    def run(self):
        """드라이버 실행."""
        if not self.connect():
            return
        
        self.running = True
        self.last_lidar_time = time.time()
        self.last_valid_time = time.time()
        
        print("\n" + "="*50)
        print("  FSDS 자율주행 드라이버 시작")
        print("  Ctrl+C로 종료")
        print("="*50 + "\n")
        
        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("\n종료 요청...")
        finally:
            self.running = False
            self.set_controls(0, 0, 1.0)
            print("드라이버 종료")
    
    def print_status(self):
        """상태 출력."""
        if self.telemetry:
            t = self.telemetry[-1]
            print(f"[{t['state']:10}] 속도: {t['speed']:.1f} m/s | "
                  f"스로틀: {t['throttle']:.2f} | 조향: {t['steering']:+.2f} | "
                  f"콘: {t['cones']}")


# =============================================================================
# 메인
# =============================================================================

if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════╗
║         FSDS Autonomous Driver - Standalone Edition       ║
║                                                           ║
║  1. FSDS.exe 실행                                         ║
║  2. 이 스크립트 실행: python fsds_driver.py               ║
║  3. 자동으로 연결 및 주행 시작                            ║
╚═══════════════════════════════════════════════════════════╝
    """)
    
    driver = FSDSDriver()
    driver.run()
