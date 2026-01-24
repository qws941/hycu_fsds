#!/usr/bin/env python3
"""
Watchdog 상태머신 모듈

자율주행 차량의 안전 상태를 관리하는 상태머신입니다.

상태:
    TRACKING: 정상 주행 (좌/우 콘 모두 탐지)
    DEGRADED: 부분 탐지 (한쪽 콘만 또는 일시적 미탐지)
    STOPPING: 안전 정지 (센서 stale, 장기 미탐지, V2X stop)

정지 사유:
    NONE: 정상
    CONES_LOST: 콘 장기 미탐지
    LIDAR_STALE: LiDAR 데이터 타임아웃
    ODOM_STALE: Odometry 데이터 타임아웃
    V2X_STOP_ZONE: V2X 정지 구역
    EMERGENCY_STOP: 비상 정지 (latched)
    RACE_COMPLETE: 레이스 완주

Author: HYCU Autonomous Driving Team
"""
from enum import Enum
from typing import Optional, Tuple
import time


# =============================================================================
# Enums (ROS 의존성 없음)
# =============================================================================

class DriveState(Enum):
    """주행 상태 열거형."""
    TRACKING = 1  # 정상 주행
    DEGRADED = 2  # 부분 탐지
    STOPPING = 3  # 안전 정지


class StopReason(Enum):
    """정지 사유 열거형."""
    NONE = 0
    CONES_LOST = 1
    LIDAR_STALE = 2
    ODOM_STALE = 3
    V2X_STOP_ZONE = 4
    EMERGENCY_STOP = 5
    RACE_COMPLETE = 6


# =============================================================================
# Watchdog Class (ROS 선택적 의존)
# =============================================================================

class Watchdog:
    """안전 상태 관리 워치독.
    
    센서 데이터 freshness, V2X 신호, E-stop 상태를 모니터링하여
    적절한 주행 상태를 결정합니다.
    
    ROS 환경에서는 rospy.Time 사용, 그 외에는 time.time() 사용.
    """
    
    def __init__(
        self,
        lidar_timeout: float = 1.0,
        odom_timeout: float = 1.0,
        recovery_timeout: float = 1.0,
        degraded_timeout: float = 1.0,
        stopping_timeout: float = 3.0,
        use_ros_time: bool = True
    ):
        """Watchdog 초기화.
        
        Args:
            lidar_timeout: LiDAR stale 판정 시간 (초)
            odom_timeout: Odometry stale 판정 시간 (초)
            recovery_timeout: STOPPING에서 복구 대기 시간 (초)
            degraded_timeout: 콘 미탐지 시 DEGRADED 전환 시간 (초)
            stopping_timeout: 콘 미탐지 시 STOPPING 전환 시간 (초)
            use_ros_time: True면 rospy.Time 사용, False면 time.time() 사용
        """
        self.lidar_stale_timeout = lidar_timeout
        self.odom_stale_timeout = odom_timeout
        self.recovery_timeout = recovery_timeout
        self.degraded_timeout = degraded_timeout
        self.stopping_timeout = stopping_timeout
        self.use_ros_time = use_ros_time
        
        self.state = DriveState.TRACKING
        self.stop_reason = StopReason.NONE
        
        self._initialized = False
        self._rospy = None
        
        self.last_lidar_time: Optional[float] = None
        self.last_odom_time: Optional[float] = None
        self.last_valid_time: Optional[float] = None
        self.recovery_start_time: Optional[float] = None
        
        self.e_stop = False
        self.e_stop_latched = False
        self.v2x_stop_zone = False
    
    def _get_time(self) -> float:
        """현재 시간 반환 (ROS 또는 시스템 시간)."""
        if self.use_ros_time:
            if self._rospy is None:
                try:
                    import rospy
                    self._rospy = rospy
                except ImportError:
                    self.use_ros_time = False
                    return time.time()
            return self._rospy.Time.now().to_sec()
        return time.time()
    
    def _ensure_initialized(self) -> None:
        """시간 초기화 (lazy initialization)."""
        if not self._initialized:
            now = self._get_time()
            self.last_lidar_time = now
            self.last_odom_time = now
            self.last_valid_time = now
            self._initialized = True
    
    def update_sensor(self, sensor_type: str) -> None:
        """센서 데이터 수신 시간 갱신.
        
        Args:
            sensor_type: 'lidar' 또는 'odom'
        """
        self._ensure_initialized()
        now = self._get_time()
        if sensor_type == 'lidar':
            self.last_lidar_time = now
        elif sensor_type == 'odom':
            self.last_odom_time = now
    
    def update_lidar_time(self) -> None:
        """LiDAR 데이터 수신 시간 갱신."""
        self.update_sensor('lidar')
    
    def update_odom_time(self) -> None:
        """Odometry 데이터 수신 시간 갱신."""
        self.update_sensor('odom')
    
    def set_estop(self, active: bool) -> None:
        """E-stop 상태 설정.
        
        E-stop이 활성화되면 latched 상태가 되어 노드 재시작 전까지 유지됩니다.
        """
        if active and not self.e_stop:
            self.e_stop = True
            self.e_stop_latched = True
        self.e_stop = active
    
    def set_v2x_stop_zone(self, active: bool) -> None:
        """V2X 정지 구역 상태 설정."""
        self.v2x_stop_zone = active
    
    def check_watchdog(self) -> Tuple[DriveState, StopReason]:
        """센서 상태 및 안전 조건 확인.
        
        Returns:
            (현재 상태, 정지 사유) 튜플
        """
        self._ensure_initialized()
        now = self._get_time()
        
        # E-stop latch check - requires node restart to clear
        if self.e_stop_latched:
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.EMERGENCY_STOP
            self.recovery_start_time = None
            return self.state, self.stop_reason
        
        if self.e_stop:
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.EMERGENCY_STOP
            self.recovery_start_time = None
            return self.state, self.stop_reason
        
        if self.v2x_stop_zone:
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.V2X_STOP_ZONE
            self.recovery_start_time = None
            return self.state, self.stop_reason
        
        # V2X stop zone cleared - auto recover
        if (self.state == DriveState.STOPPING and 
            self.stop_reason == StopReason.V2X_STOP_ZONE):
            self.state = DriveState.DEGRADED
            self.stop_reason = StopReason.NONE
            self.recovery_start_time = None
            return self.state, self.stop_reason
        
        # LiDAR stale check
        assert self.last_lidar_time is not None  # Guaranteed by _ensure_initialized
        lidar_elapsed = now - self.last_lidar_time
        if lidar_elapsed > self.lidar_stale_timeout:
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.LIDAR_STALE
            self.recovery_start_time = None
            return self.state, self.stop_reason
        
        # Odometry stale check
        assert self.last_odom_time is not None  # Guaranteed by _ensure_initialized
        odom_elapsed = now - self.last_odom_time
        if odom_elapsed > self.odom_stale_timeout:
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.ODOM_STALE
            self.recovery_start_time = None
            return self.state, self.stop_reason
        
        # Sensor recovery logic
        if (self.state == DriveState.STOPPING and 
            self.stop_reason in [StopReason.LIDAR_STALE, StopReason.ODOM_STALE]):
            if self.recovery_start_time is None:
                self.recovery_start_time = now
            elif (now - self.recovery_start_time) >= self.recovery_timeout:
                self.state = DriveState.DEGRADED
                self.stop_reason = StopReason.NONE
                self.recovery_start_time = None
        
        return self.state, self.stop_reason
    
    def update_cone_state(
        self,
        left_cone_count: int,
        right_cone_count: int
    ) -> Tuple[DriveState, StopReason]:
        """콘 탐지 상태에 따른 상태 업데이트.
        
        Args:
            left_cone_count: 좌측 콘 개수
            right_cone_count: 우측 콘 개수
        
        Returns:
            (현재 상태, 정지 사유) 튜플
        """
        self._ensure_initialized()
        now = self._get_time()
        
        # Safety gate: never override safety-critical stop reasons
        safety_stop_reasons = {
            StopReason.LIDAR_STALE, 
            StopReason.ODOM_STALE,
            StopReason.V2X_STOP_ZONE, 
            StopReason.EMERGENCY_STOP,
            StopReason.RACE_COMPLETE
        }
        if self.state == DriveState.STOPPING and self.stop_reason in safety_stop_reasons:
            return self.state, self.stop_reason
        
        has_valid_cones = left_cone_count >= 1 or right_cone_count >= 1
        has_both_sides = left_cone_count >= 1 and right_cone_count >= 1
        has_strong_perception = has_both_sides or (left_cone_count + right_cone_count >= 3)
        
        if has_valid_cones:
            if (self.state == DriveState.STOPPING and 
                self.stop_reason == StopReason.CONES_LOST):
                self.state = DriveState.DEGRADED
                self.stop_reason = StopReason.NONE
            elif self.state != DriveState.TRACKING and has_both_sides:
                self.state = DriveState.TRACKING
                self.stop_reason = StopReason.NONE
            elif has_both_sides:
                self.state = DriveState.TRACKING
                self.stop_reason = StopReason.NONE
            
            if has_strong_perception:
                self.last_valid_time = now
        else:
            if self.state == DriveState.STOPPING:
                return self.state, self.stop_reason
            
            assert self.last_valid_time is not None  # Guaranteed by _ensure_initialized
            elapsed = now - self.last_valid_time
            
            if elapsed > self.stopping_timeout:
                self.state = DriveState.STOPPING
                self.stop_reason = StopReason.CONES_LOST
            elif elapsed > self.degraded_timeout:
                self.state = DriveState.DEGRADED
        
        return self.state, self.stop_reason
    
    def get_state(self) -> Tuple[DriveState, StopReason]:
        """현재 상태와 정지 사유 반환."""
        return self.state, self.stop_reason
    
    def get_stop_reason(self) -> StopReason:
        """현재 정지 사유 반환."""
        return self.stop_reason
