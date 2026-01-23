#!/usr/bin/env python3
"""
FSDS Competition Driver - 자율주행 시뮬레이션 포뮬러 경진대회 메인 드라이버 (모듈화 버전)

모듈화된 구조:
    - src/control/pure_pursuit.py: Pure Pursuit 조향 알고리즘
    - src/control/speed.py: 곡률 기반 속도 제어
    - src/perception/cone_detector.py: 콘 검출 및 중앙선 생성
    - src/utils/watchdog.py: 안전 상태머신

=============================================================================
ROS 토픽 (Topics)
=============================================================================
Subscribe:
    /fsds/lidar/Lidar1 (PointCloud2)     - LiDAR 포인트클라우드
    /fsds/testing_only/odom (Odometry)   - 차량 위치/속도
    /v2x/speed_limit (Float32)           - V2X 속도 제한
    /v2x/hazard (Bool)                   - V2X 위험 경고
    /v2x/stop_zone (Bool)                - V2X 정지 구역

Publish:
    /fsds/control_command (ControlCommand) - 차량 제어 (throttle/steering/brake)
    /debug/* (std_msgs)                    - 텔레메트리

Author: HYCU Autonomous Driving Team
Date: 2025-01
"""
import threading
from math import sqrt
from typing import List, Dict, Optional

import numpy as np
import rospy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Int32, String
import sensor_msgs.point_cloud2 as pc2

# Local module imports
from src.control.pure_pursuit import get_lookahead_point, pure_pursuit_steering
from src.control.speed import (
    apply_rate_limit,
    calculate_target_speed,
    calculate_throttle,
    estimate_curvature,
)
from src.perception.cone_detector import (
    build_centerline,
    find_cones_filtered,
    update_track_width,
)
from src.utils.watchdog import DriveState, StopReason, Watchdog


class CompetitionDriver:
    """경진대회용 자율주행 드라이버 (모듈화 버전).
    
    Pure Pursuit 조향, 곡률 기반 속도 제어, V2X 통합, Watchdog 안전 시스템을
    통합한 메인 드라이버입니다.
    """
    
    def __init__(self):
        rospy.init_node('competition_driver')
        
        # ROS Parameters
        self.max_throttle = rospy.get_param('~max_throttle', 0.25)
        self.min_speed = rospy.get_param('~min_speed', 2.0)
        self.max_speed = rospy.get_param('~max_speed', 6.0)
        self.max_steering = rospy.get_param('~max_steering', 0.4)
        self.wheelbase = rospy.get_param('~wheelbase', 1.5)
        self.lookahead_base = rospy.get_param('~lookahead_base', 4.0)
        self.lookahead_speed_gain = rospy.get_param('~lookahead_speed_gain', 0.5)
        
        # Cone detection parameters
        self.cones_range_cutoff = rospy.get_param('~cones_range_cutoff', 12.0)
        self.cone_min_z = rospy.get_param('~cone_min_z', -0.3)
        self.cone_max_z = rospy.get_param('~cone_max_z', 0.5)
        self.cone_grouping_threshold = rospy.get_param('~cone_grouping_threshold', 0.2)
        self.cone_min_points = rospy.get_param('~cone_min_points', 3)
        self.cone_max_radius = rospy.get_param('~cone_max_radius', 0.3)
        self.cone_max_radius_std = rospy.get_param('~cone_max_radius_std', 0.15)
        
        # Rate limits
        self.throttle_rate_limit = rospy.get_param('~throttle_rate_limit', 2.0)
        self.steering_rate_limit = rospy.get_param('~steering_rate_limit', 3.0)
        self.control_dt = 1.0 / 20.0
        
        # Max lateral acceleration for curvature-based speed control
        self.max_lateral_accel = rospy.get_param('~max_lateral_accel', 4.0)
        
        # Watchdog
        self.watchdog = Watchdog(
            lidar_stale_timeout=rospy.get_param('~lidar_stale_timeout', 1.0),
            odom_stale_timeout=rospy.get_param('~odom_stale_timeout', 1.0),
            recovery_timeout=rospy.get_param('~recovery_timeout', 1.0),
            degraded_timeout=rospy.get_param('~degraded_timeout', 1.0),
            stopping_timeout=rospy.get_param('~stopping_timeout', 3.0)
        )
        
        # Publishers
        self.cmd_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        self.debug_state_pub = rospy.Publisher('/debug/state', String, queue_size=1)
        self.debug_speed_pub = rospy.Publisher('/debug/speed', Float32, queue_size=1)
        self.debug_target_speed_pub = rospy.Publisher('/debug/target_speed', Float32, queue_size=1)
        self.debug_curvature_pub = rospy.Publisher('/debug/curvature', Float32, queue_size=1)
        self.debug_steering_pub = rospy.Publisher('/debug/steering', Float32, queue_size=1)
        self.debug_cone_count_pub = rospy.Publisher('/debug/cone_count', Int32, queue_size=1)
        self.debug_stop_reason_pub = rospy.Publisher('/debug/stop_reason', String, queue_size=1)
        self.debug_commentary_pub = rospy.Publisher('/debug/commentary', String, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/v2x/speed_limit', Float32, self.v2x_speed_limit_callback)
        rospy.Subscriber('/v2x/hazard', Bool, self.v2x_hazard_callback)
        rospy.Subscriber('/v2x/stop_zone', Bool, self.v2x_stop_zone_callback)
        rospy.Subscriber('/debug/e_stop', Bool, self.estop_callback)
        
        # State variables
        self.current_speed = 0.0
        self.left_cones: List[Dict] = []
        self.right_cones: List[Dict] = []
        self.centerline: List[Dict[str, float]] = []
        self.last_valid_centerline: List[Dict[str, float]] = []
        self.last_valid_track_width = 4.0
        self.curvature_history: List[float] = []
        
        self.current_curvature = 0.0
        self.current_target_speed = self.max_speed
        self.current_steering = 0.0
        
        self.last_throttle = 0.0
        self.last_steering = 0.0
        
        # V2X state
        self.v2x_speed_limit = self.max_speed
        self.v2x_hazard = False
        
        # Sensor ready flags
        self.sensors_ready = False
        self.lidar_received = False
        self.odom_received = False
        
        # Thread safety
        self.data_lock = threading.Lock()
        self.last_log_time = 0.0
    
    def add_commentary(self, text: str) -> None:
        """주행 상태 코멘트 로깅 및 발행."""
        rospy.loginfo(f"[COMMENTARY] {text}")
        self.debug_commentary_pub.publish(String(data=text))
    
    def v2x_speed_limit_callback(self, msg: Float32) -> None:
        """V2X 속도 제한 콜백."""
        if np.isfinite(msg.data) and msg.data >= 0:
            self.v2x_speed_limit = msg.data
            self.add_commentary(f"V2X: speed limit {msg.data:.1f} m/s")
    
    def v2x_hazard_callback(self, msg: Bool) -> None:
        """V2X 위험 경고 콜백."""
        if msg.data != self.v2x_hazard:
            self.v2x_hazard = msg.data
            if msg.data:
                self.add_commentary("V2X: hazard ahead, slowing down")
            else:
                self.add_commentary("V2X: hazard cleared")
    
    def v2x_stop_zone_callback(self, msg: Bool) -> None:
        """V2X 정지 구역 콜백."""
        self.watchdog.set_v2x_stop_zone(msg.data)
        if msg.data:
            self.add_commentary("V2X: entering stop zone")
        else:
            self.add_commentary("V2X: exiting stop zone")
    
    def estop_callback(self, msg: Bool) -> None:
        """비상 정지 콜백."""
        self.watchdog.set_estop(msg.data)
        if msg.data:
            self.add_commentary("EMERGENCY STOP activated (latched - restart required)")
    
    def lidar_callback(self, msg: PointCloud2) -> None:
        """LiDAR 포인트클라우드 콜백."""
        try:
            self.watchdog.update_lidar_time()
            self.lidar_received = True
            
            points = np.array(list(pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )))
            
            if len(points) == 0:
                with self.data_lock:
                    self.left_cones = []
                    self.right_cones = []
                    self.centerline = []
                return
            
            # Filter invalid points
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                points = points[~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)]
            
            left, right = find_cones_filtered(
                points,
                cone_min_z=self.cone_min_z,
                cone_max_z=self.cone_max_z,
                cones_range_cutoff=self.cones_range_cutoff,
                cone_grouping_threshold=self.cone_grouping_threshold,
                cone_min_points=self.cone_min_points,
                cone_max_radius=self.cone_max_radius,
                cone_max_radius_std=self.cone_max_radius_std
            )
            
            center = build_centerline(left, right, self.last_valid_track_width)
            
            with self.data_lock:
                self.left_cones = left
                self.right_cones = right
                self.centerline = center
                if center:
                    self.last_valid_centerline = center
                
                # Update track width
                self.last_valid_track_width = update_track_width(
                    left, right, self.last_valid_track_width
                )
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"LiDAR callback error: {e}")
    
    def odom_callback(self, msg: Odometry) -> None:
        """Odometry 콜백."""
        try:
            self.watchdog.update_odom_time()
            self.odom_received = True
            
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            speed = sqrt(vx**2 + vy**2)
            
            if np.isfinite(speed):
                self.current_speed = speed
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Odom callback error: {e}")
    
    def publish_telemetry(self) -> None:
        """텔레메트리 발행."""
        state = self.watchdog.get_state()
        stop_reason = self.watchdog.get_stop_reason()
        
        self.debug_state_pub.publish(String(data=state.name))
        self.debug_speed_pub.publish(Float32(data=self.current_speed))
        self.debug_target_speed_pub.publish(Float32(data=self.current_target_speed))
        self.debug_curvature_pub.publish(Float32(data=self.current_curvature))
        self.debug_steering_pub.publish(Float32(data=self.current_steering))
        self.debug_cone_count_pub.publish(Int32(data=len(self.left_cones) + len(self.right_cones)))
        self.debug_stop_reason_pub.publish(String(data=stop_reason.name))
    
    def compute_control(
        self,
        centerline_snapshot: List[Dict[str, float]],
        last_valid_centerline_snapshot: List[Dict[str, float]]
    ) -> ControlCommand:
        """제어 명령 계산."""
        cmd = ControlCommand()
        state = self.watchdog.get_state()
        
        if state == DriveState.STOPPING:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake = 1.0
            self.current_target_speed = 0.0
            self.current_steering = 0.0
            return cmd
        
        if state == DriveState.DEGRADED:
            if not last_valid_centerline_snapshot:
                cmd.throttle = 0.0
                cmd.steering = 0.0
                cmd.brake = 1.0
                self.current_target_speed = 0.0
                return cmd
            
            centerline = last_valid_centerline_snapshot
            self.current_target_speed = self.min_speed
            
            speed_error = self.current_speed - self.current_target_speed
            if speed_error > 0.5:
                cmd.brake = min(0.3, speed_error * 0.2)
                cmd.throttle = 0.0
                target_point = get_lookahead_point(
                    centerline, self.current_speed,
                    self.lookahead_base, self.lookahead_speed_gain
                )
                cmd.steering = pure_pursuit_steering(
                    target_point, self.wheelbase, self.max_steering
                )
                self.current_steering = cmd.steering
                return cmd
        else:
            # TRACKING mode
            centerline = centerline_snapshot
            if not centerline:
                if last_valid_centerline_snapshot:
                    centerline = last_valid_centerline_snapshot
                    self.current_curvature = estimate_curvature(
                        centerline, self.curvature_history
                    )
                    self.current_target_speed = self.min_speed
                else:
                    cmd.throttle = 0.0
                    cmd.steering = 0.0
                    cmd.brake = 1.0
                    self.current_target_speed = 0.0
                    return cmd
            else:
                self.current_curvature = estimate_curvature(
                    centerline, self.curvature_history
                )
                self.current_target_speed = calculate_target_speed(
                    self.current_curvature,
                    max_lateral_accel=self.max_lateral_accel,
                    min_speed=self.min_speed,
                    max_speed=self.max_speed,
                    v2x_speed_limit=self.v2x_speed_limit,
                    v2x_hazard=self.v2x_hazard
                )
        
        target_point = get_lookahead_point(
            centerline, self.current_speed,
            self.lookahead_base, self.lookahead_speed_gain
        )
        
        self.current_steering = pure_pursuit_steering(
            target_point, self.wheelbase, self.max_steering
        )
        raw_throttle = calculate_throttle(
            self.current_target_speed, self.current_speed, self.max_throttle
        )
        
        cmd.steering = apply_rate_limit(
            self.current_steering, self.last_steering,
            self.steering_rate_limit, self.control_dt
        )
        cmd.throttle = apply_rate_limit(
            raw_throttle, self.last_throttle,
            self.throttle_rate_limit, self.control_dt
        )
        
        self.last_steering = cmd.steering
        self.last_throttle = cmd.throttle
        
        # Active braking
        if self.current_target_speed < 0.5:
            cmd.throttle = 0.0
            cmd.brake = 1.0
        elif self.current_speed > self.current_target_speed + 1.0:
            cmd.brake = min(0.5, (self.current_speed - self.current_target_speed) * 0.3)
        else:
            cmd.brake = 0.0
        
        return cmd
    
    def run(self) -> None:
        """메인 제어 루프."""
        rate = rospy.Rate(20)
        rospy.loginfo("=" * 50)
        rospy.loginfo("Competition Driver v2.2 (Modular)")
        rospy.loginfo("Features: Pure Pursuit + Curvature Speed + V2X + Watchdog")
        rospy.loginfo("=" * 50)
        self.add_commentary("Driver initialized, waiting for sensor data")
        
        while not rospy.is_shutdown():
            try:
                if not self.sensors_ready:
                    if self.lidar_received and self.odom_received:
                        self.sensors_ready = True
                        self.add_commentary("Sensors ready, waiting for first valid centerline")
                    else:
                        cmd = ControlCommand()
                        cmd.throttle = 0.0
                        cmd.steering = 0.0
                        cmd.brake = 1.0
                        self.cmd_pub.publish(cmd)
                        rate.sleep()
                        continue
                
                with self.data_lock:
                    has_valid_centerline = len(self.last_valid_centerline) > 0
                
                if not has_valid_centerline:
                    cmd = ControlCommand()
                    cmd.throttle = 0.0
                    cmd.steering = 0.0
                    cmd.brake = 1.0
                    self.cmd_pub.publish(cmd)
                    rate.sleep()
                    continue
                
                # Check watchdog
                self.watchdog.check_watchdog()
                
                # Get data snapshots
                with self.data_lock:
                    left_snapshot = list(self.left_cones)
                    right_snapshot = list(self.right_cones)
                    centerline_snapshot = list(self.centerline)
                    last_valid_centerline_snapshot = list(self.last_valid_centerline)
                
                # Update cone state
                self.watchdog.update_cone_state(len(left_snapshot), len(right_snapshot))
                
                # Compute and publish control
                cmd = self.compute_control(centerline_snapshot, last_valid_centerline_snapshot)
                self.cmd_pub.publish(cmd)
                self.publish_telemetry()
                
                # Periodic logging
                now_sec = rospy.Time.now().to_sec()
                if now_sec - self.last_log_time >= 2.0:
                    self.last_log_time = now_sec
                    state = self.watchdog.get_state()
                    rospy.loginfo(
                        f"[{state.name}] Speed: {self.current_speed:.1f}/{self.current_target_speed:.1f} m/s | "
                        f"Steer: {self.current_steering:.2f} | L:{len(self.left_cones)} R:{len(self.right_cones)}"
                    )
            
            except Exception as e:
                rospy.logerr(f"Main loop error: {e}")
                cmd = ControlCommand()
                cmd.throttle = 0.0
                cmd.steering = 0.0
                cmd.brake = 1.0
                self.cmd_pub.publish(cmd)
            
            rate.sleep()


if __name__ == '__main__':
    try:
        driver = CompetitionDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
