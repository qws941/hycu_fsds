#!/usr/bin/env python3
"""
FSDS Competition Driver - 자율주행 시뮬레이션 포뮬러 경진대회 메인 드라이버

=============================================================================
시스템 구성 (Architecture)
=============================================================================
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
│ Perception  │ -> │   Planning   │ -> │   Control   │ -> │    Safety    │
│ (LiDAR/Odom)│    │ (Pure Pursuit)│    │ (Throttle/  │    │  (Watchdog/  │
│             │    │              │    │  Steering)  │    │   E-Stop)    │
└─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘

=============================================================================
알고리즘 (Algorithms)
=============================================================================
1. 콘 검출 (Cone Detection)
   - Grid 기반 클러스터링 O(N): 인접 셀 BFS로 포인트 그룹화
   - Z-height 필터: 지면(-0.3m) ~ 콘 상단(0.5m) 범위만 추출
   - 품질 검증: 클러스터 반경, 분산, 최소 포인트 수 확인

2. Pure Pursuit 조향 (Steering)
   - 기하학적 경로 추종 알고리즘 (Coulter, 1992)
   - Lookahead 거리: L = L_base + v * gain (속도 비례 조정)
   - 조향각: δ = atan2(2 * wheelbase * sin(α) / L)
   - Arc-length 기반 lookahead point 선택으로 커브 안정성 확보

3. 곡률 기반 속도 제어 (Speed Control)
   - 최대 횡가속도 모델: v = √(a_lat_max / κ)
   - a_lat_max = 4.0 m/s² (기본값, ROS param으로 조정 가능)
   - V2X 속도 제한 및 위험 경고 반영

4. 상태머신 (State Machine)
   - TRACKING: 정상 주행 (좌/우 콘 탐지)
   - DEGRADED: 부분 탐지 (한쪽 콘 또는 일시적 미탐지)
   - STOPPING: 안전 정지 (센서 stale, 장기 미탐지, V2X stop)

=============================================================================
V2X 통합 (V2X Integration)
=============================================================================
RSU(Road-Side Unit)로부터 수신하는 메시지:
- /v2x/speed_limit: 구역별 속도 제한 (IVI 메시지 대응)
- /v2x/hazard: 위험 경고 플래그 (DENM 메시지 대응)
- /v2x/stop_zone: 정지 구역 플래그 (Geo-fenced zone)

=============================================================================
참고문헌 (References)
=============================================================================
[1] Coulter, R.C. (1992). "Implementation of the Pure Pursuit Path Tracking
    Algorithm". CMU-RI-TR-92-01, Carnegie Mellon University.
[2] Snider, J.M. (2009). "Automatic Steering Methods for Autonomous
    Automobile Path Tracking". CMU-RI-TR-09-08.
[3] ETSI EN 302 637-2: Intelligent Transport Systems; V2X Applications.

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
    /debug/state (String)                  - 현재 상태
    /debug/commentary (String)             - 자연어 상태 설명

Author: HYCU Autonomous Driving Team
Date: 2025-01
"""
import rospy
import threading
from collections import deque
import numpy as np
from math import atan2, sqrt
from enum import Enum
from fs_msgs.msg import ControlCommand
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Int32, Bool
import sensor_msgs.point_cloud2 as pc2


class DriveState(Enum):
    TRACKING = 1
    DEGRADED = 2
    STOPPING = 3


class StopReason(Enum):
    NONE = 0
    CONES_LOST = 1
    LIDAR_STALE = 2
    ODOM_STALE = 3
    V2X_STOP_ZONE = 4
    EMERGENCY_STOP = 5


class CompetitionDriver:
    def __init__(self):
        rospy.init_node('competition_driver')
        
        # ROS Parameters (tunable via rosparam set)
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
        
        # Timeout parameters (relaxed for stability)
        self.degraded_timeout = rospy.get_param('~degraded_timeout', 1.0)
        self.stopping_timeout = rospy.get_param('~stopping_timeout', 3.0)
        self.lidar_stale_timeout = rospy.get_param('~lidar_stale_timeout', 1.0)  # Relaxed from 0.5
        self.odom_stale_timeout = rospy.get_param('~odom_stale_timeout', 1.0)    # Relaxed from 0.5
        self.recovery_timeout = rospy.get_param('~recovery_timeout', 1.0)        # New: time to recover from STOPPING
        
        self.cmd_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        
        self.debug_state_pub = rospy.Publisher('/debug/state', String, queue_size=1)
        self.debug_speed_pub = rospy.Publisher('/debug/speed', Float32, queue_size=1)
        self.debug_target_speed_pub = rospy.Publisher('/debug/target_speed', Float32, queue_size=1)
        self.debug_curvature_pub = rospy.Publisher('/debug/curvature', Float32, queue_size=1)
        self.debug_steering_pub = rospy.Publisher('/debug/steering', Float32, queue_size=1)
        self.debug_cone_count_pub = rospy.Publisher('/debug/cone_count', Int32, queue_size=1)
        self.debug_stop_reason_pub = rospy.Publisher('/debug/stop_reason', String, queue_size=1)
        self.debug_commentary_pub = rospy.Publisher('/debug/commentary', String, queue_size=1)
        
        rospy.Subscriber('/v2x/speed_limit', Float32, self.v2x_speed_limit_callback)
        rospy.Subscriber('/v2x/hazard', Bool, self.v2x_hazard_callback)
        rospy.Subscriber('/v2x/stop_zone', Bool, self.v2x_stop_zone_callback)
        rospy.Subscriber('/debug/e_stop', Bool, self.estop_callback)
        
        self.current_speed = 0.0
        self.left_cones = []
        self.right_cones = []
        self.centerline = []
        self.current_curvature = 0.0
        self.current_target_speed = self.max_speed
        self.current_steering = 0.0
        
        self.state = DriveState.TRACKING
        self.stop_reason = StopReason.NONE
        self.last_valid_time = rospy.Time.now()
        self.last_valid_centerline = []
        self.last_valid_track_width = 4.0
        self.track_width_min = 2.0
        self.track_width_max = 6.0
        self.curvature_history = []
        self.curvature_smooth_window = 5
        self.last_lidar_time = rospy.Time.now()
        self.last_odom_time = rospy.Time.now()
        self.recovery_start_time = None
        
        self.last_throttle = 0.0
        self.last_steering = 0.0
        self.throttle_rate_limit = rospy.get_param('~throttle_rate_limit', 0.1)
        self.steering_rate_limit = rospy.get_param('~steering_rate_limit', 0.15)
        
        self.v2x_speed_limit = self.max_speed
        self.v2x_hazard = False
        self.v2x_stop_zone = False
        self.e_stop = False
        
        self.sensors_ready = False
        self.lidar_received = False
        self.odom_received = False
        
        self.commentary_history = []
        self.data_lock = threading.Lock()
        
        # Cached params for hot path (avoid rospy.get_param every cycle)
        self.max_lateral_accel = rospy.get_param('~max_lateral_accel', 4.0)
        self.last_log_time = 0.0
        
    def v2x_speed_limit_callback(self, msg):
        if np.isfinite(msg.data) and msg.data >= 0:
            self.v2x_speed_limit = msg.data
            self.add_commentary(f"V2X: speed limit {msg.data:.1f} m/s")
        else:
            rospy.logwarn_throttle(1.0, f"V2X: invalid speed_limit {msg.data}, ignored")
        
    def v2x_hazard_callback(self, msg):
        if msg.data != self.v2x_hazard:
            self.v2x_hazard = msg.data
            if msg.data:
                self.add_commentary("V2X: hazard ahead, slowing down")
            else:
                self.add_commentary("V2X: hazard cleared")
    
    def v2x_stop_zone_callback(self, msg):
        if msg.data != self.v2x_stop_zone:
            self.v2x_stop_zone = msg.data
            if msg.data:
                self.add_commentary("V2X: entering stop zone")
                self.stop_reason = StopReason.V2X_STOP_ZONE
            else:
                self.add_commentary("V2X: exiting stop zone")
    
    def estop_callback(self, msg):
        if msg.data != self.e_stop:
            self.e_stop = msg.data
            if msg.data:
                self.add_commentary("EMERGENCY STOP activated")
                self.stop_reason = StopReason.EMERGENCY_STOP
    
    def add_commentary(self, text):
        timestamp = rospy.Time.now().to_sec()
        self.commentary_history.append((timestamp, text))
        if len(self.commentary_history) > 20:
            self.commentary_history.pop(0)
        rospy.loginfo(f"[COMMENTARY] {text}")
        self.debug_commentary_pub.publish(String(data=text))
        
    def find_cones_filtered(self, points):
        if len(points) == 0:
            return [], []
        
        z_mask = (points[:, 2] > self.cone_min_z) & (points[:, 2] < self.cone_max_z)
        range_mask = np.linalg.norm(points[:, :2], axis=1) < self.cones_range_cutoff
        front_mask = points[:, 0] > 0.3
        filtered = points[z_mask & range_mask & front_mask]
        
        if len(filtered) == 0:
            return [], []
        
        grid_res = self.cone_grouping_threshold
        grid = {}
        for i, pt in enumerate(filtered):
            gx, gy = int(pt[0] / grid_res), int(pt[1] / grid_res)
            key = (gx, gy)
            if key not in grid:
                grid[key] = []
            grid[key].append(i)
        
        cones = []
        visited_cells = set()
        
        for cell_key in grid.keys():
            if cell_key in visited_cells:
                continue
            
            cluster_indices = []
            queue = deque([cell_key])
            
            while queue:
                current = queue.popleft()
                if current in visited_cells:
                    continue
                if current not in grid:
                    continue
                    
                visited_cells.add(current)
                cluster_indices.extend(grid[current])
                
                gx, gy = current
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        neighbor = (gx + dx, gy + dy)
                        if neighbor in grid and neighbor not in visited_cells:
                            queue.append(neighbor)
            
            if len(cluster_indices) < self.cone_min_points:
                continue
            
            group = filtered[cluster_indices]
            xy = group[:, :2]
            center = xy.mean(axis=0)
            radii = np.linalg.norm(xy - center, axis=1)
            
            if radii.mean() > self.cone_max_radius:
                continue
            if radii.std() > self.cone_max_radius_std:
                continue
            
            cones.append({
                'x': center[0],
                'y': center[1],
                'points': len(cluster_indices)
            })
        
        left = sorted([c for c in cones if c['y'] > 0], key=lambda c: c['x'])
        right = sorted([c for c in cones if c['y'] <= 0], key=lambda c: c['x'])
        
        return left, right
    
    def build_centerline(self, left_cones, right_cones):
        if not left_cones and not right_cones:
            return []
        
        centerline = []
        
        if left_cones and right_cones:
            all_x = sorted(set([c['x'] for c in left_cones + right_cones]))
            
            for x in all_x[:8]:
                left_at_x = [c for c in left_cones if abs(c['x'] - x) < 2.0]
                right_at_x = [c for c in right_cones if abs(c['x'] - x) < 2.0]
                
                if left_at_x and right_at_x:
                    left_y = np.mean([c['y'] for c in left_at_x])
                    right_y = np.mean([c['y'] for c in right_at_x])
                    center_y = (left_y + right_y) / 2.0
                    centerline.append({'x': x, 'y': center_y})
                elif left_at_x:
                    left_y = np.mean([c['y'] for c in left_at_x])
                    centerline.append({'x': x, 'y': left_y - self.last_valid_track_width / 2})
                elif right_at_x:
                    right_y = np.mean([c['y'] for c in right_at_x])
                    centerline.append({'x': x, 'y': right_y + self.last_valid_track_width / 2})
        
        elif left_cones:
            for c in left_cones[:5]:
                centerline.append({'x': c['x'], 'y': c['y'] - self.last_valid_track_width / 2})
        
        elif right_cones:
            for c in right_cones[:5]:
                centerline.append({'x': c['x'], 'y': c['y'] + self.last_valid_track_width / 2})
        
        centerline = sorted(centerline, key=lambda c: c['x'])
        
        if len(centerline) >= 3:
            smoothed = []
            window = 3
            for i in range(len(centerline)):
                start = max(0, i - window // 2)
                end = min(len(centerline), i + window // 2 + 1)
                avg_y = np.mean([c['y'] for c in centerline[start:end]])
                smoothed.append({'x': centerline[i]['x'], 'y': avg_y})
            return smoothed
        
        return centerline
    
    def get_lookahead_point(self, centerline):
        if not centerline:
            return None
        
        lookahead = self.lookahead_base + self.lookahead_speed_gain * self.current_speed
        
        arc_length = 0.0
        prev = {'x': 0.0, 'y': 0.0}
        
        for point in centerline:
            segment = sqrt((point['x'] - prev['x'])**2 + (point['y'] - prev['y'])**2)
            arc_length += segment
            if arc_length >= lookahead:
                return point
            prev = point
        
        return centerline[-1] if centerline else None
    
    def pure_pursuit_steering(self, target):
        if target is None:
            return 0.0
        
        alpha = atan2(target['y'], target['x'])
        lookahead_dist = sqrt(target['x']**2 + target['y']**2)
        
        if lookahead_dist < 0.1:
            return 0.0
        
        steering = atan2(2 * self.wheelbase * np.sin(alpha), lookahead_dist)
        return np.clip(steering, -self.max_steering, self.max_steering)
    
    def estimate_curvature(self, centerline):
        if len(centerline) < 3:
            return self.get_smoothed_curvature(0.0)
        
        points = np.array([[c['x'], c['y']] for c in centerline[:5]])
        
        if len(points) < 3:
            return self.get_smoothed_curvature(0.0)
        
        dx = np.gradient(points[:, 0])
        dy = np.gradient(points[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-6)**1.5
        raw_curvature = np.mean(curvature)
        return self.get_smoothed_curvature(raw_curvature)
    
    def get_smoothed_curvature(self, new_curvature):
        self.curvature_history.append(new_curvature)
        if len(self.curvature_history) > self.curvature_smooth_window:
            self.curvature_history.pop(0)
        return np.mean(self.curvature_history)
    
    def calculate_target_speed(self, curvature):
        speed = np.sqrt(self.max_lateral_accel / (abs(curvature) + 0.01))
        speed = np.clip(speed, self.min_speed, self.max_speed)
        
        if self.v2x_speed_limit <= 0:
            return 0.0
        speed = min(speed, self.v2x_speed_limit)
        speed = max(0.0, speed)
        
        if self.v2x_hazard:
            speed = min(speed, self.min_speed)
        
        if not np.isfinite(speed):
            return self.min_speed
        return speed
    
    def apply_rate_limit(self, target, current, max_rate):
        delta = target - current
        if abs(delta) > max_rate:
            return current + max_rate * np.sign(delta)
        return target
    
    def calculate_throttle(self, target_speed):
        speed_error = target_speed - self.current_speed
        throttle = 0.3 * speed_error
        return np.clip(throttle, 0.0, self.max_throttle)
    
    def check_watchdog(self):
        now = rospy.Time.now()
        
        if self.e_stop:
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.EMERGENCY_STOP
            self.recovery_start_time = None
            return
        
        if self.v2x_stop_zone:
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.V2X_STOP_ZONE
            self.recovery_start_time = None
            return
        
        if self.state == DriveState.STOPPING and self.stop_reason in [StopReason.V2X_STOP_ZONE, StopReason.EMERGENCY_STOP]:
            self.add_commentary(f"WATCHDOG: {self.stop_reason.name} cleared, resuming DEGRADED")
            self.state = DriveState.DEGRADED
            self.stop_reason = StopReason.NONE
            self.recovery_start_time = None
            return
        
        lidar_elapsed = (now - self.last_lidar_time).to_sec()
        if lidar_elapsed > self.lidar_stale_timeout:
            if self.state != DriveState.STOPPING or self.stop_reason != StopReason.LIDAR_STALE:
                self.add_commentary(f"WATCHDOG: LiDAR stale ({lidar_elapsed:.1f}s)")
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.LIDAR_STALE
            self.recovery_start_time = None
            return
        
        odom_elapsed = (now - self.last_odom_time).to_sec()
        if odom_elapsed > self.odom_stale_timeout:
            if self.state != DriveState.STOPPING or self.stop_reason != StopReason.ODOM_STALE:
                self.add_commentary(f"WATCHDOG: Odometry stale ({odom_elapsed:.1f}s)")
            self.state = DriveState.STOPPING
            self.stop_reason = StopReason.ODOM_STALE
            self.recovery_start_time = None
            return
        
        if self.state == DriveState.STOPPING and self.stop_reason in [StopReason.LIDAR_STALE, StopReason.ODOM_STALE, StopReason.CONES_LOST]:
            if self.recovery_start_time is None:
                self.recovery_start_time = now
            elif (now - self.recovery_start_time).to_sec() >= self.recovery_timeout:
                self.add_commentary("WATCHDOG: Sensors recovered, attempting resume")
                self.state = DriveState.DEGRADED
                self.stop_reason = StopReason.NONE
                self.recovery_start_time = None
    
    def update_state(self, left_cones, right_cones):
        now = rospy.Time.now()
        has_valid_cones = len(left_cones) >= 1 or len(right_cones) >= 1
        has_both_sides = len(left_cones) >= 1 and len(right_cones) >= 1
        
        if has_valid_cones:
            if self.state == DriveState.STOPPING and self.stop_reason == StopReason.CONES_LOST:
                self.add_commentary("Cones recovered, resuming DEGRADED")
                self.state = DriveState.DEGRADED
                self.stop_reason = StopReason.NONE
            elif self.state != DriveState.TRACKING and has_both_sides:
                self.add_commentary("Cones recovered, resuming TRACKING")
                self.state = DriveState.TRACKING
                self.stop_reason = StopReason.NONE
            elif has_both_sides:
                self.state = DriveState.TRACKING
                self.stop_reason = StopReason.NONE
            
            self.last_valid_time = now
            
            if left_cones and right_cones:
                left_y = np.mean([c['y'] for c in left_cones[:3]])
                right_y = np.mean([c['y'] for c in right_cones[:3]])
                raw_width = abs(left_y - right_y)
                self.last_valid_track_width = np.clip(raw_width, self.track_width_min, self.track_width_max)
        else:
            if self.state == DriveState.STOPPING:
                return
            
            elapsed = (now - self.last_valid_time).to_sec()
            
            if elapsed > self.stopping_timeout:
                if self.state != DriveState.STOPPING:
                    self.add_commentary(f"Cones lost for {elapsed:.1f}s, STOPPING")
                self.state = DriveState.STOPPING
                self.stop_reason = StopReason.CONES_LOST
            elif elapsed > self.degraded_timeout:
                if self.state != DriveState.DEGRADED:
                    self.add_commentary(f"Cones lost for {elapsed:.1f}s, DEGRADED mode")
                self.state = DriveState.DEGRADED
    
    def lidar_callback(self, msg):
        try:
            self.last_lidar_time = rospy.Time.now()
            self.lidar_received = True
            points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            
            if len(points) == 0:
                with self.data_lock:
                    self.left_cones = []
                    self.right_cones = []
                    self.centerline = []
                return
            
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                points = points[~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)]
            
            left, right = self.find_cones_filtered(points)
            center = self.build_centerline(left, right)
            
            with self.data_lock:
                self.left_cones = left
                self.right_cones = right
                self.centerline = center
                if center:
                    self.last_valid_centerline = center
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"LiDAR callback error: {e}")
    
    def odom_callback(self, msg):
        try:
            self.last_odom_time = rospy.Time.now()
            self.odom_received = True
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            speed = sqrt(vx**2 + vy**2)
            if np.isfinite(speed):
                self.current_speed = speed
            else:
                rospy.logwarn_throttle(1.0, "Odom NaN/Inf detected, using last valid speed")
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Odom callback error: {e}")
    
    def publish_telemetry(self):
        self.debug_state_pub.publish(String(data=self.state.name))
        self.debug_speed_pub.publish(Float32(data=self.current_speed))
        self.debug_target_speed_pub.publish(Float32(data=self.current_target_speed))
        self.debug_curvature_pub.publish(Float32(data=self.current_curvature))
        self.debug_steering_pub.publish(Float32(data=self.current_steering))
        self.debug_cone_count_pub.publish(Int32(data=len(self.left_cones) + len(self.right_cones)))
        self.debug_stop_reason_pub.publish(String(data=self.stop_reason.name))
    
    def compute_control(self, centerline_snapshot, last_valid_centerline_snapshot):
        """Compute control command based on current state and centerline snapshots."""
        cmd = ControlCommand()
        
        if self.state == DriveState.STOPPING:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake = 1.0
            self.current_target_speed = 0.0
            self.current_steering = 0.0
            return cmd
        
        if self.state == DriveState.DEGRADED:
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
                cmd.steering = self.pure_pursuit_steering(self.get_lookahead_point(centerline))
                self.current_steering = cmd.steering
                return cmd
        else:
            centerline = centerline_snapshot
            self.current_curvature = self.estimate_curvature(centerline)
            self.current_target_speed = self.calculate_target_speed(self.current_curvature)
        
        target_point = self.get_lookahead_point(centerline)
        
        self.current_steering = self.pure_pursuit_steering(target_point)
        raw_throttle = self.calculate_throttle(self.current_target_speed)
        
        cmd.steering = self.apply_rate_limit(self.current_steering, self.last_steering, self.steering_rate_limit)
        cmd.throttle = self.apply_rate_limit(raw_throttle, self.last_throttle, self.throttle_rate_limit)
        
        self.last_steering = cmd.steering
        self.last_throttle = cmd.throttle
        
        # Active braking when target speed is very low (V2X stop command)
        if self.current_target_speed < 0.5:
            cmd.throttle = 0.0
            cmd.brake = 1.0
        elif self.current_speed > self.current_target_speed + 1.0:
            cmd.brake = min(0.5, (self.current_speed - self.current_target_speed) * 0.3)
        else:
            cmd.brake = 0.0
        
        return cmd
    
    def run(self):
        rate = rospy.Rate(20)
        rospy.loginfo("=" * 50)
        rospy.loginfo("Competition Driver v2.1")
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
                
                self.check_watchdog()
                
                with self.data_lock:
                    left_snapshot = list(self.left_cones)
                    right_snapshot = list(self.right_cones)
                    centerline_snapshot = list(self.centerline)
                    last_valid_centerline_snapshot = list(self.last_valid_centerline)
                
                self.update_state(left_snapshot, right_snapshot)
                
                cmd = self.compute_control(centerline_snapshot, last_valid_centerline_snapshot)
                self.cmd_pub.publish(cmd)
                self.publish_telemetry()
                
                now_sec = rospy.Time.now().to_sec()
                if now_sec - self.last_log_time >= 2.0:
                    self.last_log_time = now_sec
                    rospy.loginfo(f"[{self.state.name}] Speed: {self.current_speed:.1f}/{self.current_target_speed:.1f} m/s | "
                                 f"Steer: {self.current_steering:.2f} | L:{len(self.left_cones)} R:{len(self.right_cones)}")
            except Exception as e:
                rospy.logerr(f"Main loop error: {e}")
                # Emergency stop on error
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
