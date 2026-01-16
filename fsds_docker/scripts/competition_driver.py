#!/usr/bin/env python3
import rospy
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
        
    def v2x_speed_limit_callback(self, msg):
        self.v2x_speed_limit = msg.data
        self.add_commentary(f"V2X: speed limit {msg.data:.1f} m/s")
        
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
        
        for cell_key, indices in grid.items():
            if cell_key in visited_cells:
                continue
            
            cluster_indices = list(indices)
            visited_cells.add(cell_key)
            
            gx, gy = cell_key
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    neighbor = (gx + dx, gy + dy)
                    if neighbor in grid and neighbor not in visited_cells:
                        cluster_indices.extend(grid[neighbor])
                        visited_cells.add(neighbor)
            
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
        
        return sorted(centerline, key=lambda c: c['x'])
    
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
        max_lateral_accel = rospy.get_param('~max_lateral_accel', 4.0)
        speed = np.sqrt(max_lateral_accel / (abs(curvature) + 0.01))
        speed = np.clip(speed, self.min_speed, self.max_speed)
        
        # V2X speed limit (clamp to valid range, 0 means stop)
        if self.v2x_speed_limit <= 0:
            return 0.0  # Signal full stop
        speed = min(speed, max(self.v2x_speed_limit, self.min_speed))
        
        if self.v2x_hazard:
            speed = min(speed, self.min_speed)
        
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
                self.left_cones = []
                self.right_cones = []
                self.centerline = []
                return
            
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                points = points[~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)]
            
            self.left_cones, self.right_cones = self.find_cones_filtered(points)
            self.centerline = self.build_centerline(self.left_cones, self.right_cones)
            
            if self.centerline:
                self.last_valid_centerline = self.centerline
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"LiDAR callback error: {e}")
    
    def odom_callback(self, msg):
        try:
            self.last_odom_time = rospy.Time.now()
            self.odom_received = True
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.current_speed = sqrt(vx**2 + vy**2)
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
    
    def compute_control(self):
        cmd = ControlCommand()
        
        if self.state == DriveState.STOPPING:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake = 1.0
            self.current_target_speed = 0.0
            self.current_steering = 0.0
            return cmd
        
        if self.state == DriveState.DEGRADED:
            centerline = self.last_valid_centerline
            self.current_target_speed = self.min_speed
            speed_error = self.current_speed - self.current_target_speed
            if speed_error > 0.5:
                cmd.brake = min(0.3, speed_error * 0.2)
                cmd.throttle = 0.0
                cmd.steering = self.pure_pursuit_steering(self.get_lookahead_point(centerline))
                self.current_steering = cmd.steering
                return cmd
        else:
            centerline = self.centerline
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
            if not self.sensors_ready:
                if self.lidar_received and self.odom_received:
                    self.sensors_ready = True
                    self.add_commentary("Sensors ready, starting autonomous control")
                else:
                    cmd = ControlCommand()
                    cmd.throttle = 0.0
                    cmd.steering = 0.0
                    cmd.brake = 1.0
                    self.cmd_pub.publish(cmd)
                    rate.sleep()
                    continue
            
            self.check_watchdog()
            self.update_state(self.left_cones, self.right_cones)
            
            cmd = self.compute_control()
            self.cmd_pub.publish(cmd)
            self.publish_telemetry()
            
            if rospy.Time.now().to_sec() % 2 < 0.1:
                rospy.loginfo(f"[{self.state.name}] Speed: {self.current_speed:.1f}/{self.current_target_speed:.1f} m/s | "
                             f"Steer: {self.current_steering:.2f} | L:{len(self.left_cones)} R:{len(self.right_cones)}")
            
            rate.sleep()


if __name__ == '__main__':
    try:
        driver = CompetitionDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
