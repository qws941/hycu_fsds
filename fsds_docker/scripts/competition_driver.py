#!/usr/bin/env python3
import rospy
import numpy as np
from math import atan2, sqrt
from enum import Enum
from fs_msgs.msg import ControlCommand
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2


class DriveState(Enum):
    TRACKING = 1
    DEGRADED = 2
    STOPPING = 3


class CompetitionDriver:
    def __init__(self):
        rospy.init_node('competition_driver')
        
        self.max_throttle = 0.25
        self.min_speed = 2.0
        self.max_speed = 6.0
        self.max_steering = 0.4
        self.wheelbase = 1.5
        self.lookahead_base = 4.0
        self.lookahead_speed_gain = 0.5
        
        self.cones_range_cutoff = 12.0
        self.cone_min_z = -0.3
        self.cone_max_z = 0.5
        self.cone_grouping_threshold = 0.2
        self.cone_min_points = 3
        self.cone_max_radius = 0.3
        self.cone_max_radius_std = 0.15
        
        self.degraded_timeout = 1.0
        self.stopping_timeout = 3.0
        
        self.cmd_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        
        self.current_speed = 0.0
        self.left_cones = []
        self.right_cones = []
        self.centerline = []
        
        self.state = DriveState.TRACKING
        self.last_valid_time = rospy.Time.now()
        self.last_valid_centerline = []
        self.last_valid_track_width = 4.0
        
    def find_cones_filtered(self, points):
        if len(points) == 0:
            return [], []
        
        z_mask = (points[:, 2] > self.cone_min_z) & (points[:, 2] < self.cone_max_z)
        range_mask = np.linalg.norm(points[:, :2], axis=1) < self.cones_range_cutoff
        front_mask = points[:, 0] > 0.3
        filtered = points[z_mask & range_mask & front_mask]
        
        if len(filtered) == 0:
            return [], []
        
        cones = []
        used = np.zeros(len(filtered), dtype=bool)
        
        for i in range(len(filtered)):
            if used[i]:
                continue
            
            group_indices = [i]
            used[i] = True
            
            for j in range(i + 1, len(filtered)):
                if used[j]:
                    continue
                if np.linalg.norm(filtered[i][:2] - filtered[j][:2]) < self.cone_grouping_threshold:
                    group_indices.append(j)
                    used[j] = True
            
            if len(group_indices) < self.cone_min_points:
                continue
            
            group = filtered[group_indices]
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
                'points': len(group_indices)
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
        
        for point in centerline:
            dist = sqrt(point['x']**2 + point['y']**2)
            if dist >= lookahead:
                return point
        
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
            return 0.0
        
        points = np.array([[c['x'], c['y']] for c in centerline[:5]])
        
        if len(points) < 3:
            return 0.0
        
        dx = np.gradient(points[:, 0])
        dy = np.gradient(points[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-6)**1.5
        return np.mean(curvature)
    
    def calculate_target_speed(self, curvature):
        k = 2.0
        eps = 0.01
        speed = k / (abs(curvature) + eps)
        return np.clip(speed, self.min_speed, self.max_speed)
    
    def calculate_throttle(self, target_speed):
        speed_error = target_speed - self.current_speed
        throttle = 0.3 * speed_error
        return np.clip(throttle, 0.0, self.max_throttle)
    
    def update_state(self, left_cones, right_cones):
        now = rospy.Time.now()
        has_valid_cones = len(left_cones) >= 1 and len(right_cones) >= 1
        
        if has_valid_cones:
            self.state = DriveState.TRACKING
            self.last_valid_time = now
            
            if left_cones and right_cones:
                left_y = np.mean([c['y'] for c in left_cones[:3]])
                right_y = np.mean([c['y'] for c in right_cones[:3]])
                self.last_valid_track_width = abs(left_y - right_y)
        else:
            elapsed = (now - self.last_valid_time).to_sec()
            
            if elapsed > self.stopping_timeout:
                self.state = DriveState.STOPPING
            elif elapsed > self.degraded_timeout:
                self.state = DriveState.DEGRADED
    
    def lidar_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        self.left_cones, self.right_cones = self.find_cones_filtered(points)
        self.centerline = self.build_centerline(self.left_cones, self.right_cones)
        
        if self.centerline:
            self.last_valid_centerline = self.centerline
        
        self.update_state(self.left_cones, self.right_cones)
    
    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = sqrt(vx**2 + vy**2)
    
    def compute_control(self):
        cmd = ControlCommand()
        
        if self.state == DriveState.STOPPING:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake = 1.0
            return cmd
        
        if self.state == DriveState.DEGRADED:
            centerline = self.last_valid_centerline
            target_speed = self.min_speed
        else:
            centerline = self.centerline
            curvature = self.estimate_curvature(centerline)
            target_speed = self.calculate_target_speed(curvature)
        
        target_point = self.get_lookahead_point(centerline)
        
        cmd.steering = self.pure_pursuit_steering(target_point)
        cmd.throttle = self.calculate_throttle(target_speed)
        cmd.brake = 0.0
        
        return cmd
    
    def run(self):
        rate = rospy.Rate(20)
        rospy.loginfo("Competition Driver started - Pure Pursuit + Curvature Speed Control")
        
        while not rospy.is_shutdown():
            cmd = self.compute_control()
            self.cmd_pub.publish(cmd)
            
            if rospy.Time.now().to_sec() % 2 < 0.1:
                rospy.loginfo(f"State: {self.state.name}, Speed: {self.current_speed:.1f}, "
                             f"L:{len(self.left_cones)} R:{len(self.right_cones)}")
            
            rate.sleep()


if __name__ == '__main__':
    try:
        driver = CompetitionDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
