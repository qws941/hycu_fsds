#!/usr/bin/env python3
"""
FSDS Advanced Autonomous Driver - PID Steering with Centerline Following

Competition advanced template demonstrating:
- PID controller for smooth steering response
- Centerline extraction from left/right cone positions
- Speed modulation based on cone density

Reference:
    Astrom, K.J. & Murray, R.M. (2008). Feedback Systems: An Introduction
    for Scientists and Engineers. Princeton University Press.
"""
import rospy
import numpy as np
from fs_msgs.msg import ControlCommand
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class AdvancedDriver:
    def __init__(self):
        rospy.init_node('advanced_driver')
        
        self.max_throttle = 0.25
        self.target_speed = 5.0
        self.max_steering = 0.4
        self.cones_range_cutoff = 10.0
        self.cone_grouping_threshold = 0.15
        
        self.steering_pid = PIDController(kp=0.8, ki=0.05, kd=0.1)
        self.last_time = rospy.Time.now()
        
        self.cmd_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        
        self.current_speed = 0.0
        self.left_cones = []
        self.right_cones = []
        
    def find_cones_with_sides(self, points):
        """Detect cones and classify as left/right based on Y-coordinate."""
        if len(points) == 0:
            return [], []
            
        range_mask = np.linalg.norm(points[:, :2], axis=1) < self.cones_range_cutoff
        front_mask = points[:, 0] > 0.5
        filtered = points[range_mask & front_mask]
        
        if len(filtered) == 0:
            return [], []
            
        cones = []
        used = np.zeros(len(filtered), dtype=bool)
        
        for i in range(len(filtered)):
            if used[i]:
                continue
            group = [filtered[i]]
            used[i] = True
            for j in range(i + 1, len(filtered)):
                if used[j]:
                    continue
                if np.linalg.norm(filtered[i][:2] - filtered[j][:2]) < self.cone_grouping_threshold:
                    group.append(filtered[j])
                    used[j] = True
            if len(group) >= 3:
                center = np.mean(group, axis=0)
                cones.append({'x': center[0], 'y': center[1]})
        
        left = [c for c in cones if c['y'] > 0]
        right = [c for c in cones if c['y'] <= 0]
        
        return left, right
        
    def lidar_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        self.left_cones, self.right_cones = self.find_cones_with_sides(points)
        
    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = np.sqrt(vx**2 + vy**2)
        
    def calculate_centerline_error(self):
        """Calculate lateral error from centerline (midpoint of left/right cone averages)."""
        if not self.left_cones and not self.right_cones:
            return 0.0
            
        left_avg_y = np.mean([c['y'] for c in self.left_cones]) if self.left_cones else 2.0
        right_avg_y = np.mean([c['y'] for c in self.right_cones]) if self.right_cones else -2.0
        
        center_y = (left_avg_y + right_avg_y) / 2.0
        return center_y
        
    def calculate_steering(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.05
            
        error = self.calculate_centerline_error()
        steering = -self.steering_pid.compute(error, dt)
        return np.clip(steering, -self.max_steering, self.max_steering)
        
    def calculate_throttle(self):
        num_cones = len(self.left_cones) + len(self.right_cones)
        
        if num_cones > 6:
            adjusted_target = self.target_speed * 0.7
        else:
            adjusted_target = self.target_speed
            
        speed_ratio = self.current_speed / adjusted_target
        return self.max_throttle * max(1.0 - speed_ratio, 0.0)
        
    def compute_control(self):
        cmd = ControlCommand()
        cmd.throttle = self.calculate_throttle()
        cmd.steering = self.calculate_steering()
        cmd.brake = 0.0
        return cmd
        
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            cmd = self.compute_control()
            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        driver = AdvancedDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
