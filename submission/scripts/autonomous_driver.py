#!/usr/bin/env python3
"""
FSDS Autonomous Cone Following - 콘 탐지 자율주행
경진대회 메인 템플릿
"""
import rospy
import numpy as np
from fs_msgs.msg import ControlCommand
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2

class AutonomousDriver:
    def __init__(self):
        rospy.init_node('autonomous_driver')
        
        self.max_throttle = 0.2
        self.target_speed = 4.0
        self.max_steering = 0.3
        self.cones_range_cutoff = 7.0
        self.cone_grouping_threshold = 0.1
        
        self.cmd_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        
        self.current_speed = 0.0
        self.cones = []
        
    def find_cones(self, points):
        """인접 포인트 그룹핑으로 콘 중심점 추출"""
        if len(points) == 0:
            return []
            
        range_mask = np.linalg.norm(points[:, :2], axis=1) < self.cones_range_cutoff
        filtered = points[range_mask]
        
        if len(filtered) == 0:
            return []
            
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
                
        return cones
        
    def lidar_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        self.cones = self.find_cones(points)
        
    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = np.sqrt(vx**2 + vy**2)
        
    def calculate_steering(self):
        if len(self.cones) == 0:
            return 0.0
            
        avg_y = np.mean([c['y'] for c in self.cones])
        
        if avg_y > 0:
            return -self.max_steering
        else:
            return self.max_steering
            
    def calculate_throttle(self):
        speed_ratio = self.current_speed / self.target_speed
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
        driver = AutonomousDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
