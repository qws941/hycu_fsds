#!/usr/bin/env python3
"""
FSDS Basic Drive Control - 기본 주행 제어
경진대회 기본 템플릿
"""
import rospy
import numpy as np
from fs_msgs.msg import ControlCommand
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2

class BasicDriver:
    def __init__(self):
        rospy.init_node('basic_driver')
        
        self.state = 'DRIVE'
        self.min_obstacle_distance = 3.0
        self.front_range = 10.0
        self.side_range = 2.0
        
        self.cmd_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        
        self.current_speed = 0.0
        
    def lidar_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if len(points) == 0:
            return
            
        front_mask = (
            (points[:, 0] > 0) & 
            (points[:, 0] < self.front_range) &
            (np.abs(points[:, 1]) < self.side_range)
        )
        front_points = points[front_mask]
        
        if len(front_points) > 0:
            distances = np.linalg.norm(front_points[:, :2], axis=1)
            min_distance = np.min(distances)
            
            if min_distance < self.min_obstacle_distance:
                self.state = 'STOP'
            else:
                self.state = 'DRIVE'
        else:
            self.state = 'DRIVE'
            
    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = np.sqrt(vx**2 + vy**2)
        
    def compute_control(self):
        cmd = ControlCommand()
        
        if self.state == 'STOP':
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake = 1.0
        else:
            cmd.throttle = 0.2
            cmd.steering = 0.0
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
        driver = BasicDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
