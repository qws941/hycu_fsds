#!/usr/bin/env python3
import rospy
import numpy as np
from math import cos, sin, atan2, sqrt
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft
import tf2_ros


class SimpleSLAM:
    def __init__(self):
        rospy.init_node('simple_slam')
        
        self.map_resolution = rospy.get_param('~map_resolution', 0.1)
        self.map_size = rospy.get_param('~map_size', 200)
        self.map_origin = -self.map_size * self.map_resolution / 2
        self.decay_rate = rospy.get_param('~decay_rate', 2)
        self.decay_interval = rospy.get_param('~decay_interval', 1.0)
        
        self.occupancy_grid = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.cone_map = []
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.path_history = []
        self.max_path_length = 500
        self.last_decay_time = rospy.Time.now()
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.map_pub = rospy.Publisher('/slam/map', OccupancyGrid, queue_size=1)
        self.path_pub = rospy.Publisher('/slam/path', Path, queue_size=1)
        self.pose_pub = rospy.Publisher('/slam/pose', PoseStamped, queue_size=1)
        
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("Simple SLAM initialized")
    
    def odom_callback(self, msg):
        try:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            _, _, self.robot_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            self.path_history.append((self.robot_x, self.robot_y))
            if len(self.path_history) > self.max_path_length:
                self.path_history.pop(0)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Odom callback error: {e}")
    
    def lidar_callback(self, msg):
        try:
            points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            
            if len(points) == 0:
                return
            
            z_mask = (points[:, 2] > -0.3) & (points[:, 2] < 0.5)
            filtered = points[z_mask]
            
            for p in filtered[::10]:
                world_x = self.robot_x + p[0] * cos(self.robot_yaw) - p[1] * sin(self.robot_yaw)
                world_y = self.robot_y + p[0] * sin(self.robot_yaw) + p[1] * cos(self.robot_yaw)
                
                grid_x = int((world_x - self.map_origin) / self.map_resolution)
                grid_y = int((world_y - self.map_origin) / self.map_resolution)
                
                if 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size:
                    self.occupancy_grid[grid_y, grid_x] = min(100, self.occupancy_grid[grid_y, grid_x] + 10)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Lidar callback error: {e}")
    
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = self.map_origin
        msg.info.origin.position.y = self.map_origin
        msg.info.origin.position.z = 0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.occupancy_grid.flatten().tolist()
        self.map_pub.publish(msg)
    
    def publish_path(self):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        for x, y in self.path_history:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.path_pub.publish(msg)
    
    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.robot_x
        msg.pose.position.y = self.robot_y
        q = tft.quaternion_from_euler(0, 0, self.robot_yaw)
        msg.pose.orientation = Quaternion(*q)
        self.pose_pub.publish(msg)
    
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        t2 = TransformStamped()
        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = "odom"
        t2.child_frame_id = "base_link"
        t2.transform.translation.x = self.robot_x
        t2.transform.translation.y = self.robot_y
        t2.transform.translation.z = 0.0
        q = tft.quaternion_from_euler(0, 0, self.robot_yaw)
        t2.transform.rotation.x = q[0]
        t2.transform.rotation.y = q[1]
        t2.transform.rotation.z = q[2]
        t2.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t2)
    
    def apply_decay(self):
        now = rospy.Time.now()
        if (now - self.last_decay_time).to_sec() >= self.decay_interval:
            self.occupancy_grid = np.maximum(0, self.occupancy_grid - self.decay_rate)
            self.last_decay_time = now
    
    def run(self):
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown():
            self.apply_decay()
            self.publish_tf()
            self.publish_map()
            self.publish_path()
            self.publish_pose()
            rate.sleep()


if __name__ == '__main__':
    try:
        slam = SimpleSLAM()
        slam.run()
    except rospy.ROSInterruptException:
        pass
