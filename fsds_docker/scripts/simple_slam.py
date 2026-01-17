#!/usr/bin/env python3
"""
Simple SLAM Node - Occupancy Grid Mapping for FSDS
===================================================

이 모듈은 Formula Student Driverless Simulator(FSDS)용 
2D Occupancy Grid Mapping을 구현합니다.

SLAM vs Mapping 비교 (Scope Clarification)
-------------------------------------------
본 구현은 "Full SLAM"이 아닌 **Mapping 중심 구현**입니다.

+-------------------+-----------------+---------------------------+
| SLAM 구성요소     | Full SLAM       | 본 구현                   |
+-------------------+-----------------+---------------------------+
| Scan Matching     | ICP/NDT         | (미구현) Odometry 의존    |
| Loop Closure      | Pose Graph      | (미구현)                  |
| Localization      | Particle/EKF    | Simulator Odometry 의존   |
| Mapping           | Occupancy Grid  | ✓ 구현됨                  |
+-------------------+-----------------+---------------------------+

의도된 단순화 (Design Decisions)
---------------------------------
1. **Localization**: 시뮬레이터가 제공하는 ground-truth odometry에 의존.
   실제 차량에서는 EKF/UKF 기반 센서 퓨전이 필요합니다.

2. **Occupancy Update**: Hit-only 누적 방식 사용.
   - 표준 방식: Ray-casting으로 free cell도 업데이트 (Bresenham)
   - 본 구현: Hit cell만 확률 증가 (계산 효율 + 안정성 우선)

3. **Decay 메커니즘**: Ghost obstacle 완화 목적.
   - 동적 장애물이 지나간 후에도 맵에 남는 문제 방지
   - decay_rate (기본 2) 만큼 주기적으로 occupancy 감소

4. **TF Tree 설계**: 
   - map → odom: 항등 변환 (SLAM drift 보정 미구현)
   - odom → base_link: 옵션 (~publish_base_link_tf 파라미터)

알고리즘 (Algorithm)
--------------------
1. LiDAR PointCloud2 수신 → 높이(z) 필터링 (-0.3m ~ 0.5m)
2. 로봇 좌표계 → 월드 좌표계 변환 (odom 기반)
3. 월드 좌표 → 그리드 셀 인덱스 계산
4. Hit 셀 점유 확률 증가 (+10, 최대 100)
5. 주기적 decay로 오래된 장애물 제거

참고문헌 (References)
---------------------
- Thrun, S., Burgard, W., & Fox, D. (2005). 
  Probabilistic Robotics. MIT Press. (Chapter 9: Occupancy Grid Mapping)
- Grisetti, G., Stachniss, C., & Burgard, W. (2007). 
  Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters. 
  IEEE Transactions on Robotics, 23(1), 34-46.

ROS Interface
-------------
Parameters:
    ~map_resolution (float): 그리드 해상도 (m/cell), 기본값 0.1
    ~map_size (int): 그리드 크기 (cells), 기본값 200
    ~decay_rate (int): Decay 감소량, 기본값 2
    ~decay_interval (float): Decay 주기 (초), 기본값 1.0
    ~publish_base_link_tf (bool): base_link TF 발행 여부, 기본값 False

Subscribed Topics:
    /fsds/lidar/Lidar1 (sensor_msgs/PointCloud2): LiDAR 포인트 클라우드
    /fsds/testing_only/odom (nav_msgs/Odometry): 차량 위치/자세

Published Topics:
    /slam/map (nav_msgs/OccupancyGrid): 점유 격자 맵
    /slam/path (nav_msgs/Path): 주행 경로 기록
    /slam/pose (geometry_msgs/PoseStamped): 현재 위치/자세

TF Broadcasts:
    map → odom: 항등 변환
    odom → base_link: 차량 위치 (옵션)
"""
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
        self.publish_base_link_tf = rospy.get_param('~publish_base_link_tf', False)
        
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
        
        if self.publish_base_link_tf:
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
            try:
                self.apply_decay()
                self.publish_tf()
                self.publish_map()
                self.publish_path()
                self.publish_pose()
            except Exception as e:
                rospy.logerr(f"SLAM main loop error: {e}")
            rate.sleep()


if __name__ == '__main__':
    try:
        slam = SimpleSLAM()
        slam.run()
    except rospy.ROSInterruptException:
        pass
