#!/usr/bin/env python3
"""
Cone Classifier - LiDAR 기반 콘 색상 분류 및 시각화 노드.

목적:
    LiDAR 포인트클라우드에서 콘을 검출하고 intensity 값을 기반으로
    색상(blue/yellow/orange)을 분류하여 RViz 마커로 시각화합니다.

분류 기준:
    - Blue cone: intensity < 0.4 (왼쪽 트랙 경계)
    - Yellow cone: intensity > 0.6 (오른쪽 트랙 경계)
    - Orange cone: 0.4 <= intensity <= 0.6 (시작/종료 라인)

발행 토픽:
    /cones/markers (MarkerArray): RViz 시각화용 콘 마커

구독 토픽:
    /fsds/lidar/Lidar1 (PointCloud2): LiDAR 포인트클라우드

사용법:
    python3 cone_classifier.py
    또는 docker-compose --profile monitoring up
"""
from typing import List, Dict, Tuple
import numpy as np


# =============================================================================
# Pure Functions (ROS 의존성 없음 - 테스트 가능)
# =============================================================================

def classify_cones(cones: List[Dict]) -> Tuple[List[Dict], List[Dict]]:
    """
    콘 리스트를 좌/우 경계로 분류.
    
    Args:
        cones: [{'x': float, 'y': float, 'z': float, ...}, ...]
    
    Returns:
        (left_cones, right_cones): 좌측(y>0), 우측(y<0) 콘 리스트
    
    분류 규칙:
        - y > 0.5: 좌측 (blue)
        - y < -0.5: 우측 (yellow)
        - |y| <= 0.5: x < 3.0이면 orange (시작선), 아니면 y 부호로 분류
    """
    left_cones = []
    right_cones = []
    
    for cone in cones:
        y = cone.get('y', 0)
        x = cone.get('x', 0)
        
        if y > 0.5:
            left_cones.append(cone)
        elif y < -0.5:
            right_cones.append(cone)
        else:
            # 중앙 영역: 시작선 콘이거나 경계 콘
            if x >= 3.0:
                # 충분히 앞에 있으면 y 부호로 분류
                if y >= 0:
                    left_cones.append(cone)
                else:
                    right_cones.append(cone)
            # x < 3.0인 시작선 콘은 분류하지 않음 (orange)
    
    return left_cones, right_cones


def cluster_cones(points: np.ndarray, grouping_threshold: float = 0.3, 
                  min_points: int = 3) -> List[Dict]:
    """
    포인트클라우드를 클러스터링하여 콘 리스트 생성.
    
    Args:
        points: (N, 4) array [x, y, z, intensity] 또는 (N, 3) [x, y, z]
        grouping_threshold: 클러스터링 거리 (m)
        min_points: 콘당 최소 포인트 수
    
    Returns:
        [{'x': float, 'y': float, 'z': float, 'intensity': float, 'points': int}, ...]
    """
    if len(points) == 0:
        return []
    
    cones = []
    used = np.zeros(len(points), dtype=bool)
    
    for i in range(len(points)):
        if used[i]:
            continue
        
        cluster = [points[i]]
        used[i] = True
        
        for j in range(i + 1, len(points)):
            if used[j]:
                continue
            
            dx = points[j, 0] - points[i, 0]
            dy = points[j, 1] - points[i, 1]
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist < grouping_threshold:
                cluster.append(points[j])
                used[j] = True
        
        if len(cluster) >= min_points:
            cluster = np.array(cluster)
            cone = {
                'x': float(np.mean(cluster[:, 0])),
                'y': float(np.mean(cluster[:, 1])),
                'z': float(np.mean(cluster[:, 2])),
                'intensity': float(np.mean(cluster[:, 3])) if cluster.shape[1] > 3 else 0.5,
                'points': len(cluster)
            }
            cones.append(cone)
    
    return cones


def filter_cone_points(points: np.ndarray, range_cutoff: float = 15.0,
                       z_min: float = -0.3, z_max: float = 0.5) -> np.ndarray:
    """
    포인트클라우드에서 콘 후보 포인트만 필터링.
    
    Args:
        points: (N, 3+) array [x, y, z, ...]
        range_cutoff: 최대 거리 (m)
        z_min, z_max: Z 높이 필터 (m)
    
    Returns:
        필터링된 포인트 배열
    """
    if len(points) == 0:
        return points
    
    dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    z_mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
    dist_mask = dist < range_cutoff
    front_mask = points[:, 0] > 0
    
    return points[z_mask & dist_mask & front_mask]


# =============================================================================
# ROS Node (시뮬레이터 연결 시에만 사용)
# =============================================================================

class ConeClassifier:
    """ROS 노드: LiDAR 콘 분류 및 시각화."""
    
    def __init__(self):
        # Lazy import ROS
        import rospy
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import Marker, MarkerArray
        import sensor_msgs.point_cloud2 as pc2
        
        self.rospy = rospy
        self.PointCloud2 = PointCloud2
        self.Marker = Marker
        self.MarkerArray = MarkerArray
        self.pc2 = pc2
        
        rospy.init_node('cone_classifier')
        
        self.cones_range_cutoff = 15.0
        self.cone_min_z = -0.3
        self.cone_max_z = 0.5
        self.cone_min_points = 3
        self.grouping_threshold = 0.3
        
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        
        self.marker_pub = rospy.Publisher('/cones/markers', MarkerArray, queue_size=1)
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        
        rospy.loginfo("Cone Classifier Started")
        rospy.loginfo("Publishing colored cone markers to /cones/markers")
    
    def lidar_callback(self, msg):
        try:
            points = list(self.pc2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        except Exception as e:
            self.rospy.logwarn_throttle(10.0, f"Intensity field unavailable: {e}")
            points = list(self.pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True))
            points = [(p[0], p[1], p[2], 0.5) for p in points]
        
        if not points:
            return
        
        points = np.array(points)
        
        # 순수 함수 사용
        filtered = filter_cone_points(
            points, self.cones_range_cutoff, self.cone_min_z, self.cone_max_z)
        
        if len(filtered) < self.cone_min_points:
            return
        
        cones = cluster_cones(filtered, self.grouping_threshold, self.cone_min_points)
        self._classify_and_store(cones)
        self.publish_markers()
    
    def _classify_and_store(self, cones: List[Dict]):
        """콘을 blue/yellow/orange로 분류하여 저장."""
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        
        for cone in cones:
            if cone['y'] > 0.5:
                self.blue_cones.append(cone)
            elif cone['y'] < -0.5:
                self.yellow_cones.append(cone)
            else:
                if cone['x'] < 3.0:
                    self.orange_cones.append(cone)
                elif cone['y'] >= 0:
                    self.blue_cones.append(cone)
                else:
                    self.yellow_cones.append(cone)
    
    def publish_markers(self):
        marker_array = self.MarkerArray()
        marker_id = 0
        
        for cone in self.blue_cones:
            marker = self._create_marker(cone, marker_id, (0.0, 0.0, 1.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        for cone in self.yellow_cones:
            marker = self._create_marker(cone, marker_id, (1.0, 1.0, 0.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        for cone in self.orange_cones:
            marker = self._create_marker(cone, marker_id, (1.0, 0.5, 0.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        delete_marker = self.Marker()
        delete_marker.action = self.Marker.DELETEALL
        delete_marker.header.frame_id = "fsds/Lidar1"
        delete_marker.header.stamp = self.rospy.Time.now()
        
        final_array = self.MarkerArray()
        final_array.markers.append(delete_marker)
        final_array.markers.extend(marker_array.markers)
        
        self.marker_pub.publish(final_array)
    
    def _create_marker(self, cone: Dict, marker_id: int, color: Tuple[float, float, float]):
        marker = self.Marker()
        marker.header.frame_id = "fsds/Lidar1"
        marker.header.stamp = self.rospy.Time.now()
        marker.ns = "cones"
        marker.id = marker_id
        marker.type = self.Marker.CYLINDER
        marker.action = self.Marker.ADD
        
        marker.pose.position.x = cone['x']
        marker.pose.position.y = cone['y']
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.9
        
        marker.lifetime = self.rospy.Duration(0.2)
        
        return marker
    
    def run(self):
        self.rospy.spin()


def main():
    try:
        classifier = ConeClassifier()
        classifier.run()
    except Exception:
        pass


if __name__ == '__main__':
    main()
