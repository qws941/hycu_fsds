#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2


class ConeClassifier:
    def __init__(self):
        rospy.init_node('cone_classifier')
        
        self.cones_range_cutoff = 15.0
        self.cone_min_z = -0.3
        self.cone_max_z = 0.5
        self.cone_min_points = 3
        self.grouping_threshold = 0.3
        self.intensity_threshold = 0.5
        
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        
        self.marker_pub = rospy.Publisher('/cones/markers', MarkerArray, queue_size=1)
        
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        
        rospy.loginfo("Cone Classifier Started")
        rospy.loginfo("Publishing colored cone markers to /cones/markers")
    
    def lidar_callback(self, msg):
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        except:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            points = [(p[0], p[1], p[2], 0.5) for p in points]
        
        if not points:
            return
        
        points = np.array(points)
        
        dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        z_mask = (points[:, 2] > self.cone_min_z) & (points[:, 2] < self.cone_max_z)
        dist_mask = dist < self.cones_range_cutoff
        front_mask = points[:, 0] > 0
        
        filtered = points[z_mask & dist_mask & front_mask]
        
        if len(filtered) < self.cone_min_points:
            return
        
        cones = self.cluster_cones(filtered)
        self.classify_cones(cones)
        self.publish_markers()
    
    def cluster_cones(self, points):
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
                
                if dist < self.grouping_threshold:
                    cluster.append(points[j])
                    used[j] = True
            
            if len(cluster) >= self.cone_min_points:
                cluster = np.array(cluster)
                cone = {
                    'x': np.mean(cluster[:, 0]),
                    'y': np.mean(cluster[:, 1]),
                    'z': np.mean(cluster[:, 2]),
                    'intensity': np.mean(cluster[:, 3]) if cluster.shape[1] > 3 else 0.5,
                    'points': len(cluster)
                }
                cones.append(cone)
        
        return cones
    
    def classify_cones(self, cones):
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
        marker_array = MarkerArray()
        marker_id = 0
        
        for cone in self.blue_cones:
            marker = self.create_cone_marker(cone, marker_id, (0.0, 0.0, 1.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        for cone in self.yellow_cones:
            marker = self.create_cone_marker(cone, marker_id, (1.0, 1.0, 0.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        for cone in self.orange_cones:
            marker = self.create_cone_marker(cone, marker_id, (1.0, 0.5, 0.0))
            marker_array.markers.append(marker)
            marker_id += 1
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = "fsds/Lidar1"
        delete_marker.header.stamp = rospy.Time.now()
        
        final_array = MarkerArray()
        final_array.markers.append(delete_marker)
        final_array.markers.extend(marker_array.markers)
        
        self.marker_pub.publish(final_array)
    
    def create_cone_marker(self, cone, marker_id, color):
        marker = Marker()
        marker.header.frame_id = "fsds/Lidar1"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cones"
        marker.id = marker_id
        marker.type = Marker.CONE
        marker.action = Marker.ADD
        
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
        
        marker.lifetime = rospy.Duration(0.2)
        
        return marker
    
    def run(self):
        rospy.spin()


def main():
    try:
        classifier = ConeClassifier()
        classifier.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
