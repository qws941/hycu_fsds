#!/usr/bin/env python3
import rospy
import numpy as np
from math import sqrt
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Int32


class LapTimer:
    def __init__(self):
        rospy.init_node('lap_timer')
        
        self.start_x = None
        self.start_y = None
        self.start_threshold = 3.0
        self.min_lap_distance = 50.0
        
        self.lap_count = 0
        self.lap_times = []
        self.lap_start_time = None
        self.total_distance = 0.0
        self.last_x = None
        self.last_y = None
        self.current_speed = 0.0
        self.max_speed = 0.0
        self.away_from_start = False
        
        self.lap_count_pub = rospy.Publisher('/lap/count', Int32, queue_size=1)
        self.lap_time_pub = rospy.Publisher('/lap/current_time', Float32, queue_size=1)
        self.best_lap_pub = rospy.Publisher('/lap/best_time', Float32, queue_size=1)
        self.distance_pub = rospy.Publisher('/lap/distance', Float32, queue_size=1)
        self.avg_speed_pub = rospy.Publisher('/lap/avg_speed', Float32, queue_size=1)
        self.max_speed_pub = rospy.Publisher('/lap/max_speed', Float32, queue_size=1)
        self.hud_pub = rospy.Publisher('/lap/hud', String, queue_size=1)
        
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Lap Timer Started")
        rospy.loginfo("Waiting for first position to set start/finish line...")
        rospy.loginfo("=" * 50)
    
    def odom_callback(self, msg):
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            
            if not all(map(lambda v: isinstance(v, (int, float)) and not (v != v), [x, y, vx, vy])):
                return
            
            self.current_speed = sqrt(vx**2 + vy**2)
        
            if self.current_speed > self.max_speed:
                self.max_speed = self.current_speed
            
            if self.start_x is None:
                self.start_x = x
                self.start_y = y
                self.last_x = x
                self.last_y = y
                self.lap_start_time = rospy.Time.now()
                rospy.loginfo(f"Start/Finish set at ({x:.1f}, {y:.1f})")
                return
            
            if self.last_x is not None:
                dx = x - self.last_x
                dy = y - self.last_y
                self.total_distance += sqrt(dx**2 + dy**2)
            
            self.last_x = x
            self.last_y = y
            
            dist_to_start = sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
            
            if not self.away_from_start and dist_to_start > self.min_lap_distance:
                self.away_from_start = True
            
            if self.away_from_start and dist_to_start < self.start_threshold:
                self.complete_lap()
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Odom callback error: {e}")
    
    def complete_lap(self):
        now = rospy.Time.now()
        lap_time = (now - self.lap_start_time).to_sec()
        
        if lap_time < 5.0:
            return
        
        self.lap_count += 1
        self.lap_times.append(lap_time)
        
        best_time = min(self.lap_times)
        avg_speed = self.total_distance / lap_time if lap_time > 0 else 0
        
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"LAP {self.lap_count} COMPLETE!")
        rospy.loginfo(f"Time: {lap_time:.2f}s")
        rospy.loginfo(f"Best: {best_time:.2f}s")
        rospy.loginfo(f"Avg Speed: {avg_speed:.1f} m/s")
        rospy.loginfo(f"Max Speed: {self.max_speed:.1f} m/s")
        rospy.loginfo("=" * 50)
        
        self.lap_start_time = now
        self.total_distance = 0.0
        self.max_speed = 0.0
        self.away_from_start = False
    
    def publish_telemetry(self):
        self.lap_count_pub.publish(Int32(self.lap_count))
        
        if self.lap_start_time:
            current_time = (rospy.Time.now() - self.lap_start_time).to_sec()
            self.lap_time_pub.publish(Float32(current_time))
        
        if self.lap_times:
            self.best_lap_pub.publish(Float32(min(self.lap_times)))
        
        self.distance_pub.publish(Float32(self.total_distance))
        self.max_speed_pub.publish(Float32(self.max_speed))
        
        if self.lap_start_time:
            current_time = (rospy.Time.now() - self.lap_start_time).to_sec()
            avg_speed = self.total_distance / current_time if current_time > 0 else 0
            self.avg_speed_pub.publish(Float32(avg_speed))
        
        hud = self.format_hud()
        self.hud_pub.publish(String(hud))
    
    def format_hud(self):
        if not self.lap_start_time:
            return "WAITING FOR START..."
        
        current_time = (rospy.Time.now() - self.lap_start_time).to_sec()
        best_time = min(self.lap_times) if self.lap_times else 0.0
        avg_speed = self.total_distance / current_time if current_time > 0 else 0
        
        lines = [
            "╔════════════════════════════╗",
            f"║  LAP: {self.lap_count:3d}                  ║",
            f"║  TIME: {current_time:6.1f}s             ║",
            f"║  BEST: {best_time:6.1f}s             ║",
            f"║  SPEED: {self.current_speed:5.1f} m/s          ║",
            f"║  AVG:   {avg_speed:5.1f} m/s          ║",
            f"║  DIST:  {self.total_distance:6.1f}m           ║",
            "╚════════════════════════════╝"
        ]
        return "\n".join(lines)
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_telemetry()
            rate.sleep()


def main():
    try:
        timer = LapTimer()
        timer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
