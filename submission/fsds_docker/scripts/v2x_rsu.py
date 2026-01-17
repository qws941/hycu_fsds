#!/usr/bin/env python3
"""
V2X Virtual RSU (Roadside Unit) Node
=====================================

이 모듈은 V2X (Vehicle-to-Everything) 통신을 시뮬레이션하여
도로변 인프라(RSU)에서 차량으로 교통 정보를 전송합니다.

V2X 표준 메시지 대응 (Standards Mapping)
-----------------------------------------
본 구현의 메시지는 실제 V2X 표준과 다음과 같이 대응됩니다:

+------------------+------------------------+------------------------+
| 본 구현          | SAE J2735 (US/DSRC)    | ETSI (EU/C-ITS)        |
+------------------+------------------------+------------------------+
| speed_limit      | IVI (In-Vehicle        | IVI (Infrastructure    |
|                  | Information)           | to Vehicle Info)       |
|                  | - speedLimitType       | - ISO/TS 19091         |
+------------------+------------------------+------------------------+
| hazard           | TIM (Traveler          | DENM (Decentralized    |
|                  | Information Message)   | Environmental          |
|                  | - roadSignType         | Notification Message)  |
|                  |                        | - ETSI EN 302 637-3    |
+------------------+------------------------+------------------------+
| stop_zone        | TIM + BSM extension    | DENM (causeCode:       |
|                  | - workZone             | trafficCondition)      |
|                  | - closedLane           | + SPATEM (Signal       |
|                  |                        | Phase and Timing)      |
+------------------+------------------------+------------------------+
| rsu_status       | RSA (Road Side Alert)  | CAM (Cooperative       |
| (JSON)           | + PSM (Personal        | Awareness Message)     |
|                  | Safety Message)        | + DENM aggregation     |
+------------------+------------------------+------------------------+

메시지 형식 (Message Format)
-----------------------------
/v2x/rsu_status는 JSON 형식으로 다음 필드를 포함합니다:
{
    "timestamp": 1737123456.789,  // Unix timestamp (초)
    "scenario": "normal",
    "speed_limit": 6.0,           // m/s
    "hazard": false,
    "stop_zone": false
}

참고문헌 (References)
---------------------
- SAE J2735:2022 - V2X Communications Message Set Dictionary
- SAE J2945/1 - On-Board System Requirements for V2V Safety Communications
- ETSI EN 302 637-2 - V2X Basic Set of Applications; CAM
- ETSI EN 302 637-3 - V2X Basic Set of Applications; DENM
- ISO 19091:2019 - Using V2I/I2V for Intersection Applications

Demo Scenarios
--------------
+------------+-------------+--------+-----------+---------------------+
| Scenario   | speed_limit | hazard | stop_zone | Description         |
+------------+-------------+--------+-----------+---------------------+
| normal     | 6.0 m/s     | false  | false     | 정상 주행           |
| slow_zone  | 3.0 m/s     | false  | false     | 속도 제한 구역      |
| hazard     | 6.0 m/s     | true   | false     | 위험 경고 (50% 감속)|
| stop       | 6.0 m/s     | false  | true      | 정지 구역           |
| emergency  | 2.0 m/s     | true   | false     | 긴급 상황           |
+------------+-------------+--------+-----------+---------------------+

ROS Interface
-------------
Published Topics:
    /v2x/speed_limit (std_msgs/Float32): 속도 제한 (m/s)
    /v2x/hazard (std_msgs/Bool): 위험 경고 플래그
    /v2x/stop_zone (std_msgs/Bool): 정지 구역 플래그
    /v2x/rsu_status (std_msgs/String): 통합 상태 (JSON with timestamp)

Usage:
    rosrun fsds_scripts v2x_rsu.py                    # Normal mode
    rosrun fsds_scripts v2x_rsu.py --scenario hazard  # Specific scenario
    rosrun fsds_scripts v2x_rsu.py --interactive      # Interactive control
    rosrun fsds_scripts v2x_rsu.py --demo             # Auto-cycling demo
"""

import rospy
from std_msgs.msg import Float32, Bool, String
import argparse
import sys
import threading
import time
import json


class V2XVirtualRSU:
    
    SCENARIOS = {
        'normal': {
            'speed_limit': 6.0,
            'hazard': False,
            'stop_zone': False,
            'description': 'Normal driving conditions'
        },
        'slow_zone': {
            'speed_limit': 3.0,
            'hazard': False,
            'stop_zone': False,
            'description': 'Speed limit zone (3 m/s)'
        },
        'hazard': {
            'speed_limit': 6.0,
            'hazard': True,
            'stop_zone': False,
            'description': 'Hazard warning active'
        },
        'stop': {
            'speed_limit': 6.0,
            'hazard': False,
            'stop_zone': True,
            'description': 'Stop zone active'
        },
        'emergency': {
            'speed_limit': 2.0,
            'hazard': True,
            'stop_zone': False,
            'description': 'Emergency: slow + hazard'
        }
    }
    
    def __init__(self, initial_scenario='normal'):
        rospy.init_node('v2x_rsu', anonymous=False)
        
        self.speed_limit_pub = rospy.Publisher('/v2x/speed_limit', Float32, queue_size=1)
        self.hazard_pub = rospy.Publisher('/v2x/hazard', Bool, queue_size=1)
        self.stop_zone_pub = rospy.Publisher('/v2x/stop_zone', Bool, queue_size=1)
        self.status_pub = rospy.Publisher('/v2x/rsu_status', String, queue_size=1)
        
        self.data_lock = threading.Lock()
        self.speed_limit = 6.0
        self.hazard = False
        self.stop_zone = False
        self.current_scenario = initial_scenario
        
        self.set_scenario(initial_scenario)
        self.rate = rospy.Rate(10)
        
        rospy.loginfo("=== V2X Virtual RSU Started ===")
        rospy.loginfo(f"Initial scenario: {initial_scenario}")
        rospy.loginfo("Topics: /v2x/speed_limit, /v2x/hazard, /v2x/stop_zone")
        
    def set_scenario(self, scenario_name):
        if scenario_name not in self.SCENARIOS:
            rospy.logwarn(f"Unknown scenario: {scenario_name}, using 'normal'")
            scenario_name = 'normal'
            
        scenario = self.SCENARIOS[scenario_name]
        with self.data_lock:
            self.speed_limit = scenario['speed_limit']
            self.hazard = scenario['hazard']
            self.stop_zone = scenario['stop_zone']
            self.current_scenario = scenario_name
        
        rospy.loginfo(f"[V2X RSU] Scenario: {scenario_name} - {scenario['description']}")
        rospy.loginfo(f"  speed_limit={scenario['speed_limit']} m/s, hazard={scenario['hazard']}, stop_zone={scenario['stop_zone']}")
        
    def set_speed_limit(self, limit):
        with self.data_lock:
            self.speed_limit = max(0.0, min(limit, 10.0))
        rospy.loginfo(f"[V2X RSU] Speed limit set to {self.speed_limit} m/s")
        
    def set_hazard(self, active):
        with self.data_lock:
            self.hazard = bool(active)
        rospy.loginfo(f"[V2X RSU] Hazard {'ACTIVE' if self.hazard else 'cleared'}")
        
    def set_stop_zone(self, active):
        with self.data_lock:
            self.stop_zone = bool(active)
        rospy.loginfo(f"[V2X RSU] Stop zone {'ACTIVE' if self.stop_zone else 'cleared'}")
        
    def publish_messages(self):
        with self.data_lock:
            speed = self.speed_limit
            hazard = self.hazard
            stop = self.stop_zone
            scenario = self.current_scenario
        
        self.speed_limit_pub.publish(Float32(speed))
        self.hazard_pub.publish(Bool(hazard))
        self.stop_zone_pub.publish(Bool(stop))
        
        # JSON format with timestamp for V2X standard compliance
        status = json.dumps({
            "timestamp": time.time(),
            "scenario": scenario,
            "speed_limit": speed,
            "hazard": hazard,
            "stop_zone": stop
        })
        self.status_pub.publish(String(status))
        
    def run(self):
        while not rospy.is_shutdown():
            self.publish_messages()
            self.rate.sleep()
            
    def run_demo_sequence(self):
        rospy.loginfo("=== V2X Demo Sequence Started ===")
        rospy.loginfo("Will cycle through scenarios every 10 seconds")
        
        scenario_order = ['normal', 'slow_zone', 'hazard', 'stop', 'normal']
        scenario_idx = 0
        last_switch = rospy.Time.now()
        switch_interval_sec = rospy.Duration(10.0)
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            
            if now - last_switch > switch_interval_sec:
                scenario_idx = (scenario_idx + 1) % len(scenario_order)
                self.set_scenario(scenario_order[scenario_idx])
                last_switch = now
                
            self.publish_messages()
            self.rate.sleep()


class InteractiveController:
    
    def __init__(self, rsu):
        self.rsu = rsu
        self.running = True
        
    def print_help(self):
        help_text = """
=== V2X RSU Interactive Controls ===
Scenarios:
  1 - Normal (speed=6.0, no warnings)
  2 - Slow Zone (speed=3.0)
  3 - Hazard Warning
  4 - Stop Zone
  5 - Emergency (slow + hazard)

Manual Controls:
  s <value> - Set speed limit (e.g., 's 4.0')
  h         - Toggle hazard
  x         - Toggle stop zone
  r         - Reset to normal
  q         - Quit
=====================================
"""
        print(help_text)
        
    def run(self):
        self.print_help()
        
        pub_thread = threading.Thread(target=self.rsu.run, daemon=True)
        pub_thread.start()
        
        while self.running and not rospy.is_shutdown():
            try:
                cmd = input("V2X> ").strip().lower()
                
                if not cmd:
                    continue
                elif cmd == 'q':
                    rospy.loginfo("Shutting down V2X RSU...")
                    self.running = False
                    break
                elif cmd == '1':
                    self.rsu.set_scenario('normal')
                elif cmd == '2':
                    self.rsu.set_scenario('slow_zone')
                elif cmd == '3':
                    self.rsu.set_scenario('hazard')
                elif cmd == '4':
                    self.rsu.set_scenario('stop')
                elif cmd == '5':
                    self.rsu.set_scenario('emergency')
                elif cmd == 'h':
                    self.rsu.set_hazard(not self.rsu.hazard)
                elif cmd == 'x':
                    self.rsu.set_stop_zone(not self.rsu.stop_zone)
                elif cmd == 'r':
                    self.rsu.set_scenario('normal')
                elif cmd.startswith('s '):
                    try:
                        val = float(cmd[2:])
                        self.rsu.set_speed_limit(val)
                    except ValueError:
                            rospy.logwarn("Invalid speed value")
                elif cmd == 'help' or cmd == '?':
                    self.print_help()
                else:
                    rospy.loginfo(f"Unknown command: {cmd}. Type 'help' for available commands.")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break


def main():
    parser = argparse.ArgumentParser(description='V2X Virtual RSU Node')
    parser.add_argument('--scenario', '-s', type=str, default='normal',
                        choices=list(V2XVirtualRSU.SCENARIOS.keys()),
                        help='Initial scenario (default: normal)')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Run in interactive mode with keyboard controls')
    parser.add_argument('--demo', '-d', action='store_true',
                        help='Run demo sequence cycling through scenarios')
    
    args, _ = parser.parse_known_args()
    
    try:
        rsu = V2XVirtualRSU(initial_scenario=args.scenario)
        
        if args.interactive:
            controller = InteractiveController(rsu)
            controller.run()
        elif args.demo:
            rsu.run_demo_sequence()
        else:
            rsu.run()
            
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
