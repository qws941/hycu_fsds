#!/usr/bin/env python3
"""
V2X Virtual RSU (Roadside Unit) Node
=====================================
Simulates V2X infrastructure broadcasting messages to vehicles.

Published Topics:
    /v2x/speed_limit (Float32): Current speed limit in m/s
    /v2x/hazard (Bool): Hazard warning flag
    /v2x/stop_zone (Bool): Stop zone active flag

Demo Scenarios:
    1. Normal: speed_limit=6.0, no hazard, no stop
    2. Speed Limit Zone: speed_limit=3.0
    3. Hazard Warning: hazard=True (vehicle slows to 50%)
    4. Stop Zone: stop_zone=True (vehicle stops)

Usage:
    rosrun fsds_scripts v2x_rsu.py
    rosrun fsds_scripts v2x_rsu.py --scenario hazard
    rosrun fsds_scripts v2x_rsu.py --interactive
"""

import rospy
from std_msgs.msg import Float32, Bool, String
import argparse
import sys
import threading


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
        self.speed_limit = scenario['speed_limit']
        self.hazard = scenario['hazard']
        self.stop_zone = scenario['stop_zone']
        self.current_scenario = scenario_name
        
        rospy.loginfo(f"[V2X RSU] Scenario: {scenario_name} - {scenario['description']}")
        rospy.loginfo(f"  speed_limit={self.speed_limit} m/s, hazard={self.hazard}, stop_zone={self.stop_zone}")
        
    def set_speed_limit(self, limit):
        self.speed_limit = max(0.0, min(limit, 10.0))
        rospy.loginfo(f"[V2X RSU] Speed limit set to {self.speed_limit} m/s")
        
    def set_hazard(self, active):
        self.hazard = bool(active)
        rospy.loginfo(f"[V2X RSU] Hazard {'ACTIVE' if self.hazard else 'cleared'}")
        
    def set_stop_zone(self, active):
        self.stop_zone = bool(active)
        rospy.loginfo(f"[V2X RSU] Stop zone {'ACTIVE' if self.stop_zone else 'cleared'}")
        
    def publish_messages(self):
        self.speed_limit_pub.publish(Float32(self.speed_limit))
        self.hazard_pub.publish(Bool(self.hazard))
        self.stop_zone_pub.publish(Bool(self.stop_zone))
        
        status = f"scenario={self.current_scenario},speed_limit={self.speed_limit:.1f},hazard={self.hazard},stop={self.stop_zone}"
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
        print("\n=== V2X RSU Interactive Controls ===")
        print("Scenarios:")
        print("  1 - Normal (speed=6.0, no warnings)")
        print("  2 - Slow Zone (speed=3.0)")
        print("  3 - Hazard Warning")
        print("  4 - Stop Zone")
        print("  5 - Emergency (slow + hazard)")
        print("\nManual Controls:")
        print("  s <value> - Set speed limit (e.g., 's 4.0')")
        print("  h         - Toggle hazard")
        print("  x         - Toggle stop zone")
        print("  r         - Reset to normal")
        print("  q         - Quit")
        print("=====================================\n")
        
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
                        print("Invalid speed value")
                elif cmd == 'help' or cmd == '?':
                    self.print_help()
                else:
                    print(f"Unknown command: {cmd}. Type 'help' for available commands.")
                    
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
