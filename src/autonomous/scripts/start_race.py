#!/usr/bin/env python3
import airsim
import time

def start_race():
    print("[1/2] Connecting to FSDS simulator...")
    client = airsim.CarClient(ip="172.19.0.1", port=41451)
    client.confirmConnection()
    print("      Connected!")
    
    print("[2/2] Enabling API control...")
    client.enableApiControl(True)
    print("      API control enabled!")
    
    state = client.getCarState()
    print(f"\n=== Vehicle Ready ===")
    print(f"Speed: {state.speed:.2f} m/s")
    print("\nRace ready!")
    
    return client

if __name__ == "__main__":
    start_race()
