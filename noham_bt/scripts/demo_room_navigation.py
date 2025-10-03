#!/usr/bin/env python3
"""
Demo script for the simplified room navigation behavior tree.
This shows the sequence of actions the robot will perform.
"""


def print_mission():
    print("🚀 ROOM NAVIGATION MISSION")
    print("=" * 40)
    print("📋 Mission Sequence:")
    print("1. Navigate to Room 1")
    print("   └── Do full 360° rotation")
    print("2. Navigate to Room 2")
    print("   └── Do full 360° rotation")
    print("3. Navigate to Room 3")
    print("   └── Do full 360° rotation")
    print("4. Mission Complete - Stop")
    print("=" * 40)
    print("🎯 Default Room Positions:")
    print("   Room 1: (14.19, -6.99, 0.0) - Yaw: 1.60 rad")
    print("   Room 2: (25.21, 0.06, 0.0) - Yaw: -3.09 rad")
    print("   Room 3: (14.45, 7.98, 0.0) - Yaw: -1.59 rad")
    print("=" * 40)
    print("⚡ To run the mission:")
    print("   ros2 launch noham_bt room_navigation.launch.py")
    print("=" * 40)
    print("🔧 To customize room positions:")
    print("   ros2 launch noham_bt room_navigation.launch.py \\")
    print("     room1_pose:=\"10.0,5.0,0.0,0.0,0.0,0.0\" \\")
    print("     room2_pose:=\"20.0,5.0,0.0,0.0,0.0,0.0\" \\")
    print("     room3_pose:=\"15.0,15.0,0.0,0.0,0.0,0.0\"")


if __name__ == "__main__":
    print_mission()
