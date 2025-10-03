# Noham Behavior Tree - Room Navigation

A simplified ROS2 behavior tree package for autonomous room navigation with rotation.

## What it does

The robot will:
1. Navigate to Room 1 → Do a full 360° rotation
2. Navigate to Room 2 → Do a full 360° rotation  
3. Navigate to Room 3 → Do a full 360° rotation
4. Stop execution

## How to use

### 1. Build the package
```bash
cd ~/ros2_workspaces/noham_ws
colcon build --packages-select noham_bt
source install/setup.bash
```

### 2. Run the behavior tree
```bash
ros2 launch noham_bt room_navigation.launch.py
```

### 3. Customize room positions (optional)
You can change the room positions by passing parameters:
```bash
ros2 launch noham_bt room_navigation.launch.py room1_pose:="10.0,5.0,0.0,0.0,0.0,0.0" room2_pose:="20.0,5.0,0.0,0.0,0.0,0.0" room3_pose:="15.0,15.0,0.0,0.0,0.0,0.0"
```

## Pose Format
Each room pose is specified as: `x,y,z,roll,pitch,yaw`
- `x,y,z`: Position in meters
- `roll,pitch,yaw`: Orientation in radians

## Requirements
- ROS2 Humble or newer
- Nav2 navigation stack running
- Robot with cmd_vel interface

## Troubleshooting
- Make sure Nav2 is running: `ros2 launch nav2_bringup bringup_launch.py`
- Check if the robot can receive cmd_vel commands
- Verify the room poses are reachable by the robot
