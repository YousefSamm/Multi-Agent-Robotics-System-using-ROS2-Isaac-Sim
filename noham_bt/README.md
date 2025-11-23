# Noham Behavior Tree Package

A comprehensive ROS2 behavior tree package for autonomous robotic missions using BehaviorTree.CPP 4.6. This package provides behavior tree implementations for multiple robot types including mobile manipulators, forklifts, and Spot robots.

## Overview

This package contains behavior tree implementations for:
- **Mobile Manipulator**: Complete pick-and-place missions with navigation and manipulation
- **Forklift**: Autonomous navigation and pallet manipulation missions
- **Spot Robot**: Quadrupedal locomotion and inspection tasks
- **Room Navigation**: Simple room-to-room navigation with rotation

## Package Structure

```
noham_bt/
├── src/                    # C++ source files
│   ├── behavior_tree_nodes.cpp      # Common behavior tree nodes
│   ├── mobile_manipulator_bt.cpp    # Mobile manipulator behavior tree
│   ├── forklift_bt.cpp              # Forklift mission behavior tree
│   ├── spot_bt.cpp                  # Spot robot behavior tree
│   ├── forklift_approach.cpp        # Forklift approach action
│   ├── lifter_action.cpp            # Lifter control action
│   ├── manipulator_control.cpp      # Manipulator control node
│   └── noham_actions.cpp             # Action server for all actions
├── include/                # Header files
├── launch/                 # Launch files
│   ├── mobile_manipulator_bt.launch.py
│   ├── forklift_mission.launch.py
│   ├── spot_bt.launch.py
│   ├── room_navigation.launch.py
│   ├── forklift_approach.launch.py
│   ├── manipulator_control.launch.py
│   └── lifter_action.launch.py
├── config/                 # Behavior tree XML configurations
│   ├── mobile_manipulator_tree.xml
│   ├── forklift_mission_tree.xml
│   ├── room_exploration_tree.xml
│   └── room_poses.yaml
└── scripts/                # Utility scripts
```

## Prerequisites

- ROS2 Humble or newer
- BehaviorTree.CPP 4.6
- Nav2 navigation stack
- MoveIt2 (for mobile manipulator)
- `noham_custom_interfaces` package

## Quick Start

### Build the Package

```bash
cd ~/ros2_workspaces/noham_ws
colcon build --packages-select noham_bt
source install/setup.bash
```

## Behavior Trees

### 1. Mobile Manipulator Behavior Tree

Complete pick-and-place mission for mobile manipulator (Nova Carter Base + Jaco arm).

**Launch:**
```bash
ros2 launch noham_bt mobile_manipulator_bt.launch.py
```

**What it does:**
1. Navigate to first position
2. Approach AprilTag using mobile base
3. Plan arm movement to pick position
4. Grasp object with gripper
5. Return arm to initial position
6. Navigate to drop position
7. Plan arm movement to drop position
8. Release object

**Detailed Documentation:** See [MOBILE_MANIPULATOR_README.md](MOBILE_MANIPULATOR_README.md)

### 2. Forklift Mission Behavior Tree

Autonomous forklift mission with navigation and pallet manipulation.

**Launch:**
```bash
ros2 launch noham_bt forklift_mission.launch.py
```

**What it does:**
1. Navigate to first position
2. Execute forklift approach action
3. Lift up
4. Navigate to second position
5. Navigate to final position
6. Lift down

**Detailed Documentation:** See [FORKLIFT_MISSION_README.md](FORKLIFT_MISSION_README.md)

### 3. Spot Robot Behavior Tree

Behavior tree for Boston Dynamics Spot robot.

**Launch:**
```bash
ros2 launch noham_bt spot_bt.launch.py
```

**Detailed Documentation:** See [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)

### 4. Room Navigation Behavior Tree

Simple room-to-room navigation with rotation.

**Launch:**
```bash
ros2 launch noham_bt room_navigation.launch.py
```

**What it does:**
1. Navigate to Room 1 → Do a full 360° rotation
2. Navigate to Room 2 → Do a full 360° rotation  
3. Navigate to Room 3 → Do a full 360° rotation
4. Stop execution

**Customize room positions:**
```bash
ros2 launch noham_bt room_navigation.launch.py \
  room1_pose:="10.0,5.0,0.0,0.0,0.0,0.0" \
  room2_pose:="20.0,5.0,0.0,0.0,0.0,0.0" \
  room3_pose:="15.0,15.0,0.0,0.0,0.0,0.0"
```

**Pose Format:** `x,y,z,roll,pitch,yaw` (position in meters, orientation in radians)

## Individual Action Nodes

### Forklift Approach

Launches only the forklift approach action server:

```bash
ros2 launch noham_bt forklift_approach.launch.py
```

**Documentation:** See [FORKLIFT_APPROACH_README.md](FORKLIFT_APPROACH_README.md)

### Manipulator Control

Launches the manipulator control node for mobile manipulator:

```bash
ros2 launch noham_bt manipulator_control.launch.py
```

This node handles:
- Arm planning actions
- Gripper actions
- Mobile manipulator approach actions

### Lifter Action

Launches the lifter action server:

```bash
ros2 launch noham_bt lifter_action.launch.py
```

**Documentation:** See [LIFTER_SERVICE_README.md](LIFTER_SERVICE_README.md)

## Monitoring with Groot2

Groot2 is a visual editor and debugger for BehaviorTree.CPP that allows real-time monitoring of behavior tree execution.

### Setup

1. **Download Groot2**: Get the latest release from [Groot2 Releases](https://github.com/BehaviorTree/Groot2/releases)
2. **Open Project**: In Groot2, open `config/noham_bt_groot_project.groot`
3. **Connect**: Click Connect and enter `localhost:1667`

### Visual Indicators

- 🟢 **Green nodes**: Successfully completed
- 🔴 **Red nodes**: Failed execution
- 🟡 **Yellow nodes**: Currently running
- ⚪ **White nodes**: Waiting to execute

### Test Connection

```bash
python3 scripts/test_groot_connection.py
```

**Detailed Guide:** See [GROOT_SETUP_GUIDE.md](GROOT_SETUP_GUIDE.md)

## Custom Behavior Tree Nodes

The package provides several custom node types:

1. **NavigateToPose**: Sends navigation goals to Nav2
2. **ForkliftApproach**: Executes forklift approach action
3. **LifterAction**: Controls lift mechanism (up/down)
4. **MobileManipulatorApproach**: Approaches target using mobile base
5. **ArmPlanning**: Plans arm movements via MoveIt2
6. **GripperAction**: Controls gripper (grasp/drop)

All nodes are defined in `include/noham_bt/behavior_tree_nodes.hpp` and implemented in `src/behavior_tree_nodes.cpp`.

## Action Interfaces

All behavior trees use ROS2 action interfaces defined in `noham_custom_interfaces`:

- `/mobile_manipulator_approach` - `MobileManipulatorApproach.action`
- `/arm_planning` - `ArmPlanning.action`
- `/gripper_action` - `GripperAction.action`
- `/forklift_approach` - `ForkliftApproach.action`
- `/lifter_action` - `LifterAction.action`
- `/navigate_to_pose` - `nav2_msgs/action/NavigateToPose.action`

## Configuration

Behavior tree configurations are defined in XML files in the `config/` directory:

- `mobile_manipulator_tree.xml` - Mobile manipulator mission
- `forklift_mission_tree.xml` - Forklift mission
- `room_exploration_tree.xml` - Room navigation
- `room_poses.yaml` - Room position definitions

You can modify these files to customize mission sequences and parameters.

## Dependencies

- `noham_custom_interfaces` - Action and message definitions
- `nav2_msgs` - Navigation actions
- `behaviortree_cpp` - Behavior tree framework
- `rclcpp_action` - ROS2 action client/server
- `tf2_ros` - Transform handling
- `moveit_msgs` - MoveIt2 integration (for mobile manipulator)

## Troubleshooting

### Common Issues

1. **Action servers not available**: Ensure all required nodes are running
   ```bash
   ros2 node list
   ros2 action list
   ```

2. **TF transforms missing**: Check that the robot's TF tree is properly configured
   ```bash
   ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>
   ```

3. **Navigation failures**: Verify that Nav2 is running and map is available
   ```bash
   ros2 launch nav2_bringup bringup_launch.py
   ```

4. **Arm planning failures**: Check MoveIt2 configuration and robot state
   ```bash
   ros2 launch kinova_moveit_config noham_jaco.launch.py
   ```

5. **Groot connection fails**: Ensure behavior tree is running and port 1667 is accessible
   ```bash
   python3 scripts/test_groot_connection.py
   ```

### Debug Commands

```bash
# Check running nodes
ros2 node list

# Monitor behavior tree logs
ros2 topic echo /rosout

# Check action servers
ros2 action list

# Monitor specific action
ros2 action info /arm_planning
```

## Additional Documentation

- [MOBILE_MANIPULATOR_README.md](MOBILE_MANIPULATOR_README.md) - Mobile manipulator detailed guide
- [FORKLIFT_MISSION_README.md](FORKLIFT_MISSION_README.md) - Forklift mission guide
- [FORKLIFT_APPROACH_README.md](FORKLIFT_APPROACH_README.md) - Forklift approach details
- [LIFTER_SERVICE_README.md](LIFTER_SERVICE_README.md) - Lifter service documentation
- [GROOT_SETUP_GUIDE.md](GROOT_SETUP_GUIDE.md) - Complete Groot setup guide
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Implementation details
- [SIMPLIFIED_README.md](SIMPLIFIED_README.md) - Simplified overview

## Building Individual Components

```bash
# Build only noham_bt
colcon build --packages-select noham_bt

# Build with dependencies
colcon build --packages-up-to noham_bt

# Clean and rebuild
rm -rf build install log
colcon build --symlink-install
```

## Contributing

When adding new behavior trees:
1. Create the C++ source file in `src/`
2. Add the executable to `CMakeLists.txt`
3. Create launch file in `launch/`
4. Create XML configuration in `config/`
5. Update this README
