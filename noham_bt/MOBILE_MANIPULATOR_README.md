# Mobile Manipulator Behavior Tree

This behavior tree orchestrates a complete pick-and-place mission for the mobile manipulator robot.

## Mission Overview

The behavior tree executes the following sequence of tasks:

1. **Navigate to First Position**: Move to position (14.5, -6.4, 0) with orientation (0,0,0,1)
2. **Mobile Manipulator Approach**: Approach the AprilTag using the mobile base
3. **Arm Planning - Pick**: Execute arm movement to pick position
4. **Gripper Action - Grasp**: Close gripper to grasp the object
5. **Arm Planning - Init Pos**: Return arm to initial position
6. **Navigate Back to First Position**: Return to position (14.5, -6.4, 0) with orientation (0,0,0,1)
7. **Navigate to Drop Position**: Move to position (0, 1.0, 0) with orientation (0, 0, 0.717, 0.717) - 90 degrees
8. **Arm Planning - Drop**: Execute arm movement to drop position
9. **Gripper Action - Drop**: Open gripper to release the object

## Prerequisites

- ROS2 Humble
- BehaviorTree.CPP 4.6
- Nav2 navigation stack
- MoveIt for arm planning
- AprilTag detection system
- Manipulator control node

## Launching the System

### Option 1: Launch Everything Together
```bash
ros2 launch noham_bt mobile_manipulator_bt.launch.py
```

This launch file starts both:
- `manipulator_control_node`: Handles all manipulator actions
- `mobile_manipulator_behavior_tree`: Orchestrates the mission

### Option 2: Launch Separately
```bash
# Terminal 1: Launch manipulator control
ros2 launch noham_bt manipulator_control.launch.py

# Terminal 2: Launch behavior tree
ros2 run noham_bt mobile_manipulator_bt
```

## Monitoring with Groot2

The behavior tree publishes debug information to Groot2 on port 1667:

1. Install Groot2 from: https://github.com/BehaviorTree/Groot2
2. Open Groot2
3. Connect to port 1667
4. Monitor the tree execution in real-time

## Behavior Tree Structure

```
MobileManipulatorMission (Sequence)
├── NavigateToFirstPosition (NavigateToPose)
├── MobileManipulatorApproach (MobileManipulatorApproach)
├── ArmPlanningPick (ArmPlanning)
├── GripperGrasp (GripperAction)
├── ArmPlanningInitPos (ArmPlanning)
├── NavigateBackToFirstPosition (NavigateToPose)
├── NavigateToDropPosition (NavigateToPose)
├── ArmPlanningDrop (ArmPlanning)
└── GripperDrop (GripperAction)
```

## Action Interfaces

### MobileManipulatorApproach
- **Action**: `/mobile_manipulator_approach`
- **Interface**: `noham_custom_interfaces/action/MobileManipulatorApproach`
- **Goal**: `start: true`

### ArmPlanning
- **Action**: `/arm_planning`
- **Interface**: `noham_custom_interfaces/action/ArmPlanning`
- **Goal**: `command: "pick" | "drop" | "init_pos"`

### GripperAction
- **Action**: `/gripper_action`
- **Interface**: `noham_custom_interfaces/action/GripperAction`
- **Goal**: `command: "grasp" | "drop"`

## Configuration

The behavior tree configuration is defined in `config/mobile_manipulator_tree.xml`. You can modify:

- Navigation poses
- Action sequences
- Node parameters

## Troubleshooting

### Common Issues

1. **Action servers not available**: Ensure all required nodes are running
2. **TF transforms missing**: Check that the robot's TF tree is properly configured
3. **Navigation failures**: Verify that the map and navigation stack are working
4. **Arm planning failures**: Check MoveIt configuration and robot state

### Debug Information

The behavior tree provides extensive logging:
- Console output shows mission progress
- Groot2 provides visual tree execution status
- Each node logs its actions and results

## Customization

To modify the mission:

1. Edit `config/mobile_manipulator_tree.xml` to change the sequence
2. Modify node parameters in the XML
3. Add new action nodes if needed
4. Rebuild the package: `colcon build --packages-select noham_bt`

## Dependencies

- `noham_custom_interfaces`: Action definitions
- `nav2_msgs`: Navigation actions
- `behaviortree_cpp`: Behavior tree framework
- `rclcpp_action`: ROS2 action client/server
- `tf2_ros`: Transform handling
