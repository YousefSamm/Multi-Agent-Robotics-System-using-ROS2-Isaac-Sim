# Forklift Mission Behavior Tree

This behavior tree implements a complete forklift mission sequence using ROS2 and BehaviorTree.CPP 4.6.

## Mission Sequence

The behavior tree executes the following sequence of actions:

1. **Navigate to First Position**: Move to (25.0, 0.0, 0.0) with orientation -1.57 radians
2. **Forklift Approach**: Execute the forklift approach action to approach the target
3. **Lift Up**: Raise the lift mechanism
4. **Navigate to Second Position**: Move to (27.0, 0.0, 0.0) with orientation 3.14 radians
5. **Navigate to Final Position**: Move to (-0.721848, 1.78605, 0.0) with orientation 1.57026 radians
6. **Lift Down**: Lower the lift mechanism

## Files

- `src/forklift_bt.cpp`: Main behavior tree executable
- `config/forklift_mission_tree.xml`: Behavior tree XML configuration
- `launch/forklift_mission.launch.py`: Launch file for the complete mission
- `include/noham_bt/behavior_tree_nodes.hpp`: Header with node definitions
- `src/behavior_tree_nodes.cpp`: Implementation of all behavior tree nodes

## Usage

### Build the Package

```bash
cd ~/ros2_workspaces/noham_ws
colcon build --packages-select noham_bt
source install/setup.bash
```

### Run the Mission

```bash
ros2 launch noham_bt forklift_mission.launch.py
```

This will launch:
- `forklift_behavior_tree`: The main behavior tree node
- `noham_actions`: The action server for forklift and lifter actions

### Prerequisites

- ROS2 Humble or later
- nav2 navigation stack running
- `noham_custom_interfaces` package with action definitions
- BehaviorTree.CPP 4.6 installed

### Dependencies

The behavior tree depends on:
- `nav2_msgs` for navigation actions
- `noham_custom_interfaces` for forklift and lifter actions
- `geometry_msgs` for pose messages
- `tf2` for coordinate transformations

## Behavior Tree Structure

The XML file defines a simple sequence of actions that execute in order. Each action waits for completion before proceeding to the next:

```xml
<Sequence name="ForkliftMission">
    <Action ID="NavigateToPose" name="NavigateToFirstPosition" target_pose="25.0,0.0,0.0,0.0,0.0,-1.57"/>
    <Action ID="ForkliftApproach" name="ForkliftApproachAction"/>
    <Action ID="LifterAction" name="LiftUp" lift="true"/>
    <Action ID="NavigateToPose" name="NavigateToSecondPosition" target_pose="27.0,0.0,0.0,0.0,0.0,3.14"/>
    <Action ID="NavigateToPose" name="NavigateToFinalPosition" target_pose="-0.721848,1.78605,0.0,0.0,0.0,1.57026"/>
    <Action ID="LifterAction" name="LiftDown" lift="false"/>
</Sequence>
```

## Custom Nodes

The behavior tree uses three custom node types:

1. **NavigateToPose**: Sends navigation goals to nav2
2. **ForkliftApproach**: Executes the forklift approach action
3. **LifterAction**: Controls the lift mechanism (up/down)

Each node properly handles success/failure states and integrates with the ROS2 action system.

## Debugging with Groot

### What is Groot?

Groot is a visual editor and debugger for BehaviorTree.CPP that allows you to:
- **Visualize** behavior tree structure in real-time
- **Monitor** node execution states (SUCCESS/FAILURE/RUNNING)
- **Debug** issues by seeing exactly which nodes are active
- **Analyze** performance and execution flow

### Setup Groot

1. **Download Groot2**: Get the latest release from [Groot2 Releases](https://github.com/BehaviorTree/Groot2/releases)
2. **Open Project**: In Groot, open `config/noham_bt_groot_project.groot`
3. **Connect**: Click the Connect button and enter `localhost:1667`

### Monitor Execution

Once connected, you'll see:
- 🟢 **Green nodes**: Successfully completed
- 🔴 **Red nodes**: Failed execution
- 🟡 **Yellow nodes**: Currently running
- ⚪ **White nodes**: Waiting to execute

### Test Connection

Use the provided test script to verify Groot connectivity:

```bash
python3 src/noham_bt/scripts/test_groot_connection.py
```

### Detailed Groot Guide

For comprehensive Groot setup and usage instructions, see:
- `GROOT_SETUP_GUIDE.md`: Complete setup and usage guide
- `config/noham_bt_groot_project.groot`: Groot project file

## Troubleshooting

### Common Issues

1. **Navigation fails**: Check if nav2 is running and map is available
2. **Action servers not responding**: Verify `noham_actions` node is running
3. **Groot connection fails**: Ensure behavior tree is running and port 1667 is accessible

### Debug Commands

```bash
# Check running nodes
ros2 node list

# Monitor behavior tree logs
ros2 topic echo /rosout

# Test Groot connection
python3 src/noham_bt/scripts/test_groot_connection.py
```

Each node properly handles success/failure states and integrates with the ROS2 action system.
