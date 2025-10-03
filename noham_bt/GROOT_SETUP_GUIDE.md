# Groot Debugger Setup Guide

This guide explains how to set up and use Groot (BehaviorTree.CPP visual editor) to debug and visualize your behavior trees.

## What is Groot?

Groot is a visual editor and debugger for BehaviorTree.CPP that allows you to:
- **Visualize** behavior tree structure
- **Monitor** real-time execution
- **Debug** node states and transitions
- **Edit** behavior trees visually
- **Test** tree logic before deployment

## Installation

### Option 1: Download Pre-built Binary (Recommended)

1. Go to [Groot2 Releases](https://github.com/BehaviorTree/Groot2/releases)
2. Download the latest release for your platform:
   - **Linux**: `Groot2-<version>-Linux.AppImage`
   - **Windows**: `Groot2-<version>-Windows.zip`
   - **macOS**: `Groot2-<version>-macOS.dmg`

### Option 2: Build from Source

```bash
# Clone Groot2 repository
git clone https://github.com/BehaviorTree/Groot2.git
cd Groot2

# Build (requires Qt6)
mkdir build && cd build
cmake ..
make -j4
```

## Setup

### 1. Launch Groot

- **Linux**: Make the AppImage executable and run it
- **Windows**: Extract the ZIP and run the executable
- **macOS**: Open the DMG and drag to Applications

### 2. Open the Project

1. In Groot, go to `File` → `Open Project`
2. Navigate to your workspace: `~/ros2_workspaces/noham_ws/src/noham_bt/config/`
3. Select `noham_bt_groot_project.groot`

### 3. Connect to Running Behavior Tree

1. Start your behavior tree:
   ```bash
   # For forklift mission
   ros2 launch noham_bt forklift_mission.launch.py
   
   # For spot navigation
   ros2 launch noham_bt spot_bt.launch.py
   ```

2. In Groot, click the **Connect** button (🔌)
3. Enter connection details:
   - **Host**: `localhost` (or your robot's IP)
   - **Port**: `1667` (default Groot port)
   - **Tree Name**: Leave empty for auto-detection

## Features

### Real-time Monitoring

Once connected, you'll see:
- **Node Colors**: 
  - 🟢 Green = SUCCESS
  - 🔴 Red = FAILURE  
  - 🟡 Yellow = RUNNING
  - ⚪ White = IDLE

- **Execution Flow**: Arrows show the current execution path
- **Node Status**: Hover over nodes to see detailed information

### Debugging Tools

- **Step-by-step Execution**: Pause and step through the tree
- **Node Inspection**: Click nodes to see parameters and blackboard values
- **Execution History**: View the sequence of executed nodes
- **Performance Metrics**: Monitor execution time and frequency

## Usage Examples

### 1. Monitor Forklift Mission

1. Launch the forklift mission
2. Connect Groot to port 1667
3. Watch the tree execute:
   - Navigate to first position
   - Execute forklift approach
   - Lift mechanism
   - Navigate to second position
   - Navigate to final position
   - Lower lift mechanism

### 2. Debug Navigation Issues

1. If navigation fails, the node will turn red
2. Check the node parameters and blackboard values
3. Verify the target pose format and coordinates
4. Monitor the nav2 action server status

### 3. Analyze Performance

1. Use the performance view to see execution times
2. Identify bottlenecks in the tree
3. Optimize node parameters for better performance

## Troubleshooting

### Connection Issues

- **Port 1667 blocked**: Check firewall settings
- **Connection refused**: Ensure behavior tree is running
- **Wrong tree**: Verify you're connecting to the correct executable

### Visualization Issues

- **Nodes not updating**: Check if the tree is actively executing
- **Missing nodes**: Verify XML file is loaded correctly
- **Layout problems**: Use `View` → `Reset Layout`

### Performance Issues

- **Slow updates**: Reduce tick frequency in the behavior tree
- **High CPU usage**: Close unnecessary Groot windows
- **Memory leaks**: Restart Groot periodically

## Advanced Features

### Custom Node Icons

You can create custom icons for your nodes:
1. Create SVG icons in `config/icons/`
2. Reference them in your XML files
3. Groot will automatically load and display them

### Blackboard Monitoring

Monitor ROS2 parameters and blackboard values:
1. Connect to the tree
2. Open the Blackboard panel
3. View real-time parameter updates

### Tree Validation

Groot can validate your XML files:
1. Load the XML file
2. Check for syntax errors
3. Verify node connections
4. Validate parameter types

## Tips and Best Practices

1. **Keep Groot Open**: Monitor execution in real-time during development
2. **Use Descriptive Names**: Clear node names make debugging easier
3. **Monitor Logs**: Combine Groot visualization with ROS2 logs
4. **Test Incrementally**: Build and test small tree sections first
5. **Document Changes**: Keep track of tree modifications

## Support

- **Groot Documentation**: [https://github.com/BehaviorTree/Groot2](https://github.com/BehaviorTree/Groot2)
- **BehaviorTree.CPP**: [https://github.com/BehaviorTree/BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
- **ROS2 Integration**: Check the ROS2 navigation stack documentation

## Quick Start Checklist

- [ ] Install Groot2
- [ ] Open the noham_bt project file
- [ ] Launch your behavior tree
- [ ] Connect Groot to port 1667
- [ ] Start monitoring execution
- [ ] Use debugging tools as needed

Happy debugging! 🚀
