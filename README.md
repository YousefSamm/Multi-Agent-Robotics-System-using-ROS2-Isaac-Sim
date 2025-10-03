# Multi-Agent Robotic System using ROS2 & Isaac Sim

This repository contains a comprehensive multi-agent robotic system implementation using ROS2 and NVIDIA Isaac Sim. The system includes various robotic agents including mobile manipulators, forklifts, and Spot robots, with integrated behavior trees, path planning, and simulation capabilities.

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- NVIDIA Isaac Sim 4.5+
- CUDA-compatible GPU
- Python 3.8+

### Installation

1. **Clone the repository with submodules:**
   ```bash
   git clone --recurse-submodules https://github.com/YousefSamm/Multi-Agent-Robotis-System-using-ROS2-Isaac-Sim.git
   cd Multi-Agent-Robotis-System-using-ROS2-Isaac-Sim
   ```

2. **If you already cloned without submodules, initialize them:**
   ```bash
   git submodule update --init --recursive
   ```

3. **Build the workspace:**
   ```bash
   cd src
   colcon build --symlink-install
   source install/setup.bash
   ```

## 📁 Project Structure

### Core Components

- **`noham_bt/`** - Behavior tree implementations for robotic agents
- **`noham_path_planner/`** - Navigation and path planning modules
- **`noham_custom_interfaces/`** - Custom ROS2 message and action definitions
- **`noham_localization/`** - Localization and mapping components
- **`noham_map_server/`** - Map server for navigation
- **`noham_apriltags/`** - AprilTag detection and pose estimation
- **`cmdvel_to_ackermann/`** - Velocity command conversion utilities
- **`spot_controller/`** - Boston Dynamics Spot robot controller
- **`isaacsim/`** - Isaac Sim simulation configurations and scripts

### Dependencies (Git Submodules)

- **`apriltag_ros/`** - AprilTag ROS package ([christianrauch/apriltag_ros](https://github.com/christianrauch/apriltag_ros))
- **`isaac_ros_nitros/`** - NVIDIA Isaac ROS NITROS bridge ([NVIDIA-ISAAC-ROS/isaac_ros_nitros](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros))
- **`isaac_ros_apriltag/`** - Isaac ROS AprilTag detection ([NVIDIA-ISAAC-ROS/isaac_ros_apriltag](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag))
- **`isaac_ros_common/`** - Isaac ROS common utilities ([NVIDIA-ISAAC-ROS/isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common))
- **`magic_enum/`** - C++ enum utilities ([Neargye/magic_enum](https://github.com/Neargye/magic_enum))

## 🤖 Supported Robots

### Mobile Manipulator
- **Robot**: Husky + Jaco arm
- **Capabilities**: Navigation, manipulation, object detection
- **Launch**: `ros2 launch isaacsim run_noham_mobile_manipulator.launch.py`

### Forklift
- **Robot**: Custom forklift model
- **Capabilities**: Autonomous navigation, pallet manipulation
- **Launch**: `ros2 launch isaacsim run_noham_forklift.launch.py`

### Spot Robot
- **Robot**: Boston Dynamics Spot
- **Capabilities**: Quadrupedal locomotion, inspection
- **Launch**: `ros2 launch isaacsim run_noham_spot.launch.py`

## 🎮 Usage

### Running Simulations

1. **Start Isaac Sim:**
   ```bash
   ros2 launch isaacsim run_isaacsim.launch.py
   ```

2. **Launch specific robot:**
   ```bash
   # Mobile manipulator
   ros2 launch isaacsim run_noham_mobile_manipulator.launch.py
   
   # Forklift
   ros2 launch isaacsim run_noham_forklift.launch.py
   
   # Spot robot
   ros2 launch isaacsim run_noham_spot.launch.py
   ```

### Behavior Tree Operations

1. **Run behavior trees:**
   ```bash
   # Mobile manipulator behavior tree
   ros2 launch noham_bt mobile_manipulator_bt.launch.py
   
   # Forklift mission
   ros2 launch noham_bt forklift_mission.launch.py
   
   # Spot behavior tree
   ros2 launch noham_bt spot_bt.launch.py
   ```

### Navigation and Path Planning

1. **Launch navigation stack:**
   ```bash
   # Mobile manipulator navigation
   ros2 launch noham_path_planner mobile_manipulator_path_planner.launch.py
   
   # Forklift navigation
   ros2 launch noham_path_planner forklift_path_planner.launch.py
   ```

## 🛠️ Development

### Adding New Robots

1. Create robot-specific packages in the workspace
2. Add launch files for simulation and control
3. Update behavior trees if needed
4. Configure path planning parameters

### Updating Dependencies

To update submodules to their latest versions:
```bash
git submodule update --remote
git add .
git commit -m "Updated submodules to latest versions"
```

### Building Individual Packages

```bash
# Build specific package
colcon build --packages-select <package_name>

# Build with specific dependencies
colcon build --packages-up-to <package_name>
```

## 📋 Configuration

### Isaac Sim Settings

- Simulation configurations are located in `isaacsim/config/`
- USD files for different robot models are in respective directories
- Policy files for reinforcement learning are in `isaacsim/policy/`

### Navigation Parameters

- Configuration files are in `noham_path_planner/config/`
- Behavior tree configurations are in `noham_bt/config/`

## 🐛 Troubleshooting

### Common Issues

1. **Submodule initialization failed:**
   ```bash
   git submodule update --init --recursive --force
   ```

2. **Build errors:**
   ```bash
   # Clean and rebuild
   rm -rf build install log
   colcon build --symlink-install
   ```

3. **Isaac Sim connection issues:**
   - Ensure Isaac Sim is running
   - Check ROS2 environment variables
   - Verify network connectivity

### Getting Help

- Check individual package README files for specific documentation
- Review launch file parameters for customization options
- Examine log files in the `log/` directory after building

## 📄 License

This project is licensed under the MIT License - see the individual package licenses for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📞 Support

For issues and questions:
- Create an issue in this repository
- Check the documentation in individual packages
- Review the troubleshooting section above

---

**Note**: This repository uses Git submodules to manage external dependencies. Always clone with `--recurse-submodules` or initialize submodules after cloning to ensure all dependencies are available.
