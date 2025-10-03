# Forklift Approach Service

This service node provides autonomous forklift approach functionality using AprilTag detection and camera feedback.

## Overview

The `forklift_approach` node subscribes to AprilTag detections and camera information to automatically guide a robot towards an AprilTag while keeping it centered in the camera view. The robot moves forward until the AprilTag occupies 90% of the camera view.

## Features

- **Service-based activation**: Uses `std_srvs/SetBool` service to start/stop the approach
- **AprilTag tracking**: Subscribes to `/tag_detections` topic for AprilTag detection
- **Camera feedback**: Uses `/camera_info` to get camera dimensions
- **Proportional control**: Implements proportional control for smooth steering
- **Ackerman steering support**: Always publishes both linear.x and angular.z velocities
- **Automatic stopping**: Stops when AprilTag is no longer detected
- **Auto-shutdown**: Node automatically terminates upon completion
- **Delayed service response**: Service returns success only after task completion

## Topics

### Subscribed Topics
- `/tag_detections` (isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray)
- `/camera_info` (sensor_msgs/msg/CameraInfo)

### Published Topics
- `/cmd_vel` (geometry_msgs/msg/Twist)

### Services
- `/forklift_approach` (std_srvs/srv/SetBool)

## Usage

### 1. Build the package
```bash
cd ~/ros2_workspaces/noham_ws
colcon build --packages-select noham_bt
source install/setup.bash
```

### 2. Launch the node
```bash
ros2 launch noham_bt forklift_approach.launch.py
```

### 3. Activate the service
```bash
# Start the approach
ros2 service call /forklift_approach std_srvs/srv/SetBool "data: true"

# Stop the approach
ros2 service call /forklift_approach std_srvs/srv/SetBool "data: false"
```

### 4. Monitor the robot
```bash
# Watch cmd_vel messages
ros2 topic echo /cmd_vel

# Watch AprilTag detections
ros2 topic echo /tag_detections
```

## Parameters

The following parameters can be adjusted in the code:

- `linear_speed`: Forward velocity (default: 0.1 m/s)
- `angular_gain`: Steering sensitivity (default: 0.002)
- **Success Criteria**: Task completes when AprilTag is no longer detected

## How It Works

1. **Service Activation**: When the service is called with `data: true`, the node becomes active
2. **Camera Info**: Waits for camera information to get image dimensions
3. **Tag Detection**: Continuously monitors AprilTag detections
4. **Proportional Control**: Calculates steering based on tag position relative to image center
5. **Movement**: Publishes cmd_vel commands with both linear.x and angular.z
6. **Target Reached**: Automatically stops when AprilTag is no longer detected
7. **Auto-Shutdown**: Node automatically terminates to clean up resources
8. **Service Response**: Sends success response upon task completion

## Control Algorithm

The robot uses proportional control to keep the AprilTag centered:

```cpp
// Calculate error from center
double error_x = target_centre_x - (image_width / 2.0);

// Apply proportional control
twist.linear.x = linear_speed;
twist.angular.z = -error_x * angular_gain;
```

## Safety Features

- **Service-based control**: Can be stopped at any time via service call
- **Camera info validation**: Won't start without camera information
- **Automatic stopping**: Stops when target is reached
- **Auto-shutdown**: Automatically terminates upon completion
- **Error handling**: Gracefully handles missing detections

## Troubleshooting

### Service not responding
- Ensure the node is running: `ros2 node list`
- Check if camera info is received: `ros2 topic echo /camera_info`

### Robot not moving
- Verify AprilTag detections: `ros2 topic echo /tag_detections`
- Check cmd_vel messages: `ros2 topic echo /cmd_vel`
- Ensure the service was activated: `ros2 service call /forklift_approach std_srvs/srv/SetBool "data: true"`

### Poor tracking performance
- Adjust `angular_gain` parameter in the code
- Ensure AprilTag is clearly visible
- Check camera calibration

## Dependencies

- `rclcpp`
- `std_srvs`
- `sensor_msgs`
- `isaac_ros_apriltag_interfaces`
- `geometry_msgs`
