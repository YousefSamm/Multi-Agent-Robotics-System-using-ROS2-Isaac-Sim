# Lifter Service Node

This node provides a service interface to control the lift joint of the robot. It accepts "lift" and "drop" commands and automatically moves the joint to the specified positions.

## Features

- **Lift Command**: Moves the lift joint to position 0.2
- **Drop Command**: Moves the lift joint to position 0.0
- **Position Monitoring**: Automatically tracks joint state to verify position completion
- **Tolerance Control**: Uses 0.05 tolerance for position verification
- **Continuous Operation**: Always ready for new commands

## Topics and Services

### Subscribed Topics
- `/joint_states` (sensor_msgs/JointState): Joint state feedback for position monitoring

### Published Topics
- `/joint_command` (trajectory_msgs/JointTrajectory): Joint trajectory commands for the lift joint

### Services
- `/lifter_service` (std_srvs/Trigger): Service interface for lift/drop commands
  - Request: `message` field containing "lift" or "drop"
  - Response: `success` and `message` indicating operation status

## Building

1. Navigate to your ROS2 workspace:
   ```bash
   cd ~/ros2_workspaces/noham_ws
   ```

2. Build the package:
   ```bash
   colcon build --packages-select noham_bt
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Launch the Service Node

```bash
ros2 launch noham_bt lifter_service.launch.py
```

### Using the Service

#### Command Line Interface

**Lift Command:**
```bash
ros2 service call /lifter_service std_srvs/srv/Trigger "{message: 'lift'}"
```

**Drop Command:**
```bash
ros2 service call /lifter_service std_srvs/srv/Trigger "{message: 'drop'}"
```

#### Python Example

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# Create a client
client = node.create_client(Trigger, 'lifter_service')

# Send lift command
request = Trigger.Request()
request.message = "lift"
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
```

### Testing

A test script is provided to verify the service functionality:

```bash
ros2 run noham_bt test_lifter_service.py
```

## Configuration

The node can be configured with the following parameters:

- `use_sim_time` (default: false): Use simulation time if true

## Dependencies

- `rclcpp`: ROS2 C++ client library
- `std_srvs`: Standard service types
- `sensor_msgs`: Sensor message types
- `trajectory_msgs`: Trajectory message types

## Troubleshooting

1. **Service not found**: Ensure the node is running and check the service name
2. **Joint not moving**: Verify that the `/joint_command` topic is being subscribed to by the robot controller
3. **Position not reached**: Check that `/joint_states` topic is publishing the lift_joint data

## Implementation Details

The node implements a state machine that:
1. Receives service requests
2. Publishes joint trajectory commands
3. Monitors joint state feedback
4. Tracks position completion within tolerance
5. Logs operation status

The service is designed to be non-blocking and always ready for new commands.
