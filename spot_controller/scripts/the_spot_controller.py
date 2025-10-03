#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, QoSProfile
import torch
import numpy as np
import io
import time
class SpotController(Node):
    def __init__(self):
        super().__init__('spot_control')
        self.policy_group=MutuallyExclusiveCallbackGroup()
        self.sensors_group=MutuallyExclusiveCallbackGroup()
        self.processing_group=MutuallyExclusiveCallbackGroup()
        self.policy_timer_=self.create_timer(
            0.001, #0.08
            self.policy_callback, 
            callback_group=self.policy_group
        )

        self.joint_publisher=self.create_publisher(
            JointState,
            '/joint_command',
            10
        )

        self.processing_timer=self.create_timer(
            0.005,
            self.processing_callback,
            callback_group=self.processing_group
        )

        sim_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )

        self.joint_subscriber=self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile=sim_qos_profile,
            callback_group=self.sensors_group
        )

        self.imu_subscriber=self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile=sim_qos_profile,
            callback_group=self.sensors_group
        )

        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            qos_profile=sim_qos_profile,
            callback_group=self.policy_group
        )

        self._logger=self.get_logger()
        self.declare_parameter('policy_path', 'policy/my_spot_policy.pt')
        self.set_parameters(
            [rclpy.parameter.Parameter(
                'use_sim_time', 
                rclpy.Parameter.Type.BOOL, 
                True
            )]
        )
        self.policy_path=self.get_parameter('policy_path').value
        self.load_policy()
        self.joint_state = JointState()
        self.joint_command = JointState()
        self.cmd_vel = Twist()
        self.imu = Imu()
        self.action_scale = 0.1
        self.previous_action = np.zeros(12)
        self.action = np.zeros(12)
        self.lin_vel_b = np.zeros(3)
        self.ang_vel_b = np.zeros(3)
        self.last_tick_time = self.get_clock().now().nanoseconds * 1e-9
        self._last_tick_time = self.get_clock().now().nanoseconds * 1e-9
        self.dt = 0.0
        self.obs = np.zeros(48)
        self.gravity_b=np.zeros(3)

        self.default_pos = np.array([
            0.1,   # fl_hx   5.73 Deg
            -0.1,  # fr_hx  -5.73 Deg
            0.1,   # hl_hx   5.73 Deg
            -0.1,  # hr_hx  -5.73 Deg
            0.9,   # fl_hy  51.57 Deg
            0.9,   # fr_hy  51.57 Deg
            1.1,   # hl_hy  63.11 Deg
            1.1,   # hr_hy  63.11 Deg
            -1.5,  # fl_kn -86.11 Deg
            -1.5,  # fr_kn -86.11 Deg
            -1.5,  # hl_kn -86.11 Deg
            -1.5,  # hr_kn -86.11 Deg
        ])

        self.joint_names = [
            'fl_hx',
            'fr_hx',
            'hl_hx',
            'hr_hx',  # Hip X joints
            'fl_hy',
            'fr_hy',
            'hl_hy',
            'hr_hy',  # Hip Y joints
            'fl_kn',
            'fr_kn',
            'hl_kn',
            'hr_kn'   # Knee joints
        ]
    def cmd_callback(self, msg):
        self.cmd_vel = msg

    def joint_state_callback(self, msg):
        
        self.joint_state = msg

    def imu_callback(self, msg):
        self.imu = msg
        now = self.get_clock().now().nanoseconds * 1e-9
        self.dt = (now - self.last_tick_time)
        lin_acc=np.array([
            self.imu.linear_acceleration.x,
            self.imu.linear_acceleration.y,
            self.imu.linear_acceleration.z
        ])
        quat_I = self.imu.orientation
        quat_array = np.array([quat_I.w, quat_I.x, quat_I.y, quat_I.z])
        R_BI = self.quat_to_rot_matrix(quat_array).T
        self.ang_vel_b = np.array([
            self.imu.angular_velocity.x,
            self.imu.angular_velocity.y,
            self.imu.angular_velocity.z
        ])
        self.gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))
        self.lin_vel_b = self.lin_vel_b + lin_acc * self.dt
        self.last_tick_time = now
        
        

    def policy_callback(self):
        obs=self.obs
        action=self.compute_action(obs)
        # scale action and publish joint state and set the last action value
        self.joint_command.header.stamp = self.get_clock().now().to_msg()
        self.joint_command.name = self.joint_names
        
        # Compute final joint positions by adding scaled actions to default positions
        action_pos = self.default_pos + action * self.action_scale
        self.joint_command.position = action_pos.tolist()
        self.joint_command.velocity = np.zeros(len(self.joint_names)).tolist()
        self.joint_command.effort = np.zeros(len(self.joint_names)).tolist()
        self.joint_publisher.publish(self.joint_command)
        self.previous_action = action

    def processing_callback(self):
        self.obs=self.compute_observation()
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._last_tick_time:
            self._logger.error(
                f'{self._get_stamp_prefix()} Time jumped backwards. Resetting.'
            )
        self._dt = (now - self._last_tick_time)
        self._last_tick_time = now

    def compute_observation(self):

        self.obs[:3] = self.lin_vel_b
        self.obs[3:6] = self.ang_vel_b
        self.obs[6:9] = self.gravity_b
        cmd_vel = [
            self.cmd_vel.linear.x,
            self.cmd_vel.linear.y, 
            self.cmd_vel.angular.z
        ]
        self.obs[9:12] = np.array(cmd_vel)
        if len(self.joint_state.position) < 12 or len(self.joint_state.velocity) < 12:
            self._logger.warn('JointState data incomplete. Skipping observation.')
            return self.obs  # Return previous observation or zeroed obs

        current_joint_pos = np.array(self.joint_state.position)
        current_joint_vel = np.array(self.joint_state.velocity)
        self.obs[12:24] = current_joint_pos - self.default_pos
        self.obs[24:36] = current_joint_vel
        self.obs[36:48] = self.previous_action
        obs = self.obs
        return obs
    
    def compute_action(self, obs):
        
        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).view(1, -1).float().to('cuda')
            action = self.policy(obs_tensor).detach().view(-1).cpu().numpy()
        return action

    def load_policy(self):
        """Load the neural network policy from the specified path."""
        # Load policy from file to io.BytesIO object
        with open(self.policy_path, 'rb') as f:
            buffer = io.BytesIO(f.read())
        # Load TorchScript model from buffer
        self.policy = torch.jit.load(buffer, map_location='cuda')
        self.policy.to('cuda')
        self.policy.eval()

    
    def quat_to_rot_matrix(self, quat: np.ndarray) -> np.ndarray:

        q = np.array(quat, dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < 1e-10:
            return np.identity(3)
        q *= np.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array(
            (
                (1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]),
                (q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]),
                (q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]),
            ),
            dtype=np.float64,
        )
    
    def _get_stamp_prefix(self) -> str:
        """Create a timestamp prefix for logging with both system and ROS time.
        
        Returns:
            str: Formatted timestamp string with system and ROS time
        """
        now = time.time()
        now_ros = self.get_clock().now().nanoseconds / 1e9
        return f'[{now}][{now_ros}]'
    
    def header_time_in_seconds(self, header) -> float:
        """Convert a ROS message header timestamp to seconds.
        
        Args:
            header: ROS message header containing timestamp
            
        Returns:
            float: Time in seconds
        """
        return header.stamp.sec + header.stamp.nanosec * 1e-9

def main(args=None):
    rclpy.init(args=args)
    my_node=SpotController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(my_node)
    try:
        executor.spin()
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()