#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclcpp.action import ActionClient
import time


class TestBehaviorTreeNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_bt_nodes')
        
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def test_cmd_vel_publisher(self):
        """Test that cmd_vel messages can be published"""
        pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create a simple rotation command
        twist = Twist()
        twist.angular.z = 0.5
        
        # Publish and wait
        pub.publish(twist)
        time.sleep(0.1)
        
        # If we get here without errors, the test passes
        self.assertTrue(True)
    
    def test_nav2_action_client(self):
        """Test that Nav2 action client can be created"""
        try:
            client = ActionClient(
                self.node, NavigateToPose, 'navigate_to_pose')
            # If we get here without errors, the test passes
            self.assertTrue(True)
            del client  # Clean up
        except Exception as e:
            self.fail(f"Failed to create Nav2 action client: {e}")


if __name__ == '__main__':
    unittest.main()
