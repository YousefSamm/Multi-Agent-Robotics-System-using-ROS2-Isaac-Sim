#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time


class LifterServiceTester(Node):

    def __init__(self):
        super().__init__('lifter_service_tester')
        self.client = self.create_client(SetBool, 'lifter_service')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifter service...')
        
        self.get_logger().info('Lifter service found!')

    def send_command(self, command):
        request = SetBool.Request()
        request.data = (command == "lift")
        
        self.get_logger().info(f'Sending command: {command}')
        future = self.client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Response: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False


def main():
    rclpy.init()
    
    tester = LifterServiceTester()
    
    try:
        # Test lift command
        print("\n=== Testing LIFT command ===")
        tester.send_command("lift")
        time.sleep(2)  # Wait a bit
        
        # Test drop command
        print("\n=== Testing DROP command ===")
        tester.send_command("drop")
        time.sleep(2)  # Wait a bit
        
        # Test invalid command
        print("\n=== Testing INVALID command ===")
        tester.send_command("invalid")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
