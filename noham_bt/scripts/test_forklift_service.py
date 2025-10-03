#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class ForkliftServiceTester(Node):

    def __init__(self):
        super().__init__('forklift_service_tester')
        self.client = self.create_client(SetBool, 'forklift_approach')
        self.get_logger().info('Forklift service tester initialized')

    def test_service(self):
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            
        # Test starting the service
        self.get_logger().info('Testing service start...')
        request = SetBool.Request()
        request.data = True
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Service response: {response.success} - {response.message}'
            )
        else:
            self.get_logger().error('Service call failed')

        # Wait a bit then stop
        time.sleep(5)

        # Test stopping the service
        self.get_logger().info('Testing service stop...')
        request.data = False
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Service response: {response.success} - {response.message}'
            )
        else:
            self.get_logger().error('Service call failed')


def main():
    rclpy.init()
    tester = ForkliftServiceTester()
    
    try:
        tester.test_service()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
