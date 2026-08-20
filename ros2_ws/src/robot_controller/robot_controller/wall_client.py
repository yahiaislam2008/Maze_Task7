#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult   
from std_srvs.srv import SetBool


class WallClient(Node):

    def __init__(self):
        super().__init__('wall_client')

        self.declare_parameter('toggle_period', 10.0)
        period = self.get_parameter('toggle_period').value

        self.client = self.create_client(SetBool, '/toggle_walls_1_2')
        self.wall_state = True

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for wall service...')

        self.send_request(self.wall_state)
        self.timer = self.create_timer(period, self.switch)

        self.declare_parameter('set_wall_state', True)        
        self.add_on_set_parameters_callback(self.parameter_callback)  
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'set_wall_state':
                self.get_logger().info(f'set_wall_state changed to {param.value}')
                self.wall_state = param.value
                self.send_request(param.value)
        return SetParametersResult(successful=True)

    def send_request(self, state):
        request = SetBool.Request()
        request.data = state
        future = self.client.call_async(request)

    def switch(self):
        self.wall_state = not self.wall_state
        self.send_request(self.wall_state)
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = WallClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()