#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64


class WallRetractionService(Node):

    def __init__(self):
        super().__init__('wall_retraction_service')

        self.declare_parameter('up_position', 2.0)      
        self.declare_parameter('down_position', 0.0)    
        self.declare_parameter('publish_repeats', 10)
        self.declare_parameter('publish_period', 0.1)

        self._up = self.get_parameter('up_position').value
        self._down = self.get_parameter('down_position').value
        self._repeat_total = self.get_parameter('publish_repeats').value
        self._period = self.get_parameter('publish_period').value

        self._pub_wall_1 = self.create_publisher(Float64, '/wall_1/cmd_pos', 10)
        self._pub_wall_2 = self.create_publisher(Float64, '/wall_2/cmd_pos', 10)

        self._burst_12 = self._new_burst_state()

        self.srv_toggle = self.create_service(
            SetBool, 'toggle_walls_1_2', self._handle_toggle_walls_1_2)

        self.get_logger().info("Service ready: 'toggle_walls_1_2' (SetBool).")

    @staticmethod
    def _new_burst_state():
        return {'targets': [], 'repeats_left': 0, 'timer': None}

    def _start_burst(self, state, targets):
        """targets: list of (publisher, value) pairs sent every tick."""
        state['targets'] = targets
        state['repeats_left'] = self._repeat_total
        if state['timer'] is not None:
            state['timer'].cancel()
        state['timer'] = self.create_timer(self._period, lambda: self._tick(state))

    def _tick(self, state):
        for pub, value in state['targets']:
            msg = Float64()
            msg.data = float(value)
            pub.publish(msg)

        state['repeats_left'] -= 1
        if state['repeats_left'] <= 0 and state['timer'] is not None:
            state['timer'].cancel()
            state['timer'] = None

    def _handle_toggle_walls_1_2(self, request, response):
        if request.data:
            targets = [(self._pub_wall_1, self._up), (self._pub_wall_2, self._down)]
            desc = f'wall 1 -> up ({self._up:.2f} m), wall 2 -> down ({self._down:.2f} m)'
        else:
            targets = [(self._pub_wall_1, self._down), (self._pub_wall_2, self._up)]
            desc = f'wall 1 -> down ({self._down:.2f} m), wall 2 -> up ({self._up:.2f} m)'

        self._start_burst(self._burst_12, targets)
        response.success = True
        response.message = f'Commanded: {desc}.'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WallRetractionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()