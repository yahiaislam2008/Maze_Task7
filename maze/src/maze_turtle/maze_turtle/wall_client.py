import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class WallClient(Node):

    def __init__(self):
        super().__init__('wall_client')
        
        self.declare_parameter('toggle_period', 15.0)
        period = self.get_parameter('toggle_period').value

        self.client = self.create_client(
            SetBool,
            '/toggle_walls_1_2'
        )

        self.wall_state = True

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for wall service...')

        self.send_request(self.wall_state)

        self.timer = self.create_timer(period, self.switch)

    def send_request(self , state):

        request = SetBool.Request()
        request.data = state

        future = self.client.call_async(request)
        # future.add_done_callback(self.response_callback))

    def switch(self):
        self.wall_state = not self.wall_state
        self.send_request(self.wall_state)
        # We don't need the timer anymore.
        self.timer.cancel()



def main(args=None):
    rclpy.init(args=args)

    node = WallClient()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()