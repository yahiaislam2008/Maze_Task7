#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class MazeSolve(Node):

    def __init__(self):
        super().__init__('maze_solve')

        self.move_set = self.create_client(SetParameters, '/movement_node/set_parameters')
        self.move_get = self.create_client(GetParameters, '/movement_node/get_parameters')
        self.wall_set = self.create_client(SetParameters, '/wall_client/set_parameters')

        # the four-step sequence, tunable from the command line
        self.declare_parameter('step1_yaw_deg', 90.0)
        self.declare_parameter('step2_wall_state', True)
        self.declare_parameter('step2_distance', 1.0)
        self.declare_parameter('step3_wall_state', False)
        self.declare_parameter('step3_distance', 1.0)
        self.declare_parameter('step3_yaw_deg', -90.0)
        self.declare_parameter('step4_distance', 5.0)

    def wait_for_services(self):
        for client in (self.move_set, self.move_get, self.wall_set):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for a service...')

    def set_bool(self, client, name, value):
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        req = SetParameters.Request(parameters=[param])
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def set_double(self, client, name, value):
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        req = SetParameters.Request(parameters=[param])
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def is_still_busy(self, status_name):
        req = GetParameters.Request(names=[status_name])
        future = self.move_get.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().values[0].bool_value

    def wait_until_done(self, status_name):
        while self.is_still_busy(status_name):
            time.sleep(0.2)

    def run_sequence(self):
        step1_yaw = self.get_parameter('step1_yaw_deg').value
        step2_wall = self.get_parameter('step2_wall_state').value
        step2_dist = self.get_parameter('step2_distance').value
        step3_wall = self.get_parameter('step3_wall_state').value
        step3_dist = self.get_parameter('step3_distance').value
        step3_yaw = self.get_parameter('step3_yaw_deg').value
        step4_dist = self.get_parameter('step4_distance').value

        self.get_logger().info('--- Starting maze solve sequence ---')

        # 1. turn left
        self.set_double(self.move_set, 'yaw_target_deg', step1_yaw)
        self.wait_until_done('is_rotating')

        # 2. set wall state, then move
        self.set_bool(self.wall_set, 'set_wall_state', step2_wall)
        self.set_double(self.move_set, 'move_distance', step2_dist)
        self.wait_until_done('is_moving')

        # 3. continue then turn right
        self.set_bool(self.wall_set, 'set_wall_state', step3_wall)
        self.set_double(self.move_set, 'move_distance', step3_dist)
        self.wait_until_done('is_moving')

        self.set_double(self.move_set, 'yaw_target_deg', step3_yaw)
        self.wait_until_done('is_rotating')

        # 4. move forward
        self.set_double(self.move_set, 'move_distance', step4_dist)
        self.wait_until_done('is_moving')

        self.get_logger().info('--- Maze solve sequence complete ---')


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolve()
    node.wait_for_services()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()