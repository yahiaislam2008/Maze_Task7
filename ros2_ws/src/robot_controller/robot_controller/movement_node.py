#!/usr/bin/env python3

import asyncio
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from robot_controller.action import MoveX, Yaw

class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        # 1. Declare Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('distance_tolerance', 0.02)
        self.declare_parameter('control_loop_period', 0.05)
        self.declare_parameter('odom_timeout_sec', 2.0)
        self.declare_parameter('move_distance', 0.0)
        self.declare_parameter('yaw_target_deg', 0.0)
        self.declare_parameter('is_moving', False)
        self.declare_parameter('is_rotating', False)

        # Retrieve Topic Config
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        # Publishers & Subscribers
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # State Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.target_distance = 0.0
        self.moving = False

        self.target_yaw = 0.0
        self.rotating = False

        # Odometry Tracking Variables
        self.last_odom_time = self.get_clock().now()
        self.has_odom = False

        # Control Loops
        control_period = self.get_parameter('control_loop_period').value
        self.move_timer = self.create_timer(control_period, self.move_robot)
        self.yaw_timer = self.create_timer(control_period, self.rotate_robot)

        # Parameter Callback for CLI / External Param Sets
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Action Servers
        self._action_server = ActionServer(
            self, Yaw, 'rotate_robot_yaw', self.execute_callback
        )
        self.get_logger().info('Yaw Action Server started!')

        self.move_x_server = ActionServer(
            self, MoveX, 'move_robot_x', self.execute_move_x_callback
        )
        self.get_logger().info('Move X Action Server started!')

    def odom_callback(self, msg):
        self.has_odom = True
        self.last_odom_time = self.get_clock().now()

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

    def check_odom_health(self):
        """Edge Case Handling: Verifies if /odom messages are actively arriving."""
        if not self.has_odom:
            self.get_logger().error('Edge Case Triggered: No /odom data received yet!')
            return False

        timeout = self.get_parameter('odom_timeout_sec').value
        time_since_last_odom = (
            self.get_clock().now() - self.last_odom_time
        ).nanoseconds / 1e9

        if time_since_last_odom > timeout:
            self.get_logger().error(
                f'Edge Case Triggered: /odom timed out! ({time_since_last_odom:.2f}s since last update)'
            )
            return False
        return True

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'move_distance':
                self.start_move(param.value)
            elif param.name == 'yaw_target_deg':
                self.start_rotate(math.radians(param.value))
        return SetParametersResult(successful=True)

    def start_move(self, distance):
        self.target_distance = distance
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.moving = True
        self.set_parameters([Parameter('is_moving', Parameter.Type.BOOL, True)])
        self.get_logger().info(f'Move triggered: distance={distance}')

    def start_rotate(self, angle_rad):
        # Angle normalization to handle [-pi, pi] wrap around
        self.target_yaw = math.atan2(
            math.sin(self.current_yaw + angle_rad),
            math.cos(self.current_yaw + angle_rad)
        )
        self.rotating = True
        self.set_parameters([Parameter('is_rotating', Parameter.Type.BOOL, True)])
        self.get_logger().info(f'Yaw triggered: relative_angle_rad={angle_rad:.2f}')

    def get_distance_moved(self):
        return math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )

    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def move_robot(self):
        if not self.moving:
            return

        twist = Twist()

        if not self.check_odom_health():
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            self.moving = False
            self.set_parameters([Parameter('is_moving', Parameter.Type.BOOL, False)])
            return

        distance = self.get_distance_moved()
        speed = self.get_parameter('linear_speed').value
        tolerance = self.get_parameter('distance_tolerance').value

        if (self.target_distance - distance) > tolerance:
            twist.linear.x = speed
        else:
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            self.moving = False
            self.set_parameters([Parameter('is_moving', Parameter.Type.BOOL, False)])
            self.get_logger().info('Movement Done!')
            return

        self.publisher.publish(twist)

    def rotate_robot(self):
        if not self.rotating:
            return

        twist = Twist()

        if not self.check_odom_health():
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.rotating = False
            self.set_parameters([Parameter('is_rotating', Parameter.Type.BOOL, False)])
            return

        # Normalized error calculation [-pi, pi]
        error = math.atan2(
            math.sin(self.target_yaw - self.current_yaw),
            math.cos(self.target_yaw - self.current_yaw)
        )
        speed = self.get_parameter('angular_speed').value
        tolerance = self.get_parameter('yaw_tolerance').value

        if abs(error) < tolerance:
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.rotating = False
            self.set_parameters([Parameter('is_rotating', Parameter.Type.BOOL, False)])
            self.get_logger().info('Rotation Done!')
            return

        twist.angular.z = speed if error > 0 else -speed
        self.publisher.publish(twist)

    async def execute_move_x_callback(self, goal_handle):
        self.get_logger().info("Moving X...")

        if not self.check_odom_health():
            goal_handle.abort()
            result = MoveX.Result()
            result.success = False
            return result

        self.start_move(goal_handle.request.distance)
        control_period = self.get_parameter('control_loop_period').value

        while self.moving:
            if not self.check_odom_health():
                self.moving = False
                goal_handle.abort()
                result = MoveX.Result()
                result.success = False
                return result
            await asyncio.sleep(control_period)

        goal_handle.succeed()
        result = MoveX.Result()
        result.success = True
        return result

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing rotation goal...')

        if not self.check_odom_health():
            goal_handle.abort()
            result = Yaw.Result()
            result.success = False
            return result

        feedback_msg = Yaw.Feedback()
        result = Yaw.Result()

        self.start_rotate(goal_handle.request.target_yaw_angle)
        control_period = self.get_parameter('control_loop_period').value

        while self.rotating:
            if not self.check_odom_health():
                self.rotating = False
                goal_handle.abort()
                result.success = False
                return result

            error = math.atan2(
                math.sin(self.target_yaw - self.current_yaw),
                math.cos(self.target_yaw - self.current_yaw)
            )
            feedback_msg.current_yaw_angle = self.current_yaw
            feedback_msg.remaining_angle = error
            goal_handle.publish_feedback(feedback_msg)

            await asyncio.sleep(control_period)

        goal_handle.succeed()
        result.success = True
        result.final_yaw_angle = self.current_yaw
        self.get_logger().info('Rotation complete!')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()