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
from robot_controller.PID_controller import PID


class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        # 1. Declare Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('control_loop_period', 0.05)
        self.declare_parameter('odom_timeout_sec', 2.0)

        # Linear PID
        self.declare_parameter('kp_linear', 1.2)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.1)

        # Heading PID
        self.declare_parameter('kp_heading', 2.5)
        self.declare_parameter('ki_heading', 0.0)
        self.declare_parameter('kd_heading', 0.1)

        # Yaw PID
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.15)

        # Tolerances & Limits
        self.declare_parameter('distance_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', 0.02)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)

        # Linear PID limits
        self.declare_parameter('linear_output_min', 0.0)
        self.declare_parameter('linear_output_max', 0.5)
        self.declare_parameter('linear_integral_min', -0.5)
        self.declare_parameter('linear_integral_max', 0.5)

        # Parameter-triggered actions
        self.declare_parameter('move_distance', 0.0)
        self.declare_parameter('yaw_target_deg', 0.0)
        self.declare_parameter('is_moving', False)
        self.declare_parameter('is_rotating', False)

        # Retrieve Topics & Timing Parameters
        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).value

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.control_period = self.get_parameter(
            'control_loop_period'
        ).value

        self.odom_timeout_sec = self.get_parameter(
            'odom_timeout_sec'
        ).value

        # Publisher & Subscriber
        self.publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # State Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0

        self.target_distance = 0.0
        self.target_yaw = 0.0

        self.moving = False
        self.rotating = False

        # Odometry Tracking
        self.last_odom_time = self.get_clock().now()
        self.has_odom = False

        # Linear PID
        self.linear_pid = PID(
            kp=self.get_parameter('kp_linear').value,
            ki=self.get_parameter('ki_linear').value,
            kd=self.get_parameter('kd_linear').value,
            output_limits=(
                self.get_parameter('linear_output_min').value,
                self.get_parameter('linear_output_max').value
            ),
            integral_limits=(
                self.get_parameter('linear_integral_min').value,
                self.get_parameter('linear_integral_max').value
            ),
            deadzone=self.get_parameter('distance_tolerance').value
        )

        # Dynamic parameters
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        # Action Servers
        self._action_server = ActionServer(
            self,
            Yaw,
            'rotate_robot_yaw',
            self.execute_callback
        )

        self.get_logger().info(
            'Yaw PID Action Server started!'
        )

        self.move_x_server = ActionServer(
            self,
            MoveX,
            'move_robot_x',
            self.execute_move_x_callback
        )

        self.get_logger().info(
            'Move X Closed-Loop Action Server started!'
        )

    def odom_callback(self, msg):
        self.has_odom = True
        self.last_odom_time = self.get_clock().now()

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation

        self.current_yaw = self.euler_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def check_odom_health(self):
        if not self.has_odom:
            self.get_logger().error(
                'Edge Case Triggered: No /odom data received yet!'
            )
            return False

        time_since_last_odom = (
            self.get_clock().now() - self.last_odom_time
        ).nanoseconds / 1e9

        if time_since_last_odom > self.odom_timeout_sec:
            self.get_logger().error(
                f'Edge Case Triggered: /odom timed out! '
                f'({time_since_last_odom:.2f}s since last update)'
            )
            return False

        return True

    def parameter_callback(self, params):

        kp = self.get_parameter('kp_linear').value
        ki = self.get_parameter('ki_linear').value
        kd = self.get_parameter('kd_linear').value

        for param in params:

            # Trigger linear movement
            if param.name == 'move_distance':
                self.start_move(param.value)

            # Trigger rotation
            elif param.name == 'yaw_target_deg':
                self.start_rotate(
                    math.radians(param.value)
                )

            # New linear PID parameters
            elif param.name == 'kp_linear':
                kp = param.value

            elif param.name == 'ki_linear':
                ki = param.value

            elif param.name == 'kd_linear':
                kd = param.value

            # Older linear PID parameter names
            elif param.name == 'linear_kp':
                kp = param.value

            elif param.name == 'linear_ki':
                ki = param.value

            elif param.name == 'linear_kd':
                kd = param.value

        self.linear_pid.set_gains(kp, ki, kd)

        return SetParametersResult(
            successful=True
        )

    def start_move(self, distance):
        self.target_distance = distance

        self.start_x = self.current_x
        self.start_y = self.current_y

        self.linear_pid.reset()

        self.moving = True

        self.set_parameters([
            Parameter(
                'is_moving',
                Parameter.Type.BOOL,
                True
            )
        ])

        self.get_logger().info(
            f'Move triggered: distance={distance}'
        )

    def start_rotate(self, angle_rad):
        self.target_yaw = math.atan2(
            math.sin(self.current_yaw + angle_rad),
            math.cos(self.current_yaw + angle_rad)
        )

        self.rotating = True

        self.set_parameters([
            Parameter(
                'is_rotating',
                Parameter.Type.BOOL,
                True
            )
        ])

        self.get_logger().info(
            f'Yaw triggered: relative_angle_rad={angle_rad:.2f}'
        )

    def get_distance_moved(self):
        return math.hypot(
            self.current_x - self.start_x,
            self.current_y - self.start_y
        )

    async def execute_move_x_callback(self, goal_handle):
        self.get_logger().info(
            'Executing closed-loop Move X with heading correction...'
        )

        if not self.check_odom_health():
            goal_handle.abort()

            result = MoveX.Result()
            result.success = False

            return result

        target_dist = goal_handle.request.distance

        self.start_move(target_dist)

        start_x = self.current_x
        start_y = self.current_y
        start_heading = self.current_yaw

        dist_tolerance = self.get_parameter(
            'distance_tolerance'
        ).value

        max_lin_speed = self.get_parameter(
            'max_linear_speed'
        ).value

        dt = self.control_period

        # Heading PID state
        integral_heading = 0.0
        prev_heading_error = 0.0

        twist = Twist()

        while rclpy.ok():

            if not self.check_odom_health():
                self.publisher.publish(Twist())

                self.moving = False

                self.set_parameters([
                    Parameter(
                        'is_moving',
                        Parameter.Type.BOOL,
                        False
                    )
                ])

                goal_handle.abort()

                result = MoveX.Result()
                result.success = False

                return result

            # Fetch fresh PID parameters dynamically
            kp_h = self.get_parameter(
                'kp_heading'
            ).value

            ki_h = self.get_parameter(
                'ki_heading'
            ).value

            kd_h = self.get_parameter(
                'kd_heading'
            ).value

            # Distance
            dist_moved = math.hypot(
                self.current_x - start_x,
                self.current_y - start_y
            )

            dist_error = target_dist - dist_moved

            if abs(dist_error) < dist_tolerance:
                break

            # Linear PID
            linear_cmd = self.linear_pid.compute(
                measurement=dist_moved,
                dt=dt,
                error=dist_error
            )

            # Limit linear velocity
            linear_cmd = max(
                min(linear_cmd, max_lin_speed),
                -max_lin_speed
            )

            # Heading correction
            heading_error = (
                start_heading - self.current_yaw
            )

            heading_error = math.atan2(
                math.sin(heading_error),
                math.cos(heading_error)
            )

            integral_heading += heading_error * dt

            derivative_heading = (
                heading_error - prev_heading_error
            ) / dt

            angular_cmd = (
                kp_h * heading_error
                + ki_h * integral_heading
                + kd_h * derivative_heading
            )

            prev_heading_error = heading_error

            # Limit angular velocity
            max_ang_speed = self.get_parameter(
                'max_angular_speed'
            ).value

            angular_cmd = max(
                min(angular_cmd, max_ang_speed),
                -max_ang_speed
            )

            # Publish Twist
            twist.linear.x = linear_cmd
            twist.angular.z = angular_cmd

            self.publisher.publish(twist)

            await asyncio.sleep(dt)

        # Stop robot
        self.publisher.publish(Twist())

        self.moving = False

        self.set_parameters([
            Parameter(
                'is_moving',
                Parameter.Type.BOOL,
                False
            )
        ])

        goal_handle.succeed()

        result = MoveX.Result()
        result.success = True

        self.get_logger().info(
            'Linear move completed smoothly!'
        )

        return result

    async def execute_callback(self, goal_handle):
        self.get_logger().info(
            'Executing closed-loop Yaw PID rotation...'
        )

        if not self.check_odom_health():
            goal_handle.abort()

            result = Yaw.Result()
            result.success = False

            return result

        feedback_msg = Yaw.Feedback()
        result = Yaw.Result()

        self.start_rotate(
            goal_handle.request.target_yaw_angle
        )

        target_yaw = self.target_yaw

        yaw_tolerance = self.get_parameter(
            'yaw_tolerance'
        ).value

        max_ang_speed = self.get_parameter(
            'max_angular_speed'
        ).value

        dt = self.control_period

        integral_yaw = 0.0
        prev_yaw_error = 0.0

        twist = Twist()

        while rclpy.ok():

            if not self.check_odom_health():
                self.publisher.publish(Twist())

                self.rotating = False

                self.set_parameters([
                    Parameter(
                        'is_rotating',
                        Parameter.Type.BOOL,
                        False
                    )
                ])

                goal_handle.abort()

                result.success = False

                return result

            # Fetch fresh PID parameters dynamically
            kp_y = self.get_parameter(
                'kp_yaw'
            ).value

            ki_y = self.get_parameter(
                'ki_yaw'
            ).value

            kd_y = self.get_parameter(
                'kd_yaw'
            ).value

            # Angular error
            error = target_yaw - self.current_yaw

            error = math.atan2(
                math.sin(error),
                math.cos(error)
            )

            feedback_msg.current_yaw_angle = (
                self.current_yaw
            )

            feedback_msg.remaining_angle = error

            goal_handle.publish_feedback(
                feedback_msg
            )

            if abs(error) < yaw_tolerance:
                break

            integral_yaw += error * dt

            derivative_yaw = (
                error - prev_yaw_error
            ) / dt

            angular_cmd = (
                kp_y * error
                + ki_y * integral_yaw
                + kd_y * derivative_yaw
            )

            prev_yaw_error = error

            # Limit angular velocity
            angular_cmd = max(
                min(angular_cmd, max_ang_speed),
                -max_ang_speed
            )

            twist.linear.x = 0.0
            twist.angular.z = angular_cmd

            self.publisher.publish(twist)

            await asyncio.sleep(dt)

        # Stop robot
        self.publisher.publish(Twist())

        self.rotating = False

        self.set_parameters([
            Parameter(
                'is_rotating',
                Parameter.Type.BOOL,
                False
            )
        ])

        goal_handle.succeed()

        result.success = True
        result.final_yaw_angle = self.current_yaw

        self.get_logger().info(
            'Rotation completed smoothly!'
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    node = MovementNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()