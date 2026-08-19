import asyncio
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from robot_controller.action import MoveX, Yaw


class MovementNode(Node):

  def __init__(self):
    super().__init__('movement_node')

    # 1. Declare Parameters 
    self.declare_parameter('cmd_vel_topic', '/cmd_vel')
    self.declare_parameter('odom_topic', '/odom')
    self.declare_parameter('linear_speed', 0.2)  
    self.declare_parameter('angular_speed', 0.5)  
    self.declare_parameter('yaw_tolerance', 0.05)  
    self.declare_parameter('distance_tolerance', 0.02)  
    self.declare_parameter('control_loop_period', 0.05)  
    self.declare_parameter('odom_timeout_sec', 2.0)  

    # 2. Retrieve Parameter Values
    self.cmd_vel_topic = (
        self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
    )
    self.odom_topic = (
        self.get_parameter('odom_topic').get_parameter_value().string_value
    )
    self.linear_speed = (
        self.get_parameter('linear_speed').get_parameter_value().double_value
    )
    self.angular_speed = (
        self.get_parameter('angular_speed').get_parameter_value().double_value
    )
    self.yaw_tolerance = (
        self.get_parameter('yaw_tolerance').get_parameter_value().double_value
    )
    self.distance_tolerance = (
        self.get_parameter('distance_tolerance')
        .get_parameter_value()
        .double_value
    )
    self.control_period = (
        self.get_parameter('control_loop_period')
        .get_parameter_value()
        .double_value
    )
    self.odom_timeout_sec = (
        self.get_parameter('odom_timeout_sec')
        .get_parameter_value()
        .double_value
    )

    # Publishers & Subscribers using configured topics
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

    # Timestamp tracking for Edge Case: Missing Odom / Timeout
    self.last_odom_time = self.get_clock().now()
    self.has_odom = False

    # Control Timer
    self.timer = self.create_timer(self.control_period, self.move_robot)

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

  def get_distance_moved(self):
    return math.sqrt(
        (self.current_x - self.start_x) ** 2
        + (self.current_y - self.start_y) ** 2
    )

  def euler_from_quaternion(self, x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

  def check_odom_health(self):
    """Edge Case Handling: Verifies if /odom messages are actively arriving."""
    if not self.has_odom:
      self.get_logger().error('Edge Case Triggered: No /odom data received yet!')
      return False

    time_since_last_odom = (
        self.get_clock().now() - self.last_odom_time
    ).nanoseconds / 1e9
    if time_since_last_odom > self.odom_timeout_sec:
      self.get_logger().error(
          f'Edge Case Triggered: /odom timed out! ({time_since_last_odom:.2f}s'
          ' since last update)'
      )
      return False
    return True

  def move_robot(self):
    if not self.moving:
      return

    twist = Twist()

    # Safety check for missing odom feedback
    if not self.check_odom_health():
      twist.linear.x = 0.0
      self.publisher.publish(twist)
      self.moving = False
      return

    distance = self.get_distance_moved()
    remaining = self.target_distance - distance

    if remaining > self.distance_tolerance:
      twist.linear.x = self.linear_speed
    else:
      twist.linear.x = 0.0
      self.publisher.publish(twist)
      self.moving = False
      self.get_logger().info('Movement Done!')

    self.publisher.publish(twist)

  async def execute_move_x_callback(self, goal_handle):
    self.get_logger().info('Moving X...')

    # Safety check before executing
    if not self.check_odom_health():
      goal_handle.abort()
      result = MoveX.Result()
      result.success = False
      return result

    self.target_distance = goal_handle.request.distance
    self.start_x = self.current_x
    self.start_y = self.current_y
    self.moving = True

    while self.moving:
      if not self.check_odom_health():
        self.moving = False
        goal_handle.abort()
        result = MoveX.Result()
        result.success = False
        return result
      await asyncio.sleep(self.control_period)

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

    start_yaw = self.current_yaw
    target_yaw = start_yaw + goal_handle.request.target_yaw_angle
    twist = Twist()

    while rclpy.ok():
      if not self.check_odom_health():
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        goal_handle.abort()
        result.success = False
        return result

      # Normalize angle difference to [-pi, pi]
      error = math.atan2(
          math.sin(target_yaw - self.current_yaw),
          math.cos(target_yaw - self.current_yaw),
      )

      feedback_msg.current_yaw_angle = self.current_yaw
      feedback_msg.remaining_angle = error
      goal_handle.publish_feedback(feedback_msg)

      if abs(error) < self.yaw_tolerance:
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        break

      twist.angular.z = (
          self.angular_speed if error > 0 else -self.angular_speed
      )
      self.publisher.publish(twist)

      await asyncio.sleep(self.control_period)

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