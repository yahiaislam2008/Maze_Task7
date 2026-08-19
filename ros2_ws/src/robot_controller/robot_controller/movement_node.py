#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from rclpy.action import ActionServer
from robot_controller.action import Yaw
from robot_controller.action import MoveX
import asyncio


class MovementNode(Node):

      def __init__(self):
        super().__init__('movement_node')
    
        # Creates publisher $ subscriber
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel',10
        )

        self.subscription = self.create_subscription(
           Odometry,'/odom',self.odom_callback, 10
      )
        #current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # tracking
        self.start_x = 0.0
        self.start_y = 0.0

        self.target_distance = 0.0
        self.moving = False

        self.timer = self.create_timer(  0.1, self.move_robot)

       # Create the action servers
        self._action_server = ActionServer(
            self,
            Yaw,
            'rotate_robot_yaw',
            self.execute_callback
        )
        self.get_logger().info('Yaw Action Server started!')

        self.move_x_server = ActionServer(
           self,
           MoveX,
           'move_robot_x',
           self.execute_move_x_callback
        )
        self.get_logger().info('Move X Action Server started!')


      def odom_callback(self, msg):
          # Robot position on x and y and angle
           self.current_x = msg.pose.pose.position.x
           self.current_y = msg.pose.pose.position.y
           orientation = msg.pose.pose.orientation

           self.current_yaw = self.euler_from_quaternion(
           orientation.x,
           orientation.y,
           orientation.z,
           orientation.w )
  


      # calculates distance 
      def get_distance_moved(self):

       distance = math.sqrt(
        (self.current_x - self.start_x)**2 +
        (self.current_y - self.start_y)**2
          )

       return distance
      
      def euler_from_quaternion(self, x, y, z, w):
        # Convert 3D rotation into a simple 2D angle
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)



      def move_robot(self):

         if not self.moving:
          return
         distance = self.get_distance_moved()
         twist = Twist()

        # move forward
         if distance < self.target_distance:

            twist.linear.x = 2.0

        #stop once target reached
         else:
       
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            self.moving=False
            self.get_logger().info(
                "Movement Done !" )


        #publish velocity
         self.publisher.publish(twist)


      async def execute_move_x_callback(self, goal_handle):

         self.get_logger().info("Moving X...")

         self.target_distance = goal_handle.request.distance

         self.start_x = self.current_x
         self.start_y = self.current_y

         self.moving = True
  
         while self.moving:
           await asyncio.sleep(0.1)
 
         goal_handle.succeed()

         result = MoveX.Result()
         result.success = True

         return result



      async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing rotation goal...')
        
        feedback_msg = Yaw.Feedback()
        result = Yaw.Result()
        
        # Calculate target angle
        start_yaw = self.current_yaw
        target_yaw = start_yaw + goal_handle.request.target_yaw_angle
        
        twist = Twist()
        

        while rclpy.ok():
            # Find how much angle is left to turn
            error = target_yaw - self.current_yaw
            
            # Send feedback
            feedback_msg.current_yaw_angle = self.current_yaw
            feedback_msg.remaining_angle = error
            goal_handle.publish_feedback(feedback_msg)

            # Stop when close enough to target
            if abs(error) < 0.05:
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                break

            # Turn speed based on distance left
            twist.angular.z = 1.0 if error > 0 else -1.0
            self.publisher.publish(twist)

            await asyncio.sleep(0.1)

        # Finish action successfully
        goal_handle.succeed()
        result.success = True
        result.final_yaw_angle = self.current_yaw
        self.get_logger().info('Rotation complete!')
        return result



    




def main(args=None):
    rclpy.init(args=args)

    node = MovementNode()

    # Keeps the node running
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
