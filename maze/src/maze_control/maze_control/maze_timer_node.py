#!/usr/bin/env python3


import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MazeTimerNode(Node):

    # Bridged std_msgs/Bool topics, one per TouchPlugin instance in the world.
    FINISH_TOPICS = [
        '/finish_robot/touched',
        '/finish_burger/touched',
    ]

    def __init__(self):
        super().__init__('maze_timer_node')

        self.declare_parameter('status_period', 0.5)  # live "elapsed" tick rate

        self._start_time = time.monotonic()
        self._finished = False

        self._status_pub = self.create_publisher(String, '/maze_timer/status', 10)

        self._finish_subs = [
            self.create_subscription(Bool, topic, self._on_touch, 10)
            for topic in self.FINISH_TOPICS
        ]

        status_period = self.get_parameter('status_period').value
        self._status_timer = self.create_timer(status_period, self._publish_ticking_status)

        self.get_logger().info(
            f"maze_timer_node watching {self.FINISH_TOPICS} for finish-line contact."
        )

    def _elapsed(self) -> float:
        return time.monotonic() - self._start_time

    def _publish_ticking_status(self):
        if self._finished:
            return
        self._status_pub.publish(String(data=f'Time: {self._elapsed():.1f} s'))

    def _on_touch(self, msg: Bool):
        if self._finished or not msg.data:
            return
        self._on_finish()

    def _on_finish(self):
        self._finished = True
        self._status_timer.cancel()
        elapsed = self._elapsed()

        self.get_logger().info(
            f"\033[92mCongratulations! Finished in {elapsed:.2f} s\033[0m"
        )
        self._status_pub.publish(String(data=f'FINISHED! {elapsed:.2f} s'))

        # Give the final status message a moment to actually get bridged
        # and shown before tearing everything down.
        self.create_timer(1.0, self._shutdown_once)

    def _shutdown_once(self):
        self.get_logger().info('Shutting down maze simulation.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MazeTimerNode()
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