import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D


class PoseResetForwarder(Node):

    def __init__(self) -> None:
        super().__init__('pose_reset_forwarder')
        self.declare_parameter('robot_id', 'p0')
        self.robot_id = self.get_parameter('robot_id').value

        pose_topic = f'/{self.robot_id}/pose2D'
        reset_topic = f'/sim/{self.robot_id}/reset_pose2D'

        self._sent = False
        self._publisher = self.create_publisher(Pose2D, reset_topic, 10)
        self.create_subscription(Pose2D, pose_topic, self.pose_callback, 10)

        self.get_logger().info(
            f'Waiting for Pose2D on {pose_topic}; forwarding first message to {reset_topic}'
        )

    def pose_callback(self, msg: Pose2D) -> None:
        if self._sent:
            return

        forward_msg = Pose2D()
        forward_msg.x = msg.x
        forward_msg.y = msg.y
        forward_msg.theta = msg.theta

        self._publisher.publish(forward_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseResetForwarder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
