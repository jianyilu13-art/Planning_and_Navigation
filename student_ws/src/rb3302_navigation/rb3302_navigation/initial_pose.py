"""Initial-pose publisher retained as a standalone diagnostic utility."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('rb3302_initial_pose')
        self.declare_parameter('x', 0.0); self.declare_parameter('y', 0.0); self.declare_parameter('yaw', 0.0)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(1.0, self.publish_once)
        self.sent = False

    def publish_once(self):
        if self.sent: return
        yaw = self.get_parameter('yaw').value
        msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.get_parameter('x').value; msg.pose.pose.position.y = self.get_parameter('y').value
        msg.pose.pose.orientation.z = math.sin(yaw / 2); msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.09; msg.pose.covariance[35] = math.radians(45) ** 2
        self.publisher.publish(msg); self.sent = True; self.get_logger().info('Initial pose published.')


def main():
    rclpy.init(); node = InitialPosePublisher(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
