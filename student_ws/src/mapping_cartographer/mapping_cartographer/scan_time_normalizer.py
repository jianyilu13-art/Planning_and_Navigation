#!/usr/bin/env python3
"""Republish LaserScan with non-overlapping per-beam timestamps.

The active RPLIDAR reports a scan duration longer than the observed interval
between some consecutive header stamps. Cartographer then drops the overlapping
tail as "earlier points". Holding one scan until the next header arrives lets
us cap its beam duration to the measured interval without inventing clock time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class ScanTimeNormalizer(Node):
    def __init__(self):
        super().__init__('scan_time_normalizer')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_cartographer')
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.callback, qos_profile_sensor_data)
        self.pending = None
        self.corrected = 0

    def callback(self, current):
        if self.pending is None:
            self.pending = current
            return

        previous = self.pending
        self.pending = current
        interval = stamp_seconds(current) - stamp_seconds(previous)
        count = len(previous.ranges)
        if interval <= 0.0:
            self.get_logger().error(
                'Dropping scan with duplicate/non-monotonic header stamp',
                throttle_duration_sec=2.0)
            return

        if count > 1:
            reported_duration = previous.time_increment * (count - 1)
            if reported_duration <= 0.0:
                reported_duration = previous.scan_time
            # Leave a 1% gap so floating-point rounding cannot overlap the
            # next scan's first point in Cartographer's ordered queue.
            safe_duration = min(reported_duration, interval * 0.99)
            if safe_duration < reported_duration:
                self.corrected += 1
                self.get_logger().warn(
                    f'Capped overlapping scan duration from '
                    f'{reported_duration * 1e3:.2f} to '
                    f'{safe_duration * 1e3:.2f} ms '
                    f'(corrections={self.corrected})',
                    throttle_duration_sec=5.0)
            previous.time_increment = safe_duration / (count - 1)
            previous.scan_time = safe_duration

        self.publisher.publish(previous)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimeNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # ros2 launch can deliver a second SIGINT during teardown.
            pass


if __name__ == '__main__':
    main()
