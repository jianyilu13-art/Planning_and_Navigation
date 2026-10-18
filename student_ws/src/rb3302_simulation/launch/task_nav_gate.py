#!/usr/bin/env python3
"""Start the unchanged manual Nav2 launch only after Gazebo is stable."""
import argparse
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


class NavGate(Node):
    def __init__(self, rviz):
        super().__init__('rb3302_nav_gate')
        self.rviz = rviz
        self.last_clock = None
        self.stable_since = None
        self.clock_samples = self.scan_samples = self.odom_samples = 0
        self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.create_subscription(LaserScan, '/scan', lambda _msg: self._scan(), 10)
        self.create_subscription(Odometry, '/odom', lambda _msg: self._odom(), 10)
        self.create_timer(0.5, self.check)

    def clock_cb(self, msg):
        now = (msg.clock.sec, msg.clock.nanosec)
        if self.last_clock is None:
            self.stable_since = now
        elif now > self.last_clock:
            self.clock_samples += 1
        else:
            self.get_logger().warn('Clock was not monotonic; restarting Gazebo readiness sampling.')
            self.clock_samples = self.scan_samples = self.odom_samples = 0
            self.stable_since = now
        self.last_clock = now

    def _scan(self): self.scan_samples += 1
    def _odom(self): self.odom_samples += 1

    def clock_stable_for(self, seconds):
        if self.stable_since is None or self.last_clock is None:
            return False
        return ((self.last_clock[0] + self.last_clock[1] * 1e-9) -
                (self.stable_since[0] + self.stable_since[1] * 1e-9)) >= seconds

    def check(self):
        if (self.clock_samples >= 20 and self.clock_stable_for(5.0) and
                self.scan_samples >= 2 and self.odom_samples >= 2):
            self.get_logger().info('Gazebo clock, scan, and odom are stable; starting manual Nav2 launch.')
            self.destroy_node()
            rclpy.shutdown()
            os.execvp('ros2', ['ros2', 'launch', 'rb3302_simulation', 'sim_navigation.launch.py', f'rviz:={self.rviz}'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rviz', default='true')
    args = parser.parse_args()
    rclpy.init()
    rclpy.spin(NavGate(args.rviz))


if __name__ == '__main__':
    main()
