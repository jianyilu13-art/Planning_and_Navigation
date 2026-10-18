#!/usr/bin/env python3
"""Gate task_manager on the actual manual-stack readiness conditions."""
import argparse
import os

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ReadinessGate(Node):
    def __init__(self, args):
        super().__init__('rb3302_task_readiness')
        self.args = args
        self.clock_samples = 0
        self.last_clock = None
        self.stable_since = None
        self.scan_samples = 0
        self.odom_samples = 0
        self.states = {}
        self.state_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                              for name in ('amcl', 'planner_server', 'controller_server')}
        self.pending = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.5, self.check)

    def clock_cb(self, msg):
        now = (msg.clock.sec, msg.clock.nanosec)
        if self.last_clock is None:
            self.stable_since = now
        elif now > self.last_clock:
            self.clock_samples += 1
        else:
            self.get_logger().warn('Clock was not monotonic; restarting readiness sampling.')
            self.clock_samples = self.scan_samples = self.odom_samples = 0
            self.stable_since = now
        self.last_clock = now

    def scan_cb(self, _msg): self.scan_samples += 1
    def odom_cb(self, _msg): self.odom_samples += 1

    def clock_stable_for(self, seconds):
        if self.stable_since is None or self.last_clock is None:
            return False
        return ((self.last_clock[0] + self.last_clock[1] * 1e-9) -
                (self.stable_since[0] + self.stable_since[1] * 1e-9)) >= seconds

    def check(self):
        for name, client in self.state_clients.items():
            future = self.pending.get(name)
            if future is not None and future.done():
                try:
                    self.states[name] = future.result().current_state.id
                except Exception:
                    pass
                self.pending.pop(name, None)
            if name not in self.pending and client.service_is_ready():
                self.pending[name] = client.call_async(GetState.Request())
        active = all(self.states.get(name) == State.PRIMARY_STATE_ACTIVE
                     for name in self.state_clients)
        tf_ready = self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time())
        if (self.clock_samples >= 20 and self.clock_stable_for(5.0) and
                self.scan_samples >= 2 and self.odom_samples >= 2 and active and tf_ready):
            self.get_logger().info('Manual-stack readiness confirmed; starting task_manager.')
            self.destroy_node()
            rclpy.shutdown()
            os.execvp('ros2', ['ros2', 'run', 'rb3302_navigation', 'task_manager', '--ros-args',
                               '-p', 'use_sim_time:=true', '-p', f'task_file:={self.args.task_file}',
                               '-p', f'landmarks_file:={self.args.landmarks_file}',
                               # Match the previous automatic-run acceptance margin to Nav2's
                               # goal checker; task_manager's standalone default is stricter.
                               '-p', f'goal_xy_tolerance:={self.args.goal_xy_tolerance}'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task-file', required=True)
    parser.add_argument('--landmarks-file', required=True)
    parser.add_argument('--goal-xy-tolerance', default='0.30')
    args = parser.parse_args()
    rclpy.init()
    node = ReadinessGate(args)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
