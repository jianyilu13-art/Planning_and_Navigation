#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist


HELP_TEXT = """
WAVE ROVER Keyboard Teleop
w/s: forward/backward    a/d: rotate left/right
i/k: linear speed +/-    o/l: angular speed +/-
space: stop              q: quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("waverover_keyboard_teleop")

        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("key_timeout", 0.5)
        self.declare_parameter("linear_speed", 0.4)
        self.declare_parameter("angular_speed", 1.5)
        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("angular_step", 0.1)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("min_angular_speed", 0.1)
        self.declare_parameter("max_angular_speed", 3.0)

        self.publish_rate = self.get_parameter("publish_rate").value
        self.key_timeout = self.get_parameter("key_timeout").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.linear_step = self.get_parameter("linear_step").value
        self.angular_step = self.get_parameter("angular_step").value
        self.min_linear_speed = self.get_parameter("min_linear_speed").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.min_angular_speed = self.get_parameter("min_angular_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value

        self.active_key = None
        self.last_motion_key_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Save terminal state so it can always be restored.
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        self.terminal_restored = False

        tty.setcbreak(sys.stdin.fileno())

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self._timer_callback,
        )

        print(HELP_TEXT)
        self._print_speed()

    def _get_key_nonblocking(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)

        if readable:
            return sys.stdin.read(1)

        return None

    def _timer_callback(self):
        key = self._get_key_nonblocking()

        if key is not None:
            self._handle_key(key)

        # q may have shut down the ROS context.
        if not rclpy.ok():
            return

        age = (
            self.get_clock().now() - self.last_motion_key_time
        ).nanoseconds / 1e9

        if age > self.key_timeout:
            self.active_key = None

        self.cmd_pub.publish(
            self._make_twist_from_key(self.active_key)
        )

    def _handle_key(self, key):
        if key == "q":
            self._quit()
            return

        if key == " ":
            self.active_key = None
            self._publish_stop()
            return

        if key == "i":
            self.linear_speed = min(
                self.max_linear_speed,
                self.linear_speed + self.linear_step,
            )
            self._print_speed()
            return

        if key == "k":
            self.linear_speed = max(
                self.min_linear_speed,
                self.linear_speed - self.linear_step,
            )
            self._print_speed()
            return

        if key == "o":
            self.angular_speed = min(
                self.max_angular_speed,
                self.angular_speed + self.angular_step,
            )
            self._print_speed()
            return

        if key == "l":
            self.angular_speed = max(
                self.min_angular_speed,
                self.angular_speed - self.angular_step,
            )
            self._print_speed()
            return

        if key in ("w", "s", "a", "d"):
            self.active_key = key
            self.last_motion_key_time = self.get_clock().now()

    def _quit(self):
        """Stop the rover and shut ROS down immediately."""
        print("\nExiting WAVE ROVER teleop...")

        self.active_key = None

        # Send zero velocity before shutting down.
        self._publish_stop()

        # Stop generating commands.
        if self.timer is not None:
            self.timer.cancel()

        # This causes rclpy.spin() to return immediately.
        if rclpy.ok():
            rclpy.shutdown()

    def _make_twist_from_key(self, key):
        msg = Twist()

        if key == "w":
            msg.linear.x = self.linear_speed
        elif key == "s":
            msg.linear.x = -self.linear_speed
        elif key == "a":
            msg.angular.z = self.angular_speed
        elif key == "d":
            msg.angular.z = -self.angular_speed

        return msg

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _print_speed(self):
        print(
            f"Linear: {self.linear_speed:.2f} m/s | "
            f"Angular: {self.angular_speed:.2f} rad/s"
        )

    def _restore_terminal(self):
        if self.terminal_restored:
            return

        try:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.old_terminal_settings,
            )
        except (termios.error, AttributeError):
            pass
        finally:
            self.terminal_restored = True

    def destroy_node(self):
        # Make sure the rover receives a final stop command.
        if rclpy.ok():
            self._publish_stop()

        self._restore_terminal()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = KeyboardTeleop()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nCtrl+C received. Exiting...")

    except ExternalShutdownException:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        # q already calls shutdown(), so don't call it twice.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

