#!/usr/bin/env python3

import json
import math
import threading
import time
import subprocess
import atexit

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Float32, Float32MultiArray, Int32MultiArray
from tf2_ros import TransformBroadcaster

import serial


class DisplayServiceGuard:
    def __init__(self, service_name: str, logger):
        self.service_name = service_name
        self.logger = logger
        self.stopped_by_us = False

    def stop(self):
        try:
            result = subprocess.run(
                ["sudo", "/bin/systemctl", "stop", self.service_name],
                capture_output=True,
                text=True,
                check=False,
                timeout=5.0,
            )
            if result.returncode == 0:
                self.stopped_by_us = True
                self.logger.info(f"Stopped {self.service_name}")
            else:
                self.logger.warning(f"Could not stop {self.service_name}: {result.stderr.strip()}")
        except Exception as exc:
            self.logger.warning(f"Failed to stop {self.service_name}: {exc}")

    def start(self):
        if not self.stopped_by_us:
            return
        try:
            result = subprocess.run(
                ["sudo", "/bin/systemctl", "start", self.service_name],
                capture_output=True,
                text=True,
                check=False,
                timeout=5.0,
            )
            if result.returncode == 0:
                self.logger.info(f"Restarted {self.service_name}")
                self.stopped_by_us = False
            else:
                self.logger.warning(f"Could not restart {self.service_name}: {result.stderr.strip()}")
        except Exception as exc:
            self.logger.warning(f"Failed to restart {self.service_name}: {exc}")


class SerialTransport:
    def __init__(self, port: str, baud: int, timeout: float, logger):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.logger = logger
        self.ser = None
        self.io_lock = threading.RLock()

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.logger.info(f"Opened serial port {self.port} at {self.baud} baud")
            return True
        except serial.SerialException as exc:
            self.logger.error(f"Failed to open serial port {self.port}: {exc}")
            return False

    def close(self):
        with self.io_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def write_json(self, cmd: dict):
        if not self.ser or not self.ser.is_open:
            return
        line = json.dumps(cmd, separators=(",", ":")) + "\n"
        with self.io_lock:
            try:
                self.ser.write(line.encode("utf-8"))
                self.ser.flush()
            except serial.SerialException as exc:
                self.logger.warning(f"Serial write error: {exc}")

    def send_cmd(self, cmd: dict) -> dict:
        if not self.ser or not self.ser.is_open:
            return {}
        line = json.dumps(cmd, separators=(",", ":")) + "\n"
        with self.io_lock:
            try:
                self.ser.write(line.encode("utf-8"))
                self.ser.flush()
                deadline = time.time() + self.timeout
                while time.time() < deadline:
                    raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if raw.startswith("{"):
                        try:
                            return json.loads(raw)
                        except json.JSONDecodeError:
                            continue
            except serial.SerialException as exc:
                self.logger.warning(f"Serial send error: {exc}")
        return {}

    def read_json_line(self) -> dict:
        if not self.ser or not self.ser.is_open:
            return {}
        with self.io_lock:
            try:
                if self.ser.in_waiting <= 0:
                    return {}
                raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException:
                return {}
        if raw.startswith("{"):
            try:
                return json.loads(raw)
            except json.JSONDecodeError:
                return {}
        return {}


class WaveRoverDriver(Node):
    def __init__(self):
        super().__init__("waverover_driver")

        self.declare_parameter("serial_port", "/dev/waverover")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        self.declare_parameter("max_linear_vel", 1.0)
        self.declare_parameter("max_angular_vel", 2.0)
        self.declare_parameter("cmd_vel_timeout", 0.8)
        self.declare_parameter("heartbeat_rate", 20.0)
        self.declare_parameter("load_calib_on_start", True)
        self.declare_parameter("rezero_imu_on_start", False)
        self.declare_parameter("heartbeat_delay_ms", 3000)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_odom_tf", False)
        self.declare_parameter("odom_topic", "odom/raw")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("feedback_rate", 100.0)
        self.declare_parameter("feedback_interval_ms", 10)
        self.declare_parameter("battery_rate", 1.0)
        self.declare_parameter("display_update_rate", 1.0)

        self.declare_parameter("battery_full_voltage", 12.6)
        self.declare_parameter("battery_empty_voltage", 9.0)
        self.declare_parameter("low_battery_percentage", 20.0)
        self.declare_parameter("very_low_battery_percentage", 10.0)

        self.declare_parameter("show_battery_percentage", True)
        self.declare_parameter("display_line_num", 4)
        self.declare_parameter("very_low_flash_rate", 2.0)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.serial_timeout = self.get_parameter("serial_timeout").value
        self.max_linear_vel = self.get_parameter("max_linear_vel").value
        self.max_angular_vel = self.get_parameter("max_angular_vel").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        self.heartbeat_rate = self.get_parameter("heartbeat_rate").value
        self.load_calib_on_start = self.get_parameter("load_calib_on_start").value
        self.rezero_imu_on_start = self.get_parameter("rezero_imu_on_start").value
        self.heartbeat_delay_ms = self.get_parameter("heartbeat_delay_ms").value
        self.publish_odom = self.get_parameter("publish_odom").value
        self.publish_odom_tf = self.get_parameter("publish_odom_tf").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.feedback_rate = self.get_parameter("feedback_rate").value
        self.feedback_interval_ms = self.get_parameter("feedback_interval_ms").value
        self.battery_rate = self.get_parameter("battery_rate").value
        self.display_update_rate = self.get_parameter("display_update_rate").value

        self.battery_full_voltage = self.get_parameter("battery_full_voltage").value
        self.battery_empty_voltage = self.get_parameter("battery_empty_voltage").value
        self.low_battery_percentage = self.get_parameter("low_battery_percentage").value
        self.very_low_battery_percentage = self.get_parameter("very_low_battery_percentage").value

        self.show_battery_percentage = self.get_parameter("show_battery_percentage").value
        self.display_line_num = self.get_parameter("display_line_num").value
        self.very_low_flash_rate = self.get_parameter("very_low_flash_rate").value

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        self.feedback_lock = threading.Lock()
        self.latest_json_feedback = "{}"
        self.dist_L = 0.0
        self.dist_R = 0.0
        self.dist_centre = 0.0
        self.yaw_odom = 0.0
        self.yaw_imu = 0.0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.voltage = 0.0
        self.battery_percentage = 0.0
        self.enc_A = 0
        self.enc_B = 0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.prev_dist_centre = None
        self.prev_odom_time = self.get_clock().now()
        self.running = True
        self.display_flash_on = True
        self.display_update_count = 0

        self.display_service = DisplayServiceGuard("waverover-init.service", self.get_logger())
        self.display_service.stop()
        atexit.register(self.display_service.start)
        time.sleep(0.5)

        self.transport = SerialTransport(self.serial_port, self.baud_rate, self.serial_timeout, self.get_logger())
        if not self.transport.open():
            self.display_service.start()
            raise SystemExit(1)

        self._initialize_robot()

        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)

        self.json_feedback_pub = self.create_publisher(String, "waverover/json_feedback", 10)
        self.raw_feedback_pub = self.create_publisher(Float32MultiArray, "waverover/raw_feedback", 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, "encoders", 10)
        self.battery_pub = self.create_publisher(BatteryState, "battery", 10)
        self.battery_percentage_pub = self.create_publisher(Float32, "battery_percentage", 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / self.heartbeat_rate, self._heartbeat_callback)
        self.create_timer(1.0 / self.feedback_rate, self._feedback_publish_callback)
        self.create_timer(1.0 / self.battery_rate, self._battery_publish_callback)
        self.create_timer(1.0 / self.display_update_rate, self._battery_display_callback)

        self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self._send_display_text("BAT --%", force=True)
        self.get_logger().info("WAVE ROVER driver started")

    def _initialize_robot(self):
        self.get_logger().info("Initializing WAVE ROVER base")
        self.transport.send_cmd({"T": 900, "main": 1, "module": 0})
        time.sleep(0.2)

        if self.load_calib_on_start:
            self.transport.send_cmd({"T": 161})
            time.sleep(0.2)

        if self.rezero_imu_on_start:
            self.get_logger().info("Re-zeroing IMU")
            time.sleep(2.0)
            self.transport.send_cmd({"T": 127})
            time.sleep(0.2)

        self.transport.send_cmd({"T": 151})
        time.sleep(0.2)
        self.transport.send_cmd({"T": 136, "cmd": self.heartbeat_delay_ms})
        time.sleep(0.1)
        self.transport.send_cmd({"T": 142, "cmd": int(self.feedback_interval_ms)})
        time.sleep(0.1)
        self.transport.send_cmd({"T": 131, "cmd": 1})
        time.sleep(0.1)
        self.transport.send_cmd({"T": 143, "cmd": 0})
        time.sleep(0.1)
        self._send_velocity(0.0, 0.0)

    def _cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = self.get_clock().now()
        linear = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        self.current_linear = linear
        self.current_angular = angular
        self._send_velocity(linear, angular)

    def _heartbeat_callback(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if age > self.cmd_vel_timeout:
            if self.current_linear != 0.0 or self.current_angular != 0.0:
                self.current_linear = 0.0
                self.current_angular = 0.0
                self.get_logger().warning("cmd_vel timeout, stopping robot")
            self._send_velocity(0.0, 0.0)
            return
        self._send_velocity(self.current_linear, self.current_angular)

    def _send_velocity(self, linear: float, angular: float):
        self.transport.write_json({"T": 13, "X": round(linear, 4), "Z": round(angular, 4)})

    def _serial_reader_loop(self):
        while self.running:
            try:
                data = self.transport.read_json_line()
                if data:
                    self._process_feedback(data)
                else:
                    time.sleep(0.002)
            except Exception as exc:
                self.get_logger().warning(f"Serial read loop error: {exc}")
                time.sleep(0.5)

    def _process_feedback(self, data: dict):
        with self.feedback_lock:
            self.latest_json_feedback = json.dumps(data, separators=(",", ":"))
            t_code = data.get("T", 0)
            if t_code == 1001:
                self.dist_L = float(data.get("dist_L", self.dist_L))
                self.dist_R = float(data.get("dist_R", self.dist_R))
                self.dist_centre = float(data.get("dist_centre", self.dist_centre))
                self.yaw_odom = float(data.get("yaw_odom", self.yaw_odom))
                self.yaw_imu = float(data.get("yaw_imu", self.yaw_imu))
                self.enc_A = int(data.get("encA", self.enc_A))
                self.enc_B = int(data.get("encB", self.enc_B))
                self.voltage = float(data.get("voltage", self.voltage))
                self.battery_percentage = self._estimate_battery_percentage(self.voltage)
            elif t_code == 1002:
                self.yaw_imu = float(data.get("yaw_imu", self.yaw_imu))
                self.imu_roll = float(data.get("r", self.imu_roll))
                self.imu_pitch = float(data.get("p", self.imu_pitch))

    def _estimate_battery_percentage(self, voltage: float) -> float:
        if voltage <= 0.01:
            return 0.0
        span = self.battery_full_voltage - self.battery_empty_voltage
        if span <= 0.01:
            return 0.0
        pct = (voltage - self.battery_empty_voltage) / span * 100.0
        return max(0.0, min(100.0, pct))

    def _feedback_publish_callback(self):
        with self.feedback_lock:
            json_text = self.latest_json_feedback
            raw_values = [
                float(self.dist_L), float(self.dist_R), float(self.dist_centre),
                float(self.yaw_odom), float(self.yaw_imu),
                float(self.imu_roll), float(self.imu_pitch),
                float(self.voltage), float(self.battery_percentage),
            ]
            encoders = [int(self.enc_A), int(self.enc_B)]

        json_msg = String()
        json_msg.data = json_text
        self.json_feedback_pub.publish(json_msg)

        raw_msg = Float32MultiArray()
        raw_msg.data = raw_values
        self.raw_feedback_pub.publish(raw_msg)

        encoder_msg = Int32MultiArray()
        encoder_msg.data = encoders
        self.encoder_pub.publish(encoder_msg)

        if self.publish_odom:
            self._publish_odom()

    def _yaw_radians(self, yaw_value: float) -> float:
        if abs(yaw_value) > 2.0 * math.pi:
            return math.radians(yaw_value)
        return yaw_value

    def _publish_odom(self):
        now = self.get_clock().now()
        with self.feedback_lock:
            dist_centre = float(self.dist_centre)
            yaw = self._yaw_radians(float(self.yaw_odom))

        if self.prev_dist_centre is None:
            delta_dist = 0.0
        else:
            delta_dist = dist_centre - self.prev_dist_centre

        dt = (now - self.prev_odom_time).nanoseconds / 1e9
        linear_velocity = delta_dist / dt if dt > 1e-6 else 0.0

        self.odom_x += delta_dist * math.cos(yaw)
        self.odom_y += delta_dist * math.sin(yaw)
        self.prev_dist_centre = dist_centre
        self.prev_odom_time = now

        half_yaw = yaw * 0.5
        orientation_z = math.sin(half_yaw)
        orientation_w = math.cos(half_yaw)
        stamp = now.to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.orientation.z = orientation_z
        odom_msg.pose.pose.orientation.w = orientation_w
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = float(self.current_angular)
        self.odom_pub.publish(odom_msg)

        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.odom_x
            transform.transform.translation.y = self.odom_y
            transform.transform.rotation.z = orientation_z
            transform.transform.rotation.w = orientation_w
            self.tf_broadcaster.sendTransform(transform)

    def _battery_publish_callback(self):
        with self.feedback_lock:
            voltage = float(self.voltage)
            percent = float(self.battery_percentage)

        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = voltage
        battery_msg.percentage = percent / 100.0
        battery_msg.present = True
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_pub.publish(battery_msg)

        pct_msg = Float32()
        pct_msg.data = percent
        self.battery_percentage_pub.publish(pct_msg)

    def _battery_display_callback(self):
        if not self.show_battery_percentage:
            return

        with self.feedback_lock:
            percent = float(self.battery_percentage)
            voltage = float(self.voltage)

        if voltage <= 0.01:
            self._send_display_text("BAT --%", force=True)
            return

        pct_int = int(round(percent))

        if percent <= self.very_low_battery_percentage:
            self.display_update_count += 1
            self.display_flash_on = not self.display_flash_on
            text = f"CRIT {pct_int}%" if self.display_flash_on else ""
            self._send_display_text(text, force=True)
            return

        if percent <= self.low_battery_percentage:
            self._send_display_text(f"LOW {pct_int}%", force=True)
            return

        self._send_display_text(f"BAT {pct_int}%", force=True)

    def _send_display_text(self, text: str, force: bool = False):
        # Always force for display reliability because other firmware/UI tasks may repaint the line.
        self.transport.write_json({"T": 3, "lineNum": int(self.display_line_num), "Text": str(text)})

    def destroy_node(self):
        self.get_logger().info("Stopping WAVE ROVER")
        try:
            self.running = False
            self._send_velocity(0.0, 0.0)
            time.sleep(0.1)
            self.transport.write_json({"T": 131, "cmd": 0})
            time.sleep(0.1)
            self.transport.close()
        finally:
            self.display_service.start()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaveRoverDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
