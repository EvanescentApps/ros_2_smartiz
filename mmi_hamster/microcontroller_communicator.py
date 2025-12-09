#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Int32, Bool, String

import serial
import threading
import time


class MicrocontrollerCommunicator(Node):
    """
    Bridge entre ROS2 et l'Arduino.

    - Ouvre le port série.
    - Reçoit :
        * "IR <digit>"   -> /smartiz/ir_digit (Int32)
        * "BLOW <pct>"   -> /smartiz/blow_percent (Int32)
        * "BLOW_OK"      -> /smartiz/blow_success (Bool True)
    - Envoie :
        * /smartiz/arduino_cmd (String) -> lignes texte vers Arduino
    """

    def __init__(self):
        super().__init__("microcontroller_communicator")

        # --- Params ---
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        # --- Serial ---
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Opened serial port {port} at {baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")

        # --- State ---
        self.arduino_connected = self.ser is not None
        self.last_heartbeat_time = self.get_clock().now()

        # --- Publishers ---
        self.connected_pub = self.create_publisher(Bool, "/smartiz/arduino_connected", 10)
        self.ir_digit_pub = self.create_publisher(Int32, "/smartiz/ir_digit", 10)
        self.blow_pub = self.create_publisher(Int32, "/smartiz/blow_percent", 10)
        self.blow_success_pub = self.create_publisher(Bool, "/smartiz/blow_success", 10)

        # --- Souscription commandes vers Arduino ---
        self.cmd_sub = self.create_subscription(
            String, "/smartiz/arduino_cmd", self.arduino_cmd_callback, 10
        )

        # --- Timers ---
        self.status_timer = self.create_timer(0.5, self.publish_status)

        # --- Thread lecture série ---
        self.reader_thread = None
        self.reader_running = False
        if self.ser is not None:
            self.reader_running = True
            self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
            self.reader_thread.start()

        self.get_logger().info("microcontroller_communicator ready (no handshake).")

    # =========================================================
    # Envoi vers Arduino
    # =========================================================
    def write_line(self, line: str):
        if self.ser is None:
            return
        try:
            self.ser.write((line + "\n").encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.arduino_connected = False

    def arduino_cmd_callback(self, msg: String):
        self.write_line(msg.data)

    # =========================================================
    # Lecture série (thread)
    # =========================================================
    def read_loop(self):
        buffer = b""
        while self.reader_running:
            if self.ser is None:
                time.sleep(0.1)
                continue

            try:
                data = self.ser.read(128)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.arduino_connected = False
                time.sleep(0.1)
                continue

            if not data:
                continue

            buffer += data
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="ignore").strip()
                if line:
                    self.handle_line_from_arduino(line)

    def handle_line_from_arduino(self, line: str):
        # Un peu de debug possible :
        # self.get_logger().info(f"From Arduino: {line}")

        # IR digit: "IR 3"
        if line.startswith("IR "):
            try:
                digit_str = line.split(" ", 1)[1]
                digit = int(digit_str)
                msg = Int32()
                msg.data = digit
                self.ir_digit_pub.publish(msg)
                self.update_heartbeat()
            except ValueError:
                self.get_logger().warn(f"Bad IR line: {line}")
            return

        # Blow percent: "BLOW 42"
        if line.startswith("BLOW "):
            try:
                pct_str = line.split(" ", 1)[1]
                pct = int(pct_str)
                msg = Int32()
                msg.data = pct
                self.blow_pub.publish(msg)
                self.update_heartbeat()
            except ValueError:
                self.get_logger().warn(f"Bad BLOW line: {line}")
            return

        # Blow success: "BLOW_OK"
        if line == "BLOW_OK":
            self.get_logger().info("Received BLOW_OK from Arduino.")
            msg = Bool()
            msg.data = True
            self.blow_success_pub.publish(msg)
            self.update_heartbeat()
            return

        # Toute autre ligne = activité
        self.update_heartbeat()

    # =========================================================
    # Heartbeat / status
    # =========================================================
    def update_heartbeat(self):
        self.last_heartbeat_time = self.get_clock().now()
        self.arduino_connected = True

    def publish_status(self):
        # Très simple : "on a vu des données récemment ?" 
        now = self.get_clock().now()
        dt = (now - self.last_heartbeat_time).nanoseconds / 1e9
        if dt > 5.0:
            self.arduino_connected = False

        msg = Bool()
        msg.data = self.arduino_connected
        self.connected_pub.publish(msg)

    # =========================================================
    # Shutdown
    # =========================================================
    def destroy_node(self):
        self.reader_running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrocontrollerCommunicator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
