#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Int32, Bool, String, Empty


class BlowChallengeNode(Node):
    """
    Blow challenge node.

    - Waits for /smartiz/blow_start to be triggered by GameMaster.
    - While active, reads /smartiz/blow_percent (0..100) and:
        * updates a bar graph on the LCD (LCD0),
        * shows "Blow XX%" on LCD1.
    - When blow_percent >= threshold for a short stable time,
      publishes /smartiz/blow_success = True once and sends "SND BLOW_OK".
    """

    def __init__(self):
        super().__init__("blow_challenge_node")

        # Parameters
        self.threshold = 60          # % required
        self.min_hold_time = 0.5     # seconds above threshold needed

        # State
        self.active = False
        self.last_percent = -1
        self.first_above_time = None
        self.success_already_sent = False

        # ROS interface
        self.blow_sub = self.create_subscription(
            Int32, "/smartiz/blow_percent", self.blow_callback, 10
        )
        self.start_sub = self.create_subscription(
            Empty, "/smartiz/blow_start", self.start_callback, 10
        )
        self.cancel_sub = self.create_subscription(
            Empty, "/smartiz/blow_cancel", self.cancel_callback, 10
        )

        self.cmd_pub = self.create_publisher(String, "/smartiz/arduino_cmd", 10)
        self.success_pub = self.create_publisher(Bool, "/smartiz/blow_success", 10)

        # Timer to refresh UI even if percent doesn't change (optional)
        self.ui_timer = self.create_timer(0.1, self.ui_timer_callback)

        self.get_logger().info("blow_challenge_node ready.")

    # ------------------------------------------------------------------
    # Helper to send commands to Arduino
    # ------------------------------------------------------------------
    def send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Start / cancel
    # ------------------------------------------------------------------
    def start_callback(self, _: Empty):
        """
        Called by GameMaster when it wants to begin the blow challenge.
        """
        self.get_logger().info("Blow challenge START.")
        self.active = True
        self.last_percent = -1
        self.first_above_time = None
        self.success_already_sent = False

        # Initial UI
        self.send_cmd("CLEAR")
        self.send_cmd("LCD0 Blow on the fan")
        self.send_cmd("LCD1 Waiting...    ")

    def cancel_callback(self, _: Empty):
        """
        Called when GameMaster wants to abort the blow challenge
        (e.g. user failed earlier, or returning to idle).
        """
        self.get_logger().info("Blow challenge CANCEL.")
        self.active = False
        self.first_above_time = None

    # ------------------------------------------------------------------
    # Blow readings
    # ------------------------------------------------------------------
    def blow_callback(self, msg: Int32):
        """
        Called whenever a new blow percentage is received from Arduino.
        """
        if not self.active:
            return

        percent = max(0, min(100, msg.data))

        # Update UI if changed
        if percent != self.last_percent:
            self.last_percent = percent
            self.update_lcd(percent)

        # Check threshold logic
        self.check_success_condition(percent)

    def check_success_condition(self, percent: int):
        """
        Detects when blow_percent stays above threshold for min_hold_time seconds.
        """
        now = self.get_clock().now()

        if percent >= self.threshold:
            # Start or continue counting time above threshold
            if self.first_above_time is None:
                self.first_above_time = now
            else:
                elapsed = (now - self.first_above_time).nanoseconds / 1e9
                if (
                    not self.success_already_sent
                    and elapsed >= self.min_hold_time
                ):
                    self.report_success()
        else:
            # Reset if user drops below threshold
            self.first_above_time = None

    def report_success(self):
        """
        Publishes blow_success=True and plays a short confirmation sound.
        """
        self.get_logger().info("Blow challenge SUCCESS.")
        self.success_already_sent = True
        self.active = False  # stop updating once we've succeeded

        # Notify GameMaster
        msg = Bool()
        msg.data = True
        self.success_pub.publish(msg)

        # UI feedback
        self.send_cmd("CLEAR")
        self.send_cmd("LCD0 Bravo!")
        self.send_cmd("LCD1 Blow success :)")
        self.send_cmd("SND BLOW_OK")

    # ------------------------------------------------------------------
    # LCD update
    # ------------------------------------------------------------------
    def update_lcd(self, percent: int):
        """
        Build a simple bar (16 chars) on line 0 and text on line 1.
        """
        # Bar length from 0 to 16
        bar_len = int(percent * 16 / 100)
        bar = "#" * bar_len + " " * (16 - bar_len)

        self.send_cmd(f"LCD0 {bar}")
        self.send_cmd(f"LCD1 Blow {percent:3d}%")

    # ------------------------------------------------------------------
    # UI timer (optional)
    # ------------------------------------------------------------------
    def ui_timer_callback(self):
        """
        Regular UI refresh in case you want to animate something even
        when blow percent is constant. Here we simply ensure the last
        known percent is still shown if active.
        """
        if self.active and self.last_percent >= 0:
            # Re-send current bar occasionally (it's cheap & robust)
            self.update_lcd(self.last_percent)


def main(args=None):
    rclpy.init(args=args)
    node = BlowChallengeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
