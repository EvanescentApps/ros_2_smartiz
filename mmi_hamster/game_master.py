#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool, String
from std_msgs.msg import Empty


class GameMaster(Node):
    """
    High‑level orchestrator for the Smartiz+ game.

    Responsibilities:
      1. Wait for a smile from /dominant_color_code (2 = smile).
      2. Start the math quiz via /smartiz/math_start.
      3. If math_result == True -> start blow challenge (BLOW_START to Arduino).
      4. If blow_success == True:
           - Check Smarties cooldown.
           - If cooldown OK: open servo (SERVO_OPEN), play reward sound.
           - If cooldown NOT OK: tell Arduino to show Smarties cooldown message.

    The Arduino handles:
      - LCD UI (idle, smile OK, math question, blow gauge, reward screens),
      - IR keypad input + local math answer display,
      - Blow sensor + blow success detection,
      - Servo auto‑close timing.

    This node only drives the game stages and the Smarties cooldown.
    """

    STAGE_WAIT_FACE   = 0
    STAGE_AFTER_SMILE = 1
    STAGE_MATH        = 2
    STAGE_BLOW        = 3

    def __init__(self):
        super().__init__("game_master")

        # --- Configuration ---
        self.cooldown_seconds = 30.0          # minimum time between 2 rewards
        self.after_smile_delay = 1.5          # delay from smile to math start (s)

        # --- State ---
        self.stage = self.STAGE_WAIT_FACE
        self.first_smile_seen = False
        self.smile_time = None                # rclpy Time

        self.reward_count = 0
        self.last_reward_time = None          # rclpy Time of last Smarties reward

        # --- Subscriptions ---
        # 0 = no face, 1 = face, 2 = smile
        self.face_sub = self.create_subscription(
            Int32, "/dominant_color_code", self.face_callback, 10
        )

        # True = math success, False = math failed / locked
        self.math_res_sub = self.create_subscription(
            Bool, "/smartiz/math_result", self.math_result_callback, 10
        )

        # True = blow challenge success, emitted once per challenge
        self.blow_success_sub = self.create_subscription(
            Bool, "/smartiz/blow_success", self.blow_success_callback, 10
        )

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(String, "/smartiz/arduino_cmd", 10)
        self.math_start_pub = self.create_publisher(Empty, "/smartiz/math_start", 10)
        self.math_cancel_pub = self.create_publisher(Empty, "/smartiz/math_cancel", 10)

        # Periodic timer for stage transitions
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Initial idle screen
        self.send_cmd("GAME_IDLE")

        self.get_logger().info("GameMaster ready (Arduino handles UI + blow).")

    # =========================================================
    # Helper to send text commands to Arduino
    # =========================================================
    def send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    # =========================================================
    # Stage management
    # =========================================================
    def set_stage(self, new_stage: int):
        if new_stage == self.stage:
            return

        self.get_logger().info(f"Stage change: {self.stage} -> {new_stage}")
        self.stage = new_stage

        if new_stage == self.STAGE_WAIT_FACE:
            self.enter_wait_face_stage()
        elif new_stage == self.STAGE_AFTER_SMILE:
            # Nothing special, timer_callback will move to MATH
            pass
        elif new_stage == self.STAGE_MATH:
            self.enter_math_stage()
        elif new_stage == self.STAGE_BLOW:
            self.enter_blow_stage()

    def enter_wait_face_stage(self):
        """Reset to initial state: waiting for a smile."""
        self.first_smile_seen = False
        self.smile_time = None

        # Stop ongoing challenges
        self.math_cancel_pub.publish(Empty())
        self.send_cmd("BLOW_CANCEL")

        # Let Arduino show its idle screen
        self.send_cmd("GAME_IDLE")
        # Turn off LED
        self.send_cmd("LED 0 0 0")

    def enter_math_stage(self):
        """Start the math challenge."""
        self.get_logger().info("Starting math challenge.")
        # Small "go" beep
        self.send_cmd("SND COUNT")
        # Ask the math quiz node to generate a new question
        self.math_start_pub.publish(Empty())

    def enter_blow_stage(self):
        """Start the blow challenge on the Arduino side."""
        self.get_logger().info("Starting blow challenge.")
        # Arduino will show its own blow UI when it receives BLOW_START
        self.send_cmd("BLOW_START")

    # =========================================================
    # Face / smile handling
    # =========================================================
    def face_callback(self, msg: Int32):
        """
        /dominant_color_code:
          0 = no face, 1 = face, 2 = smile.

        We only use the first smile event to move from WAIT_FACE to AFTER_SMILE.
        """
        code = msg.data

        # LED feedback based on current face state
        if code == 0:
            self.send_cmd("LED 255 0 0")   # red = no face
        elif code == 1:
            self.send_cmd("LED 0 255 0")   # green = face
        elif code == 2:
            self.send_cmd("LED 0 0 255")   # blue = smile

        # Only react to smiles when we are in the initial stage
        if self.stage != self.STAGE_WAIT_FACE:
            return

        if code == 2 and not self.first_smile_seen:
            # First smile detected
            self.first_smile_seen = True
            self.smile_time = self.get_clock().now()

            # Ask Arduino to show a "Smile OK" message + play a short sound
            self.send_cmd("SMILE_OK")
            self.send_cmd("SND SMILE")

            # Move to intermediate stage, timer will then start math
            self.set_stage(self.STAGE_AFTER_SMILE)

    # =========================================================
    # Timer: after-smile delay -> start math
    # =========================================================
    def timer_callback(self):
        if self.stage == self.STAGE_AFTER_SMILE and self.smile_time is not None:
            elapsed = (self.get_clock().now() - self.smile_time).nanoseconds / 1e9
            if elapsed >= self.after_smile_delay:
                self.set_stage(self.STAGE_MATH)

    # =========================================================
    # Math result handling
    # =========================================================
    def math_result_callback(self, msg: Bool):
        """
        Called by math_quiz_node when the user answer is validated.

        - True  => success, move to blow challenge.
        - False => wrong answer / math locked, go back to smile stage.
                  UI and lock countdown are managed by math_quiz_node + Arduino
                  via MATH_FAIL.
        """
        if self.stage != self.STAGE_MATH:
            return

        if msg.data:
            # Math success -> start blow challenge
            self.set_stage(self.STAGE_BLOW)
        else:
            # Math failed: small feedback, then reset to WAIT_FACE.
            self.send_cmd("SND COUNT")
            self.set_stage(self.STAGE_WAIT_FACE)

    # =========================================================
    # Blow success handling
    # =========================================================
    def blow_success_callback(self, msg: Bool):
        """
        Called when Arduino sends BLOW_OK and microcontroller_communicator
        publishes /smartiz/blow_success = True.
        """
        if not msg.data:
            return
        if self.stage != self.STAGE_BLOW:
            # Ignore stray events
            return

        remaining = self.get_cooldown_remaining_seconds()
        if remaining > 0:
            # <<< THIS IS WHAT YOU SEE IN THE LOG >>>
            self.get_logger().info(
                f"Reward cooldown not finished ({remaining}s remaining)."
            )

            # Tell the Arduino to show a Smarties cooldown countdown
            self.send_cmd(f"SMARTIES_COOLDOWN {remaining}")
            self.send_cmd("SND COUNT")

            # Back to initial stage (Arduino stays on cooldown screen)
            self.set_stage(self.STAGE_WAIT_FACE)
            return

        # Cooldown OK -> reward
        self.trigger_reward()


    # =========================================================
    # Cooldown helpers
    # =========================================================
    def is_cooldown_over(self) -> bool:
        """Returns True if enough time has passed since the last reward."""
        if self.last_reward_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_reward_time).nanoseconds / 1e9
        return elapsed >= self.cooldown_seconds

    def get_cooldown_remaining_seconds(self) -> int:
        """
        Returns the remaining cooldown in full seconds (rounded up).
        0 means "ready now".
        """
        if self.last_reward_time is None:
            return 0

        elapsed = (self.get_clock().now() - self.last_reward_time).nanoseconds / 1e9
        remaining = self.cooldown_seconds - elapsed
        if remaining <= 0:
            return 0

        # round up to next integer second
        return int(remaining + 0.999)

    # =========================================================
    # Reward
    # =========================================================
    def trigger_reward(self):
        """
        Actually open the servo and play a reward sound.
        The Arduino:
          - shows a "Bravo / reward" screen,
          - plays a melody,
          - opens + auto‑closes the trapdoor.
        """
        self.get_logger().info("Triggering Smarties reward.")

        # Open servo for 300 ms (auto‑close handled by Arduino)
        self.send_cmd("SERVO_OPEN 300")
        self.send_cmd("SND REWARD")

        # Update cooldown state
        self.last_reward_time = self.get_clock().now()
        self.reward_count += 1

        # Back to initial stage
        self.set_stage(self.STAGE_WAIT_FACE)


def main(args=None):
    rclpy.init(args=args)
    node = GameMaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
