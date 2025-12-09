#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Int32, Bool, String, Empty


class MathQuizNode(Node):
    """
    Gère le challenge de calcul mental avec la télécommande IR.

    - Reçoit les digits IR: /smartiz/ir_digit (Int32).
    - Démarre / annule le challenge via:
        /smartiz/math_start (Empty)
        /smartiz/math_cancel (Empty)
    - Informe le GameMaster du résultat via:
        /smartiz/math_result (Bool)
    - Dialogue avec l'Arduino via /smartiz/arduino_cmd (String):
        MATH_Q a b      -> affiche "a x b = ?" + "Ans: __"
        MATH_OK         -> "Correct! Next: blow test"
        MATH_FAIL       -> "Wrong answer / Wait & retry"
        MATH_LOCK N     -> "Math locked, wait N s" (géré par Arduino)
    """

    def __init__(self):
        super().__init__("math_quiz_node")

        # Paramètres
        self.lock_seconds = 30.0

        # État du quiz
        self.active = False             # on attend une réponse ?
        self.locked_until = None        # rclpy Time ou None
        self.current_a = 0
        self.current_b = 0
        self.current_answer = 0
        self.required_digits = 0        # nb de digits attendus
        self.input_str = ""             # digits courants

        # ROS pubs/subs
        self.cmd_pub = self.create_publisher(String, "/smartiz/arduino_cmd", 10)
        self.result_pub = self.create_publisher(Bool, "/smartiz/math_result", 10)

        self.ir_sub = self.create_subscription(
            Int32, "/smartiz/ir_digit", self.ir_digit_callback, 10
        )
        self.start_sub = self.create_subscription(
            Empty, "/smartiz/math_start", self.start_callback, 10
        )
        self.cancel_sub = self.create_subscription(
            Empty, "/smartiz/math_cancel", self.cancel_callback, 10
        )

        # Timer pour gérer le lock (affichage du temps restant)
        self.lock_timer = self.create_timer(0.5, self.lock_timer_callback)

        self.get_logger().info("math_quiz_node ready.")

    # ----------------------------------------------------------
    # Helpers Arduino
    # ----------------------------------------------------------
    def send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    # ----------------------------------------------------------
    # Start / Cancel
    # ----------------------------------------------------------
    def start_callback(self, _msg: Empty):
        """GameMaster veut lancer un challenge math."""
        now = self.get_clock().now()

        # Si on est locké, prévenir Arduino et ne pas lancer de question
        if self.locked_until is not None and now < self.locked_until:
            remaining = (self.locked_until - now).nanoseconds / 1e9
            remaining_sec = max(1, int(remaining + 0.999))

            self.get_logger().info(
                f"Math locked, {remaining_sec}s remaining. Ignoring math_start."
            )

            # Laisse Arduino afficher un message explicite
            self.send_cmd(f"MATH_LOCK {remaining_sec}")
            return

        # Sinon -> nouvelle question
        self.start_new_question()

    def cancel_callback(self, _msg: Empty):
        """GameMaster annule le challenge math (retour au début du jeu)."""
        if self.active:
            self.get_logger().info("Math challenge cancelled by GameMaster.")
        self.active = False
        self.input_str = ""

    # ----------------------------------------------------------
    # Lock timer
    # ----------------------------------------------------------
    def lock_timer_callback(self):
        """Juste pour log / debug, pas d'affichage dynamique côté Arduino."""
        if self.locked_until is None:
            return
        now = self.get_clock().now()
        if now >= self.locked_until:
            # Lock terminé
            self.get_logger().info("Math lock finished, next smile can start a new quiz.")
            self.locked_until = None

    # ----------------------------------------------------------
    # Question
    # ----------------------------------------------------------
    def start_new_question(self):
        """
        Crée une nouvelle question simple.
        (Tu peux complexifier selon le nombre de Smarties, etc.)
        """
        import random

        self.active = True
        self.input_str = ""

        # Exemple: 2..9 x 2..9
        self.current_a = random.randint(2, 9)
        self.current_b = random.randint(2, 9)
        self.current_answer = self.current_a * self.current_b

        # Nombre de digits
        if self.current_answer >= 100:
            self.required_digits = 3
        elif self.current_answer >= 10:
            self.required_digits = 2
        else:
            self.required_digits = 1

        self.get_logger().info(
            f"New question: {self.current_a} x {self.current_b} = {self.current_answer} "
            f"(digits={self.required_digits})"
        )

        # Envoyer à l'Arduino
        self.send_cmd(f"MATH_Q {self.current_a} {self.current_b}")

    # ----------------------------------------------------------
    # IR digits
    # ----------------------------------------------------------
    def ir_digit_callback(self, msg: Int32):
        """Un chiffre de la télécommande est arrivé."""
        digit = msg.data

        now = self.get_clock().now()

        # Si lock encore actif -> ignorer clairement
        if self.locked_until is not None and now < self.locked_until:
            remaining = (self.locked_until - now).nanoseconds / 1e9
            remaining_sec = max(1, int(remaining + 0.999))
            self.get_logger().info(
                f"Ignoring IR digit {digit} because math is locked for {remaining_sec}s."
            )
            # On pourrait renvoyer MATH_LOCK de temps en temps mais ce n'est pas obligatoire.
            return

        # Si pas de question active -> on ignore aussi (ex : pas encore de smile, etc.)
        if not self.active:
            self.get_logger().info(f"Ignoring IR digit {digit} (no active question).")
            return

        # Ajout du digit dans la string courante
        if len(self.input_str) >= 4:
            # On limite à 4 digits pour éviter les débordements
            self.get_logger().info(f"Ignoring extra digit {digit} (input already full: {self.input_str}).")
            return

        self.input_str += str(digit)
        self.get_logger().info(
            f"User input now: {self.input_str} (needs {self.required_digits} digits)"
        )

        # L'Arduino gère déjà l'affichage des digits via IR, donc
        # on n'a pas besoin d'envoyer une commande spéciale ici.

        # Quand on a assez de digits -> on valide
        if len(self.input_str) >= self.required_digits:
            self.check_answer()

    # ----------------------------------------------------------
    # Validation
    # ----------------------------------------------------------
    def check_answer(self):
        """Compare la réponse utilisateur avec la bonne réponse."""
        try:
            user_val = int(self.input_str)
        except ValueError:
            user_val = -9999

        self.get_logger().info(
            f"Checking answer: user={user_val}, correct={self.current_answer}"
        )

        result_msg = Bool()

        if user_val == self.current_answer:
            # Bonne réponse
            result_msg.data = True
            self.result_pub.publish(result_msg)

            # UI côté Arduino
            self.send_cmd("MATH_OK")

            self.active = False
            self.input_str = ""

        else:
            # Mauvaise réponse -> lock
            result_msg.data = False
            self.result_pub.publish(result_msg)

            self.locked_until = self.get_clock().now() + Duration(seconds=self.lock_seconds)
            self.get_logger().info(
                f"Wrong answer -> locked for {int(self.lock_seconds)}s."
            )

            # Arduino: affiche "Wrong answer / Wait & retry"
            self.send_cmd("MATH_FAIL")

            self.active = False
            self.input_str = ""


def main(args=None):
    rclpy.init(args=args)
    node = MathQuizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
