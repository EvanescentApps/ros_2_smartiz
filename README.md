# Smartiz+ ROS2 + Arduino Game

Interactive Smarties dispenser that mixes computer vision, an IR remote math quiz, and a blow sensor challenge. A ROS2 stack orchestrates the flow (smile -> math -> blow -> reward) while an Arduino drives the LCD, RGB LED, buzzer, IR decoding, blow sensor, and the servo-controlled trapdoor.

<img width="4000" height="3000" alt="20251209_111410" src="https://github.com/user-attachments/assets/11e21c5f-5b11-4132-967e-bf62b681c358" />

## System Overview
- **Camera → Face/Smile detection**: `cam_reader` publishes `/camera/image_raw`; `face_smile_processor` uses Haar cascades to publish `/dominant_color_code` (0 no face, 1 face, 2 smile).
- **GameMaster** (`mmi_hamster/game_master.py`): orchestrates the stages, sends UI/sound/servo commands to the Arduino on `/smartiz/arduino_cmd`, and handles cooldowns between rewards.
- **Math quiz** (`math_quiz_node.py`): generates a multiplication question on the Arduino LCD, ingests IR digits from `/smartiz/ir_digit`, and reports success/fail on `/smartiz/math_result` (with a 30 s lockout on failure).
- **Arduino bridge** (`microcontroller_communicator.py`): serial link to the board (`/dev/ttyACM0` by default). Publishes IR digits (`/smartiz/ir_digit`), blow percentages (`/smartiz/blow_percent`), and blow success (`/smartiz/blow_success`); relays `/smartiz/arduino_cmd` strings to the sketch.
- **Blow challenge**: handled locally on the Arduino. A BLOW_OK serial message becomes `/smartiz/blow_success`, which lets GameMaster trigger the servo reward once the cooldown is over.
- **Launch**: `launch/hamster_all.launch.py` starts all nodes.

### ROS Topics (main ones)
- Input: `/camera/image_raw` (Image), `/dominant_color_code` (Int32), `/smartiz/ir_digit` (Int32), `/smartiz/blow_percent` (Int32), `/smartiz/blow_success` (Bool)
- Control: `/smartiz/math_start` (Empty), `/smartiz/math_cancel` (Empty)
- Output to Arduino: `/smartiz/arduino_cmd` (String)
- Results: `/smartiz/math_result` (Bool)

<img width="3000" height="4000" alt="20251209_111725" src="https://github.com/user-attachments/assets/a2b6b655-55d2-4acb-a9ff-86838a0b5311" />

## Arduino Sketch (arduino_code.ino/arduino_code.ino.ino)
- Hardware: RGB LED (pins 9/10/11), buzzer (12), servo (2), IR receiver (13), LCD RS/E/D4..7 (7,8,4,5,6,3), blow sensor on A0.
- Behaviors:
  - Calibrates blow baseline at boot; maps sensor delta to 0–100%.
  - Displays idle/smile/math/blow/cooldown screens on the 16x2 LCD and plays short sounds.
  - Detects IR remote digits and echoes `IR <digit>` over serial (used by the math quiz).
  - Runs the blow gauge locally and emits `BLOW_OK` when threshold is exceeded.
  - Drives the servo for rewards (`SERVO_OPEN <ms>` auto-closes after the duration).
- Serial commands understood:
  - `GAME_IDLE`, `SMILE_OK`
  - `MATH_Q a b`, `MATH_OK`, `MATH_FAIL`, `MATH_LOCK <sec>`
  - `BLOW_START`, `BLOW_CANCEL`
  - `SMARTIES_COOLDOWN <sec>`
  - `LED r g b`, `SERVO_OPEN <ms>`, `SERVO_CLOSE`
  - `SND SMILE|BLOW_OK|COUNT|REWARD`
  - Arduino → ROS: `IR <d>`, `BLOW <pct>`, `BLOW_OK`

## Building and Running
1. Install dependencies (ROS 2 + OpenCV/bridge, pyserial):
   - `sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport python3-serial`
2. From your ROS workspace root:
   ```bash
   colcon build --packages-select mmi_hamster
   source install/setup.bash
   ```
3. Connect the USB camera and the Arduino (adjust serial port if not `/dev/ttyACM0`).
4. Launch everything:
   ```bash
   ros2 launch mmi_hamster hamster_all.launch.py
   ```
   - Override serial port: `ros2 run mmi_hamster microcontroller_communicator --ros-args -p port:=/dev/ttyUSB0`

## Game Flow
1. User smiles at the camera → `face_smile_processor` publishes code 2.
2. GameMaster waits `after_smile_delay` then tells `math_quiz_node` to generate a multiplication.
3. User types the answer with the IR remote (digits relayed by Arduino). Correct → `math_result=True`.
4. GameMaster asks the Arduino to start the blow challenge (`BLOW_START`). When the Arduino reports `BLOW_OK`, GameMaster checks cooldown and triggers the reward servo + melody.
5. Cooldown (`cooldown_seconds`, default 30 s) blocks new rewards; Arduino shows the remaining seconds.

### Timing parameters (ROS parameters on `game_master`)
- `after_smile_delay` (default 2.5 s): pause between the first smile and the start of the math quiz.
- `reward_delay` (default 2.0 s): pause between blow success and the reward (gives you time to speak/demo before the trapdoor opens).
- `cooldown_seconds` (default 30 s): minimum delay between two rewards.

## Development Notes
- Package: `mmi_hamster` (ament_python). Key sources under `mmi_hamster/`.
- Vision: Haar cascades from `/usr/share/opencv4/haarcascades` (`face_smile_processor.py`); ensure cascades are available on the target OS.
- Legacy/unused nodes removed: `blow_challenge_node.py` (ROS-driven blow UI) and `image_processor.py` (old dominant color detector).
- Tests are lint-only stubs in `test/`.

## Hardware Checklist
- Arduino-compatible board with LiquidCrystal, Servo, and IRremote libraries.
- 16x2 LCD wired as in the sketch, RGB LED + resistor pack, piezo buzzer, hobby servo, IR receiver, blow sensor on A0.
