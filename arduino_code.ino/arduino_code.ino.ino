#include <LiquidCrystal.h>
#include <Servo.h>
#include <IRremote.hpp>

// ======================================================
// PIN DEFINITIONS
// ======================================================
const int RED_PIN    = 9;
const int GREEN_PIN  = 10;
const int BLUE_PIN   = 11;

const int BUZZER_PIN = 12;
const int SERVO_PIN  = 2;
const int IR_PIN     = 13;

// LCD : RS, E, D4, D5, D6, D7
LiquidCrystal lcd(7, 8, 4, 5, 6, 3);

// Blow sensor (analog)
const int MOTOR_SENSE_PIN = A0;

// ======================================================
// SERVO CONFIG
// ======================================================
const int SERVO_CLOSED_ANGLE = 80;
const int SERVO_OPEN_ANGLE   = 175;
const unsigned long DEFAULT_SERVO_OPEN_MS = 300;

Servo rewardServo;
bool servoIsOpen = false;
unsigned long servoCloseAt = 0;

// ======================================================
// BLOW CHALLENGE (LOCAL, ON ARDUINO)
// ======================================================
int baseline = 0;
const int N_BASELINE = 100;
int MAX_DELTA = 175;              // tweak if needed
const int BLOW_SUCCESS_PERCENT = 60;

bool blowActive = false;
bool blowOkSent = false;
unsigned long lastBlowSampleMs = 0;
const unsigned long BLOW_REPORT_INTERVAL_MS = 100;



// Smarties cooldown UI state (driven by ROS via SMARTIES_COOLDOWN)
bool smartiesCooldownActive = false;
unsigned long smartiesCooldownEndMs = 0;
int lastSmartiesCooldownSeconds = -1;


// ======================================================
// IR REMOTE
// ======================================================
uint16_t lastIrCmd = 0;
unsigned long lastIrTime = 0;
const unsigned long IR_DEBOUNCE_MS = 200;

// ======================================================
// SERIAL / LCD
// ======================================================
const int SERIAL_BUF_SIZE = 64;
char serialBuf[SERIAL_BUF_SIZE];
int serialPos = 0;

// ======================================================
// MATH UI STATE (LOCAL, UI ONLY)
// ======================================================
bool mathModeActive = false;
int  mathA = 0;
int  mathB = 0;
char mathInput[5] = {0};
int  mathInputLen = 0;

// Math lock (30s countdown) – UI only
bool mathLockActive = false;
unsigned long mathLockEndMs = 0;
int lastMathLockSeconds = -1;

// ======================================================
// FORWARD DECLARATIONS
// ======================================================
void handleSerialInput();
void handleSerialCommand(char* line);

void handleIrRemote();
void onRemoteDigit(int digit);

void handleBlow();
int  computeBlowPercent();

void handleMathLock();
void handleSmartiesCooldown();

void openServo(unsigned long durationMs);
void closeServo();
void handleServoAutoClose();

void setColor(int r, int g, int b);
void resetLcd();
void printPadded16(const char* text);

// Sound helpers
void buzzerBeep(int freqHz, int durationMs);
void playSmileBeep();
void playBlowSuccessBeepBeep();
void playCountdownTick();
void playRewardMelody();

// Math UI
void showMathQuestion(int a, int b);
void updateMathAnswerDisplay();

// Global UI
void showIdleScreen();
void startBlowScreen();
void showBlowGauge(int percent);
void showBlowSuccess();

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  pinMode(RED_PIN,   OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN,  OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  rewardServo.attach(SERVO_PIN);
  rewardServo.write(SERVO_CLOSED_ANGLE);
  servoIsOpen = false;
  servoCloseAt = 0;

  resetLcd();
  showIdleScreen();
  setColor(0, 0, 0);

  // Blow baseline calibration
  delay(800);
  long sum = 0;
  for (int i = 0; i < N_BASELINE; i++) {
    sum += analogRead(MOTOR_SENSE_PIN);
    delay(5);
  }
  baseline = sum / N_BASELINE;

  // IR
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  randomSeed(analogRead(A1));
}


void handleSmartiesCooldown() {
  if (!smartiesCooldownActive) return;

  unsigned long now = millis();
  if (now >= smartiesCooldownEndMs) {
    smartiesCooldownActive = false;
    // when cooldown ends, only show idle if nothing else is active
    if (!mathModeActive && !blowActive) {
      showIdleScreen();
    }
    return;
  }

  unsigned long remainingMs = smartiesCooldownEndMs - now;
  int remainingSec = (remainingMs + 999) / 1000;  // round up
  if (remainingSec == lastSmartiesCooldownSeconds) {
    return;  // no need to refresh LCD each loop
  }
  lastSmartiesCooldownSeconds = remainingSec;

  lcd.clear();
  lcd.setCursor(0, 0);
  printPadded16("No Smarties now");
  lcd.setCursor(0, 1);
  char line[17];
  snprintf(line, 17, "Wait %2ds", remainingSec);
  printPadded16(line);
}


// ======================================================
// MAIN LOOP
// ======================================================
void loop() {
  handleSerialInput();
  handleIrRemote();
  handleMathLock();          // update math lock countdown
  handleSmartiesCooldown();  // update Smarties cooldown countdown
  handleBlow();
  handleServoAutoClose();
  delay(2);
}

// ======================================================
// SERIAL INPUT
// ======================================================
void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      serialBuf[serialPos] = '\0';
      if (serialPos > 0) {
        handleSerialCommand(serialBuf);
      }
      serialPos = 0;
    } else {
      if (serialPos < SERIAL_BUF_SIZE - 1) {
        serialBuf[serialPos++] = c;
      } else {
        serialPos = 0; // overflow -> reset
      }
    }
  }
}

void handleSerialCommand(char* line) {


  // GAME_IDLE: go back to base screen (unless we show a countdown)
  if (strcmp(line, "GAME_IDLE") == 0) {
    mathModeActive = false;
    blowActive = false;
    blowOkSent = false;

    if (!mathLockActive && !smartiesCooldownActive) {
      showIdleScreen();
    }
    return;
  }

  
  // SMARTIES_COOLDOWN <sec> : show Smarties cooldown countdown
  {
    int sec = 0;
    if (sscanf(line, "SMARTIES_COOLDOWN %d", &sec) == 1 && sec > 0) {
      smartiesCooldownActive      = true;
      smartiesCooldownEndMs       = millis() + (unsigned long)sec * 1000UL;
      lastSmartiesCooldownSeconds = -1;

      lcd.clear();
      lcd.setCursor(0, 0);
      printPadded16("No Smarties now");
      lcd.setCursor(0, 1);
      char lineBuf[17];
      snprintf(lineBuf, 17, "Wait %2ds", sec);
      printPadded16(lineBuf);
      playCountdownTick();
      return;
    }
  }

  // SMILE_OK: short "Smile detected" screen
  if (strcmp(line, "SMILE_OK") == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    printPadded16("Smile detected!");
    lcd.setCursor(0, 1);
    printPadded16("Math coming...");
    return;
  }

  // MATH_Q <a> <b>: display question
  if (strncmp(line, "MATH_Q ", 7) == 0) {
    int a, b;
    if (sscanf(line + 7, "%d %d", &a, &b) == 2) {
      showMathQuestion(a, b);
    }
    return;
  }

  // MATH_OK: correct answer
  if (strcmp(line, "MATH_OK") == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    printPadded16("Correct!");
    lcd.setCursor(0, 1);
    printPadded16("Next: blow test");
    playBlowSuccessBeepBeep();
    mathModeActive = false;
    mathInputLen   = 0;
    mathInput[0]   = '\0';
    delay(800);
    return;
  }

  // MATH_FAIL: wrong answer / challenge locked
  if (strcmp(line, "MATH_FAIL") == 0) {
    mathModeActive = false;
    mathInputLen   = 0;
    mathInput[0]   = '\0';

    mathLockActive        = true;
    mathLockEndMs         = millis() + 30000UL; // 30 seconds
    lastMathLockSeconds   = -1;

    lcd.clear();
    lcd.setCursor(0, 0);
    printPadded16("Math locked 30s");
    lcd.setCursor(0, 1);
    printPadded16("Wait & try again");
    playCountdownTick();
    return;
  }

  // BLOW_START: enable blow challenge
  if (strcmp(line, "BLOW_START") == 0) {
    blowActive = true;
    blowOkSent = false;
    startBlowScreen();
    return;
  }

  // BLOW_CANCEL
  if (strcmp(line, "BLOW_CANCEL") == 0) {
    blowActive = false;
    blowOkSent = false;
    if (!mathLockActive && !smartiesCooldownActive) {
      showIdleScreen();
    }
    return;
  }


  // LED r g b
  if (strncmp(line, "LED ", 4) == 0) {
    int r, g, b;
    if (sscanf(line + 4, "%d %d %d", &r, &g, &b) == 3) {
      r = constrain(r, 0, 255);
      g = constrain(g, 0, 255);
      b = constrain(b, 0, 255);
      setColor(r, g, b);
    }
    return;
  }

  // SERVO_OPEN <ms>
  if (strncmp(line, "SERVO_OPEN", 10) == 0) {
    unsigned int dur;
    if (sscanf(line + 10, "%u", &dur) != 1) {
      dur = DEFAULT_SERVO_OPEN_MS;
    }
    openServo(dur);
    return;
  }

  if (strcmp(line, "SERVO_CLOSE") == 0) {
    closeServo();
    return;
  }

  // SND commands
  if (strncmp(line, "SND ", 4) == 0) {
    char* p = line + 4;
    if (strcmp(p, "SMILE") == 0)         playSmileBeep();
    else if (strcmp(p, "BLOW_OK") == 0)  playBlowSuccessBeepBeep();
    else if (strcmp(p, "COUNT") == 0)    playCountdownTick();
    else if (strcmp(p, "REWARD") == 0)   playRewardMelody();
    return;
  }
}

// ======================================================
// IR REMOTE
// ======================================================
void handleIrRemote() {
  if (!IrReceiver.decode()) {
    return;
  }

  auto &data = IrReceiver.decodedIRData;
  uint16_t cmd = data.command;
  unsigned long now = millis();

  // Simple debounce
  if (cmd == lastIrCmd && (now - lastIrTime) < IR_DEBOUNCE_MS) {
    IrReceiver.resume();
    return;
  }
  lastIrCmd = cmd;
  lastIrTime = now;

  int digit = -1;
  switch (cmd) {
    case 0x16: digit = 0; break;
    case 0x0C: digit = 1; break;
    case 0x18: digit = 2; break;
    case 0x5E: digit = 3; break;
    case 0x08: digit = 4; break;
    case 0x1C: digit = 5; break;
    case 0x5A: digit = 6; break;
    case 0x42: digit = 7; break;
    case 0x52: digit = 8; break;
    case 0x4A: digit = 9; break;
    default:   digit = -1; break;
  }

  if (digit != -1) {
    onRemoteDigit(digit);
  }

  IrReceiver.resume();
}

void onRemoteDigit(int digit) {
  // Always send digit event back to ROS
  Serial.print("IR ");
  Serial.println(digit);

  // Local math answer UI
  if (mathModeActive) {
    if (mathInputLen < 4) {
      mathInput[mathInputLen] = char('0' + digit);
      mathInputLen++;
      mathInput[mathInputLen] = '\0';
    }
    updateMathAnswerDisplay();
  }
}

// ======================================================
// MATH LOCK HANDLING (COUNTDOWN ON LCD)
// ======================================================
void handleMathLock() {
  if (!mathLockActive) return;

  unsigned long now = millis();
  if (now >= mathLockEndMs) {
    mathLockActive = false;
    // If Smarties cooldown is also active, its handler will draw
    if (!smartiesCooldownActive) {
      showIdleScreen();
    }
    return;
  }

  unsigned long remainingMs = mathLockEndMs - now;
  int remainingSec = (remainingMs + 999) / 1000; // round up
  if (remainingSec == lastMathLockSeconds) {
    return; // no LCD refresh needed
  }
  lastMathLockSeconds = remainingSec;

  lcd.clear();
  lcd.setCursor(0, 0);
  printPadded16("Math locked");
  lcd.setCursor(0, 1);
  char line[17];
  snprintf(line, 17, "Wait %2ds", remainingSec);
  printPadded16(line);
}

// ======================================================
// SMARTIES COOLDOWN HANDLING (COUNTDOWN ON LCD)
// ======================================================
// void handleSmartiesCooldown() {
//   if (!smartiesCooldownActive) return;

//   unsigned long now = millis();
//   if (now >= smartiesCooldownEndMs) {
//     smartiesCooldownActive = false;
//     // If math is locked, its handler owns the screen
//     if (!mathLockActive) {
//       showIdleScreen();
//     }
//     return;
//   }

//   unsigned long remainingMs = smartiesCooldownEndMs - now;
//   int remainingSec = (remainingMs + 999) / 1000; // round up
//   if (remainingSec == lastSmartiesCooldownSeconds) {
//     return; // nothing new to show
//   }
//   lastSmartiesCooldownSeconds = remainingSec;

//   lcd.clear();
//   lcd.setCursor(0, 0);
//   printPadded16("No Smarties now");
//   lcd.setCursor(0, 1);
//   char line[17];
//   snprintf(line, 17, "Wait %2ds", remainingSec);
//   printPadded16(line);
// }

// ======================================================
// BLOW CHALLENGE
// ======================================================
void handleBlow() {
  if (!blowActive) return;

  unsigned long now = millis();
  if (now - lastBlowSampleMs < BLOW_REPORT_INTERVAL_MS) {
    return;
  }
  lastBlowSampleMs = now;

  int percent = computeBlowPercent();
  percent = constrain(percent, 0, 100);

  // LCD gauge + percentage
  showBlowGauge(percent);

  // Debug / logging to ROS
  Serial.print("BLOW ");
  Serial.println(percent);

  // Success threshold
  if (!blowOkSent && percent >= BLOW_SUCCESS_PERCENT) {
    blowOkSent = true;
    blowActive = false;
    Serial.println("BLOW_OK");  // -> ROS
    showBlowSuccess();
    playBlowSuccessBeepBeep();
    delay(800);
    // Do NOT force idle here, GameMaster decides next step
  }
}

int computeBlowPercent() {
  const int N = 4;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(MOTOR_SENSE_PIN);
    delay(1);
  }

  int val   = sum / N;
  int delta = val - baseline;
  if (delta < 0) delta = 0;

  int percent = map(delta, 0, MAX_DELTA, 0, 100);
  return percent;
}

// ======================================================
// SERVO
// ======================================================
void openServo(unsigned long durationMs) {
  // Reward screen + sound before opening the trap
  lcd.clear();
  lcd.setCursor(0, 0);
  printPadded16("Bravo! Reward :)");
  lcd.setCursor(0, 1);
  printPadded16("Smarties ----->");
  playRewardMelody();
  delay(1500);

  rewardServo.write(SERVO_OPEN_ANGLE);
  servoIsOpen  = true;
  servoCloseAt = millis() + durationMs;
}

void closeServo() {
  rewardServo.write(SERVO_CLOSED_ANGLE);
  servoIsOpen  = false;
  servoCloseAt = 0;
}

void handleServoAutoClose() {
  if (servoIsOpen && millis() >= servoCloseAt) {
    closeServo();
    if (!mathLockActive && !smartiesCooldownActive) {
      showIdleScreen();
    }
  }
}

// ======================================================
// LCD HELPERS
// ======================================================
void resetLcd() {
  lcd.begin(16, 2);
  lcd.clear();
  delay(10);
}

void printPadded16(const char* text) {
  char buf[17];
  strncpy(buf, text, 16);
  buf[16] = '\0';
  int len = strlen(buf);
  for (int i = len; i < 16; i++) {
    buf[i] = ' ';
  }
  buf[16] = '\0';
  lcd.print(buf);
}

void showIdleScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  printPadded16("SMARTiz+ ready");
  lcd.setCursor(0, 1);
  printPadded16("Smile to start :)");
}

// ======================================================
// BLOW UI
// ======================================================
void startBlowScreen() {
  // Directly show gauge with 0% so the user understands what to do
  showBlowGauge(0);
}

void showBlowGauge(int percent) {
  // First line: bar
  lcd.setCursor(0, 0);
  int filled = map(percent, 0, 100, 0, 16);
  for (int i = 0; i < 16; i++) {
    lcd.print(i < filled ? '#' : ' ');
  }

  // Second line: numeric percentage
  lcd.setCursor(0, 1);
  char line[17];
  snprintf(line, 17, "Blow %3d%%", percent);
  printPadded16(line);
}

void showBlowSuccess() {
  lcd.clear();
  lcd.setCursor(0, 0);
  printPadded16("Bravo!");
  lcd.setCursor(0, 1);
  printPadded16("Blow success :)");
}

// ======================================================
// LED
// ======================================================
void setColor(int r, int g, int b) {
  analogWrite(RED_PIN,   r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN,  b);
}

// ======================================================
// SOUND
// ======================================================
void buzzerBeep(int freqHz, int durationMs) {
  if (freqHz <= 0 || durationMs <= 0) return;
  int periodMicros = 1000000L / freqHz;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(periodMicros / 2);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(periodMicros / 2);
  }
}

void playSmileBeep() {
  buzzerBeep(2000, 80);
}

void playBlowSuccessBeepBeep() {
  buzzerBeep(1600, 80);
  delay(80);
  buzzerBeep(2200, 120);
}

void playCountdownTick() {
  buzzerBeep(1500, 50);
}

void playRewardMelody() {
  buzzerBeep(1800, 80);
  delay(40);
  buzzerBeep(2200, 80);
  delay(40);
  buzzerBeep(2600, 120);
}

// ======================================================
// MATH UI
// ======================================================
void showMathQuestion(int a, int b) {
  mathModeActive = true;
  mathA          = a;
  mathB          = b;
  mathInputLen   = 0;
  mathInput[0]   = '\0';

  lcd.clear();
  lcd.setCursor(0, 0);
  char qline[17];
  snprintf(qline, 17, "%2d x %2d = ?", a, b);
  printPadded16(qline);

  lcd.setCursor(0, 1);
  printPadded16("Ans: __");
}

void updateMathAnswerDisplay() {
  if (!mathModeActive) return;
  lcd.setCursor(0, 1);
  if (mathInputLen == 0) {
    printPadded16("Ans: __");
  } else {
    char line[17];
    snprintf(line, 17, "Ans: %s", mathInput);
    printPadded16(line);
  }
}
