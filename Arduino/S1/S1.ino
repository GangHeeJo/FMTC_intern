/*
  ROS2 SerialSender compatible Arduino Mega code
  PWM throttle version

  Expected serial lines:
    - "TH:<-1000..1000> STN:<-1000..1000>"
    - "CAL:START"

  Behavior:
    - Drive command is latched until a new command arrives
    - TH magnitude is converted to PWM
    - STN controls steering angle
*/

#include <Arduino.h>
#include <errno.h>

// -------------------- Pins --------------------
const int MOTOR_L_IN1 = 7;
const int MOTOR_L_IN2 = 6;
const int MOTOR_R_IN1 = 4;
const int MOTOR_R_IN2 = 5;

const int STEER_IN1 = 2;
const int STEER_IN2 = 3;
const int STEER_PWM = 8;

const int POT_PIN = A4;

// -------------------- Serial --------------------
const long BAUD = 115200;

// -------------------- Drive params --------------------
const int TH_MAX = 1000;          // incoming TH range: -1000..1000
const int DRIVE_PWM_MAX = 180;    // set your max drive PWM here (0..255)
const float DRIVE_GAIN = 1.0f;    // extra global gain
const int DRIVE_MIN_PWM = 0;      // set >0 if motor deadband exists

// optional left/right trim
const float LEFT_GAIN = 1.00f;
const float RIGHT_GAIN = 1.00f;

// -------------------- Calibration params --------------------
const unsigned long CALIB_MOTOR_MS  = 2200;
const unsigned long CALIB_SETTLE_MS = 250;
const int CALIB_PWM = 150;

// -------------------- Steering control params --------------------
const int STN_MAX = 1000;
const int STEER_ANGLE_MAX_DEG = 30;

const int STEER_DEAD_BAND_DEG = 3;
const int STEER_HYST_DEG = 4;
const int STEER_PWM_RUN = 150;
const unsigned long STEER_UPDATE_MS = 10;

// -------------------- Serial parser --------------------
const int LINE_BUF = 80;
char lineBuf[LINE_BUF];
int lineLen = 0;
bool lineOverflow = false;

// -------------------- Calibration state --------------------
int val_min = 0;
int val_max = 1023;
int val_mid = 512;
bool calibrated = false;

bool higherPotIsDir1 = true;

// -------------------- Command state --------------------
int cmd_th = 0;              // -1000 .. 1000
int cmd_stn = 0;             // -1000 .. 1000
int target_angle_deg = 0;
bool control_activated = false;

// -------------------- Timing / control state --------------------
unsigned long last_steer_update_ms = 0;
bool steer_active = false;
int last_steer_dir = 0;
unsigned long last_packet_ms = 0;      // 마지막 시리얼 패킷 수신 시각
const unsigned long PACKET_TIMEOUT_MS = 2000; // 2초 타임아웃

// -------------------- Helpers --------------------
int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int potReadAvg(int n = 3) {
  long s = 0;
  for (int i = 0; i < n; i++) {
    s += analogRead(POT_PIN);
    delayMicroseconds(300);
  }
  return (int)(s / n);
}

// -------------------- Drive motor control --------------------
void driveStop() {
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, 0);
}

void driveForwardPwm(int pwmL, int pwmR) {
  analogWrite(MOTOR_L_IN1, pwmL);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, pwmR);
  analogWrite(MOTOR_R_IN2, 0);
}

void driveBackwardPwm(int pwmL, int pwmR) {
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, pwmL);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, pwmR);
}

int throttleToPwm(int th) {
  int mag = abs(clampi(th, -TH_MAX, TH_MAX));

  if (mag == 0) return 0;

  float ratio = (float)mag / (float)TH_MAX;   // 0..1
  float pwm_f = ratio * (float)DRIVE_PWM_MAX * DRIVE_GAIN;

  int pwm = (int)(pwm_f + 0.5f);
  pwm = clampi(pwm, 0, DRIVE_PWM_MAX);

  if (pwm > 0 && pwm < DRIVE_MIN_PWM) {
    pwm = DRIVE_MIN_PWM;
  }

  return pwm;
}

// -------------------- Steering motor control --------------------
void steerStop() {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, LOW);
  analogWrite(STEER_PWM, 0);
  last_steer_dir = 0;
}

void steerDir1(int pwm) {
  digitalWrite(STEER_IN1, HIGH);
  digitalWrite(STEER_IN2, LOW);
  analogWrite(STEER_PWM, pwm);
}

void steerDir2(int pwm) {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, HIGH);
  analogWrite(STEER_PWM, pwm);
}

void steerTowardHigherPot(int pwm) {
  if (higherPotIsDir1) {
    steerDir1(pwm);
    last_steer_dir = 1;
  } else {
    steerDir2(pwm);
    last_steer_dir = 1;
  }
}

void steerTowardLowerPot(int pwm) {
  if (higherPotIsDir1) {
    steerDir2(pwm);
    last_steer_dir = -1;
  } else {
    steerDir1(pwm);
    last_steer_dir = -1;
  }
}

// -------------------- Pot / angle conversion --------------------
int potToAngleDeg(int pot) {
  if (!calibrated) return 0;

  if (pot >= val_mid) {
    int span = max(1, val_max - val_mid);
    long num = (long)(pot - val_mid) * STEER_ANGLE_MAX_DEG;
    return (int)(num / span);
  } else {
    int span = max(1, val_mid - val_min);
    long num = (long)(val_mid - pot) * STEER_ANGLE_MAX_DEG;
    return -(int)(num / span);
  }
}

int angleDegToTargetPot(int angleDeg) {
  angleDeg = clampi(angleDeg, -STEER_ANGLE_MAX_DEG, STEER_ANGLE_MAX_DEG);

  if (!calibrated) return val_mid;

  if (angleDeg >= 0) {
    long pot = (long)val_mid + (long)(val_max - val_mid) * angleDeg / STEER_ANGLE_MAX_DEG;
    return (int)pot;
  } else {
    long pot = (long)val_mid - (long)(val_mid - val_min) * (-angleDeg) / STEER_ANGLE_MAX_DEG;
    return (int)pot;
  }
}

int stnToAngleDeg(int stn) {
  stn = clampi(stn, -STN_MAX, STN_MAX);
  long angle = (long)stn * STEER_ANGLE_MAX_DEG / STN_MAX;
  return (int)angle;
}

// -------------------- Non-blocking update --------------------
void updateDrive() {
  if (!calibrated || !control_activated) {
    driveStop();
    return;
  }
  int pwmBase = throttleToPwm(cmd_th);

  int pwmL = clampi((int)(pwmBase * LEFT_GAIN + 0.5f), 0, DRIVE_PWM_MAX);
  int pwmR = clampi((int)(pwmBase * RIGHT_GAIN + 0.5f), 0, DRIVE_PWM_MAX);

  if (cmd_th > 0) {
    driveForwardPwm(pwmL, pwmR);
  } else if (cmd_th < 0) {
    driveBackwardPwm(pwmL, pwmR);
  } else {
    driveStop();
  }
}

void updateSteering() {
  unsigned long now = millis();

  if (!calibrated || !control_activated) {
    steerStop();
    return;
  }

  if (now - last_steer_update_ms < STEER_UPDATE_MS) {
    return;
  }
  last_steer_update_ms = now;

  int desired_angle = clampi(target_angle_deg, -STEER_ANGLE_MAX_DEG, STEER_ANGLE_MAX_DEG);

  int p = potReadAvg(2);
  int curAngle = potToAngleDeg(p);
  int errDeg = desired_angle - curAngle;

  if (!steer_active) {
    if (abs(errDeg) <= STEER_HYST_DEG) {
      steerStop();
      return;
    }
    steer_active = true;
  } else {
    if (abs(errDeg) <= STEER_DEAD_BAND_DEG) {
      steer_active = false;
      steerStop();
      return;
    }
  }

  int targetPot = angleDegToTargetPot(desired_angle);

  if (p < targetPot) {
    steerTowardHigherPot(STEER_PWM_RUN);
  } else {
    steerTowardLowerPot(STEER_PWM_RUN);
  }
}

// -------------------- Calibration helpers --------------------
void moveSteeringRawForMs(bool dir1, int pwm, unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (dir1) steerDir1(pwm);
    else steerDir2(pwm);
    delay(2);
  }
  steerStop();
}

bool moveSteeringToTargetPotBlocking(int targetPot, unsigned long timeoutMs) {
  targetPot = clampi(targetPot, val_min, val_max);
  unsigned long t0 = millis();

  while (millis() - t0 < timeoutMs) {
    int p = potReadAvg(3);
    int err = targetPot - p;

    if (abs(err) <= 5) {
      steerStop();
      return true;
    }

    if (err > 0) steerTowardHigherPot(STEER_PWM_RUN);
    else steerTowardLowerPot(STEER_PWM_RUN);

    delay(4);
  }

  steerStop();
  return false;
}

// -------------------- Calibration --------------------
void discardBufferedCommands() {
  // Commands queued while calibration blocked are too old to start driving.
  while (Serial.available() > 0) Serial.read();
  lineLen = 0;
  lineOverflow = false;
}

void calibrateSteering() {
  Serial.println("CALIB:START");

  calibrated = false;
  control_activated = false;
  steer_active = false;
  cmd_th = 0;
  cmd_stn = 0;
  target_angle_deg = 0;
  driveStop();
  steerStop();
  delay(200);

  moveSteeringRawForMs(true, CALIB_PWM, CALIB_MOTOR_MS);
  delay(CALIB_SETTLE_MS);
  int p1 = potReadAvg(12);

  Serial.print("CALIB end1=");
  Serial.println(p1);

  moveSteeringRawForMs(false, CALIB_PWM, CALIB_MOTOR_MS);
  delay(CALIB_SETTLE_MS);
  int p2 = potReadAvg(12);

  Serial.print("CALIB end2=");
  Serial.println(p2);

  if (p1 > p2) {
    higherPotIsDir1 = true;
    val_max = p1;
    val_min = p2;
  } else {
    higherPotIsDir1 = false;
    val_max = p2;
    val_min = p1;
  }

  if (abs(val_max - val_min) < 50) {
    Serial.println("CALIB:FAILED span too small");
    calibrated = false;
    steerStop();
    driveStop();
    discardBufferedCommands();
    return;
  }

  val_mid = (val_min + val_max) / 2;

  Serial.print("CALIB val_min=");
  Serial.println(val_min);
  Serial.print("CALIB val_max=");
  Serial.println(val_max);
  Serial.print("CALIB val_mid=");
  Serial.println(val_mid);

  if (!moveSteeringToTargetPotBlocking(val_mid, 2500)) {
    Serial.println("CALIB:FAILED center timeout");
    steerStop();
    driveStop();
    discardBufferedCommands();
    return;
  }

  int p = potReadAvg(8);
  Serial.print("CALIB mid_reached pot=");
  Serial.println(p);
  calibrated = true;
  steer_active = false;
  discardBufferedCommands();
  Serial.println("CALIB:DONE");
}

// -------------------- Serial command handling --------------------
bool parseControlLine(const char* s, int& th, int& stn) {
  if (strncmp(s, "TH:", 3) != 0) return false;
  char* end;
  errno = 0;
  long rawTh = strtol(s + 3, &end, 10);
  if (end == s + 3 || errno == ERANGE) return false;
  if (*end != ' ' && *end != '\t') return false;
  while (*end == ' ' || *end == '\t') ++end;
  if (strncmp(end, "STN:", 4) != 0) return false;
  const char* start = end + 4;
  errno = 0;
  long rawStn = strtol(start, &end, 10);
  if (end == start || errno == ERANGE) return false;
  while (*end == ' ' || *end == '\t') ++end;
  if (*end != '\0') return false;
  th = rawTh > TH_MAX ? TH_MAX : (rawTh < -TH_MAX ? -TH_MAX : (int)rawTh);
  stn = rawStn > STN_MAX ? STN_MAX : (rawStn < -STN_MAX ? -STN_MAX : (int)rawStn);
  return true;
}

void handleLine(const char* s) {
  while (*s == ' ' || *s == '\r' || *s == '\n' || *s == '\t') s++;

  if (strcmp(s, "STATUS") == 0) {
    Serial.println(calibrated ? "STATUS:READY" : "STATUS:NOT_READY");
    return;
  }

  if (strcmp(s, "CAL:START") == 0) {
    calibrateSteering();
    return;
  }

  int th = 0;
  int stn = 0;
  if (parseControlLine(s, th, stn)) {
    if (!calibrated) {
      cmd_th = 0;
      cmd_stn = 0;
      control_activated = false;
      driveStop();
      steerStop();
      return;
    }
    // 패킷이 들어왔으므로 타임아웃 타이머 리셋
    last_packet_ms = millis();

    // [진단용] 유효한 신호를 받을 때마다 아두이노 내장 LED를 반전시킵니다.
    // LED가 미친듯이 깜빡여야 정상입니다. 안 깜빡이면 신호가 안 오는 겁니다.
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    // [수정 포인트] 시동 후 처음으로 조이스틱이나 스로틀을 건드리면 제어 활성화
    // 10은 미세한 노이즈를 무시하기 위한 최소값입니다.
    if (!control_activated && (abs(th) > 10 || abs(stn) > 10)) {
      control_activated = true;
    }

    if (control_activated) {
      cmd_th = clampi(th, -TH_MAX, TH_MAX);
      cmd_stn = clampi(stn, -STN_MAX, STN_MAX);
      target_angle_deg = stnToAngleDeg(cmd_stn);
    }
    return;
  }

  if (strlen(s) > 0) {
    Serial.print("UNKNOWN:");
    Serial.println(s);
  }
}

void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (!lineOverflow) handleLine(lineBuf);
      lineLen = 0;
      lineOverflow = false;
    } else if (c != '\r') {
      if (lineOverflow) continue;
      if (lineLen < LINE_BUF - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;
        lineOverflow = true;
      }
    }
  }
}

// -------------------- Arduino setup / loop --------------------
void setup() {
  Serial.begin(BAUD);

  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  pinMode(STEER_PWM, OUTPUT);

  pinMode(POT_PIN, INPUT);

  driveStop();
  steerStop();

  delay(500);
  Serial.println("BOOT");

  calibrateSteering();
}

void loop() {
  readSerialLines();

   // No valid control packet for more than 2 seconds: stop both actuators.
  if (control_activated && (millis() - last_packet_ms > PACKET_TIMEOUT_MS)) {
    cmd_th = 0;
    cmd_stn = 0;
    target_angle_deg = 0;
    driveStop();
    steerStop();
  } else {
    // 신호가 살아있는 동안은 마지막 명령을 유지하며 계속 실행
  updateDrive();
  updateSteering();
  }
}
