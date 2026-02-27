// ===== Pin mapping =====
// Left back: 2=IN2, 3=IN1  (방향 반대 -> SW에서 보정)
// Right back: 4=IN2, 5=IN1 (정상)
// Front steer: 6=IN2, 7=IN1 (Forward=좌, Reverse=우)

const int LB_IN2 = 2;
const int LB_IN1 = 3;

const int RB_IN2 = 4;
const int RB_IN1 = 5;

const int F_IN2  = 6;
const int F_IN1  = 7;

const int LED = 13;

// ---------- helpers ----------
void stop_back() {
  digitalWrite(LB_IN1, LOW); digitalWrite(LB_IN2, LOW);
  digitalWrite(RB_IN1, LOW); digitalWrite(RB_IN2, LOW);
}
void stop_front() {
  digitalWrite(F_IN1, LOW); digitalWrite(F_IN2, LOW);
}

// 왼뒤: 논리 forward/reverse를 실제 동작에 맞게 뒤집어둠
void left_back_forward() {   // 실제 전진
  digitalWrite(LB_IN1, LOW);
  digitalWrite(LB_IN2, HIGH);
}
void left_back_reverse() {   // 실제 후진
  digitalWrite(LB_IN1, HIGH);
  digitalWrite(LB_IN2, LOW);
}

// 오른뒤: 정상
void right_back_forward() {
  digitalWrite(RB_IN1, HIGH);
  digitalWrite(RB_IN2, LOW);
}
void right_back_reverse() {
  digitalWrite(RB_IN1, LOW);
  digitalWrite(RB_IN2, HIGH);
}

// 앞 조향: Forward=좌, Reverse=우
void steer_left() {
  digitalWrite(F_IN1, HIGH);
  digitalWrite(F_IN2, LOW);
}
void steer_right() {
  digitalWrite(F_IN1, LOW);
  digitalWrite(F_IN2, HIGH);
}

bool parse_line(const String& line, int &th, int &st) {
  // "TH:-1 ST:2"
  return (sscanf(line.c_str(), "TH:%d ST:%d", &th, &st) == 2);
}

void apply_cmd(int th, int st) {
  // LED로 구동 상태 표시(전진/후진이면 켜짐)
  digitalWrite(LED, (th != 0) ? HIGH : LOW);

  // 1) 구동(뒤 바퀴): th = -1/0/+1
  if (th == 0) {
    stop_back();
  } else if (th > 0) {
    left_back_forward();
    right_back_forward();
  } else { // th < 0
    left_back_reverse();
    right_back_reverse();
  }

  // 2) 조향(앞): TH와 무관하게 ST로 동작
  if (st < 0) {
    steer_left();
  } else if (st > 0) {
    steer_right();
  } else {
    stop_front();
  }
}

void setup() {
  pinMode(LED, OUTPUT);

  pinMode(LB_IN1, OUTPUT); pinMode(LB_IN2, OUTPUT);
  pinMode(RB_IN1, OUTPUT); pinMode(RB_IN2, OUTPUT);
  pinMode(F_IN1, OUTPUT);  pinMode(F_IN2, OUTPUT);

  stop_back();
  stop_front();
  digitalWrite(LED, LOW);

  Serial.begin(115200);
}

void loop() {
  static String line = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line.trim();
      int th = 0, st = 0;
      if (parse_line(line, th, st)) {
        // 안전 제한
        if (th > 1) th = 1;
        if (th < -1) th = -1;
        if (st < -2) st = -2;
        if (st > 2) st = 2;
        apply_cmd(th, st);
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 64) line = "";
    }
  }
}
