// Compile the actual firmware with mocked GPIO, UART, time, and pot samples.
// This is neither an AVR board build nor a steering/vehicle physics model.
#include "Arduino.h"
#include <iostream>
#include <stdexcept>
#include "../../Arduino/S1/S1.ino"

void require(bool ok, const std::string& msg) { if (!ok) throw std::runtime_error(msg); }
void reset_state() {
  fake_us = 0; std::fill(std::begin(pins), std::end(pins), 0); writes.clear();
  pot_samples.clear(); pot_default = 512; Serial = FakeSerial{};
  lineLen = 0; lineOverflow = false; val_min = 0; val_max = 1023; val_mid = 512;
  calibrated = false; higherPotIsDir1 = true; cmd_th = cmd_stn = target_angle_deg = 0;
  control_activated = false; last_steer_update_ms = 0; steer_active = false;
  last_steer_dir = 0; last_packet_ms = 0;
}
void send(const std::string& s) { Serial.enqueue(s + "\n"); loop(); }
bool drive_stopped() { return pins[MOTOR_L_IN1] == 0 && pins[MOTOR_L_IN2] == 0 && pins[MOTOR_R_IN1] == 0 && pins[MOTOR_R_IN2] == 0; }
void good_span(int mid) {
  for (int i = 0; i < 12; ++i) pot_samples.push_back(800);
  for (int i = 0; i < 12; ++i) pot_samples.push_back(200);
  pot_default = mid;
}

int main() {
  try {
    reset_state(); calibrated = true; delay(20); send("TH:500 STN:333");
    require(cmd_th == 500 && cmd_stn == 333 && target_angle_deg == 9, "valid parse");
    require(pins[MOTOR_L_IN1] == 90 && pins[MOTOR_R_IN1] == 90, "PWM output");
    send("TH:0 STN:0"); require(drive_stopped(), "stop command");

    reset_state(); calibrated = true; send("TH:-2000 STN:-2000");
    require(cmd_th == -1000 && cmd_stn == -1000 && pins[MOTOR_L_IN2] == 180, "reverse/clamp");

    reset_state(); calibrated = true; send("TH:500 STN:0"); delay(2000); loop();
    require(!drive_stopped(), "timeout strict boundary"); delay(1); loop();
    require(drive_stopped() && pins[STEER_PWM] == 0, "receive watchdog");

    reset_state(); setup();
    require(!calibrated && Serial.output.find("CALIB:FAILED") != std::string::npos, "span failure");
    send("TH:300 STN:500"); require(drive_stopped() && pins[STEER_PWM] == 0 && !control_activated, "failed calibration must block drive");

    reset_state(); good_span(200); calibrateSteering();
    require(!calibrated && Serial.output.find("CALIB:FAILED center timeout") != std::string::npos, "center timeout");
    require(Serial.output.find("CALIB:DONE") == std::string::npos, "no false completion");
    send("TH:300 STN:0"); require(drive_stopped(), "center failure must block drive");

    reset_state(); good_span(500); Serial.enqueue("TH:500 STN:0\n"); setup(); loop();
    require(calibrated && !control_activated && drive_stopped() && Serial.input.empty(), "discard commands buffered during setup");
    send("TH:500 STN:0"); require(!drive_stopped(), "fresh post-calibration command");
    good_span(500); Serial.enqueue("CAL:START\nTH:500 STN:0\n"); loop();
    require(calibrated && drive_stopped() && !control_activated, "discard commands buffered during recalibration");

    reset_state(); calibrated = true;
    Serial.enqueue("TH:500 STN:"); loop(); require(drive_stopped(), "partial command must not drive");
    Serial.enqueue("0\n"); loop(); require(!drive_stopped(), "complete split UART line");
    send("TH:0 STN:0");
    for (const auto& bad : {"TH:500", "TH:500 STN:0garbage", "TH:999999999999999999999999 STN:0", "CAL:STARTgarbage"}) {
      send(bad); require(drive_stopped(), "reject malformed line");
    }
    send(std::string(80, 'X') + "TH:500 STN:0");
    require(drive_stopped(), "overflow tail must not be interpreted as new command");
    send("TH:200 STN:0"); require(pins[MOTOR_L_IN1] == 36, "recover at next newline");

    reset_state(); send("STATUS");
    require(Serial.output.find("STATUS:NOT_READY") != std::string::npos, "not-ready reply");
    calibrated = true; send("STATUS");
    require(Serial.output.find("STATUS:READY") != std::string::npos, "ready reply");
    require(!control_activated && last_packet_ms == 0, "status must not activate/refresh motion watchdog");
    std::cout << "PASS actual S1.ino: command/PWM/stop, clamp, watchdog, span/center failure, startup/recalibration stale RX, split/malformed/overflow lines, STATUS.\n";
  } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
