#pragma once
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <deque>
#include <sstream>
#include <string>
#include <vector>
using std::max;
constexpr int A4 = 58, LED_BUILTIN = 13, LOW = 0, HIGH = 1, OUTPUT = 1, INPUT = 0;
struct Write { unsigned long ms; int pin; int value; bool analog; };
inline unsigned long long fake_us = 0;
inline int pins[100]{};
inline std::vector<Write> writes;
inline std::deque<int> pot_samples;
inline int pot_default = 512;
inline unsigned long millis() { return static_cast<unsigned long>(fake_us / 1000); }
inline void delay(unsigned long ms) { fake_us += ms * 1000ULL; }
inline void delayMicroseconds(unsigned int us) { fake_us += us; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { pins[pin] = value; writes.push_back({millis(), pin, value, false}); }
inline int digitalRead(int pin) { return pins[pin]; }
inline void analogWrite(int pin, int value) { pins[pin] = value; writes.push_back({millis(), pin, value, true}); }
inline int analogRead(int) { if (pot_samples.empty()) return pot_default; auto x = pot_samples.front(); pot_samples.pop_front(); return x; }
struct FakeSerial {
  std::deque<char> input;
  std::string output;
  long baud = 0;
  void begin(long b) { baud = b; }
  int available() { return static_cast<int>(input.size()); }
  int read() { int c = input.front(); input.pop_front(); return c; }
  void enqueue(const std::string& s) { for (char c : s) input.push_back(c); }
  template<class T> void print(const T& value) { std::ostringstream s; s << value; output += s.str(); }
  template<class T> void println(const T& value) { print(value); output += '\n'; }
};
inline FakeSerial Serial;
