// Minimal Arduino shim so the library can be compiled and tested on a PC.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

uint32_t millis();
void delay(uint32_t ms);

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(const char* s) { fputs(s, stdout); return strlen(s); }
  size_t print(const char* s) { return write(s); }
  size_t print(int v) { char b[24]; snprintf(b, sizeof b, "%d", v); return write(b); }
  size_t print(unsigned v) { char b[24]; snprintf(b, sizeof b, "%u", v); return write(b); }
  size_t print(unsigned v, int base) { char b[24]; snprintf(b, sizeof b, base == 16 ? "%X" : "%u", v); return write(b); }
  size_t print(float v, int d = 2) { char b[40]; snprintf(b, sizeof b, "%.*f", d, (double)v); return write(b); }
  size_t println(const char* s = "") { size_t n = write(s); return n + write("\n"); }
  size_t println(int v) { size_t n = print(v); return n + write("\n"); }
  size_t println(unsigned v) { size_t n = print(v); return n + write("\n"); }
  size_t print(int v, int base) { char b[24]; snprintf(b, sizeof b, base == 16 ? "%X" : "%d", v); return write(b); }
  size_t println(int v, int base) { size_t n = print(v, base); return n + write("\n"); }
  size_t println(unsigned v, int base) { size_t n = print(v, base); return n + write("\n"); }
  size_t println(float v, int d = 2) { size_t n = print(v, d); return n + write("\n"); }
};

class HardwareSerial : public Print {
 public:
  void begin(unsigned long) {}
  operator bool() const { return true; }
};
extern HardwareSerial Serial;

#define HEX 16
#define DEC 10

#define INPUT_PULLUP 2
#define FALLING 2
inline void pinMode(int, int) {}
inline void attachInterrupt(int, void (*)(), int) {}
inline int digitalPinToInterrupt(int p) { return p; }
