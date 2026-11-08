// Minimal Wire shim: forwards transactions to a simulated device.
#pragma once
#include <stdint.h>
#include <stddef.h>

struct I2cDevice {
  virtual ~I2cDevice() {}
  virtual bool present(uint8_t addr) = 0;
  virtual bool onWrite(uint8_t addr, const uint8_t* data, size_t n) = 0;  // false = NACK
  virtual size_t onRead(uint8_t addr, uint8_t* out, size_t n) = 0;
};

class TwoWire {
 public:
  static I2cDevice* device;
  void begin() {}
  void beginTransmission(uint8_t a) { addr_ = a; txLen_ = 0; }
  size_t write(uint8_t b) { if (txLen_ < sizeof(tx_)) tx_[txLen_++] = b; return 1; }
  uint8_t endTransmission(bool = true) {
    if (!device || !device->present(addr_)) return 2;
    return device->onWrite(addr_, tx_, txLen_) ? 0 : 3;
  }
  int requestFrom(int a, int n) {
    rxPos_ = rxLen_ = 0;
    if (!device || !device->present((uint8_t)a)) return 0;
    rxLen_ = device->onRead((uint8_t)a, rx_, (size_t)n);
    return (int)rxLen_;
  }
  int read() { return rxPos_ < rxLen_ ? rx_[rxPos_++] : -1; }
  int available() { return (int)(rxLen_ - rxPos_); }
 private:
  uint8_t addr_ = 0;
  uint8_t tx_[64];
  size_t txLen_ = 0;
  uint8_t rx_[64];
  size_t rxPos_ = 0, rxLen_ = 0;
};
extern TwoWire Wire;
