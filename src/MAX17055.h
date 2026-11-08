// MAX17055.h - Maxim / Analog Devices MAX17055 single-cell ModelGauge m5 EZ fuel gauge.
//
//   * I2C address 0x36, DevName 0x4010.
//   * Low-side sensing: sense resistor between CSP (battery side, also the IC
//     ground) and CSN (load side).
//   * Thermistor on THRM/AIN (see README for the divider).
//   * EZ configuration also seeds dQAcc and dPAcc, as in the MAX17055
//     software implementation flow.
//
// Licensed under the MIT License (see LICENSE).

#pragma once
#include "ModelGaugeM5.h"

class MAX17055 : public ModelGaugeM5 {
 public:
  static constexpr uint8_t  DEFAULT_ADDRESS = 0x36;
  static constexpr uint16_t DEV_NAME        = 0x4010;

  MAX17055() : ModelGaugeM5(Variant::Max17055) {}

  // Look for a MAX17055 on the bus (only 0x36 is possible for this part).
  static bool detect(TwoWire& wire = Wire) {
    uint8_t a = 0;
    return findAddress(wire, DEV_NAME, a);
  }
};
