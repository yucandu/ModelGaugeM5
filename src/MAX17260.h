// MAX17260.h - Maxim / Analog Devices MAX17260 single-cell ModelGauge m5 EZ fuel gauge.
//
//   * I2C address 0x36 ("S" parts) - the "B" ordering variant uses another
//     address, use detect() to find it. DevName 0x4031.
//   * Low-side OR high-side current sensing. The IC works out which one at
//     boot from the CSN voltage; this library preserves that choice when it
//     writes ModelCfg (see ModelGaugeM5.cpp, writeModelCfgAndRefresh()).
//   * Thermistor on TH (10 k or 100 k, Config::thermistor100k) or the
//     +-1 degC die sensor.
//   * Wiring reminders: low-side: CSN to the load side of the shunt, GND to
//     the battery side, CSPH tied to BATT. High-side: CSN to the cell side,
//     CSPH to the system side of the shunt. See the datasheet figures.
//
// Licensed under the MIT License (see LICENSE).

#pragma once
#include "ModelGaugeM5.h"

class MAX17260 : public ModelGaugeM5 {
 public:
  static constexpr uint8_t  DEFAULT_ADDRESS = 0x36;
  static constexpr uint16_t DEV_NAME        = 0x4031;

  MAX17260() : ModelGaugeM5(Variant::Max17260) {}

  // Probe 0x36, 0x0D and 0x1A for a MAX17260 and return its 7-bit address.
  static bool detect(TwoWire& wire, uint8_t& address) {
    return findAddress(wire, DEV_NAME, address);
  }

  // Raw ModelCfg.CSEL bit (the gauge's own high/low-side detection result).
  // The datasheet does not say which level means which, so it is not
  // interpreted here - read it once on your board and note the value.
  bool readCsel(bool& csel) {
    uint16_t v = 0;
    if (!readRegister(m5g::Reg::MODELCFG, v)) return false;
    csel = (v & m5g::ModelCfgBit::CSEL) != 0;
    return true;
  }
};
