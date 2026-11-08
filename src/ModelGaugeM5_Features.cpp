// ModelGaugeM5_Features.cpp - measurements, calibration, alerts, power modes,
// algorithm tuning, AtRate / dynamic power.
// Licensed under the MIT License (see LICENSE).

#include "ModelGaugeM5.h"

#include <string.h>

using namespace m5g;

namespace {

long roundToLong(float x) { return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f); }

uint8_t clampU8(long v) { return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); }

int8_t clampS8(long v) { return (int8_t)(v < -128 ? -128 : (v > 127 ? 127 : v)); }

uint16_t clampU16(long v) { return (uint16_t)(v < 0 ? 0 : (v > 65535 ? 65535 : v)); }

}  // namespace

// ===========================================================================
// Register-type readers
// ===========================================================================

bool ModelGaugeM5::readS16(uint8_t reg, int16_t& v) {
  uint16_t raw = 0;
  if (!readReg(reg, raw)) return false;
  v = (int16_t)raw;
  return true;
}

bool ModelGaugeM5::readPercent(uint8_t reg, float& pct) {
  uint16_t raw = 0;
  if (!readReg(reg, raw)) return false;
  pct = (float)raw * Scale::PERCENT_PER_LSB;
  return true;
}

bool ModelGaugeM5::readCapacity(uint8_t reg, float& mAh) {
  uint16_t raw = 0;
  if (!readReg(reg, raw)) return false;
  mAh = (float)raw * capLsbMah_;
  return true;
}

bool ModelGaugeM5::readSignedCapacity(uint8_t reg, float& mAh) {
  int16_t raw = 0;
  if (!readS16(reg, raw)) return false;
  mAh = (float)raw * capLsbMah_;
  return true;
}

bool ModelGaugeM5::readVoltage(uint8_t reg, float& volts) {
  uint16_t raw = 0;
  if (!readReg(reg, raw)) return false;
  volts = (float)raw * Scale::VOLTAGE_V_PER_LSB;
  return true;
}

bool ModelGaugeM5::readCurrentReg(uint8_t reg, float& mA) {
  int16_t raw = 0;
  if (!readS16(reg, raw)) return false;
  mA = (float)raw * curLsbMa_;
  return true;
}

bool ModelGaugeM5::readTemperatureReg(uint8_t reg, float& degC) {
  int16_t raw = 0;
  if (!readS16(reg, raw)) return false;
  degC = (float)raw * Scale::TEMP_C_PER_LSB;
  return true;
}

bool ModelGaugeM5::readTime(uint8_t reg, float& seconds) {
  uint16_t raw = 0;
  if (!readReg(reg, raw)) return false;
  seconds = (raw == 0xFFFF) ? NAN : (float)raw * Scale::TIME_S_PER_LSB;
  return true;
}

bool ModelGaugeM5::readPowerReg(uint8_t reg, float& watts) {
  int16_t raw = 0;
  if (!readS16(reg, raw)) return false;
  // 8 uV^2 / Rsense per count, multiplied back by the gauge-side divisor
  const float lsbW = Scale::POWER_V2_PER_LSB / (cfg_.senseResistorMohm / 1000.0f) * scale_;
  watts = (float)raw * lsbW;
  return true;
}

// ===========================================================================
// Public measurements
// ===========================================================================

bool ModelGaugeM5::readStatus(uint16_t& s) { return readReg(Reg::STATUS, s); }
bool ModelGaugeM5::readFStat(uint16_t& s) { return readReg(Reg::FSTAT, s); }

bool ModelGaugeM5::readVCell(float& v) { return readVoltage(Reg::VCELL, v); }
bool ModelGaugeM5::readAvgVCell(float& v) { return readVoltage(Reg::AVGVCELL, v); }
bool ModelGaugeM5::readVfOcv(float& v) { return readVoltage(Reg::VFOCV, v); }

bool ModelGaugeM5::readCurrent(float& mA) { return readCurrentReg(Reg::CURRENT, mA); }
bool ModelGaugeM5::readAvgCurrent(float& mA) { return readCurrentReg(Reg::AVGCURRENT, mA); }

bool ModelGaugeM5::readSoc(float& p) { return readPercent(Reg::REPSOC, p); }
bool ModelGaugeM5::readAvSoc(float& p) { return readPercent(Reg::AVSOC, p); }
bool ModelGaugeM5::readMixSoc(float& p) { return readPercent(Reg::MIXSOC, p); }
bool ModelGaugeM5::readVfSoc(float& p) { return readPercent(Reg::VFSOC, p); }

bool ModelGaugeM5::readRepCap(float& mAh) { return readCapacity(Reg::REPCAP, mAh); }
bool ModelGaugeM5::readFullCapRep(float& mAh) { return readCapacity(Reg::FULLCAPREP, mAh); }
bool ModelGaugeM5::readFullCapNom(float& mAh) { return readCapacity(Reg::FULLCAPNOM, mAh); }
bool ModelGaugeM5::readFullCap(float& mAh) { return readCapacity(Reg::FULLCAP, mAh); }
bool ModelGaugeM5::readAvCap(float& mAh) { return readCapacity(Reg::AVCAP, mAh); }
bool ModelGaugeM5::readMixCap(float& mAh) { return readCapacity(Reg::MIXCAP, mAh); }
bool ModelGaugeM5::readQResidual(float& mAh) { return readCapacity(Reg::QRESIDUAL, mAh); }
bool ModelGaugeM5::readCoulombCounter(float& mAh) { return readSignedCapacity(Reg::QH, mAh); }

bool ModelGaugeM5::readTemperature(float& c) { return readTemperatureReg(Reg::TEMP, c); }
bool ModelGaugeM5::readAvgTemperature(float& c) { return readTemperatureReg(Reg::AVGTA, c); }
bool ModelGaugeM5::readDieTemperature(float& c) { return readTemperatureReg(Reg::DIETEMP, c); }

bool ModelGaugeM5::readTimeToEmpty(float& s) { return readTime(Reg::TTE, s); }
bool ModelGaugeM5::readTimeToFull(float& s) { return readTime(Reg::TTF, s); }

bool ModelGaugeM5::readAge(float& pct) { return readPercent(Reg::AGE, pct); }

bool ModelGaugeM5::readCycles(float& cycles) {
  uint16_t raw = 0;
  if (!readReg(Reg::CYCLES, raw)) return false;
  cycles = (float)raw * Scale::CYCLES_PER_LSB;
  return true;
}

bool ModelGaugeM5::readRCell(float& mOhm) {
  uint16_t raw = 0;
  if (!readReg(Reg::RCELL, raw)) return false;
  mOhm = (float)raw * Scale::RESISTANCE_OHM_PER_LSB * 1000.0f;
  return true;
}

bool ModelGaugeM5::readPower(float& w) { return readPowerReg(Reg::POWER, w); }
bool ModelGaugeM5::readAvgPower(float& w) { return readPowerReg(Reg::AVGPOWER, w); }

bool ModelGaugeM5::readTimeSincePor(float& seconds) {
  uint16_t lo = 0, hi = 0;
  if (!readReg(Reg::TIMER, lo) || !readReg(Reg::TIMERH, hi)) return false;
  const uint32_t raw = ((uint32_t)hi << 16) | lo;
  seconds = (float)raw * Scale::TIMER_S_PER_LSB;
  return true;
}

bool ModelGaugeM5::readMaxMinVoltage(MinMax& mm) {
  uint16_t raw = 0;
  if (!readReg(Reg::MAXMINVOLT, raw)) return false;
  mm.max = (float)(raw >> 8) * Scale::VALRT_V_PER_LSB;
  mm.min = (float)(raw & 0xFF) * Scale::VALRT_V_PER_LSB;
  return true;
}

bool ModelGaugeM5::readMaxMinCurrent(MinMax& mm) {
  uint16_t raw = 0;
  if (!readReg(Reg::MAXMINCURR, raw)) return false;
  const float lsbMa = Scale::SENSE_V_PER_COUNT_IALRT * 1000.0f /
                      (cfg_.senseResistorMohm / 1000.0f) * scale_;
  mm.max = (float)(int8_t)(raw >> 8) * lsbMa;
  mm.min = (float)(int8_t)(raw & 0xFF) * lsbMa;
  return true;
}

bool ModelGaugeM5::readMaxMinTemperature(MinMax& mm) {
  uint16_t raw = 0;
  if (!readReg(Reg::MAXMINTEMP, raw)) return false;
  mm.max = (float)(int8_t)(raw >> 8);
  mm.min = (float)(int8_t)(raw & 0xFF);
  return true;
}

bool ModelGaugeM5::resetMaxMin() {
  return writeReg(Reg::MAXMINVOLT, 0x00FF) && writeReg(Reg::MAXMINCURR, 0x807F) &&
         writeReg(Reg::MAXMINTEMP, 0x807F);
}

bool ModelGaugeM5::readSnapshot(Snapshot& s) {
  s.vCellV = s.avgVCellV = s.vfOcvV = NAN;
  s.currentMa = s.avgCurrentMa = NAN;
  s.socPct = s.vfSocPct = NAN;
  s.repCapMah = s.fullCapRepMah = s.fullCapNomMah = NAN;
  s.tempC = s.rCellMohm = NAN;
  s.tteSeconds = s.ttfSeconds = NAN;
  s.cycles = s.agePct = NAN;
  s.status = 0;
  s.fstat = 0;

  bool essential = true;
  essential &= readVCell(s.vCellV);
  essential &= readCurrent(s.currentMa);
  essential &= readSoc(s.socPct);
  essential &= readRepCap(s.repCapMah);

  readAvgVCell(s.avgVCellV);
  readVfOcv(s.vfOcvV);
  readAvgCurrent(s.avgCurrentMa);
  readVfSoc(s.vfSocPct);
  readFullCapRep(s.fullCapRepMah);
  readFullCapNom(s.fullCapNomMah);
  readTemperature(s.tempC);
  readRCell(s.rCellMohm);
  readTimeToEmpty(s.tteSeconds);
  readTimeToFull(s.ttfSeconds);
  readCycles(s.cycles);
  readAge(s.agePct);
  readStatus(s.status);
  readFStat(s.fstat);
  s.ok = essential;
  return essential;
}

// ===========================================================================
// Current calibration
// ===========================================================================

ModelGaugeM5::CurrentCalibration ModelGaugeM5::calibration() const { return cal_; }

ModelGaugeM5::Status ModelGaugeM5::setCalibration(const CurrentCalibration& cal) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (!(cal.gainTrim > 0.1f && cal.gainTrim < 10.0f)) return Status::InvalidConfig;

  const long cg = roundToLong(1024.0f * cal.gainTrim / (float)plannedScale_);
  if (cg < 1 || cg > 0xFFFF) return Status::OutOfRange;

  const CurrentCalibration oldCal = cal_;
  const uint16_t oldCgain = cgain_;
  cal_ = cal;
  cgain_ = (uint16_t)cg;
  recomputeUnits();
  s = writeCalibrationRegisters(false);
  if (s != Status::Ok) {
    cal_ = oldCal;
    cgain_ = oldCgain;
    recomputeUnits();
    return s;
  }
  logf("calibration: trim %.4f offset %d -> CGain 0x%04X", (double)cal_.gainTrim,
       (int)cal_.offsetCounts, (unsigned)cgain_);
  return Status::Ok;
}

ModelGaugeM5::Status ModelGaugeM5::calibrateGain(float reportedMa, float trueMa) {
  if (reportedMa == 0.0f || trueMa == 0.0f || (reportedMa > 0.0f) != (trueMa > 0.0f)) {
    return Status::InvalidConfig;
  }
  const float ratio = trueMa / reportedMa;
  if (!(ratio > 0.5f && ratio < 2.0f)) return Status::OutOfRange;
  CurrentCalibration n = cal_;
  n.gainTrim = cal_.gainTrim * ratio;
  return setCalibration(n);
}

ModelGaugeM5::Status ModelGaugeM5::calibrateOffset(float reportedMaAtZero) {
  const long deltaCounts = roundToLong(reportedMaAtZero / curLsbMa_);
  const long n = (long)cal_.offsetCounts - deltaCounts;
  if (n < -32768 || n > 32767) return Status::OutOfRange;
  CurrentCalibration c = cal_;
  c.offsetCounts = (int16_t)n;
  return setCalibration(c);
}

ModelGaugeM5::Status ModelGaugeM5::calibrateTwoPoint(float r1, float t1, float r2, float t2) {
  const float dr = r2 - r1;
  if (fabsf(dr) < 2.0f * curLsbMa_) return Status::InvalidConfig;  // points too close together
  const float a = (t2 - t1) / dr;
  if (!(a > 0.5f && a < 2.0f)) return Status::OutOfRange;
  const float b = t1 - a * r1;  // mA, true = a * reported + b

  // reg' = a * reg + b / L  (L = mA per count, unchanged by the gain change)
  const float newOffset = a * (float)cal_.offsetCounts + b / curLsbMa_;
  const long off = roundToLong(newOffset);
  if (off < -32768 || off > 32767) return Status::OutOfRange;

  CurrentCalibration c;
  c.gainTrim = cal_.gainTrim * a;
  c.offsetCounts = (int16_t)off;
  return setCalibration(c);
}

// ===========================================================================
// Temperature
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::setTemperatureSource(TempSource source) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (source == TempSource::Thermistor) {
    // TSel = 1 needs ETHRM = 1; Ten must be set too (datasheet).
    return modifyRegister(Reg::CONFIG, 0, ConfigBit::TSEL | ConfigBit::ETHRM | ConfigBit::TEN);
  }
  return modifyRegister(Reg::CONFIG, ConfigBit::TSEL, 0);
}

ModelGaugeM5::Status ModelGaugeM5::setThermistor(const Thermistor& ntc) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (!writeReg(Reg::TGAIN, ntc.tGain)) return Status::I2cError;
  if (!writeReg(Reg::TOFF, ntc.tOff)) return Status::I2cError;
  // The Curve high byte belongs to copper-trace sensing - leave it alone.
  return modifyRegister(Reg::CURVE, 0x00FF, (uint16_t)(ntc.tCurve & 0x00FF));
}

ModelGaugeM5::Status ModelGaugeM5::setForceThermistorBias(bool force) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return modifyRegister(Reg::CONFIG, ConfigBit::FTHRM, force ? ConfigBit::FTHRM : 0);
}

ModelGaugeM5::Status ModelGaugeM5::setCopperTraceSensing(bool enable) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  cfg_.copperTraceSensing = enable;
  return writeReg(Reg::CGTEMPCO, enable ? (uint16_t)0x20C8 : (uint16_t)0x0000) ? Status::Ok
                                                                                  : Status::I2cError;
}

ModelGaugeM5::Status ModelGaugeM5::setCopperTraceCurve(uint8_t mCurve) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return modifyRegister(Reg::CURVE, 0xFF00, (uint16_t)((uint16_t)mCurve << 8));
}

// ===========================================================================
// Alerts
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::configureAlerts(const AlertConfig& a) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;

  uint16_t vTh = 0xFF00, tTh = 0x7F80, sTh = 0xFF00, iTh = 0x7F80;  // power-up = disabled

  if (a.voltage) {
    if (a.vMinV > a.vMaxV) return Status::InvalidConfig;
    const uint8_t hi = clampU8(roundToLong(a.vMaxV / Scale::VALRT_V_PER_LSB));
    const uint8_t lo = clampU8(roundToLong(a.vMinV / Scale::VALRT_V_PER_LSB));
    vTh = (uint16_t)(((uint16_t)hi << 8) | lo);
  }
  if (a.temperature) {
    if (a.tMinC > a.tMaxC) return Status::InvalidConfig;
    const int8_t hi = clampS8(roundToLong(a.tMaxC));
    const int8_t lo = clampS8(roundToLong(a.tMinC));
    tTh = (uint16_t)(((uint16_t)(uint8_t)hi << 8) | (uint8_t)lo);
  }
  if (a.soc) {
    if (a.sMinPct > a.sMaxPct) return Status::InvalidConfig;
    const uint8_t hi = clampU8(roundToLong(a.sMaxPct));
    const uint8_t lo = clampU8(roundToLong(a.sMinPct));
    sTh = (uint16_t)(((uint16_t)hi << 8) | lo);
  }
  if (a.current) {
    if (a.iMinMa > a.iMaxMa) return Status::InvalidConfig;
    const float lsbMa = Scale::SENSE_V_PER_COUNT_IALRT * 1000.0f /
                        (cfg_.senseResistorMohm / 1000.0f) * scale_;
    const int8_t hi = clampS8(roundToLong(a.iMaxMa / lsbMa));
    const int8_t lo = clampS8(roundToLong(a.iMinMa / lsbMa));
    iTh = (uint16_t)(((uint16_t)(uint8_t)hi << 8) | (uint8_t)lo);
  }

  // Thresholds first, then enable (datasheet: avoids false interrupts).
  if (!writeReg(Reg::VALRTTH, vTh)) return Status::I2cError;
  if (!writeReg(Reg::TALRTTH, tTh)) return Status::I2cError;
  if (!writeReg(Reg::SALRTTH, sTh)) return Status::I2cError;
  if (!writeReg(Reg::IALRTTH, iTh)) return Status::I2cError;

  s = modifyRegister(Reg::STATUS, StatusBit::ALL_ALERTS, 0);  // drop stale latched flags
  if (s != Status::Ok) return s;

  const bool any = a.voltage || a.temperature || a.soc || a.current || a.socChange;
  const uint16_t stickyBits = ConfigBit::SS | ConfigBit::TS | ConfigBit::VS | ConfigBit::IS;
  s = modifyRegister(Reg::CONFIG, (uint16_t)(stickyBits | ConfigBit::AEN),
                     (uint16_t)((a.sticky ? stickyBits : 0) | (any ? ConfigBit::AEN : 0)));
  if (s != Status::Ok) return s;

  return modifyRegister(Reg::CONFIG2, Config2Bit::DSOCEN, a.socChange ? Config2Bit::DSOCEN : 0);
}

ModelGaugeM5::Status ModelGaugeM5::disableAlerts() {
  AlertConfig none;
  return configureAlerts(none);
}

uint16_t ModelGaugeM5::alertFlags() {
  uint16_t st = 0;
  if (!readReg(Reg::STATUS, st)) return 0;
  return (uint16_t)(st & StatusBit::ALL_ALERTS);
}

ModelGaugeM5::Status ModelGaugeM5::clearAlerts(uint16_t flags) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  // Read-modify-write: an alert that latches between the read and the write
  // is lost. That is inherent to the register and the same as Maxim's code.
  return modifyRegister(Reg::STATUS, (uint16_t)(flags & StatusBit::ALL_ALERTS), 0);
}

ModelGaugeM5::Status ModelGaugeM5::setSocAlertSource(uint8_t sacfg) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (sacfg > 3) return Status::InvalidConfig;
  return modifyRegister(Reg::MISCCFG, MiscCfgBit::SACFG_MASK, sacfg);
}

// ===========================================================================
// Power modes
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::setHibernateEnabled(bool enable) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return modifyRegister(Reg::HIBCFG, HibCfgBit::ENHIB, enable ? HibCfgBit::ENHIB : 0);
}

ModelGaugeM5::Status ModelGaugeM5::wake(uint16_t* savedHibCfg) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  uint16_t saved = 0;
  s = wakeForInit(saved);
  if (savedHibCfg) *savedHibCfg = saved;
  return s;
}

ModelGaugeM5::Status ModelGaugeM5::restoreHibCfg(uint16_t savedHibCfg) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return writeReg(Reg::HIBCFG, savedHibCfg) ? Status::Ok : Status::I2cError;
}

bool ModelGaugeM5::isHibernating(bool& hibernating) {
  uint16_t v = 0;
  if (!readReg(Reg::STATUS2, v)) return false;
  hibernating = (v & Status2Bit::HIB) != 0;
  return true;
}

ModelGaugeM5::Status ModelGaugeM5::shutdown(bool fast) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  // Datasheet: to command shutdown within the 22.5 s window, enter active mode
  // first (HibCfg = 0) and shorten ShdnTimer.
  if (!writeReg(Reg::HIBCFG, 0x0000)) return Status::I2cError;
  if (fast && !writeReg(Reg::SHDNTIMER, 0x001E)) return Status::I2cError;
  return modifyRegister(Reg::CONFIG, 0, ConfigBit::SHDN);
}

// ===========================================================================
// Learning / tuning
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::readLearnedParams(LearnedParams& out) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  LearnedParams p;
  if (!readReg(Reg::RCOMP0, p.rcomp0) || !readReg(Reg::TEMPCO, p.tempCo) ||
      !readReg(Reg::FULLCAPREP, p.fullCapRep) || !readReg(Reg::FULLCAPNOM, p.fullCapNom) ||
      !readReg(Reg::CYCLES, p.cycles)) {
    return Status::I2cError;
  }
  p.cgain = cgain_;
  p.valid = true;
  out = p;
  return Status::Ok;
}

bool ModelGaugeM5::learnedParamsDue() {
  uint16_t cycles = 0;
  if (!readReg(Reg::CYCLES, cycles)) return false;
  const bool bit6 = (cycles & 0x0040) != 0;
  if (!lastCyclesBit6Valid_) {
    lastCyclesBit6Valid_ = true;
    lastCyclesBit6_ = bit6;
    return false;
  }
  const bool toggled = (bit6 != lastCyclesBit6_);
  lastCyclesBit6_ = bit6;
  return toggled;
}

ModelGaugeM5::Status ModelGaugeM5::setLearnStage(uint8_t stage) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (stage > 7) return Status::InvalidConfig;
  return modifyRegister(Reg::LEARNCFG, LearnCfgBit::LS_MASK,
                        (uint16_t)((uint16_t)stage << LearnCfgBit::LS_SHIFT));
}

ModelGaugeM5::Status ModelGaugeM5::setFullSocThreshold(uint8_t percent) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (percent > 100) return Status::InvalidConfig;
  return writeAndVerifyRegister(Reg::FULLSOCTHR, (uint16_t)(((uint16_t)percent << 8) | 0x05));
}

ModelGaugeM5::Status ModelGaugeM5::setRelaxLoadThresholdMa(float mA) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (!(mA >= 0.0f)) return Status::InvalidConfig;
  // The gauge compares the voltage it sees across the shunt, which the
  // scaling has divided by scale_. LOAD is 7 bits at 50 uV/LSB (bits 15:9).
  const float gaugeUv = mA / 1000.0f * (cfg_.senseResistorMohm / 1000.0f) / scale_ * 1.0e6f;
  const long counts = roundToLong(gaugeUv / 50.0f);
  if (counts > 127) return Status::OutOfRange;
  return modifyRegister(Reg::RELAXCFG, 0xFE00, (uint16_t)((uint16_t)counts << 9));
}

ModelGaugeM5::Status ModelGaugeM5::setSocHold(bool hold99Percent, float emptyVoltHoldV,
                                              float emptySocHoldPct) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  const long vBits = roundToLong(emptyVoltHoldV / 0.010f);   // 10 mV, 0..1.27 V
  const long sBits = roundToLong(emptySocHoldPct / 0.5f);    // 0.5 %, 0..15.5 %
  if (vBits < 0 || vBits > 127 || sBits < 0 || sBits > 31) return Status::OutOfRange;
  const uint16_t value = (uint16_t)((hold99Percent ? 0x1000 : 0) | (vBits << 5) | sBits);
  return writeReg(Reg::SOCHOLD, value) ? Status::Ok : Status::I2cError;
}

ModelGaugeM5::Status ModelGaugeM5::setLiFePO4OcvKeepOut(float lowV, float highV) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (!(highV >= lowV)) return Status::InvalidConfig;
  const long low = roundToLong((lowV - 2.56f) / 0.005f);    // 9 bits, 5 mV from 2.56 V
  const long delta = roundToLong((highV - lowV) / 0.0025f); // 7 bits, 2.5 mV
  if (low < 0 || low > 511 || delta < 0 || delta > 127) return Status::OutOfRange;
  return writeReg(Reg::SCOCVLIM, (uint16_t)((low << 7) | delta)) ? Status::Ok : Status::I2cError;
}

// ===========================================================================
// AtRate / dynamic power
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::enableAtRate(bool enable) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return modifyRegister(Reg::CONFIG2, Config2Bit::ATRATEEN, enable ? Config2Bit::ATRATEEN : 0);
}

ModelGaugeM5::Status ModelGaugeM5::enableDynamicPower(bool enable) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return modifyRegister(Reg::CONFIG2, Config2Bit::DPEN, enable ? Config2Bit::DPEN : 0);
}

ModelGaugeM5::Status ModelGaugeM5::setConstantPowerMode(bool enable) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return modifyRegister(Reg::CONFIG2, Config2Bit::CPMODE, enable ? Config2Bit::CPMODE : 0);
}

ModelGaugeM5::Status ModelGaugeM5::setAtRate(float dischargeMa) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  const float mag = fabsf(dischargeMa);
  long counts = roundToLong(-mag / curLsbMa_);  // AtRate must be negative (discharge)
  if (counts < -32768) return Status::OutOfRange;
  return writeReg(Reg::ATRATE, (uint16_t)(int16_t)counts) ? Status::Ok : Status::I2cError;
}

bool ModelGaugeM5::readAtTimeToEmpty(float& s) { return readTime(Reg::ATTTE, s); }
bool ModelGaugeM5::readAtSoc(float& p) { return readPercent(Reg::ATAVSOC, p); }
bool ModelGaugeM5::readAtCapacity(float& mAh) { return readCapacity(Reg::ATAVCAP, mAh); }
bool ModelGaugeM5::readAtQResidual(float& mAh) { return readCapacity(Reg::ATQRESIDUAL, mAh); }

bool ModelGaugeM5::readMaxPeakPowerMw(float& mW) {
  int16_t raw = 0;
  if (!readS16(Reg::MAXPEAKPOWER, raw)) return false;
  mW = (float)raw * Scale::PEAKPOWER_MW_PER_LSB;
  return true;
}

bool ModelGaugeM5::readSusPeakPowerMw(float& mW) {
  int16_t raw = 0;
  if (!readS16(Reg::SUSPEAKPOWER, raw)) return false;
  mW = (float)raw * Scale::PEAKPOWER_MW_PER_LSB;
  return true;
}

bool ModelGaugeM5::readMppCurrentRaw(int16_t& raw) { return readS16(Reg::MPPCURRENT, raw); }
bool ModelGaugeM5::readSppCurrentRaw(int16_t& raw) { return readS16(Reg::SPPCURRENT, raw); }

ModelGaugeM5::Status ModelGaugeM5::setPackResistanceMohm(float mOhm) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (!(mOhm >= 0.0f)) return Status::InvalidConfig;
  const long counts = roundToLong(mOhm / (Scale::PACKRES_OHM_PER_LSB * 1000.0f));
  if (counts > 65535) return Status::OutOfRange;
  return writeReg(Reg::PACKRESISTANCE, clampU16(counts)) ? Status::Ok : Status::I2cError;
}

ModelGaugeM5::Status ModelGaugeM5::setSysResistanceMohm(float mOhm) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  if (!(mOhm >= 0.0f)) return Status::InvalidConfig;
  const long counts = roundToLong(mOhm / (Scale::PACKRES_OHM_PER_LSB * 1000.0f));
  if (counts > 65535) return Status::OutOfRange;
  return writeReg(Reg::SYSRESISTANCE, clampU16(counts)) ? Status::Ok : Status::I2cError;
}

ModelGaugeM5::Status ModelGaugeM5::setMinSysVoltageRaw(uint16_t raw) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return writeReg(Reg::MINSYSVOLTAGE, raw) ? Status::Ok : Status::I2cError;
}

// ===========================================================================
// Serial number
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::readSerialNumber(uint16_t words[8]) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;

  uint16_t cfg2 = 0;
  if (!readReg(Reg::CONFIG2, cfg2)) return Status::I2cError;
  const uint16_t both = Config2Bit::ATRATEEN | Config2Bit::DPEN;
  if (!writeReg(Reg::CONFIG2, (uint16_t)(cfg2 & ~both))) return Status::I2cError;

  // Wait for Status2.SNReady
  Status result = Status::Timeout;
  const uint32_t start = millis();
  while (millis() - start < cfg_.timeoutMs) {
    uint16_t st2 = 0;
    if (readReg(Reg::STATUS2, st2) && (st2 & Status2Bit::SN_READY)) {
      static const uint8_t addrs[8] = {0xD4, 0xD5, 0xD9, 0xDA, 0xDC, 0xDD, 0xDE, 0xDF};
      result = Status::Ok;
      for (uint8_t i = 0; i < 8; ++i) {
        if (!readReg(addrs[i], words[i])) {
          result = Status::I2cError;
          break;
        }
      }
      break;
    }
    delay(10);
  }

  // Put AtRate / dynamic power back the way they were.
  writeReg(Reg::CONFIG2, cfg2);
  return result;
}
