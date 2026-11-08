// ModelGaugeM5.cpp - core: I2C access, scaling plan, initialization sequences.
// Licensed under the MIT License (see LICENSE).

#include "ModelGaugeM5.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

using namespace m5g;

namespace {

long roundToLong(float x) { return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f); }

}  // namespace

// ===========================================================================
// Construction / strings
// ===========================================================================

ModelGaugeM5::ModelGaugeM5(Variant v) : variant_(v) {
  if (v == Variant::Max17055) {
    expectedDevName_   = 0x4010;
    defaultAddr_       = 0x36;
    isHighSideCapable_ = false;
    writesDqAccDpAcc_  = true;   // MAX17055 EZ sequence also seeds dQAcc / dPAcc
    hasR100_           = false;
  } else {
    expectedDevName_   = 0x4031;
    defaultAddr_       = 0x36;
    isHighSideCapable_ = true;
    writesDqAccDpAcc_  = false;  // the MAX1726x EZ sequence does not
    hasR100_           = true;
  }
}

const char* ModelGaugeM5::statusToString(Status s) {
  switch (s) {
    case Status::Ok:              return "ok";
    case Status::I2cError:        return "I2C error";
    case Status::NotFound:        return "no device at address";
    case Status::WrongDevice:     return "unexpected DevName";
    case Status::InvalidConfig:   return "invalid configuration";
    case Status::OutOfRange:      return "value out of range";
    case Status::Timeout:         return "timeout";
    case Status::VerifyFailed:    return "write verify failed";
    case Status::ModelLoadFailed: return "model load failed";
    case Status::NotInitialized:  return "begin() not called";
    case Status::Unsupported:     return "not supported on this IC";
  }
  return "?";
}

void ModelGaugeM5::logf(const char* fmt, ...) {
  if (!cfg_.debug) return;
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  cfg_.debug->print("[gauge] ");
  cfg_.debug->print(buf);
  cfg_.debug->print("\n");
}

// ===========================================================================
// Low level I2C
// ===========================================================================

bool ModelGaugeM5::readReg(uint8_t reg, uint16_t& v) {
  if (!wire_) return false;
  for (uint8_t attempt = 0; attempt <= cfg_.i2cRetries; ++attempt) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) == 0 && wire_->requestFrom((int)addr_, 2) == 2) {
      uint8_t lo = wire_->read();
      uint8_t hi = wire_->read();
      v = (uint16_t)(((uint16_t)hi << 8) | lo);
      return true;
    }
    delay(1);
  }
  return false;
}

bool ModelGaugeM5::writeReg(uint8_t reg, uint16_t v) {
  if (!wire_) return false;
  for (uint8_t attempt = 0; attempt <= cfg_.i2cRetries; ++attempt) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write((uint8_t)(v & 0xFF));
    wire_->write((uint8_t)(v >> 8));
    if (wire_->endTransmission() == 0) return true;
    delay(1);
  }
  return false;
}

ModelGaugeM5::Status ModelGaugeM5::writeAndVerifyRegister(uint8_t reg, uint16_t value,
                                                          uint8_t attempts, uint16_t compareMask) {
  // Implementation guide "WriteAndVerifyRegister": write, wait 1 ms, read back.
  for (uint8_t a = 0; a < attempts; ++a) {
    if (!writeReg(reg, value)) return Status::I2cError;
    delay(1);
    uint16_t rb = 0;
    if (readReg(reg, rb) && (((rb ^ value) & compareMask) == 0)) return Status::Ok;
  }
  logf("verify failed reg 0x%02X want 0x%04X", reg, value);
  return Status::VerifyFailed;
}

ModelGaugeM5::Status ModelGaugeM5::modifyRegister(uint8_t reg, uint16_t clearMask, uint16_t setMask) {
  uint16_t v = 0;
  if (!readReg(reg, v)) return Status::I2cError;
  uint16_t nv = (uint16_t)((v & ~clearMask) | setMask);
  if (nv == v) return Status::Ok;
  return writeReg(reg, nv) ? Status::Ok : Status::I2cError;
}

bool ModelGaugeM5::findAddress(TwoWire& wire, uint16_t expectedDevName, uint8_t& foundAddress) {
  static const uint8_t candidates[] = {0x36, 0x0D, 0x1A};
  for (uint8_t i = 0; i < sizeof(candidates); ++i) {
    wire.beginTransmission(candidates[i]);
    wire.write((uint8_t)Reg::DEVNAME);
    if (wire.endTransmission(false) != 0) continue;
    if (wire.requestFrom((int)candidates[i], 2) != 2) continue;
    uint8_t lo = wire.read();
    uint8_t hi = wire.read();
    if ((uint16_t)(((uint16_t)hi << 8) | lo) == expectedDevName) {
      foundAddress = candidates[i];
      return true;
    }
  }
  return false;
}

// ===========================================================================
// Scaling plan
// ===========================================================================

uint8_t ModelGaugeM5::planScale(float senseResistorMohm, float designCapacityMah,
                                float peakCurrentMa, uint16_t maxCapacityCounts) {
  if (!(senseResistorMohm > 0.0f) || !(designCapacityMah > 0.0f)) return 0;
  const float rOhm = senseResistorMohm / 1000.0f;
  static const uint8_t candidates[] = {1, 2, 4, 8};
  for (uint8_t i = 0; i < sizeof(candidates); ++i) {
    const float n = (float)candidates[i];
    const float capLsbMah = Scale::SENSE_V_PER_COUNT_CAP * 1000.0f / rOhm * n;
    const float rangeMa   = Scale::SENSE_FULLSCALE_V * 1000.0f / rOhm * n;
    const bool capOk  = (designCapacityMah / capLsbMah) <= (float)maxCapacityCounts;
    const bool curOk  = (peakCurrentMa <= 0.0f) || (peakCurrentMa <= 0.9f * rangeMa);
    if (capOk && curOk) return candidates[i];
  }
  return 0;
}

void ModelGaugeM5::recomputeUnits() {
  const float rOhm = cfg_.senseResistorMohm / 1000.0f;
  scale_     = cal_.gainTrim * 1024.0f / (float)cgain_;
  capLsbMah_ = Scale::SENSE_V_PER_COUNT_CAP * 1000.0f / rOhm * scale_;
  curLsbMa_  = Scale::SENSE_V_PER_COUNT * 1000.0f / rOhm * scale_;
}

ModelGaugeM5::Status ModelGaugeM5::plan() {
  const Config& c = cfg_;

  if (!(c.designCapacityMah > 0.0f)) {
    logf("designCapacityMah must be set");
    return Status::InvalidConfig;
  }
  if (!(c.senseResistorMohm > 0.0f)) {
    logf("senseResistorMohm must be > 0");
    return Status::InvalidConfig;
  }
  if (!(c.gainTrim > 0.1f && c.gainTrim < 10.0f)) {
    logf("gainTrim %.3f is not plausible", (double)c.gainTrim);
    return Status::InvalidConfig;
  }
  if (!(c.emptyVoltageV > 0.0f && c.emptyVoltageV <= 5.11f) ||
      !(c.recoveryVoltageV > c.emptyVoltageV && c.recoveryVoltageV <= 5.08f)) {
    logf("need 0 < empty < recovery <= 5.08 V");
    return Status::InvalidConfig;
  }
  if (!(c.chargeVoltageV > 2.0f && c.chargeVoltageV < 5.0f)) {
    logf("chargeVoltageV out of range");
    return Status::InvalidConfig;
  }
  if (c.fullSocThrPercent > 100) {
    logf("fullSocThrPercent > 100");
    return Status::InvalidConfig;
  }
  const uint8_t rawId = (uint8_t)c.modelId;
  if (rawId > 15) {
    logf("ModelID must fit in 4 bits");
    return Status::InvalidConfig;
  }
  if (variant_ == Variant::Max17055 && c.modelId == ModelID::NcrNca) {
    logf("note: ModelID 2 is only documented for MAX1726x, not the MAX17055");
  }
  if (!hasR100_ && c.thermistor100k) {
    logf("thermistor100k is MAX17260-only; ignored");
  }

  // ---- choose the gauge-side divisor N ----
  uint8_t n = c.scale;
  if (n == 0) {
    n = planScale(c.senseResistorMohm, c.designCapacityMah, c.peakCurrentMa, c.maxCapacityCounts);
    if (n == 0) {
      logf("no scale 1/2/4/8 fits this shunt/capacity/current; use a bigger/smaller shunt");
      return Status::OutOfRange;
    }
  } else if (n != 1 && n != 2 && n != 4 && n != 8) {
    logf("scale must be 0 (auto), 1, 2, 4 or 8");
    return Status::InvalidConfig;
  }
  plannedScale_ = n;

  // ---- CGain ----
  const long cg = roundToLong(1024.0f * c.gainTrim / (float)n);
  if (cg < 1 || cg > 0xFFFF) return Status::OutOfRange;
  cgain_ = (uint16_t)cg;
  cal_.gainTrim = c.gainTrim;
  cal_.offsetCounts = c.currentOffsetCounts;
  recomputeUnits();

  // ---- capacity / current registers ----
  const float capCounts = c.designCapacityMah / capLsbMah_;
  if (capCounts < 1.0f || capCounts > 65535.0f) {
    logf("DesignCap needs %.0f counts (max 65535)", (double)capCounts);
    return Status::OutOfRange;
  }
  if (capCounts > (float)c.maxCapacityCounts) {
    logf("warning: DesignCap = %.0f counts exceeds maxCapacityCounts %u",
         (double)capCounts, (unsigned)c.maxCapacityCounts);
  }
  designCapCounts_ = (uint16_t)roundToLong(capCounts);

  const float rangeMa = currentRangeMa();
  if (c.peakCurrentMa > 0.0f && c.peakCurrentMa > 0.9f * rangeMa) {
    logf("warning: peak current %.0f mA is above 90%% of the +-%.0f mA range",
         (double)c.peakCurrentMa, (double)rangeMa);
  }

  const float termMa = (c.chargeTerminationMa > 0.0f) ? c.chargeTerminationMa
                                                      : c.designCapacityMah / 20.0f;
  if (c.chargeTerminationMa <= 0.0f) logf("chargeTerminationMa not set, using C/20 = %.1f mA", (double)termMa);
  const float termCounts = termMa / curLsbMa_;
  if (termCounts < 1.0f || termCounts > 32767.0f) {
    logf("IChgTerm = %.1f counts is not representable", (double)termCounts);
    return Status::OutOfRange;
  }
  if (termCounts < 4.0f) {
    logf("warning: IChgTerm is only %.1f counts - termination detection will be coarse",
         (double)termCounts);
  }
  ichgTermCounts_ = (uint16_t)roundToLong(termCounts);

  logf("scale N=%u CGain=0x%04X effScale=%.4f capLSB=%.4f mAh curLSB=%.5f mA", (unsigned)n,
       (unsigned)cgain_, (double)scale_, (double)capLsbMah_, (double)curLsbMa_);
  logf("DesignCap=0x%04X (%u) IChgTerm=0x%04X (%u)", (unsigned)designCapCounts_,
       (unsigned)designCapCounts_, (unsigned)ichgTermCounts_, (unsigned)ichgTermCounts_);
  return Status::Ok;
}

// ===========================================================================
// begin / init
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::begin(TwoWire& wire, const Config& config, uint8_t i2cAddress) {
  begun_ = false;
  ranInit_ = false;
  lastCyclesBit6Valid_ = false;
  wire_ = &wire;
  cfg_ = config;
  addr_ = i2cAddress ? i2cAddress : defaultAddr_;

  haveCustom_ = (config.customModel != nullptr);
  if (haveCustom_) {
    custom_ = *config.customModel;
    cfg_.customModel = &custom_;  // never keep the caller's pointer
  }

  Status s = plan();
  if (s != Status::Ok) return s;

  uint16_t dev = 0;
  if (!readReg(Reg::DEVNAME, dev)) {
    logf("no response at 0x%02X", (unsigned)addr_);
    return Status::NotFound;
  }
  if (!cfg_.skipDeviceCheck && dev != expectedDevName_) {
    logf("DevName 0x%04X, expected 0x%04X", (unsigned)dev, (unsigned)expectedDevName_);
    return Status::WrongDevice;
  }
  begun_ = true;

  bool por = false;
  s = checkPor(por);
  if (s != Status::Ok) return s;

  if (por || cfg_.forceInit) {
    logf("loading model (POR=%d force=%d)", por ? 1 : 0, cfg_.forceInit ? 1 : 0);
    s = runInit();
    if (s != Status::Ok) return s;
    ranInit_ = true;
  } else {
    logf("gauge kept its state, model not reloaded");
  }
  return applyAlwaysOnSettings();
}

ModelGaugeM5::Status ModelGaugeM5::readDevName(uint16_t& devName) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  return readReg(Reg::DEVNAME, devName) ? Status::Ok : Status::I2cError;
}

ModelGaugeM5::Status ModelGaugeM5::checkPor(bool& porSet) {
  uint16_t st = 0;
  if (!readReg(Reg::STATUS, st)) return Status::I2cError;
  porSet = (st & StatusBit::POR) != 0;
  return Status::Ok;
}

ModelGaugeM5::Status ModelGaugeM5::initialize() {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  s = runInit();
  if (s == Status::Ok) {
    ranInit_ = true;
    s = applyAlwaysOnSettings();
  }
  return s;
}

ModelGaugeM5::Status ModelGaugeM5::waitClear(uint8_t reg, uint16_t mask, const char* what) {
  const uint32_t start = millis();
  while (true) {
    uint16_t v = 0;
    if (!readReg(reg, v)) return Status::I2cError;
    if ((v & mask) == 0) return Status::Ok;
    if (millis() - start >= cfg_.timeoutMs) {
      logf("timeout waiting for %s", what);
      return Status::Timeout;
    }
    delay(10);
  }
}

ModelGaugeM5::Status ModelGaugeM5::waitDnr() {
  return waitClear(Reg::FSTAT, FStatBit::DNR, "FSTAT.DNR");
}

ModelGaugeM5::Status ModelGaugeM5::wakeForInit(uint16_t& savedHib) {
  if (!readReg(Reg::HIBCFG, savedHib)) return Status::I2cError;
  // Exit hibernate (implementation guide, step 2)
  if (!writeReg(Reg::COMMAND, Cmd::SOFT_WAKEUP)) return Status::I2cError;
  if (!writeReg(Reg::HIBCFG, 0x0000)) return Status::I2cError;
  if (!writeReg(Reg::COMMAND, Cmd::CLEAR)) return Status::I2cError;
  return Status::Ok;
}

ModelGaugeM5::Status ModelGaugeM5::clearPor() {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    uint16_t st = 0;
    if (!readReg(Reg::STATUS, st)) return Status::I2cError;
    if (!writeReg(Reg::STATUS, (uint16_t)(st & ~StatusBit::POR))) return Status::I2cError;
    delay(1);
    uint16_t rb = 0;
    if (readReg(Reg::STATUS, rb) && (rb & StatusBit::POR) == 0) return Status::Ok;
  }
  logf("could not clear Status.POR");
  return Status::VerifyFailed;
}

ModelGaugeM5::Status ModelGaugeM5::runInit() {
  // Step 1: wait for the gauge to finish its own start-up
  Status s = waitDnr();
  if (s != Status::Ok) return s;

  // Step 2: leave hibernate, load a model, restore HibCfg
  uint16_t savedHib = 0;
  s = wakeForInit(savedHib);
  if (s == Status::Ok) {
    if (haveCustom_) {
      s = custom_.fullModel ? loadFullCustomModel() : loadShortCustomModel();
    } else {
      s = loadEzModel();
    }
  }
  const bool hibOk = writeReg(Reg::HIBCFG, savedHib);  // always try to restore
  if (s != Status::Ok) return s;
  if (!hibOk) return Status::I2cError;

  // Step 3: tell the world the model is loaded
  s = clearPor();
  if (s != Status::Ok) return s;

  if (cfg_.quickStartAfterInit) s = quickStart(2000);
  return s;
}

uint16_t ModelGaugeM5::encodeVEmpty() const {
  long ve = roundToLong(cfg_.emptyVoltageV / 0.010f);
  long vr = roundToLong(cfg_.recoveryVoltageV / 0.040f);
  if (ve < 0) ve = 0;
  if (ve > 511) ve = 511;
  if (vr < 0) vr = 0;
  if (vr > 127) vr = 127;
  return (uint16_t)((ve << 7) | vr);
}

uint16_t ModelGaugeM5::modelCfgForEz() {
  uint16_t v = (uint16_t)(((uint16_t)cfg_.modelId << ModelCfgBit::MODELID_SHIFT) &
                          ModelCfgBit::MODELID_MASK);
  if (cfg_.chargeVoltageV > 4.275f) v |= ModelCfgBit::VCHG;
  if (hasR100_ && cfg_.thermistor100k) v |= ModelCfgBit::R100;
  return v;
}

ModelGaugeM5::Status ModelGaugeM5::writeModelCfgAndRefresh(uint16_t modelCfgValue) {
  uint16_t value = modelCfgValue;
  if (isHighSideCapable_) {
    // CSEL (high-side/low-side) is auto-detected at boot and the datasheet says
    // applications should not change it, so carry the current value over.
    uint16_t cur = 0;
    if (readReg(Reg::MODELCFG, cur)) {
      value = (uint16_t)((value & ~ModelCfgBit::CSEL) | (cur & ModelCfgBit::CSEL));
    }
  }
  value |= ModelCfgBit::REFRESH;
  if (!writeReg(Reg::MODELCFG, value)) return Status::I2cError;
  logf("ModelCfg <- 0x%04X", (unsigned)value);
  return waitClear(Reg::MODELCFG, ModelCfgBit::REFRESH, "ModelCfg.Refresh");
}

ModelGaugeM5::Status ModelGaugeM5::writeCalibrationRegisters(bool includeCopper) {
  Status s = writeAndVerifyRegister(Reg::CGAIN, cgain_);
  if (s != Status::Ok) return s;
  s = writeAndVerifyRegister(Reg::COFF, (uint16_t)cal_.offsetCounts);
  if (s != Status::Ok) return s;
  if (includeCopper) {
    // 0x20C8 = copper at 0.4 %/degC; 0 disables compensation for a normal shunt
    if (!writeReg(Reg::CGTEMPCO, cfg_.copperTraceSensing ? (uint16_t)0x20C8 : (uint16_t)0x0000)) {
      return Status::I2cError;
    }
  }
  return Status::Ok;
}

// ---------------------------------------------------------------------------
// EZ configuration (implementation guide Option 1)
// ---------------------------------------------------------------------------
ModelGaugeM5::Status ModelGaugeM5::loadEzModel() {
  // CGain/COff first, so every count the gauge integrates from now on is in
  // the units DesignCap and IChgTerm are written in.
  Status s = writeCalibrationRegisters(true);
  if (s != Status::Ok) return s;

  if (!writeReg(Reg::DESIGNCAP, designCapCounts_)) return Status::I2cError;
  uint16_t dQAcc = 0;
  if (writesDqAccDpAcc_) {
    dQAcc = (uint16_t)(designCapCounts_ / 32);
    if (!writeReg(Reg::DQACC, dQAcc)) return Status::I2cError;
  }
  if (!writeReg(Reg::ICHGTERM, ichgTermCounts_)) return Status::I2cError;
  if (!writeReg(Reg::VEMPTY, encodeVEmpty())) return Status::I2cError;
  if (writesDqAccDpAcc_) {
    const uint32_t k = (cfg_.chargeVoltageV > 4.275f) ? 51200UL : 44138UL;
    const uint16_t dPAcc = (uint16_t)(((uint32_t)dQAcc * k) / designCapCounts_);
    if (!writeReg(Reg::DPACC, dPAcc)) return Status::I2cError;
  }

  s = writeModelCfgAndRefresh(modelCfgForEz());
  if (s != Status::Ok) return s;

  // Model refresh must not be allowed to leave the current scaling behind.
  s = writeCalibrationRegisters(false);
  if (s != Status::Ok) return s;

  if (cfg_.fullSocThrPercent > 0) {
    s = writeAndVerifyRegister(Reg::FULLSOCTHR, (uint16_t)(((uint16_t)cfg_.fullSocThrPercent << 8) | 0x05));
    if (s != Status::Ok) return s;
  }
  return Status::Ok;
}

// ---------------------------------------------------------------------------
// Custom short format (Option 2)
// ---------------------------------------------------------------------------
ModelGaugeM5::Status ModelGaugeM5::loadShortCustomModel() {
  const CustomModel& m = custom_;
  Status s = writeCalibrationRegisters(true);
  if (s != Status::Ok) return s;

  if (!writeReg(Reg::DESIGNCAP, designCapCounts_)) return Status::I2cError;
  if (!writeReg(Reg::ICHGTERM, ichgTermCounts_)) return Status::I2cError;
  if (!writeReg(Reg::VEMPTY, m.vEmpty)) return Status::I2cError;
  if (m.optionalMask & OPT_LEARNCFG) {
    s = writeAndVerifyRegister(Reg::LEARNCFG, m.learnCfg);
    if (s != Status::Ok) return s;
  }
  if (m.optionalMask & OPT_FULLSOCTHR) {
    s = writeAndVerifyRegister(Reg::FULLSOCTHR, m.fullSocThr);
    if (s != Status::Ok) return s;
  }
  s = writeModelCfgAndRefresh(m.modelCfg);
  if (s != Status::Ok) return s;

  if (!writeReg(Reg::RCOMP0, m.rcomp0)) return Status::I2cError;
  if (!writeReg(Reg::TEMPCO, m.tempCo)) return Status::I2cError;
  if (!writeReg(Reg::QRTABLE00, m.qrTable00)) return Status::I2cError;
  if (!writeReg(Reg::QRTABLE10, m.qrTable10)) return Status::I2cError;
  if (m.optionalMask & OPT_QRTABLE2030) {
    if (!writeReg(Reg::QRTABLE20, m.qrTable20)) return Status::I2cError;
    if (!writeReg(Reg::QRTABLE30, m.qrTable30)) return Status::I2cError;
  }
  return writeCalibrationRegisters(false);
}

// ---------------------------------------------------------------------------
// Custom full format (Option 3): OCV + X tables
// ---------------------------------------------------------------------------
ModelGaugeM5::Status ModelGaugeM5::writeBlock(uint8_t first, const uint16_t* words, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    if (!writeReg((uint8_t)(first + i), words[i])) return Status::I2cError;
  }
  return Status::Ok;
}

bool ModelGaugeM5::blockMatches(uint8_t first, const uint16_t* words, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    uint16_t v = 0;
    if (!readReg((uint8_t)(first + i), v) || v != words[i]) return false;
  }
  return true;
}

bool ModelGaugeM5::blockIsZero(uint8_t first, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    uint16_t v = 0xFFFF;
    if (!readReg((uint8_t)(first + i), v) || v != 0) return false;
  }
  return true;
}

ModelGaugeM5::Status ModelGaugeM5::writeOptionalCustomRegisters(const CustomModel& m) {
  Status s = Status::Ok;
  if (m.optionalMask & OPT_LEARNCFG) {
    s = writeAndVerifyRegister(Reg::LEARNCFG, m.learnCfg);
    if (s != Status::Ok) return s;
  }
  if ((m.optionalMask & OPT_RELAXCFG) && !writeReg(Reg::RELAXCFG, m.relaxCfg)) return Status::I2cError;
  if ((m.optionalMask & OPT_CONFIG) && !writeReg(Reg::CONFIG, m.config)) return Status::I2cError;
  if ((m.optionalMask & OPT_CONFIG2) && !writeReg(Reg::CONFIG2, m.config2)) return Status::I2cError;
  if ((m.optionalMask & OPT_FULLSOCTHR) && !writeReg(Reg::FULLSOCTHR, m.fullSocThr)) return Status::I2cError;
  if (m.optionalMask & OPT_THERMISTOR) {
    if (!writeReg(Reg::TGAIN, m.tGain)) return Status::I2cError;
    if (!writeReg(Reg::TOFF, m.tOff)) return Status::I2cError;
    if (!writeReg(Reg::CURVE, m.curve)) return Status::I2cError;
  }
  if ((m.optionalMask & OPT_MISCCFG) && !writeReg(Reg::MISCCFG, m.miscCfg)) return Status::I2cError;
  return Status::Ok;
}

ModelGaugeM5::Status ModelGaugeM5::loadFullCustomModel() {
  const CustomModel& m = custom_;
  Status s = writeCalibrationRegisters(true);
  if (s != Status::Ok) return s;

  if (m.modelCfgBeforeLoad) {
    uint16_t v = m.modelCfgBeforeLoad;
    if (isHighSideCapable_) {
      uint16_t cur = 0;
      if (readReg(Reg::MODELCFG, cur)) v = (uint16_t)((v & ~ModelCfgBit::CSEL) | (cur & ModelCfgBit::CSEL));
    }
    if (!writeReg(Reg::MODELCFG, v)) return Status::I2cError;
  }

  // ---- 2.3.1 - 2.3.4: unlock, write, verify, lock, verify lock ----
  bool modelOk = false;
  for (uint8_t attempt = 0; attempt < 3 && !modelOk; ++attempt) {
    if (!writeReg(Reg::MODELACCESS1, ModelAccess::UNLOCK1)) return Status::I2cError;
    if (!writeReg(Reg::MODELACCESS2, ModelAccess::UNLOCK2)) return Status::I2cError;

    s = writeBlock(Reg::OCVTABLE0, m.table0, 16);
    if (s == Status::Ok) s = writeBlock(Reg::XTABLE0, m.table1, 16);
    if (s == Status::Ok && !writeReg(Reg::RCOMPSEG, m.rCompSeg)) s = Status::I2cError;
    if (s != Status::Ok) return s;

    uint16_t seg = 0;
    const bool verified = blockMatches(Reg::OCVTABLE0, m.table0, 16) &&
                          blockMatches(Reg::XTABLE0, m.table1, 16) &&
                          readReg(Reg::RCOMPSEG, seg) && seg == m.rCompSeg;
    if (!verified) {
      logf("model readback mismatch (attempt %u)", (unsigned)(attempt + 1));
      continue;  // back to 2.3.1
    }

    bool locked = false;
    for (uint8_t la = 0; la < 3 && !locked; ++la) {
      if (!writeReg(Reg::MODELACCESS1, ModelAccess::LOCK)) return Status::I2cError;
      if (!writeReg(Reg::MODELACCESS2, ModelAccess::LOCK)) return Status::I2cError;
      delay(1);
      uint16_t segLocked = 0xFFFF;
      locked = blockIsZero(Reg::OCVTABLE0, 32) && readReg(Reg::RCOMPSEG, segLocked) && segLocked == 0;
    }
    if (!locked) {
      // Never leave the model unlocked: the gauge cannot track capacity then.
      logf("model access did not lock");
      return Status::ModelLoadFailed;
    }
    modelOk = true;
  }
  if (!modelOk) {
    writeReg(Reg::MODELACCESS1, ModelAccess::LOCK);
    writeReg(Reg::MODELACCESS2, ModelAccess::LOCK);
    return Status::ModelLoadFailed;
  }

  // ---- 2.3.5: write custom parameters ----
  delay(100);
  s = writeAndVerifyRegister(Reg::REPCAP, 0x0000);
  if (s != Status::Ok) return s;
  if (!writeReg(Reg::DESIGNCAP, designCapCounts_)) return Status::I2cError;

  uint16_t loadFullCapRep = designCapCounts_;
  uint16_t loadFullCapNom = designCapCounts_;
  uint16_t loadRcomp0 = m.rcomp0;
  uint16_t loadTempCo = m.tempCo;
  uint16_t loadCycles = 0;
  if (m.saved.valid) {
    if (m.saved.cgain == cgain_) {
      loadFullCapRep = m.saved.fullCapRep;
      loadFullCapNom = m.saved.fullCapNom;
      loadRcomp0 = m.saved.rcomp0;
      loadTempCo = m.saved.tempCo;
      loadCycles = m.saved.cycles;
      logf("restoring saved learned parameters");
    } else {
      logf("saved parameters were taken with CGain 0x%04X, now 0x%04X - ignoring them",
           (unsigned)m.saved.cgain, (unsigned)cgain_);
    }
  }
  if (!writeReg(Reg::FULLCAPREP, loadFullCapRep)) return Status::I2cError;

  for (uint8_t attempt = 0; attempt < 4; ++attempt) {
    if (!writeReg(Reg::DQACC, (uint16_t)(loadFullCapNom / 2))) return Status::I2cError;
    if (!writeReg(Reg::DPACC, 0x0C80)) return Status::I2cError;
    delay(10);
    if (!writeReg(Reg::FULLCAPNOM, loadFullCapNom)) return Status::I2cError;
    uint16_t a = 0, b = 0, c = 0;
    if (readReg(Reg::FULLCAPNOM, a) && readReg(Reg::DQACC, b) && readReg(Reg::DPACC, c) &&
        a == loadFullCapNom && b == (uint16_t)(loadFullCapNom / 2) && c == 0x0C80) {
      break;
    }
  }

  uint16_t vfsoc = 0;
  if (!readReg(Reg::VFSOC, vfsoc)) return Status::I2cError;
  const uint16_t updateCapacity = (uint16_t)((float)vfsoc / 25600.0f * (float)loadFullCapNom);
  if (!writeReg(Reg::MIXCAP, updateCapacity)) return Status::I2cError;
  if (!writeReg(Reg::AVCAP, updateCapacity)) return Status::I2cError;
  delay(200);

  if (!writeReg(Reg::ICHGTERM, ichgTermCounts_)) return Status::I2cError;
  if (!writeReg(Reg::VEMPTY, m.vEmpty)) return Status::I2cError;
  if (!writeReg(Reg::RCOMP0, loadRcomp0)) return Status::I2cError;
  if (!writeReg(Reg::TEMPCO, loadTempCo)) return Status::I2cError;
  if (!writeReg(Reg::QRTABLE00, m.qrTable00)) return Status::I2cError;
  if (!writeReg(Reg::QRTABLE10, m.qrTable10)) return Status::I2cError;
  if (m.optionalMask & OPT_QRTABLE2030) {
    if (!writeReg(Reg::QRTABLE20, m.qrTable20)) return Status::I2cError;
    if (!writeReg(Reg::QRTABLE30, m.qrTable30)) return Status::I2cError;
  }

  // ---- 2.3.6: optional registers ----
  s = writeOptionalCustomRegisters(m);
  if (s != Status::Ok) return s;

  // ---- 2.3.7: initiate model loading, poll LdMdl ----
  uint16_t cfg2 = 0;
  if (!readReg(Reg::CONFIG2, cfg2)) return Status::I2cError;
  if (!writeReg(Reg::CONFIG2, (uint16_t)(cfg2 | Config2Bit::LDMDL))) return Status::I2cError;
  const uint32_t start = millis();
  while (true) {
    uint16_t v = 0;
    if (!readReg(Reg::CONFIG2, v)) return Status::I2cError;
    if ((v & Config2Bit::LDMDL) == 0) break;
    if (millis() - start >= cfg_.timeoutMs) {
      logf("timeout waiting for Config2.LdMdl");
      return Status::Timeout;
    }
    // Per the guide, Current and AvgCurrent are zeroed while waiting
    writeReg(Reg::CURRENT, 0x0000);
    writeReg(Reg::AVGCURRENT, 0x0000);
    delay(10);
  }

  // ---- 2.3.8: QRTable20/30 and Cycles ----
  if (m.optionalMask & OPT_QRTABLE2030) {
    s = writeAndVerifyRegister(Reg::QRTABLE20, m.qrTable20);
    if (s != Status::Ok) return s;
    s = writeAndVerifyRegister(Reg::QRTABLE30, m.qrTable30);
    if (s != Status::Ok) return s;
  }
  s = writeAndVerifyRegister(Reg::CYCLES, loadCycles);
  if (s != Status::Ok) return s;

  return writeCalibrationRegisters(false);
}

// ---------------------------------------------------------------------------
// Settings that are cheap and idempotent: applied on every begin()
// ---------------------------------------------------------------------------
ModelGaugeM5::Status ModelGaugeM5::applyAlwaysOnSettings() {
  // If the gauge kept its state, make sure CGain/COff still match this config.
  uint16_t g = 0, o = 0;
  if (!readReg(Reg::CGAIN, g) || !readReg(Reg::COFF, o)) return Status::I2cError;
  if (g != cgain_ || o != (uint16_t)cal_.offsetCounts) {
    logf("CGain/COff were 0x%04X/0x%04X, rewriting", (unsigned)g, (unsigned)o);
    Status s = writeCalibrationRegisters(false);
    if (s != Status::Ok) return s;
  }

  Status s = setTemperatureSource(cfg_.tempSource);
  if (s != Status::Ok) return s;
  if (cfg_.tempSource == TempSource::Thermistor && (ranInit_ || cfg_.forceInit)) {
    s = setThermistor(cfg_.thermistor);
    if (s != Status::Ok) return s;
  }
  return Status::Ok;
}

// ===========================================================================
// Quick start, reset
// ===========================================================================

ModelGaugeM5::Status ModelGaugeM5::quickStart(uint32_t timeoutMs) {
  Status s = requireBegun();
  if (s != Status::Ok) return s;

  // T1: set MiscCfg.QS
  uint16_t misc = 0;
  if (!readReg(Reg::MISCCFG, misc)) return Status::I2cError;
  if (!writeReg(Reg::MISCCFG, (uint16_t)(misc | MiscCfgBit::QS))) return Status::I2cError;

  // T2: wait until BOTH MiscCfg.QS and FSTAT.DNR are clear. (The pseudo-code
  // in the Maxim guide uses "and" in the while condition, which would exit as
  // soon as either one cleared; the text says wait for both.)
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint16_t m = 0, f = 0;
    if (readReg(Reg::MISCCFG, m) && readReg(Reg::FSTAT, f) &&
        (m & MiscCfgBit::QS) == 0 && (f & FStatBit::DNR) == 0) {
      return Status::Ok;
    }
    delay(10);
  }
  logf("quick start timed out");
  return Status::Timeout;
}

ModelGaugeM5::Status ModelGaugeM5::reset() {
  Status s = requireBegun();
  if (s != Status::Ok) return s;
  // Active mode first so the command is not delayed by hibernate.
  uint16_t hib = 0;
  if (!readReg(Reg::HIBCFG, hib)) return Status::I2cError;
  if (!writeReg(Reg::COMMAND, Cmd::SOFT_WAKEUP)) return Status::I2cError;
  if (!writeReg(Reg::HIBCFG, 0x0000)) return Status::I2cError;
  if (!writeReg(Reg::COMMAND, Cmd::CLEAR)) return Status::I2cError;
  if (!writeReg(Reg::COMMAND, Cmd::FG_RESET)) return Status::I2cError;
  delay(10);
  return Status::Ok;  // HibCfg returns to its power-up value with the reset
}
