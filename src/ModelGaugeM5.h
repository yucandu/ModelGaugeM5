// ModelGaugeM5.h
//
// Driver core for the Maxim / Analog Devices ModelGauge m5 EZ fuel gauges
// MAX17055 and MAX17260 (single-cell Li-ion/LiPo/LiFePO4).
//
// Use the concrete classes:
//
//     #include <MAX17055.h>   ->  class MAX17055
//     #include <MAX17260.h>   ->  class MAX17260   (also supports high-side sensing)
//
// Both derive from ModelGaugeM5, which holds everything the two parts share.
//
// Highlights
//   * Follows the ModelGauge m5 host-side software implementation guide
//     (POR check -> wait for FSTAT.DNR -> leave hibernate -> load model ->
//     restore HibCfg -> clear POR) for EZ, short custom and full custom models.
//   * Battery chemistry selection (ModelID 0 / 2 / 6) and 4.2 V / 4.35-4.4 V
//     charge voltage (ModelCfg.VChg).
//   * Any sense resistor value. All physical units (mA, mAh, W, ...) are
//     derived from the shunt value you configure.
//   * "CGain scaling": for big pack / small shunt combinations the capacity
//     counts overflow what the gauge handles comfortably. The library can run
//     the gauge with CGain = 0x400/N so it sees 1/N of the real current and
//     capacity (N = 1, 2, 4, 8), and transparently multiplies back by N.
//     Chosen automatically or forced.
//   * Current gain / offset calibration (single point and two point) on top of
//     the scaling, plus shunt-tolerance trim.
//   * Quick start, hibernate control, soft reset, alerts, thermistor setup,
//     copper-trace sensing, dynamic power / AtRate, serial number, learned
//     parameter save/restore, SOC hold, LiFePO4 OCV keep-out window.
//
// Licensed under the MIT License (see LICENSE).

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "ModelGaugeM5_Registers.h"

class ModelGaugeM5 {
 public:
  // -------------------------------------------------------------------------
  // Result codes
  // -------------------------------------------------------------------------
  enum class Status : uint8_t {
    Ok = 0,
    I2cError,         // NACK / short read that survived the retries
    NotFound,         // nothing answered at the I2C address
    WrongDevice,      // DevName register does not match this class
    InvalidConfig,    // configuration rejected before touching the gauge
    OutOfRange,       // value (or scaling) cannot be represented
    Timeout,          // gauge did not finish (DNR / Refresh / LdMdl / QS)
    VerifyFailed,     // write-and-verify did not stick
    ModelLoadFailed,  // custom model could not be written / locked
    NotInitialized,   // call begin() first
    Unsupported       // feature not available on this IC
  };
  static const char* statusToString(Status s);

  // -------------------------------------------------------------------------
  // Chemistry / model selection (ModelCfg.ModelID)
  // -------------------------------------------------------------------------
  enum class ModelID : uint8_t {
    LiCoO2  = 0,  // most Li-ion / LiPo (lithium cobalt oxide variants)
    NcrNca  = 2,  // Panasonic-style NCR / NCA cells (documented for MAX1726x;
                  // NOT listed in the MAX17055 user guide - see README)
    LiFePO4 = 6   // lithium iron phosphate (a custom model is strongly advised)
  };

  enum class TempSource : uint8_t {
    Internal   = 0,  // die temperature, +-1 degC, no thermistor needed
    Thermistor = 1   // external NTC on THRM/AIN (055) or TH (260)
  };

  // TGain / TOff / Curve.TCurve values for a given NTC (user-guide Table 7)
  struct Thermistor {
    uint16_t tGain;
    uint16_t tOff;
    uint16_t tCurve;  // goes in the LOW byte of the Curve register
    static Thermistor Ntc10k_B3435() {  // SEMITEC 103AT-2, Murata NCP15XH103F03RC
      Thermistor t = {0xEE56, 0x1DA4, 0x0025};
      return t;
    }
    static Thermistor Ntc10k_B3974() {  // Fenwal 197-103LAG-A01
      Thermistor t = {0xF49A, 0x16A1, 0x0064};
      return t;
    }
    static Thermistor Ntc10k_B4550() {  // TDK Type F
      Thermistor t = {0xF284, 0x18E8, 0x0035};
      return t;
    }
  };

  // -------------------------------------------------------------------------
  // Learned parameters (host-side save/restore, implementation guide 3.4/2.3.5)
  // -------------------------------------------------------------------------
  struct LearnedParams {
    uint16_t rcomp0;
    uint16_t tempCo;
    uint16_t fullCapRep;
    uint16_t fullCapNom;
    uint16_t cycles;
    uint16_t cgain;   // CGain in effect when saved; restore is refused if it differs
    bool     valid;
    LearnedParams() : rcomp0(0), tempCo(0), fullCapRep(0), fullCapNom(0),
                      cycles(0), cgain(0), valid(false) {}
  };

  // -------------------------------------------------------------------------
  // Custom (characterized) model. Values are the raw register words found in
  // the .ini file Analog Devices generates for your cell.
  //
  // IMPORTANT: the raw words are only meaningful for the sense resistor the
  // characterization assumed. DesignCap and IChgTerm are NOT taken from the
  // ini - the library computes them from Config.designCapacityMah /
  // chargeTerminationMa and your shunt/scale - but QRTable*, RCOMP0, TempCo,
  // the OCV/X tables etc. are written exactly as given.
  // -------------------------------------------------------------------------
  enum CustomOptional : uint16_t {
    OPT_LEARNCFG   = 0x0001,
    OPT_FULLSOCTHR = 0x0002,
    OPT_QRTABLE2030= 0x0004,
    OPT_RELAXCFG   = 0x0008,
    OPT_CONFIG     = 0x0010,
    OPT_CONFIG2    = 0x0020,
    OPT_THERMISTOR = 0x0040,  // tGain / tOff / curve
    OPT_MISCCFG    = 0x0080
  };

  struct CustomModel {
    // --- short format (Option 2) fields, also needed by the long format ---
    uint16_t modelCfg;    // ini "modelcfg" (Refresh is added by the library)
    uint16_t vEmpty;      // ini "VEmpty" raw word (overrides Config empty/recovery voltage)
    uint16_t rcomp0;
    uint16_t tempCo;
    uint16_t qrTable00;
    uint16_t qrTable10;
    uint16_t qrTable20;   // used if OPT_QRTABLE2030
    uint16_t qrTable30;   // used if OPT_QRTABLE2030
    // --- optional registers (apply only if the matching OPT_* bit is set) ---
    uint16_t optionalMask;
    uint16_t learnCfg;
    uint16_t fullSocThr;
    uint16_t relaxCfg;
    uint16_t config;
    uint16_t config2;
    uint16_t tGain, tOff, curve;
    uint16_t miscCfg;
    // --- long format (Option 3): full OCV model ---
    bool     fullModel;       // true = Option 3 (OCV + X tables), false = Option 2
    uint16_t table0[16];      // 16 words -> 0x80..0x8F
    uint16_t table1[16];      // 16 words -> 0x90..0x9F
    uint16_t rCompSeg;        // -> 0xAF
    uint16_t modelCfgBeforeLoad;  // optional, written before the model if non-zero
                                  // (e.g. 0x0060 enables LiFePO4 mode per the user guide)
    LearnedParams saved;      // if saved.valid, restored per implementation guide 2.3.5

    CustomModel()
        : modelCfg(0x8000), vEmpty(0), rcomp0(0), tempCo(0), qrTable00(0), qrTable10(0),
          qrTable20(0), qrTable30(0), optionalMask(0), learnCfg(0), fullSocThr(0),
          relaxCfg(0), config(0), config2(0), tGain(0), tOff(0), curve(0), miscCfg(0),
          fullModel(false), rCompSeg(0), modelCfgBeforeLoad(0) {
      for (uint8_t i = 0; i < 16; ++i) { table0[i] = 0; table1[i] = 0; }
    }
  };

  // -------------------------------------------------------------------------
  // Current calibration (applied through CGain / COff)
  //   Current register = ADC * CGain/0x400 + COff
  // -------------------------------------------------------------------------
  struct CurrentCalibration {
    float   gainTrim;       // multiplies the reading (1.0 = none)
    int16_t offsetCounts;   // COff, in current-register counts
    CurrentCalibration() : gainTrim(1.0f), offsetCounts(0) {}
  };

  // -------------------------------------------------------------------------
  // Configuration
  // -------------------------------------------------------------------------
  struct Config {
    // ---- battery ----
    float   designCapacityMah   = 0.0f;   // REQUIRED: label capacity of the pack
    float   chargeTerminationMa = 0.0f;   // charger termination current; 0 -> C/20
    float   chargeVoltageV      = 4.2f;   // > 4.275 selects the 4.3-4.4 V model (VChg)
    ModelID modelId             = ModelID::LiCoO2;
    float   emptyVoltageV       = 3.3f;   // VEmpty.VE   (10 mV steps, Maxim default 3.3 V)
    float   recoveryVoltageV    = 3.88f;  // VEmpty.VR   (40 mV steps, Maxim default 3.88 V)
    uint8_t fullSocThrPercent   = 80;     // FullSOCThr; 80 = Maxim's EZ recommendation,
                                          // 0 = leave at the 95 % power-up default

    // ---- current sensing ----
    float   senseResistorMohm   = 10.0f;  // the physical shunt value
    uint8_t scale               = 0;      // 0 = auto, or force 1 / 2 / 4 / 8 (CGain = 0x400/scale)
    float   peakCurrentMa       = 0.0f;   // optional: largest expected |current| for auto scaling
    uint16_t maxCapacityCounts  = 0x7000; // auto scaling keeps DesignCap at or below this
    float   gainTrim            = 1.0f;   // see CurrentCalibration
    int16_t currentOffsetCounts = 0;
    bool    copperTraceSensing  = false;  // sense resistor is a PCB trace (CGTempCo = 0x20C8)

    // ---- temperature ----
    TempSource tempSource       = TempSource::Internal;
    Thermistor thermistor       = Thermistor::Ntc10k_B3435();
    bool    thermistor100k      = false;  // MAX17260 only (ModelCfg.R100)

    // ---- behaviour ----
    bool    quickStartAfterInit = false;  // run quick start right after loading the model
    bool    forceInit           = false;  // load the model even if POR is not set
    bool    skipDeviceCheck     = false;  // do not verify DevName
    uint32_t timeoutMs          = 10000;  // DNR / Refresh / LdMdl timeout
    uint8_t i2cRetries          = 2;      // extra attempts per register access
    const CustomModel* customModel = nullptr;  // use a characterized model instead of EZ
    Print*  debug               = nullptr;     // optional diagnostics

    // ---- ready-made starting points ----
    static Config LiIon(float capacityMah, float terminationMa, float chargeV = 4.2f) {
      Config c;
      c.designCapacityMah = capacityMah;
      c.chargeTerminationMa = terminationMa;
      c.chargeVoltageV = chargeV;
      c.modelId = ModelID::LiCoO2;
      return c;
    }
    static Config NcrNca(float capacityMah, float terminationMa, float chargeV = 4.2f) {
      Config c = LiIon(capacityMah, terminationMa, chargeV);
      c.modelId = ModelID::NcrNca;
      return c;
    }
    // LiFePO4: the empty/recovery voltages below are sensible starting points
    // chosen by this library (the datasheets give no LiFePO4 values) - check
    // them against your cell. The 3.88 V default recovery voltage used for
    // Li-ion can never be reached by a 3.65 V cell, so it is not used here.
    static Config LiFePO4(float capacityMah, float terminationMa, float chargeV = 3.65f) {
      Config c;
      c.designCapacityMah = capacityMah;
      c.chargeTerminationMa = terminationMa;
      c.chargeVoltageV = chargeV;
      c.modelId = ModelID::LiFePO4;
      c.emptyVoltageV = 2.8f;
      c.recoveryVoltageV = 3.2f;
      return c;
    }
  };

  // -------------------------------------------------------------------------
  // Alerts
  // -------------------------------------------------------------------------
  enum AlertFlag : uint16_t {
    ALERT_SOC_MAX  = m5g::StatusBit::SMX,
    ALERT_TEMP_MAX = m5g::StatusBit::TMX,
    ALERT_VOLT_MAX = m5g::StatusBit::VMX,
    ALERT_SOC_MIN  = m5g::StatusBit::SMN,
    ALERT_TEMP_MIN = m5g::StatusBit::TMN,
    ALERT_VOLT_MIN = m5g::StatusBit::VMN,
    ALERT_SOC_1PCT = m5g::StatusBit::DSOCI,
    ALERT_CURR_MAX = m5g::StatusBit::IMX,
    ALERT_CURR_MIN = m5g::StatusBit::IMN,
    ALERT_ALL      = m5g::StatusBit::ALL_ALERTS
  };

  struct AlertConfig {
    bool  voltage = false;  float vMinV = 0.0f;     float vMaxV = 5.1f;     // 20 mV steps
    bool  temperature = false; float tMinC = -128.0f; float tMaxC = 127.0f; // 1 degC steps
    bool  soc = false;      float sMinPct = 0.0f;   float sMaxPct = 100.0f; // 1 % steps
    bool  current = false;  float iMinMa = 0.0f;    float iMaxMa = 0.0f;    // see README (steps of 0.4 mV/Rsense)
    bool  socChange = false;   // alert on every 1 % RepSOC change (dSOCi)
    bool  sticky = false;      // alerts latch until clearAlerts() (Config.SS/TS/VS/IS)
  };

  // -------------------------------------------------------------------------
  // One-shot read of the commonly displayed values. Failed reads give NAN.
  // -------------------------------------------------------------------------
  struct Snapshot {
    float vCellV, avgVCellV, vfOcvV;
    float currentMa, avgCurrentMa;
    float socPct, vfSocPct;
    float repCapMah, fullCapRepMah, fullCapNomMah;
    float tempC;
    float rCellMohm;
    float tteSeconds, ttfSeconds;    // NAN when the gauge reports "not applicable"
    float cycles;
    float agePct;
    uint16_t status, fstat;
    bool  ok;                        // true if every essential read succeeded
  };

  struct MinMax { float max; float min; };

  // =========================================================================
  // Lifecycle
  // =========================================================================

  // Attach to the gauge, verify it, load the configuration if the gauge has
  // just powered up (POR) or Config.forceInit is set, and apply the always-on
  // settings (current calibration, temperature source). Safe to call on every
  // boot, including after ESP32 deep sleep: if the gauge kept its state
  // (no POR) the model is NOT reloaded.
  // i2cAddress 0 = the part's default (0x36).
  Status begin(TwoWire& wire, const Config& config, uint8_t i2cAddress = 0);
  Status begin(const Config& config, uint8_t i2cAddress = 0) { return begin(Wire, config, i2cAddress); }

  // True when begin() actually (re)loaded the model on its last call.
  bool didInitialize() const { return ranInit_; }

  // Raw ID check / POR check (no configuration side effects)
  Status readDevName(uint16_t& devName);
  Status checkPor(bool& porSet);

  // Run the full load sequence again (EZ / custom per Config).
  Status initialize();

  // Quick start (implementation guide steps T1/T2): re-sample OCV and
  // re-estimate the initial SOC. Only use once supply and battery have
  // settled; see README.
  Status quickStart(uint32_t timeoutMs = 2000);

  // Software fuel-gauge reset; the POR bit is set again afterwards. Follow
  // with initialize() (or begin()). See Cmd::FG_RESET note in the register header.
  Status reset();

  // =========================================================================
  // Conversions / derived values
  // =========================================================================
  float effectiveScale()      const { return scale_; }        // gauge-side divisor in effect
  uint16_t cgainRegister()    const { return cgain_; }
  float capacityLsbMah()      const { return capLsbMah_; }    // mAh per capacity count
  float currentLsbMa()        const { return curLsbMa_; }     // mA per current count
  float currentRangeMa()      const { return 32768.0f * curLsbMa_; }
  uint16_t designCapCounts()  const { return designCapCounts_; }
  uint16_t chargeTermCounts() const { return ichgTermCounts_; }
  uint8_t  i2cAddress()       const { return addr_; }
  const Config& config()      const { return cfg_; }

  // Pure helper, usable without a gauge: choose the smallest N (1/2/4/8) so
  // DesignCap fits in maxCapacityCounts and peakCurrentMa (if > 0) fits in
  // 90 % of the current range. Returns 0 if no N works.
  static uint8_t planScale(float senseResistorMohm, float designCapacityMah,
                           float peakCurrentMa = 0.0f, uint16_t maxCapacityCounts = 0x7000);

  // Shunt tolerance trim: pass the nominal and the measured shunt value.
  static float shuntTrim(float nominalMohm, float measuredMohm) {
    return (measuredMohm > 0.0f) ? nominalMohm / measuredMohm : 1.0f;
  }

  // =========================================================================
  // Measurements (return false on I2C failure; out-parameters untouched then)
  // =========================================================================
  bool readStatus(uint16_t& status);
  bool readFStat(uint16_t& fstat);

  bool readVCell(float& volts);
  bool readAvgVCell(float& volts);
  bool readVfOcv(float& volts);

  bool readCurrent(float& mA);          // + = charging, - = discharging
  bool readAvgCurrent(float& mA);

  bool readSoc(float& percent);         // RepSOC (what you show the user)
  bool readAvSoc(float& percent);
  bool readMixSoc(float& percent);
  bool readVfSoc(float& percent);

  bool readRepCap(float& mAh);          // remaining capacity (RepCap)
  bool readFullCapRep(float& mAh);      // full capacity matching RepCap
  bool readFullCapNom(float& mAh);
  bool readFullCap(float& mAh);
  bool readAvCap(float& mAh);
  bool readMixCap(float& mAh);
  bool readQResidual(float& mAh);
  bool readCoulombCounter(float& mAh);  // QH, signed raw coulomb count

  bool readTemperature(float& degC);    // Temp (source per Config.tempSource)
  bool readAvgTemperature(float& degC);
  bool readDieTemperature(float& degC);

  // Seconds; NAN if the register holds 0xFFFF. TTE is only meaningful while
  // discharging, TTF only while charging.
  bool readTimeToEmpty(float& seconds);
  bool readTimeToFull(float& seconds);

  bool readAge(float& percent);         // FullCapRep / DesignCap
  bool readCycles(float& cycles);
  bool readRCell(float& mOhm);          // see README note on non-10 mOhm shunts
  bool readPower(float& watts);         // signed
  bool readAvgPower(float& watts);
  bool readTimeSincePor(float& seconds);

  bool readMaxMinVoltage(MinMax& mm);       // volts, 20 mV resolution
  bool readMaxMinCurrent(MinMax& mm);       // mA, 0.4 mV/Rsense resolution
  bool readMaxMinTemperature(MinMax& mm);   // degC, 1 degC resolution
  bool resetMaxMin();                       // restore the power-up values

  bool readSnapshot(Snapshot& snap);

  // =========================================================================
  // Current calibration
  // =========================================================================
  CurrentCalibration calibration() const;
  // Writes CGain/COff (verified) and updates the unit conversions.
  Status setCalibration(const CurrentCalibration& cal);

  // Single point gain: reportedMa = what readCurrent() gave, trueMa = DMM.
  Status calibrateGain(float reportedMa, float trueMa);
  // Offset only: reportedMaAtZero = readCurrent() with the true current at 0.
  Status calibrateOffset(float reportedMaAtZero);
  // Two point gain + offset: true = a * reported + b.
  Status calibrateTwoPoint(float reported1Ma, float true1Ma,
                           float reported2Ma, float true2Ma);

  // NOTE: calibrating while running changes what a capacity/current count
  // means by the (small) calibration ratio. DesignCap / IChgTerm already in
  // the gauge are not rewritten; call initialize() (or begin() with
  // Config.forceInit) afterwards if you want them re-expressed, or simply
  // pass the stored calibration in Config at the next boot.

  // =========================================================================
  // Temperature
  // =========================================================================
  Status setTemperatureSource(TempSource source);
  Status setThermistor(const Thermistor& ntc);   // TGain, TOff, Curve.TCurve
  Status setForceThermistorBias(bool force);     // Config.FTHRM (adds ~200 uA)
  Status setCopperTraceSensing(bool enable);     // CGTempCo 0x20C8 / 0
  Status setCopperTraceCurve(uint8_t mCurve);    // Curve high byte

  // =========================================================================
  // Alerts
  // =========================================================================
  Status configureAlerts(const AlertConfig& alerts);
  Status disableAlerts();
  uint16_t alertFlags();                         // Status & ALERT_ALL (0 if the read fails)
  Status clearAlerts(uint16_t flags = ALERT_ALL);
  Status setSocAlertSource(uint8_t sacfg);       // 0 RepSOC, 1 AvSOC, 2 MixSOC, 3 VFSOC

  // =========================================================================
  // Power modes
  // =========================================================================
  Status setHibernateEnabled(bool enable);       // HibCfg.EnHib
  Status wake(uint16_t* savedHibCfg = nullptr);  // leave hibernate (saves HibCfg, then sets it 0)
  Status restoreHibCfg(uint16_t savedHibCfg);
  bool   isHibernating(bool& hibernating);       // Status2.Hib
  // Software shutdown. The gauge wakes on communication / pin edges and
  // comes back with POR set (outputs are lost). fast = ShdnTimer 0x001E (~22 s)
  // instead of the 45 s default.
  Status shutdown(bool fast = true);

  // =========================================================================
  // Learning / algorithm tuning
  // =========================================================================
  Status readLearnedParams(LearnedParams& out);
  // True each time Cycles bit 6 toggles = a good moment to save (guide 3.4).
  bool   learnedParamsDue();
  Status setLearnStage(uint8_t stage);           // 7 = jump to coulomb-counter dominant
  Status setFullSocThreshold(uint8_t percent);
  // RelaxCfg.LOAD: current below which the cell counts as unloaded (mA, real units).
  Status setRelaxLoadThresholdMa(float mA);
  // SOCHold: hold RepSOC >= emptySocHoldPct until VEmpty+emptyVoltHoldV is crossed,
  // and (optionally) hold at 99 % until charge termination.
  Status setSocHold(bool hold99Percent, float emptyVoltHoldV, float emptySocHoldPct);
  // LiFePO4 only (ModelID 6): OCV keep-out window used for capacity learning.
  Status setLiFePO4OcvKeepOut(float lowV, float highV);

  // =========================================================================
  // AtRate / dynamic power
  // =========================================================================
  Status enableAtRate(bool enable);
  Status enableDynamicPower(bool enable);
  Status setConstantPowerMode(bool enable);
  // dischargeMa is a positive magnitude; the library writes it negative as
  // the datasheet requires. Wait ~351 ms (active) / 11.25 s (hibernate)
  // before reading the results.
  Status setAtRate(float dischargeMa);
  bool   readAtTimeToEmpty(float& seconds);
  bool   readAtSoc(float& percent);
  bool   readAtCapacity(float& mAh);
  bool   readAtQResidual(float& mAh);
  bool   readMaxPeakPowerMw(float& mW);          // 0.8 mW/LSB per the user guide, signed (negative)
  bool   readSusPeakPowerMw(float& mW);
  bool   readMppCurrentRaw(int16_t& raw);        // unit/LSB not given in the supplied docs
  bool   readSppCurrentRaw(int16_t& raw);
  Status setPackResistanceMohm(float mOhm);
  Status setSysResistanceMohm(float mOhm);
  Status setMinSysVoltageRaw(uint16_t raw);      // LSB not given in the supplied docs

  // =========================================================================
  // Misc
  // =========================================================================
  // Unique 128-bit serial number as 8 words (temporarily disables AtRate/DP).
  Status readSerialNumber(uint16_t words[8]);

  // Raw access (always available)
  bool   readRegister(uint8_t reg, uint16_t& value) { return readReg(reg, value); }
  bool   writeRegister(uint8_t reg, uint16_t value) { return writeReg(reg, value); }
  Status writeAndVerifyRegister(uint8_t reg, uint16_t value, uint8_t attempts = 3,
                                uint16_t compareMask = 0xFFFF);
  Status modifyRegister(uint8_t reg, uint16_t clearMask, uint16_t setMask);

  // Probe 7-bit addresses 0x36, 0x0D and 0x1A for a gauge with the given DevName
  // (the "B" ordering variant of the MAX17260 uses a different address).
  static bool findAddress(TwoWire& wire, uint16_t expectedDevName, uint8_t& foundAddress);

 protected:
  enum class Variant : uint8_t { Max17055, Max17260 };
  explicit ModelGaugeM5(Variant v);

 private:
  // ---- low level ----
  bool readReg(uint8_t reg, uint16_t& v);
  bool writeReg(uint8_t reg, uint16_t v);
  bool readS16(uint8_t reg, int16_t& v);
  bool readPercent(uint8_t reg, float& pct);
  bool readCapacity(uint8_t reg, float& mAh);
  bool readSignedCapacity(uint8_t reg, float& mAh);
  bool readVoltage(uint8_t reg, float& volts);
  bool readCurrentReg(uint8_t reg, float& mA);
  bool readTemperatureReg(uint8_t reg, float& degC);
  bool readTime(uint8_t reg, float& seconds);
  bool readPowerReg(uint8_t reg, float& watts);
  void logf(const char* fmt, ...);
  Status requireBegun() const { return begun_ ? Status::Ok : Status::NotInitialized; }

  // ---- planning ----
  Status plan();                       // validate config, pick scale, compute counts
  void   recomputeUnits();

  // ---- init sequence ----
  Status runInit();
  Status waitDnr();
  Status waitClear(uint8_t reg, uint16_t mask, const char* what);
  Status wakeForInit(uint16_t& savedHib);
  Status loadEzModel();
  Status loadShortCustomModel();
  Status loadFullCustomModel();
  Status writeCalibrationRegisters(bool includeCopper);
  Status applyAlwaysOnSettings();
  Status clearPor();
  Status writeModelCfgAndRefresh(uint16_t modelCfgValue);
  uint16_t modelCfgForEz();
  uint16_t encodeVEmpty() const;
  Status writeOptionalCustomRegisters(const CustomModel& m);
  Status writeBlock(uint8_t first, const uint16_t* words, uint8_t n);
  bool   blockMatches(uint8_t first, const uint16_t* words, uint8_t n);
  bool   blockIsZero(uint8_t first, uint8_t n);

  // ---- state ----
  Variant   variant_;
  uint16_t  expectedDevName_;
  uint8_t   defaultAddr_;
  bool      isHighSideCapable_;   // MAX17260
  bool      writesDqAccDpAcc_;    // MAX17055 EZ sequence
  bool      hasR100_;             // MAX17260

  TwoWire*  wire_ = nullptr;
  uint8_t   addr_ = 0x36;
  Config    cfg_;
  CustomModel custom_;            // private copy so the caller's object may go away
  bool      haveCustom_ = false;
  bool      begun_ = false;
  bool      ranInit_ = false;

  uint8_t   plannedScale_ = 1;
  uint16_t  cgain_ = 0x0400;
  float     scale_ = 1.0f;        // trim * 1024 / cgain_
  float     capLsbMah_ = 0.5f;
  float     curLsbMa_ = 0.15625f;
  uint16_t  designCapCounts_ = 0;
  uint16_t  ichgTermCounts_ = 0;
  CurrentCalibration cal_;
  bool      lastCyclesBit6Valid_ = false;
  bool      lastCyclesBit6_ = false;

  friend class MAX17055;
  friend class MAX17260;
};
