// Host-side tests for ModelGaugeM5 using a simulated MAX17055 / MAX17260.
//
//   cd test && make
//
// The simulator models the behaviours the init sequences depend on: FSTAT.DNR
// clearing ~710 ms after power-up, ModelCfg.Refresh / Config2.LdMdl / MiscCfg.QS
// self-clearing, model-access lock/unlock, serial-number mode, fuel-gauge
// reset, and injectable I2C NACKs. It is NOT a model of the fuel-gauge
// algorithm - it cannot tell you whether a configuration gives good SOC.

#include <Arduino.h>
#include <Wire.h>

#include <stdarg.h>
#include <string>
#include <vector>

#include "MAX17055.h"
#include "MAX17260.h"

using namespace m5g;

// ---------------------------------------------------------------------------
// Simulated time
// ---------------------------------------------------------------------------
static uint32_t g_ms = 0;
uint32_t millis() { return g_ms; }
void delay(uint32_t ms) { g_ms += ms; }
HardwareSerial Serial;
I2cDevice* TwoWire::device = nullptr;
TwoWire Wire;

// ---------------------------------------------------------------------------
// Simulated gauge
// ---------------------------------------------------------------------------
struct WriteLogEntry { uint8_t reg; uint16_t value; };

class FakeGauge : public I2cDevice {
 public:
  uint16_t regs[256];
  uint8_t  address;
  uint16_t devName;
  std::vector<WriteLogEntry> log;

  // behaviour switches
  int  nackCount = 0;            // fail this many upcoming transactions
  bool qsNeverClears = false;
  bool stuckUnlocked = false;    // model access never locks
  bool refreshNeverClears = false;

  uint16_t modelTables[33];      // 0x80..0x9F + 0xAF (index 32)
  uint16_t serial[8];

  FakeGauge(uint8_t addr, uint16_t dev) : address(addr), devName(dev) {
    for (int i = 0; i < 8; ++i) serial[i] = (uint16_t)(0xA000 + i);
    powerUp();
  }

  void powerUp() {
    memset(regs, 0, sizeof(regs));
    memset(modelTables, 0, sizeof(modelTables));
    regs[Reg::STATUS] = 0x8082;
    regs[Reg::FSTAT] = 0x0001;
    regs[Reg::DEVNAME] = devName;
    regs[Reg::HIBCFG] = 0x870C;
    regs[Reg::CGAIN] = 0x0400;
    regs[Reg::COFF] = 0x0000;
    regs[Reg::CONFIG] = 0x2210;
    regs[Reg::CONFIG2] = 0x3658;
    regs[Reg::VEMPTY] = 0xA561;
    regs[Reg::MISCCFG] = 0x3870;
    regs[Reg::LEARNCFG] = 0x4486;
    regs[Reg::FULLSOCTHR] = 0x5F05;
    regs[Reg::RELAXCFG] = 0x2039;
    regs[Reg::ICHGTERM] = 0x0640;
    regs[Reg::DESIGNCAP] = 0x0BB8;
    regs[Reg::MODELCFG] = 0x8400;
    regs[Reg::SOCHOLD] = 0x1002;
    regs[Reg::SCOCVLIM] = 0x479E;
    regs[Reg::CURVE] = 0x3C00;  // pretend a copper-trace curve is already set
    regs[Reg::MAXMINVOLT] = 0x00FF;
    regs[Reg::MAXMINCURR] = 0x807F;
    regs[Reg::MAXMINTEMP] = 0x807F;
    dnrClearAt = g_ms + 710;
    refreshDoneAt = qsDoneAt = ldmdlDoneAt = snReadyAt = 0;
    unlocked = false;
  }

  void clearLog() { log.clear(); }
  bool wrote(uint8_t reg) const {
    for (size_t i = 0; i < log.size(); ++i) if (log[i].reg == reg) return true;
    return false;
  }
  uint16_t lastWritten(uint8_t reg) const {
    for (size_t i = log.size(); i-- > 0;) if (log[i].reg == reg) return log[i].value;
    return 0xDEAD;
  }
  int firstIndex(uint8_t reg) const {
    for (size_t i = 0; i < log.size(); ++i) if (log[i].reg == reg) return (int)i;
    return -1;
  }

  // --- I2cDevice ---
  bool present(uint8_t addr) override { return addr == address; }

  bool onWrite(uint8_t, const uint8_t* d, size_t n) override {
    if (nackCount > 0) { --nackCount; return false; }
    update();
    if (n == 0) return true;
    ptr_ = d[0];
    for (size_t i = 1; i + 1 < n; i += 2) {
      writeRegister((uint8_t)(ptr_ + (i - 1) / 2), (uint16_t)(d[i] | (d[i + 1] << 8)));
    }
    return true;
  }

  size_t onRead(uint8_t, uint8_t* out, size_t n) override {
    if (nackCount > 0) { --nackCount; return 0; }
    update();
    for (size_t i = 0; i + 1 < n + 1 && i < n; i += 2) {
      uint16_t v = readRegister((uint8_t)(ptr_ + i / 2));
      out[i] = (uint8_t)(v & 0xFF);
      if (i + 1 < n) out[i + 1] = (uint8_t)(v >> 8);
    }
    return n;
  }

 private:
  uint8_t  ptr_ = 0;
  uint32_t dnrClearAt, refreshDoneAt, qsDoneAt, ldmdlDoneAt, snReadyAt;
  bool     unlocked;

  void update() {
    if ((regs[Reg::FSTAT] & FStatBit::DNR) && g_ms >= dnrClearAt) regs[Reg::FSTAT] &= ~FStatBit::DNR;
    if (refreshDoneAt && g_ms >= refreshDoneAt && !refreshNeverClears) {
      regs[Reg::MODELCFG] &= ~ModelCfgBit::REFRESH;
      refreshDoneAt = 0;
    }
    if (qsDoneAt && g_ms >= qsDoneAt && !qsNeverClears) {
      regs[Reg::MISCCFG] &= ~MiscCfgBit::QS;
      qsDoneAt = 0;
    }
    if (ldmdlDoneAt && g_ms >= ldmdlDoneAt) {
      regs[Reg::CONFIG2] &= ~Config2Bit::LDMDL;
      ldmdlDoneAt = 0;
    }
    if (snReadyAt && g_ms >= snReadyAt) {
      regs[Reg::STATUS2] |= Status2Bit::SN_READY;
      snReadyAt = 0;
    }
  }

  bool snMode() const {
    return (regs[Reg::CONFIG2] & (Config2Bit::ATRATEEN | Config2Bit::DPEN)) == 0;
  }

  uint16_t readRegister(uint8_t r) {
    if ((r >= 0x80 && r <= 0x9F) || r == 0xAF) {
      if (!unlocked) return 0;
      return modelTables[r == 0xAF ? 32 : r - 0x80];
    }
    if (snMode()) {
      switch (r) {
        case 0xD4: return serial[0];
        case 0xD5: return serial[1];
        case 0xD9: return serial[2];
        case 0xDA: return serial[3];
        case 0xDC: return serial[4];
        case 0xDD: return serial[5];
        case 0xDE: return serial[6];
        case 0xDF: return serial[7];
        default: break;
      }
    }
    return regs[r];
  }

  void writeRegister(uint8_t r, uint16_t v) {
    log.push_back({r, v});
    if (r == Reg::DEVNAME || r == Reg::FSTAT) return;  // read-only
    if ((r >= 0x80 && r <= 0x9F) || r == 0xAF) {
      if (unlocked) modelTables[r == 0xAF ? 32 : r - 0x80] = v;
      return;
    }
    if (r == Reg::MODELACCESS1 || r == Reg::MODELACCESS2) {
      regs[r] = v;
      unlocked = !stuckUnlocked ? (regs[Reg::MODELACCESS1] == 0x0059 && regs[Reg::MODELACCESS2] == 0x00C4)
                                : true;
      if (stuckUnlocked && regs[Reg::MODELACCESS1] == 0 && regs[Reg::MODELACCESS2] == 0) unlocked = true;
      return;
    }
    if (r == Reg::COMMAND && v == Cmd::FG_RESET) {
      powerUp();
      return;
    }
    regs[r] = v;
    if (r == Reg::MODELCFG && (v & ModelCfgBit::REFRESH)) refreshDoneAt = g_ms + 80;
    if (r == Reg::MISCCFG && (v & MiscCfgBit::QS)) qsDoneAt = g_ms + 120;
    if (r == Reg::CONFIG2) {
      if (v & Config2Bit::LDMDL) ldmdlDoneAt = g_ms + 200;
      if ((v & (Config2Bit::ATRATEEN | Config2Bit::DPEN)) == 0) snReadyAt = g_ms + 50;
      else regs[Reg::STATUS2] &= ~Status2Bit::SN_READY;
    }
  }
};

// ---------------------------------------------------------------------------
// Tiny test framework
// ---------------------------------------------------------------------------
static int g_checks = 0, g_failed = 0;
static const char* g_current = "";

#define CHECK(cond)                                                                \
  do {                                                                             \
    ++g_checks;                                                                    \
    if (!(cond)) {                                                                 \
      ++g_failed;                                                                  \
      printf("  FAIL [%s] line %d: %s\n", g_current, __LINE__, #cond);             \
    }                                                                              \
  } while (0)

#define CHECK_HEX(actual, expected)                                                \
  do {                                                                             \
    ++g_checks;                                                                    \
    unsigned a_ = (unsigned)(actual), e_ = (unsigned)(expected);                   \
    if (a_ != e_) {                                                                \
      ++g_failed;                                                                  \
      printf("  FAIL [%s] line %d: %s = 0x%04X, expected 0x%04X\n", g_current,     \
             __LINE__, #actual, a_, e_);                                           \
    }                                                                              \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                          \
  do {                                                                             \
    ++g_checks;                                                                    \
    double a_ = (double)(actual), e_ = (double)(expected);                         \
    if (!(fabs(a_ - e_) <= (tol))) {                                               \
      ++g_failed;                                                                  \
      printf("  FAIL [%s] line %d: %s = %.6f, expected %.6f\n", g_current,         \
             __LINE__, #actual, a_, e_);                                           \
    }                                                                              \
  } while (0)

#define CHECK_OK(expr)                                                             \
  do {                                                                             \
    ++g_checks;                                                                    \
    ModelGaugeM5::Status s_ = (expr);                                              \
    if (s_ != ModelGaugeM5::Status::Ok) {                                          \
      ++g_failed;                                                                  \
      printf("  FAIL [%s] line %d: %s -> %s\n", g_current, __LINE__, #expr,        \
             ModelGaugeM5::statusToString(s_));                                    \
    }                                                                              \
  } while (0)

#define CHECK_STATUS(expr, want)                                                   \
  do {                                                                             \
    ++g_checks;                                                                    \
    ModelGaugeM5::Status s_ = (expr);                                              \
    if (s_ != (want)) {                                                            \
      ++g_failed;                                                                  \
      printf("  FAIL [%s] line %d: %s -> %s, expected %s\n", g_current, __LINE__,  \
             #expr, ModelGaugeM5::statusToString(s_), ModelGaugeM5::statusToString(want)); \
    }                                                                              \
  } while (0)

#define TEST(name) static void name()
#define RUN(name)                                                                  \
  do {                                                                             \
    g_current = #name;                                                             \
    int before = g_failed;                                                         \
    name();                                                                        \
    printf("%s %s\n", g_failed == before ? "PASS" : "FAIL", #name);                \
  } while (0)

typedef ModelGaugeM5::Status St;
typedef ModelGaugeM5::Config Cfg;

static FakeGauge* newGauge(uint16_t dev, uint8_t addr = 0x36) {
  static FakeGauge* current = nullptr;
  delete current;
  g_ms = 0;                       // reset the clock BEFORE the gauge computes its DNR time
  current = new FakeGauge(addr, dev);
  TwoWire::device = current;
  return current;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(max17055_ez_10mohm_unscaled) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);          // 2000 mAh, 100 mA termination, 4.2 V
  CHECK_OK(fg.begin(Wire, c));
  CHECK(fg.didInitialize());
  CHECK_NEAR(fg.effectiveScale(), 1.0, 1e-6);
  CHECK_HEX(fg.cgainRegister(), 0x0400);
  CHECK_NEAR(fg.capacityLsbMah(), 0.5, 1e-6);
  CHECK_NEAR(fg.currentLsbMa(), 0.15625, 1e-6);
  CHECK_HEX(g->regs[Reg::DESIGNCAP], 4000);
  CHECK_HEX(g->regs[Reg::ICHGTERM], 640);
  CHECK_HEX(g->regs[Reg::VEMPTY], 0xA561);       // 3.3 V / 3.88 V
  CHECK_HEX(g->regs[Reg::DQACC], 125);           // DesignCap / 32
  CHECK_HEX(g->regs[Reg::DPACC], 1379);          // dQAcc * 44138 / DesignCap
  CHECK_HEX(g->lastWritten(Reg::MODELCFG), 0x8000);
  CHECK_HEX(g->regs[Reg::FULLSOCTHR], 0x5005);
  CHECK_HEX(g->regs[Reg::HIBCFG], 0x870C);       // restored
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) == 0);
  CHECK_HEX(g->regs[Reg::CGAIN], 0x0400);
  // ordering from the implementation guide: DesignCap < ModelCfg < Status
  CHECK(g->firstIndex(Reg::DESIGNCAP) < g->firstIndex(Reg::MODELCFG));
  CHECK(g->firstIndex(Reg::COMMAND) < g->firstIndex(Reg::DESIGNCAP));
  CHECK(g->lastWritten(Reg::HIBCFG) == 0x870C);
}

TEST(max17055_high_voltage_ncr_model) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  Cfg c = Cfg::NcrNca(2000.0f, 100.0f, 4.35f);
  CHECK_OK(fg.begin(Wire, c));
  CHECK_HEX(g->lastWritten(Reg::MODELCFG), 0x8000 | 0x0400 | 0x0020);
  CHECK_HEX(g->regs[Reg::DPACC], 1600);          // 51200 / 32
}

TEST(max17260_big_pack_auto_scaling_matches_v3) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::NcrNca(56780.0f, 450.0f);
  c.senseResistorMohm = 4.0f;
  CHECK_OK(fg.begin(Wire, c));
  CHECK_NEAR(fg.effectiveScale(), 2.0, 1e-6);
  CHECK_HEX(fg.cgainRegister(), 0x0200);
  CHECK_HEX(g->regs[Reg::CGAIN], 0x0200);
  CHECK_NEAR(fg.capacityLsbMah(), 2.5, 1e-6);
  CHECK_NEAR(fg.currentLsbMa(), 0.78125, 1e-6);
  CHECK_HEX(g->regs[Reg::DESIGNCAP], 22712);
  CHECK_HEX(g->regs[Reg::ICHGTERM], 576);
  CHECK(!g->wrote(Reg::DQACC));                  // MAX1726x EZ does not touch dQAcc/dPAcc
  CHECK(!g->wrote(Reg::DPACC));
  CHECK_HEX(g->lastWritten(Reg::MODELCFG), 0x8020);
  // conversions multiply the scaling back in
  g->regs[Reg::REPCAP] = 22712;
  g->regs[Reg::CURRENT] = (uint16_t)(int16_t)-1000;
  g->regs[Reg::REPSOC] = 0x3280;                 // 50.5 %
  float mah = 0, ma = 0, soc = 0;
  CHECK(fg.readRepCap(mah));
  CHECK(fg.readCurrent(ma));
  CHECK(fg.readSoc(soc));
  CHECK_NEAR(mah, 56780.0, 0.5);
  CHECK_NEAR(ma, -781.25, 1e-3);
  CHECK_NEAR(soc, 50.5, 1e-3);
}

TEST(scale_planning) {
  CHECK(ModelGaugeM5::planScale(10.0f, 2000.0f) == 1);
  CHECK(ModelGaugeM5::planScale(4.0f, 56780.0f) == 2);
  CHECK(ModelGaugeM5::planScale(2.0f, 56780.0f) == 1);      // 2 mOhm: 22712 counts at N=1
  CHECK(ModelGaugeM5::planScale(10.0f, 2000.0f, 8000.0f) == 2);   // peak current forces N=2
  CHECK(ModelGaugeM5::planScale(10.0f, 2000.0f, 15000.0f) == 4);   // 0.9 * 20.48 A = 18.4 A
  CHECK(ModelGaugeM5::planScale(10.0f, 2000.0f, 20000.0f) == 8);
  CHECK(ModelGaugeM5::planScale(10.0f, 2000.0f, 50000.0f) == 0);   // beyond N = 8
  CHECK(ModelGaugeM5::planScale(10.0f, 1.0e6f) == 0);       // impossible
  CHECK(ModelGaugeM5::planScale(0.0f, 1000.0f) == 0);
}

TEST(forced_scale_and_limits) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiIon(56780.0f, 450.0f);
  c.senseResistorMohm = 4.0f;
  c.scale = 1;                                   // force: allowed (fits 16 bit), only warns
  CHECK_OK(fg.begin(Wire, c));
  CHECK_HEX(g->regs[Reg::DESIGNCAP], 45424);
  CHECK_HEX(g->regs[Reg::CGAIN], 0x0400);

  g = newGauge(0x4031);
  MAX17260 fg2;
  c.scale = 4;
  CHECK_OK(fg2.begin(Wire, c));
  CHECK_HEX(g->regs[Reg::CGAIN], 0x0100);
  CHECK_HEX(g->regs[Reg::DESIGNCAP], 11356);

  g = newGauge(0x4031);
  MAX17260 fg3;
  c.scale = 3;                                   // not a power of two
  CHECK_STATUS(fg3.begin(Wire, c), St::InvalidConfig);
  c.scale = 0;
  c.designCapacityMah = 1.0e6f;
  c.senseResistorMohm = 10.0f;
  CHECK_STATUS(fg3.begin(Wire, c), St::OutOfRange);
  c.designCapacityMah = 0.0f;
  CHECK_STATUS(fg3.begin(Wire, c), St::InvalidConfig);
  (void)g;
}

TEST(no_reload_when_gauge_kept_state) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiIon(3000.0f, 150.0f);
  CHECK_OK(fg.begin(Wire, c));
  CHECK(fg.didInitialize());
  g->clearLog();
  MAX17260 fg2;                                  // e.g. ESP32 waking from deep sleep
  CHECK_OK(fg2.begin(Wire, c));
  CHECK(!fg2.didInitialize());
  CHECK(!g->wrote(Reg::MODELCFG));
  CHECK(!g->wrote(Reg::DESIGNCAP));
  CHECK(!g->wrote(Reg::STATUS));
  // forceInit reloads even without POR
  c.forceInit = true;
  CHECK_OK(fg2.begin(Wire, c));
  CHECK(fg2.didInitialize());
  CHECK(g->wrote(Reg::MODELCFG));
}

TEST(cgain_self_heals_without_por) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiIon(56780.0f, 450.0f);
  c.senseResistorMohm = 4.0f;
  CHECK_OK(fg.begin(Wire, c));
  g->regs[Reg::CGAIN] = 0x0400;                  // something reset it
  MAX17260 fg2;
  CHECK_OK(fg2.begin(Wire, c));
  CHECK(!fg2.didInitialize());
  CHECK_HEX(g->regs[Reg::CGAIN], 0x0200);
}

TEST(max17260_preserves_csel) {
  FakeGauge* g = newGauge(0x4031);
  g->regs[Reg::MODELCFG] = 0x8404;               // CSEL set by the gauge at boot
  MAX17260 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(3000.0f, 150.0f)));
  CHECK_HEX(g->lastWritten(Reg::MODELCFG), 0x8004);
  bool csel = false;
  CHECK(fg.readCsel(csel));
  CHECK(csel);
}

TEST(max17260_r100_flag) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiIon(3000.0f, 150.0f);
  c.thermistor100k = true;
  CHECK_OK(fg.begin(Wire, c));
  CHECK_HEX(g->lastWritten(Reg::MODELCFG), 0x8000 | 0x2000);
}

TEST(device_checks) {
  FakeGauge* g = newGauge(0x4031);
  MAX17055 wrong;
  CHECK_STATUS(wrong.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)), St::WrongDevice);
  CHECK(!g->wrote(Reg::DESIGNCAP));
  MAX17055 skip;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.skipDeviceCheck = true;
  CHECK_OK(skip.begin(Wire, c));
  MAX17260 nothing;
  CHECK_STATUS(nothing.begin(Wire, Cfg::LiIon(2000.0f, 100.0f), 0x55), St::NotFound);

  // other I2C address variant
  FakeGauge* b = newGauge(0x4031, 0x0D);
  uint8_t found = 0;
  CHECK(MAX17260::detect(Wire, found));
  CHECK(found == 0x0D);
  MAX17260 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f), 0x0D));
  (void)b;
}

TEST(quick_start) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  CHECK_OK(fg.quickStart());
  CHECK((g->regs[Reg::MISCCFG] & MiscCfgBit::QS) == 0);
  CHECK_HEX(g->lastWritten(Reg::MISCCFG) & MiscCfgBit::QS, MiscCfgBit::QS);
  CHECK_HEX(g->lastWritten(Reg::MISCCFG), 0x3870 | 0x0400);   // other bits preserved
  g->qsNeverClears = true;
  CHECK_STATUS(fg.quickStart(300), St::Timeout);

  FakeGauge* g2 = newGauge(0x4031);
  MAX17260 fg2;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.quickStartAfterInit = true;
  CHECK_OK(fg2.begin(Wire, c));
  CHECK(g2->wrote(Reg::MISCCFG));
}

TEST(gain_trim_and_offset_in_config) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.gainTrim = ModelGaugeM5::shuntTrim(10.0f, 9.8f);   // shunt measured 9.8 mOhm -> 1.0204
  c.currentOffsetCounts = -12;
  CHECK_OK(fg.begin(Wire, c));
  CHECK_HEX(g->regs[Reg::CGAIN], 1045);                // round(1024 * 1.020408)
  CHECK_HEX(g->regs[Reg::COFF], (uint16_t)(int16_t)-12);
  // the units use the *actual* applied gain, so a count is worth exactly trim*1024/cgain of nominal
  CHECK_NEAR(fg.effectiveScale(), 1.020408 * 1024.0 / 1045.0, 1e-5);
}

TEST(calibration_single_two_point_offset) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  CHECK_OK(fg.calibrateGain(1000.0f, 1020.0f));        // gauge reads 2 % low
  CHECK_HEX(g->regs[Reg::CGAIN], 1044);
  CHECK_NEAR(fg.calibration().gainTrim, 1.02, 1e-5);
  CHECK_STATUS(fg.calibrateGain(1000.0f, -1000.0f), St::InvalidConfig);
  CHECK_STATUS(fg.calibrateGain(1000.0f, 3000.0f), St::OutOfRange);

  FakeGauge* g2 = newGauge(0x4010);
  MAX17055 fg2;
  CHECK_OK(fg2.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  CHECK_OK(fg2.calibrateOffset(15.625f));              // reads +15.625 mA at zero load -> 100 counts
  CHECK_HEX(g2->regs[Reg::COFF], (uint16_t)(int16_t)-100);

  FakeGauge* g3 = newGauge(0x4010);
  MAX17055 fg3;
  CHECK_OK(fg3.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  // true = a * reported + b with a = 1.0157895, b = 8.4211 mA
  CHECK_OK(fg3.calibrateTwoPoint(100.0f, 110.0f, 2000.0f, 2040.0f));
  CHECK_HEX(g3->regs[Reg::CGAIN], 1040);               // round(1024 * 1.0157895) = 1040
  CHECK_HEX(g3->regs[Reg::COFF], 54);                  // 8.4211 mA / 0.15625 mA
  CHECK_STATUS(fg3.calibrateTwoPoint(100.0f, 110.0f, 100.0f, 110.0f), St::InvalidConfig);
  (void)g;
}

TEST(calibration_with_scaling_keeps_scale) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiIon(56780.0f, 450.0f);
  c.senseResistorMohm = 4.0f;
  CHECK_OK(fg.begin(Wire, c));                         // N = 2, CGain 0x200
  CHECK_OK(fg.calibrateGain(5000.0f, 5100.0f));        // +2 %
  CHECK_HEX(g->regs[Reg::CGAIN], 522);                 // round(512 * 1.02)
  CHECK_NEAR(fg.effectiveScale(), 1.02 * 1024.0 / 522.0, 1e-5);
  CHECK_NEAR(fg.effectiveScale(), 2.0, 0.01);
}

TEST(alerts_encoding) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  ModelGaugeM5::AlertConfig a;
  a.voltage = true;  a.vMinV = 3.0f; a.vMaxV = 4.2f;
  a.temperature = true; a.tMinC = 0.0f; a.tMaxC = 45.0f;
  a.soc = true; a.sMinPct = 10.0f; a.sMaxPct = 100.0f;
  a.current = true; a.iMinMa = -1000.0f; a.iMaxMa = 2000.0f;   // 10 mOhm: 40 mA / step
  a.socChange = true;
  a.sticky = true;
  g->regs[Reg::STATUS] |= StatusBit::VMN | StatusBit::SMX;      // stale flags
  CHECK_OK(fg.configureAlerts(a));
  CHECK_HEX(g->regs[Reg::VALRTTH], (210 << 8) | 150);
  CHECK_HEX(g->regs[Reg::TALRTTH], (45 << 8) | 0);
  CHECK_HEX(g->regs[Reg::SALRTTH], (100 << 8) | 10);
  CHECK_HEX(g->regs[Reg::IALRTTH], (50 << 8) | 0xE7);           // 2000/40 = 50, -1000/40 = -25
  CHECK((g->regs[Reg::STATUS] & (StatusBit::VMN | StatusBit::SMX)) == 0);
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::AEN) != 0);
  CHECK((g->regs[Reg::CONFIG] & (ConfigBit::SS | ConfigBit::TS | ConfigBit::VS | ConfigBit::IS)) ==
        (ConfigBit::SS | ConfigBit::TS | ConfigBit::VS | ConfigBit::IS));
  CHECK((g->regs[Reg::CONFIG2] & Config2Bit::DSOCEN) != 0);
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) == 0);

  // thresholds are programmed before Aen is set
  int thr = g->firstIndex(Reg::VALRTTH);
  int aen = -1;
  for (size_t i = 0; i < g->log.size(); ++i)
    if (g->log[i].reg == Reg::CONFIG && (g->log[i].value & ConfigBit::AEN)) { aen = (int)i; break; }
  CHECK(thr >= 0 && aen > thr);

  g->regs[Reg::STATUS] |= StatusBit::VMX | StatusBit::IMN | StatusBit::POR;
  CHECK_HEX(fg.alertFlags(), StatusBit::VMX | StatusBit::IMN);
  CHECK_OK(fg.clearAlerts(ModelGaugeM5::ALERT_VOLT_MAX));
  CHECK_HEX(fg.alertFlags(), StatusBit::IMN);
  CHECK_OK(fg.clearAlerts());
  CHECK_HEX(fg.alertFlags(), 0);
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) != 0);          // untouched

  CHECK_OK(fg.disableAlerts());
  CHECK_HEX(g->regs[Reg::VALRTTH], 0xFF00);
  CHECK_HEX(g->regs[Reg::TALRTTH], 0x7F80);
  CHECK_HEX(g->regs[Reg::SALRTTH], 0xFF00);
  CHECK_HEX(g->regs[Reg::IALRTTH], 0x7F80);
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::AEN) == 0);

  a.vMinV = 4.0f; a.vMaxV = 3.0f;
  CHECK_STATUS(fg.configureAlerts(a), St::InvalidConfig);
}

TEST(alert_current_scaling) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiIon(56780.0f, 450.0f);
  c.senseResistorMohm = 4.0f;                                    // N = 2
  CHECK_OK(fg.begin(Wire, c));
  ModelGaugeM5::AlertConfig a;
  a.current = true; a.iMinMa = -5000.0f; a.iMaxMa = 10000.0f;
  CHECK_OK(fg.configureAlerts(a));
  // step = 0.4 mV / 4 mOhm * 2 = 200 mA
  CHECK_HEX(g->regs[Reg::IALRTTH], (50 << 8) | (uint8_t)(int8_t)-25);
}

TEST(thermistor_and_temperature_source) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.tempSource = ModelGaugeM5::TempSource::Thermistor;
  c.thermistor = ModelGaugeM5::Thermistor::Ntc10k_B3974();
  CHECK_OK(fg.begin(Wire, c));
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::TSEL) != 0);
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::ETHRM) != 0);
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::TEN) != 0);
  CHECK_HEX(g->regs[Reg::TGAIN], 0xF49A);
  CHECK_HEX(g->regs[Reg::TOFF], 0x16A1);
  CHECK_HEX(g->regs[Reg::CURVE], 0x3C64);               // copper-curve high byte preserved
  CHECK_OK(fg.setTemperatureSource(ModelGaugeM5::TempSource::Internal));
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::TSEL) == 0);
  CHECK_OK(fg.setForceThermistorBias(true));
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::FTHRM) != 0);
  CHECK_OK(fg.setCopperTraceSensing(true));
  CHECK_HEX(g->regs[Reg::CGTEMPCO], 0x20C8);
  CHECK_OK(fg.setCopperTraceCurve(0x55));
  CHECK_HEX(g->regs[Reg::CURVE], 0x5564);
}

TEST(internal_temperature_default_clears_tsel) {
  FakeGauge* g = newGauge(0x4031);
  g->regs[Reg::CONFIG] |= ConfigBit::TSEL;               // something left it on
  MAX17260 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::TSEL) == 0);
}

TEST(conversions) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  float v = 0;
  g->regs[Reg::VCELL] = 51200;                           // 4.000 V
  CHECK(fg.readVCell(v)); CHECK_NEAR(v, 4.0, 1e-4);
  g->regs[Reg::TEMP] = (uint16_t)(int16_t)(-5 * 256);
  CHECK(fg.readTemperature(v)); CHECK_NEAR(v, -5.0, 1e-4);
  g->regs[Reg::TTE] = 0x0100;
  CHECK(fg.readTimeToEmpty(v)); CHECK_NEAR(v, 1440.0, 1e-3);
  g->regs[Reg::TTF] = 0xFFFF;
  CHECK(fg.readTimeToFull(v)); CHECK(isnan(v));
  g->regs[Reg::TIMER] = 100; g->regs[Reg::TIMERH] = 1;
  CHECK(fg.readTimeSincePor(v)); CHECK_NEAR(v, (65536 + 100) * 0.17578125, 1e-2);
  g->regs[Reg::CYCLES] = 250;
  CHECK(fg.readCycles(v)); CHECK_NEAR(v, 2.5, 1e-5);
  g->regs[Reg::AVGCURRENT] = (uint16_t)(int16_t)6400;   // 6400 * 0.15625 = 1000 mA
  CHECK(fg.readAvgCurrent(v)); CHECK_NEAR(v, 1000.0, 1e-3);
  g->regs[Reg::AVGPOWER] = (uint16_t)(int16_t)-1250;    // 8 uV^2 / 10 mOhm = 0.8 mW per count
  CHECK(fg.readAvgPower(v)); CHECK_NEAR(v, -1.0, 1e-4);
  g->regs[Reg::MAXPEAKPOWER] = (uint16_t)(int16_t)-5000;
  CHECK(fg.readMaxPeakPowerMw(v)); CHECK_NEAR(v, -4000.0, 1e-3);
  g->regs[Reg::MAXMINCURR] = (uint16_t)((50 << 8) | (uint8_t)(int8_t)-25);   // 40 mA per step
  ModelGaugeM5::MinMax mm;
  CHECK(fg.readMaxMinCurrent(mm)); CHECK_NEAR(mm.max, 2000.0, 1e-3); CHECK_NEAR(mm.min, -1000.0, 1e-3);
  g->regs[Reg::MAXMINVOLT] = (uint16_t)((212 << 8) | 150);
  CHECK(fg.readMaxMinVoltage(mm)); CHECK_NEAR(mm.max, 4.24, 1e-4); CHECK_NEAR(mm.min, 3.0, 1e-4);
  g->regs[Reg::MAXMINTEMP] = (uint16_t)((30 << 8) | (uint8_t)(int8_t)-4);
  CHECK(fg.readMaxMinTemperature(mm)); CHECK_NEAR(mm.max, 30.0, 1e-6); CHECK_NEAR(mm.min, -4.0, 1e-6);
  CHECK(fg.resetMaxMin());
  CHECK_HEX(g->regs[Reg::MAXMINVOLT], 0x00FF);
  CHECK_HEX(g->regs[Reg::MAXMINCURR], 0x807F);

  ModelGaugeM5::Snapshot snap;
  g->regs[Reg::REPSOC] = 0x6400;
  CHECK(fg.readSnapshot(snap));
  CHECK_NEAR(snap.socPct, 100.0, 1e-3);
  CHECK(snap.ok);
}

TEST(tuning_encodings) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  CHECK_OK(fg.setLiFePO4OcvKeepOut(3.275f, 3.350f));
  CHECK_HEX(g->regs[Reg::SCOCVLIM], 0x479E);                // Maxim's documented default
  CHECK_OK(fg.setSocHold(true, 0.0f, 1.0f));
  CHECK_HEX(g->regs[Reg::SOCHOLD], 0x1002);                 // Maxim's documented default
  CHECK_OK(fg.setSocHold(false, 0.3f, 6.0f));
  CHECK_HEX(g->regs[Reg::SOCHOLD], (30 << 5) | 12);
  CHECK_OK(fg.setLearnStage(7));
  CHECK_HEX(g->regs[Reg::LEARNCFG], 0x4486 | 0x0070);
  CHECK_OK(fg.setRelaxLoadThresholdMa(80.0f));
  CHECK_HEX(g->regs[Reg::RELAXCFG], 0x2039);                // 80 mA @ 10 mOhm = default
  CHECK_OK(fg.setRelaxLoadThresholdMa(160.0f));
  CHECK_HEX(g->regs[Reg::RELAXCFG], (32 << 9) | 0x39);
  CHECK_OK(fg.setFullSocThreshold(95));
  CHECK_HEX(g->regs[Reg::FULLSOCTHR], 0x5F05);
  CHECK_OK(fg.setSocAlertSource(3));
  CHECK_HEX(g->regs[Reg::MISCCFG] & 3, 3);
  CHECK_OK(fg.setPackResistanceMohm(100.0f));
  CHECK_HEX(g->regs[Reg::PACKRESISTANCE], 410);             // 100 mOhm / 0.244 mOhm
  CHECK_STATUS(fg.setSocHold(false, 2.0f, 1.0f), St::OutOfRange);
  CHECK_STATUS(fg.setLearnStage(8), St::InvalidConfig);
}

TEST(atrate_is_written_negative) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  CHECK_OK(fg.setAtRate(2500.0f));                          // 2500 mA at 0.15625 mA/count = 16000
  CHECK_HEX(g->regs[Reg::ATRATE], (uint16_t)(int16_t)-16000);
  CHECK_OK(fg.setAtRate(-2500.0f));                         // sign of the argument does not matter
  CHECK_HEX(g->regs[Reg::ATRATE], (uint16_t)(int16_t)-16000);
  CHECK_OK(fg.enableAtRate(true));
  CHECK_OK(fg.enableDynamicPower(false));
  CHECK((g->regs[Reg::CONFIG2] & Config2Bit::DPEN) == 0);
  CHECK_OK(fg.setConstantPowerMode(true));
  CHECK((g->regs[Reg::CONFIG2] & Config2Bit::CPMODE) != 0);
}

TEST(power_modes) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  uint16_t saved = 0;
  CHECK_OK(fg.wake(&saved));
  CHECK_HEX(saved, 0x870C);
  CHECK_HEX(g->regs[Reg::HIBCFG], 0x0000);
  CHECK_HEX(g->regs[Reg::COMMAND], 0x0000);                 // soft-wakeup command cleared again
  CHECK_OK(fg.restoreHibCfg(saved));
  CHECK_HEX(g->regs[Reg::HIBCFG], 0x870C);
  CHECK_OK(fg.setHibernateEnabled(false));
  CHECK_HEX(g->regs[Reg::HIBCFG], 0x070C);
  CHECK_OK(fg.shutdown(true));
  CHECK_HEX(g->regs[Reg::SHDNTIMER], 0x001E);
  CHECK((g->regs[Reg::CONFIG] & ConfigBit::SHDN) != 0);
  bool hib = true;
  g->regs[Reg::STATUS2] = 0;
  CHECK(fg.isHibernating(hib)); CHECK(!hib);
}

TEST(reset_then_reinit) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  CHECK_OK(fg.begin(Wire, c));
  CHECK_OK(fg.reset());
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) != 0);
  CHECK_OK(fg.begin(Wire, c));
  CHECK(fg.didInitialize());
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) == 0);
}

TEST(serial_number) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  uint16_t words[8] = {0};
  CHECK_OK(fg.readSerialNumber(words));
  for (int i = 0; i < 8; ++i) CHECK_HEX(words[i], 0xA000 + i);
  CHECK_HEX(g->regs[Reg::CONFIG2] & (Config2Bit::ATRATEEN | Config2Bit::DPEN),
            Config2Bit::ATRATEEN | Config2Bit::DPEN);       // restored
}

TEST(i2c_retries) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.i2cRetries = 2;
  CHECK_OK(fg.begin(Wire, c));
  uint16_t st = 0;
  g->nackCount = 2;                                         // survives (3 attempts)
  CHECK(fg.readStatus(st));
  g->nackCount = 5;                                         // does not
  CHECK(!fg.readStatus(st));
  g->nackCount = 0;
  CHECK(fg.readStatus(st));
}

TEST(learned_params_trigger) {
  FakeGauge* g = newGauge(0x4010);
  MAX17055 fg;
  CHECK_OK(fg.begin(Wire, Cfg::LiIon(2000.0f, 100.0f)));
  g->regs[Reg::CYCLES] = 0x0000;
  CHECK(!fg.learnedParamsDue());                            // first call only records the state
  g->regs[Reg::CYCLES] = 0x0030;
  CHECK(!fg.learnedParamsDue());
  g->regs[Reg::CYCLES] = 0x0040;                            // bit 6 toggled
  CHECK(fg.learnedParamsDue());
  CHECK(!fg.learnedParamsDue());
  g->regs[Reg::CYCLES] = 0x0080;                            // toggled back
  CHECK(fg.learnedParamsDue());
  g->regs[Reg::RCOMP0] = 0x1234; g->regs[Reg::TEMPCO] = 0x2345;
  g->regs[Reg::FULLCAPREP] = 3900; g->regs[Reg::FULLCAPNOM] = 3950;
  ModelGaugeM5::LearnedParams p;
  CHECK_OK(fg.readLearnedParams(p));
  CHECK(p.valid);
  CHECK_HEX(p.rcomp0, 0x1234); CHECK_HEX(p.fullCapNom, 3950); CHECK_HEX(p.cgain, 0x0400);
}

static void fillModel(ModelGaugeM5::CustomModel& m) {
  m.modelCfg = 0x8000;
  m.vEmpty = 0x965A;
  m.rcomp0 = 0x0070; m.tempCo = 0x223E;
  m.qrTable00 = 0x1050; m.qrTable10 = 0x0014; m.qrTable20 = 0x1300; m.qrTable30 = 0x0C00;
  m.optionalMask = ModelGaugeM5::OPT_QRTABLE2030 | ModelGaugeM5::OPT_FULLSOCTHR | ModelGaugeM5::OPT_LEARNCFG;
  m.fullSocThr = 0x5F05;
  m.learnCfg = 0x4486;
  for (int i = 0; i < 16; ++i) { m.table0[i] = (uint16_t)(0x1000 + i); m.table1[i] = (uint16_t)(0x2000 + i); }
  m.rCompSeg = 0x0222;
}

TEST(custom_short_model) {
  FakeGauge* g = newGauge(0x4031);
  ModelGaugeM5::CustomModel m;
  fillModel(m);
  m.fullModel = false;
  MAX17260 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.customModel = &m;
  CHECK_OK(fg.begin(Wire, c));
  CHECK_HEX(g->regs[Reg::VEMPTY], 0x965A);
  CHECK_HEX(g->regs[Reg::RCOMP0], 0x0070);
  CHECK_HEX(g->regs[Reg::TEMPCO], 0x223E);
  CHECK_HEX(g->regs[Reg::QRTABLE00], 0x1050);
  CHECK_HEX(g->regs[Reg::QRTABLE30], 0x0C00);
  CHECK_HEX(g->regs[Reg::FULLSOCTHR], 0x5F05);
  CHECK_HEX(g->regs[Reg::DESIGNCAP], 4000);
  CHECK(g->firstIndex(Reg::MODELCFG) < g->firstIndex(Reg::RCOMP0));   // QR/RCOMP after the refresh
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) == 0);
}

TEST(custom_full_model) {
  FakeGauge* g = newGauge(0x4031);
  g->regs[Reg::VFSOC] = 0x3200;                             // 50 %
  ModelGaugeM5::CustomModel m;
  fillModel(m);
  m.fullModel = true;
  MAX17260 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.customModel = &m;
  CHECK_OK(fg.begin(Wire, c));
  CHECK_HEX(g->regs[Reg::MODELACCESS1], 0);                 // locked again
  CHECK_HEX(g->regs[Reg::MODELACCESS2], 0);
  for (int i = 0; i < 16; ++i) {
    CHECK_HEX(g->modelTables[i], 0x1000 + i);
    CHECK_HEX(g->modelTables[16 + i], 0x2000 + i);
  }
  CHECK_HEX(g->modelTables[32], 0x0222);
  CHECK_HEX(g->regs[Reg::FULLCAPREP], 4000);
  CHECK_HEX(g->regs[Reg::FULLCAPNOM], 4000);
  CHECK_HEX(g->regs[Reg::DQACC], 2000);
  CHECK_HEX(g->regs[Reg::DPACC], 0x0C80);
  CHECK_HEX(g->regs[Reg::MIXCAP], 2000);                    // VFSOC 50 % of FullCapNom
  CHECK_HEX(g->regs[Reg::AVCAP], 2000);
  CHECK_HEX(g->regs[Reg::CYCLES], 0);
  CHECK_HEX(g->regs[Reg::QRTABLE20], 0x1300);
  CHECK((g->regs[Reg::CONFIG2] & Config2Bit::LDMDL) == 0);
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) == 0);
  CHECK_HEX(g->regs[Reg::HIBCFG], 0x870C);
  // locked reads return zero
  uint16_t w = 0xFFFF;
  CHECK(fg.readRegister(0x80, w)); CHECK_HEX(w, 0);

  // saved learning restored on a fresh load with the same CGain
  FakeGauge* g2 = newGauge(0x4031);
  g2->regs[Reg::VFSOC] = 0x3200;
  m.saved.valid = true; m.saved.cgain = 0x0400;
  m.saved.fullCapRep = 3800; m.saved.fullCapNom = 3900; m.saved.rcomp0 = 0x0077;
  m.saved.tempCo = 0x2222; m.saved.cycles = 150;
  MAX17260 fg2;
  CHECK_OK(fg2.begin(Wire, c));
  CHECK_HEX(g2->regs[Reg::FULLCAPREP], 3800);
  CHECK_HEX(g2->regs[Reg::FULLCAPNOM], 3900);
  CHECK_HEX(g2->regs[Reg::RCOMP0], 0x0077);
  CHECK_HEX(g2->regs[Reg::CYCLES], 150);
  CHECK_HEX(g2->regs[Reg::MIXCAP], 1950);

  // ... but ignored if the scaling differs
  FakeGauge* g3 = newGauge(0x4031);
  g3->regs[Reg::VFSOC] = 0x3200;
  m.saved.cgain = 0x0200;
  MAX17260 fg3;
  CHECK_OK(fg3.begin(Wire, c));
  CHECK_HEX(g3->regs[Reg::FULLCAPREP], 4000);
}

TEST(custom_full_model_lock_failure) {
  FakeGauge* g = newGauge(0x4031);
  g->stuckUnlocked = true;
  ModelGaugeM5::CustomModel m;
  fillModel(m);
  m.fullModel = true;
  MAX17260 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.customModel = &m;
  CHECK_STATUS(fg.begin(Wire, c), St::ModelLoadFailed);
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) != 0);      // POR must stay set on failure
  CHECK_HEX(g->regs[Reg::HIBCFG], 0x870C);                  // HibCfg still restored
}

TEST(refresh_timeout_keeps_por_and_restores_hibcfg) {
  FakeGauge* g = newGauge(0x4010);
  g->refreshNeverClears = true;
  MAX17055 fg;
  Cfg c = Cfg::LiIon(2000.0f, 100.0f);
  c.timeoutMs = 500;
  CHECK_STATUS(fg.begin(Wire, c), St::Timeout);
  CHECK((g->regs[Reg::STATUS] & StatusBit::POR) != 0);
  CHECK_HEX(g->regs[Reg::HIBCFG], 0x870C);
}

TEST(lifepo4_preset) {
  FakeGauge* g = newGauge(0x4031);
  MAX17260 fg;
  Cfg c = Cfg::LiFePO4(6000.0f, 300.0f);
  CHECK_OK(fg.begin(Wire, c));
  CHECK_HEX(g->lastWritten(Reg::MODELCFG), 0x8060);
  CHECK_HEX(g->regs[Reg::VEMPTY], (280 << 7) | 80);         // 2.8 V / 3.2 V
  Cfg bad = Cfg::LiIon(2000.0f, 100.0f);
  bad.recoveryVoltageV = 3.0f;                              // recovery below empty
  MAX17260 fg2;
  CHECK_STATUS(fg2.begin(Wire, bad), St::InvalidConfig);
}

TEST(not_begun_guards) {
  MAX17260 fg;
  CHECK_STATUS(fg.quickStart(), St::NotInitialized);
  CHECK_STATUS(fg.setHibernateEnabled(true), St::NotInitialized);
  float v;
  CHECK(!fg.readVCell(v));
}

int main() {
  RUN(max17055_ez_10mohm_unscaled);
  RUN(max17055_high_voltage_ncr_model);
  RUN(max17260_big_pack_auto_scaling_matches_v3);
  RUN(scale_planning);
  RUN(forced_scale_and_limits);
  RUN(no_reload_when_gauge_kept_state);
  RUN(cgain_self_heals_without_por);
  RUN(max17260_preserves_csel);
  RUN(max17260_r100_flag);
  RUN(device_checks);
  RUN(quick_start);
  RUN(gain_trim_and_offset_in_config);
  RUN(calibration_single_two_point_offset);
  RUN(calibration_with_scaling_keeps_scale);
  RUN(alerts_encoding);
  RUN(alert_current_scaling);
  RUN(thermistor_and_temperature_source);
  RUN(internal_temperature_default_clears_tsel);
  RUN(conversions);
  RUN(tuning_encodings);
  RUN(atrate_is_written_negative);
  RUN(power_modes);
  RUN(reset_then_reinit);
  RUN(serial_number);
  RUN(i2c_retries);
  RUN(learned_params_trigger);
  RUN(custom_short_model);
  RUN(custom_full_model);
  RUN(custom_full_model_lock_failure);
  RUN(refresh_timeout_keeps_por_and_restores_hibcfg);
  RUN(lifepo4_preset);
  RUN(not_begun_guards);
  printf("\n%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
