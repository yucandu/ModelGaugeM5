// Load a characterized battery model (the .ini Analog Devices generates for your
// cell) instead of the EZ model. Recommended for LiFePO4 and for NCR/NCA cells
// where EZ is not accurate enough.
//
// Two ini flavours are supported:
//   short ini  -> fullModel = false   (QRTable / RCOMP0 / TempCo, Option 2)
//   long ini   -> fullModel = true    (adds the 32-word OCV/X tables, Option 3)
//
// Copy the raw hex words from the ini. DesignCap and IChgTerm are NOT taken
// from it: the library computes them from your capacity / termination current
// and your shunt. The ini's other words are only valid for the sense resistor the
// characterization used - check which one that was.

#include <Wire.h>
#include <MAX17260.h>

MAX17260 gauge;
ModelGaugeM5::LearnedParams savedParams;   // keep in NVS / RTC memory across power loss

void setup() {
  Serial.begin(115200);
  Wire.begin();

  ModelGaugeM5::CustomModel m;
  m.fullModel = false;
  m.modelCfg  = 0x8000;
  m.vEmpty    = 0xA561;
  m.rcomp0    = 0x004D;
  m.tempCo    = 0x223E;
  m.qrTable00 = 0x1050;
  m.qrTable10 = 0x2012;
  // Optional ini entries: set the matching OPT_ bit when you provide them.
  // m.optionalMask |= ModelGaugeM5::OPT_QRTABLE2030;
  // m.qrTable20 = 0x....; m.qrTable30 = 0x....;

  // Long format only:
  // m.fullModel = true;
  // const uint16_t t0[16] = {0x....};  memcpy(m.table0, t0, sizeof(t0));
  // const uint16_t t1[16] = {0x....};  memcpy(m.table1, t1, sizeof(t1));
  // m.rCompSeg = 0x....;
  // m.modelCfgBeforeLoad = 0x0060;     // LiFePO4 only, per the user guide
  // m.saved = savedParams;             // restore learning after a power loss

  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::LiIon(2600.0f, 130.0f);
  cfg.customModel = &m;
  cfg.debug = &Serial;

  ModelGaugeM5::Status s = gauge.begin(Wire, cfg);
  Serial.println(ModelGaugeM5::statusToString(s));
}

void loop() {
  // Maxim's guide: save the learned parameters whenever Cycles bit 6 toggles
  // (about every 64 % of cycled charge).
  if (gauge.learnedParamsDue()) {
    if (gauge.readLearnedParams(savedParams) == ModelGaugeM5::Status::Ok) {
      Serial.println("learned parameters captured - write savedParams to non-volatile memory");
    }
  }
  float soc;
  if (gauge.readSoc(soc)) { Serial.print(soc, 1); Serial.println(" %"); }
  delay(5000);
}
