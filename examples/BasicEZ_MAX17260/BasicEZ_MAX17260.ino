// MAX17260 + single cell. Works for low-side and high-side shunt layouts: the
// gauge detects which one you built at boot and the library leaves that alone.
//
// Chemistry: pick the ModelID that matches the cell.
//   Config::LiIon()   ModelID 0  most Li-ion / LiPo
//   Config::NcrNca()  ModelID 2  Panasonic-style NCR / NCA cells
//   Config::LiFePO4() ModelID 6  (a custom model is strongly recommended)

#include <Wire.h>
#include <MAX17260.h>

MAX17260 gauge;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Optional: find the gauge if it is the "B" variant at a different address.
  uint8_t addr = 0;
  if (!MAX17260::detect(Wire, addr)) {
    Serial.println("No MAX17260 found");
    while (true) delay(1000);
  }

  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::NcrNca(3400.0f, 170.0f);  // 3400 mAh 18650, C/20 term.
  cfg.senseResistorMohm = 10.0f;
  cfg.emptyVoltageV = 3.0f;       // application cut-off (Maxim default is 3.3 V)
  cfg.recoveryVoltageV = 3.6f;

  ModelGaugeM5::Status s = gauge.begin(Wire, cfg, addr);
  if (s != ModelGaugeM5::Status::Ok) {
    Serial.print("Gauge init failed: ");
    Serial.println(ModelGaugeM5::statusToString(s));
    while (true) delay(1000);
  }
}

void loop() {
  float v, ma, soc, cap, tte;
  if (gauge.readVCell(v) && gauge.readCurrent(ma) && gauge.readSoc(soc) && gauge.readRepCap(cap)) {
    Serial.print(v, 3);   Serial.print(" V  ");
    Serial.print(ma, 1);  Serial.print(" mA  ");
    Serial.print(soc, 1); Serial.print(" %  ");
    Serial.print(cap, 0); Serial.print(" mAh");
    if (gauge.readTimeToEmpty(tte) && tte == tte) {   // tte == tte: not NAN
      Serial.print("  TTE ");
      Serial.print(tte / 3600.0f, 2);
      Serial.print(" h");
    }
    Serial.println();
  }
  delay(2000);
}
