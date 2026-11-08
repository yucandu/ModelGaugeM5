// Big pack on a small shunt: the "CGain / 2" trick, done by the library.
//
// 56 780 mAh on a 4 mOhm shunt would need 45 424 capacity counts (1.25 mAh per
// count). The library instead programs CGain = 0x400 / N so the gauge sees 1/N of
// the current and capacity, and multiplies the readings back by N. Here it
// picks N = 2 (CGain 0x200, 2.5 mAh and 0.78 mA per count). Every value you
// read through the library is already in real mA / mAh.
//
// The same mechanism extends the current range: with N = 2 the 4 mOhm shunt
// measures +-25.6 A instead of +-12.8 A. Tell the library your worst-case
// current (peakCurrentMa) and it will pick N for that too.

#include <Wire.h>
#include <MAX17260.h>

MAX17260 gauge;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::NcrNca(56780.0f, 450.0f);
  cfg.senseResistorMohm = 4.0f;
  cfg.peakCurrentMa = 20000.0f;     // optional: largest current you expect
  // cfg.scale = 2;                 // or force N (1, 2, 4 or 8) instead of auto
  cfg.debug = &Serial;

  ModelGaugeM5::Status s = gauge.begin(Wire, cfg);
  if (s != ModelGaugeM5::Status::Ok) {
    Serial.print("init failed: ");
    Serial.println(ModelGaugeM5::statusToString(s));
    while (true) delay(1000);
  }
  Serial.print("N = ");            Serial.println(gauge.effectiveScale(), 3);
  Serial.print("CGain = 0x");      Serial.println(gauge.cgainRegister(), HEX);
  Serial.print("mAh per count = "); Serial.println(gauge.capacityLsbMah(), 4);
  Serial.print("mA per count = ");  Serial.println(gauge.currentLsbMa(), 5);
  Serial.print("current range = +-"); Serial.print(gauge.currentRangeMa(), 0); Serial.println(" mA");
}

void loop() {
  float ma, cap, soc;
  if (gauge.readCurrent(ma) && gauge.readRepCap(cap) && gauge.readSoc(soc)) {
    Serial.print(ma, 1);  Serial.print(" mA  ");
    Serial.print(cap, 0); Serial.print(" mAh  ");
    Serial.print(soc, 1); Serial.println(" %");
  }
  delay(1000);
}
