// MAX17055 + single Li-ion cell, ModelGauge m5 EZ configuration.
//
// Wiring: SDA/SCL to the MCU, BATT to cell +, shunt between CSP (battery
// side, also the IC's ground) and CSN (load side), 0.1 uF on BATT, 0.47 uF on REG.

#include <Wire.h>
#include <MAX17055.h>

MAX17055 gauge;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // 2000 mAh pack, charger terminates at 100 mA, 4.2 V cell, 10 mOhm shunt.
  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::LiIon(2000.0f, 100.0f);
  cfg.senseResistorMohm = 10.0f;
  cfg.debug = &Serial;               // optional: see what the library is doing

  ModelGaugeM5::Status s = gauge.begin(Wire, cfg);
  if (s != ModelGaugeM5::Status::Ok) {
    Serial.print("Gauge init failed: ");
    Serial.println(ModelGaugeM5::statusToString(s));
    while (true) delay(1000);
  }
  Serial.println(gauge.didInitialize() ? "Model loaded" : "Gauge already running");
}

void loop() {
  ModelGaugeM5::Snapshot snap;
  if (gauge.readSnapshot(snap)) {
    Serial.print("V=");   Serial.print(snap.vCellV, 3);
    Serial.print(" I=");  Serial.print(snap.currentMa, 1);
    Serial.print("mA SOC="); Serial.print(snap.socPct, 1);
    Serial.print("% Cap="); Serial.print(snap.repCapMah, 0);
    Serial.print("/");     Serial.print(snap.fullCapRepMah, 0);
    Serial.print("mAh T="); Serial.print(snap.tempC, 1);
    Serial.println("C");
  } else {
    Serial.println("read failed");
  }
  delay(2000);
}
