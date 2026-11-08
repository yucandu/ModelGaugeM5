// Use an external NTC instead of the gauge's +-1 degC die sensor.
//
// MAX17055: NTC divider on THRM / AIN (10 k pull-up to THRM, 10 k NTC to CSP)
// MAX17260: NTC from TH to GND, 10 k by default or 100 k with thermistor100k.
//
// IMPORTANT: Config.TSel must be set for the gauge to actually use the
// thermistor. The library does that when tempSource = Thermistor (setting only
// ETHRM, the bias enable, keeps reading the die temperature).
//
// Presets: Ntc10k_B3435(), Ntc10k_B3974(), Ntc10k_B4550(). For anything else
// fill in the Thermistor struct (TGain, TOff, TCurve) from the user guide.

#include <Wire.h>
#include <MAX17260.h>

MAX17260 gauge;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::LiIon(3000.0f, 150.0f);
  cfg.tempSource = ModelGaugeM5::TempSource::Thermistor;
  cfg.thermistor = ModelGaugeM5::Thermistor::Ntc10k_B3435();
  // cfg.thermistor100k = true;     // MAX17260 with a 100 k NTC

  if (gauge.begin(Wire, cfg) != ModelGaugeM5::Status::Ok) {
    Serial.println("init failed");
    while (true) delay(1000);
  }
  // gauge.setForceThermistorBias(true);   // faster battery-removal detect, costs ~200 uA
}

void loop() {
  float t, die;
  if (gauge.readTemperature(t) && gauge.readDieTemperature(die)) {
    Serial.print("pack ");  Serial.print(t, 1);
    Serial.print(" C, die "); Serial.print(die, 1);
    Serial.println(" C");
  }
  delay(2000);
}
