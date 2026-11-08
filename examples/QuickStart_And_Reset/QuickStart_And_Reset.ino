// Quick start and software reset.
//
// Quick start makes the gauge re-sample the open-circuit voltage and redo its
// initial SOC estimate. Maxim recommends it only when the first estimate was
// disturbed (battery protection tripped or a voltage transient during
// start-up). Call it when the cell has relaxed and nothing is charging, or the
// new estimate will be wrong in a different way.
//
// Config::quickStartAfterInit = true does it automatically after every model
// load, which is what you want if your supply is not stable when the MCU
// boots.

#include <Wire.h>
#include <MAX17260.h>

MAX17260 gauge;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::LiIon(3000.0f, 150.0f);
  // cfg.quickStartAfterInit = true;
  if (gauge.begin(Wire, cfg) != ModelGaugeM5::Status::Ok) {
    Serial.println("init failed");
    while (true) delay(1000);
  }

  // Manual quick start (blocks up to 2 s):
  ModelGaugeM5::Status s = gauge.quickStart();
  Serial.print("quick start: ");
  Serial.println(ModelGaugeM5::statusToString(s));

  // Full software reset of the gauge, then reload the configuration:
  //   gauge.reset();
  //   gauge.begin(Wire, cfg);
}

void loop() {
  float soc;
  if (gauge.readSoc(soc)) { Serial.print(soc, 1); Serial.println(" %"); }
  delay(1000);
}
