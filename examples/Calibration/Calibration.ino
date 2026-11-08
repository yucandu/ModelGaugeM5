// Current calibration against a reference meter.
//
// 1. Run this sketch with NO load connected and note "zero-load reading".
// 2. Apply a known load, read the DMM, put the numbers below, flash again.
// 3. The sketch prints the trim and offset: copy them into Config
//    (gainTrim / currentOffsetCounts) so they are applied at every power-up,
//    because CGain and COff live in RAM and are lost on a gauge POR.
//
// Single point:  gauge.calibrateGain(reportedMa, trueMa)
// Offset only:   gauge.calibrateOffset(reportedMaAtZeroLoad)
// Two point:     gauge.calibrateTwoPoint(rep1, true1, rep2, true2)
//
// Tip: if all you know is the shunt's measured resistance, skip the DMM and
// use  cfg.gainTrim = ModelGaugeM5::shuntTrim(nominalMohm, measuredMohm);

#include <Wire.h>
#include <MAX17055.h>

MAX17055 gauge;

// Fill these in from your meter (reported = what the gauge printed before).
const bool  DO_CALIBRATION = false;
const float REPORTED_1_MA = 100.0f,  TRUE_1_MA = 103.0f;
const float REPORTED_2_MA = 2000.0f, TRUE_2_MA = 2041.0f;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::LiIon(2000.0f, 100.0f);
  if (gauge.begin(Wire, cfg) != ModelGaugeM5::Status::Ok) {
    Serial.println("init failed");
    while (true) delay(1000);
  }

  if (DO_CALIBRATION) {
    ModelGaugeM5::Status s = gauge.calibrateTwoPoint(REPORTED_1_MA, TRUE_1_MA, REPORTED_2_MA, TRUE_2_MA);
    Serial.println(ModelGaugeM5::statusToString(s));
    ModelGaugeM5::CurrentCalibration cal = gauge.calibration();
    Serial.print("cfg.gainTrim = ");            Serial.print(cal.gainTrim, 6);       Serial.println("f;");
    Serial.print("cfg.currentOffsetCounts = "); Serial.print((int)cal.offsetCounts); Serial.println(";");
  }
}

void loop() {
  float ma, avg;
  if (gauge.readCurrent(ma) && gauge.readAvgCurrent(avg)) {
    Serial.print("current ");      Serial.print(ma, 2);
    Serial.print(" mA, average "); Serial.print(avg, 2);
    Serial.println(" mA   (zero-load reading when nothing is connected)");
  }
  delay(1000);
}
