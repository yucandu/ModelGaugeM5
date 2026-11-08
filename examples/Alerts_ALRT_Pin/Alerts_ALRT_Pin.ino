// Use the gauge's open-drain ALRT pin as an interrupt source.
// ALRT needs a pull-up (the internal one of most MCUs is fine for a hobby board).

#include <Wire.h>
#include <MAX17260.h>

const int ALRT_PIN = 4;          // change to suit your board

MAX17260 gauge;
volatile bool alertSeen = false;
void onAlert() { alertSeen = true; }

void setup() {
  Serial.begin(115200);
  Wire.begin();

  ModelGaugeM5::Config cfg = ModelGaugeM5::Config::LiIon(3000.0f, 150.0f);
  if (gauge.begin(Wire, cfg) != ModelGaugeM5::Status::Ok) {
    Serial.println("init failed");
    while (true) delay(1000);
  }

  ModelGaugeM5::AlertConfig a;
  a.voltage = true;   a.vMinV = 3.2f;   a.vMaxV = 4.25f;
  a.temperature = true; a.tMinC = 0.0f; a.tMaxC = 45.0f;
  a.soc = true;       a.sMinPct = 15.0f; a.sMaxPct = 100.0f;
  a.socChange = false;
  a.sticky = true;    // stay latched until we clear it
  gauge.configureAlerts(a);

  pinMode(ALRT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ALRT_PIN), onAlert, FALLING);
}

void loop() {
  if (alertSeen) {
    alertSeen = false;
    uint16_t flags = gauge.alertFlags();
    if (flags & ModelGaugeM5::ALERT_VOLT_MIN) Serial.println("Voltage low");
    if (flags & ModelGaugeM5::ALERT_VOLT_MAX) Serial.println("Voltage high");
    if (flags & ModelGaugeM5::ALERT_TEMP_MIN) Serial.println("Too cold");
    if (flags & ModelGaugeM5::ALERT_TEMP_MAX) Serial.println("Too hot");
    if (flags & ModelGaugeM5::ALERT_SOC_MIN)  Serial.println("SOC low");
    gauge.clearAlerts(flags);     // releases ALRT
  }
  delay(100);
}
