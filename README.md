# ModelGaugeM5

Arduino / PlatformIO driver for the **MAX17055** and **MAX17260** single-cell
ModelGauge m5 EZ fuel gauges (Maxim, now Analog Devices).

Both parts share one register map, so they share one core
(`ModelGaugeM5`); `MAX17055` and `MAX17260` are thin classes on top of it that
carry the few differences.

```cpp
#include <MAX17260.h>

MAX17260 gauge;

void setup() {
  Wire.begin();
  auto cfg = ModelGaugeM5::Config::NcrNca(3400.0f /*mAh*/, 170.0f /*term. mA*/);
  cfg.senseResistorMohm = 10.0f;
  gauge.begin(Wire, cfg);
}

void loop() {
  float soc, mA;
  if (gauge.readSoc(soc) && gauge.readCurrent(mA)) { /* ... */ }
}
```

## Features

| | |
|---|---|
| **Models** | EZ (no characterization), short custom ini, full custom ini with OCV table (Maxim host-side implementation guide, options 1-3) |
| **Chemistries** | ModelID 0 (LiCoO2 / most Li-ion), 2 (NCR/NCA), 6 (LiFePO4); 4.2 V and 4.3-4.4 V charge voltage (`VChg`) |
| **Any shunt** | Every mA / mAh / W value is derived from the shunt you configure (1 mΩ … 1 Ω), including PCB-trace sensing |
| **CGain scaling** | Big pack on a small shunt? The library runs the gauge at `CGain = 0x400/N` (N = 1, 2, 4, 8), chosen automatically or forced, and multiplies back. Also extends the current range |
| **Calibration** | Single-point gain, offset, two-point gain+offset, shunt tolerance trim, all through CGain/COff |
| **Quick start** | Correct "wait for both `QS` and `DNR`" sequence |
| **Robust init** | POR check, `FSTAT.DNR` wait, hibernate exit/restore, write-and-verify where the guide asks for it, `HibCfg` restored even when init fails, POR left set if init fails |
| **MAX17260 high-side** | The IC auto-detects high/low-side sensing at boot; the library preserves `ModelCfg.CSEL` instead of overwriting it |
| **Alerts** | Voltage / temperature / SOC / current windows, 1 % SOC-change alert, sticky latching, correct enable ordering |
| **Temperature** | Die sensor or NTC (3 presets + custom), `TSel` handled, 100 kΩ NTC on the MAX17260 |
| **Power** | Hibernate control, soft wake, software shutdown, fuel-gauge reset |
| **Extras** | AtRate, dynamic power, SOC hold, LiFePO4 OCV keep-out window, relax threshold, learned-parameter capture/restore, unique serial number, min/max registers |
| **Portable** | Plain Arduino API (`TwoWire&`), any core, C++11. Works with a second I2C bus (`Wire1`) |

## Install

*PlatformIO:* `lib_deps = https://github.com/YOUR-USER/ModelGaugeM5.git`
*Arduino IDE:* download the repository as a ZIP and use *Sketch → Include Library → Add .ZIP Library*.

## The two chips

| | MAX17055 | MAX17260 |
|---|---|---|
| I2C address | 0x36 | 0x36 ("S" parts); "B" variant differs, use `MAX17260::detect()` |
| `DevName` | 0x4010 | 0x4031 |
| Shunt | low-side only (CSP = IC ground, CSN = load side) | low-side **or** high-side (auto-detected) |
| Thermistor pins | THRM + AIN divider | TH to GND, 10 k or 100 k (`thermistor100k`) |
| EZ init extras | also seeds `dQAcc` / `dPAcc` | not needed |
| Documented ModelIDs | 0, 6 | 0, 2, 6 |

> **ModelID 2 on the MAX17055:** the MAX17055 user guide only lists ModelID 0
> and 6; ModelID 2 (NCR/NCA) is documented for the MAX1726x family. The library
> allows it on the MAX17055 (and prints a note if `debug` is set), but it is
> outside what the supplied MAX17055 documentation describes. Test with your cell.

## Configuration

```cpp
ModelGaugeM5::Config cfg;
cfg.designCapacityMah   = 3400;   // required: label capacity
cfg.chargeTerminationMa = 170;    // 0 -> C/20 (and a debug note)
cfg.chargeVoltageV      = 4.2f;   // > 4.275 selects the 4.3-4.4 V model
cfg.modelId             = ModelGaugeM5::ModelID::NcrNca;
cfg.emptyVoltageV       = 3.0f;   // default 3.3 V
cfg.recoveryVoltageV    = 3.6f;   // default 3.88 V, must exceed emptyVoltageV
cfg.fullSocThrPercent   = 80;     // Maxim's EZ recommendation; 0 = leave at 95 %

cfg.senseResistorMohm   = 10;     // physical shunt
cfg.scale               = 0;      // 0 = auto, or 1 / 2 / 4 / 8
cfg.peakCurrentMa       = 0;      // optional, lets auto-scaling protect the current range
cfg.gainTrim            = 1.0f;   // calibration, see below
cfg.currentOffsetCounts = 0;

cfg.tempSource          = ModelGaugeM5::TempSource::Internal;
cfg.quickStartAfterInit = false;
cfg.debug               = &Serial;
```

Presets: `Config::LiIon()`, `Config::NcrNca()`, `Config::LiFePO4()`.

`LiFePO4()` uses 2.8 V empty / 3.2 V recovery. **Those two numbers are this
library's starting points, not Maxim values** (the datasheets give none for
LiFePO4). The Li-ion default recovery of 3.88 V cannot be reached by a 3.65 V
cell, so it must not be used. Maxim also recommends a custom model for LiFePO4.

### What `begin()` does

1. Validate the configuration, pick the scaling, compute `DesignCap` / `IChgTerm`.
2. Read `DevName`.
3. If `Status.POR` is set (or `forceInit`): wait for `FSTAT.DNR`, leave hibernate,
   write CGain/COff, load the model, wait for `Refresh`, re-verify CGain/COff,
   restore `HibCfg`, clear `POR`.
4. Every time: make sure CGain/COff match, apply the temperature source.

If the gauge kept its state (no POR, e.g. an ESP32 waking from deep sleep with
the gauge still powered) the model is **not** reloaded, so learned capacity is
not thrown away. `gauge.didInitialize()` tells you which happened.

All results are `ModelGaugeM5::Status`; `ModelGaugeM5::statusToString(s)` gives text.

## Shunt size, capacity and the CGain scaling

The capacity register is `5 µVh / Rsense` per count and the current register is
`1.5625 µV / Rsense`. A big pack on a small shunt needs a lot of counts:

| Pack | Shunt | Counts at N = 1 | Policy |
|---|---|---|---|
| 2 000 mAh | 10 mΩ | 4 000 | N = 1 |
| 56 780 mAh | 4 mΩ | 45 424 | N = 2 → 22 712 counts |

With `N`, the gauge is programmed with `CGain = 0x400 / N`. It then sees `1/N` of
the real current, so every capacity count is worth `N` times more, and
`DesignCap` / `IChgTerm` are written in those scaled units. The library
multiplies everything back, so `readCurrent()`, `readRepCap()`, alert
thresholds, `setAtRate()` etc. are all in real mA / mAh. SOC (%) is unaffected.

**Auto selection:** the smallest N in {1, 2, 4, 8} such that
`DesignCap ≤ maxCapacityCounts` (default `0x7000` = 28 672) **and**, if you set
`peakCurrentMa`, the peak is below 90 % of the current range
(`51.2 mV / Rsense × N`).

> The 0x7000 limit is a conservative policy of this library, derived from the
> 56 780 mAh / 4 mΩ build this library grew out of. Maxim documents the capacity
> registers as 16-bit unsigned (up to 65 535 counts) and gives no tighter limit.
> Change `maxCapacityCounts` if you disagree, or force `scale`.

Costs of N > 1:

* Resolution is N× coarser (4 mΩ, N = 2: 0.78 mA and 2.5 mAh per count).
* The CGain trim granularity becomes `1/(1024/N)` (0.2 % at N = 2, 0.8 % at N = 8).
  The library folds the rounding into the unit conversion, so readings stay
  correct, but you can't trim finer than that. COff trims offset.
* Quantities the *gauge itself* derives from the scaled current (`RCell`,
  dynamic-power outputs) see a pack that looks N× smaller. Treat them as
  indicative when N > 1 and verify against a known load.
* `RelaxCfg.LOAD` is set in volts across the shunt as seen by the gauge, so the
  effective real-current threshold is N× larger. `setRelaxLoadThresholdMa()` takes real mA and
  compensates.

`ModelGaugeM5::planScale(rMohm, capMah, peakMa)` runs the selection without a gauge.

## Calibration

`Current = ADC × CGain/0x400 + COff`, both registers live in RAM. The library
re-applies them after every POR; store the numbers somewhere and put them in
the `Config`.

```cpp
gauge.calibrateGain(reportedMa, trueMa);                  // single point
gauge.calibrateOffset(readingAtZeroLoadMa);               // offset only
gauge.calibrateTwoPoint(rep1, true1, rep2, true2);        // gain + offset
auto cal = gauge.calibration();                           // -> cfg.gainTrim, cfg.currentOffsetCounts
cfg.gainTrim = ModelGaugeM5::shuntTrim(10.0f, 9.8f);      // from a measured shunt value
```

Calibrating while running changes what a capacity/current count means by the
calibration ratio; DesignCap/IChgTerm already in the gauge are not rewritten.
For an exact result put the numbers in `Config` and let `begin()` (or
`forceInit`) apply them.

## Custom models

```cpp
ModelGaugeM5::CustomModel m;   // raw words from the ini Analog Devices generates
m.fullModel = false;           // true = long ini with OCV/X tables
...
cfg.customModel = &m;
```

See `examples/CustomModel_FromIni`. Capacity and termination current always come
from your `Config` (computed for your shunt and scale); the other words are
written verbatim. `learnedParamsDue()` + `readLearnedParams()` implement the
guide's "save when Cycles bit 6 toggles" rule, and `CustomModel::saved`
restores them (full-model flow only, as documented by Maxim).

## Things that bite (and what the library does about them)

* **Temperature source:** the thermistor is only used when `Config.TSel` is set;
  enabling `ETHRM` alone keeps reading the die. `tempSource = Thermistor` sets both (plus `Ten`).
* **AtRate must be negative.** `setAtRate()` takes a positive magnitude and writes it negative.
* **Quick start waits for both bits.** The pseudo-code in Maxim's guide uses
  `QS and DNR` in the `while`, which would leave as soon as either clears.
* **CSEL on the MAX17260.** `ModelCfg` contains the auto-detected `CSEL`;
  writing it as plain `0x8000` would overwrite it.
* **CGain is RAM.** A gauge POR resets it; `begin()` re-checks it every boot.
* **Hibernate** slows the gauge's updates to every 5.625 s. `wake()` / `restoreHibCfg()`.
* **Alert flag clears are read-modify-write**; an alert that latches between the read and the write is lost (inherent to the register).

## Not documented in the supplied datasheets / guides

These are provided but flagged, because the documentation I worked from
doesn't pin them down:

* `Cmd::FG_RESET` (0x000F) used by `reset()` - not in the user-guide tables.
* `readMppCurrentRaw()` / `readSppCurrentRaw()` / `setMinSysVoltageRaw()` - units/LSB not given, so raw.
* `readMaxPeakPowerMw()` uses the user guide's fixed 0.8 mW/LSB. `readPower()` / `readAvgPower()` use `8 µV²/Rsense` (datasheet) times N. The two agree only at 10 mΩ.
* `readRCell()` is not rescaled for Rsense.
* `MAX17260::readCsel()` returns the raw bit; which level means high-side isn't stated.

## Tests

`test/` contains a host-side simulator of the gauge (DNR timing, `Refresh` /
`LdMdl` / `QS` self-clearing, model lock, serial-number mode, NACK injection)
and a test-suite:

```
make -C test            # C++11
make -C test test17     # C++17
sh test/check_examples.sh
```

The simulator verifies **sequencing and register encoding**. It cannot tell you
whether a configuration produces accurate SOC on a real cell; that needs a
real discharge test.

## Credits / trademarks

Register behaviour follows the MAX17055 and MAX17260 datasheets, the MAX1726x
ModelGauge m5 EZ and MAX17055 user guides and the ModelGauge m5 host-side
software implementation guide (UG6595). ModelGauge is a trademark of Analog
Devices. This project is not affiliated with Analog Devices.

MIT licensed.
