// ModelGaugeM5_Registers.h
//
// Register map and bit masks for the Maxim/Analog Devices ModelGauge m5 EZ
// fuel gauges MAX17055 and MAX17260. Both parts share the same 16-bit
// register map (page 0x00-0xFF at I2C address 0x36); the few differences are
// handled in the MAX17055 / MAX17260 classes.
//
// All registers are 16 bit, little-endian on the wire (LSB first).
//
// Licensed under the MIT License (see LICENSE).

#pragma once
#include <stdint.h>

namespace m5g {

// ---------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------
namespace Reg {
constexpr uint8_t STATUS         = 0x00;
constexpr uint8_t VALRTTH        = 0x01;
constexpr uint8_t TALRTTH        = 0x02;
constexpr uint8_t SALRTTH        = 0x03;
constexpr uint8_t ATRATE         = 0x04;
constexpr uint8_t REPCAP         = 0x05;
constexpr uint8_t REPSOC         = 0x06;
constexpr uint8_t AGE            = 0x07;
constexpr uint8_t TEMP           = 0x08;
constexpr uint8_t VCELL          = 0x09;
constexpr uint8_t CURRENT        = 0x0A;
constexpr uint8_t AVGCURRENT     = 0x0B;
constexpr uint8_t QRESIDUAL      = 0x0C;
constexpr uint8_t MIXSOC         = 0x0D;
constexpr uint8_t AVSOC          = 0x0E;
constexpr uint8_t MIXCAP         = 0x0F;
constexpr uint8_t FULLCAPREP     = 0x10;
constexpr uint8_t TTE            = 0x11;
constexpr uint8_t QRTABLE00      = 0x12;
constexpr uint8_t FULLSOCTHR     = 0x13;
constexpr uint8_t RCELL          = 0x14;
constexpr uint8_t AVGTA          = 0x16;
constexpr uint8_t CYCLES         = 0x17;
constexpr uint8_t DESIGNCAP      = 0x18;
constexpr uint8_t AVGVCELL       = 0x19;
constexpr uint8_t MAXMINTEMP     = 0x1A;
constexpr uint8_t MAXMINVOLT     = 0x1B;
constexpr uint8_t MAXMINCURR     = 0x1C;
constexpr uint8_t CONFIG         = 0x1D;
constexpr uint8_t ICHGTERM       = 0x1E;
constexpr uint8_t AVCAP          = 0x1F;
constexpr uint8_t TTF            = 0x20;
constexpr uint8_t DEVNAME        = 0x21;
constexpr uint8_t QRTABLE10      = 0x22;
constexpr uint8_t FULLCAPNOM     = 0x23;
constexpr uint8_t AIN            = 0x27;
constexpr uint8_t LEARNCFG       = 0x28;
constexpr uint8_t FILTERCFG      = 0x29;
constexpr uint8_t RELAXCFG       = 0x2A;
constexpr uint8_t MISCCFG        = 0x2B;
constexpr uint8_t TGAIN          = 0x2C;
constexpr uint8_t TOFF           = 0x2D;
constexpr uint8_t CGAIN          = 0x2E;
constexpr uint8_t COFF           = 0x2F;
constexpr uint8_t QRTABLE20      = 0x32;
constexpr uint8_t DIETEMP        = 0x34;
constexpr uint8_t FULLCAP        = 0x35;
constexpr uint8_t RCOMP0         = 0x38;
constexpr uint8_t TEMPCO         = 0x39;
constexpr uint8_t VEMPTY         = 0x3A;
constexpr uint8_t FSTAT          = 0x3D;
constexpr uint8_t TIMER          = 0x3E;
constexpr uint8_t SHDNTIMER      = 0x3F;
constexpr uint8_t QRTABLE30      = 0x42;
constexpr uint8_t RGAIN          = 0x43;
constexpr uint8_t DQACC          = 0x45;
constexpr uint8_t DPACC          = 0x46;
constexpr uint8_t CONVGCFG       = 0x49;
constexpr uint8_t VFREMCAP       = 0x4A;
constexpr uint8_t QH             = 0x4D;
constexpr uint8_t COMMAND        = 0x60;
constexpr uint8_t MODELACCESS1   = 0x62;   // write 0x0059 to unlock / 0x0000 to lock
constexpr uint8_t MODELACCESS2   = 0x63;   // write 0x00C4 to unlock / 0x0000 to lock
constexpr uint8_t OCVTABLE0      = 0x80;   // 0x80..0x8F (16 words)
constexpr uint8_t XTABLE0        = 0x90;   // 0x90..0x9F (16 words)
constexpr uint8_t RCOMPSEG       = 0xAF;
constexpr uint8_t STATUS2        = 0xB0;
constexpr uint8_t POWER          = 0xB1;
constexpr uint8_t AVGPOWER       = 0xB3;
constexpr uint8_t IALRTTH        = 0xB4;
constexpr uint8_t TTFCFG         = 0xB5;
constexpr uint8_t CVMIXCAP       = 0xB6;
constexpr uint8_t CVHALFTIME     = 0xB7;
constexpr uint8_t CGTEMPCO       = 0xB8;
constexpr uint8_t CURVE          = 0xB9;
constexpr uint8_t HIBCFG         = 0xBA;
constexpr uint8_t CONFIG2        = 0xBB;
constexpr uint8_t VRIPPLE        = 0xBC;
constexpr uint8_t RIPPLECFG      = 0xBD;
constexpr uint8_t TIMERH         = 0xBE;
constexpr uint8_t RSENSE_USER    = 0xD0;   // "RSense / UserMem3"
constexpr uint8_t SCOCVLIM       = 0xD1;
constexpr uint8_t SOCHOLD        = 0xD3;
constexpr uint8_t MAXPEAKPOWER   = 0xD4;
constexpr uint8_t SUSPEAKPOWER   = 0xD5;
constexpr uint8_t PACKRESISTANCE = 0xD6;
constexpr uint8_t SYSRESISTANCE  = 0xD7;
constexpr uint8_t MINSYSVOLTAGE  = 0xD8;
constexpr uint8_t MPPCURRENT     = 0xD9;
constexpr uint8_t SPPCURRENT     = 0xDA;
constexpr uint8_t MODELCFG       = 0xDB;
constexpr uint8_t ATQRESIDUAL    = 0xDC;
constexpr uint8_t ATTTE          = 0xDD;
constexpr uint8_t ATAVSOC        = 0xDE;
constexpr uint8_t ATAVCAP        = 0xDF;
constexpr uint8_t VFOCV          = 0xFB;
constexpr uint8_t VFSOC          = 0xFF;
}  // namespace Reg

// ---------------------------------------------------------------------------
// Bit masks
// ---------------------------------------------------------------------------

// Status (0x00)
namespace StatusBit {
constexpr uint16_t BR   = 0x8000;  // battery removal
constexpr uint16_t SMX  = 0x4000;  // SOC max alert
constexpr uint16_t TMX  = 0x2000;  // temperature max alert
constexpr uint16_t VMX  = 0x1000;  // voltage max alert
constexpr uint16_t BI   = 0x0800;  // battery insertion
constexpr uint16_t SMN  = 0x0400;  // SOC min alert
constexpr uint16_t TMN  = 0x0200;  // temperature min alert
constexpr uint16_t VMN  = 0x0100;  // voltage min alert
constexpr uint16_t DSOCI= 0x0080;  // 1% SOC change
constexpr uint16_t IMX  = 0x0040;  // current max alert
constexpr uint16_t BST  = 0x0008;  // battery absent (host-side)
constexpr uint16_t IMN  = 0x0004;  // current min alert
constexpr uint16_t POR  = 0x0002;  // power-on reset happened
// every latched alert flag (everything except POR / BST)
constexpr uint16_t ALL_ALERTS = SMX | TMX | VMX | SMN | TMN | VMN | DSOCI | IMX | IMN;
}  // namespace StatusBit

// FStat (0x3D, read only)
namespace FStatBit {
constexpr uint16_t DNR    = 0x0001;  // data not ready (clear = outputs valid)
constexpr uint16_t RELDT2 = 0x0040;  // long relaxation
constexpr uint16_t FQ     = 0x0080;  // full qualified
constexpr uint16_t EDET   = 0x0100;  // empty detected
constexpr uint16_t RELDT  = 0x0200;  // relaxed
}  // namespace FStatBit

// Config (0x1D)
namespace ConfigBit {
constexpr uint16_t TSEL   = 0x8000;  // 1 = thermistor, 0 = die temperature
constexpr uint16_t SS     = 0x4000;  // SOC alert sticky
constexpr uint16_t TS     = 0x2000;  // temperature alert sticky
constexpr uint16_t VS     = 0x1000;  // voltage alert sticky
constexpr uint16_t IS     = 0x0800;  // current alert sticky
constexpr uint16_t AINSH  = 0x0400;  // MAX17055: AIN pin shutdown / MAX17260: THSH (TH pin shutdown)
constexpr uint16_t TEN    = 0x0200;  // enable temperature channel
constexpr uint16_t TEX    = 0x0100;  // external (host-written) temperature
constexpr uint16_t SHDN   = 0x0080;  // force shutdown after ShdnTimer
constexpr uint16_t COMMSH = 0x0040;  // communication shutdown
constexpr uint16_t ETHRM  = 0x0010;  // enable thermistor bias
constexpr uint16_t FTHRM  = 0x0008;  // force thermistor bias on
constexpr uint16_t AEN    = 0x0004;  // alerts reach the ALRT pin
constexpr uint16_t BEI    = 0x0002;  // battery insertion alert
constexpr uint16_t BER    = 0x0001;  // battery removal alert
}  // namespace ConfigBit

// Config2 (0xBB)
namespace Config2Bit {
constexpr uint16_t ATRATEEN = 0x2000;
constexpr uint16_t DPEN     = 0x1000;
constexpr uint16_t POWR_MASK= 0x0F00;
constexpr uint16_t DSOCEN   = 0x0080;
constexpr uint16_t TALRTEN  = 0x0040;
constexpr uint16_t LDMDL    = 0x0020;
constexpr uint16_t CPMODE   = 0x0002;
}  // namespace Config2Bit

// ModelCfg (0xDB)
namespace ModelCfgBit {
constexpr uint16_t REFRESH       = 0x8000;
constexpr uint16_t R100          = 0x2000;  // MAX17260 only: 100 kOhm thermistor
constexpr uint16_t VCHG          = 0x0400;  // 1 = charge voltage above 4.25 V (4.3-4.4 V)
constexpr uint16_t MODELID_MASK  = 0x00F0;
constexpr uint8_t  MODELID_SHIFT = 4;
constexpr uint16_t CSEL          = 0x0004;  // MAX17260 only: high-side/low-side (auto-detected at boot)
}  // namespace ModelCfgBit

// MiscCfg (0x2B)
namespace MiscCfgBit {
constexpr uint16_t QS         = 0x0400;  // quick start
constexpr uint16_t SACFG_MASK = 0x0003;  // which SOC register drives SOC alerts
}  // namespace MiscCfgBit

// LearnCfg (0x28): learn stage in bits 6:4
namespace LearnCfgBit {
constexpr uint16_t LS_MASK  = 0x0070;
constexpr uint8_t  LS_SHIFT = 4;
}  // namespace LearnCfgBit

// Status2 (0xB0)
namespace Status2Bit {
constexpr uint16_t ATRATE_READY = 0x2000;
constexpr uint16_t DP_READY     = 0x1000;
constexpr uint16_t SN_READY     = 0x0100;
constexpr uint16_t FULLDET      = 0x0020;
constexpr uint16_t HIB          = 0x0002;
}  // namespace Status2Bit

// HibCfg (0xBA)
namespace HibCfgBit {
constexpr uint16_t ENHIB = 0x8000;
}  // namespace HibCfgBit

// Command register (0x60)
namespace Cmd {
constexpr uint16_t CLEAR       = 0x0000;
constexpr uint16_t SOFT_WAKEUP = 0x0090;
// Fuel-gauge reset. NOT listed in the MAX17055 / MAX1726x user-guide tables
// supplied with this library; taken from solarpowerbankv3.cpp, which uses it
// to force a POR on a MAX17260.
constexpr uint16_t FG_RESET    = 0x000F;
}  // namespace Cmd

// Model access unlock/lock keys (custom model loading, 0x62 / 0x63)
namespace ModelAccess {
constexpr uint16_t UNLOCK1 = 0x0059;
constexpr uint16_t UNLOCK2 = 0x00C4;
constexpr uint16_t LOCK    = 0x0000;
}  // namespace ModelAccess

// ---------------------------------------------------------------------------
// Fixed scale factors (nominal, before the sense-resistor / CGain scaling)
// ---------------------------------------------------------------------------
namespace Scale {
constexpr float VOLTAGE_V_PER_LSB   = 78.125e-6f;      // VCell, AvgVCell, VFOCV
constexpr float PERCENT_PER_LSB     = 1.0f / 256.0f;   // all SOC registers
constexpr float TEMP_C_PER_LSB      = 1.0f / 256.0f;   // Temp, AvgTA, DieTemp
constexpr float TIME_S_PER_LSB      = 5.625f;          // TTE, TTF, AtTTE
constexpr float RESISTANCE_OHM_PER_LSB = 1.0f / 4096.0f; // RCell, Pack/SysResistance
constexpr float CYCLES_PER_LSB      = 0.01f;           // Cycles register: 1 % of a cycle
constexpr float TIMER_S_PER_LSB     = 0.17578125f;     // Timer (0x3E) + TimerH (0xBE) as 32 bit
constexpr float VALRT_V_PER_LSB     = 0.020f;          // VAlrtTh and MaxMinVolt byte fields
constexpr float SENSE_V_PER_COUNT   = 1.5625e-6f;      // current ADC LSB across the shunt
constexpr float SENSE_V_PER_COUNT_CAP = 5.0e-6f;       // capacity LSB (V*h) across the shunt
constexpr float SENSE_V_PER_COUNT_IALRT = 0.4e-3f;     // IAlrtTh / MaxMinCurr byte fields
constexpr float SENSE_FULLSCALE_V   = 51.2e-3f;        // +-51.2 mV current range
constexpr float POWER_V2_PER_LSB    = 8.0e-6f;         // Power/AvgPower: 8 uV^2 / Rsense
constexpr float PEAKPOWER_MW_PER_LSB= 0.8f;            // MaxPeakPower / SusPeakPower
constexpr float PACKRES_OHM_PER_LSB = 0.244140625e-3f; // PackResistance / SysResistance
}  // namespace Scale

}  // namespace m5g
