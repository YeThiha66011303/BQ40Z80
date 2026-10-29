#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/i2c.h"
#include "esp_err.h"

// =============================================================================
// BQ40Z80.h
//
// ESP-IDF driver for the Texas Instruments BQ40Z80 battery pack manager,
// communicating over SMBus (SBS protocol layered on I2C) with PEC enabled.
//
// LAYERING
//   Layer 1  Raw SMBus transactions   - word/block read+write, PEC calc/check
//   Layer 2  Standard SBS commands    - Voltage(), Current(), RSOC(), etc.
//                                       (Smart Battery Data spec, stable across
//                                        virtually all SBS-compliant gauges)
//   Layer 3  ManufacturerAccess() /
//            ManufacturerBlockAccess() - BQ40Z80-specific extended commands
//                                        (DAStatus blocks, security, etc.)
//   Layer 4  Data Flash access        - raw config read/write. Requires
//                                        Unsealed / Full Access mode.
//
// =============================================================================

// -----------------------------------------------------------------------------
// Layer 2: Standard SBS command codes (Smart Battery Data spec, rev 1.1)
// -----------------------------------------------------------------------------
enum class SbsCommand : uint8_t {
    ManufacturerAccess     = 0x00,
    RemainingCapacityAlarm = 0x01,
    RemainingTimeAlarm     = 0x02,
    BatteryMode            = 0x03,
    AtRate                 = 0x04,
    AtRateTimeToFull       = 0x05,
    AtRateTimeToEmpty      = 0x06,
    AtRateOK                = 0x07,
    Temperature             = 0x08,
    Voltage                 = 0x09,
    Current                 = 0x0A,
    AverageCurrent          = 0x0B,
    MaxError                = 0x0C,
    RelativeStateOfCharge   = 0x0D,
    AbsoluteStateOfCharge   = 0x0E,
    RemainingCapacity       = 0x0F,
    FullChargeCapacity      = 0x10,
    RunTimeToEmpty          = 0x11,
    AverageTimeToEmpty      = 0x12,
    AverageTimeToFull       = 0x13,
    ChargingCurrent          = 0x14,
    ChargingVoltage          = 0x15,
    BatteryStatus            = 0x16,
    CycleCount               = 0x17,
    DesignCapacity           = 0x18,
    DesignVoltage            = 0x19,
    SpecificationInfo        = 0x1A,
    ManufactureDate          = 0x1B,
    SerialNumber             = 0x1C,
    ManufacturerName         = 0x20,   // block
    DeviceName               = 0x21,   // block
    DeviceChemistry          = 0x22,   // block
    ManufacturerData         = 0x23,   // block, paired with ManufacturerBlockAccess (0x44)
    ManufacturerBlockAccess  = 0x44,   // block

    StateOfHealth            = 0x4F,   // % of design capacity
    MaxTurboPwr              = 0x59,   // cW
    SusTurboPwr              = 0x5A,   // cW
    TurboPackR               = 0x5B,   // mOhm
    TurboSysR                = 0x5C,   // mOhm
    TurboEdv                 = 0x5D,   // mV
    MaxTurboCurr             = 0x5E,   // mA
    SusTurboCurr             = 0x5F,   // mA

    CellVoltage4    = 0x3C,
    CellVoltage3    = 0x3D,
    CellVoltage2    = 0x3E,
    CellVoltage1    = 0x3F,
};

// -----------------------------------------------------------------------------
// Layer 3: ManufacturerAccess() / ManufacturerBlockAccess() subcommands.
// -----------------------------------------------------------------------------
enum class MfaSubcommand : uint16_t {
    DeviceType      = 0x0001,
    FirmwareVersion = 0x0002,
    ChemicalId      = 0x0006,  // loaded chemistry ID
    DAStatus1       = 0x0071,  // cell voltages, pack V/I/power
    DAStatus2       = 0x0072,  // temperatures
    DAStatus3       = 0x007B,  // cells 5-7

    PdsgFetToggle   = 0x001C,
    FuseToggle      = 0x001D,
    PchgFetToggle   = 0x001E,
    ChgFetToggle    = 0x001F,
    DsgFetToggle    = 0x0020,
    Gauging         = 0x0021,
    FetControl      = 0x0022,
    LifetimeDataCollection = 0x0023,
    PermanentFailure = 0x0024,
    DeviceReset      = 0x0041,
    OperationStatus  = 0x0054,
    SafetyStatus     = 0x0051,
    ManufacturingStatus = 0x0057,
    GaugingStatus3   = 0x0075,
    GaugingStatus    = 0x0056, // CHG/DSG/REST state bits
    GaugingStatus1   = 0x0073, // CompRes (cell resistance), cells 1-4
    GaugingStatus4   = 0x007C, // QMax/QMaxDOD0, cells 5-6

    // --- Added for full telemetry. All layouts confirmed vs TRM SLUUBT5C. ---
    SafetyAlert      = 0x0050, // SafetyStatus layout, set before a trip
    PfAlert          = 0x0052,
    PfStatus         = 0x0053,
    ChargingStatus   = 0x0055, // 24-bit flags + temperature range
    NoLoadRemCap     = 0x005A, // word: RemainingCapacity() with no load
    LifetimeData1    = 0x0060, // cell max/min voltages + max delta
    GaugingStatus2   = 0x0074, // grids, LStatus, DOD0, DODEOC
    LifetimeData2    = 0x0061, // max currents, temperature extremes, resets, balance time
    LifetimeData3    = 0x0062, // firmware runtime and time per temp band
    CBStatus         = 0x0076, // cell balance times and their DODs
    GaugingStatus5   = 0x007D, // cell 7 IT data (zeros on a 6S pack)
    StateOfHealthFcc = 0x0077, // SOH full charge capacity + energy
    FilterCapacity   = 0x0078, // smoothed capacity and energy
};

// -----------------------------------------------------------------------------
// Parsed result structs for extended (Layer 3) block reads
// -----------------------------------------------------------------------------
struct DAStatus1 {
    int16_t cellVoltage1_mV;
    int16_t cellVoltage2_mV;
    int16_t cellVoltage3_mV;
    int16_t cellVoltage4_mV;
    int16_t batVoltage_mV;      // internal battery-side voltage (across cell stack)
    int16_t packVoltage_mV;     // external pack terminal voltage (after CHG/DSG FETs)
    int16_t cellCurrent1_mA;
    int16_t cellCurrent2_mA;
    int16_t cellCurrent3_mA;
    int16_t cellCurrent4_mA;
    int16_t cellPower1;         // cW
    int16_t cellPower2;
    int16_t cellPower3;
    int16_t cellPower4;
    int16_t allCellPower;
    int16_t averagePower;
    uint8_t raw[32];
    uint8_t rawLen;
};

struct DAStatus3 {
    int16_t cellVoltage5_mV;
    int16_t cellCurrent5_mA;
    int16_t cellPower5;      // cW
    int16_t cellVoltage6_mV;
    int16_t cellCurrent6_mA;
    int16_t cellPower6;      // cW
    int16_t cellVoltage7_mV;
    int16_t cellCurrent7_mA;
    int16_t cellPower7;      // cW
    uint8_t raw[18];
    uint8_t rawLen;
};

// DAStatus2 (0x0072). 14 bytes, all 0.1 K. TRM sec 18.1.54.
// TS3/TS4 read about -273.2 C when no thermistor is fitted.
struct DAStatus2 {
    int16_t intTemp_dK;
    int16_t ts1Temp_dK;
    int16_t ts2Temp_dK;
    int16_t ts3Temp_dK;
    int16_t ts4Temp_dK;
    int16_t cellTemp_dK;
    int16_t fetTemp_dK;
    uint8_t rawLen;
};

// GaugingStatus2 (0x0074). 32 bytes. TRM sec 18.1.56.
// Grid values only update during DISCHARGE with [R_DIS] = 0.
struct GaugingStatus2 {
    uint8_t packGrid;
    uint8_t lStatus;           // bit3 QMax, bit2 ITEN, bits1-0 CF1:CF0
    uint8_t cellGrid[4];
    uint32_t stateTime_s;      // since last DISCHARGE/CHARGE/REST change
    int16_t dod0[4];
    int16_t dod0PassedQ_mAh;
    int16_t dod0PassedE_cWh;
    int16_t dod0Time_hr16;     // hr/16
    int16_t dodEoc[4];         // depth of discharge at end of charge
    uint8_t rawLen;

    // LStatus decode
    bool itEnabled() const      { return (lStatus >> 2) & 1; }
    bool qmaxUpdatedField() const { return (lStatus >> 3) & 1; }
    uint8_t qmaxStatus() const  { return lStatus & 0x03; } // 0=OK, 1=QMax first update, 2=QMax+Ra
};

// FilterCapacity (0x0078). 8 bytes. TRM sec 18.1.60.
// Smoothed values, reported even when [SMOOTH] = 0.
struct FilterCapacity {
    int16_t remainingCapacity_mAh;
    int16_t remainingEnergy_mWh;
    int16_t fullChargeCapacity_mAh;
    int16_t fullChargeEnergy_mWh;
    uint8_t rawLen;
};

// CBStatus (0x0076). 30 bytes. TRM sec 18.1.58.
struct CBStatus {
    int16_t balanceTime_s[7];  // calculated balancing time per cell
    int16_t balanceDod[7];     // DOD used to compute each cell's balance time
    int16_t totalDodCharge;
    uint8_t rawLen;
};

// Lifetime Data Block 1 (0x0060). 30 bytes. TRM sec 18.1.47.
struct LifetimeData1 {
    int16_t cellMaxVoltage_mV[7];
    int16_t cellMinVoltage_mV[7];
    int16_t maxDeltaCellVoltage_mV;
    uint8_t rawLen;
};

// Lifetime Data Block 2 (0x0061). 25 bytes. TRM sec 18.1.48.
// Temperatures here are single bytes, not the 0.1 K words used elsewhere.
struct LifetimeData2 {
    int16_t maxChargeCurrent_mA;
    int16_t maxDischargeCurrent_mA;
    int16_t maxAvgDsgCurrent_mA;
    int16_t maxAvgDsgPower_cW;
    int8_t  maxTempCell_C;
    int8_t  minTempCell_C;
    int8_t  maxDeltaCellTemp_C;
    int8_t  maxTempIntSensor_C;
    int8_t  minTempIntSensor_C;
    int8_t  maxTempFet_C;
    uint8_t numShutdowns;
    uint8_t numPartialResets;
    uint8_t numFullResets;
    uint8_t numWdtResets;
    uint8_t cbTimeCell_h[7];   // cumulative balance time per cell
    uint8_t rawLen;
};

// Lifetime Data Block 3 (0x0062). 16 bytes. TRM sec 18.1.49.
struct LifetimeData3 {
    uint16_t totalFwRuntime;
    uint16_t timeInUT;
    uint16_t timeInLT;
    uint16_t timeInSTL;
    uint16_t timeInRT;
    uint16_t timeInSTH;
    uint16_t timeInHT;
    uint16_t timeInOT;
    uint8_t  rawLen;
};

// GaugingStatus5 (0x007D). 15 bytes, all cell 7. TRM sec 18.1.65.
// Reads as zeros on a 6S pack; included so a 7S build needs no driver change.
struct GaugingStatus5 {
    int16_t raScale6;
    int16_t compRes6;
    int16_t dod0_6;
    int16_t dodEoc6;
    int16_t qmax6_mAh;
    int16_t qmaxDod0_6;
    int16_t rawDod0_6;
    uint8_t cellGrid6;
    uint8_t rawLen;
};

// ChargingStatus (0x0055). 24-bit flags. TRM sec 18.1.42.
struct ChargingStatus {
    uint32_t raw;
    // Charging status, bits 23-8
    bool nct;   // Bit19: near charge termination, within ~40 s
    bool ccc;   // Bit18: charging loss compensation
    bool cvr;   // Bit17: charging voltage rate of change
    bool ccr;   // Bit16: charging current rate of change
    bool vct;   // Bit15: charge termination
    bool mchg;  // Bit14: maintenance charge
    bool su;    // Bit13: suspend charge
    bool in;    // Bit12: charge inhibit
    bool hv;    // Bit11: high voltage region
    bool mv;    // Bit10: medium voltage region
    bool lv;    // Bit9:  low voltage region
    bool pv;    // Bit8:  precharge voltage region
    // Temperature range, bits 6-0
    bool ot;    // Bit6: over temperature
    bool ht;    // Bit5: high temperature
    bool sth;   // Bit4: standard temperature high
    bool rt;    // Bit3: recommended temperature
    bool stl;   // Bit2: standard temperature low
    bool lt;    // Bit1: low temperature
    bool ut;    // Bit0: under temperature
};

struct OperationStatus {
    bool iataCterm;   // Bit31: IATA charge control
    bool emShut;      // Bit29: Emergency FET shutdown
    bool cb;          // Bit28: Cell balancing status
    bool slpCc;       // Bit27: CC measurement in SLEEP mode
    bool slpAd;       // Bit26: ADC measurement in SLEEP mode
    bool smblCal;     // Bit25: Auto CC calibration when bus is low
    bool init;        // Bit24: Initialization after full reset
    bool sleepM;      // Bit23: SLEEP mode triggered via command
    bool xl;          // Bit22: 400-kHz SMBus mode
    bool calOffset;   // Bit21: Calibration output (raw CC offset data)
    bool cal;         // Bit20: Calibration output (raw ADC and CC data)
    bool autoCalM;    // Bit19: Auto CC Offset calibration in progress
    bool auth;        // Bit18: Authentication in progress
    bool led;         // Bit17: LED display on/off
    bool sdm;         // Bit16: Shutdown triggered via command
    bool sleep;       // Bit15: SLEEP mode conditions met
    bool xchg;        // Bit14: Charging disabled
    bool xdsg;        // Bit13: Discharging disabled
    bool pf;          // Bit12: PERMANENT FAILURE mode status
    bool ss;          // Bit11: SAFETY status (OR of all Safety Status bits)
    bool sdv;         // Bit10: Shutdown triggered via low battery voltage
    uint8_t sec;      // Bits9-8: SECURITY mode (0=Reserved,1=FullAccess,2=Unsealed,3=Sealed)
    bool btpInt;      // Bit7: Battery trip point interrupt
    bool fuse;        // Bit5: Fuse status
    bool pdsg;        // Bit4: Pre-discharge FET status
    bool pchg;        // Bit3: Precharge FET status
    bool chg;         // Bit2: CHG FET status
    bool dsg;         // Bit1: DSG FET status
    bool pres;        // Bit0: System present low
    uint32_t raw;     // full raw value
};

inline const char *operationStatusSecName(uint8_t sec) {
    switch (sec) {
        case 0: return "Reserved";
        case 1: return "FullAccess";
        case 2: return "Unsealed";
        case 3: return "Sealed";
        default: return "?";
    }
}

struct SafetyStatus {
    bool ocdl;   // Bit29: Overcurrent in discharge (latch)
    bool covl;   // Bit28: Cell overvoltage latch
    bool utd;    // Bit27: Undertemperature during discharge
    bool utc;    // Bit26: Undertemperature during charge
    bool pchgc;  // Bit25: Over-precharge current
    bool chgv;   // Bit24: Overcharging voltage
    bool chgc;   // Bit23: Overcharging current
    bool oc;     // Bit22: Overcharge
    bool cto;    // Bit20: Charge timeout
    bool pto;    // Bit18: Precharge timeout
    bool otf;    // Bit16: Overtemperature FET
    bool cuvc;   // Bit14: Cell undervoltage compensated
    bool otd;    // Bit13: Overtemperature during discharge
    bool otc;    // Bit12: Overtemperature during charge
    bool ascdl;  // Bit11: Short-circuit during discharge latch
    bool ascd;   // Bit10: Short-circuit during discharge
    bool asccl;  // Bit9:  Short-circuit during charge latch
    bool ascc;   // Bit8:  Short-circuit during charge
    bool aoldl;  // Bit7:  Overload during discharge latch
    bool aold;   // Bit6:  Overload during discharge
    bool ocd2;   // Bit5:  Overcurrent during discharge 2
    bool ocd1;   // Bit4:  Overcurrent during discharge 1
    bool occ2;   // Bit3:  Overcurrent during charge 2
    bool occ1;   // Bit2:  Overcurrent during charge 1
    bool cov;    // Bit1:  Cell overvoltage
    bool cuv;    // Bit0:  Cell undervoltage
    uint32_t raw;
};

struct ManufacturingStatus {
    bool calEn;    // Bit15: CALIBRATION mode
    bool ltTest;   // Bit14: LIFETIME SPEED UP mode
    bool pdsgEn;   // Bit13: Pre-discharge FET test activated
    bool ledEn;    // Bit9:  LED display enabled with push button
    bool fuseEn;   // Bit8:  Fuse action enabled
    bool bbrEn;    // Bit7:  Black box recorder enabled
    bool pfEn;     // Bit6:  Permanent failure enabled
    bool lfEn;     // Bit5:  Lifetime data collection enabled
    bool fetEn;    // Bit4:  All FET action -- 1=Enabled, 0=Disabled
    bool gaugeEn;  // Bit3:  Gas gauging enabled
    bool dsgEn;    // Bit2:  Discharge FET test activated
    bool chgEn;    // Bit1:  Charge FET test activated
    bool pchgEn;   // Bit0:  Precharge FET test activated
    uint16_t raw;
};

struct GaugingStatus3 {
    int16_t qmax1_mAh;
    int16_t qmax2_mAh;
    int16_t qmax3_mAh;
    int16_t qmax4_mAh;
    int16_t qmaxDod0_1;  // DOD0 saved for next QMax update, Cell 1 (valid only if VOK=1)
    int16_t qmaxDod0_2;
    int16_t qmaxDod0_3;
    int16_t qmaxDod0_4;
    int16_t qmaxPassedQ_mAh; // Pass capacity since last QMax DOD value saved
    int16_t qmaxTime;    // Time since last QMax DOD saved, units of hr/16
    int16_t tempK;       // Thermal model temperature factor
    int16_t tempA;       // Thermal model temperature
    int16_t rawDod0_1;
    int16_t rawDod0_2;
    int16_t rawDod0_3;
    int16_t rawDod0_4;
    uint8_t rawLen;
};

struct GaugingStatus {
    bool ocvfr;    // Bit20: OCV in flat region (during RELAX)
    bool ldmd;     // Bit19: LOAD mode (1=Constant Power, 0=Constant Current)
    bool rx;       // Bit18: Resistance update (toggles after every update)
    bool qmaxTog;  // Bit17: QMax update (toggles after every QMax update)
    bool vdq;      // Bit16: Discharge qualified for learning
    bool nsfm;     // Bit15: Negative scale factor mode
    bool slpQmax;  // Bit13: OCV update in SLEEP mode
    bool qen;      // Bit12: Impedance Track gauging enabled
    bool vok;      // Bit11: Voltages OK for QMax update (DOD saved)
    bool rDis;     // Bit10: Resistance updates disabled
    bool rest;     // Bit8:  Rest -- 1 = OCV reading taken (in/was in RELAX)
    bool cf;       // Bit7:  Condition flag (MaxError() > limit)
    bool dsg;      // Bit6:  1 = charging NOT detected, 0 = charging detected
    bool edv;      // Bit5:  End-of-discharge termination voltage reached
    bool balEn;    // Bit4:  Cell balancing possible
    bool tc;       // Bit3:  Terminate charge detected
    bool td;       // Bit2:  Terminate discharge detected
    bool fc;       // Bit1:  Fully charged
    bool fd;       // Bit0:  Fully discharged
    uint32_t raw;
};

struct GaugingStatus1 {
    int16_t trueRemQ_mAh;
    int16_t trueRemE_cWh;
    int16_t initialQ_mAh;
    int16_t initialE_cWh;
    int16_t trueFccQ_mAh;
    int16_t trueFccE_cWh;
    int16_t tSim;       // 0.1K
    int16_t tAmbient;   // 0.1K
    int16_t raScale1;
    int16_t raScale2;
    int16_t raScale3;
    int16_t raScale4;
    int16_t compRes1;   // cell resistance, 2-10 ohm range; used by CUVC
    int16_t compRes2;
    int16_t compRes3;
    int16_t compRes4;
    uint8_t rawLen;
};

struct OvercurrentConfig {
    int16_t occ1Threshold_mA;   // 0x4BE4, TI default  6000
    uint8_t occ1Delay_s;        // 0x4BE6, TI default     6
    int16_t occ2Threshold_mA;   // 0x4BE7, TI default  8000
    uint8_t occ2Delay_s;        // 0x4BE9, TI default     3
    int16_t ocd1Threshold_mA;   // 0x4BED, TI default -6000
    uint8_t ocd1Delay_s;        // 0x4BEF, TI default     6
    int16_t ocd2Threshold_mA;   // 0x4BF0, TI default -8000
    uint8_t ocd2Delay_s;        // 0x4BF2, TI default     3
};

struct ChargingCurrentConfig {
    // index 0 = voltage range Low, 1 = Med, 2 = High
    int16_t stdTempLow_mA[3];   // 0x4CA9 / 0x4CAB / 0x4CAD, TI defaults 1980 / 4004 / 2992
    int16_t stdTempHigh_mA[3];  // 0x4CB1 / 0x4CB3 / 0x4CB5, TI defaults 1980 / 4004 / 2992
    int16_t recTemp_mA[3];      // 0x4CC1 / 0x4CC3 / 0x4CC5, TI defaults 2508 / 4488 / 3520
};

struct CellVoltageLimits {
    int16_t covThresholdStdTempLow_mV;      // 0x4BCE, TI default 4300 (Li-ion)
    int16_t covThresholdStdTempHigh_mV;     // 0x4BD0, TI default 4300 (Li-ion)
    int16_t covRecoveryStdTempLow_mV;       // 0x4BD9, TI default 3900 (Li-ion)
    int16_t cuvThreshold_mV;                // 0x4BC2, TI default 2500
    int16_t cuvRecovery_mV;                 // 0x4BC5, TI default 3000
    // Charging voltage is PER TEMPERATURE RANGE. ChargingVoltage() reports whichever
    // row matches the current Temp Range, so reading only the Standard Temp rows is
    // misleading: at room temperature the gauge is usually in Rec Temp (0x4CBF),
    // whose 4100 mV default x 6 cells is where a reported 24600 mV comes from.
    int16_t chargingVoltageLowTemp_mV;      // 0x4C9F, TI default 4000 (Li-ion)
    int16_t chargingVoltageStdTempLow_mV;   // 0x4CA7, TI default 4200 (Li-ion)
    int16_t chargingVoltageStdTempHigh_mV;  // 0x4CAF, TI default 4200 (Li-ion)
    int16_t chargingVoltageHighTemp_mV;     // 0x4CB7, TI default 4000 (Li-ion)
    int16_t chargingVoltageRecTemp_mV;      // 0x4CBF, TI default 4100 (Li-ion)
    int16_t prechargeCurrent_mA;            // 0x4CC7, TI default 88 -- if ChargingCurrent()
                                             // reports 88 mA, the gauge is in PRECHARGE mode
    int16_t prechargeStartVoltage_mV;       // 0x4CCB, TI default 2500
    int16_t voltageRangeLow_mV;             // 0x4CCD, TI default 2900
    int16_t voltageRangeMed_mV;             // 0x4CCF, TI default 3600
    int16_t voltageRangeHigh_mV;            // 0x4CD1, TI default 4000
};

struct GaugingStatus4 {
    int16_t raScale5;      // Cell 5
    int16_t compRes5;
    int16_t dod0_5;
    int16_t dodEoc5;
    int16_t qmax5_mAh;
    int16_t qmaxDod0_5;
    int16_t rawDod0_5;
    uint8_t cellGrid5;
    int16_t raScale6;      // Cell 6
    int16_t compRes6;
    int16_t dod0_6;
    int16_t dodEoc6;
    int16_t qmax6_mAh;
    int16_t qmaxDod0_6;
    int16_t rawDod0_6;
    uint8_t cellGrid6;
    uint8_t rawLen;
};

// =============================================================================
// BQ40Z80 driver class
// =============================================================================
class BQ40Z80 {
public:
    static constexpr uint8_t  kDefaultAddress = 0x0B;

    static constexpr uint16_t kDefaultUnsealKeyWord1     = 0x0414;
    static constexpr uint16_t kDefaultUnsealKeyWord2     = 0x3672;
    static constexpr uint16_t kDefaultFullAccessKeyWord1 = 0xFFFF;
    static constexpr uint16_t kDefaultFullAccessKeyWord2 = 0xFFFF;

    // ---- Construction / lifecycle ------------------------------------------
    BQ40Z80(gpio_num_t sclPin, gpio_num_t sdaPin, i2c_port_t port,
            uint8_t deviceAddress = kDefaultAddress, uint32_t clockHz = 100000);
    ~BQ40Z80();

    esp_err_t begin();

    void setPecEnabled(bool enabled) { pecEnabled_ = enabled; }
    bool getPecEnabled() const { return pecEnabled_; }

    // ---- Layer 2: standard SBS getters (word-based) -------------------------
    esp_err_t getVoltage(uint16_t &mV);
    esp_err_t getCurrent(int16_t &mA);
    esp_err_t getAverageCurrent(int16_t &mA);
    esp_err_t getTemperature(uint16_t &deciKelvin);       // raw units = 0.1 K
    esp_err_t getRelativeStateOfCharge(uint8_t &percent);
    esp_err_t getAbsoluteStateOfCharge(uint8_t &percent);
    esp_err_t getRemainingCapacity(uint16_t &mAh);
    esp_err_t getFullChargeCapacity(uint16_t &mAh);
    esp_err_t getRunTimeToEmpty(uint16_t &minutes);
    esp_err_t getAverageTimeToEmpty(uint16_t &minutes);
    esp_err_t getAverageTimeToFull(uint16_t &minutes);
    esp_err_t getChargingCurrent(uint16_t &mA);
    esp_err_t getChargingVoltage(uint16_t &mV);
    esp_err_t getBatteryStatus(uint16_t &flags);
    esp_err_t getCycleCount(uint16_t &cycles);
    esp_err_t getDesignCapacity(uint16_t &mAh);
    esp_err_t setDesignCapacity(uint16_t mAh);
    esp_err_t getDesignVoltage(uint16_t &mV);
    esp_err_t getSerialNumber(uint16_t &serial);
    esp_err_t getManufactureDate(uint16_t &packedDate);   // (year-1980)<<9 | month<<5 | day

    // ---- Layer 2: individual cell voltages (confirmed direct word reads, cells 1-4) --
    esp_err_t getCellVoltage1(uint16_t &mV);
    esp_err_t getCellVoltage2(uint16_t &mV);
    esp_err_t getCellVoltage3(uint16_t &mV);
    esp_err_t getCellVoltage4(uint16_t &mV);

    // ---- Layer 2: standard SBS getters (block-based, null-terminates buf) ---
    esp_err_t getManufacturerName(char *buf, size_t bufLen);
    esp_err_t getDeviceName(char *buf, size_t bufLen);
    esp_err_t getDeviceChemistry(char *buf, size_t bufLen);

    // ---- Layer 3: extended manufacturer block reads --------------------------
    esp_err_t readDAStatus1(DAStatus1 &out);
    esp_err_t getBatVoltage(uint16_t &mV);
    esp_err_t readDAStatus3(DAStatus3 &out);
    esp_err_t getCellVoltage5(uint16_t &mV);
    esp_err_t getCellVoltage6(uint16_t &mV);
    esp_err_t getDeviceType(uint16_t &type);
    esp_err_t getFirmwareVersion(uint8_t *buf, uint8_t &len, uint8_t maxLen);

    esp_err_t getOperationStatus(OperationStatus &out);

    esp_err_t getSafetyStatus(SafetyStatus &out);

    esp_err_t getManufacturingStatus(ManufacturingStatus &out);

    esp_err_t getGaugingStatus3(GaugingStatus3 &out);

    esp_err_t getGaugingStatus(GaugingStatus &out);
    esp_err_t getGaugingStatus1(GaugingStatus1 &out); // CompRes (Cell Resistance) for cells 1-4
    esp_err_t getGaugingStatus4(GaugingStatus4 &out);

    // ---- Layer 4: data flash ----------------------------------------------------
    // writeDataFlash takes any address in 0x4000-0x5FFF. Application code must
    // not call it directly: main/df_settings.cpp is the one caller, and it only
    // writes a fixed list of settings with range checks and a read-back verify.
    // Nothing in the firmware writes on its own -- every write is a user action.
    esp_err_t readDataFlash(uint16_t flashAddress, uint8_t *data, uint8_t len);
    esp_err_t writeDataFlash(uint16_t flashAddress, const uint8_t *data, uint8_t len);

    esp_err_t getDaConfigurationNR(bool &nr, uint16_t &rawValue);

    esp_err_t getOvercurrentConfig(OvercurrentConfig &out);
    esp_err_t getChargingCurrentConfig(ChargingCurrentConfig &out);
    esp_err_t getCellVoltageLimits(CellVoltageLimits &out);

    esp_err_t getChemId(uint16_t &chemId);

    esp_err_t getCounterDecDelayOCD(uint8_t &seconds);

    // Enabled Protections C (0x4BC0). Bit 6 is OC (Overcharge).
    esp_err_t getEnabledProtectionsC(uint8_t &raw);

    // Mfg Status Init (0x4800). ManufacturingStatus() is reloaded from this on
    // every reset and on a seal -- TRM SLUUBT5C sec 16.1. The live
    // ManufacturingStatus() can therefore disagree with it, and after a reset
    // this value is the one that wins. If FET_EN (bit 4) is clear here, a reset
    // takes firmware FET control away whatever the RAM copy currently says.
    esp_err_t getMfgStatusInit(uint16_t &raw);

    // ---- Gauge reset ---------------------------------------------------------
    // DeviceReset, MAC 0x0041 (TRM SLUUBT5C sec 18.1.36). Unlike every other
    // method here this is a genuine control command, not a block read that
    // happens to need a subcommand written first. It reboots the gauge,
    // clears the latched SafetyStatus bits, bumps the lifetime reset counters,
    // and reloads ManufacturingStatus() from Mfg Status Init in data flash.
    //
    // That last part is the dangerous one: if Mfg Status Init[FET_EN] is clear,
    // the firmware stops driving the CHG/DSG/PCHG FETs after this returns and
    // the pack will not charge or discharge until it is restored in BQStudio.
    // Call getMfgStatusInit() and check bit 4 before using this.
    esp_err_t deviceReset();

    // ---- Additional telemetry (everything BQStudio's Registers view shows) ----
    esp_err_t readDAStatus2(DAStatus2 &out);            // all 7 temperatures
    esp_err_t readGaugingStatus2(GaugingStatus2 &out);  // grids, LStatus, DOD0, DODEOC
    esp_err_t readFilterCapacity(FilterCapacity &out);  // smoothed capacity/energy
    esp_err_t readCBStatus(CBStatus &out);              // cell balance times
    esp_err_t readLifetimeData1(LifetimeData1 &out);    // cell max/min voltage records
    esp_err_t readLifetimeData2(LifetimeData2 &out);    // max currents, temp extremes, resets
    esp_err_t readLifetimeData3(LifetimeData3 &out);    // runtime, time per temperature band
    esp_err_t readGaugingStatus5(GaugingStatus5 &out);  // cell 7 IT data (zeros on 6S)
    esp_err_t getChargingStatus(ChargingStatus &out);

    // SafetyAlert shares SafetyStatus's bit layout but is set before a trip,
    // so it is the early warning that something is approaching a threshold.
    esp_err_t getSafetyAlert(SafetyStatus &out);

    // PF bit layouts are not yet mapped against the TRM, so these return raw.
    esp_err_t getPfStatusRaw(uint32_t &raw);
    esp_err_t getPfAlertRaw(uint32_t &raw);

    esp_err_t getNoLoadRemainingCapacity(uint16_t &mAh);
    esp_err_t getStateOfHealthFcc(uint16_t &mAh, uint16_t &cWh);

    // ---- Additional standard SBS word reads ----------------------------------
    esp_err_t getStateOfHealth(uint16_t &percent);
    esp_err_t getBatteryMode(uint16_t &flags);
    esp_err_t getMaxError(uint8_t &percent);
    esp_err_t getRemainingCapacityAlarm(uint16_t &mAh);
    esp_err_t getRemainingTimeAlarm(uint16_t &minutes);
    esp_err_t getAtRate(int16_t &mA);
    esp_err_t getAtRateTimeToFull(uint16_t &minutes);
    esp_err_t getAtRateTimeToEmpty(uint16_t &minutes);
    esp_err_t getAtRateOK(uint16_t &ok);
    esp_err_t getMaxTurboPower(int16_t &cW);
    esp_err_t getSustainedTurboPower(int16_t &cW);
    esp_err_t getMaxTurboCurrent(int16_t &mA);
    esp_err_t getSustainedTurboCurrent(int16_t &mA);
    esp_err_t getTurboPackResistance(uint16_t &mOhm);
    esp_err_t getTurboSysResistance(uint16_t &mOhm);
    esp_err_t getTurboEdv(uint16_t &mV);
    esp_err_t getCounterDecDelayAOLD(uint8_t &seconds);
    esp_err_t getCounterDecDelayASCC(uint8_t &seconds);
    esp_err_t getCounterDecDelayASCD(uint8_t &seconds);

    // ---- Layer 1: raw primitives (read side, exposed for custom reads) --------
    esp_err_t smbusReadWord(uint8_t command, uint16_t &value);
    esp_err_t smbusReadBlock(uint8_t command, uint8_t *data, uint8_t &len, uint8_t maxLen);
    esp_err_t manufacturerAccessReadWord(uint16_t subcommand, uint16_t &value);
    esp_err_t manufacturerBlockRead(uint16_t subcommand, uint8_t *data, uint8_t &len, uint8_t maxLen);

    // ---- Bus recovery ---------------------------------------------------------
    esp_err_t recoverBus();

private:
    // Write transport: a MAC block read writes its subcommand first.
    esp_err_t smbusWriteWord(uint8_t command, uint16_t value);
    esp_err_t smbusWriteBlock(uint8_t command, const uint8_t *data, uint8_t len);
    esp_err_t manufacturerAccessSend(uint16_t subcommand);

    // SafetyAlert and SafetyStatus share a bit layout; one decoder serves both.
    esp_err_t decodeSafetyBits(uint16_t subcommand, SafetyStatus &out);

    gpio_num_t  sclPin_;
    gpio_num_t  sdaPin_;
    i2c_port_t  port_;
    uint8_t     address_;
    uint32_t    clockHz_;
    bool        pecEnabled_  = true;
    bool        initialized_ = false;

    static uint8_t crc8Smbus(const uint8_t *data, size_t len);

    esp_err_t smbusReadWordOnce(uint8_t command, uint16_t &value);
    esp_err_t smbusWriteWordOnce(uint8_t command, uint16_t value);
    esp_err_t smbusReadBlockOnce(uint8_t command, uint8_t *data, uint8_t &len, uint8_t maxLen);
    esp_err_t smbusWriteBlockOnce(uint8_t command, const uint8_t *data, uint8_t len);
};