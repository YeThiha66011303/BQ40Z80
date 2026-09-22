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
//   Layer 5  High-level convenience   - unseal(), reset(), etc. (built from
//                                        Layers 3/4)
//
// =============================================================================

// -----------------------------------------------------------------------------
// Layer 2: Standard SBS command codes (Smart Battery Data spec, rev 1.1)
// These are stable and safe -- identical across essentially all SBS gauges.
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

    CellVoltage4    = 0x3C,
    CellVoltage3    = 0x3D,
    CellVoltage2    = 0x3E,
    CellVoltage1    = 0x3F,
};

// -----------------------------------------------------------------------------
// Layer 3: ManufacturerAccess() / ManufacturerBlockAccess() subcommands.
// -----------------------------------------------------------------------------
enum class MfaSubcommand : uint16_t {
    DeviceType      = 0x0001,  // VERIFY vs TRM
    FirmwareVersion = 0x0002,  // VERIFY vs TRM
    ChemicalId      = 0x0006,  // Confirmed vs TRM SLUUBT5C sec 17 example. READ-ONLY.
                                // Returns the loaded chemistry ID. 4xx series = LiFePO4.
                                // 0x0100 (chem 100) is the Li-ion default.
    DAStatus1       = 0x0071,  // VERIFY vs TRM (cell voltages, pack V/I/power)
    DAStatus2       = 0x0072,  // Confirmed vs TRM SLUUBT5C ToC sec 18.1.54
    DAStatus3       = 0x007B,  // Confirmed vs TI E2E forum (was WRONG as 0x0073 --
                                // 0x0073 is actually GaugingStatus1 per TRM ToC sec 18.1.55)

    PdsgFetToggle   = 0x001C,  // Confirmed vs TRM SLUUBT5C sec 18.1.13 (pre-discharge)
    FuseToggle      = 0x001D,  // Confirmed vs TRM SLUUBT5C sec 18.1.14
    PchgFetToggle   = 0x001E,  // Confirmed vs TRM SLUUBT5C sec 18.1.15 (precharge)
    ChgFetToggle    = 0x001F,  // Confirmed vs TRM SLUUBT5C sec 18.1.16
    DsgFetToggle    = 0x0020,  // Confirmed vs TRM SLUUBT5C sec 18.1.17
    Gauging         = 0x0021,  // Confirmed vs TRM SLUUBT5C sec 18.1.18
    FetControl      = 0x0022,  // Confirmed vs TRM SLUUBT5C sec 18.1.19 --
                                // exact enable/disable semantics of the FET
                                // Control command body still need to be read
                                // (not just the ToC) before relying on this.
                                // Per TI forum guidance, the Toggle commands
                                // only take effect once firmware FET
                                // auto-control is handed to manual/host
                                // control via this command; sequencing not
                                // yet confirmed here.
    LifetimeDataCollection = 0x0023, // Confirmed vs TRM SLUUBT5C sec 18.1.20
    PermanentFailure = 0x0024, // Confirmed vs TRM SLUUBT5C sec 18.1.21
    DeviceReset      = 0x0041, // Confirmed vs TRM SLUUBT5C sec 18.1.36
    OperationStatus  = 0x0054, // Confirmed vs TRM SLUUBT5C sec 18.1.41, bits confirmed
    SafetyStatus     = 0x0051, // Confirmed vs TRM SLUUBT5C sec 18.1.38, bits confirmed
    ManufacturingStatus = 0x0057, // Confirmed vs TRM SLUUBT5C sec 18.1.44, bits confirmed
    GaugingStatus3   = 0x0075, // Confirmed vs TRM SLUUBT5C sec 18.1.57, layout confirmed
    GaugingStatus    = 0x0056, // Confirmed vs TRM SLUUBT5C sec 18.1.43, bits confirmed --
                                // has the actual CHG/DSG/REST state bits
    GaugingStatus1   = 0x0073, // Confirmed vs TRM SLUUBT5C sec 18.1.55, layout confirmed --
                                // has CompRes (Cell Resistance) for cells 1-4
    GaugingStatus4   = 0x007C, // Confirmed vs TRM SLUUBT5C sec 18.1.64, layout confirmed --
                                // QMax/QMaxDOD0 for cells 5-6

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
    int16_t cellPower1;         // units VERIFY vs TRM (cW vs mW)
    int16_t cellPower2;
    int16_t cellPower3;
    int16_t cellPower4;
    int16_t allCellPower;
    int16_t averagePower;
    uint8_t raw[32];            // raw bytes as received, for debugging/verification
    uint8_t rawLen;             // actual bytes returned by device (should be 32)
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
    uint8_t raw[18];         // raw bytes as received, for debugging/verification
    uint8_t rawLen;          // actual bytes returned by device (should be 18)
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
    bool fetEn;    // Bit4:  All FET action -- 1=Enabled, 0=Disabled (THE bit we need)
    bool gaugeEn;  // Bit3:  Gas gauging enabled -- if 0, could explain
                   //        persistent Initialized=0 independent of FET_EN
    bool dsgEn;    // Bit2:  Discharge FET test activated
    bool chgEn;    // Bit1:  Charge FET test activated
    bool pchgEn;   // Bit0:  Precharge FET test activated
    uint16_t raw;
};

struct GaugingStatus3 {
    int16_t qmax1_mAh;   // QMax of Cell 1
    int16_t qmax2_mAh;   // QMax of Cell 2
    int16_t qmax3_mAh;   // QMax of Cell 3
    int16_t qmax4_mAh;   // QMax of Cell 4
    int16_t qmaxDod0_1;  // DOD0 saved for next QMax update, Cell 1 (valid only if VOK=1)
    int16_t qmaxDod0_2;  // Cell 2
    int16_t qmaxDod0_3;  // Cell 3
    int16_t qmaxDod0_4;  // Cell 4
    int16_t qmaxPassedQ_mAh; // Pass capacity since last QMax DOD value saved
    int16_t qmaxTime;    // Time since last QMax DOD saved, units of hr/16
    int16_t tempK;       // Thermal model temperature factor
    int16_t tempA;       // Thermal model temperature
    int16_t rawDod0_1;   // Raw depth of discharge, Cell 1
    int16_t rawDod0_2;   // Cell 2
    int16_t rawDod0_3;   // Cell 3
    int16_t rawDod0_4;   // Cell 4
    uint8_t rawLen;      // actual bytes returned (should be 32)
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
                    //        (the key CHARGE vs not-CHARGE indicator)
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
    int16_t compRes1;   // Cell Resistance, Cell 1 (2-10 ohm range) -- the R
    int16_t compRes2;   // value used in the CUVC compensation formula
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
    int16_t qmaxDod0_5;    // <-- QMax DOD0 for Cell 5 (what was asked for)
    int16_t rawDod0_5;
    uint8_t cellGrid5;
    int16_t raScale6;      // Cell 6
    int16_t compRes6;
    int16_t dod0_6;
    int16_t dodEoc6;
    int16_t qmax6_mAh;
    int16_t qmaxDod0_6;    // <-- QMax DOD0 for Cell 6 (what was asked for)
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

    static constexpr uint16_t kDefaultUnsealKeyWord1     = 0x0414;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultUnsealKeyWord2     = 0x3672;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultFullAccessKeyWord1 = 0xFFFF;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultFullAccessKeyWord2 = 0xFFFF;  // VERIFY vs TRM / your pack

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

    // ---- Layer 3/5: charge/discharge/precharge/pre-discharge FET control --------
    esp_err_t toggleChargeFet();
    esp_err_t toggleDischargeFet();
    esp_err_t togglePrechargeFet();     // PCHG FET, 0x001E -- precharge path only
    esp_err_t togglePreDischargeFet();  // PDSG FET, 0x001C -- pre-discharge path only
    esp_err_t toggleFuse();             // 0x001D -- WARNING: can be a one-way/destructive
    esp_err_t toggleFetEnable();

    esp_err_t toggleGaugingEnable();

    // ---- Layer 3: extended manufacturer block reads --------------------------
    esp_err_t readDAStatus1(DAStatus1 &out);
    esp_err_t getBatVoltage(uint16_t &mV);
    esp_err_t readDAStatus3(DAStatus3 &out);
    esp_err_t getCellVoltage5(uint16_t &mV);
    esp_err_t getCellVoltage6(uint16_t &mV);
    esp_err_t getDeviceType(uint16_t &type);
    esp_err_t getFirmwareVersion(uint8_t *buf, uint8_t &len, uint8_t maxLen);

    // ---- Layer 3/5: security & lifecycle --------------------------------------
    esp_err_t unseal(uint16_t keyWord1 = kDefaultUnsealKeyWord1,
                      uint16_t keyWord2 = kDefaultUnsealKeyWord2);
    esp_err_t fullAccess(uint16_t keyWord1 = kDefaultFullAccessKeyWord1,
                          uint16_t keyWord2 = kDefaultFullAccessKeyWord2);
    esp_err_t seal();
    esp_err_t reset();

    esp_err_t getOperationStatus(OperationStatus &out);

    esp_err_t getSafetyStatus(SafetyStatus &out);

    esp_err_t getManufacturingStatus(ManufacturingStatus &out);

    esp_err_t getGaugingStatus3(GaugingStatus3 &out);

    esp_err_t getGaugingStatus(GaugingStatus &out);
    esp_err_t getGaugingStatus1(GaugingStatus1 &out); // CompRes (Cell Resistance) for cells 1-4
    esp_err_t getGaugingStatus4(GaugingStatus4 &out);

    // ---- Layer 4: data flash (raw) --------------------------------------------
    esp_err_t readDataFlash(uint16_t flashAddress, uint8_t *data, uint8_t len);
    esp_err_t writeDataFlash(uint16_t flashAddress, const uint8_t *data, uint8_t len);

    esp_err_t getDaConfigurationNR(bool &nr, uint16_t &rawValue);

    esp_err_t getOvercurrentConfig(OvercurrentConfig &out);
    esp_err_t getChargingCurrentConfig(ChargingCurrentConfig &out);
    esp_err_t getCellVoltageLimits(CellVoltageLimits &out);

    esp_err_t getChemId(uint16_t &chemId);

    esp_err_t getCounterDecDelayOCD(uint8_t &seconds);
    esp_err_t getCounterDecDelayAOLD(uint8_t &seconds);
    esp_err_t getCounterDecDelayASCC(uint8_t &seconds);
    esp_err_t getCounterDecDelayASCD(uint8_t &seconds);

    // ---- Layer 1: raw primitives (exposed for advanced/custom use) ------------
    esp_err_t smbusReadWord(uint8_t command, uint16_t &value);
    esp_err_t smbusWriteWord(uint8_t command, uint16_t value);
    esp_err_t smbusReadBlock(uint8_t command, uint8_t *data, uint8_t &len, uint8_t maxLen);
    esp_err_t smbusWriteBlock(uint8_t command, const uint8_t *data, uint8_t len);

    esp_err_t manufacturerAccessSend(uint16_t subcommand);
    esp_err_t manufacturerAccessReadWord(uint16_t subcommand, uint16_t &value);
    esp_err_t manufacturerBlockRead(uint16_t subcommand, uint8_t *data, uint8_t &len, uint8_t maxLen);
    esp_err_t manufacturerBlockWrite(uint16_t subcommand, const uint8_t *data, uint8_t len);

    // ---- Bus recovery ---------------------------------------------------------
    esp_err_t recoverBus();

private:
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

    esp_err_t i2cWriteBytes(const uint8_t *data, size_t len, bool sendStop);
    esp_err_t i2cWriteThenRead(const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen);
};