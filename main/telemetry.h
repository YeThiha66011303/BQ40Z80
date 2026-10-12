#pragma once
// =============================================================================
// Telemetry snapshot -- READ ONLY
//
// One FreeRTOS task owns the BQ40Z80 object and is the only thing that touches
// the SMBus. It writes into a snapshot guarded by a mutex. HTTP handlers copy
// the snapshot and serialize it; they never talk to the gauge. That keeps the
// I2C transactions off the network task and out of httpd's stack.
// =============================================================================
#include "BQ40Z80.h"
#include <cstddef>
#include <cstdint>

// Values that never change while the pack is running. Read once at startup.
struct StaticInfo {
    bool     valid;
    char     manufacturer[24];
    char     deviceName[24];
    char     chemistry[16];
    uint16_t chemId;
    uint16_t deviceType;
    uint16_t serialNumber;
    uint16_t designCapacity_mAh;
    uint16_t designVoltage_mV;
    uint16_t manufactureDate;     // (year-1980)<<9 | month<<5 | day
    uint8_t  enabledProtectionsC;
    bool     enabledProtectionsCValid;
    uint16_t mfgStatusInit;          // data flash 0x4800
    bool     mfgStatusInitValid;
};

struct Telemetry {
    bool     online;              // gauge ACKed on the most recent poll
    uint32_t uptime_s;
    uint32_t pollCount;
    uint32_t errorCount;
    uint32_t lastSlowPoll_s;

    // ---- SBS word registers (fast tier) ---------------------------------
    uint16_t voltage_mV;
    int16_t  current_mA;
    int16_t  avgCurrent_mA;
    uint16_t temperature_dK;
    uint8_t  rsoc_pct;
    uint8_t  asoc_pct;
    uint16_t remainingCapacity_mAh;
    uint16_t fullChargeCapacity_mAh;
    uint16_t chargingCurrent_mA;
    uint16_t chargingVoltage_mV;
    uint16_t batteryStatus;

    // ---- SBS word registers (slow tier) ---------------------------------
    uint8_t  maxError_pct;
    uint16_t batteryMode;
    uint16_t cycleCount;
    uint16_t stateOfHealth_pct;
    uint16_t runTimeToEmpty_min;
    uint16_t avgTimeToEmpty_min;
    uint16_t avgTimeToFull_min;
    uint16_t remCapAlarm_mAh;
    uint16_t remTimeAlarm_min;
    int16_t  atRate_mA;
    uint16_t atRateTimeToFull_min;
    uint16_t atRateTimeToEmpty_min;
    uint16_t atRateOK;
    int16_t  maxTurboPower_cW;
    int16_t  susTurboPower_cW;
    int16_t  maxTurboCurrent_mA;
    int16_t  susTurboCurrent_mA;
    uint16_t turboPackR_mOhm;
    uint16_t turboSysR_mOhm;
    uint16_t turboEdv_mV;
    uint16_t noLoadRemCap_mAh;
    uint16_t sohFcc_mAh;
    uint16_t sohFcc_cWh;
    uint32_t pfStatus;
    uint32_t pfAlert;

    // ---- Block reads (fast tier) ----------------------------------------
    DAStatus1           da1;      // cells 1-4, BAT/PACK pin, currents, powers
    DAStatus2           da2;      // all 7 temperatures
    DAStatus3           da3;      // cells 5-7
    OperationStatus     op;
    SafetyStatus        safetyStatus;
    SafetyStatus        safetyAlert;
    ManufacturingStatus mfg;
    ChargingStatus      chargingStatus;
    GaugingStatus       gaugingStatus;

    // ---- Block reads (slow tier) ----------------------------------------
    GaugingStatus1 g1;            // true remaining, RaScale, CompRes 1-4
    GaugingStatus2 g2;            // grids, LStatus, DOD0, DODEOC
    GaugingStatus3 g3;            // QMax 1-4, QMax DOD0, raw DOD0
    GaugingStatus4 g4;            // cells 5-6 Impedance Track
    FilterCapacity filt;
    CBStatus       cb;
    LifetimeData1  lt1;
    LifetimeData2  lt2;
    LifetimeData3  lt3;
};

// Hand the module the gauge it should poll. Call before telemetryStart().
void   telemetryInit(BQ40Z80 *gauge);

// Read the one-shot static registers. Safe to call even if the gauge is absent;
// returns false in that case so the caller can retry once the pack appears.
bool   telemetryReadStatic();

// Spawn the polling task. Returns false if the task could not be created.
bool   telemetryStart();

// Mutex-protected copy of the latest snapshot.
void   telemetryGet(Telemetry &out);
void   telemetryGetStatic(StaticInfo &out);

// Serialize the snapshot as JSON. Returns bytes written (excluding the NUL).
size_t telemetryToJson(char *buf, size_t cap);

// The SMBus is shared between the poll task and anything else that reads the
// gauge directly, such as the boot-time configuration dump and the serial
// heartbeat. A ManufacturerAccess block read is two transactions -- the
// subcommand write, then the payload read -- so another task must not slip
// between them. Hold this around any direct gauge access.
bool telemetryLockBus(uint32_t timeoutMs);
void telemetryUnlockBus();

// ---- The one state-changing path in this firmware -------------------------
struct ResetInfo {
    bool     everTried;
    uint32_t count;       // resets issued since the ESP32 booted
    uint32_t lastAt_s;    // uptime when the last one was issued
    int      lastErr;     // esp_err_t from the last attempt
};

// Issue DeviceReset (MAC 0x0041). Takes the bus lock first so it cannot land
// in the middle of a block read. Everything else here only reads.
esp_err_t telemetryRequestReset();
void      telemetryGetResetInfo(ResetInfo &out);

// True when Mfg Status Init has FET_EN set, so a reset will not strip firmware
// FET control. Also false when that flash word could not be read at all --
// unknown is treated as unsafe on purpose.
bool telemetryResetIsSafe();
