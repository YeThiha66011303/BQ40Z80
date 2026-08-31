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
// !!! IMPORTANT - READ BEFORE USING ANYTHING MARKED "VERIFY vs TRM" !!!
// Standard SBS commands (Layer 2) are part of the Smart Battery Data spec and
// are safe to trust as-is. Anything BQ40Z80-specific -- ManufacturerAccess
// subcommand codes, unseal/full-access keys, and ALL data flash addresses --
// varies by firmware/configuration and is marked "VERIFY vs TRM" below as a
// placeholder. Confirm every one of these against YOUR device's Technical
// Reference Manual (SLUUBT5x) and/or .gg/.dffs data flash export BEFORE
// issuing any write. An incorrect data flash write can corrupt configuration
// or, in the worst case, leave the device permanently sealed/unusable.
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

    // --- Confirmed individual cell voltage commands (TI E2E forum, standard word reads) ---
    CellVoltage4    = 0x3C,
    CellVoltage3    = 0x3D,
    CellVoltage2    = 0x3E,
    CellVoltage1    = 0x3F,
};

// -----------------------------------------------------------------------------
// Layer 3: ManufacturerAccess() / ManufacturerBlockAccess() subcommands.
// PLACEHOLDERS -- confirm every value against your BQ40Z80 TRM before use.
// -----------------------------------------------------------------------------
enum class MfaSubcommand : uint16_t {
    DeviceType      = 0x0001,  // VERIFY vs TRM
    FirmwareVersion = 0x0002,  // VERIFY vs TRM
    DAStatus1       = 0x0071,  // VERIFY vs TRM (cell voltages, pack V/I/power)
    DAStatus2       = 0x0072,  // Confirmed vs TRM SLUUBT5C ToC sec 18.1.54
    DAStatus3       = 0x007B,  // Confirmed vs TI E2E forum (was WRONG as 0x0073 --
                                // 0x0073 is actually GaugingStatus1 per TRM ToC sec 18.1.55)

    // --- Confirmed against TI BQ40Z80 TRM SLUUBT5C table of contents ---
    // NOTE: these are RAM-only overrides -- state is lost on reset/seal.
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

    // Additional subcommands (SafetyAlert, SafetyStatus, PFAlert, PFStatus,
    // OperationStatus, Authenticate, Unseal-related keys, etc.) to be added
    // here once confirmed vs TRM.
};

// -----------------------------------------------------------------------------
// Parsed result structs for extended (Layer 3) block reads
// -----------------------------------------------------------------------------
struct DAStatus1 {
    // Layout cross-validated against: (1) TI E2E forum confirming DAStatus1
    // returns "cell voltages, pack voltage, bat voltage, cell currents, cell
    // powers, power, and average power" as a 32-byte block, and (2) matches
    // real-world field order/units used by multiple independent BQ40-family
    // SBS drivers. All fields are little-endian int16_t words (16 words = 32
    // bytes total, matching TI's confirmed block size).
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
    // Confirmed directly from BQ40Z80 TRM (SLUUBT5C, page 143, sec 18.1.63):
    // 18 bytes = 9 little-endian int16_t words, interleaved per cell as
    // (voltage, current, power) for cells 5, 6, 7 in that order.
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

// =============================================================================
// BQ40Z80 driver class
// =============================================================================
class BQ40Z80 {
public:
    static constexpr uint8_t  kDefaultAddress = 0x0B;

    // PLACEHOLDERS -- TI's commonly-published default unseal/full-access keys
    // are shown split into two 16-bit words below for reference, but keys are
    // frequently changed per-customer in production firmware. Do not assume
    // these are correct for your specific pack; verify before use.
    static constexpr uint16_t kDefaultUnsealKeyWord1     = 0x0414;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultUnsealKeyWord2     = 0x3672;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultFullAccessKeyWord1 = 0xFFFF;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultFullAccessKeyWord2 = 0xFFFF;  // VERIFY vs TRM / your pack

    // ---- Construction / lifecycle ------------------------------------------
    BQ40Z80(gpio_num_t sclPin, gpio_num_t sdaPin, i2c_port_t port,
            uint8_t deviceAddress = kDefaultAddress, uint32_t clockHz = 100000);
    ~BQ40Z80();

    // Installs and configures the I2C driver on the given port/pins.
    // Must be called once before any other method.
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
    esp_err_t getDesignVoltage(uint16_t &mV);
    esp_err_t getSerialNumber(uint16_t &serial);
    esp_err_t getManufactureDate(uint16_t &packedDate);   // (year-1980)<<9 | month<<5 | day

    // ---- Layer 2: individual cell voltages (confirmed direct word reads, cells 1-4) --
    esp_err_t getCellVoltage1(uint16_t &mV);
    esp_err_t getCellVoltage2(uint16_t &mV);
    esp_err_t getCellVoltage3(uint16_t &mV);
    esp_err_t getCellVoltage4(uint16_t &mV);
    // Cells 5-6 (6S pack) are not exposed as simple word commands -- they come from
    // the DAStatus3 block (0x007B). See readDAStatus3() below; byte-level field
    // layout for cells 5/6 within that block is not yet confirmed against TRM body
    // text, so raw bytes are returned in DAStatus3::raw rather than parsed values.

    // ---- Layer 2: standard SBS getters (block-based, null-terminates buf) ---
    esp_err_t getManufacturerName(char *buf, size_t bufLen);
    esp_err_t getDeviceName(char *buf, size_t bufLen);
    esp_err_t getDeviceChemistry(char *buf, size_t bufLen);

    // ---- Layer 3/5: charge/discharge/precharge/pre-discharge FET control --------
    // WARNING: these are RAM-only overrides of the firmware-controlled FETs.
    // State is lost on reset or seal(); firmware protection logic (OVP/UVP/
    // OCP/OTP etc.) still governs the FETs unless/until FET Control (0x0022)
    // hands control to manual/host mode -- semantics of that handoff are not
    // yet fully verified here (see MfaSubcommand::FetControl comment above).
    // Confirm against TRM body text before relying on this in a product; do
    // not assume the toggle methods alone will override a live protection trip.
    //
    // "Toggle" reflects the underlying MAC command semantics -- it flips
    // current FET state rather than taking an explicit on/off argument.
    // Read back actual FET state via BatteryStatus()/OperationStatus() bits
    // (OperationStatus() subcommand TBD) rather than assuming toggle succeeded.
    //
    // toggleFuse() in particular may correspond to a physically one-way/
    // destructive action (blowing an in-line fuse) depending on your board's
    // FUSE pin circuit -- do not call this without confirming the TRM body
    // text and your hardware design first.
    esp_err_t toggleChargeFet();
    esp_err_t toggleDischargeFet();
    esp_err_t togglePrechargeFet();     // PCHG FET, 0x001E -- precharge path only
    esp_err_t togglePreDischargeFet();  // PDSG FET, 0x001C -- pre-discharge path only
    esp_err_t toggleFuse();             // 0x001D -- WARNING: can be a one-way/destructive
                                         // blown-fuse action depending on hardware fuse
                                         // design; confirm TRM body text + your board's
                                         // fuse circuit before ever calling this.
    esp_err_t setFetControlMode(bool manualControl); // semantics VERIFY vs TRM body text

    // ---- Layer 3: extended manufacturer block reads --------------------------
    // DAStatus1 field layout confirmed (see struct comment above). getBatVoltage()
    // is a convenience wrapper that calls readDAStatus1() and returns just the
    // batVoltage_mV field -- this is the reliable total pack voltage reading
    // (unlike the standard Voltage() SBS command, which needs per-channel
    // calibration this chip has not yet received).
    esp_err_t readDAStatus1(DAStatus1 &out);
    esp_err_t getBatVoltage(uint16_t &mV);
    // DAStatus3 field layout confirmed (TRM SLUUBT5C page 143, sec 18.1.63).
    // getCellVoltage5()/6() are convenience wrappers, matching the pattern of
    // getCellVoltage1()-4() above but sourced from this block instead of a
    // standalone word command (cells 5-7 have no standalone SBS command).
    esp_err_t readDAStatus3(DAStatus3 &out);
    esp_err_t getCellVoltage5(uint16_t &mV);
    esp_err_t getCellVoltage6(uint16_t &mV);
    esp_err_t getDeviceType(uint16_t &type);
    esp_err_t getFirmwareVersion(uint8_t *buf, uint8_t &len, uint8_t maxLen);

    // ---- Layer 3/5: security & lifecycle --------------------------------------
    // Default-arg overloads use the placeholder keys above; pass your pack's
    // real keys explicitly once confirmed.
    esp_err_t unseal(uint16_t keyWord1 = kDefaultUnsealKeyWord1,
                      uint16_t keyWord2 = kDefaultUnsealKeyWord2);
    esp_err_t fullAccess(uint16_t keyWord1 = kDefaultFullAccessKeyWord1,
                          uint16_t keyWord2 = kDefaultFullAccessKeyWord2);
    esp_err_t seal();
    esp_err_t reset();

    // ---- Layer 4: data flash (raw) --------------------------------------------
    // Requires Unsealed (and, for some rows, Full Access) mode -- see above.
    esp_err_t readDataFlash(uint16_t flashAddress, uint8_t *data, uint8_t len);
    esp_err_t writeDataFlash(uint16_t flashAddress, const uint8_t *data, uint8_t len);

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
    // Manually bit-bangs up to 9 SCL pulses (releasing the I2C peripheral's
    // control of the pins temporarily) to force a wedged slave to release SDA,
    // then issues a manual STOP condition, then reinitializes the I2C driver.
    // Called automatically by the read/write primitives below on
    // ESP_ERR_TIMEOUT (one retry); can also be called directly if desired.
    esp_err_t recoverBus();

private:
    gpio_num_t  sclPin_;
    gpio_num_t  sdaPin_;
    i2c_port_t  port_;
    uint8_t     address_;
    uint32_t    clockHz_;
    bool        pecEnabled_  = true;
    bool        initialized_ = false;

    // SMBus CRC-8 (poly x^8 + x^2 + x^1 + 1, init 0x00) over [addr+rw, cmd, data...]
    static uint8_t crc8Smbus(const uint8_t *data, size_t len);

    // Single-attempt bodies (no retry/recovery) -- wrapped by the public
    // smbusRead*/smbusWrite* methods, which add one automatic recoverBus()
    // + retry on ESP_ERR_TIMEOUT.
    esp_err_t smbusReadWordOnce(uint8_t command, uint16_t &value);
    esp_err_t smbusWriteWordOnce(uint8_t command, uint16_t value);
    esp_err_t smbusReadBlockOnce(uint8_t command, uint8_t *data, uint8_t &len, uint8_t maxLen);
    esp_err_t smbusWriteBlockOnce(uint8_t command, const uint8_t *data, uint8_t len);

    esp_err_t i2cWriteBytes(const uint8_t *data, size_t len, bool sendStop);
    esp_err_t i2cWriteThenRead(const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen);
};