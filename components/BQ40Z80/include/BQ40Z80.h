#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/i2c.h"
#include "esp_err.h"

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
};

enum class MfaSubcommand : uint16_t {
    DeviceType      = 0x0001,  
    FirmwareVersion = 0x0002,  
    DAStatus1       = 0x0071,  
    DAStatus3       = 0x0073,   

    // NOTE: these are RAM-only overrides -- state is lost on reset/seal.
    PdsgFetToggle   = 0x001C,  
    FuseToggle      = 0x001D,  
    PchgFetToggle   = 0x001E,  
    ChgFetToggle    = 0x001F,  
    DsgFetToggle    = 0x0020,  
    Gauging         = 0x0021,  
    FetControl      = 0x0022,  
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

};

// -----------------------------------------------------------------------------
// Parsed result structs for extended (Layer 3) block reads
// -----------------------------------------------------------------------------
struct DAStatus1 {
    // confirm field count, order, and units (mV/mA/cW)
    // against the DAStatus1 table in TRM before trusting these values.
    int16_t cellVoltage_mV[7];  // index 0 = Cell1 ... last valid cell per pack config
    int16_t packVoltage_mV;
    int16_t packCurrent_mA;
    int16_t power;              // units VERIFY vs TRM (cW vs mW)
    uint8_t rawLen;             // actual bytes returned by device
};

struct DAStatus3 {
    // field layout not yet mapped. Raw bytes preserved so
    // nothing is lost; parse once the TRM table is confirmed.
    uint8_t raw[32];
    uint8_t rawLen;
};

// =============================================================================
// BQ40Z80 driver class
// =============================================================================
class BQ40Z80 {
public:
    static constexpr uint8_t  kDefaultAddress = 0x0B;

    // TI's commonly-published default unseal/full-access keys
    // are shown split into two 16-bit words below for reference, but keys are
    // frequently changed per-customer in production firmware. Do not assume
    // these are correct for your specific pack; verify before use.
    static constexpr uint16_t kDefaultUnsealKeyWord1     = 0x0414;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultUnsealKeyWord2     = 0x3672;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultFullAccessKeyWord1 = 0xFFFF;  // VERIFY vs TRM / your pack
    static constexpr uint16_t kDefaultFullAccessKeyWord2 = 0xFFFF;  // VERIFY vs TRM / your pack

    BQ40Z80(gpio_num_t sclPin, gpio_num_t sdaPin, i2c_port_t port,
            uint8_t deviceAddress = kDefaultAddress, uint32_t clockHz = 100000);
    ~BQ40Z80();

    esp_err_t begin();

    void setPecEnabled(bool enabled) { pecEnabled_ = enabled; }
    bool getPecEnabled() const { return pecEnabled_; }

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

    esp_err_t getManufacturerName(char *buf, size_t bufLen);
    esp_err_t getDeviceName(char *buf, size_t bufLen);
    esp_err_t getDeviceChemistry(char *buf, size_t bufLen);

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
    esp_err_t readDAStatus1(DAStatus1 &out);
    esp_err_t readDAStatus3(DAStatus3 &out);
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

    esp_err_t i2cWriteBytes(const uint8_t *data, size_t len, bool sendStop);
    esp_err_t i2cWriteThenRead(const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen);
};