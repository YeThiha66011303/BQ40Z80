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

    // --- Confirmed individual cell voltage commands (TI E2E forum, standard word reads) ---
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
    esp_err_t getManufactureDate(uint16_t &packedDate);   

    esp_err_t getCellVoltage1(uint16_t &mV);
    esp_err_t getCellVoltage2(uint16_t &mV);
    esp_err_t getCellVoltage3(uint16_t &mV);
    esp_err_t getCellVoltage4(uint16_t &mV);

    esp_err_t getManufacturerName(char *buf, size_t bufLen);
    esp_err_t getDeviceName(char *buf, size_t bufLen);
    esp_err_t getDeviceChemistry(char *buf, size_t bufLen);

    esp_err_t toggleChargeFet();
    esp_err_t toggleDischargeFet();
    esp_err_t togglePrechargeFet();     
    esp_err_t togglePreDischargeFet();  
    esp_err_t toggleFuse();            
    esp_err_t setFetControlMode(bool manualControl); 

    esp_err_t readDAStatus1(DAStatus1 &out);
    esp_err_t getBatVoltage(uint16_t &mV);

    esp_err_t readDAStatus3(DAStatus3 &out);
    esp_err_t getCellVoltage5(uint16_t &mV);
    esp_err_t getCellVoltage6(uint16_t &mV);
    esp_err_t getDeviceType(uint16_t &type);
    esp_err_t getFirmwareVersion(uint8_t *buf, uint8_t &len, uint8_t maxLen);


    esp_err_t unseal(uint16_t keyWord1 = kDefaultUnsealKeyWord1,
                      uint16_t keyWord2 = kDefaultUnsealKeyWord2);
    esp_err_t fullAccess(uint16_t keyWord1 = kDefaultFullAccessKeyWord1,
                          uint16_t keyWord2 = kDefaultFullAccessKeyWord2);
    esp_err_t seal();
    esp_err_t reset();

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