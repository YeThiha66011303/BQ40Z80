#include "BQ40Z80.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <cstring>

// =============================================================================
// Construction / lifecycle
// =============================================================================
BQ40Z80::BQ40Z80(gpio_num_t sclPin, gpio_num_t sdaPin, i2c_port_t port,
                  uint8_t deviceAddress, uint32_t clockHz)
    : sclPin_(sclPin), sdaPin_(sdaPin), port_(port),
      address_(deviceAddress), clockHz_(clockHz) {}

BQ40Z80::~BQ40Z80() {
    if (initialized_) {
        i2c_driver_delete(port_);
    }
}

esp_err_t BQ40Z80::begin() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdaPin_;
    conf.scl_io_num = sclPin_;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clockHz_;

    esp_err_t err = i2c_param_config(port_, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
    if (err != ESP_OK) return err;

    initialized_ = true;
    return ESP_OK;
}

// =============================================================================
// Layer 1: SMBus CRC-8 (PEC) -- poly x^8+x^2+x^1+1 (0x07), init 0x00
// =============================================================================
uint8_t BQ40Z80::crc8Smbus(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// =============================================================================
// Layer 1: word read, e.g. Voltage(), Current(), Temperature(), RSOC(), etc.
// =============================================================================
esp_err_t BQ40Z80::smbusReadWordOnce(uint8_t command, uint16_t &value) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    uint8_t rx[3] = {0}; // [0]=data low, [1]=data high, [2]=PEC (if enabled)

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((address_ << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_start(cmd); // repeated start
    i2c_master_write_byte(cmd, (uint8_t)((address_ << 1) | I2C_MASTER_READ), true);
    i2c_master_read_byte(cmd, &rx[0], I2C_MASTER_ACK);
    if (pecEnabled_) {
        i2c_master_read_byte(cmd, &rx[1], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &rx[2], I2C_MASTER_NACK);
    } else {
        i2c_master_read_byte(cmd, &rx[1], I2C_MASTER_NACK);
    }
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;

    if (pecEnabled_) {
        uint8_t pecInput[5] = {
            (uint8_t)((address_ << 1) | I2C_MASTER_WRITE),
            command,
            (uint8_t)((address_ << 1) | I2C_MASTER_READ),
            rx[0], rx[1]
        };
        uint8_t expectedPec = crc8Smbus(pecInput, sizeof(pecInput));
        if (expectedPec != rx[2]) {
            return ESP_ERR_INVALID_CRC;
        }
    }

    value = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    return ESP_OK;
}

esp_err_t BQ40Z80::smbusWriteWordOnce(uint8_t command, uint16_t value) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    uint8_t dataLow  = (uint8_t)(value & 0xFF);
    uint8_t dataHigh = (uint8_t)((value >> 8) & 0xFF);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((address_ << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_write_byte(cmd, dataLow, true);
    i2c_master_write_byte(cmd, dataHigh, true);

    if (pecEnabled_) {
        uint8_t pecInput[4] = {
            (uint8_t)((address_ << 1) | I2C_MASTER_WRITE),
            command, dataLow, dataHigh
        };
        uint8_t pec = crc8Smbus(pecInput, sizeof(pecInput));
        i2c_master_write_byte(cmd, pec, true);
    }
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}

// =============================================================================
// Layer 1: block read/write, e.g. ManufacturerName(), ManufacturerData(), etc.
// =============================================================================
esp_err_t BQ40Z80::smbusReadBlockOnce(uint8_t command, uint8_t *data, uint8_t &len, uint8_t maxLen) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    uint8_t countByte = 0;
    uint8_t buf[34] = {0}; // up to 32 data bytes + count byte handled separately + PEC
    uint8_t pec = 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((address_ << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_start(cmd); // repeated start
    i2c_master_write_byte(cmd, (uint8_t)((address_ << 1) | I2C_MASTER_READ), true);
    i2c_master_read_byte(cmd, &countByte, I2C_MASTER_ACK);

    for (int i = 0; i < 32; i++) {
        bool isLastByte = (i == 31) && !pecEnabled_;
        i2c_master_read_byte(cmd, &buf[i], isLastByte ? I2C_MASTER_NACK : I2C_MASTER_ACK);
    }
    if (pecEnabled_) {
        i2c_master_read_byte(cmd, &pec, I2C_MASTER_NACK);
    }
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;

    if (countByte > 32) countByte = 32; // guard against garbage/corrupt count byte

    if (pecEnabled_) {
        uint8_t pecInput[4 + 32] = {0};
        pecInput[0] = (uint8_t)((address_ << 1) | I2C_MASTER_WRITE);
        pecInput[1] = command;
        pecInput[2] = (uint8_t)((address_ << 1) | I2C_MASTER_READ);
        pecInput[3] = countByte;
        for (int i = 0; i < countByte; i++) pecInput[4 + i] = buf[i];

        uint8_t expectedPec = crc8Smbus(pecInput, 4 + countByte);
        if (expectedPec != pec) {
            return ESP_ERR_INVALID_CRC;
        }
    }

    uint8_t copyLen = (countByte < maxLen) ? countByte : maxLen;
    for (int i = 0; i < copyLen; i++) data[i] = buf[i];
    len = copyLen;

    return ESP_OK;
}

esp_err_t BQ40Z80::smbusWriteBlockOnce(uint8_t command, const uint8_t *data, uint8_t len) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (len > 32) return ESP_ERR_INVALID_ARG;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((address_ << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_write_byte(cmd, len, true); // byte count
    i2c_master_write(cmd, (uint8_t *)data, len, true);

    if (pecEnabled_) {
        uint8_t pecInput[2 + 1 + 32] = {0};
        pecInput[0] = (uint8_t)((address_ << 1) | I2C_MASTER_WRITE);
        pecInput[1] = command;
        pecInput[2] = len;
        for (int i = 0; i < len; i++) pecInput[3 + i] = data[i];
        uint8_t pec = crc8Smbus(pecInput, 3 + len);
        i2c_master_write_byte(cmd, pec, true);
    }
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t BQ40Z80::recoverBus() {
    if (initialized_) {
        i2c_driver_delete(port_);
        initialized_ = false;
    }

    gpio_config_t ioConf = {};
    ioConf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    ioConf.pull_up_en = GPIO_PULLUP_DISABLE; // external pull-ups already present
    ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioConf.intr_type = GPIO_INTR_DISABLE;
    ioConf.pin_bit_mask = (1ULL << sclPin_) | (1ULL << sdaPin_);
    gpio_config(&ioConf);

    gpio_set_level(sclPin_, 1);
    gpio_set_level(sdaPin_, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    bool sdaFreed = false;
    for (int i = 0; i < 9; i++) {
        if (gpio_get_level(sdaPin_) == 1) {
            sdaFreed = true;
            break; // slave already released the bus
        }
        gpio_set_level(sclPin_, 0);
        esp_rom_delay_us(5);
        gpio_set_level(sclPin_, 1);
        esp_rom_delay_us(5);
    }

    gpio_set_level(sdaPin_, 0);
    esp_rom_delay_us(5);
    gpio_set_level(sclPin_, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sdaPin_, 1);
    esp_rom_delay_us(5);

    esp_err_t err = begin();
    if (err != ESP_OK) return err;

    return sdaFreed ? ESP_OK : ESP_ERR_INVALID_STATE; // report if recovery fully succeeded
}

// =============================================================================
// Public Layer 1 wrappers: one automatic recoverBus() + retry on
// =============================================================================
esp_err_t BQ40Z80::smbusReadWord(uint8_t command, uint16_t &value) {
    esp_err_t err = smbusReadWordOnce(command, value);
    if (err == ESP_ERR_TIMEOUT) {
        recoverBus();
        err = smbusReadWordOnce(command, value);
    }
    return err;
}

esp_err_t BQ40Z80::smbusWriteWord(uint8_t command, uint16_t value) {
    esp_err_t err = smbusWriteWordOnce(command, value);
    if (err == ESP_ERR_TIMEOUT) {
        recoverBus();
        err = smbusWriteWordOnce(command, value);
    }
    return err;
}

esp_err_t BQ40Z80::smbusReadBlock(uint8_t command, uint8_t *data, uint8_t &len, uint8_t maxLen) {
    esp_err_t err = smbusReadBlockOnce(command, data, len, maxLen);
    if (err == ESP_ERR_TIMEOUT) {
        recoverBus();
        err = smbusReadBlockOnce(command, data, len, maxLen);
    }
    return err;
}

esp_err_t BQ40Z80::smbusWriteBlock(uint8_t command, const uint8_t *data, uint8_t len) {
    esp_err_t err = smbusWriteBlockOnce(command, data, len);
    if (err == ESP_ERR_TIMEOUT) {
        recoverBus();
        err = smbusWriteBlockOnce(command, data, len);
    }
    return err;
}

esp_err_t BQ40Z80::manufacturerAccessSend(uint16_t subcommand) {
    return smbusWriteWord(static_cast<uint8_t>(SbsCommand::ManufacturerAccess), subcommand);
}

esp_err_t BQ40Z80::manufacturerAccessReadWord(uint16_t subcommand, uint16_t &value) {
    esp_err_t err = manufacturerAccessSend(subcommand);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(2)); // small settle delay between write and read-back
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::ManufacturerAccess), value);
}

esp_err_t BQ40Z80::manufacturerBlockRead(uint16_t subcommand, uint8_t *data, uint8_t &len, uint8_t maxLen) {
    uint8_t subBytes[2] = {
        (uint8_t)(subcommand & 0xFF),
        (uint8_t)((subcommand >> 8) & 0xFF)
    };
    esp_err_t err = smbusWriteBlock(static_cast<uint8_t>(SbsCommand::ManufacturerBlockAccess), subBytes, 2);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(2)); // small settle delay before reading the payload back

    return smbusReadBlock(static_cast<uint8_t>(SbsCommand::ManufacturerData), data, len, maxLen);
}

esp_err_t BQ40Z80::manufacturerBlockWrite(uint16_t subcommand, const uint8_t *data, uint8_t len) {
    if (len > 30) return ESP_ERR_INVALID_ARG; // 2 subcommand bytes + data must fit in 32
    uint8_t payload[32] = {0};
    payload[0] = (uint8_t)(subcommand & 0xFF);
    payload[1] = (uint8_t)((subcommand >> 8) & 0xFF);
    for (int i = 0; i < len; i++) payload[2 + i] = data[i];
    return smbusWriteBlock(static_cast<uint8_t>(SbsCommand::ManufacturerBlockAccess), payload, (uint8_t)(2 + len));
}

esp_err_t BQ40Z80::readDataFlash(uint16_t flashAddress, uint8_t *data, uint8_t len) {
    uint8_t rawLen = 0;
    return manufacturerBlockRead(flashAddress, data, rawLen, len);
}

esp_err_t BQ40Z80::writeDataFlash(uint16_t flashAddress, const uint8_t *data, uint8_t len) {
    return manufacturerBlockWrite(flashAddress, data, len);
}

esp_err_t BQ40Z80::getDaConfigurationNR(bool &nr, uint16_t &rawValue) {
    uint8_t raw[2] = {0};
    esp_err_t err = readDataFlash(0x4D03, raw, 2);
    if (err != ESP_OK) return err;
    rawValue = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    nr = (rawValue >> 2) & 1; // Bit 2, confirmed TRM sec 19.2.1.15
    return ESP_OK;
}

// ---- What the charger is actually being told to do --------------------------
// ChargingCurrent()/ChargingVoltage() are the values the gauge broadcasts to an
// SMBus smart charger. A compliant charger will not exceed ChargingCurrent(),
// so if this reads well below your expected charge rate, THIS is your limit --
// and it comes from the Advanced Charging Algorithm table, not from OCC.
esp_err_t BQ40Z80::getChargingCurrent(uint16_t &mA) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::ChargingCurrent), mA);
}

esp_err_t BQ40Z80::getChargingVoltage(uint16_t &mV) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::ChargingVoltage), mV);
}

// ---- Data flash: actual configured overcurrent trip thresholds ---------------
// All addresses confirmed vs TRM SLUUBT5C Table 19-1 (Data Flash Table).
// Read-only. Requires at least Unsealed access -- if the device is Sealed these
// will fail or return zeros (check OperationStatus SEC field).
esp_err_t BQ40Z80::getOvercurrentConfig(OvercurrentConfig &out) {
    uint8_t buf[2] = {0};
    esp_err_t err;

    auto readI2 = [&](uint16_t addr, int16_t &dest) -> esp_err_t {
        esp_err_t e = readDataFlash(addr, buf, 2);
        if (e == ESP_OK) dest = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        return e;
    };

    err = readI2(0x4BE4, out.occ1Threshold_mA);           if (err != ESP_OK) return err;
    err = readDataFlash(0x4BE6, &out.occ1Delay_s, 1);     if (err != ESP_OK) return err;
    err = readI2(0x4BE7, out.occ2Threshold_mA);           if (err != ESP_OK) return err;
    err = readDataFlash(0x4BE9, &out.occ2Delay_s, 1);     if (err != ESP_OK) return err;
    err = readI2(0x4BED, out.ocd1Threshold_mA);           if (err != ESP_OK) return err;
    err = readDataFlash(0x4BEF, &out.ocd1Delay_s, 1);     if (err != ESP_OK) return err;
    err = readI2(0x4BF0, out.ocd2Threshold_mA);           if (err != ESP_OK) return err;
    err = readDataFlash(0x4BF2, &out.ocd2Delay_s, 1);     if (err != ESP_OK) return err;

    return ESP_OK;
}

// ---- Data flash: Advanced Charging Algorithm (JEITA) current table -----------
esp_err_t BQ40Z80::getChargingCurrentConfig(ChargingCurrentConfig &out) {
    uint8_t buf[2] = {0};

    auto readI2 = [&](uint16_t addr, int16_t &dest) -> esp_err_t {
        esp_err_t e = readDataFlash(addr, buf, 2);
        if (e == ESP_OK) dest = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        return e;
    };

    // Standard Temp Low / Standard Temp High / Rec Temp, each Low/Med/High voltage range
    const uint16_t addrs[3][3] = {
        { 0x4CA9, 0x4CAB, 0x4CAD },
        { 0x4CB1, 0x4CB3, 0x4CB5 },
        { 0x4CC1, 0x4CC3, 0x4CC5 },
    };
    int16_t *dests[3] = { out.stdTempLow_mA, out.stdTempHigh_mA, out.recTemp_mA };

    for (int range = 0; range < 3; range++) {
        for (int v = 0; v < 3; v++) {
            esp_err_t err = readI2(addrs[range][v], dests[range][v]);
            if (err != ESP_OK) return err;
        }
    }
    return ESP_OK;
}

// ---- Data flash: cell voltage limits (chemistry sanity check) ----------------
// Addresses confirmed vs TRM SLUUBT5C Table 19-1. Read-only.
// Compare these against your ACTUAL cell chemistry. TI's shipping defaults are
// Li-ion (COV 4300 mV, charge to 4200 mV) and are dangerous on an LFP pack.
esp_err_t BQ40Z80::getCellVoltageLimits(CellVoltageLimits &out) {
    uint8_t buf[2] = {0};

    auto readI2 = [&](uint16_t addr, int16_t &dest) -> esp_err_t {
        esp_err_t e = readDataFlash(addr, buf, 2);
        if (e == ESP_OK) dest = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        return e;
    };

    struct { uint16_t addr; int16_t *dest; } rows[] = {
        { 0x4BCE, &out.covThresholdStdTempLow_mV     },
        { 0x4BD0, &out.covThresholdStdTempHigh_mV    },
        { 0x4BD9, &out.covRecoveryStdTempLow_mV      },
        { 0x4BC2, &out.cuvThreshold_mV               },
        { 0x4BC5, &out.cuvRecovery_mV                },
        { 0x4C9F, &out.chargingVoltageLowTemp_mV     },
        { 0x4CA7, &out.chargingVoltageStdTempLow_mV  },
        { 0x4CAF, &out.chargingVoltageStdTempHigh_mV },
        { 0x4CB7, &out.chargingVoltageHighTemp_mV    },
        { 0x4CBF, &out.chargingVoltageRecTemp_mV     },
        { 0x4CC7, &out.prechargeCurrent_mA           },
        { 0x4CCB, &out.prechargeStartVoltage_mV      },
        { 0x4CCD, &out.voltageRangeLow_mV            },
        { 0x4CCF, &out.voltageRangeMed_mV            },
        { 0x4CD1, &out.voltageRangeHigh_mV           },
    };

    for (auto &r : rows) {
        esp_err_t err = readI2(r.addr, *r.dest);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

// ---- Chemistry ID (MAC 0x0006) ----------------------------------------------
// Confirmed vs TRM SLUUBT5C sec 17: write 06 00 to ManufacturerBlockAccess, read
// back 2 bytes little endian from ManufacturerData. The chemistry dataset itself
// (OCV curve + Ra tables) is NOT writable over SMBus -- only BQStudio can load it.
esp_err_t BQ40Z80::getChemId(uint16_t &chemId) {
    uint8_t raw[2] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::ChemicalId),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;
    if (rawLen < 2) return ESP_ERR_INVALID_SIZE;

    chemId = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    return ESP_OK;
}

// ---- DeviceChemistry() -- declared previously but never defined --------------
esp_err_t BQ40Z80::getDeviceChemistry(char *buf, size_t bufLen) {
    if (bufLen == 0) return ESP_ERR_INVALID_ARG;

    uint8_t raw[32] = {0};
    uint8_t rawLen = 0;
    uint8_t maxLen = (bufLen - 1 > 32) ? 32 : (uint8_t)(bufLen - 1);
    esp_err_t err = smbusReadBlock(static_cast<uint8_t>(SbsCommand::DeviceChemistry),
                                    raw, rawLen, maxLen);
    if (err != ESP_OK) return err;

    memcpy(buf, raw, rawLen);
    buf[rawLen] = '\0';
    return ESP_OK;
}

esp_err_t BQ40Z80::getCounterDecDelayOCD(uint8_t &seconds) {
    return readDataFlash(0x4BF7, &seconds, 1);
}

esp_err_t BQ40Z80::getCounterDecDelayAOLD(uint8_t &seconds) {
    return readDataFlash(0x4BFA, &seconds, 1);
}

esp_err_t BQ40Z80::getCounterDecDelayASCC(uint8_t &seconds) {
    return readDataFlash(0x4BFE, &seconds, 1);
}

esp_err_t BQ40Z80::getCounterDecDelayASCD(uint8_t &seconds) {
    return readDataFlash(0x4C02, &seconds, 1);
}

esp_err_t BQ40Z80::reset() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::DeviceReset));
}

esp_err_t BQ40Z80::getOperationStatus(OperationStatus &out) {
    uint8_t raw[4] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::OperationStatus),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;
    if (rawLen < 4) return ESP_ERR_INVALID_SIZE;

    uint32_t v = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                 ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
    out.raw = v;

    out.iataCterm = (v >> 31) & 1;
    out.emShut    = (v >> 29) & 1;
    out.cb        = (v >> 28) & 1;
    out.slpCc     = (v >> 27) & 1;
    out.slpAd     = (v >> 26) & 1;
    out.smblCal   = (v >> 25) & 1;
    out.init      = (v >> 24) & 1;
    out.sleepM    = (v >> 23) & 1;
    out.xl        = (v >> 22) & 1;
    out.calOffset = (v >> 21) & 1;
    out.cal       = (v >> 20) & 1;
    out.autoCalM  = (v >> 19) & 1;
    out.auth      = (v >> 18) & 1;
    out.led       = (v >> 17) & 1;
    out.sdm       = (v >> 16) & 1;
    out.sleep     = (v >> 15) & 1;
    out.xchg      = (v >> 14) & 1;
    out.xdsg      = (v >> 13) & 1;
    out.pf        = (v >> 12) & 1;
    out.ss        = (v >> 11) & 1;
    out.sdv       = (v >> 10) & 1;
    out.sec       = (v >> 8) & 0x3;
    out.btpInt    = (v >> 7) & 1;
    out.fuse      = (v >> 5) & 1;
    out.pdsg      = (v >> 4) & 1;
    out.pchg      = (v >> 3) & 1;
    out.chg       = (v >> 2) & 1;
    out.dsg       = (v >> 1) & 1;
    out.pres      = (v >> 0) & 1;

    return ESP_OK;
}

esp_err_t BQ40Z80::toggleChargeFet() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::ChgFetToggle));
}

esp_err_t BQ40Z80::toggleDischargeFet() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::DsgFetToggle));
}

esp_err_t BQ40Z80::togglePrechargeFet() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::PchgFetToggle));
}

esp_err_t BQ40Z80::togglePreDischargeFet() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::PdsgFetToggle));
}

esp_err_t BQ40Z80::toggleFuse() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::FuseToggle));
}

esp_err_t BQ40Z80::toggleFetEnable() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::FetControl));
}

esp_err_t BQ40Z80::toggleGaugingEnable() {
    return manufacturerAccessSend(static_cast<uint16_t>(MfaSubcommand::Gauging));
}

esp_err_t BQ40Z80::getSafetyStatus(SafetyStatus &out) {
    uint8_t raw[4] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::SafetyStatus),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;
    if (rawLen < 4) return ESP_ERR_INVALID_SIZE;

    uint32_t v = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                 ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
    out.raw = v;

    out.ocdl  = (v >> 29) & 1;
    out.covl  = (v >> 28) & 1;
    out.utd   = (v >> 27) & 1;
    out.utc   = (v >> 26) & 1;
    out.pchgc = (v >> 25) & 1;
    out.chgv  = (v >> 24) & 1;
    out.chgc  = (v >> 23) & 1;
    out.oc    = (v >> 22) & 1;
    out.cto   = (v >> 20) & 1;
    out.pto   = (v >> 18) & 1;
    out.otf   = (v >> 16) & 1;
    out.cuvc  = (v >> 14) & 1;
    out.otd   = (v >> 13) & 1;
    out.otc   = (v >> 12) & 1;
    out.ascdl = (v >> 11) & 1;
    out.ascd  = (v >> 10) & 1;
    out.asccl = (v >> 9) & 1;
    out.ascc  = (v >> 8) & 1;
    out.aoldl = (v >> 7) & 1;
    out.aold  = (v >> 6) & 1;
    out.ocd2  = (v >> 5) & 1;
    out.ocd1  = (v >> 4) & 1;
    out.occ2  = (v >> 3) & 1;
    out.occ1  = (v >> 2) & 1;
    out.cov   = (v >> 1) & 1;
    out.cuv   = (v >> 0) & 1;

    return ESP_OK;
}

esp_err_t BQ40Z80::getManufacturingStatus(ManufacturingStatus &out) {
    uint8_t raw[2] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::ManufacturingStatus),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;
    if (rawLen < 2) return ESP_ERR_INVALID_SIZE;

    uint16_t v = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    out.raw = v;

    out.calEn   = (v >> 15) & 1;
    out.ltTest  = (v >> 14) & 1;
    out.pdsgEn  = (v >> 13) & 1;
    out.ledEn   = (v >> 9) & 1;
    out.fuseEn  = (v >> 8) & 1;
    out.bbrEn   = (v >> 7) & 1;
    out.pfEn    = (v >> 6) & 1;
    out.lfEn    = (v >> 5) & 1;
    out.fetEn   = (v >> 4) & 1;
    out.gaugeEn = (v >> 3) & 1;
    out.dsgEn   = (v >> 2) & 1;
    out.chgEn   = (v >> 1) & 1;
    out.pchgEn  = (v >> 0) & 1;

    return ESP_OK;
}

esp_err_t BQ40Z80::getGaugingStatus3(GaugingStatus3 &out) {
    uint8_t raw[32] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::GaugingStatus3),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;

    out.rawLen = rawLen;
    if (rawLen < 32) return ESP_ERR_INVALID_SIZE;

    int16_t *words = reinterpret_cast<int16_t *>(raw);
    out.qmax1_mAh       = words[0];
    out.qmax2_mAh       = words[1];
    out.qmax3_mAh       = words[2];
    out.qmax4_mAh       = words[3];
    out.qmaxDod0_1      = words[4];
    out.qmaxDod0_2      = words[5];
    out.qmaxDod0_3      = words[6];
    out.qmaxDod0_4      = words[7];
    out.qmaxPassedQ_mAh = words[8];
    out.qmaxTime        = words[9];
    out.tempK           = words[10];
    out.tempA           = words[11];
    out.rawDod0_1       = words[12];
    out.rawDod0_2       = words[13];
    out.rawDod0_3       = words[14];
    out.rawDod0_4       = words[15];

    return ESP_OK;
}

esp_err_t BQ40Z80::getGaugingStatus(GaugingStatus &out) {
    uint8_t raw[4] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::GaugingStatus),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;
    if (rawLen < 3) return ESP_ERR_INVALID_SIZE; // need at least 3 bytes (24 bits)

    uint32_t v = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
    if (rawLen >= 4) v |= ((uint32_t)raw[3] << 24);
    out.raw = v;

    out.ocvfr   = (v >> 20) & 1;
    out.ldmd    = (v >> 19) & 1;
    out.rx      = (v >> 18) & 1;
    out.qmaxTog = (v >> 17) & 1;
    out.vdq     = (v >> 16) & 1;
    out.nsfm    = (v >> 15) & 1;
    out.slpQmax = (v >> 13) & 1;
    out.qen     = (v >> 12) & 1;
    out.vok     = (v >> 11) & 1;
    out.rDis    = (v >> 10) & 1;
    out.rest    = (v >> 8) & 1;
    out.cf      = (v >> 7) & 1;
    out.dsg     = (v >> 6) & 1;
    out.edv     = (v >> 5) & 1;
    out.balEn   = (v >> 4) & 1;
    out.tc      = (v >> 3) & 1;
    out.td      = (v >> 2) & 1;
    out.fc      = (v >> 1) & 1;
    out.fd      = (v >> 0) & 1;

    return ESP_OK;
}

esp_err_t BQ40Z80::getGaugingStatus1(GaugingStatus1 &out) {
    uint8_t raw[32] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::GaugingStatus1),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;

    out.rawLen = rawLen;
    if (rawLen < 32) return ESP_ERR_INVALID_SIZE;

    int16_t *words = reinterpret_cast<int16_t *>(raw);
    out.trueRemQ_mAh  = words[0];
    out.trueRemE_cWh  = words[1];
    out.initialQ_mAh  = words[2];
    out.initialE_cWh  = words[3];
    out.trueFccQ_mAh  = words[4];
    out.trueFccE_cWh  = words[5];
    out.tSim          = words[6];
    out.tAmbient      = words[7];
    out.raScale1      = words[8];
    out.raScale2      = words[9];
    out.raScale3      = words[10];
    out.raScale4      = words[11];
    out.compRes1      = words[12];
    out.compRes2      = words[13];
    out.compRes3      = words[14];
    out.compRes4      = words[15];

    return ESP_OK;
}

esp_err_t BQ40Z80::getGaugingStatus4(GaugingStatus4 &out) {
    uint8_t raw[30] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::GaugingStatus4),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;

    out.rawLen = rawLen;
    if (rawLen < 30) return ESP_ERR_INVALID_SIZE;

    int16_t *words = reinterpret_cast<int16_t *>(raw); // first 14 bytes = 7 words
    out.raScale5   = words[0];
    out.compRes5   = words[1];
    out.dod0_5     = words[2];
    out.dodEoc5    = words[3];
    out.qmax5_mAh  = words[4];
    out.qmaxDod0_5 = words[5];
    out.rawDod0_5  = words[6];
    out.cellGrid5  = raw[14]; // byte 14, single byte

    int16_t *words2 = reinterpret_cast<int16_t *>(raw + 15); // next 14 bytes
    out.raScale6   = words2[0];
    out.compRes6   = words2[1];
    out.dod0_6     = words2[2];
    out.dodEoc6    = words2[3];
    out.qmax6_mAh  = words2[4];
    out.qmaxDod0_6 = words2[5];
    out.rawDod0_6  = words2[6];
    out.cellGrid6  = raw[29]; // byte 29, single byte

    return ESP_OK;
}

// =============================================================================
// Layer 2: standard SBS getters
// =============================================================================
esp_err_t BQ40Z80::getTemperature(uint16_t &deciKelvin) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::Temperature), deciKelvin);
}

esp_err_t BQ40Z80::getVoltage(uint16_t &mV) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::Voltage), mV);
}

esp_err_t BQ40Z80::getBatteryStatus(uint16_t &flags) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::BatteryStatus), flags);
}

esp_err_t BQ40Z80::getCurrent(int16_t &mA) {
    uint16_t raw = 0;
    esp_err_t err = smbusReadWord(static_cast<uint8_t>(SbsCommand::Current), raw);
    if (err == ESP_OK) {
        mA = (int16_t)raw; // Current() is signed: negative = discharging
    }
    return err;
}

esp_err_t BQ40Z80::getRemainingCapacity(uint16_t &mAh) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::RemainingCapacity), mAh);
}

esp_err_t BQ40Z80::getFullChargeCapacity(uint16_t &mAh) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::FullChargeCapacity), mAh);
}

esp_err_t BQ40Z80::getDesignCapacity(uint16_t &mAh) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::DesignCapacity), mAh);
}

esp_err_t BQ40Z80::setDesignCapacity(uint16_t mAh) {
    return smbusWriteWord(static_cast<uint8_t>(SbsCommand::DesignCapacity), mAh);
}

esp_err_t BQ40Z80::getCellVoltage1(uint16_t &mV) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::CellVoltage1), mV);
}
esp_err_t BQ40Z80::getCellVoltage2(uint16_t &mV) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::CellVoltage2), mV);
}
esp_err_t BQ40Z80::getCellVoltage3(uint16_t &mV) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::CellVoltage3), mV);
}
esp_err_t BQ40Z80::getCellVoltage4(uint16_t &mV) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::CellVoltage4), mV);
}

esp_err_t BQ40Z80::readDAStatus1(DAStatus1 &out) {
    uint8_t raw[32] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::DAStatus1),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;

    out.rawLen = rawLen;
    if (rawLen < 32) return ESP_ERR_INVALID_SIZE; // shouldn't happen, but guard anyway
    memcpy(out.raw, raw, 32);

    int16_t *words = reinterpret_cast<int16_t *>(raw);
    out.cellVoltage1_mV = words[0];
    out.cellVoltage2_mV = words[1];
    out.cellVoltage3_mV = words[2];
    out.cellVoltage4_mV = words[3];
    out.batVoltage_mV   = words[4];
    out.packVoltage_mV  = words[5];
    out.cellCurrent1_mA = words[6];
    out.cellCurrent2_mA = words[7];
    out.cellCurrent3_mA = words[8];
    out.cellCurrent4_mA = words[9];
    out.cellPower1      = words[10];
    out.cellPower2      = words[11];
    out.cellPower3      = words[12];
    out.cellPower4      = words[13];
    out.allCellPower    = words[14];
    out.averagePower    = words[15];

    return ESP_OK;
}

esp_err_t BQ40Z80::getBatVoltage(uint16_t &mV) {
    DAStatus1 status = {};
    esp_err_t err = readDAStatus1(status);
    if (err == ESP_OK) {
        mV = (uint16_t)status.batVoltage_mV;
    }
    return err;
}

esp_err_t BQ40Z80::readDAStatus3(DAStatus3 &out) {
    uint8_t raw[18] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::DAStatus3),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;

    out.rawLen = rawLen;
    if (rawLen < 18) return ESP_ERR_INVALID_SIZE; // shouldn't happen, but guard anyway
    memcpy(out.raw, raw, 18);

    int16_t *words = reinterpret_cast<int16_t *>(raw);
    out.cellVoltage5_mV = words[0];
    out.cellCurrent5_mA = words[1];
    out.cellPower5      = words[2];
    out.cellVoltage6_mV = words[3];
    out.cellCurrent6_mA = words[4];
    out.cellPower6      = words[5];
    out.cellVoltage7_mV = words[6];
    out.cellCurrent7_mA = words[7];
    out.cellPower7      = words[8];

    return ESP_OK;
}

esp_err_t BQ40Z80::getCellVoltage5(uint16_t &mV) {
    DAStatus3 status = {};
    esp_err_t err = readDAStatus3(status);
    if (err == ESP_OK) {
        mV = (uint16_t)status.cellVoltage5_mV;
    }
    return err;
}

esp_err_t BQ40Z80::getCellVoltage6(uint16_t &mV) {
    DAStatus3 status = {};
    esp_err_t err = readDAStatus3(status);
    if (err == ESP_OK) {
        mV = (uint16_t)status.cellVoltage6_mV;
    }
    return err;
}