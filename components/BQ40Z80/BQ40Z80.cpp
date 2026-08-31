#include "BQ40Z80.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

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
// SMBus block protocol: command byte, then a device-reported byte-count byte,
// then that many data bytes, then PEC (if enabled).
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

    // We don't know countByte's value yet at command-build time (the i2c
    // driver requires the read length to be fixed when the command list is
    // built), so we conservatively queue reads for the max block size (32
    // bytes) and only interpret the first countByte of them afterward. The
    // very last byte read in the transaction must be NACK'd to terminate it
    // correctly -- which byte that is depends on whether PEC is enabled.
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
        // PEC input: [addr+W, command, addr+R, countByte, data...]
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

// =============================================================================
// Bus recovery: manually bit-bang up to 9 SCL pulses (releasing the I2C
// peripheral's control of the pins temporarily) to force a wedged slave to
// release SDA, then issue a manual STOP condition, then reinitialize the I2C
// driver. Standard I2C/SMBus bus-recovery pattern -- not BQ40Z80-specific.
// =============================================================================
esp_err_t BQ40Z80::recoverBus() {
    if (initialized_) {
        i2c_driver_delete(port_);
        initialized_ = false;
    }

    // Configure both lines as open-drain GPIO so we can bit-bang them like
    // the bus naturally works: only ever drive low, or release (let the
    // external pull-up bring the line high).
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

    // Manual STOP condition: SDA low->high while SCL is high.
    gpio_set_level(sdaPin_, 0);
    esp_rom_delay_us(5);
    gpio_set_level(sclPin_, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sdaPin_, 1);
    esp_rom_delay_us(5);

    // Hand the pins back to the I2C peripheral.
    esp_err_t err = begin();
    if (err != ESP_OK) return err;

    return sdaFreed ? ESP_OK : ESP_ERR_INVALID_STATE; // report if recovery fully succeeded
}

// =============================================================================
// Public Layer 1 wrappers: one automatic recoverBus() + retry on
// ESP_ERR_TIMEOUT, so a wedged bus self-heals instead of failing forever.
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

// =============================================================================
// Layer 2: standard SBS getters
// =============================================================================
esp_err_t BQ40Z80::getVoltage(uint16_t &mV) {
    return smbusReadWord(static_cast<uint8_t>(SbsCommand::Voltage), mV);
}

esp_err_t BQ40Z80::getCurrent(int16_t &mA) {
    uint16_t raw = 0;
    esp_err_t err = smbusReadWord(static_cast<uint8_t>(SbsCommand::Current), raw);
    if (err == ESP_OK) {
        mA = (int16_t)raw; // Current() is signed: negative = discharging
    }
    return err;
}

// ---- Individual cell voltages (confirmed direct word reads, cells 1-4) --------
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

// ---- DAStatus1 (0x0071) -- confirmed 16-word layout, see struct comment ------
esp_err_t BQ40Z80::readDAStatus1(DAStatus1 &out) {
    uint8_t raw[32] = {0};
    uint8_t rawLen = 0;
    esp_err_t err = manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::DAStatus1),
                                           raw, rawLen, sizeof(raw));
    if (err != ESP_OK) return err;

    out.rawLen = rawLen;
    if (rawLen < 32) return ESP_ERR_INVALID_SIZE; // shouldn't happen, but guard anyway

    // Little-endian int16_t words, in the confirmed field order.
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

// ---- DAStatus3 (0x007B) -- raw bytes only; cell5/6 field layout not yet --------
// ---- confirmed against TRM body text, so we do not parse it here yet. ---------
esp_err_t BQ40Z80::readDAStatus3(DAStatus3 &out) {
    return manufacturerBlockRead(static_cast<uint16_t>(MfaSubcommand::DAStatus3),
                                  out.raw, out.rawLen, sizeof(out.raw));
}