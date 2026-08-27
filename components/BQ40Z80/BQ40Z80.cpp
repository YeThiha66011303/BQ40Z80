#include "BQ40Z80.h"
#include "driver/gpio.h"

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

esp_err_t BQ40Z80::smbusReadWord(uint8_t command, uint16_t &value) {
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