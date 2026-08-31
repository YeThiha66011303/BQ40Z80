#include "BQ40Z80.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

// Confirmed from Arduino's official Nano ESP32 pinout diagram:
// A4 (SDA) = GPIO11, A5 (SCL) = GPIO12
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

extern "C" void app_main(void)
{
    BQ40Z80 gauge(SCL_PIN, SDA_PIN, I2C_NUM_0);

    esp_err_t err = gauge.begin();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "begin() failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "BQ40Z80 initialized OK");

    while (1) {
        uint16_t packMv = 0;
        int16_t  packMa = 0;
        uint16_t cellMv[6] = {0};
        esp_err_t cellErr[6];

        esp_err_t vErr = gauge.getVoltage(packMv);
        esp_err_t iErr = gauge.getCurrent(packMa);

        cellErr[0] = gauge.getCellVoltage1(cellMv[0]);
        cellErr[1] = gauge.getCellVoltage2(cellMv[1]);
        cellErr[2] = gauge.getCellVoltage3(cellMv[2]);
        cellErr[3] = gauge.getCellVoltage4(cellMv[3]);

        // Reliable total pack voltage from DAStatus1 -- unlike the standard
        // Voltage() SBS command, this doesn't depend on per-channel ADC
        // calibration this chip hasn't received yet (see earlier findings).
        uint16_t batMv = 0;
        esp_err_t batErr = gauge.getBatVoltage(batMv);
        if (batErr == ESP_OK) {
            ESP_LOGI(TAG, "Total Battery Voltage (DAStatus1 BAT): %u mV", batMv);
        } else {
            ESP_LOGE(TAG, "getBatVoltage() failed: %s", esp_err_to_name(batErr));
        }

        // DAStatus1's OWN internal cell1-4 fields -- computed via whatever
        // path produces the correct BAT value above. Comparing these against
        // the standalone CellVoltage1()-CellVoltage4() commands (0x3C-0x3F,
        // printed further below) tells us whether the standalone commands'
        // per-channel calibration is the actual problem, or something else.
        DAStatus1 da1 = {};
        esp_err_t da1Err = gauge.readDAStatus1(da1);
        if (da1Err == ESP_OK) {
            ESP_LOGI(TAG, "[DAStatus1] Cell1: %d mV, Cell2: %d mV, Cell3: %d mV, Cell4: %d mV",
                     da1.cellVoltage1_mV, da1.cellVoltage2_mV, da1.cellVoltage3_mV, da1.cellVoltage4_mV);
        } else {
            ESP_LOGE(TAG, "readDAStatus1() failed: %s", esp_err_to_name(da1Err));
        }

        // Cells 5/6 -- not exposed as simple word commands on this chip.
        // Pulled from DAStatus3 (0x007B) as raw bytes for now; the exact
        // byte offsets for cell5/cell6 within that block are not yet
        // confirmed against the TRM body text (see BQ40Z80.h comments),
        // so we print the raw block here rather than guess a parse.
        DAStatus3 da3 = {};
        esp_err_t da3Err = gauge.readDAStatus3(da3);

        if (vErr == ESP_OK) {
            ESP_LOGI(TAG, "Pack Voltage: %u mV", packMv);
        } else {
            ESP_LOGE(TAG, "getVoltage() failed: %s", esp_err_to_name(vErr));
        }

        if (iErr == ESP_OK) {
            ESP_LOGI(TAG, "Pack Current: %d mA", packMa);
        } else {
            ESP_LOGE(TAG, "getCurrent() failed: %s", esp_err_to_name(iErr));
        }

        for (int i = 0; i < 4; i++) {
            if (cellErr[i] == ESP_OK) {
                ESP_LOGI(TAG, "Cell %d Voltage: %u mV", i + 1, cellMv[i]);
            } else {
                ESP_LOGE(TAG, "getCellVoltage%d() failed: %s", i + 1, esp_err_to_name(cellErr[i]));
            }
        }

        if (da3Err == ESP_OK) {
            // Raw dump -- cells 5/6 are somewhere in here, offsets TBD.
            char hex[3 * 32 + 1] = {0};
            for (int i = 0; i < da3.rawLen && i < 32; i++) {
                sprintf(hex + i * 3, "%02X ", da3.raw[i]);
            }
            ESP_LOGI(TAG, "DAStatus3 raw (%d bytes): %s", da3.rawLen, hex);
        } else {
            ESP_LOGE(TAG, "readDAStatus3() failed: %s", esp_err_to_name(da3Err));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}