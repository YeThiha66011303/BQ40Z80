#include "BQ40Z80.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

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

        // DAStatus1 -- print every field (TRM SLUUBT5C page 139, sec 18.1.53)
        DAStatus1 da1 = {};
        esp_err_t da1Err = gauge.readDAStatus1(da1);
        if (da1Err == ESP_OK) {
            ESP_LOGI(TAG, "[DAStatus1] Cell1: %d mV, Cell2: %d mV, Cell3: %d mV, Cell4: %d mV",
                     da1.cellVoltage1_mV, da1.cellVoltage2_mV, da1.cellVoltage3_mV, da1.cellVoltage4_mV);
            ESP_LOGI(TAG, "[DAStatus1] BAT: %d mV, PACK: %d mV",
                     da1.batVoltage_mV, da1.packVoltage_mV);
            ESP_LOGI(TAG, "[DAStatus1] CellCurrent1: %d mA, CellCurrent2: %d mA, CellCurrent3: %d mA, CellCurrent4: %d mA",
                     da1.cellCurrent1_mA, da1.cellCurrent2_mA, da1.cellCurrent3_mA, da1.cellCurrent4_mA);
            ESP_LOGI(TAG, "[DAStatus1] CellPower1: %d cW, CellPower2: %d cW, CellPower3: %d cW, CellPower4: %d cW",
                     da1.cellPower1, da1.cellPower2, da1.cellPower3, da1.cellPower4);
            ESP_LOGI(TAG, "[DAStatus1] Power(V*I): %d cW, AveragePower: %d cW",
                     da1.allCellPower, da1.averagePower);

            char hex1[3 * 32 + 1] = {0};
            for (int i = 0; i < da1.rawLen && i < 32; i++) {
                sprintf(hex1 + i * 3, "%02X ", da1.raw[i]);
            }
            ESP_LOGI(TAG, "DAStatus1 raw (%d bytes): %s", da1.rawLen, hex1);
        } else {
            ESP_LOGE(TAG, "readDAStatus1() failed: %s", esp_err_to_name(da1Err));
        }

        // Cells 5/6 -- confirmed via DAStatus3 (TRM SLUUBT5C page 143).
        DAStatus3 da3 = {};
        esp_err_t da3Err = gauge.readDAStatus3(da3);
        if (da3Err == ESP_OK) {
            ESP_LOGI(TAG, "[DAStatus3] Cell5: %d mV, CellCurrent5: %d mA, CellPower5: %d cW",
                     da3.cellVoltage5_mV, da3.cellCurrent5_mA, da3.cellPower5);
            ESP_LOGI(TAG, "[DAStatus3] Cell6: %d mV, CellCurrent6: %d mA, CellPower6: %d cW",
                     da3.cellVoltage6_mV, da3.cellCurrent6_mA, da3.cellPower6);
            ESP_LOGI(TAG, "[DAStatus3] Cell7: %d mV, CellCurrent7: %d mA, CellPower7: %d cW",
                     da3.cellVoltage7_mV, da3.cellCurrent7_mA, da3.cellPower7);

            char hex3[3 * 18 + 1] = {0};
            for (int i = 0; i < da3.rawLen && i < 18; i++) {
                sprintf(hex3 + i * 3, "%02X ", da3.raw[i]);
            }
            ESP_LOGI(TAG, "DAStatus3 raw (%d bytes): %s", da3.rawLen, hex3);
        } else {
            ESP_LOGE(TAG, "readDAStatus3() failed: %s", esp_err_to_name(da3Err));
        }

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

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}