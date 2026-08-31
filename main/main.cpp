#include "BQ40Z80.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

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
        uint16_t mV = 0;
        int16_t mA = 0;

        esp_err_t vErr = gauge.getVoltage(mV);
        esp_err_t iErr = gauge.getCurrent(mA);

        if (vErr == ESP_OK) {
            ESP_LOGI(TAG, "Voltage: %u mV", mV);
        } else {
            ESP_LOGE(TAG, "getVoltage() failed: %s", esp_err_to_name(vErr));
        }

        if (iErr == ESP_OK) {
            ESP_LOGI(TAG, "Current: %d mA", mA);
        } else {
            ESP_LOGE(TAG, "getCurrent() failed: %s", esp_err_to_name(iErr));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}