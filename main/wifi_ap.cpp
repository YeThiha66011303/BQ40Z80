#include "wifi_ap.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"      // MACSTR / MAC2STR
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <cstring>

static const char *TAG = "wifi_ap";

static void onWifiEvent(void *, esp_event_base_t, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        auto *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "station " MACSTR " joined (aid %d)", MAC2STR(e->mac), e->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        auto *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "station " MACSTR " left (aid %d)", MAC2STR(e->mac), e->aid);
    }
}

esp_err_t wifiApStart()
{
    // Wi-Fi keeps calibration data in NVS. A version change or a full flash
    // erase leaves the partition unusable until it is reinitialized.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &onWifiEvent, nullptr, nullptr));

    wifi_config_t wc = {};
    std::strncpy((char *)wc.ap.ssid, WIFI_AP_SSID, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = (uint8_t)std::strlen(WIFI_AP_SSID);
    std::strncpy((char *)wc.ap.password, WIFI_AP_PASSWORD, sizeof(wc.ap.password));
    wc.ap.channel        = WIFI_AP_CHANNEL;
    wc.ap.max_connection = WIFI_AP_MAX_CONN;
    wc.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    wc.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP up: SSID \"%s\"  password \"%s\"",
             WIFI_AP_SSID, WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "open http://192.168.4.1/ once connected");
    return ESP_OK;
}
