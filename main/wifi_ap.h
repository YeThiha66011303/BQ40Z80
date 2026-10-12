#pragma once
// =============================================================================
// SoftAP bring-up.
//
// The ESP32 broadcasts its own network instead of joining one, so the pack can
// be inspected anywhere -- a bench, a vehicle, a lab with no Wi-Fi -- without a
// router in the path. Connect a phone or laptop to the SSID below and open
// http://192.168.4.1/
// =============================================================================
#include "esp_err.h"

#define WIFI_AP_SSID      "BQ40Z80-BMS"
#define WIFI_AP_PASSWORD  "bms12345"      // >= 8 chars, or WPA2 setup fails
#define WIFI_AP_CHANNEL   1
#define WIFI_AP_MAX_CONN  4

esp_err_t wifiApStart();
