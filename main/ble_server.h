#pragma once
// =============================================================================
// BLE GATT server -- the view the HTTP dashboard gives, plus settings, for the
// phone app. See docs/BLE_PROTOCOL.md for the byte layouts.
//
// One custom service, five characteristics:
//   LIVE     read / notify  fast-tier snapshot, pushed once a second
//   INFO     read           static identity, read once after connecting
//   DETAIL   read           slow-tier Impedance Track and lifetime data
//   CMD      write          DeviceReset or one data flash setting, behind pairing
//   SETTINGS read           current values of the writable data flash settings
//
// LIVE, INFO and DETAIL come from the telemetry snapshot. SETTINGS and the
// setting writes go through df_settings, which takes the SMBus lock.
// =============================================================================
#include "esp_err.h"

// The phone shows this during pairing. Pairing is only asked for when the app
// writes CMD, so monitoring needs no PIN. Change it before the pack leaves
// your bench.
#define BLE_PASSKEY 123456

// Cells actually wired on this pack. The packets always carry seven slots.
#define BLE_CELL_COUNT 6

esp_err_t bleServerStart();
