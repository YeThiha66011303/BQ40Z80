#pragma once
// =============================================================================
// BLE GATT server -- the same read-only view the HTTP dashboard gives, for the
// phone app. See docs/BLE_PROTOCOL.md for the byte layouts.
//
// One custom service, four characteristics:
//   LIVE    read / notify   fast-tier snapshot, pushed once a second
//   INFO    read            static identity, read once after connecting
//   DETAIL  read            slow-tier Impedance Track and lifetime data
//   CMD     write           DeviceReset, behind pairing and a confirm token
//
// Like http_server, nothing here talks to the gauge. Everything is built from
// the telemetry snapshot; the only state change is telemetryRequestReset().
// =============================================================================
#include "esp_err.h"

// The phone shows this during pairing. Pairing is only asked for when the app
// writes CMD, so monitoring needs no PIN. Change it before the pack leaves
// your bench.
#define BLE_PASSKEY 123456

// Cells actually wired on this pack. The packets always carry seven slots.
#define BLE_CELL_COUNT 6

esp_err_t bleServerStart();
