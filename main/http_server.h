#pragma once
// =============================================================================
// HTTP server
//
//   GET  /           the dashboard page, embedded in the firmware image
//   GET  /api/data   the current telemetry snapshot as JSON
//   POST /api/reset  DeviceReset, behind a confirmation token
//
// No route changes data flash: the Wi-Fi AP has no authentication, so settings
// are only written from the passkey-paired app.
// =============================================================================
#include "esp_err.h"

esp_err_t httpServerStart();
