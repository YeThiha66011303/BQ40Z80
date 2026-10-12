#pragma once
// =============================================================================
// HTTP server -- READ ONLY.
//
// Two routes, both GET:
//   /           the dashboard page, embedded in the firmware image
//   /api/data   the current telemetry snapshot as JSON
//
// There is deliberately no route that writes anything to the gauge. The server
// can only hand out what the poll task has already read.
// =============================================================================
#include "esp_err.h"

esp_err_t httpServerStart();
