#pragma once
// =============================================================================
// Data flash settings the phone app may change.
//
// A fixed list: each entry is one BQStudio Data Memory value, with its address
// from TRM SLUUBT5C table 19 (firmware 4800_0_04), a safe range, and for
// threshold/recovery pairs the order the two must keep. Nothing outside the
// list can be written, and every write is read back and compared.
//
// Ranges are for this pack's LiFePO4 cells. Before reusing the firmware with
// another chemistry, revisit kItems in df_settings.cpp.
//
// The HTTP dashboard stays read only: the Wi-Fi AP has no authentication. The
// only way in is the BLE CMD characteristic, which needs a passkey-paired link.
// =============================================================================
#include "BQ40Z80.h"
#include <cstddef>
#include <cstdint>

#define DF_SETTINGS_VERSION 1

enum class DfWriteResult : uint8_t {
    Ok,
    UnknownId,     // not in the list
    OutOfRange,    // outside the item's min/max
    Conflict,      // would cross its partner (e.g. recovery above threshold)
    Sealed,        // gauge is SEALED; data flash writes are ignored
    BusBusy,       // could not get the SMBus from the poll task
    WriteFailed,   // SMBus error on the write
    VerifyFailed,  // read back a different value
};

void dfSettingsInit(BQ40Z80 *gauge);

// Reads every listed setting from the gauge and packs it for the SETTINGS
// characteristic (layout in docs/BLE_PROTOCOL.md). Returns 0 on failure.
size_t dfSettingsBuildPacket(uint8_t *buf, size_t cap);

// Writes one setting. For bit registers, bits outside the item's writable
// mask keep their current value whatever [value] says.
DfWriteResult dfSettingsWrite(uint8_t id, int32_t value);

// Bumped after every successful write, so cached packets can be refreshed.
uint32_t dfSettingsGeneration();
