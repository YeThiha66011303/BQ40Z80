// =============================================================================
// Data flash settings -- see df_settings.h.
// =============================================================================
#include "df_settings.h"
#include "df_catalog.h"
#include "telemetry.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "df_settings";

namespace {

using T = DfType;

uint8_t sizeOf(T t)
{
    switch (t) {
    case T::U1: case T::I1: case T::H1: return 1;
    case T::U2: case T::I2: case T::H2: return 2;
    case T::U4: case T::I4: case T::H4: return 4;
    }
    return 0;
}

struct Item {
    uint8_t  id;
    uint16_t addr;
    T        type;
    int32_t  min, max;
    uint8_t  mask;     // H1 only: bits the app may change; others are preserved
    uint8_t  partner;  // 0 = none
    int8_t   order;    // +1: must stay above partner, -1: must stay below
};

// Temperatures are raw 0.1 K, as stored: 2732 = 0.0 °C.
#define DEG(c) ((int32_t)(2732 + (c) * 10))

// IDs are part of the BLE protocol: never renumber, only add.
constexpr Item kItems[] = {
    // ---- Cell balancing ---------------------------------------------------
    // Balancing Configuration: CB, CBR, CB_CHG_DOD0EW, CB_RLX_DOD0EW, CBS.
    // CBM (bit 1, external balancing) is excluded: this board has no external
    // balancing FETs, and bits 7-6 are reserved.
    { 1, 0x4B8C, T::H1, 0, 0xFF, 0x3D, 0, 0 },
    { 2, 0x4B93, T::U1, 1, 100,  0, 0, 0 },          // Min Start Balance Delta, mV
    { 3, 0x4B94, T::U4, 10, 36000, 0, 0, 0 },        // Relax Balance Interval, s
    { 4, 0x4B98, T::U1, 0, 100,  0, 0, 0 },          // Min RSOC for Balancing, %
    // Bal Time/mAh is the seconds of bypass per mAh moved: it follows from the
    // balancing resistor, so only change it if that resistor changes.
    { 5, 0x4B8D, T::U2, 50, 5000, 0, 0, 0 },         // Bal Time/mAh Cell 1, s/mAh
    { 6, 0x4B8F, T::U2, 50, 5000, 0, 0, 0 },         // Bal Time/mAh Cell 2-6
    { 7, 0x4B91, T::U2, 50, 5000, 0, 0, 0 },         // Bal Time/mAh Cell 7
    { 8, 0x4B99, T::U1, 0, 100, 0, 9, +1 },          // Start RSOC for Bal in Sleep, above End
    { 9, 0x4B9A, T::U1, 0, 100, 0, 8, -1 },          // End RSOC for Bal in Sleep
    { 60, 0x4B9B, T::U2, 0, 65520, 0, 0, 0 },        // Start Time for Bal in Sleep, h

    // ---- Cell undervoltage ------------------------------------------------
    { 10, 0x4BC2, T::I2, 2000, 3000, 0, 11, -1 },    // CUV Threshold, below Recovery
    { 11, 0x4BC5, T::I2, 2200, 3400, 0, 10, +1 },    // CUV Recovery
    { 12, 0x4BC4, T::U1, 1, 30, 0, 0, 0 },           // CUV Delay, s

    // ---- Cell overvoltage: threshold above recovery, per temperature range -
    { 13, 0x4BCC, T::I2, 3500, 3800, 0, 19, +1 },    // Low Temp
    { 14, 0x4BCE, T::I2, 3500, 3800, 0, 20, +1 },    // Standard Temp Low
    { 15, 0x4BD0, T::I2, 3500, 3800, 0, 21, +1 },    // Standard Temp High
    { 16, 0x4BD2, T::I2, 3500, 3800, 0, 22, +1 },    // High Temp
    { 17, 0x4BD4, T::I2, 3500, 3800, 0, 23, +1 },    // Rec Temp
    { 18, 0x4BD6, T::U1, 1, 30, 0, 0, 0 },           // COV Delay, s
    { 19, 0x4BD7, T::I2, 3300, 3700, 0, 13, -1 },    // Recovery Low Temp
    { 20, 0x4BD9, T::I2, 3300, 3700, 0, 14, -1 },    // Recovery Standard Temp Low
    { 21, 0x4BDB, T::I2, 3300, 3700, 0, 15, -1 },    // Recovery Standard Temp High
    { 22, 0x4BDD, T::I2, 3300, 3700, 0, 16, -1 },    // Recovery High Temp
    { 23, 0x4BDF, T::I2, 3300, 3700, 0, 17, -1 },    // Recovery Rec Temp

    // ---- Charging voltage per cell: must stay below that range's COV ------
    { 30, 0x4C9F, T::I2, 3300, 3650, 0, 13, -1 },    // Low Temp
    { 31, 0x4CA7, T::I2, 3300, 3650, 0, 14, -1 },    // Standard Temp Low
    { 32, 0x4CAF, T::I2, 3300, 3650, 0, 15, -1 },    // Standard Temp High
    { 33, 0x4CB7, T::I2, 3300, 3650, 0, 16, -1 },    // High Temp
    { 34, 0x4CBF, T::I2, 3300, 3650, 0, 17, -1 },    // Rec Temp
    { 35, 0x4CF3, T::I2, 50, 5000, 0, 0, 0 },        // Charge Term Taper Current, mA

    // ---- Temperature protections ------------------------------------------
    { 40, 0x4C05, T::I2, DEG(30), DEG(60),  0, 41, +1 },   // OTC Threshold
    { 41, 0x4C08, T::I2, DEG(25), DEG(55),  0, 40, -1 },   // OTC Recovery
    { 42, 0x4C0A, T::I2, DEG(40), DEG(75),  0, 43, +1 },   // OTD Threshold
    { 43, 0x4C0D, T::I2, DEG(30), DEG(70),  0, 42, -1 },   // OTD Recovery
    { 44, 0x4C14, T::I2, DEG(-10), DEG(15), 0, 45, -1 },   // UTC Threshold
    { 45, 0x4C17, T::I2, DEG(-5), DEG(20),  0, 44, +1 },   // UTC Recovery
    { 46, 0x4C19, T::I2, DEG(-30), DEG(10), 0, 47, -1 },   // UTD Threshold
    { 47, 0x4C1C, T::I2, DEG(-25), DEG(15), 0, 46, +1 },   // UTD Recovery

    // ---- Enabled Protections A-D (reserved bits preserved) ----------------
    { 50, 0x4BBE, T::H1, 0, 0xFF, 0xBF, 0, 0 },   // A: bit 6 RSVD_ONE
    { 51, 0x4BBF, T::H1, 0, 0xFF, 0x7B, 0, 0 },   // B: bit 7 RSVD, bit 2 RSVD_ONE
    { 52, 0x4BC0, T::H1, 0, 0xFF, 0xD7, 0, 0 },   // C: bits 5, 3 RSVD
    { 53, 0x4BC1, T::H1, 0, 0xFF, 0x3F, 0, 0 },   // D: bits 7-6 RSVD
};

constexpr size_t kCount = sizeof(kItems) / sizeof(kItems[0]);

BQ40Z80          *s_gauge = nullptr;
volatile uint32_t s_generation = 0;

const Item *find(uint8_t id)
{
    for (const Item &it : kItems) {
        if (it.id == id) return &it;
    }
    return nullptr;
}

const Item *findByAddr(uint16_t addr)
{
    for (const Item &it : kItems) {
        if (it.addr == addr) return &it;
    }
    return nullptr;
}

const DfParam *findWritable(uint16_t addr)
{
    size_t lo = 0, hi = sizeof(kDfWritable) / sizeof(kDfWritable[0]);
    while (lo < hi) {
        const size_t mid = (lo + hi) / 2;
        if (kDfWritable[mid].addr == addr) return &kDfWritable[mid];
        if (kDfWritable[mid].addr < addr) lo = mid + 1; else hi = mid;
    }
    return nullptr;
}

int32_t decode(T type, const uint8_t *b)
{
    uint32_t u = 0;
    for (int i = sizeOf(type) - 1; i >= 0; i--) u = (u << 8) | b[i];
    switch (type) {
    case T::I1: return (int8_t)u;
    case T::I2: return (int16_t)u;
    default:    return (int32_t)u;
    }
}

void encode(T type, int32_t v, uint8_t *b)
{
    for (int i = 0; i < sizeOf(type); i++) b[i] = (uint8_t)((uint32_t)v >> (8 * i));
}

// Bus lock must be held.
esp_err_t readValue(uint16_t addr, T type, int32_t &out)
{
    uint8_t b[4] = {0};
    esp_err_t err = s_gauge->readDataFlash(addr, b, sizeOf(type));
    if (err == ESP_OK) out = decode(type, b);
    return err;
}

bool isSealed()
{
    Telemetry t;
    telemetryGet(t);
    return ((t.op.raw >> 8) & 0x3) == 3;   // OperationStatus SEC1:SEC0
}

// Bus lock must be held. Writes, then reads back until it matches.
DfWriteResult writeVerify(uint16_t addr, T type, int32_t value)
{
    uint8_t b[4];
    encode(type, value, b);
    if (s_gauge->writeDataFlash(addr, b, sizeOf(type)) != ESP_OK) return DfWriteResult::WriteFailed;
    // The flash write completes inside the gauge after the SMBus transaction
    // ends; give it time before reading back.
    for (int attempt = 0; attempt < 3; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(attempt == 0 ? 100 : 50));
        int32_t back = 0;
        if (readValue(addr, type, back) == ESP_OK && back == value) return DfWriteResult::Ok;
    }
    return DfWriteResult::VerifyFailed;
}

void afterWrite(uint16_t addr, int32_t from, int32_t to)
{
    s_generation = s_generation + 1;
    ESP_LOGW(TAG, "data flash 0x%04X: %ld -> %ld", addr, (long)from, (long)to);
    // INFO carries Enabled Protections C.
    if (addr == 0x4BC0) telemetryReadStatic();
}

} // namespace

void dfSettingsInit(BQ40Z80 *gauge)
{
    s_gauge = gauge;
}

uint32_t dfSettingsGeneration()
{
    return s_generation;
}

size_t dfSettingsBuildPacket(uint8_t *buf, size_t cap)
{
    const size_t need = 2 + kCount * 6;
    if (!s_gauge || cap < need) return 0;
    if (!telemetryLockBus(1500)) return 0;

    size_t n = 0;
    buf[n++] = DF_SETTINGS_VERSION;
    buf[n++] = (uint8_t)kCount;
    for (const Item &it : kItems) {
        int32_t v = 0;
        const bool ok = readValue(it.addr, it.type, v) == ESP_OK;
        buf[n++] = it.id;
        buf[n++] = ok ? 1 : 0;
        for (int i = 0; i < 4; i++) buf[n++] = (uint8_t)((uint32_t)v >> (8 * i));
    }
    telemetryUnlockBus();
    return n;
}

DfWriteResult dfSettingsWrite(uint8_t id, int32_t value)
{
    const Item *it = find(id);
    if (!it || !s_gauge) return DfWriteResult::UnknownId;
    if (value < it->min || value > it->max) return DfWriteResult::OutOfRange;
    if (isSealed()) return DfWriteResult::Sealed;
    if (!telemetryLockBus(3000)) return DfWriteResult::BusBusy;

    DfWriteResult result = DfWriteResult::Ok;
    int32_t current = 0, partner = 0;
    const Item *p = it->partner ? find(it->partner) : nullptr;

    if (readValue(it->addr, it->type, current) != ESP_OK ||
        (p && readValue(p->addr, p->type, partner) != ESP_OK)) {
        result = DfWriteResult::WriteFailed;
    } else {
        if (it->type == T::H1) value = (current & ~it->mask) | (value & it->mask);

        if (p && ((it->order > 0 && value <= partner) || (it->order < 0 && value >= partner))) {
            result = DfWriteResult::Conflict;
        } else if (value != current) {
            result = writeVerify(it->addr, it->type, value);
        }
    }
    telemetryUnlockBus();

    if (result == DfWriteResult::Ok && value != current) {
        afterWrite(it->addr, current, value);
    } else if (result != DfWriteResult::Ok) {
        ESP_LOGE(TAG, "setting %u -> %ld rejected (%d)", id, (long)value, (int)result);
    }
    return result;
}

DfWriteResult dfWriteAddress(uint16_t addr, int32_t value)
{
    // The hand-picked settings keep their tighter limits, masks and pairing.
    if (const Item *it = findByAddr(addr)) return dfSettingsWrite(it->id, value);

    const DfParam *p = findWritable(addr);
    if (!p || !s_gauge) return DfWriteResult::UnknownId;
    if (value < p->min || value > p->max) return DfWriteResult::OutOfRange;
    if (isSealed()) return DfWriteResult::Sealed;
    if (!telemetryLockBus(3000)) return DfWriteResult::BusBusy;

    int32_t current = 0;
    DfWriteResult result = DfWriteResult::Ok;
    if (readValue(addr, p->type, current) != ESP_OK) {
        result = DfWriteResult::WriteFailed;
    } else if (value != current) {
        result = writeVerify(addr, p->type, value);
    }
    telemetryUnlockBus();

    if (result == DfWriteResult::Ok && value != current) {
        afterWrite(addr, current, value);
    } else if (result != DfWriteResult::Ok) {
        ESP_LOGE(TAG, "0x%04X -> %ld rejected (%d)", addr, (long)value, (int)result);
    }
    return result;
}

bool dfReadRaw(uint16_t addr, uint16_t len, uint8_t *out)
{
    if (!s_gauge || len == 0 || len > DF_READ_MAX) return false;
    if (addr < 0x4000 || (uint32_t)addr + len > 0x6000) return false;
    if (!telemetryLockBus(1500)) return false;

    bool ok = true;
    for (uint16_t off = 0; ok && off < len; off += 32) {
        uint8_t block[32];
        uint8_t got = 0;
        ok = s_gauge->manufacturerBlockRead(addr + off, block, got, sizeof(block)) == ESP_OK;
        const uint16_t want = (len - off < 32) ? (len - off) : 32;
        ok = ok && got >= want;
        for (uint16_t i = 0; ok && i < want; i++) out[off + i] = block[i];
    }
    telemetryUnlockBus();
    return ok;
}
