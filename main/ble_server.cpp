// =============================================================================
// BLE GATT server. Reads are open; the two state changes -- the guarded reset
// and the whitelisted data flash settings -- go through CMD, which needs a
// passkey-paired link. See ble_server.h.
//
// Packets are packed little-endian and written field by field, so the layout
// does not depend on struct padding. docs/BLE_PROTOCOL.md is the reference;
// the Flutter app reads the fields back in the same order.
// =============================================================================
#include "ble_server.h"
#include "df_settings.h"
#include "telemetry.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <cstdio>
#include <cstring>

// Not declared in any public NimBLE header.
extern "C" void ble_store_config_init(void);

static const char *TAG = "ble";

#define NOTIFY_PERIOD_MS 1000

#define LIVE_VERSION   1
#define INFO_VERSION   1
#define DETAIL_VERSION 1

// Application ATT errors returned from CMD writes.
#define CMD_ERR_BAD_TOKEN   0x80   // token did not match the current reset hazard
#define CMD_ERR_FAILED      0x81   // DeviceReset itself failed on the SMBus
#define CMD_ERR_UNKNOWN_OP  0x82
#define CMD_ERR_BAD_SETTING 0x83   // setting id not in the whitelist
#define CMD_ERR_RANGE       0x84   // value outside the setting's limits
#define CMD_ERR_CONFLICT    0x85   // would cross its threshold/recovery partner
#define CMD_ERR_SEALED      0x86   // gauge is SEALED
#define CMD_ERR_BUSY        0x87   // SMBus busy
#define CMD_ERR_WRITE       0x88   // SMBus error writing data flash
#define CMD_ERR_VERIFY      0x89   // read back a different value

#define CMD_OP_RESET   0x01
#define CMD_OP_SETTING 0x02
#define CMD_OP_DF_WRITE 0x03

// -----------------------------------------------------------------------------
// UUIDs: 9a1b000X-5f3c-4e8a-b7d2-1c6e4f8a2b30, X = 1 service, 2..5 chars.
// BLE_UUID128_INIT takes the bytes least significant first.
// -----------------------------------------------------------------------------
#define BMS_UUID(x) BLE_UUID128_INIT(0x30, 0x2b, 0x8a, 0x4f, 0x6e, 0x1c, 0xd2, 0xb7, \
                                     0x8a, 0x4e, 0x3c, 0x5f, (x), 0x00, 0x1b, 0x9a)

static const ble_uuid128_t kSvcUuid    = BMS_UUID(0x01);
static const ble_uuid128_t kLiveUuid   = BMS_UUID(0x02);
static const ble_uuid128_t kInfoUuid   = BMS_UUID(0x03);
static const ble_uuid128_t kDetailUuid = BMS_UUID(0x04);
static const ble_uuid128_t kCmdUuid    = BMS_UUID(0x05);
static const ble_uuid128_t kSettingsUuid = BMS_UUID(0x06);
static const ble_uuid128_t kDfReadUuid   = BMS_UUID(0x07);

enum Chr : int { CHR_LIVE, CHR_INFO, CHR_DETAIL, CHR_CMD, CHR_SETTINGS, CHR_DFREAD };

static uint16_t s_liveHandle;
static uint8_t  s_ownAddrType;
static char     s_name[16];

// Written by the NimBLE host task, read by the notify task. Single connection.
static volatile uint16_t s_conn           = BLE_HS_CONN_HANDLE_NONE;
static volatile bool     s_liveSubscribed = false;

// -----------------------------------------------------------------------------
// Packet writer
// -----------------------------------------------------------------------------
namespace {

struct Packer {
    uint8_t *buf;
    size_t   cap;
    size_t   len;

    void u8(uint8_t v)  { if (len < cap) buf[len++] = v; }
    void i8(int8_t v)   { u8((uint8_t)v); }
    void u16(uint16_t v){ u8(v & 0xFF); u8(v >> 8); }
    void i16(int16_t v) { u16((uint16_t)v); }
    void u32(uint32_t v){ u16(v & 0xFFFF); u16(v >> 16); }
    void i32(int32_t v) { u32((uint32_t)v); }

    void arr16(const int16_t *v, int n) { for (int i = 0; i < n; ++i) i16(v[i]); }

    // Length-prefixed, at most 255 bytes, no terminator.
    void str(const char *s)
    {
        size_t n = strnlen(s, 255);
        u8((uint8_t)n);
        for (size_t i = 0; i < n; ++i) u8((uint8_t)s[i]);
    }

    bool overflowed() const { return len >= cap; }
};

} // namespace

// 152 bytes today. Needs an ATT MTU of at least 155 to fit one notification.
#define LIVE_BUF_BYTES 192

static size_t buildLive(uint8_t *buf, size_t cap)
{
    Telemetry t;
    StaticInfo s;
    ResetInfo r;
    telemetryGet(t);
    telemetryGetStatic(s);
    telemetryGetResetInfo(r);

    Packer p{buf, cap, 0};
    p.u8(LIVE_VERSION);
    p.u8((t.online ? 0x01 : 0) | (s.valid ? 0x02 : 0) |
         (telemetryResetIsSafe() ? 0x04 : 0));
    p.u8(BLE_CELL_COUNT);
    p.u8(t.rsoc_pct);
    p.u8(t.asoc_pct);
    p.u8(t.maxError_pct);

    p.u32(t.uptime_s);
    p.u32(t.pollCount);
    p.u32(t.errorCount);

    p.u16(t.voltage_mV);
    p.i16(t.current_mA);
    p.i16(t.avgCurrent_mA);
    p.u16(t.temperature_dK);
    p.u16(t.remainingCapacity_mAh);
    p.u16(t.fullChargeCapacity_mAh);
    p.u16(t.chargingCurrent_mA);
    p.u16(t.chargingVoltage_mV);
    p.u16(t.batteryStatus);

    p.i16(t.da1.batVoltage_mV);
    p.i16(t.da1.packVoltage_mV);
    p.i16(t.da1.allCellPower);
    p.i16(t.da1.averagePower);

    // Cells 1-4 come from DAStatus1, 5-7 from DAStatus3.
    const int16_t cv[7] = { t.da1.cellVoltage1_mV, t.da1.cellVoltage2_mV,
                            t.da1.cellVoltage3_mV, t.da1.cellVoltage4_mV,
                            t.da3.cellVoltage5_mV, t.da3.cellVoltage6_mV,
                            t.da3.cellVoltage7_mV };
    const int16_t ci[7] = { t.da1.cellCurrent1_mA, t.da1.cellCurrent2_mA,
                            t.da1.cellCurrent3_mA, t.da1.cellCurrent4_mA,
                            t.da3.cellCurrent5_mA, t.da3.cellCurrent6_mA,
                            t.da3.cellCurrent7_mA };
    const int16_t cp[7] = { t.da1.cellPower1, t.da1.cellPower2,
                            t.da1.cellPower3, t.da1.cellPower4,
                            t.da3.cellPower5, t.da3.cellPower6,
                            t.da3.cellPower7 };
    p.arr16(cv, 7);
    p.arr16(ci, 7);
    p.arr16(cp, 7);

    // Int, TS1-TS4, Cell, FET in raw 0.1 K.
    const int16_t temps[7] = { t.da2.intTemp_dK, t.da2.ts1Temp_dK, t.da2.ts2Temp_dK,
                               t.da2.ts3Temp_dK, t.da2.ts4Temp_dK, t.da2.cellTemp_dK,
                               t.da2.fetTemp_dK };
    p.arr16(temps, 7);

    p.u32(t.op.raw);
    p.u32(t.safetyStatus.raw);
    p.u32(t.safetyAlert.raw);
    p.u32(t.chargingStatus.raw);
    p.u32(t.gaugingStatus.raw);
    p.u16(t.mfg.raw);
    p.u32(t.pfStatus);
    p.u32(t.pfAlert);

    p.u16(t.cycleCount);
    p.u16(t.stateOfHealth_pct);
    p.u16(t.runTimeToEmpty_min);
    p.u16(t.avgTimeToEmpty_min);
    p.u16(t.avgTimeToFull_min);

    p.u32(r.count);
    p.u32(r.lastAt_s);
    p.i32(r.lastErr);

    return p.overflowed() ? 0 : p.len;
}

static size_t buildInfo(uint8_t *buf, size_t cap)
{
    StaticInfo s;
    telemetryGetStatic(s);

    Packer p{buf, cap, 0};
    p.u8(INFO_VERSION);
    p.u8((s.valid ? 0x01 : 0) | (s.enabledProtectionsCValid ? 0x02 : 0) |
         (s.mfgStatusInitValid ? 0x04 : 0));
    p.u8(BLE_CELL_COUNT);
    p.u8(s.enabledProtectionsC);
    p.u16(s.chemId);
    p.u16(s.deviceType);
    p.u16(s.serialNumber);
    p.u16(s.designCapacity_mAh);
    p.u16(s.designVoltage_mV);
    p.u16(s.manufactureDate);
    p.u16(s.mfgStatusInit);
    p.str(s.manufacturer);
    p.str(s.deviceName);
    p.str(s.chemistry);

    return p.overflowed() ? 0 : p.len;
}

// ATT caps an attribute at 512 bytes; this is about 300.
#define DETAIL_BUF_BYTES 512

static size_t buildDetail(uint8_t *buf, size_t cap)
{
    Telemetry t;
    telemetryGet(t);

    Packer p{buf, cap, 0};
    p.u8(DETAIL_VERSION);
    p.u32(t.lastSlowPoll_s);

    // ---- slow SBS words ---------------------------------------------------
    p.u16(t.batteryMode);
    p.u16(t.remCapAlarm_mAh);
    p.u16(t.remTimeAlarm_min);
    p.i16(t.atRate_mA);
    p.u16(t.atRateTimeToFull_min);
    p.u16(t.atRateTimeToEmpty_min);
    p.u16(t.atRateOK);
    p.i16(t.maxTurboPower_cW);
    p.i16(t.susTurboPower_cW);
    p.i16(t.maxTurboCurrent_mA);
    p.i16(t.susTurboCurrent_mA);
    p.u16(t.turboPackR_mOhm);
    p.u16(t.turboSysR_mOhm);
    p.u16(t.turboEdv_mV);
    p.u16(t.noLoadRemCap_mAh);
    p.u16(t.sohFcc_mAh);
    p.u16(t.sohFcc_cWh);

    // ---- Impedance Track, per cell (slot 7 is always 0) -------------------
    const int16_t qmax[7]    = { t.g3.qmax1_mAh, t.g3.qmax2_mAh, t.g3.qmax3_mAh,
                                 t.g3.qmax4_mAh, t.g4.qmax5_mAh, t.g4.qmax6_mAh, 0 };
    const int16_t qmaxDod[7] = { t.g3.qmaxDod0_1, t.g3.qmaxDod0_2, t.g3.qmaxDod0_3,
                                 t.g3.qmaxDod0_4, t.g4.qmaxDod0_5, t.g4.qmaxDod0_6, 0 };
    const int16_t rawDod[7]  = { t.g3.rawDod0_1, t.g3.rawDod0_2, t.g3.rawDod0_3,
                                 t.g3.rawDod0_4, t.g4.rawDod0_5, t.g4.rawDod0_6, 0 };
    const int16_t raScale[7] = { t.g1.raScale1, t.g1.raScale2, t.g1.raScale3,
                                 t.g1.raScale4, t.g4.raScale5, t.g4.raScale6, 0 };
    const int16_t compRes[7] = { t.g1.compRes1, t.g1.compRes2, t.g1.compRes3,
                                 t.g1.compRes4, t.g4.compRes5, t.g4.compRes6, 0 };
    const int16_t dod0[7]    = { t.g2.dod0[0], t.g2.dod0[1], t.g2.dod0[2],
                                 t.g2.dod0[3], t.g4.dod0_5, t.g4.dod0_6, 0 };
    const int16_t dodEoc[7]  = { t.g2.dodEoc[0], t.g2.dodEoc[1], t.g2.dodEoc[2],
                                 t.g2.dodEoc[3], t.g4.dodEoc5, t.g4.dodEoc6, 0 };
    const int16_t grid[7]    = { (int16_t)t.g2.cellGrid[0], (int16_t)t.g2.cellGrid[1],
                                 (int16_t)t.g2.cellGrid[2], (int16_t)t.g2.cellGrid[3],
                                 (int16_t)t.g4.cellGrid5, (int16_t)t.g4.cellGrid6, 0 };
    p.arr16(qmax, 7);
    p.arr16(qmaxDod, 7);
    p.arr16(rawDod, 7);
    p.arr16(raScale, 7);
    p.arr16(compRes, 7);
    p.arr16(dod0, 7);
    p.arr16(dodEoc, 7);
    p.arr16(grid, 7);

    p.u8(t.g2.packGrid);
    p.u8(t.g2.lStatus);
    p.u32(t.g2.stateTime_s);
    p.i16(t.g2.dod0PassedQ_mAh);
    p.i16(t.g2.dod0PassedE_cWh);
    p.i16(t.g2.dod0Time_hr16);
    p.i16(t.g3.qmaxPassedQ_mAh);
    p.i16(t.g3.qmaxTime);
    p.i16(t.g3.tempK);
    p.i16(t.g3.tempA);
    p.i16(t.g1.trueRemQ_mAh);
    p.i16(t.g1.trueRemE_cWh);
    p.i16(t.g1.initialQ_mAh);
    p.i16(t.g1.initialE_cWh);
    p.i16(t.g1.trueFccQ_mAh);
    p.i16(t.g1.trueFccE_cWh);
    p.i16(t.g1.tSim);
    p.i16(t.g1.tAmbient);

    // ---- filtered capacity -----------------------------------------------
    p.i16(t.filt.remainingCapacity_mAh);
    p.i16(t.filt.remainingEnergy_mWh);
    p.i16(t.filt.fullChargeCapacity_mAh);
    p.i16(t.filt.fullChargeEnergy_mWh);

    // ---- cell balancing ---------------------------------------------------
    p.arr16(t.cb.balanceTime_s, 7);
    p.arr16(t.cb.balanceDod, 7);
    p.i16(t.cb.totalDodCharge);

    // ---- lifetime records -------------------------------------------------
    p.arr16(t.lt1.cellMaxVoltage_mV, 7);
    p.arr16(t.lt1.cellMinVoltage_mV, 7);
    p.i16(t.lt1.maxDeltaCellVoltage_mV);

    p.i16(t.lt2.maxChargeCurrent_mA);
    p.i16(t.lt2.maxDischargeCurrent_mA);
    p.i16(t.lt2.maxAvgDsgCurrent_mA);
    p.i16(t.lt2.maxAvgDsgPower_cW);
    p.i8(t.lt2.maxTempCell_C);
    p.i8(t.lt2.minTempCell_C);
    p.i8(t.lt2.maxDeltaCellTemp_C);
    p.i8(t.lt2.maxTempIntSensor_C);
    p.i8(t.lt2.minTempIntSensor_C);
    p.i8(t.lt2.maxTempFet_C);
    p.u8(t.lt2.numShutdowns);
    p.u8(t.lt2.numPartialResets);
    p.u8(t.lt2.numFullResets);
    p.u8(t.lt2.numWdtResets);
    for (int i = 0; i < 7; ++i) p.u8(t.lt2.cbTimeCell_h[i]);

    p.u16(t.lt3.totalFwRuntime);
    p.u16(t.lt3.timeInUT);
    p.u16(t.lt3.timeInLT);
    p.u16(t.lt3.timeInSTL);
    p.u16(t.lt3.timeInRT);
    p.u16(t.lt3.timeInSTH);
    p.u16(t.lt3.timeInHT);
    p.u16(t.lt3.timeInOT);

    return p.overflowed() ? 0 : p.len;
}

// -----------------------------------------------------------------------------
// GATT access
// -----------------------------------------------------------------------------

// A long read arrives as several Read Blob requests, and NimBLE rebuilds the
// whole value for each one before trimming to the offset. Serving DETAIL from a
// copy that only refreshes between reads keeps the pieces from two different
// snapshots out of one result.
static uint8_t s_detailBuf[DETAIL_BUF_BYTES];
static size_t  s_detailLen;
static int64_t s_detailAtUs;

// SETTINGS reads the gauge's data flash, so it is cached the same way, and
// also rebuilt as soon as a write lands.
#define SETTINGS_BUF_BYTES 512   // ATT maximum; 42 settings use 254
static uint8_t  s_settingsBuf[SETTINGS_BUF_BYTES];
static size_t   s_settingsLen;
static int64_t  s_settingsAtUs;
static uint32_t s_settingsGen;

// DFREAD: the client writes the range it wants, then reads it. The bytes are
// fetched once, on the write, so the Read Blob pieces of one long read all
// come from the same snapshot.
#define DFREAD_VERSION 1
static uint8_t s_dfBuf[5 + DF_READ_MAX];
static size_t  s_dfLen;

static int writeDfRead(os_mbuf *om)
{
    uint8_t  req[4];
    uint16_t len = 0;
    if (OS_MBUF_PKTLEN(om) != sizeof(req) ||
        ble_hs_mbuf_to_flat(om, req, sizeof(req), &len) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    const uint16_t addr = req[0] | (req[1] << 8);
    const uint16_t n    = req[2] | (req[3] << 8);
    s_dfLen = 0;
    if (!dfReadRaw(addr, n, &s_dfBuf[5])) return BLE_ATT_ERR_UNLIKELY;
    s_dfBuf[0] = DFREAD_VERSION;
    s_dfBuf[1] = req[0];
    s_dfBuf[2] = req[1];
    s_dfBuf[3] = req[2];
    s_dfBuf[4] = req[3];
    s_dfLen = 5 + n;
    return 0;
}

static int readValue(Chr which, os_mbuf *om)
{
    int rc;
    if (which == CHR_DFREAD) {
        if (s_dfLen == 0) return BLE_ATT_ERR_UNLIKELY;
        rc = os_mbuf_append(om, s_dfBuf, s_dfLen);
    } else if (which == CHR_SETTINGS) {
        const int64_t  nowUs = esp_timer_get_time();
        const uint32_t gen   = dfSettingsGeneration();
        if (s_settingsLen == 0 || gen != s_settingsGen || nowUs - s_settingsAtUs > 2000000) {
            s_settingsLen  = dfSettingsBuildPacket(s_settingsBuf, sizeof(s_settingsBuf));
            s_settingsAtUs = nowUs;
            s_settingsGen  = gen;
        }
        if (s_settingsLen == 0) return BLE_ATT_ERR_UNLIKELY;
        rc = os_mbuf_append(om, s_settingsBuf, s_settingsLen);
    } else if (which == CHR_DETAIL) {
        const int64_t nowUs = esp_timer_get_time();
        if (s_detailLen == 0 || nowUs - s_detailAtUs > 1000000) {
            s_detailLen  = buildDetail(s_detailBuf, sizeof(s_detailBuf));
            s_detailAtUs = nowUs;
        }
        if (s_detailLen == 0) return BLE_ATT_ERR_UNLIKELY;
        rc = os_mbuf_append(om, s_detailBuf, s_detailLen);
    } else {
        uint8_t buf[LIVE_BUF_BYTES];
        size_t n = (which == CHR_LIVE) ? buildLive(buf, sizeof(buf))
                                       : buildInfo(buf, sizeof(buf));
        if (n == 0) return BLE_ATT_ERR_UNLIKELY;
        rc = os_mbuf_append(om, buf, n);
    }
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int cmdResult(DfWriteResult r);

static int writeSetting(const uint8_t *buf, uint16_t len)
{
    if (len != 6) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    const int32_t value = (int32_t)((uint32_t)buf[2] | ((uint32_t)buf[3] << 8) |
                                    ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 24));
    return cmdResult(dfSettingsWrite(buf[1], value));
}

static int cmdResult(DfWriteResult r)
{
    switch (r) {
    case DfWriteResult::Ok:           return 0;
    case DfWriteResult::UnknownId:    return CMD_ERR_BAD_SETTING;
    case DfWriteResult::OutOfRange:   return CMD_ERR_RANGE;
    case DfWriteResult::Conflict:     return CMD_ERR_CONFLICT;
    case DfWriteResult::Sealed:       return CMD_ERR_SEALED;
    case DfWriteResult::BusBusy:      return CMD_ERR_BUSY;
    case DfWriteResult::WriteFailed:  return CMD_ERR_WRITE;
    case DfWriteResult::VerifyFailed: return CMD_ERR_VERIFY;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int writeDf(const uint8_t *buf, uint16_t len)
{
    if (len != 7) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    const uint16_t addr  = buf[1] | (buf[2] << 8);
    const int32_t  value = (int32_t)((uint32_t)buf[3] | ((uint32_t)buf[4] << 8) |
                                     ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24));
    return cmdResult(dfWriteAddress(addr, value));
}

// CMD payload: one opcode byte, then the argument.
//   0x01 "CONFIRM"          DeviceReset when Mfg Status Init keeps FET_EN
//   0x01 "CONFIRM-UNSAFE"   DeviceReset when it does not
//   0x02 id value(i32 LE)   write one curated data flash setting
//   0x03 addr(u16) value(i32) write any catalog parameter by address
// Same token rule as POST /api/reset: a client that does not know a reset would
// strip FET control cannot send the token that allows it.
static int writeCmd(os_mbuf *om)
{
    uint8_t  buf[32];
    uint16_t len = 0;
    if (OS_MBUF_PKTLEN(om) >= sizeof(buf) ||
        ble_hs_mbuf_to_flat(om, buf, sizeof(buf) - 1, &len) != 0 || len < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    buf[len] = '\0';

    if (buf[0] == CMD_OP_SETTING) return writeSetting(buf, len);
    if (buf[0] == CMD_OP_DF_WRITE) return writeDf(buf, len);
    if (buf[0] != CMD_OP_RESET) return CMD_ERR_UNKNOWN_OP;

    const char *arg   = (const char *)&buf[1];
    const bool  safe  = telemetryResetIsSafe();
    const char *token = safe ? "CONFIRM" : "CONFIRM-UNSAFE";
    if (strcmp(arg, token) != 0) {
        ESP_LOGW(TAG, "reset rejected: confirmation token did not match");
        return CMD_ERR_BAD_TOKEN;
    }
    return telemetryRequestReset() == ESP_OK ? 0 : CMD_ERR_FAILED;
}

static int accessCb(uint16_t, uint16_t, ble_gatt_access_ctxt *ctxt, void *arg)
{
    const Chr which = (Chr)(intptr_t)arg;
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        return readValue(which, ctxt->om);
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (which == CHR_CMD)    return writeCmd(ctxt->om);
        if (which == CHR_DFREAD) return writeDfRead(ctxt->om);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// Zero-initialized then assigned field by field, as in http_server.cpp: the
// NimBLE structs gain members between IDF releases, and -Wextra rejects a
// designated initializer that leaves any of them out. The all-zero entry at
// the end of each array is the terminator NimBLE looks for.
static ble_gatt_chr_def s_chrs[7];
static ble_gatt_svc_def s_svcs[2];

static void buildGattTable()
{
    auto chr = [](int i, const ble_uuid128_t &uuid, Chr which, ble_gatt_chr_flags flags) {
        s_chrs[i].uuid      = &uuid.u;
        s_chrs[i].access_cb = accessCb;
        s_chrs[i].arg       = (void *)(intptr_t)which;
        s_chrs[i].flags     = flags;
    };
    chr(0, kLiveUuid,   CHR_LIVE,   BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY);
    chr(1, kInfoUuid,   CHR_INFO,   BLE_GATT_CHR_F_READ);
    chr(2, kDetailUuid, CHR_DETAIL, BLE_GATT_CHR_F_READ);
    // CMD needs an authenticated (passkey) link. Reads stay open so the app
    // can show the pack without pairing first.
    chr(3, kCmdUuid,    CHR_CMD,    BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC |
                                    BLE_GATT_CHR_F_WRITE_AUTHEN);
    chr(4, kSettingsUuid, CHR_SETTINGS, BLE_GATT_CHR_F_READ);
    // Selecting a range to read changes nothing on the gauge, so it needs no
    // pairing -- only writes through CMD do.
    chr(5, kDfReadUuid,   CHR_DFREAD,   BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE);
    s_chrs[0].val_handle = &s_liveHandle;

    s_svcs[0].type            = BLE_GATT_SVC_TYPE_PRIMARY;
    s_svcs[0].uuid            = &kSvcUuid.u;
    s_svcs[0].characteristics = s_chrs;
}

// -----------------------------------------------------------------------------
// GAP
// -----------------------------------------------------------------------------
static int gapEvent(ble_gap_event *event, void *);

static void startAdvertising()
{
    // The service UUID goes in the advertisement so the app can filter scans
    // by it; the name does not fit beside it and rides in the scan response.
    ble_hs_adv_fields adv = {};
    adv.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.uuids128              = &kSvcUuid;
    adv.num_uuids128          = 1;
    adv.uuids128_is_complete  = 1;
    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv fields: %d", rc);
        return;
    }

    ble_hs_adv_fields rsp = {};
    rsp.name             = (const uint8_t *)s_name;
    rsp.name_len         = strlen(s_name);
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "scan response fields: %d", rc);
        return;
    }

    ble_gap_adv_params params = {};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_ownAddrType, nullptr, BLE_HS_FOREVER, &params,
                           gapEvent, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv start: %d", rc);
    }
}

static int gapEvent(ble_gap_event *event, void *)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected (handle %d)", s_conn);
        } else {
            startAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected (reason 0x%03x)", event->disconnect.reason);
        s_conn           = BLE_HS_CONN_HANDLE_NONE;
        s_liveSubscribed = false;
        startAdvertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        startAdvertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_liveHandle) {
            s_liveSubscribed = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU %d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "encryption %s", event->enc_change.status == 0 ? "on" : "failed");
        return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        // DisplayOnly: we "display" a fixed key and the phone asks the user
        // to type it in.
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            ble_sm_io io = {};
            io.action  = BLE_SM_IOACT_DISP;
            io.passkey = BLE_PASSKEY;
            ble_sm_inject_io(event->passkey.conn_handle, &io);
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // The phone forgot the bond but we did not. Drop ours and let it pair
        // again rather than locking it out.
        ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        return 0;
    }
}

static void onSync()
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_ownAddrType);
    startAdvertising();
    ESP_LOGI(TAG, "advertising as \"%s\"", s_name);
}

static void onReset(int reason)
{
    ESP_LOGW(TAG, "host reset, reason %d", reason);
}

static void hostTask(void *)
{
    nimble_port_run();               // returns only after nimble_port_stop()
    nimble_port_freertos_deinit();
}

// -----------------------------------------------------------------------------
// LIVE push
// -----------------------------------------------------------------------------
static void notifyTask(void *)
{
    uint8_t buf[LIVE_BUF_BYTES];
    bool    warnedMtu = false;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(NOTIFY_PERIOD_MS));

        const uint16_t conn = s_conn;
        if (conn == BLE_HS_CONN_HANDLE_NONE || !s_liveSubscribed) {
            warnedMtu = false;
            continue;
        }

        const size_t n = buildLive(buf, sizeof(buf));
        if (n == 0) continue;

        // A truncated notification would be worse than none. A client stuck
        // at a small MTU can still poll LIVE with a long read.
        const uint16_t mtu = ble_att_mtu(conn);
        if (mtu < 3 || n > (size_t)(mtu - 3)) {
            if (!warnedMtu) {
                ESP_LOGW(TAG, "MTU %u too small for %u-byte LIVE; client must poll",
                         mtu, (unsigned)n);
                warnedMtu = true;
            }
            continue;
        }

        os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
        if (om) ble_gatts_notify_custom(conn, s_liveHandle, om);   // consumes om
    }
}

// -----------------------------------------------------------------------------
// Public
// -----------------------------------------------------------------------------
esp_err_t bleServerStart()
{
    // NVS must already be up: wifiApStart() does it, and bonds are kept there.
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_BT);
    snprintf(s_name, sizeof(s_name), "HeroBMS-%02X%02X", mac[4], mac[5]);

    ble_hs_cfg.sync_cb         = onSync;
    ble_hs_cfg.reset_cb        = onReset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_mitm           = 1;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    buildGattTable();
    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc == 0) rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT table: %d", rc);
        return ESP_FAIL;
    }
    ble_svc_gap_device_name_set(s_name);
    ble_store_config_init();

    nimble_port_freertos_init(hostTask);

    if (xTaskCreate(notifyTask, "ble_notify", 3072, nullptr, 4, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "could not create notify task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
