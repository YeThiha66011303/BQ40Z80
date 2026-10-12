// =============================================================================
// Telemetry snapshot -- READ ONLY. See telemetry.h for the threading contract.
// =============================================================================
#include "telemetry.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

static const char *TAG = "telemetry";

// Fast tier: the values a person watches change. Slow tier: Impedance Track
// internals and lifetime records, which move over hours.
#define FAST_POLL_MS   1000
#define SLOW_POLL_MS  30000

// The gauge is considered offline once this many fast polls in a row fail
// outright. One failed read is usually a bus glitch, not a missing pack.
#define OFFLINE_AFTER_FAILS 3

static BQ40Z80         *s_gauge = nullptr;
static SemaphoreHandle_t s_lock    = nullptr;  // guards the snapshot
static SemaphoreHandle_t s_busLock = nullptr;  // guards the SMBus
static Telemetry        s_data  = {};
static StaticInfo       s_static = {};

// -----------------------------------------------------------------------------
// Polling
// -----------------------------------------------------------------------------
// Every read is allowed to fail independently: a pack that drops one block
// should still publish the other forty values rather than blanking the page.
#define TRY(expr) do { if ((expr) != ESP_OK) ++fails; } while (0)

static void pollFast(Telemetry &t, int &fails)
{
    BQ40Z80 &g = *s_gauge;

    TRY(g.getVoltage(t.voltage_mV));
    TRY(g.getCurrent(t.current_mA));
    TRY(g.getAverageCurrent(t.avgCurrent_mA));
    TRY(g.getTemperature(t.temperature_dK));
    TRY(g.getRelativeStateOfCharge(t.rsoc_pct));
    TRY(g.getAbsoluteStateOfCharge(t.asoc_pct));
    TRY(g.getRemainingCapacity(t.remainingCapacity_mAh));
    TRY(g.getFullChargeCapacity(t.fullChargeCapacity_mAh));
    TRY(g.getChargingCurrent(t.chargingCurrent_mA));
    TRY(g.getChargingVoltage(t.chargingVoltage_mV));
    TRY(g.getBatteryStatus(t.batteryStatus));

    TRY(g.readDAStatus1(t.da1));
    TRY(g.readDAStatus2(t.da2));
    TRY(g.readDAStatus3(t.da3));
    TRY(g.getOperationStatus(t.op));
    TRY(g.getSafetyStatus(t.safetyStatus));
    TRY(g.getSafetyAlert(t.safetyAlert));
    TRY(g.getManufacturingStatus(t.mfg));
    TRY(g.getChargingStatus(t.chargingStatus));
    TRY(g.getGaugingStatus(t.gaugingStatus));
}

static void pollSlow(Telemetry &t, int &fails)
{
    BQ40Z80 &g = *s_gauge;

    TRY(g.getMaxError(t.maxError_pct));
    TRY(g.getBatteryMode(t.batteryMode));
    TRY(g.getCycleCount(t.cycleCount));
    TRY(g.getStateOfHealth(t.stateOfHealth_pct));
    TRY(g.getRunTimeToEmpty(t.runTimeToEmpty_min));
    TRY(g.getAverageTimeToEmpty(t.avgTimeToEmpty_min));
    TRY(g.getAverageTimeToFull(t.avgTimeToFull_min));
    TRY(g.getRemainingCapacityAlarm(t.remCapAlarm_mAh));
    TRY(g.getRemainingTimeAlarm(t.remTimeAlarm_min));
    TRY(g.getAtRate(t.atRate_mA));
    TRY(g.getAtRateTimeToFull(t.atRateTimeToFull_min));
    TRY(g.getAtRateTimeToEmpty(t.atRateTimeToEmpty_min));
    TRY(g.getAtRateOK(t.atRateOK));
    TRY(g.getMaxTurboPower(t.maxTurboPower_cW));
    TRY(g.getSustainedTurboPower(t.susTurboPower_cW));
    TRY(g.getMaxTurboCurrent(t.maxTurboCurrent_mA));
    TRY(g.getSustainedTurboCurrent(t.susTurboCurrent_mA));
    TRY(g.getTurboPackResistance(t.turboPackR_mOhm));
    TRY(g.getTurboSysResistance(t.turboSysR_mOhm));
    TRY(g.getTurboEdv(t.turboEdv_mV));
    TRY(g.getNoLoadRemainingCapacity(t.noLoadRemCap_mAh));
    TRY(g.getStateOfHealthFcc(t.sohFcc_mAh, t.sohFcc_cWh));
    TRY(g.getPfStatusRaw(t.pfStatus));
    TRY(g.getPfAlertRaw(t.pfAlert));

    TRY(g.getGaugingStatus1(t.g1));
    TRY(g.readGaugingStatus2(t.g2));
    TRY(g.getGaugingStatus3(t.g3));
    TRY(g.getGaugingStatus4(t.g4));
    TRY(g.readFilterCapacity(t.filt));
    TRY(g.readCBStatus(t.cb));
    TRY(g.readLifetimeData1(t.lt1));
    TRY(g.readLifetimeData2(t.lt2));
    TRY(g.readLifetimeData3(t.lt3));
}

static void pollTask(void *)
{
    Telemetry working = {};
    uint32_t  consecutiveTotalFailures = 0;
    int64_t   nextSlowUs = 0;

    for (;;) {
        int fails = 0;
        int64_t nowUs = esp_timer_get_time();

        bool didSlow = false;
        int  fastFails = 0;

        if (telemetryLockBus(2000)) {
            pollFast(working, fails);
            fastFails = fails;
            if (nowUs >= nextSlowUs) {
                pollSlow(working, fails);
                nextSlowUs = nowUs + (int64_t)SLOW_POLL_MS * 1000;
                didSlow = true;
            }
            telemetryUnlockBus();
        } else {
            ESP_LOGW(TAG, "bus busy, skipping this poll");
            vTaskDelay(pdMS_TO_TICKS(FAST_POLL_MS));
            continue;
        }

        // Every fast read failing means nothing is answering at 0x0B.
        if (fastFails >= 20) {
            ++consecutiveTotalFailures;
        } else {
            consecutiveTotalFailures = 0;
        }

        working.online      = (consecutiveTotalFailures < OFFLINE_AFTER_FAILS);
        working.uptime_s    = (uint32_t)(nowUs / 1000000);
        working.pollCount  += 1;
        working.errorCount += fails;
        if (didSlow) working.lastSlowPoll_s = working.uptime_s;

        if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) == pdTRUE) {
            s_data = working;
            xSemaphoreGive(s_lock);
        }

        vTaskDelay(pdMS_TO_TICKS(FAST_POLL_MS));
    }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void telemetryInit(BQ40Z80 *gauge)
{
    s_gauge = gauge;
    if (!s_lock)    s_lock    = xSemaphoreCreateMutex();
    if (!s_busLock) s_busLock = xSemaphoreCreateMutex();
}

bool telemetryReadStatic()
{
    if (!s_gauge) return false;
    if (!telemetryLockBus(3000)) return false;

    BQ40Z80 &g = *s_gauge;
    StaticInfo s = {};

    g.getManufacturerName(s.manufacturer, sizeof(s.manufacturer));
    g.getDeviceName(s.deviceName, sizeof(s.deviceName));
    g.getDeviceChemistry(s.chemistry, sizeof(s.chemistry));
    g.getChemId(s.chemId);
    g.getDeviceType(s.deviceType);
    g.getSerialNumber(s.serialNumber);
    g.getDesignCapacity(s.designCapacity_mAh);
    g.getDesignVoltage(s.designVoltage_mV);
    g.getManufactureDate(s.manufactureDate);
    s.enabledProtectionsCValid =
        (g.getEnabledProtectionsC(s.enabledProtectionsC) == ESP_OK);
    s.mfgStatusInitValid = (g.getMfgStatusInit(s.mfgStatusInit) == ESP_OK);

    // DeviceType is the cheapest proof that something actually answered. If it
    // did not, the pack is absent and the caller should try again later rather
    // than publish a block of zeros as though it were real.
    s.valid = (s.deviceType != 0);

    telemetryUnlockBus();

    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
        s_static = s;
        xSemaphoreGive(s_lock);
    } else {
        s_static = s;
    }
    return s.valid;
}

bool telemetryStart()
{
    if (!s_gauge || !s_lock) {
        ESP_LOGE(TAG, "telemetryInit() must be called first");
        return false;
    }
    // 6 kB: the snapshot is a local in pollTask and the structs are sizeable.
    BaseType_t ok = xTaskCreate(pollTask, "bq_poll", 6144, nullptr, 5, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "could not create poll task");
        return false;
    }
    ESP_LOGI(TAG, "poll task started (fast %d ms, slow %d ms)",
             FAST_POLL_MS, SLOW_POLL_MS);
    return true;
}

void telemetryGet(Telemetry &out)
{
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
        out = s_data;
        xSemaphoreGive(s_lock);
    } else {
        out = s_data;
    }
}

bool telemetryLockBus(uint32_t timeoutMs)
{
    if (!s_busLock) return true;   // not initialized yet: single-threaded
    return xSemaphoreTake(s_busLock, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void telemetryUnlockBus()
{
    if (s_busLock) xSemaphoreGive(s_busLock);
}

void telemetryGetStatic(StaticInfo &out)
{
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
        out = s_static;
        xSemaphoreGive(s_lock);
    } else {
        out = s_static;
    }
}

// -----------------------------------------------------------------------------
// Reset -- the only call in this firmware that changes the gauge
// -----------------------------------------------------------------------------
static ResetInfo s_reset = {};

bool telemetryResetIsSafe()
{
    StaticInfo s;
    telemetryGetStatic(s);
    if (!s.mfgStatusInitValid) return false;   // unknown counts as unsafe
    return ((s.mfgStatusInit >> 4) & 1) != 0;  // FET_EN
}

esp_err_t telemetryRequestReset()
{
    if (!s_gauge) return ESP_ERR_INVALID_STATE;

    if (!telemetryLockBus(3000)) {
        ESP_LOGE(TAG, "reset aborted: bus busy");
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGW(TAG, "issuing DeviceReset (MAC 0x0041) to the gauge");
    esp_err_t err = s_gauge->deviceReset();
    telemetryUnlockBus();

    s_reset.everTried = true;
    s_reset.count    += 1;
    s_reset.lastErr   = (int)err;
    s_reset.lastAt_s  = (uint32_t)(esp_timer_get_time() / 1000000);

    if (err == ESP_OK) {
        ESP_LOGW(TAG, "reset sent; the gauge will be unresponsive briefly");
    } else {
        ESP_LOGE(TAG, "reset failed: %s", esp_err_to_name(err));
    }
    return err;
}

void telemetryGetResetInfo(ResetInfo &out)
{
    out = s_reset;
}
