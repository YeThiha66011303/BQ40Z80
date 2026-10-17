// =============================================================================
// BQ40Z80 monitor
//
// This firmware never changes the state of the gauge on its own. It issues no
// FET toggles, no FET_EN or GAUGE_EN changes, no seal/unseal. Protection and
// charge control belong entirely to the BQ40Z80's own firmware.
//
// The only state changes are user actions from the paired phone app: the
// guarded DeviceReset, and writes to a fixed list of data flash settings with
// range checks and read-back verification (df_settings.cpp). The web
// dashboard stays read only.
//
// NOTE ON "WRITES": reading a ManufacturerAccess block (DAStatus, OperationStatus,
// SafetyStatus, GaugingStatus, QMax, ...) requires writing the 2-byte subcommand
// to ManufacturerBlockAccess (0x44) before reading the payload back from
// ManufacturerData (0x23). That write is part of the read transaction and does
// not alter device state. It is not a control command.
// =============================================================================
#include "BQ40Z80.h"
#include "ble_server.h"
#include "df_settings.h"
#include "http_server.h"
#include "telemetry.h"
#include "wifi_ap.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

static const char *TAG = "app_main";

#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#define SERIAL_LOG_INTERVAL_MS 10000
#define CELL_COUNT       6

static void logStaticConfig(BQ40Z80 &gauge)
{
    ESP_LOGI(TAG, "===== Static configuration (read once) =====");

    char chem[16] = {0};
    if (gauge.getDeviceChemistry(chem, sizeof(chem)) == ESP_OK) {
        ESP_LOGI(TAG, "DeviceChemistry(): \"%s\"", chem);
    }

    uint16_t chemId = 0;
    if (gauge.getChemId(chemId) == ESP_OK) {
        bool isLfp = ((chemId & 0xFF00) == 0x0400) || ((chemId & 0xF000) == 0x4000);
        ESP_LOGI(TAG, "ChemID: %04X (%s)", chemId,
                 isLfp ? "LiFePO4 family" : "NOT an LFP id -- wrong OCV curve for these cells");
    }

    uint16_t designCapMah = 0;
    if (gauge.getDesignCapacity(designCapMah) == ESP_OK) {
        ESP_LOGI(TAG, "DesignCapacity: %u mAh", designCapMah);
    }

    uint8_t protC = 0;
    if (gauge.getEnabledProtectionsC(protC) == ESP_OK) {
        bool ocArmed = (protC & 0x40) != 0;
        ESP_LOGI(TAG, "Enabled Protections C: 0x%02X -- OC (Overcharge) is %s",
                 protC, ocArmed ? "ENABLED" : "DISABLED");
        if (!ocArmed) {
            ESP_LOGW(TAG, "  OC disabled: temporary learning-cycle state. COV is the only");
            ESP_LOGW(TAG, "  overcharge backstop. Restore 0x%02X once FCC has learned.",
                     (unsigned)(protC | 0x40));
        }
    }

    CellVoltageLimits lim = {};
    if (gauge.getCellVoltageLimits(lim) == ESP_OK) {
        ESP_LOGI(TAG, "COV trip / recovery:   %d / %d mV",
                 lim.covThresholdStdTempLow_mV, lim.covRecoveryStdTempLow_mV);
        ESP_LOGI(TAG, "CUV trip / recovery:   %d / %d mV",
                 lim.cuvThreshold_mV, lim.cuvRecovery_mV);
        ESP_LOGI(TAG, "Charging voltage per temp range (LowT/StdL/StdH/HighT/RecT):");
        ESP_LOGI(TAG, "  %d / %d / %d / %d / %d mV",
                 lim.chargingVoltageLowTemp_mV, lim.chargingVoltageStdTempLow_mV,
                 lim.chargingVoltageStdTempHigh_mV, lim.chargingVoltageHighTemp_mV,
                 lim.chargingVoltageRecTemp_mV);
        ESP_LOGI(TAG, "Voltage range Low/Med/High: %d / %d / %d mV",
                 lim.voltageRangeLow_mV, lim.voltageRangeMed_mV, lim.voltageRangeHigh_mV);
        ESP_LOGI(TAG, "Precharge current / start V: %d mA / %d mV",
                 lim.prechargeCurrent_mA, lim.prechargeStartVoltage_mV);
    }

    OvercurrentConfig occ = {};
    if (gauge.getOvercurrentConfig(occ) == ESP_OK) {
        ESP_LOGI(TAG, "OCC1/OCC2 (charge trip):    %d / %d mA",
                 occ.occ1Threshold_mA, occ.occ2Threshold_mA);
        ESP_LOGI(TAG, "OCD1/OCD2 (discharge trip): %d / %d mA",
                 occ.ocd1Threshold_mA, occ.ocd2Threshold_mA);
    }

    ChargingCurrentConfig ccc = {};
    if (gauge.getChargingCurrentConfig(ccc) == ESP_OK) {
        ESP_LOGI(TAG, "Charge current table, V range Low/Med/High:");
        ESP_LOGI(TAG, "  Std Temp Low:  %d / %d / %d mA",
                 ccc.stdTempLow_mA[0], ccc.stdTempLow_mA[1], ccc.stdTempLow_mA[2]);
        ESP_LOGI(TAG, "  Std Temp High: %d / %d / %d mA",
                 ccc.stdTempHigh_mA[0], ccc.stdTempHigh_mA[1], ccc.stdTempHigh_mA[2]);
        ESP_LOGI(TAG, "  Rec Temp:      %d / %d / %d mA",
                 ccc.recTemp_mA[0], ccc.recTemp_mA[1], ccc.recTemp_mA[2]);
    }

    ManufacturingStatus mfg = {};
    if (gauge.getManufacturingStatus(mfg) == ESP_OK) {
        ESP_LOGI(TAG, "ManufacturingStatus: 0x%04X  FET_EN=%d GAUGE_EN=%d",
                 mfg.raw, mfg.fetEn, mfg.gaugeEn);
        if (!mfg.fetEn) {
            ESP_LOGW(TAG, "  FET_EN=0: gauge firmware is NOT driving the FETs.");
            ESP_LOGW(TAG, "  Protections cannot open them. Set FET_EN in BQStudio.");
        }
        if (!mfg.gaugeEn) {
            ESP_LOGW(TAG, "  GAUGE_EN=0: Impedance Track gauging is disabled.");
        }
    }

    ESP_LOGI(TAG, "===== End static configuration =====");
}

// -----------------------------------------------------------------------------
// Only the values that actually move, so a trip is visible rather than buried.
// -----------------------------------------------------------------------------
static void logLiveData(BQ40Z80 &gauge)
{
    // ---- Cell voltages: 1-4 from DAStatus1, 5-7 from DAStatus3 --------------
    DAStatus1 da1 = {};
    DAStatus3 da3 = {};
    esp_err_t da1Err = gauge.readDAStatus1(da1);
    esp_err_t da3Err = gauge.readDAStatus3(da3);

    if (da1Err == ESP_OK && da3Err == ESP_OK) {
        int16_t cells[CELL_COUNT] = {
            da1.cellVoltage1_mV, da1.cellVoltage2_mV,
            da1.cellVoltage3_mV, da1.cellVoltage4_mV,
            da3.cellVoltage5_mV, da3.cellVoltage6_mV
        };

        int16_t minMv = cells[0], maxMv = cells[0];
        int32_t sumMv = 0;
        for (int i = 0; i < CELL_COUNT; i++) {
            if (cells[i] < minMv) minMv = cells[i];
            if (cells[i] > maxMv) maxMv = cells[i];
            sumMv += cells[i];
        }

        ESP_LOGI(TAG, "Cells: %d %d %d %d %d %d mV",
                 cells[0], cells[1], cells[2], cells[3], cells[4], cells[5]);
        ESP_LOGI(TAG, "  min=%d max=%d spread=%d | sum=%ld BAT=%d PACK=%d mV",
                 minMv, maxMv, (int)(maxMv - minMv), (long)sumMv,
                 da1.batVoltage_mV, da1.packVoltage_mV);
    } else {
        if (da1Err != ESP_OK) ESP_LOGE(TAG, "readDAStatus1() failed: %s", esp_err_to_name(da1Err));
        if (da3Err != ESP_OK) ESP_LOGE(TAG, "readDAStatus3() failed: %s", esp_err_to_name(da3Err));
    }

    // ---- Pack level ---------------------------------------------------------
    uint16_t packMv = 0;
    int16_t  packMa = 0;
    uint16_t tempDeciK = 0;
    bool haveCurrent = (gauge.getCurrent(packMa) == ESP_OK);

    if (gauge.getVoltage(packMv) == ESP_OK && haveCurrent) {
        ESP_LOGI(TAG, "Pack: %u mV, %d mA", packMv, packMa);
    }
    if (gauge.getTemperature(tempDeciK) == ESP_OK) {
        ESP_LOGI(TAG, "Temperature: %.1f C", (tempDeciK / 10.0f) - 273.15f);
    }

    uint16_t chgMa = 0, chgMv = 0;
    if (gauge.getChargingCurrent(chgMa) == ESP_OK &&
        gauge.getChargingVoltage(chgMv) == ESP_OK) {
        ESP_LOGI(TAG, "Requested of charger: %u mA @ %u mV", chgMa, chgMv);
    }

    uint16_t remainMah = 0, fccMah = 0;
    bool haveRemain = (gauge.getRemainingCapacity(remainMah) == ESP_OK);
    bool haveFcc    = (gauge.getFullChargeCapacity(fccMah) == ESP_OK);
    if (haveRemain && haveFcc) {
        ESP_LOGI(TAG, "Capacity: %u / %u mAh%s", remainMah, fccMah,
                 (fccMah == 0) ? "  (FCC=0: no learning cycle completed yet)" : "");
    }

    // ---- FET and protection state ------------------------------------------
    OperationStatus op = {};
    if (gauge.getOperationStatus(op) == ESP_OK) {
        ESP_LOGI(TAG, "FETs: CHG=%d DSG=%d PCHG=%d PDSG=%d | XCHG=%d XDSG=%d | SEC=%s",
                 op.chg, op.dsg, op.pchg, op.pdsg, op.xchg, op.xdsg,
                 operationStatusSecName(op.sec));
        if (op.pf) {
            ESP_LOGE(TAG, "  PERMANENT FAILURE is set.");
        }
    }

    SafetyStatus safety = {};
    if (gauge.getSafetyStatus(safety) == ESP_OK) {
        if (safety.raw == 0) {
            ESP_LOGI(TAG, "SafetyStatus: clean");
        } else {
            ESP_LOGW(TAG, "SafetyStatus raw: 0x%08lX", (unsigned long)safety.raw);
            if (safety.cuv)   ESP_LOGW(TAG, "  CUV   Cell undervoltage");
            if (safety.cuvc)  ESP_LOGW(TAG, "  CUVC  Cell undervoltage, compensated");
            if (safety.cov)   ESP_LOGW(TAG, "  COV   Cell overvoltage");
            if (safety.covl)  ESP_LOGW(TAG, "  COVL  Cell overvoltage, latched");
            if (safety.occ1)  ESP_LOGW(TAG, "  OCC1  Overcurrent in charge 1");
            if (safety.occ2)  ESP_LOGW(TAG, "  OCC2  Overcurrent in charge 2");
            if (safety.ocd1)  ESP_LOGW(TAG, "  OCD1  Overcurrent in discharge 1");
            if (safety.ocd2)  ESP_LOGW(TAG, "  OCD2  Overcurrent in discharge 2");
            if (safety.ocdl)  ESP_LOGW(TAG, "  OCDL  Overcurrent in discharge, latched");
            if (safety.aold)  ESP_LOGW(TAG, "  AOLD  Overload in discharge");
            if (safety.aoldl) ESP_LOGW(TAG, "  AOLDL Overload in discharge, latched");
            if (safety.ascc)  ESP_LOGW(TAG, "  ASCC  Short circuit in charge");
            if (safety.asccl) ESP_LOGW(TAG, "  ASCCL Short circuit in charge, latched");
            if (safety.ascd)  ESP_LOGW(TAG, "  ASCD  Short circuit in discharge");
            if (safety.ascdl) ESP_LOGW(TAG, "  ASCDL Short circuit in discharge, latched");
            if (safety.oc)    ESP_LOGW(TAG, "  OC    Overcharge (capacity based)");
            if (safety.chgv)  ESP_LOGW(TAG, "  CHGV  Overcharging voltage");
            if (safety.chgc)  ESP_LOGW(TAG, "  CHGC  Overcharging current");
            if (safety.pchgc) ESP_LOGW(TAG, "  PCHGC Over-precharge current");
            if (safety.otc)   ESP_LOGW(TAG, "  OTC   Overtemperature in charge");
            if (safety.otd)   ESP_LOGW(TAG, "  OTD   Overtemperature in discharge");
            if (safety.otf)   ESP_LOGW(TAG, "  OTF   Overtemperature FET");
            if (safety.utc)   ESP_LOGW(TAG, "  UTC   Undertemperature in charge");
            if (safety.utd)   ESP_LOGW(TAG, "  UTD   Undertemperature in discharge");
            if (safety.cto)   ESP_LOGW(TAG, "  CTO   Charge timeout");
            if (safety.pto)   ESP_LOGW(TAG, "  PTO   Precharge timeout");
        }
    }

    // ---- Gauging state ------------------------------------------------------
    GaugingStatus gs = {};
    if (gauge.getGaugingStatus(gs) == ESP_OK) {
        const char *state;
        if (!gs.dsg) {
            state = "CHARGE";
        } else if (haveCurrent && abs(packMa) <= 50) {
            state = gs.rest ? "RELAX (OCV taken)" : "RELAX (OCV not taken yet)";
        } else {
            state = "DISCHARGE";
        }
        ESP_LOGI(TAG, "Gauging: %s | QEN=%d VOK=%d REST=%d | TC=%d FC=%d TD=%d FD=%d",
                 state, gs.qen, gs.vok, gs.rest, gs.tc, gs.fc, gs.td, gs.fd);
    }
}

// =============================================================================
// Serial commands
//
// Typed into the ESP-IDF monitor. Reset is two steps on purpose: "reset"
// explains what it will cost and arms a short window, "yes" inside that window
// actually sends it. That mirrors the confirm dialog on the web page, so
// neither route can fire on a single keystroke.
// =============================================================================
#define CONFIRM_WINDOW_US (30LL * 1000000)

static int64_t s_confirmDeadlineUs = 0;

static void printResetWarning()
{
    const bool safe = telemetryResetIsSafe();
    StaticInfo s;
    telemetryGetStatic(s);

    ESP_LOGW(TAG, "DeviceReset (MAC 0x0041) will:");
    ESP_LOGW(TAG, "  - reboot the gauge and clear the latched SafetyStatus bits");
    ESP_LOGW(TAG, "  - increment the lifetime reset counters");
    ESP_LOGW(TAG, "  - reload ManufacturingStatus from Mfg Status Init (0x4800)");

    if (!s.mfgStatusInitValid) {
        ESP_LOGE(TAG, "Mfg Status Init could not be read. Treating as UNSAFE.");
    } else if (!safe) {
        ESP_LOGE(TAG, "Mfg Status Init = 0x%04X -- FET_EN is CLEAR.", s.mfgStatusInit);
        ESP_LOGE(TAG, "After the reset the firmware stops driving the CHG, DSG");
        ESP_LOGE(TAG, "and PCHG FETs. The pack will not charge or discharge");
        ESP_LOGE(TAG, "until FET_EN is restored in BQStudio.");
    } else {
        ESP_LOGI(TAG, "Mfg Status Init = 0x%04X -- FET_EN set, FET control survives.",
                 s.mfgStatusInit);
    }
    ESP_LOGW(TAG, "Type 'yes' within 30 s to proceed, anything else to cancel.");
}

static void handleCommand(const char *cmd)
{
    const bool awaitingConfirm =
        (s_confirmDeadlineUs != 0) && (esp_timer_get_time() < s_confirmDeadlineUs);

    if (awaitingConfirm) {
        s_confirmDeadlineUs = 0;
        if (std::strcmp(cmd, "yes") == 0) {
            telemetryRequestReset();
        } else {
            ESP_LOGI(TAG, "Reset cancelled.");
        }
        return;
    }

    if (std::strcmp(cmd, "reset") == 0) {
        printResetWarning();
        s_confirmDeadlineUs = esp_timer_get_time() + CONFIRM_WINDOW_US;
    } else if (std::strcmp(cmd, "status") == 0) {
        Telemetry t;
        telemetryGet(t);
        ESP_LOGI(TAG, "gauge %s | %u mV | %d mA | %u%% | %u/%u mAh",
                 t.online ? "online" : "NOT RESPONDING",
                 t.voltage_mV, t.current_mA, t.rsoc_pct,
                 t.remainingCapacity_mAh, t.fullChargeCapacity_mAh);
    } else if (std::strcmp(cmd, "help") == 0) {
        ESP_LOGI(TAG, "commands: status, reset, help");
    } else {
        ESP_LOGW(TAG, "unknown command '%s' -- try 'help'", cmd);
    }
}

static void serialCommandTask(void *)
{
    char   line[24];
    size_t n = 0;

    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {                 // console VFS is non-blocking
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (c == '\r' || c == '\n') {   // monitors send either
            line[n] = '\0';
            if (n > 0) handleCommand(line);
            n = 0;
        } else if (n + 1 < sizeof(line)) {
            line[n++] = (char)c;
        }
    }
}

extern "C" void app_main(void)
{
    // Static storage: the polling task keeps a pointer to this object for the
    // life of the firmware, so it must outlive app_main's stack frame.
    static BQ40Z80 gauge(SCL_PIN, SDA_PIN, I2C_NUM_0);

    esp_err_t err = gauge.begin();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "begin() failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "BQ40Z80 monitor started (settings change only from the paired app)");

    // Network first. The dashboard is the main way to see what the pack is
    // doing, including the case where the pack is not answering at all, so it
    // must not sit behind a probe that can take half a minute to give up.
    telemetryInit(&gauge);
    dfSettingsInit(&gauge);
    if (!telemetryStart()) {
        ESP_LOGE(TAG, "telemetry task failed to start -- not serving HTTP");
        return;
    }
    ESP_ERROR_CHECK(wifiApStart());
    ESP_ERROR_CHECK(httpServerStart());

    // BLE after Wi-Fi: it relies on the NVS partition wifiApStart() opened.
    // A BLE failure leaves the web dashboard running rather than halting.
    if (bleServerStart() != ESP_OK) {
        ESP_LOGE(TAG, "BLE failed to start -- web dashboard only");
    }

    xTaskCreate(serialCommandTask, "serial_cmd", 4096, nullptr, 3, nullptr);
    ESP_LOGI(TAG, "serial commands ready -- type 'help'");

    // The first MAC read after power-up often times out while the gauge wakes.
    // Retry rather than giving up, but never write anything to provoke it.
    ManufacturingStatus probe = {};
    for (int attempt = 1; attempt <= 40; attempt++) {
        bool ok = false;
        if (telemetryLockBus(3000)) {
            ok = (gauge.getManufacturingStatus(probe) == ESP_OK);
            telemetryUnlockBus();
        }
        if (ok) {
            ESP_LOGI(TAG, "Gauge responding (attempt %d)", attempt);
            break;
        }
        if (attempt == 40) {
            ESP_LOGE(TAG, "Gauge not responding after 40 attempts -- check SMBus wiring");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (telemetryLockBus(5000)) {
        logStaticConfig(gauge);
        telemetryUnlockBus();
    }
    bool staticRead = telemetryReadStatic();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SERIAL_LOG_INTERVAL_MS));

        // A pack plugged in after boot should still fill in the device panel.
        if (!staticRead) {
            Telemetry t;
            telemetryGet(t);
            if (t.online) staticRead = telemetryReadStatic();
        }

        if (telemetryLockBus(3000)) {
            logLiveData(gauge);
            telemetryUnlockBus();
        } else {
            ESP_LOGW(TAG, "bus busy, skipping serial log");
        }
    }
}
