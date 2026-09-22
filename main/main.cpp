#include "BQ40Z80.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstdlib>

static const char *TAG = "app_main";

#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define RUN_FET_TOGGLE_TEST 0
#define RUN_MANUAL_MODE_BOTH_FETS_ON 0
#define RUN_READ_NR_AND_COUNTER_DEC_DELAY 0
#define RUN_GAUGE_CYCLE_TEST 0
#define RUN_READ_CURRENT_LIMITS 1

extern "C" void app_main(void)
{
    BQ40Z80 gauge(SCL_PIN, SDA_PIN, I2C_NUM_0);

    esp_err_t err = gauge.begin();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "begin() failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "BQ40Z80 initialized OK");

    vTaskDelay(pdMS_TO_TICKS(1000));

    ManufacturingStatus mfgBefore = {};
    esp_err_t mfgBeforeErr = ESP_FAIL;
    const int kMaxMfgReadAttempts = 40;
    for (int attempt = 1; attempt <= kMaxMfgReadAttempts; attempt++) {
        mfgBeforeErr = gauge.getManufacturingStatus(mfgBefore);
        if (mfgBeforeErr == ESP_OK) {
            ESP_LOGI(TAG, "getManufacturingStatus() (pre-check) succeeded on attempt %d/%d",
                     attempt, kMaxMfgReadAttempts);
            break;
        }
        ESP_LOGW(TAG, "getManufacturingStatus() (pre-check) attempt %d/%d failed: %s -- retrying...",
                 attempt, kMaxMfgReadAttempts, esp_err_to_name(mfgBeforeErr));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (mfgBeforeErr != ESP_OK) {
        ESP_LOGE(TAG, "getManufacturingStatus() (pre-check) failed after %d attempts, giving up",
                 kMaxMfgReadAttempts);
    } else {
        ESP_LOGI(TAG, "ManufacturingStatus before: raw=0x%04X GAUGE_EN=%d FET_EN=%d",
                 mfgBefore.raw, mfgBefore.gaugeEn, mfgBefore.fetEn);

        if (!mfgBefore.gaugeEn) {
            ESP_LOGI(TAG, "GAUGE_EN is 0 -- toggling Gauging (0x0021)...");
            esp_err_t e = gauge.toggleGaugingEnable();
            ESP_LOGI(TAG, "toggleGaugingEnable() returned: %s", esp_err_to_name(e));
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            ESP_LOGI(TAG, "GAUGE_EN already 1 -- not toggling.");
        }

        if (!mfgBefore.fetEn) {
            ESP_LOGI(TAG, "FET_EN is 0 -- toggling FET Control (0x0022)...");
            esp_err_t e = gauge.toggleFetEnable();
            ESP_LOGI(TAG, "toggleFetEnable() returned: %s", esp_err_to_name(e));
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            ESP_LOGI(TAG, "FET_EN already 1 -- not toggling.");
        }

        ManufacturingStatus mfgAfter = {};
        esp_err_t mfgAfterErr = gauge.getManufacturingStatus(mfgAfter);
        if (mfgAfterErr == ESP_OK) {
            ESP_LOGI(TAG, "ManufacturingStatus after: raw=0x%04X GAUGE_EN=%d FET_EN=%d",
                     mfgAfter.raw, mfgAfter.gaugeEn, mfgAfter.fetEn);
        } else {
            ESP_LOGE(TAG, "getManufacturingStatus() (verify) failed: %s", esp_err_to_name(mfgAfterErr));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // give the gauge time to re-evaluate and act on it

#if RUN_FET_TOGGLE_TEST
    ESP_LOGI(TAG, "=== FET toggle test starting ===");

    ESP_LOGI(TAG, "Step 1: toggling FET_EN off (manual control)...");
    esp_err_t t1 = gauge.toggleFetEnable();
    ESP_LOGI(TAG, "toggleFetEnable() returned: %s", esp_err_to_name(t1));
    vTaskDelay(pdMS_TO_TICKS(500));

    ManufacturingStatus mfgCheck = {};
    if (gauge.getManufacturingStatus(mfgCheck) == ESP_OK) {
        ESP_LOGI(TAG, "FET_EN now: %d (expect 0)", mfgCheck.fetEn);
    }

    OperationStatus opBefore = {};
    gauge.getOperationStatus(opBefore);
    ESP_LOGI(TAG, "Before CHG toggle: CHG FET=%d, DSG FET=%d", opBefore.chg, opBefore.dsg);

    ESP_LOGI(TAG, "Step 2: toggling CHG FET...");
    gauge.toggleChargeFet();
    vTaskDelay(pdMS_TO_TICKS(500));
    OperationStatus opAfterChg = {};
    gauge.getOperationStatus(opAfterChg);
    ESP_LOGI(TAG, "After CHG toggle: CHG FET=%d, DSG FET=%d", opAfterChg.chg, opAfterChg.dsg);

    ESP_LOGI(TAG, "Toggling CHG FET back...");
    gauge.toggleChargeFet();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Step 3: toggling DSG FET...");
    gauge.toggleDischargeFet();
    vTaskDelay(pdMS_TO_TICKS(500));
    OperationStatus opAfterDsg = {};
    gauge.getOperationStatus(opAfterDsg);
    ESP_LOGI(TAG, "After DSG toggle: CHG FET=%d, DSG FET=%d", opAfterDsg.chg, opAfterDsg.dsg);

    ESP_LOGI(TAG, "Toggling DSG FET back...");
    gauge.toggleDischargeFet();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Step 4: restoring FET_EN to 1 (firmware control)...");
    esp_err_t t4 = gauge.toggleFetEnable();
    ESP_LOGI(TAG, "toggleFetEnable() returned: %s", esp_err_to_name(t4));
    vTaskDelay(pdMS_TO_TICKS(500));

    ManufacturingStatus mfgFinal = {};
    if (gauge.getManufacturingStatus(mfgFinal) == ESP_OK) {
        ESP_LOGI(TAG, "FET_EN now: %d (expect 1, back to firmware control)", mfgFinal.fetEn);
    }
    ESP_LOGI(TAG, "=== FET toggle test complete ===");
#endif

#if RUN_GAUGE_CYCLE_TEST
    ESP_LOGI(TAG, "=== GAUGE_EN cycle test starting ===");

    GaugingStatus3 gs3Before = {};
    if (gauge.getGaugingStatus3(gs3Before) == ESP_OK) {
        ESP_LOGI(TAG, "QMax Time BEFORE: %d (hr/16), QMax PassedQ: %d mAh",
                 gs3Before.qmaxTime, gs3Before.qmaxPassedQ_mAh);
    }

    ManufacturingStatus mfgStart = {};
    if (gauge.getManufacturingStatus(mfgStart) == ESP_OK && mfgStart.gaugeEn) {
        ESP_LOGI(TAG, "GAUGE_EN currently 1, cycling off then on...");

        gauge.toggleGaugingEnable(); // -> 0
        vTaskDelay(pdMS_TO_TICKS(500));
        ManufacturingStatus mfgMid = {};
        if (gauge.getManufacturingStatus(mfgMid) == ESP_OK) {
            ESP_LOGI(TAG, "GAUGE_EN after first toggle: %d (expect 0)", mfgMid.gaugeEn);
        }

        gauge.toggleGaugingEnable(); // -> 1
        vTaskDelay(pdMS_TO_TICKS(500));
        ManufacturingStatus mfgEnd = {};
        if (gauge.getManufacturingStatus(mfgEnd) == ESP_OK) {
            ESP_LOGI(TAG, "GAUGE_EN after second toggle: %d (expect 1, back to normal)", mfgEnd.gaugeEn);
        }
    } else {
        ESP_LOGW(TAG, "GAUGE_EN was not 1 before starting -- skipping cycle to avoid leaving it wrong.");
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); 

    GaugingStatus3 gs3After = {};
    if (gauge.getGaugingStatus3(gs3After) == ESP_OK) {
        ESP_LOGI(TAG, "QMax Time AFTER: %d (hr/16), QMax PassedQ: %d mAh",
                 gs3After.qmaxTime, gs3After.qmaxPassedQ_mAh);
    }
    ESP_LOGI(TAG, "=== GAUGE_EN cycle test complete ===");
#endif

#if RUN_MANUAL_MODE_BOTH_FETS_ON
    ESP_LOGI(TAG, "=== Entering manual mode, forcing both FETs ON ===");

    ManufacturingStatus mfgStart2 = {};
    if (gauge.getManufacturingStatus(mfgStart2) == ESP_OK) {
        ESP_LOGI(TAG, "FET_EN before: %d", mfgStart2.fetEn);
        if (mfgStart2.fetEn) {
            gauge.toggleFetEnable();
            vTaskDelay(pdMS_TO_TICKS(300));
            ManufacturingStatus mfgCheck2 = {};
            if (gauge.getManufacturingStatus(mfgCheck2) == ESP_OK) {
                ESP_LOGI(TAG, "FET_EN after toggle: %d (expect 0)", mfgCheck2.fetEn);
            }
        } else {
            ESP_LOGI(TAG, "FET_EN already 0 (manual mode already active).");
        }
    }

    OperationStatus opStart2 = {};
    if (gauge.getOperationStatus(opStart2) == ESP_OK) {
        ESP_LOGI(TAG, "Before: CHG FET=%d, DSG FET=%d", opStart2.chg, opStart2.dsg);

        if (!opStart2.chg) {
            ESP_LOGI(TAG, "Turning CHG FET ON...");
            gauge.toggleChargeFet();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        if (!opStart2.dsg) {
            ESP_LOGI(TAG, "Turning DSG FET ON...");
            gauge.toggleDischargeFet();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

    OperationStatus opFinal2 = {};
    if (gauge.getOperationStatus(opFinal2) == ESP_OK) {
        ESP_LOGI(TAG, "Final: CHG FET=%d, DSG FET=%d (expect both 1)", opFinal2.chg, opFinal2.dsg);
    }
    ESP_LOGI(TAG, "=== Ready -- set EL15 to 1A CC discharge now, watch SafetyStatus/CHG FET/DSG FET below ===");
#endif

#if RUN_READ_NR_AND_COUNTER_DEC_DELAY
    ESP_LOGI(TAG, "=== Reading DA Configuration[NR] and Counter Dec Delay values ===");

    bool nr = false;
    uint16_t daConfigRaw = 0;
    esp_err_t nrErr = gauge.getDaConfigurationNR(nr, daConfigRaw);
    if (nrErr == ESP_OK) {
        ESP_LOGI(TAG, "DA Configuration raw: 0x%04X -- NR=%d (%s)", daConfigRaw, nr,
                 nr ? "NON-REMOVABLE, clears via Reset time" : "REMOVABLE (default), clears via PRES transition");
    } else {
        ESP_LOGE(TAG, "getDaConfigurationNR() failed: %s", esp_err_to_name(nrErr));
    }

    uint8_t ocdDelay = 0, aoldDelay = 0, asccDelay = 0, ascdDelay = 0;
    if (gauge.getCounterDecDelayOCD(ocdDelay) == ESP_OK) {
        ESP_LOGI(TAG, "OCD Counter Dec Delay: %d s", ocdDelay);
    }
    if (gauge.getCounterDecDelayAOLD(aoldDelay) == ESP_OK) {
        ESP_LOGI(TAG, "AOLD Counter Dec Delay: %d s", aoldDelay);
    }
    if (gauge.getCounterDecDelayASCC(asccDelay) == ESP_OK) {
        ESP_LOGI(TAG, "ASCC Counter Dec Delay: %d s", asccDelay);
    }
    if (gauge.getCounterDecDelayASCD(ascdDelay) == ESP_OK) {
        ESP_LOGI(TAG, "ASCD Counter Dec Delay: %d s", ascdDelay);
    }
    ESP_LOGI(TAG, "NOTE: your observed faults (OC, OCC/CHGC) don't use this counter/latch "
             "mechanism at all -- they use a simple threshold-based Recovery instead.");
    ESP_LOGI(TAG, "=== Done reading NR / Counter Dec Delay ===");
#endif

#if RUN_READ_CURRENT_LIMITS
    ESP_LOGI(TAG, "=== Reading ACTUAL configured current limits from this device ===");
    ESP_LOGI(TAG, "(datasheet Table 7-1 is an EXAMPLE config, not this pack's settings)");

    OvercurrentConfig occ = {};
    esp_err_t occErr = gauge.getOvercurrentConfig(occ);
    if (occErr == ESP_OK) {
        ESP_LOGI(TAG, "OCC1 (charge trip):    %d mA after %d s  [TI default  6000]",
                 occ.occ1Threshold_mA, occ.occ1Delay_s);
        ESP_LOGI(TAG, "OCC2 (charge trip):    %d mA after %d s  [TI default  8000]",
                 occ.occ2Threshold_mA, occ.occ2Delay_s);
        ESP_LOGI(TAG, "OCD1 (discharge trip): %d mA after %d s  [TI default -6000]",
                 occ.ocd1Threshold_mA, occ.ocd1Delay_s);
        ESP_LOGI(TAG, "OCD2 (discharge trip): %d mA after %d s  [TI default -8000]",
                 occ.ocd2Threshold_mA, occ.ocd2Delay_s);
        ESP_LOGI(TAG, "NOTE: these TRIP the FETs (current -> 0). They never clamp current "
                 "to an intermediate value, so they cannot by themselves explain a steady 3 A.");
    } else {
        ESP_LOGE(TAG, "getOvercurrentConfig() failed: %s (device Sealed? check SEC field)",
                 esp_err_to_name(occErr));
    }

    ChargingCurrentConfig ccc = {};
    esp_err_t cccErr = gauge.getChargingCurrentConfig(ccc);
    if (cccErr == ESP_OK) {
        ESP_LOGI(TAG, "Advanced Charging Algorithm table (feeds ChargingCurrent(), "
                 "which a smart charger obeys):");
        ESP_LOGI(TAG, "  Standard Temp Low  -- V range Low/Med/High: %d / %d / %d mA",
                 ccc.stdTempLow_mA[0], ccc.stdTempLow_mA[1], ccc.stdTempLow_mA[2]);
        ESP_LOGI(TAG, "  Standard Temp High -- V range Low/Med/High: %d / %d / %d mA",
                 ccc.stdTempHigh_mA[0], ccc.stdTempHigh_mA[1], ccc.stdTempHigh_mA[2]);
        ESP_LOGI(TAG, "  Rec Temp           -- V range Low/Med/High: %d / %d / %d mA",
                 ccc.recTemp_mA[0], ccc.recTemp_mA[1], ccc.recTemp_mA[2]);
    } else {
        ESP_LOGE(TAG, "getChargingCurrentConfig() failed: %s", esp_err_to_name(cccErr));
    }
    // ---- Chemistry sanity check -------------------------------------------
    char chem[16] = {0};
    if (gauge.getDeviceChemistry(chem, sizeof(chem)) == ESP_OK) {
        ESP_LOGI(TAG, "DeviceChemistry(): \"%s\"  (expect an LFP/LiFePO4 profile)", chem);
    }

    uint16_t chemId = 0;
    if (gauge.getChemId(chemId) == ESP_OK) {
        bool isLfp = ((chemId & 0xFF00) == 0x0400) || ((chemId & 0xF000) == 0x4000);
        ESP_LOGI(TAG, "ChemID: %04X  (%s)", chemId,
                 isLfp ? "LiFePO4 family, correct for this pack"
                       : "NOT an LFP id -- wrong OCV curve and Ra tables for these cells");
        if (chemId == 0x0100) {
            ESP_LOGW(TAG, "  0100 is TI's Li-ion (LiCoO2) shipping default.");
        }
        if (!isLfp) {
            ESP_LOGE(TAG, "!!! State of charge cannot be correct: the OCV curve loaded here !!!");
            ESP_LOGE(TAG, "!!! belongs to a different chemistry. Load an LFP id in BQStudio.   !!!");
        }
    }

    CellVoltageLimits lim = {};
    esp_err_t limErr = gauge.getCellVoltageLimits(lim);
    if (limErr == ESP_OK) {
        ESP_LOGI(TAG, "--- Cell voltage limits programmed in this device ---");
        ESP_LOGI(TAG, "  COV trip (Std Temp Low/High): %d / %d mV   [LFP wants <= ~3800]",
                 lim.covThresholdStdTempLow_mV, lim.covThresholdStdTempHigh_mV);
        ESP_LOGI(TAG, "  COV recovery:                 %d mV        [LFP wants ~3500]",
                 lim.covRecoveryStdTempLow_mV);
        ESP_LOGI(TAG, "  CUV trip / recovery:          %d / %d mV",
                 lim.cuvThreshold_mV, lim.cuvRecovery_mV);
        ESP_LOGI(TAG, "  Charging voltage per temp range  [LFP wants ~3650 on all]:");
        ESP_LOGI(TAG, "    LowTemp %d | StdLow %d | StdHigh %d | HighTemp %d | RecTemp %d mV",
                 lim.chargingVoltageLowTemp_mV, lim.chargingVoltageStdTempLow_mV,
                 lim.chargingVoltageStdTempHigh_mV, lim.chargingVoltageHighTemp_mV,
                 lim.chargingVoltageRecTemp_mV);
        ESP_LOGI(TAG, "    (ChargingVoltage() reports whichever row matches Temp Range,");
        ESP_LOGI(TAG, "     x %d cells. At room temp that is normally the RecTemp row.)", 6);
        ESP_LOGI(TAG, "  Precharge current / start V:  %d mA / %d mV",
                 lim.prechargeCurrent_mA, lim.prechargeStartVoltage_mV);
        ESP_LOGI(TAG, "  Voltage range Low/Med/High:   %d / %d / %d mV",
                 lim.voltageRangeLow_mV, lim.voltageRangeMed_mV, lim.voltageRangeHigh_mV);

        if (lim.covThresholdStdTempLow_mV > 3900 || lim.chargingVoltageStdTempLow_mV > 3700) {
            ESP_LOGE(TAG, "!!! DANGER: these are Li-ion voltages on a LiFePO4 pack. !!!");
            ESP_LOGE(TAG, "!!! Do NOT charge with a charger that obeys ChargingVoltage(). !!!");
        }
    } else {
        ESP_LOGE(TAG, "getCellVoltageLimits() failed: %s", esp_err_to_name(limErr));
    }
    ESP_LOGI(TAG, "=== Done reading current limits ===");
#endif

    while (1) {
        uint16_t packMv = 0;
        int16_t  packMa = 0;
        esp_err_t vErr = gauge.getVoltage(packMv);
        esp_err_t iErr = gauge.getCurrent(packMa);

        uint16_t chgCurrentMa = 0, chgVoltageMv = 0;
        esp_err_t chgIErr = gauge.getChargingCurrent(chgCurrentMa);
        esp_err_t chgVErr = gauge.getChargingVoltage(chgVoltageMv);
        if (chgIErr == ESP_OK && chgVErr == ESP_OK) {
            ESP_LOGI(TAG, "ChargingCurrent() (requested of charger): %u mA, ChargingVoltage(): %u mV",
                     chgCurrentMa, chgVoltageMv);
        }

        DAStatus1 da1 = {};
        esp_err_t da1Err = gauge.readDAStatus1(da1);

        DAStatus3 da3 = {};
        esp_err_t da3Err = gauge.readDAStatus3(da3);

        #define UV_CUTOFF_MV 2500
        #define OV_CUTOFF_MV 3650
        #define OV_RECOVER_MV 3600
        static bool manualModeActive = false;
        static bool dsgLatchedOff = false;

        if (da1Err == ESP_OK && da3Err == ESP_OK) {
            int16_t minCellMv = da1.cellVoltage1_mV;
            if (da1.cellVoltage2_mV < minCellMv) minCellMv = da1.cellVoltage2_mV;
            if (da1.cellVoltage3_mV < minCellMv) minCellMv = da1.cellVoltage3_mV;
            if (da1.cellVoltage4_mV < minCellMv) minCellMv = da1.cellVoltage4_mV;
            if (da3.cellVoltage5_mV < minCellMv) minCellMv = da3.cellVoltage5_mV;
            if (da3.cellVoltage6_mV < minCellMv) minCellMv = da3.cellVoltage6_mV;

            int16_t maxCellMv = da1.cellVoltage1_mV;
            if (da1.cellVoltage2_mV > maxCellMv) maxCellMv = da1.cellVoltage2_mV;
            if (da1.cellVoltage3_mV > maxCellMv) maxCellMv = da1.cellVoltage3_mV;
            if (da1.cellVoltage4_mV > maxCellMv) maxCellMv = da1.cellVoltage4_mV;
            if (da3.cellVoltage5_mV > maxCellMv) maxCellMv = da3.cellVoltage5_mV;
            if (da3.cellVoltage6_mV > maxCellMv) maxCellMv = da3.cellVoltage6_mV;

            bool needUvCutoff = (minCellMv <= UV_CUTOFF_MV);
            bool needOvCutoff = (maxCellMv >= OV_CUTOFF_MV);
            bool okToRecoverOv = (maxCellMv <= OV_RECOVER_MV);

            if (!manualModeActive && (needUvCutoff || needOvCutoff)) {
                ESP_LOGW(TAG, "!!! CUTOFF TRIGGERED: min=%d mV max=%d mV -- entering manual FET mode !!!",
                         minCellMv, maxCellMv);
                ManufacturingStatus mfg = {};
                if (gauge.getManufacturingStatus(mfg) == ESP_OK && mfg.fetEn) {
                    gauge.toggleFetEnable();
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                manualModeActive = true;
            }

            if (manualModeActive) {
                ManufacturingStatus mfgNow = {};
                OperationStatus op = {}; // still read for comparison/telemetry
                if (gauge.getManufacturingStatus(mfgNow) == ESP_OK && gauge.getOperationStatus(op) == ESP_OK) {
                    ESP_LOGI(TAG, "[Check] ManufacturingStatus chgEn=%d dsgEn=%d  vs  OperationStatus CHG=%d DSG=%d",
                             mfgNow.chgEn, mfgNow.dsgEn, op.chg, op.dsg);

                    if (needOvCutoff && mfgNow.chgEn) {
                        ESP_LOGW(TAG, "Opening CHG FET (overvoltage, %d mV)...", maxCellMv);
                        esp_err_t toggleErr = gauge.toggleChargeFet();
                        ESP_LOGW(TAG, "toggleChargeFet() returned: %s", esp_err_to_name(toggleErr));
                        vTaskDelay(pdMS_TO_TICKS(300));
                        ManufacturingStatus mfgVerify = {};
                        if (gauge.getManufacturingStatus(mfgVerify) == ESP_OK) {
                            ESP_LOGW(TAG, "chgEn after toggle attempt: %d (expect 0)", mfgVerify.chgEn);
                            if (mfgVerify.chgEn) {
                                ESP_LOGE(TAG, "chgEn DID NOT CLEAR -- retrying once immediately...");
                                esp_err_t retryErr = gauge.toggleChargeFet();
                                ESP_LOGE(TAG, "Retry toggleChargeFet() returned: %s", esp_err_to_name(retryErr));
                                vTaskDelay(pdMS_TO_TICKS(300));
                                ManufacturingStatus mfgVerify2 = {};
                                if (gauge.getManufacturingStatus(mfgVerify2) == ESP_OK) {
                                    ESP_LOGE(TAG, "chgEn after retry: %d (expect 0)", mfgVerify2.chgEn);
                                }
                            }
                        }
                    } else if (okToRecoverOv && !mfgNow.chgEn && !dsgLatchedOff) {
                        ESP_LOGI(TAG, "Cell voltage recovered to %d mV (<= %d) -- closing CHG FET",
                                 maxCellMv, OV_RECOVER_MV);
                        esp_err_t toggleErr = gauge.toggleChargeFet();
                        ESP_LOGI(TAG, "toggleChargeFet() returned: %s", esp_err_to_name(toggleErr));
                        vTaskDelay(pdMS_TO_TICKS(300));
                    }

                    if (needUvCutoff && mfgNow.dsgEn && !dsgLatchedOff) {
                        ESP_LOGW(TAG, "Opening DSG FET (undervoltage, %d mV) -- LATCHED, "
                                 "requires restart to clear", minCellMv);
                        esp_err_t toggleErr = gauge.toggleDischargeFet();
                        ESP_LOGW(TAG, "toggleDischargeFet() returned: %s", esp_err_to_name(toggleErr));
                        vTaskDelay(pdMS_TO_TICKS(300));
                        dsgLatchedOff = true;
                    }
                    else if (!mfgNow.dsgEn && !dsgLatchedOff && !needUvCutoff) {
                        ESP_LOGI(TAG, "DSG FET was open but not from our own UV latch "
                                 "(min=%d mV, safe) -- closing it", minCellMv);
                        esp_err_t toggleErr = gauge.toggleDischargeFet();
                        ESP_LOGI(TAG, "toggleDischargeFet() returned: %s", esp_err_to_name(toggleErr));
                        vTaskDelay(pdMS_TO_TICKS(300));
                    }
                }

                OperationStatus opFinal = {};
                if (gauge.getOperationStatus(opFinal) == ESP_OK) {
                    ESP_LOGI(TAG, "[Manual mode] CHG FET=%d, DSG FET=%d, DSG latched=%d",
                             opFinal.chg, opFinal.dsg, dsgLatchedOff);
                }
            }
        }

        uint16_t tempDeciK = 0;
        esp_err_t tempErr = gauge.getTemperature(tempDeciK);
        if (tempErr == ESP_OK) {
            float tempC = (tempDeciK / 10.0f) - 273.15f;
            ESP_LOGI(TAG, "Temperature: %.1f C (raw: %u, 0.1K units)", tempC, tempDeciK);
        } else {
            ESP_LOGE(TAG, "getTemperature() failed: %s", esp_err_to_name(tempErr));
        }

        if (da1Err == ESP_OK) {
            char hex1[3 * 32 + 1] = {0};
            for (int i = 0; i < da1.rawLen && i < 32; i++) {
                sprintf(hex1 + i * 3, "%02X ", da1.raw[i]);
            }
            ESP_LOGI(TAG, "DAStatus1 raw (%d bytes): %s", da1.rawLen, hex1);
        } else {
            ESP_LOGE(TAG, "readDAStatus1() failed: %s", esp_err_to_name(da1Err));
        }

        if (da3Err == ESP_OK) {
            char hex3[3 * 18 + 1] = {0};
            for (int i = 0; i < da3.rawLen && i < 18; i++) {
                sprintf(hex3 + i * 3, "%02X ", da3.raw[i]);
            }
            ESP_LOGI(TAG, "DAStatus3 raw (%d bytes): %s", da3.rawLen, hex3);
        } else {
            ESP_LOGE(TAG, "readDAStatus3() failed: %s", esp_err_to_name(da3Err));
        }

        if (da1Err == ESP_OK) {
            ESP_LOGI(TAG, "[DAStatus1] Cell1: %d mV, Cell2: %d mV, Cell3: %d mV, Cell4: %d mV",
                     da1.cellVoltage1_mV, da1.cellVoltage2_mV, da1.cellVoltage3_mV, da1.cellVoltage4_mV);
        }
        if (da3Err == ESP_OK) {
            ESP_LOGI(TAG, "[DAStatus3] Cell5: %d mV, Cell6: %d mV, Cell7: %d mV",
                     da3.cellVoltage5_mV, da3.cellVoltage6_mV, da3.cellVoltage7_mV);
        }
        if (vErr == ESP_OK) {
            ESP_LOGI(TAG, "Pack Voltage (Voltage()): %u mV", packMv);
        } else {
            ESP_LOGE(TAG, "getVoltage() failed: %s", esp_err_to_name(vErr));
        }
        if (iErr == ESP_OK) {
            ESP_LOGI(TAG, "Pack Current: %d mA", packMa);
        } else {
            ESP_LOGE(TAG, "getCurrent() failed: %s", esp_err_to_name(iErr));
        }
        if (da1Err == ESP_OK) {
            ESP_LOGI(TAG, "[DAStatus1] CellCurrent1: %d mA, CellCurrent2: %d mA, CellCurrent3: %d mA, CellCurrent4: %d mA",
                     da1.cellCurrent1_mA, da1.cellCurrent2_mA, da1.cellCurrent3_mA, da1.cellCurrent4_mA);
            ESP_LOGI(TAG, "[DAStatus1] CellPower1: %d cW, CellPower2: %d cW, CellPower3: %d cW, CellPower4: %d cW",
                     da1.cellPower1, da1.cellPower2, da1.cellPower3, da1.cellPower4);
            ESP_LOGI(TAG, "[DAStatus1] Power(V*I): %d cW, AveragePower: %d cW",
                     da1.allCellPower, da1.averagePower);
        }
        if (da3Err == ESP_OK) {
            ESP_LOGI(TAG, "[DAStatus3] CellCurrent5: %d mA, CellCurrent6: %d mA, CellCurrent7: %d mA",
                     da3.cellCurrent5_mA, da3.cellCurrent6_mA, da3.cellCurrent7_mA);
            ESP_LOGI(TAG, "[DAStatus3] CellPower5: %d cW, CellPower6: %d cW, CellPower7: %d cW",
                     da3.cellPower5, da3.cellPower6, da3.cellPower7);
        }

        uint16_t battStatus = 0;
        esp_err_t bsErr = gauge.getBatteryStatus(battStatus);
        if (bsErr == ESP_OK) {
            ESP_LOGI(TAG, "BatteryStatus raw: 0x%04X", battStatus);
            ESP_LOGI(TAG, "  OverChargedAlarm:      %d", (battStatus >> 15) & 1);
            ESP_LOGI(TAG, "  TerminateChargeAlarm:  %d", (battStatus >> 14) & 1);
            ESP_LOGI(TAG, "  OverTempAlarm:         %d", (battStatus >> 12) & 1);
            ESP_LOGI(TAG, "  TerminateDischargeAlarm: %d", (battStatus >> 11) & 1);
            ESP_LOGI(TAG, "  RemainingCapacityAlarm: %d", (battStatus >> 9) & 1);
            ESP_LOGI(TAG, "  RemainingTimeAlarm:    %d", (battStatus >> 8) & 1);
            ESP_LOGI(TAG, "  Initialized (INIT):    %d", (battStatus >> 7) & 1);
            ESP_LOGI(TAG, "  Discharging (DSG):     %d  (0 = gauge thinks it is CHARGING)",
                     (battStatus >> 6) & 1);
            ESP_LOGI(TAG, "  FullyCharged (FC):     %d", (battStatus >> 5) & 1);
            ESP_LOGI(TAG, "  FullyDischarged (FD):  %d", (battStatus >> 4) & 1);
            ESP_LOGI(TAG, "  ErrorCode (bits 0-3):  %d", battStatus & 0x0F);
        } else {
            ESP_LOGE(TAG, "getBatteryStatus() failed: %s", esp_err_to_name(bsErr));
        }

        OperationStatus opStatus = {};
        esp_err_t opErr = gauge.getOperationStatus(opStatus);
        if (opErr == ESP_OK) {
            ESP_LOGI(TAG, "OperationStatus raw: 0x%08lX", (unsigned long)opStatus.raw);
            ESP_LOGI(TAG, "  Security mode: %s (SEC=%d)", operationStatusSecName(opStatus.sec), opStatus.sec);
            ESP_LOGI(TAG, "  ChargingDisabled(XCHG): %d, DischargingDisabled(XDSG): %d",
                     opStatus.xchg, opStatus.xdsg);
            ESP_LOGI(TAG, "  SafetyStatusActive(SS): %d, PermanentFailure(PF): %d, BTP_INT: %d",
                     opStatus.ss, opStatus.pf, opStatus.btpInt);
            ESP_LOGI(TAG, "  CHG FET: %d, DSG FET: %d, PCHG FET: %d, PDSG FET: %d, FUSE: %d",
                     opStatus.chg, opStatus.dsg, opStatus.pchg, opStatus.pdsg, opStatus.fuse);
            ESP_LOGI(TAG, "  Present(PRES): %d, Initialized(INIT): %d, Sleep: %d",
                     opStatus.pres, opStatus.init, opStatus.sleep);
            ESP_LOGI(TAG, "  CellBalancing(CB): %d (1=at least one bypass FET is currently active)",
                     opStatus.cb);
        } else {
            ESP_LOGE(TAG, "getOperationStatus() failed: %s", esp_err_to_name(opErr));
        }

        SafetyStatus safety = {};
        esp_err_t safetyErr = gauge.getSafetyStatus(safety);
        if (safetyErr == ESP_OK) {
            ESP_LOGI(TAG, "SafetyStatus raw: 0x%08lX", (unsigned long)safety.raw);
            if (safety.raw == 0) {
                ESP_LOGI(TAG, "  No safety faults active.");
            } else {
                if (safety.ocdl)  ESP_LOGW(TAG, "  ACTIVE: OCDL  (Overcurrent in discharge, latched)");
                if (safety.covl)  ESP_LOGW(TAG, "  ACTIVE: COVL  (Cell overvoltage, latched)");
                if (safety.utd)   ESP_LOGW(TAG, "  ACTIVE: UTD   (Undertemperature during discharge)");
                if (safety.utc)   ESP_LOGW(TAG, "  ACTIVE: UTC   (Undertemperature during charge)");
                if (safety.pchgc) ESP_LOGW(TAG, "  ACTIVE: PCHGC (Over-precharge current)");
                if (safety.chgv)  ESP_LOGW(TAG, "  ACTIVE: CHGV  (Overcharging voltage)");
                if (safety.chgc)  ESP_LOGW(TAG, "  ACTIVE: CHGC  (Overcharging current)");
                if (safety.oc)    ESP_LOGW(TAG, "  ACTIVE: OC    (Overcharge)");
                if (safety.cto)   ESP_LOGW(TAG, "  ACTIVE: CTO   (Charge timeout)");
                if (safety.pto)   ESP_LOGW(TAG, "  ACTIVE: PTO   (Precharge timeout)");
                if (safety.otf)   ESP_LOGW(TAG, "  ACTIVE: OTF   (Overtemperature FET)");
                if (safety.cuvc)  ESP_LOGW(TAG, "  ACTIVE: CUVC  (Cell undervoltage, compensated)");
                if (safety.otd)   ESP_LOGW(TAG, "  ACTIVE: OTD   (Overtemperature during discharge)");
                if (safety.otc)   ESP_LOGW(TAG, "  ACTIVE: OTC   (Overtemperature during charge)");
                if (safety.ascdl) ESP_LOGW(TAG, "  ACTIVE: ASCDL (Short-circuit during discharge, latched)");
                if (safety.ascd)  ESP_LOGW(TAG, "  ACTIVE: ASCD  (Short-circuit during discharge)");
                if (safety.asccl) ESP_LOGW(TAG, "  ACTIVE: ASCCL (Short-circuit during charge, latched)");
                if (safety.ascc)  ESP_LOGW(TAG, "  ACTIVE: ASCC  (Short-circuit during charge)");
                if (safety.aoldl) ESP_LOGW(TAG, "  ACTIVE: AOLDL (Overload during discharge, latched)");
                if (safety.aold)  ESP_LOGW(TAG, "  ACTIVE: AOLD  (Overload during discharge)");
                if (safety.ocd2)  ESP_LOGW(TAG, "  ACTIVE: OCD2  (Overcurrent during discharge 2)");
                if (safety.ocd1)  ESP_LOGW(TAG, "  ACTIVE: OCD1  (Overcurrent during discharge 1)");
                if (safety.occ2)  ESP_LOGW(TAG, "  ACTIVE: OCC2  (Overcurrent during charge 2)");
                if (safety.occ1)  ESP_LOGW(TAG, "  ACTIVE: OCC1  (Overcurrent during charge 1)");
                if (safety.cov)   ESP_LOGW(TAG, "  ACTIVE: COV   (Cell overvoltage)");
                if (safety.cuv)   ESP_LOGW(TAG, "  ACTIVE: CUV   (Cell undervoltage)");
            }
        } else {
            ESP_LOGE(TAG, "getSafetyStatus() failed: %s", esp_err_to_name(safetyErr));
        }

        ManufacturingStatus mfgStatus = {};
        esp_err_t mfgErr = gauge.getManufacturingStatus(mfgStatus);
        if (mfgErr == ESP_OK) {
            ESP_LOGI(TAG, "ManufacturingStatus raw: 0x%04X", mfgStatus.raw);
            ESP_LOGI(TAG, "  FET_EN: %d (1=firmware controls FETs, 0=it does not)", mfgStatus.fetEn);
            ESP_LOGI(TAG, "  GAUGE_EN: %d (1=gas gauging active, 0=disabled)", mfgStatus.gaugeEn);
            ESP_LOGI(TAG, "  PF_EN: %d, LF_EN: %d, BBR_EN: %d", mfgStatus.pfEn, mfgStatus.lfEn, mfgStatus.bbrEn);
        } else {
            ESP_LOGE(TAG, "getManufacturingStatus() failed: %s", esp_err_to_name(mfgErr));
        }

        GaugingStatus3 gs3 = {};
        esp_err_t gs3Err = gauge.getGaugingStatus3(gs3);
        if (gs3Err == ESP_OK) {
            ESP_LOGI(TAG, "QMax: Cell1=%d mAh, Cell2=%d mAh, Cell3=%d mAh, Cell4=%d mAh",
                     gs3.qmax1_mAh, gs3.qmax2_mAh, gs3.qmax3_mAh, gs3.qmax4_mAh);
            ESP_LOGI(TAG, "QMax DOD0: Cell1=%d, Cell2=%d, Cell3=%d, Cell4=%d",
                     gs3.qmaxDod0_1, gs3.qmaxDod0_2, gs3.qmaxDod0_3, gs3.qmaxDod0_4);
            ESP_LOGI(TAG, "QMax PassedQ: %d mAh, QMax Time: %d (hr/16)",
                     gs3.qmaxPassedQ_mAh, gs3.qmaxTime);
        } else {
            ESP_LOGE(TAG, "getGaugingStatus3() failed: %s", esp_err_to_name(gs3Err));
        }

        #define RELAX_CURRENT_THRESHOLD_MA 50
        GaugingStatus gs = {};
        esp_err_t gsErr = gauge.getGaugingStatus(gs);
        if (gsErr == ESP_OK) {
            const char *stateGuess;
            if (!gs.dsg) {
                stateGuess = "CHARGE";
            } else if (vErr == ESP_OK && abs(packMa) <= RELAX_CURRENT_THRESHOLD_MA) {
                stateGuess = gs.rest ? "RELAX (OCV already taken)" : "RELAX (OCV not taken yet)";
            } else {
                stateGuess = "DISCHARGE";
            }
            ESP_LOGI(TAG, "GaugingStatus raw: 0x%06lX -- state: %s", (unsigned long)gs.raw, stateGuess);
            ESP_LOGI(TAG, "  DSG(chg not detected)=%d REST(OCV taken)=%d QEN=%d VOK=%d",
                     gs.dsg, gs.rest, gs.qen, gs.vok);
            ESP_LOGI(TAG, "  TC=%d TD=%d FC=%d FD=%d EDV=%d", gs.tc, gs.td, gs.fc, gs.fd, gs.edv);
        } else {
            ESP_LOGE(TAG, "getGaugingStatus() failed: %s", esp_err_to_name(gsErr));
        }

        GaugingStatus1 gs1 = {};
        esp_err_t gs1Err = gauge.getGaugingStatus1(gs1);
        if (gs1Err == ESP_OK) {
            ESP_LOGI(TAG, "Cell Resistance (CompRes): Cell1=%d, Cell2=%d, Cell3=%d, Cell4=%d (2-10 ohm range)",
                     gs1.compRes1, gs1.compRes2, gs1.compRes3, gs1.compRes4);
        } else {
            ESP_LOGE(TAG, "getGaugingStatus1() failed: %s", esp_err_to_name(gs1Err));
        }

        GaugingStatus4 gs4 = {};
        esp_err_t gs4Err = gauge.getGaugingStatus4(gs4);
        if (gs4Err == ESP_OK) {
            ESP_LOGI(TAG, "QMax: Cell5=%d mAh, Cell6=%d mAh", gs4.qmax5_mAh, gs4.qmax6_mAh);
            ESP_LOGI(TAG, "QMax DOD0: Cell5=%d, Cell6=%d", gs4.qmaxDod0_5, gs4.qmaxDod0_6);
            ESP_LOGI(TAG, "Cell Resistance (CompRes): Cell5=%d, Cell6=%d (2-10 ohm range)",
                     gs4.compRes5, gs4.compRes6);
        } else {
            ESP_LOGE(TAG, "getGaugingStatus4() failed: %s", esp_err_to_name(gs4Err));
        }

        uint16_t designCapMah = 0;
        esp_err_t designCapErr = gauge.getDesignCapacity(designCapMah);
        if (designCapErr == ESP_OK) {
            ESP_LOGI(TAG, "DesignCapacity: %u mAh (manufacturer rated: 15000 mAh)", designCapMah);
        } else {
            ESP_LOGE(TAG, "getDesignCapacity() failed: %s", esp_err_to_name(designCapErr));
        }


        uint16_t remainCapMah = 0;
        esp_err_t remainCapErr = gauge.getRemainingCapacity(remainCapMah);
        uint16_t fullChargeCapMah = 0;
        esp_err_t fullChargeCapErr = gauge.getFullChargeCapacity(fullChargeCapMah);
        if (remainCapErr == ESP_OK) {
            ESP_LOGI(TAG, "RemainingCapacity (live, right now): %u mAh", remainCapMah);
        } else {
            ESP_LOGE(TAG, "getRemainingCapacity() failed: %s", esp_err_to_name(remainCapErr));
        }
        if (fullChargeCapErr == ESP_OK) {
            ESP_LOGI(TAG, "FullChargeCapacity (learned, updates over time): %u mAh", fullChargeCapMah);
        } else {
            ESP_LOGE(TAG, "getFullChargeCapacity() failed: %s", esp_err_to_name(fullChargeCapErr));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}