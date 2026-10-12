// =============================================================================
// Telemetry -> JSON.
//
// Bit-mapped registers are emitted as raw integers rather than as decoded
// booleans. The browser owns the bit-name tables, which keeps this file small
// and keeps each register's bit layout written down in exactly one place.
// =============================================================================
#include "telemetry.h"

#include "esp_log.h"

#include <cstdarg>
#include <cstdio>

static const char *TAG = "tlm_json";

namespace {

struct Json {
    char  *buf;
    size_t cap;
    size_t len;

    void put(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
    {
        if (len >= cap) return;
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(buf + len, cap - len, fmt, ap);
        va_end(ap);
        if (n < 0) return;
        len = ((size_t)n >= cap - len) ? cap : len + (size_t)n;
    }

    // Device strings come from the pack and are not trusted to be clean JSON.
    void putStr(const char *key, const char *s)
    {
        put("\"%s\":\"", key);
        for (const char *p = s; *p && len < cap; ++p) {
            unsigned char c = (unsigned char)*p;
            if (c == '"' || c == '\\')      put("\\%c", c);
            else if (c >= 0x20 && c < 0x7F) put("%c", c);
            else                            put("\\u%04x", c);
        }
        put("\",");
    }

    void arr16(const char *key, const int16_t *v, int n)
    {
        put("\"%s\":[", key);
        for (int i = 0; i < n; ++i) put("%d%s", v[i], (i + 1 < n) ? "," : "");
        put("],");
    }

    void arr8(const char *key, const uint8_t *v, int n)
    {
        put("\"%s\":[", key);
        for (int i = 0; i < n; ++i) put("%u%s", v[i], (i + 1 < n) ? "," : "");
        put("],");
    }

    // Drops the trailing comma the last member left behind, then closes.
    void close(char c)
    {
        if (len > 0 && len < cap && buf[len - 1] == ',') --len;
        put("%c", c);
    }
};

} // namespace

size_t telemetryToJson(char *buf, size_t cap)
{
    Telemetry  t;
    StaticInfo s;
    telemetryGet(t);
    telemetryGetStatic(s);

    Json j{buf, cap, 0};

    j.put("{\"online\":%s,\"uptime\":%lu,\"polls\":%lu,\"errors\":%lu,"
          "\"lastSlow\":%lu,",
          t.online ? "true" : "false",
          (unsigned long)t.uptime_s, (unsigned long)t.pollCount,
          (unsigned long)t.errorCount, (unsigned long)t.lastSlowPoll_s);

    // ---- static identity -------------------------------------------------
    j.put("\"static\":{\"valid\":%s,", s.valid ? "true" : "false");
    j.putStr("mfg",  s.manufacturer);
    j.putStr("dev",  s.deviceName);
    j.putStr("chem", s.chemistry);
    j.put("\"chemId\":%u,\"devType\":%u,\"serial\":%u,\"designCap\":%u,"
          "\"designV\":%u,\"mfgDate\":%u,\"protC\":%u,\"protCValid\":%s,"
          "\"mfgInit\":%u,\"mfgInitValid\":%s",
          s.chemId, s.deviceType, s.serialNumber, s.designCapacity_mAh,
          s.designVoltage_mV, s.manufactureDate, s.enabledProtectionsC,
          s.enabledProtectionsCValid ? "true" : "false",
          s.mfgStatusInit, s.mfgStatusInitValid ? "true" : "false");
    j.close('}');
    j.put(",");

    // ---- reset state ------------------------------------------------------
    ResetInfo r;
    telemetryGetResetInfo(r);
    j.put("\"reset\":{\"safe\":%s,\"tried\":%s,\"count\":%lu,\"lastAt\":%lu,"
          "\"lastErr\":%d},",
          telemetryResetIsSafe() ? "true" : "false",
          r.everTried ? "true" : "false",
          (unsigned long)r.count, (unsigned long)r.lastAt_s, r.lastErr);

    // ---- SBS word registers ----------------------------------------------
    j.put("\"sbs\":{"
          "\"voltage\":%u,\"current\":%d,\"avgCurrent\":%d,\"temp\":%u,"
          "\"rsoc\":%u,\"asoc\":%u,\"remCap\":%u,\"fcc\":%u,"
          "\"chgCurrent\":%u,\"chgVoltage\":%u,\"maxError\":%u,"
          "\"cycleCount\":%u,\"soh\":%u,\"rtte\":%u,\"atte\":%u,\"attf\":%u,"
          "\"remCapAlarm\":%u,\"remTimeAlarm\":%u,\"atRate\":%d,"
          "\"atRateTTF\":%u,\"atRateTTE\":%u,\"atRateOK\":%u,"
          "\"maxTurboPwr\":%d,\"susTurboPwr\":%d,\"maxTurboCurr\":%d,"
          "\"susTurboCurr\":%d,\"turboPackR\":%u,\"turboSysR\":%u,"
          "\"turboEdv\":%u,\"noLoadRemCap\":%u,\"sohFccQ\":%u,\"sohFccE\":%u},",
          t.voltage_mV, t.current_mA, t.avgCurrent_mA, t.temperature_dK,
          t.rsoc_pct, t.asoc_pct, t.remainingCapacity_mAh,
          t.fullChargeCapacity_mAh, t.chargingCurrent_mA, t.chargingVoltage_mV,
          t.maxError_pct, t.cycleCount, t.stateOfHealth_pct,
          t.runTimeToEmpty_min, t.avgTimeToEmpty_min, t.avgTimeToFull_min,
          t.remCapAlarm_mAh, t.remTimeAlarm_min, t.atRate_mA,
          t.atRateTimeToFull_min, t.atRateTimeToEmpty_min, t.atRateOK,
          t.maxTurboPower_cW, t.susTurboPower_cW, t.maxTurboCurrent_mA,
          t.susTurboCurrent_mA, t.turboPackR_mOhm, t.turboSysR_mOhm,
          t.turboEdv_mV, t.noLoadRemCap_mAh, t.sohFcc_mAh, t.sohFcc_cWh);

    // ---- per-cell, stitched from DAStatus1 (1-4) and DAStatus3 (5-7) -----
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

    j.put("\"cells\":{");
    j.arr16("v", cv, 7);
    j.arr16("i", ci, 7);
    j.arr16("p", cp, 7);
    j.put("\"bat\":%d,\"pack\":%d,\"allCellPower\":%d,\"avgPower\":%d},",
          t.da1.batVoltage_mV, t.da1.packVoltage_mV,
          t.da1.allCellPower, t.da1.averagePower);

    // ---- temperatures, raw 0.1 K -----------------------------------------
    j.put("\"temps\":{\"int\":%d,\"ts1\":%d,\"ts2\":%d,\"ts3\":%d,\"ts4\":%d,"
          "\"cell\":%d,\"fet\":%d},",
          t.da2.intTemp_dK, t.da2.ts1Temp_dK, t.da2.ts2Temp_dK,
          t.da2.ts3Temp_dK, t.da2.ts4Temp_dK, t.da2.cellTemp_dK,
          t.da2.fetTemp_dK);

    // ---- bit-mapped registers, raw ---------------------------------------
    j.put("\"bits\":{"
          "\"batteryMode\":%u,\"batteryStatus\":%u,\"op\":%lu,"
          "\"safetyStatus\":%lu,\"safetyAlert\":%lu,"
          "\"pfStatus\":%lu,\"pfAlert\":%lu,"
          "\"chargingStatus\":%lu,\"gauging\":%lu,\"mfgStatus\":%u,"
          "\"lStatus\":%u},",
          t.batteryMode, t.batteryStatus, (unsigned long)t.op.raw,
          (unsigned long)t.safetyStatus.raw, (unsigned long)t.safetyAlert.raw,
          (unsigned long)t.pfStatus, (unsigned long)t.pfAlert,
          (unsigned long)t.chargingStatus.raw,
          (unsigned long)t.gaugingStatus.raw, t.mfg.raw, t.g2.lStatus);

    // ---- Impedance Track --------------------------------------------------
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

    j.put("\"it\":{");
    j.arr16("qmax",     qmax,    7);
    j.arr16("qmaxDod0", qmaxDod, 7);
    j.arr16("rawDod0",  rawDod,  7);
    j.arr16("raScale",  raScale, 7);
    j.arr16("compRes",  compRes, 7);
    j.arr16("dod0",     dod0,    7);
    j.arr16("dodEoc",   dodEoc,  7);
    j.arr16("cellGrid", grid,    7);
    j.put("\"packGrid\":%u,\"stateTime\":%lu,"
          "\"dod0PassedQ\":%d,\"dod0PassedE\":%d,\"dod0Time\":%d,"
          "\"qmaxPassedQ\":%d,\"qmaxTime\":%d,\"tempK\":%d,\"tempA\":%d,"
          "\"trueRemQ\":%d,\"trueRemE\":%d,\"initialQ\":%d,\"initialE\":%d,"
          "\"trueFccQ\":%d,\"trueFccE\":%d,\"tSim\":%d,\"tAmbient\":%d},",
          t.g2.packGrid, (unsigned long)t.g2.stateTime_s,
          t.g2.dod0PassedQ_mAh, t.g2.dod0PassedE_cWh, t.g2.dod0Time_hr16,
          t.g3.qmaxPassedQ_mAh, t.g3.qmaxTime, t.g3.tempK, t.g3.tempA,
          t.g1.trueRemQ_mAh, t.g1.trueRemE_cWh, t.g1.initialQ_mAh,
          t.g1.initialE_cWh, t.g1.trueFccQ_mAh, t.g1.trueFccE_cWh,
          t.g1.tSim, t.g1.tAmbient);

    // ---- filtered capacity ------------------------------------------------
    j.put("\"filt\":{\"remQ\":%d,\"remE\":%d,\"fccQ\":%d,\"fccE\":%d},",
          t.filt.remainingCapacity_mAh, t.filt.remainingEnergy_mWh,
          t.filt.fullChargeCapacity_mAh, t.filt.fullChargeEnergy_mWh);

    // ---- cell balancing ---------------------------------------------------
    j.put("\"cb\":{");
    j.arr16("time", t.cb.balanceTime_s, 7);
    j.arr16("dod",  t.cb.balanceDod,    7);
    j.put("\"totalDodChg\":%d},", t.cb.totalDodCharge);

    // ---- lifetime records -------------------------------------------------
    j.put("\"lt\":{");
    j.arr16("cellMaxV", t.lt1.cellMaxVoltage_mV, 7);
    j.arr16("cellMinV", t.lt1.cellMinVoltage_mV, 7);
    j.arr8 ("cbTime",   t.lt2.cbTimeCell_h,      7);
    j.put("\"maxDeltaV\":%d,"
          "\"maxChgCurr\":%d,\"maxDsgCurr\":%d,\"maxAvgDsgCurr\":%d,"
          "\"maxAvgDsgPwr\":%d,\"maxTempCell\":%d,\"minTempCell\":%d,"
          "\"maxDeltaTemp\":%d,\"maxTempInt\":%d,\"minTempInt\":%d,"
          "\"maxTempFet\":%d,\"shutdowns\":%u,\"partialResets\":%u,"
          "\"fullResets\":%u,\"wdtResets\":%u,"
          "\"fwRuntime\":%u,\"tUT\":%u,\"tLT\":%u,\"tSTL\":%u,\"tRT\":%u,"
          "\"tSTH\":%u,\"tHT\":%u,\"tOT\":%u}",
          t.lt1.maxDeltaCellVoltage_mV,
          t.lt2.maxChargeCurrent_mA, t.lt2.maxDischargeCurrent_mA,
          t.lt2.maxAvgDsgCurrent_mA, t.lt2.maxAvgDsgPower_cW,
          t.lt2.maxTempCell_C, t.lt2.minTempCell_C, t.lt2.maxDeltaCellTemp_C,
          t.lt2.maxTempIntSensor_C, t.lt2.minTempIntSensor_C,
          t.lt2.maxTempFet_C, t.lt2.numShutdowns, t.lt2.numPartialResets,
          t.lt2.numFullResets, t.lt2.numWdtResets,
          t.lt3.totalFwRuntime, t.lt3.timeInUT, t.lt3.timeInLT,
          t.lt3.timeInSTL, t.lt3.timeInRT, t.lt3.timeInSTH,
          t.lt3.timeInHT, t.lt3.timeInOT);

    j.close('}');

    if (j.len >= cap) {
        ESP_LOGW(TAG, "JSON truncated -- buffer of %u bytes is too small",
                 (unsigned)cap);
        return 0;
    }
    buf[j.len] = '\0';
    return j.len;
}
