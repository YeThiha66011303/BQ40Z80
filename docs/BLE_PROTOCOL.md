# BLE protocol

The ESP32 advertises as `HeroBMS-XXXX` (last two bytes of the BT MAC) with one
primary service. All multi-byte fields are **little-endian**, packed with no
padding. `i16[7]` means seven consecutive `i16` values.

| Role    | UUID                                   | Properties     |
|---------|----------------------------------------|----------------|
| Service | `9a1b0001-5f3c-4e8a-b7d2-1c6e4f8a2b30` |                |
| LIVE    | `9a1b0002-5f3c-4e8a-b7d2-1c6e4f8a2b30` | read, notify   |
| INFO    | `9a1b0003-5f3c-4e8a-b7d2-1c6e4f8a2b30` | read           |
| DETAIL  | `9a1b0004-5f3c-4e8a-b7d2-1c6e4f8a2b30` | read (long)    |
| CMD     | `9a1b0005-5f3c-4e8a-b7d2-1c6e4f8a2b30` | write, encrypted + authenticated |
| SETTINGS | `9a1b0006-5f3c-4e8a-b7d2-1c6e4f8a2b30` | read (long) |

Every packet starts with a version byte. A client that sees a version it does
not know must not guess at the layout.

## LIVE (version 1, 152 bytes)

Notified once a second while subscribed. Needs an ATT MTU of at least 155; with
a smaller MTU the firmware skips the notification and the client should read
the characteristic instead.

| Type    | Field | Notes |
|---------|-------|-------|
| u8      | version | 1 |
| u8      | flags | bit0 gauge online, bit1 static info valid, bit2 reset is safe (FET_EN set in Mfg Status Init) |
| u8      | cellCount | cells wired on this pack |
| u8      | RSOC % | |
| u8      | ASOC % | |
| u8      | MaxError % | slow tier |
| u32     | uptime s | ESP32 |
| u32     | poll count | |
| u32     | error count | cumulative failed SMBus reads |
| u16     | Voltage mV | |
| i16     | Current mA | + charge, − discharge |
| i16     | AverageCurrent mA | |
| u16     | Temperature 0.1 K | |
| u16     | RemainingCapacity mAh | |
| u16     | FullChargeCapacity mAh | |
| u16     | ChargingCurrent mA | requested of the charger |
| u16     | ChargingVoltage mV | |
| u16     | BatteryStatus | bits |
| i16     | BAT mV | DAStatus1 |
| i16     | PACK mV | DAStatus1 |
| i16     | AllCellPower cW | |
| i16     | AveragePower cW | |
| i16[7]  | cell voltage mV | 1-4 DAStatus1, 5-7 DAStatus3 |
| i16[7]  | cell current mA | |
| i16[7]  | cell power cW | |
| i16[7]  | temps 0.1 K | Int, TS1, TS2, TS3, TS4, Cell, FET; 0 = not fitted |
| u32     | OperationStatus | |
| u32     | SafetyStatus | |
| u32     | SafetyAlert | |
| u32     | ChargingStatus | |
| u32     | GaugingStatus | IT status in bits 23-8 |
| u16     | ManufacturingStatus | |
| u32     | PFStatus | slow tier |
| u32     | PFAlert | slow tier |
| u16     | CycleCount | slow tier |
| u16     | StateOfHealth % | slow tier |
| u16     | RunTimeToEmpty min | slow tier |
| u16     | AverageTimeToEmpty min | slow tier |
| u16     | AverageTimeToFull min | slow tier |
| u32     | resets issued since boot | |
| u32     | uptime of last reset s | |
| i32     | esp_err_t of last reset | |

## INFO (version 1, variable)

| Type | Field |
|------|-------|
| u8   | version (1) |
| u8   | flags: bit0 valid, bit1 Enabled Protections C valid, bit2 Mfg Status Init valid |
| u8   | cellCount |
| u8   | Enabled Protections C |
| u16  | ChemID |
| u16  | DeviceType |
| u16  | SerialNumber |
| u16  | DesignCapacity mAh |
| u16  | DesignVoltage mV |
| u16  | ManufactureDate `(year-1980)<<9 \| month<<5 \| day` |
| u16  | Mfg Status Init (data flash 0x4800) |
| str  | ManufacturerName |
| str  | DeviceName |
| str  | DeviceChemistry |

`str` is a `u8` length followed by that many bytes, no terminator.

## DETAIL (version 1, 296 bytes)

Slow tier, refreshed on the gauge every 30 s. Longer than one MTU, so the
client does a long read; the firmware serves all pieces of one long read from
the same snapshot.

| Type | Field |
|------|-------|
| u8   | version (1) |
| u32  | uptime of last slow poll s |
| u16  | BatteryMode |
| u16  | RemainingCapacityAlarm mAh |
| u16  | RemainingTimeAlarm min |
| i16  | AtRate mA |
| u16  | AtRateTimeToFull min |
| u16  | AtRateTimeToEmpty min |
| u16  | AtRateOK |
| i16  | MaxTurboPower cW |
| i16  | SustainedTurboPower cW |
| i16  | MaxTurboCurrent mA |
| i16  | SustainedTurboCurrent mA |
| u16  | TurboPackResistance mΩ |
| u16  | TurboSysResistance mΩ |
| u16  | TurboEDV mV |
| u16  | NoLoadRemCap mAh |
| u16  | SOH FCC mAh |
| u16  | SOH FCC cWh |
| i16[7] × 8 | QMax, QMax DOD0, Raw DOD0, RaScale, CompRes, DOD0, DODEOC, Cell Grid (slot 7 is 0) |
| u8   | Pack Grid |
| u8   | LStatus |
| u32  | State Time s |
| i16 × 15 | DOD0 Passed Q, DOD0 Passed E, DOD0 Time, QMax Passed Q, QMax Time, Temp K, Temp A, True Rem Q, True Rem E, Initial Q, Initial E, True FCC Q, True FCC E, T_sim, T_ambient |
| i16 × 4 | Filtered: Rem Q mAh, Rem E mWh, FCC Q mAh, FCC E mWh |
| i16[7] | balance time s (really u16: mask with 0xFFFF, or 65007 s reads as −529) |
| i16[7] | balance DOD |
| i16  | total DOD charge |
| i16[7] | lifetime cell max mV |
| i16[7] | lifetime cell min mV |
| i16  | lifetime max Δ cell mV |
| i16 × 4 | lifetime max charge mA, max discharge mA, max avg dsg mA, max avg dsg power cW |
| i8 × 6 | lifetime max/min cell °C, max Δ cell °C, max/min internal °C, max FET °C |
| u8 × 4 | shutdowns, partial resets, full resets, WDT resets |
| u8[7] | lifetime balance time per cell h |
| u16 × 8 | total FW runtime, time in UT, LT, STL, RT, STH, HT, OT |

## CMD

Write-with-response. The characteristic needs an encrypted, authenticated link,
so the first write makes the phone pair with the passkey `BLE_PASSKEY` from
`main/ble_server.h`.

| Payload | Action |
|---------|--------|
| `01` `"CONFIRM"` | DeviceReset, accepted only while Mfg Status Init has FET_EN set |
| `01` `"CONFIRM-UNSAFE"` | DeviceReset, accepted only while it does not |
| `02` `id:u8` `value:i32` | Write one data flash setting (see SETTINGS) |

The token rule is the same as `POST /api/reset`. Application ATT errors:
`0x80` token did not match the current hazard, `0x81` the reset failed on the
SMBus, `0x82` unknown opcode.

Setting writes return `0x83` id not in the list, `0x84` value outside its
limits, `0x85` value would cross its partner (a recovery level past its
threshold, or a charging voltage at or above COV), `0x86` the gauge is SEALED,
`0x87` SMBus busy, `0x88` SMBus error on the write, `0x89` the value read back
did not match. A write equal to the current value succeeds without touching
flash. For bit registers only the bits in the item's mask change; reserved bits
keep their current value.

## SETTINGS (version 1, 2 + 6 × count bytes)

Read on demand. The firmware reads each value from the gauge's data flash, so a
read takes a few hundred ms; the result is cached for 2 s and rebuilt after any
successful write.

| Type | Field |
|------|-------|
| u8   | version, 1 |
| u8   | count |

then `count` entries of:

| Type | Field |
|------|-------|
| u8   | id |
| u8   | 1 = read OK, 0 = read failed |
| i32  | raw value as stored |

Raw units: mV, mA, s, % as in BQStudio; temperatures in 0.1 K (2732 = 0.0 °C);
bit registers as their byte value. Addresses from TRM SLUUBT5C for firmware
4800_0_04. Limits are for LiFePO4 cells and are enforced by the firmware.

| id | Setting | Address | Type | Limits |
|----|---------|---------|------|--------|
| 1 | Balancing Configuration | 0x4B8C | H1 | writable bits CB, CBR, CB_CHG_DOD0EW, CB_RLX_DOD0EW, CBS (mask 0x3D) |
| 2 | Min Start Balance Delta | 0x4B93 | U1 | 1–100 mV |
| 3 | Relax Balance Interval | 0x4B94 | U4 | 10–36000 s |
| 4 | Min RSOC for Balancing | 0x4B98 | U1 | 0–100 % |
| 5 / 6 / 7 | Bal Time/mAh Cell 1 / Cell 2-6 / Cell 7 | 0x4B8D / 0x4B8F / 0x4B91 | U2 | 50–5000 s/mAh |
| 8 / 9 | Start / End RSOC for Bal in Sleep | 0x4B99 / 0x4B9A | U1 | 0–100 %, start above end |
| 60 | Start Time for Bal in Sleep | 0x4B9B | U2 | 0–65520 h |
| 10 / 11 | CUV Threshold / Recovery | 0x4BC2 / 0x4BC5 | I2 | 2000–3000 / 2200–3400 mV, threshold < recovery |
| 12 | CUV Delay | 0x4BC4 | U1 | 1–30 s |
| 13–17 | COV Threshold Low / Std Low / Std High / High / Rec Temp | 0x4BCC–0x4BD4 | I2 | 3500–3800 mV, above its recovery |
| 18 | COV Delay | 0x4BD6 | U1 | 1–30 s |
| 19–23 | COV Recovery, same five ranges | 0x4BD7–0x4BDF | I2 | 3300–3700 mV |
| 30–34 | Charging Voltage, same five ranges | 0x4C9F, 0x4CA7, 0x4CAF, 0x4CB7, 0x4CBF | I2 | 3300–3650 mV, below that range's COV |
| 35 | Charge Term Taper Current | 0x4CF3 | I2 | 50–5000 mA |
| 40 / 41 | OTC Threshold / Recovery | 0x4C05 / 0x4C08 | I2 | 30–60 / 25–55 °C, threshold > recovery |
| 42 / 43 | OTD Threshold / Recovery | 0x4C0A / 0x4C0D | I2 | 40–75 / 30–70 °C |
| 44 / 45 | UTC Threshold / Recovery | 0x4C14 / 0x4C17 | I2 | −10–15 / −5–20 °C, threshold < recovery |
| 46 / 47 | UTD Threshold / Recovery | 0x4C19 / 0x4C1C | I2 | −30–10 / −25–15 °C |
| 50–53 | Enabled Protections A–D | 0x4BBE–0x4BC1 | H1 | masks 0xBF, 0x7B, 0xD7, 0x3F |

IDs are fixed: new settings get new IDs, existing ones are never renumbered.
