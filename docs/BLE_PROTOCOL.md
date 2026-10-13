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
| i16[7] | balance time s |
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

The token rule is the same as `POST /api/reset`. Application ATT errors:
`0x80` token did not match the current hazard, `0x81` the reset failed on the
SMBus, `0x82` unknown opcode.
