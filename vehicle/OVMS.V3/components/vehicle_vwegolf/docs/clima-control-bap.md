# Climate Control via BAP Battery Control Channel — RE Notes

References:
- https://github.com/karlsen-technologies/smartkar-cano-new/tree/2b191b8e9066d494125b3c0338787a347ec8d205/docs/canbus-reverse-engineering — `BAP_PROTOCOL.md` and `BAP_BATTERY_CONTROL.md` (thomasakarlsen; supersedes e-golf-comfort-can for BAP). Pinned to the commit we validated against (2026-07-09).
- MIB2 firmware RE by SCjona (PR #1430 review thread) — message/field names below marked *(MIB2 FW)*
- Local BAP framing notes: `vw-bap-protocol.md` (being aligned to standard terminology)

Terminology (standard BAP; earlier revisions of this doc used ad-hoc terms):

| This doc (old) | Standard BAP | Meaning |
|---|---|---|
| "node" 0x25 | **LSG ID** 0x25 | Logical control unit: Battery Control |
| "port" | **Function ID** | Function within the LSG |
| "command"/"status push" opcode 2 | **OpCode** 0x02 SetGet | Set value, request confirmation |
| "rolling counter" | **Transaction-ID** (array header) | Echoed in FSG response |

## Channel

Battery Control channel, LSG 0x25. OVMS acts as an **ASG** (client); the Battery Control Unit is the **FSG** (server).

| Direction | CAN ID (29-bit) | Description |
|---|---|---|
| ASG → FSG | `0x17332501` | Commands (also carries FSG keepalive polls / 5 s status bursts observed on-car) |
| FSG → ASG | `0x17332510` | Status responses, ACKs, broadcasts |

## Function IDs (LSG 0x25)

Names per smartkar BAP_BATTERY_CONTROL.md; "obs." column = what we observed on-car (Captures 2/7 etc.).

| Func | Name | Dir | On-car observations |
|---|---|---|---|
| 0x01 | GetAllProperties | → FSG | empty SetGet poll seen |
| 0x02 | BAP-Config | → FSG | empty SetGet poll seen |
| 0x10 | PlugState | ← FSG | — |
| 0x11 | ChargeState | ← FSG | — |
| 0x12 | ClimateState | ← FSG | 7-byte status ~1 s; payload[0] is a multi-valued sub-state, **not** a boolean — `0x11` OVMS-triggered, `0x09` schedule-triggered, `0x05` brief pre-check pulses, `0x00` idle. Not fully decoded. |
| 0x13 | (status) | ← FSG | `04 04`=active `04 00`=idle |
| 0x14–0x16 | timer/schedule slots 0–2 | ↔ | 8-byte records; Ack confirms writes |
| 0x18 | ClimateOperationModeInstallation | → FSG | start/stop trigger, 2-byte payload |
| 0x19 | ProfilesArray | ↔ | Battery Control Profiles (see below) |
| 0x1A | PowerProvidersArray | ← FSG | slot Ack / status observed |

**Authoritative run/stop detection: `0x03B5` ClimaRunning bit7** — this is what firmware uses to drive `ms_v_env_hvac` (`vehicle_vwegolf.cpp`), and it tracks real conditioning within <0.2 s.

Two earlier readings are **superseded**:

- Function 0x12 payload[0] `05`→`00` was documented here as the authoritative stop signal. Wrong — payload[0] is multi-valued (see table above); `0x05` shows up only on brief pre-check pulses, while real sessions read `0x11` (OVMS-triggered) or `0x09` (schedule-triggered). Sustained sessions were verified against `0x03B5` transitions across 3 independent captures.
- `0x5EA` remote_mode lags actual stop by 17–30 s (measured: 29.7 s and 16.8 s in two OVMS-triggered sessions) — never use it for stop detection.

## Battery Control Profiles (Function 0x19)

Charging/climate settings live in 4 stored **profiles** (= "Charge Locations" in infotainment): profile 0 = hidden global/immediate profile used by "start now" operations; 1–3 = departure timers. The infotainment field dictionary + value domains for these profiles (decode prior for captured writes) are in `emanager-ui-model.yml`; note the open charge-location-vs-timer index discrepancy flagged there. Commands do not carry one-shot parameters — they **write profile fields**, then trigger via Function 0x18.

Profiles are BAP arrays; the **RecordAddr** in the array header selects the record format:
- RecordAddr 0: full profile, 20+ bytes — includes **temperature** at byte 12, encoding `raw = °C × 10 − 100` (22.0 °C → `0x78`), unit byte, lead/holding times, name…
- RecordAddr 6: compact profile, 4 bytes — `operation, operation2, maxCurrent, targetChargeLevel`. **No temperature field.**

operation flags (byte 0): bit0 charge · bit1 climate · bit2 climateWithoutExternalSupply · bit3 autoDefrost · bits4–7 seat heaters. `0x06` = climate + climate-on-battery.

Full field tables: smartkar `BAP_BATTERY_CONTROL.md`.

## Climate Start/Stop — 3-Frame Sequence

All on `0x17332501`. Step 1 is a SetGet on ProfilesArray (0x19) writing profile 0 in **compact (RecordAddr 6)** format; step 2 triggers it.

### Frames 1+2 — profile 0 partial update (long BAP message, group 0)

```
Frame 1 (start):  80 08 29 59  [ah0] 06 00 01
Frame 2 (cont):   C0  06 00 20 00
```

- `80` = long-message start, group 0 · `08` = payload length
- `29 59` = BAP header: OpCode 0x02 SetGet, LSG 0x25, Function 0x19
- 8-byte payload = 4-byte array header + 4-byte compact record:

| Off | Value | Field |
|---|---|---|
| 0 | `[ah0]` | array header: [ASG-ID:4][**Transaction-ID**:4] — TID echoed in FSG response (our TX code rolls the TID; thomasakarlsen identified this as the array header on PR #1430). **Not a start/stop selector** — an implementation that hardcodes `0x22`=start/`0x23`=stop is reading two consecutive TIDs of one start/stop pair. Proven on-car (`can3-bapprobe-20260723`, E1a/E1b): a START burst carrying `0x23` started clima, a STOP carrying `0x22` stopped it — direction is set by frame 3 alone. Car-native traffic shows ASG-ID 2, TID rolling `0x2a`/`0x2b`/`0x2c`. |
| 1 | `06` | array header: RecordAddr = 6 (compact record) *(MIB2 FW: "revision tag")* |
| 2 | `00` | array header: startIndex |
| 3 | `01` | array header: elementCount = 1 |
| 4 | `06` | operation = climate + climateWithoutExternalSupply |
| 5 | `00` | operation2 = none |
| 6 | `20` | **maxCurrent = 32 A** (1 A/LSB) |
| 7 | `00` | targetChargeLevel = 0 (not charging) |

**There is no temperature in this message.** The car climatizes to the temperature stored in the global profile (set in infotainment / via a RecordAddr-0 profile write). An earlier revision of this doc misread byte 6 as a temperature (the `(°C−10)×10` encoding was confirmed on the `0x17330110` setpoint *status broadcast*, ports 0x1B/0x21, dash-knob sweep, Captures 2/7 — a real encoding, wrong message). Corrected per MIB2 firmware RE (`SetBatteryControlProfileListRA6`) and smartkar docs, PR #1430 review.

> Firmware note: earlier `SendClimaBapBurst()` encoded the `cc-temp` config into byte 6 (sent as maxCurrent, e.g. 21 °C → 110 A). Fixed to constant `0x20`; the `cc-temp` param and web slider were removed since they had no wire effect. Confirmed on-car: old firmware really did put `0x96` there for a 25 °C setting, current firmware sends `0x20` — the value tracked our config, not the car's behaviour, which is what makes it our bug rather than a temperature field.
>
> Setting an explicit temperature would need a RecordAddr-0 ProfilesArray write (byte 12, `raw = °C × 10 − 100`). **We have never sent one.** Note the asymmetry: RecordAddr-0 *Get requests* (`80 04 19 59 [tid] 00 00 04`) appear in nearly every capture in the corpus, but they are RX — car/OCU-originated. We have never decoded the *content* of a RecordAddr-0 response, so the byte-12 temperature field remains unconfirmed on this car.

### Frame 3 — trigger (Function 0x18, short message)

```
29 58  00 01     START (immediate = profile 0)
29 58  00 00     STOP
```

`29 58` = SetGet, LSG 0x25, Function 0x18 (ClimateOperationModeInstallation). Payload byte 0 = profileId 0 (global); byte 1 = bitmask: bit0 immediately, bit1–3 timers 1–3 *(MIB2 FW)*. Observed on-car: `01` starts, `00` stops.

## Timer/Schedule Slot Encoding (Functions 0x14–0x16)

8-byte FSG status record: `ff ff ff [hour] [temp_byte] fe [slot_id] 00`

- Byte 3: departure hour (direct decimal, e.g. `0x11` = 17:00)
- Byte 4: `celsius + 35` (e.g. 20 °C → `0x37`)
- Byte 6: slot ID (0–2)

(On-car observation; not yet reconciled with smartkar's full-profile field layout.)

## KCAN Bus Wake-Up

### NM Alive Frame (sent from deep sleep — requirement INFERRED, not proven)

The reasoning is that the FSG rejects BAP from nodes outside the OSEK NM ring. **No capture in our corpus demonstrates this** — we have never recorded a clean bus-asleep BAP command sent *without* the NM frame and observed a rejection, so the requirement is inferred from protocol docs plus the fact that the sequence below works. Treat as unconfirmed.

The 2026-07-23 probe session (`can3-bapprobe-20260723`) looked like a confirmation at the car — a burst with no NM frame was ignored, and rejoining the ring "fixed" it — but the capture does not hold up: the two no-NM failures coincided with a climbing TX-error count (real bus contention), and the failed bursts were also partly scrambled by a wolfSSH paste truncation, so "no ring membership" and "garbled/contended frames" cannot be separated. Still inferred. A clean settle needs a single well-formed burst with no preceding NM alive, sent on a verified-quiet bus (zero `3CER` errors in the surrounding seconds): silent → confirmed; starts → the requirement is false. The probe runbook (`bap-probe-runbook.md`) has the method.

Send before any BAP command when bus was sleeping:

```
CAN ID:  0x1B000067  (29-bit extended)
DLC:     8
Data:    67 10 41 84 14 00 00 00
```

- `0x67` = OVMS KCAN NM node ID
- Byte 1 = `0x10` = NM alive with ring participation bit
- Wait ≥ 1 s for ring to settle before sending BAP

Implemented in `CommandWakeup()`, called by `CommandClimateControl()` when `m_bus_idle_ticks >= VWEGOLF_BUS_TIMEOUT_SECS`.

### Full Wake Sequence (bus sleeping)

1. Send `0x17330301` (dominant-bit wake, TX_Fail expected — wakes transceivers)
2. Send `0x1B000067` NM alive (joins ring)
3. Wait 1 s
4. Send 3-frame clima sequence

**Warning:** do not use the start frame as wake ping — if TX_Fail, continuation arrives orphaned (~110 ms later), FSG discards the partial long message, clima doesn't start.

(smartkar additionally documents a `0x5A7` keepalive every 200–500 ms and wake payload `40 00 01 1F`; our sequence above is what's validated on this car.)

### Channel-open ping (bus already active)
```
can can3 tx extended 17332501 19 42
can can3 tx extended 17332501 19 41
```
The car sends this two-frame channel-open before its own transactions. On-car
test (Capture `can3-bapprobe-20260723`, run E3a vs a rejoin-only start) shows a
warm-bus clima start succeeds **with or without** it — not required once the bus
is up. Necessity from deep sleep is untested.

(An earlier revision of this doc gave the ping as a single `09 41` frame; that
byte pair appears in **zero** captures across the corpus while `19 41`/`19 42`
appear throughout — the `09 41` was a transcription slip.)

## Test Commands (OVMS shell)

Start:
```
can can3 tx extended 17332501 80 08 29 59 01 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 01
```

Stop:
```
can can3 tx extended 17332501 80 08 29 59 02 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 00
```

Bus effects: `0x3E9` transitions from sentinel to live values; `0x3B5`/`0x530` follow. Blower activates within ~10 s.

## FSG ACK Pattern (0x17332510)

Immediate response (~1 s after command):
```
80 0a 49 59  {tid} 04 46 00   [+ continuation]
```
`49 59` = OpCode 0x04 Status, LSG 0x25, Function 0x19 — the SetGet confirmation. `{tid}` = our array-header byte 0, matching command to response via the Transaction-ID.

**The `| 0x80` echo is write-ACK-specific, not a general rewrite.** For this SetGet-*write* ACK the FSG returns our byte with the high bit set (`0x07` → `0x87`, 16/16 write bursts across the corpus). A plain *GET* reply on the same function echoes the byte **verbatim** — on-car `0x17332501` GETs with byte 4 = `0x0a` and `0x2a` came back `0x0a` and `0x2a` (Capture `can3-bapprobe-20260723`, runs E2a/E2b). So the high bit is a property of the write-ACK exchange, not an ASG-ID rewrite of an "invalid" nibble — an earlier revision of this doc guessed the latter and it is disproven.

After ACK: FSG sends ~4 keepalive cycles on `17332501` at 5 s intervals (~16 s), then silent until next command.
