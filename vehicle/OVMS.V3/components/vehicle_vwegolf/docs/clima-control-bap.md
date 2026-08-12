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

### RecordAddr-0 Response Decode (Get replies)

RA0 *Get requests* use a 4-byte array header:
`[TID][flags|RecordAddr][startIndex][elementCount]` — every observed request
(RA0 gets, the RA6 compact write above) carries `0x00`/`0x06` in the
flags/RecordAddr byte, so whether the high bits are really flags on the
request side, as they are on the response side below, is unconfirmed; the
notation is kept parallel to the response header rather than stripped down.
The **response** array header is one byte longer — 5 bytes — followed by a
leading record-body byte before the rest of the record:

| Off | Field |
|---|---|
| 0 | Transaction-ID (echoes the request TID) |
| 1 | `0x04`/`0x05` — response opcode/status |
| 2 | flags \| RecordAddr |
| 3 | startIndex |
| 4 | elementCount |
| 5 | leading record-body byte, informally called "profileId" below — values observed: `0x00`–`0x04`. Relationship to startIndex is unresolved: a `plen=39` single-record response has `startIndex=3` yet this byte reads `0x04` |
| 6+ | rest of record body — RecordAddr-0 full profile fields, byte 12 = temperature (per above) |

Assuming the response header matches the request's 4-byte shape misaligns
every record by one byte. Falsified directly on the corpus by record-split
exact-consumption: 577/577 RA0 responses split cleanly with zero leftover
bytes at header length 5, 0/577 at header length 4.

Content is confirmed, not just the header shape: 74 complete `plen=130`
four-record RA0 responses on CAN id `0x17332510` across the corpus. Worked
example: `tests/candumps/all-168388a82-dirty_ota_0_edge-20260524-221008.crtd`,
a 19-frame long message (sequence `80 c0..cf c0 c1`) — reassembled-payload
offset 18 (= header 5 + leading byte 1 + record-body offset 12, the "byte 12"
convention above) reads `0x78` = 22.0 °C.

Observed byte-12 values across the four profiles (by record position in a
full `plen=130` response — profile 0 = hidden global, 1–3 = departure timers
per the naming above, not the disputed leading-byte value from the table):
`0x64`/`0x78`/`0x96` = 20.0/22.0/25.0 °C. Profiles 1–3 read `0x00` at that
offset in every capture seen — only profile 0 carries a setpoint. That is
what the corpus shows, not a protocol guarantee: it does not prove profiles
1–3 can never hold one.

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
> Setting an explicit temperature would need a RecordAddr-0 ProfilesArray write (byte 12, `raw = °C × 10 − 100`). **We have never sent one.** Note the asymmetry: RecordAddr-0 *Get requests* (`80 04 19 59 [tid] 00 00 04`) appear in nearly every capture in the corpus, but they are RX — car/OCU-originated. We have decoded the *content* of RecordAddr-0 responses (see "RecordAddr-0 Response Decode" above) — the byte-12 temperature field is confirmed on this car.

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
CAN ID:  0x1B00007D  (29-bit extended)
DLC:     8
Data:    7D 10 08 01 14 00 00 00
```

- `0x7D` = OVMS KCAN NM node ID (`0x1B000067`/node `0x67` is a foreign ECU already
  on the ring, not ours — see `vwegolf.dbc`)
- Byte 1 = `0x10` = NM alive with ring participation bit
- Wait ≥ 1 s for ring to settle before sending BAP

Implemented in `WakeKcanBus()`, called by `CommandWakeup()`/`CommandClimateControl()`
when `m_bus_idle_ticks >= VWEGOLF_BUS_TIMEOUT_SECS`, and re-sent periodically by
`SendNmAlive()`.

### Full Wake Sequence (bus sleeping)

1. Send `0x17330301` (dominant-bit wake, TX_Fail expected — wakes transceivers)
2. Send `0x1B00007D` NM alive (joins ring)
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

## Confirmation-Signal Semantics — Corpus-Wide Scan (WI-CLIMA-CONF-1)

Scanned all 40 real captures currently symlinked into `tests/candumps/`
(the two committed `*-synthetic.crtd` fixtures excluded — they model DBC
signal decode, not BAP transaction pairing). Read directly off the raw
`.crtd` lines — RX, OVMS TX, and `CER TX_Fail` records all included.
`crtd.py`'s `load()`/`iter_frames()` only recognize `NRxx` record types and
silently drop our own TX and TX_Fail lines (capture-analysis skill rule 7);
a BAP confirmation is a TX→RX pair, so any scan built only on `load()`
would see zero commands and misread the whole channel as one-way. Pinned by
`tests/analysis/test_bap_confirm.py`.

### (a) `49 58 <flag>` is a transaction confirmation, never a state broadcast

27 `49 58 <flag>` short frames on `0x17332510` across **9** captures
(`all-168388a82-...-221008`, `all-168388a82-...-224827`,
`all-3.3.006-269-gab4f52853-...-124542`, `all-dc583be4a-...-132333`,
`can3-1aec82f33-...-095821`, `can3-bapprobe-20260723`,
`can3-dff221ec0-...-173035`, `can3-dff221ec0-...-180433`,
`kcan-can3-clima_control`). Every single one has a preceding own
`29 58 00 <flag>` (TX or `TX_Fail`) on `0x17332501` within 5.4 s — well
inside a 25 s correlation window — and zero have no preceding command at
all. `<flag>` mirrors the command byte (16× `01` start-ack, 11× `00`
stop-ack). No `49 58` was ever preceded only by a failed transmit — every
reply traces to an electrically successful send.

This is **one-directional**: every reply we observed is command-correlated,
but not every command gets a reply back (see "(e)" below) — the corpus has
5 `29 58` sends that got no `49 58` at all despite transmitting fine.

### (b) Confirm latency bands

Measured from the `29 58 00 <flag>` TX to the matching `49 58 <flag>` RX,
using `0x3B5` `ClimaRunning` bit7 (see `vwegolf.dbc` `BO_ 949`) sampled
immediately before the command to classify scenario:

| Scenario | n | Range |
|---|---|---|
| start, from off (`ClimaRunning`=0 before cmd) | 13 | 3.935–5.380 s |
| start, already running (`ClimaRunning`=1 before cmd) | 3 | 0.128–0.214 s |
| stop | 11 | 0.033–0.094 s |

All 27 replies land in one of these three bands (13+3+11=27); all matched
commands were electrically-successful TX (`TX_Fail` sends never got a
reply — consistent with "(a)").

### (c) TID-matched `49 59` latency

Function 0x19 (profile write, long message `80 08 29 59 [tid] 06 00 01` +
continuation) is a separate transaction from the 0x18 trigger in "(a)"/"(b)".
Its confirmation is the long message `80 0a 49 59 [tid|0x80] ...`.

29 electrically-successful TX bursts of this shape in the corpus (plus 4
that never left the transceiver — `TX_Fail`). Of the 29: **23 matched** a
`49 59` reply with `TID = ours | 0x80` — latency **0.023–2.612 s**. The
remaining **6 got no matched `49 59` at all**, despite transmitting fine.
`TX_Fail` bursts never matched (0/4) — a frame that never reached the bus
cannot be confirmed.

**"Present on every successful burst" does not hold in the strict sense.**
Two of the 6 unmatched bursts belong to sessions that plainly *did* start:
`can3-dff221ec0-...-180433` tid `0x01` (t≈+911.7 s) and
`can3-bapprobe-20260723` tid `0x23` (t≈+1811.9 s) — both have no `49 59`
for their profile-write burst, yet each is followed by a normal `29 58 00
01` trigger that *does* get a `49 58 01` ack (5.38 s later for the
bapprobe case, whose `0x3B5` `ClimaRunning` bit7 also flips to 1 at
+1.95 s, before the transactional ack). So a missing `49 59` does not by
itself mean the write failed — it only means the FSG chose not to send
that particular ack this time. The other 4 unmatched bursts (`can3-bapprobe
-20260723` tid `0x01` ×3 and tid `0x22`, `can3-dff221ec0-...-113543` tid
`0x05`) sit in sessions where no BAP session resulted at all (see "(e)").
So: **presence** of a TID-matched `49 59` was never seen on a failed burst
(23/23 successes in this corpus); **absence** does not reliably distinguish
success from failure on its own.

Separately, the raw corpus has **24** `49 59` long-message-start replies,
not 23 — one (`can3-bapprobe-20260723`, tid `0x83`, t≈+1900.3 s) has no
corresponding OVMS-sent tid-`0x03` burst anywhere in that file. The same
capture also shows a foreign ASG issuing its own Gets on the shared
`0x17332501` channel with plain-echoed TIDs (`0x2c`–`0x2e`, no `|0x80`
rewrite, matching the existing "FSG ACK Pattern" note above) — the stray
`0x83` reply most likely belongs to that foreign ASG's own SetGet sharing
the same TID number space, not to us. Not investigated further; noted so a
future scan doesn't miscount it as ours.

### (d) BAP opcode-7 (Error)

**Zero** opcode-7 (Error) elements — short-message (`byte0 & 0xF0 ==
0x70`) or long-message-start (`byte0` in `0x80/0x90/0xA0/0xB0`, `byte2 &
0xF0 == 0x70`) form — on either `0x17332501` or `0x17332510`, anywhere in
the 40-file corpus. See `vw-bap-protocol.md` "OpCodes"/"Frame Encoding"
for the bit layout this is derived from.

### (e) BCU-status recency vs. success/failure — correlation, not proof

Gap = time since the last RX frame on `0x17332510` (any function) before a
`29 58` command TX.

- **Successes** (27, got a `49 58` reply): gap **0.004–1.388 s**.
- **Failures** split into two distinct modes, conflating them hides real
  structure:
  - **TX succeeded, no reply** (5): gap **0.018–67.882 s** — mostly large
    (4 of 5 are ≥4.5 s) but **one outlier is 0.018 s**
    (`can3-dff221ec0-...-180433`, t≈+911.7 s — the same event flagged in
    "(c)" whose accompanying 0x19 burst *did* get confirmed 0.984 s later).
    So low BCU-status recency does not guarantee a reply.
  - **`TX_Fail` — never reached the bus** (2, both
    `kcan-can3-clima_off`): gap **0.153–0.838 s**. This is arbitration
    failure (dominant-bit collision on a busy bus), not a BCU-silence
    story — recency doesn't explain it at all, because the frame was never
    transmitted to be received.
- Net: recency correlates loosely with the TX-succeeded-no-reply failure
  mode (successes cap at 1.388 s vs. 4 of 5 such failures ≥4.5 s), but it
  is not a clean discriminator, and a second, unrelated failure mode
  (transmit arbitration loss) exists that recency says nothing about.

### (f) NM-release evidence

`all-dc583be4a-dirty_ota_0_edge-20260503-132333.crtd`: our own NM alive TX
(node `0x67` — this capture predates the WI-NM-1 node fix; `vwegolf.dbc`
`BO_ 2600468583`/`BO_ 2600468605` comments record that `0x67` was later
disproven as a foreign ECU's node and OVMS moved to `0x7D`) runs from
+0.04 s to **+8.72 s**
after the capture's first frame (10 sends), then stops for good. BAP
traffic (`0x17332501`/`0x17332510`) and the file itself both continue to
**+533.6 s** (the file is truncated by a module reset ~7 min later per
this capture's own `.md` notes, so this is a lower bound on the true
session length, not necessarily its natural end). `0x3B5` `ClimaRunning`
bit7 flips to 1 at +2.82 s and stays on for the rest of the (truncated)
file. Meanwhile foreign NM node `0x1B000046` carries the ring throughout
at a **~200 ms** median frame interval (n=2669, spans the full file) —
BAP traffic keeps flowing for ~525 s after our own NM participation
stops, with a foreign node visibly holding the ring the whole time.
