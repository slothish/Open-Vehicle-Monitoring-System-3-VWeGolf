# can3-3.3.006-364-gaefeffec5_ota_0_edge-20260731-112417.crtd — Capture Notes

## Capture info

| Field | Value |
|---|---|
| Firmware | `3.3.006-364-gaefeffec5/ota_0/edge` |
| Bus | can3 (KCAN) |
| Captured | 20260731-112417 |
| Duration | ~180s wall, 29.3s of bus traffic |
| Frames | 14384 raw |
| Logger stats | Messages:14408 Dropped:0 Filtered:78 |

## Sequence

WI-NM-3 gate. Car left to fall asleep with the module attached; bus confirmed
quiet (0.0 f/s across 71s, logger recorded 1 frame in 4 minutes). `wakeup`
issued over SSH at 11:25:51. Capture spans the sleep tail, the wake, and the
bus coming back up.

Firmware under test is `aefeffec5` — the NM node fix that stops the module
impersonating live ECU node `0x1B000067` and moves it to `0x1B00007D`.

---

## Notes

### Wake succeeded

| | pre-wake 11:25:39 | post-wake 11:26:04 |
|---|---|---|
| Rx pkt | 479485 | 493676 (delta 14191, ~570 f/s) |
| Tx pkt | 0 | 193 |
| Tx fails | 0 | 1 |
| Wdg Resets | 0 | 0 |

### Impersonation fix confirmed on the wire

| NM node | RX | TX |
|---|---|---|
| `1B000067` | 20 | 0 |
| `1B00007D` | 0 | 30 |
| `1B00000C` `0E` `10` `14` `46` `4A` `4B` `A9` | RX only | 0 |

`0x67` is RX-only — the live ECU owns it and we no longer transmit it.
`0x7D` is TX-only with no other claimant on the bus. This is the direct
on-wire confirmation the fix never had across three prior rounds.

### Attribution: dominant bits woke the bus, not the NM frame

```
first RX frame       +0.000s
our first NM TX      +0.039s
RX before first TX   47 frames
```

Module log shows `WakeKcanBus: asserting dominant bits on KCAN` immediately
before traffic appears. 47 frames arrived before our first `0x1B00007D`.
The supportable claim is **"wake works and the new node does not break it"**,
NOT "node 0x7D wakes the bus".

### TX error excursion is benign

`txerr` climbed 8 -> 128 in +8 steps with `TX_Err_Warn` then `TX_Err_Passv`,
one `txfail`, then decayed 122 -> 76 as the bus came up. This is ordinary
transmit-into-a-dead-bus behaviour: unacknowledged frames bump the TX error
counter until other nodes wake and start ACKing. Not a defect.

### PII-bearing frame IDs in this capture (IDs only — the .crtd stays local)

Scanned every id's concatenated payload for printable-ASCII runs. Three ids
carry strings; everything else is binary. Recording the ids so future captures
can be triaged without re-deriving this:

| ID | Frames | Content |
|---|---|---|
| `6B4` | 147 | VIN — DBC `BO_ 1716 VIN_Broadcast`, spread over 3 frames (index in d[0]). DBC documents it as FCAN, but it appears here in a bus-3-filtered capture. |
| `17330D10` | 98 | Driver/infotainment **profile name**. This capture holds the generic default (`Gast`), but the frame class carries user-chosen names. |
| `17333310` | 60 | Unidentified BAP string frame, not in the DBC. Short ASCII runs. Treat as PII until identified. |

`candumps/.gitignore` already excludes `*.crtd`, so none of this leaves the
machine — but a scrub is required before pasting excerpts anywhere, and
`17330D10` in particular would leak a real name on a car with a personalised
profile.

### OPEN — our NM payload differs from every real node

```
real nodes   0e 00 01 01 04 08 04 00
             a9 00 01 01 04 00 00 00      pattern: <id> 00 01 01 04 ...
ours         7d 10 08 01 14 00 00 00      pattern: <id> 10 08 01 14 ...
```

All 9 observed real nodes use `00 01 01 04` in bytes 1-4; we use `10 08 01 14`.
Byte 1 `0x10` is plausibly correct (AUTOSAR NM control bit vector, Active
Wakeup bit, set by the wake initiator). Bytes 2-4 have no such justification.
Nothing observably broke. Tracked as WI-NM-4; relates to the unsettled
OSEK-vs-AUTOSAR question at `src/vehicle_vwegolf.h:114-115`.
