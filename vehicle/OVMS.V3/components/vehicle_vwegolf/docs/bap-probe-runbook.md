# BAP Probe Runbook — at the car

Shell-only. No build, no flash. Settles the byte-4 selector question, the
ASG-ID/echo question, and channel-open necessity on a warm bus.

Signal definitions live in `vwegolf.dbc`. Frame semantics live in
`clima-control-bap.md`. Nothing is restated here.

**Run everything inside ONE interactive SSH session.** One `ssh` invocation
per command puts hundreds of ms between frames and a BAP inter-frame timeout
will masquerade as "the car rejected it".

---

## Connecting

Join the OVMS WiFi hotspot first. Default host is `192.168.4.1` (override with
the `OVMS` env var, or add `ovms.local` to `/etc/hosts`).

One SSH session does everything — capture included. `capture.sh` is only a
convenience wrapper around the same console commands, and it needs a laptop with
`bash` and `scp`; driving the log by hand does not.

### Capture, driven from the console

Logging goes to the module's SD card at `/sd` (`SD_DIR` in `capture.sh`). Start
before R0:

```
can log start vfs crtd /sd/can3-bapprobe-20260723.crtd 3
can log status
```

The trailing `3` is the bus filter for can3 (`1`/`2` for can1/can2; omit for all
buses). Confirm `can log status` reports the log active before sending anything —
a matrix run against a dead logger is a wasted trip.

At the end:
```
can log stop
```

Retrieve later, from the laptop on the hotspot:
```
scp -O -o StrictHostKeyChecking=no \
       -o HostKeyAlgorithms=+ssh-rsa \
       -o PubkeyAcceptedKeyTypes=+ssh-rsa \
       ovms@192.168.4.1:/sd/can3-bapprobe-20260723.crtd .
```

Rename it to the corpus convention (`can3-<fw-version>-<YYYYMMDD-HHMMSS>.crtd`,
get the version from `metrics list m.version`) when you file it and register it
in `captures.tsv`.

If the SD card is missing or unmounted, `can log start` fails and there is no
point continuing — `ls /sd` first if unsure.

### The session

OVMS runs wolfSSH, which only offers legacy
algorithms; a modern OpenSSH client refuses them unless told otherwise. Same
flags `capture.sh` uses:

```
ssh -o StrictHostKeyChecking=no \
    -o HostKeyAlgorithms=+ssh-rsa \
    -o PubkeyAcceptedKeyTypes=+ssh-rsa \
    ovms@192.168.4.1
```

(`capture.sh` also passes `-o BatchMode=yes`; leave it off here so you get a
password prompt if the key isn't accepted.)

Optional, if you're on a laptop: run `script ~/bap-probe-$(date +%Y%m%d-%H%M).log`
first and connect inside it. The console echo is then the only record of what the
module actually accepted when a run goes sideways. The `.crtd` is the real
evidence either way, so skip this on a phone.

**Never** run commands as `ssh ovms@192.168.4.1 "can can3 tx ..."`, one
invocation per line. Each connection costs hundreds of ms and the burst frames
must land inside ~200 ms of each other. Paste each block into the open session.

### Paste pre-flight — do this once before R0

wolfSSH has a small input buffer and no flow control, so a fast multi-line paste
may be truncated or mangled. Prove it isn't, with a harmless block:

```
metrics list v.e.hvac
metrics list v.b.soc
metrics list m.version
```

All three must echo and execute. If any line is swallowed or garbled, the burst
blocks will break the same way — fall back to sending each burst line by hand as
fast as you can, and treat any negative result as suspect until the capture
confirms the frame gaps (see the analysis gotcha below).

## Preconditions

Capture already logging (`can log status` active) and paste pre-flight passed.
Then, in the same session:
```
xvg camping status          # must be off — camping trips clima and re-arms the keepalive
xvg offline                 # stops 0x5A7 AND 0x1B000067 (both gated on m_ocu_active)
metrics list v.b.soc        # abort if below 40% — the matrix runs the blower repeatedly
metrics list v.e.hvac
```

Car: ignition off, doors closed, locked, parked. KCAN warm — run within a few
minutes of locking.

**Do not** use the OVMS Connect app, `wakeup`, `xvg climate`, or camping during
the matrix. `WakeKcanBus()` and `SendClimaBapBurst()` both re-arm `m_ocu_active`,
which restarts the keepalive and invalidates every run after it.

---

## Run marker

Send immediately before each run:

```
can can3 tx extended 1FFFFF00 <run_id> <b4> <trigger> <chanopen>
```

| run | id | example marker |
|---|---|---|
| R0 control start | `00` | `can can3 tx extended 1FFFFF00 00 01 01 00` |
| R1 control stop | `01` | `can can3 tx extended 1FFFFF00 01 02 00 00` |
| E2a get nibble 0 | `2A` | `can can3 tx extended 1FFFFF00 2A 0A 00 00` |
| E2b get nibble 2 | `2B` | `can can3 tx extended 1FFFFF00 2B 2A 00 00` |
| E2c setget nibble 2 | `2C` | `can can3 tx extended 1FFFFF00 2C 2A 00 00` |
| E1a start w/ 0x23 | `1A` | `can can3 tx extended 1FFFFF00 1A 23 01 00` |
| E1b stop w/ 0x22 | `1B` | `can can3 tx extended 1FFFFF00 1B 22 00 00` |
| E3a chanopen warm | `3A` | `can can3 tx extended 1FFFFF00 3A 03 01 01` |
| E3b cold no prefix | `3B` | `can can3 tx extended 1FFFFF00 3B 0B 00 00` |
| E3c cold w/ prefix | `3C` | `can can3 tx extended 1FFFFF00 3C 0C 00 01` |
| E4C maxCurrent preserve | `4C` | `can can3 tx extended 1FFFFF00 4C 0D 01 00` |

`0x1FFFFF00` is absent from `vwegolf.dbc`, outside the NM range (`0x1B0000xx`)
and outside BAP (`0x1733xxxx`). No ECU filters for it.

## Ring rejoin — before every run

`xvg offline` stops the periodic NM alive, and ring membership decays within
~1 min. So after the marker and before each run's BAP frames, rejoin the ring
and wait ~1 s:

```
can can3 tx extended 1B000067 67 10 41 84 14 00 00 00
```

Observed 2026-07-23: bursts sent without a recent rejoin were ignored (no FSG
reply, no start). This is *practically* necessary but its *mechanism* is not
proven — those same failures were confounded by bus contention and a scrambled
paste, so whether the ring is truly required is still open (see
`clima-control-bap.md`). The one run that would settle it: a single clean,
well-formed burst with **no** preceding rejoin, on a verified-quiet bus (no
`3CER` errors in the surrounding seconds). Silent → ring required; starts → not.

Every run block below assumes the marker + rejoin have gone first.

---

## R0 — control start

Without this, every negative below is uninterpretable.

```
can can3 tx extended 17332501 80 08 29 59 01 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 01
```

## R1 — control stop

```
can can3 tx extended 17332501 80 08 29 59 02 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 00
```

## E2a / E2b — ASG-ID echo (read-only, highest value)

Single frame each, ~5 s apart. Watch `0x17332510` between them.

```
can can3 tx extended 17332501 80 04 19 59 0a 00 00 04
```
```
can can3 tx extended 17332501 80 04 19 59 2a 00 00 04
```

## E2c — disambiguator

**Only if E2a and E2b echo identically.** Clima must already be idle — the
trigger below is a stop, so nothing actuates.

```
can can3 tx extended 17332501 80 08 29 59 2a 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 00
```

## E1a — START carrying the "stop" selector 0x23

```
can can3 tx extended 17332501 80 08 29 59 23 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 01
```

## E1b — STOP carrying the "start" selector 0x22

Run **while E1a's conditioning is live**. Wait for `v.e.hvac` true, then 60 s.

```
can can3 tx extended 17332501 80 08 29 59 22 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 00
```

## E3a — channel-open, warm bus

R0 is the "absent" arm; this is the "present" arm.

```
can can3 tx extended 17332501 19 42
can can3 tx extended 17332501 19 41
can can3 tx extended 17332501 80 08 29 59 03 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 01
```

Then stop with the R1 block.

## E3b / E3c — cold bus, protocol half

Separate captures. Leave the car untouched and locked ≥15 min; confirm from the
capture that KCAN has gone quiet. Still `xvg offline`.

E3b, no channel-open:
```
can can3 tx extended 17330301 40 00 01 1F 00 00 00 00
can can3 tx extended 1B000067 67 10 41 84 14 00 00 00
can can3 tx extended 17332501 80 04 19 59 0b 00 00 04
```

Sleep the bus again ≥15 min, then E3c — same, with `19 42` / `19 41` inserted
before the GET and TID `0c`.

This tests whether the FSG **answers** from cold, not whether clima **starts**
from cold. The actuation half needs firmware.

## EXP-4R — let the car write the setpoint

Separate capture, bus awake. Change the departure-climate temperature in the
infotainment **charging/climate menu** (NOT the dash HVAC knob — that is
`0x17330110`, a different message) from its current value to a distinctly
different one, e.g. 20 °C → 25 °C. Wait 10 s. Change it back.

Captures the car's own RecordAddr-0 SET, which yields the write-direction
framing from a guaranteed-correct example.

## E4C — does a clima start overwrite the user's max charge current?

Settles whether our unconditional `maxCurrent = 0x20` in the compact record
overwrites a configured value, or whether the field is preserved (which is what
the OEM trace suggests — thomasakarlsen's capture also shows `0x20` there, so we
may simply be emulating the car correctly).

**Setup, in the infotainment charging menu:** note the current max charge current,
then set it to a distinctly different low value — 10 A or 5 A, something that
cannot be confused with 32. Write the original down; step 4 restores it.

Warm bus, `xvg offline` still in force.

Read before:
```
can can3 tx extended 17332501 80 04 19 59 0d 00 00 04
```

Start burst (this is the write under test):
```
can can3 tx extended 17332501 80 08 29 59 05 06 00 01
can can3 tx extended 17332501 C0 06 00 20 00
can can3 tx extended 17332501 29 58 00 01
```

Wait 20 s for the profile response to settle, then read after:
```
can can3 tx extended 17332501 80 04 19 59 0e 00 00 04
```

Stop with the R1 block, then **restore the original max charge current in the
infotainment** — this run deliberately leaves a non-default setting behind.

Compare `maxCurrent` at abs offset 8 of profile 0's record between the two reads.
Reassembly is required; see the analysis gotcha below.

## R9 — leave the car in a known state

Run the R1 stop block, confirm `v.e.hvac` false, `xvg camping status`.

---

## Verdicts

`0x03B5` ClimaRunning is ~8 Hz and **thermostat-cycles with off-gaps up to
~14 s**. Therefore:

- **START confirmed** = at least one `ClimaRunning == 1` within 20 s of frame 3.
- **STOP confirmed** = **zero** `ClimaRunning == 1` for a continuous **≥30 s**.
  A 15 s window reports a thermostat gap as a successful stop. This is the most
  likely way to publish a wrong result.

`metrics list v.e.hvac` is a usable live proxy for start, but lags stop by
`VWEGOLF_HVAC_RUN_HOLD_SECS` (20 s).

| run | confirms | refutes | ambiguous → rerun |
|---|---|---|---|
| R0 | start within 20 s | no start → **ABORT MATRIX**, nothing below is interpretable | FSG replies but no blower → recheck preconditions |
| R1 | zero running ≥30 s | still running past 30 s | stops at 20–30 s → thermostat boundary, rerun at 60 s |
| E1a | start within 20 s → frame 3 rules, byte 4 is **not** a selector | no start, **and** E1b starts conditioning → selector reading is live | neither does anything → check inter-frame gaps before concluding |
| E1b | zero running ≥30 s → frame 3 rules both ways | keeps running past 30 s, or starts | ACK present, no state change → rerun after a clean R0 |
| E2a | echo reads `0x8a` | echo reads `0x0a` verbatim | no reply in 3 s → bus not warm, run R0/R1 first |
| E2b | echo reads `0x2a` verbatim → **rewrite hypothesis confirmed** | echo reads `0xaa` → hypothesis killed, `0x80` is a direction bit | E2a and E2b echo alike → nibble is not the discriminator, run E2c |
| E2c | `0x2a` verbatim → discriminator is the ASG-ID nibble | `0xaa` while E2b gave `0x2a` → discriminator is the **opcode**, publishable in its own right | no reply → rerun once, else declare claim 2 unsettled |
| E3a | starts, indistinguishable from R0 → channel-open unnecessary warm | R0 started, E3a did not → prefix actively harmful | both fail → bus state changed, rerun both back to back |
| E3b/E3c | FSG replies in **both** → channel-open not required from cold | reply only with the prefix → channel-open **required** on the wake path | neither replies → wake itself failed; verify the NM frame went out |
| EXP-4R | a RecordAddr-0 SET appears with byte 12 tracking the new setpoint | no SET appears → setpoint reaches the ECU another way | SET appears, byte 12 static → wrong menu used, rerun |
| E4C | `maxCurrent` still reads the value you set → field **preserved**, no defect, close it | reads `0x20` after having read your low value before → real overwrite, worth a one-line upstream fix | "before" read already shows `0x20` → the menu change never reached the profile; recheck the menu and rerun |

---

## Analysis gotcha — read before touching the capture

`crtd.py` **silently drops every TX record.** `_VALID_TYPES` is R-only, so
`load()` shows no markers and none of our own command frames, with no error.

Recover them by grep first:

```
rg -n '1FFFFF00' <capture>.crtd        # marker timestamps + run params
rg -n '3T29 17332501' <capture>.crtd   # our TX frames + inter-frame gaps
```

Convert those absolute timestamps against `Capture.t0_unix`, then use
`diff_window` / `cap.decode(dbc, '3B5', 'ClimaRunning')` on the RX-only capture
for the response side.

**Discard any negative result whose burst shows frame 1 → frame 3 gaps over
~200 ms.**

---

## Abort

```
can can3 tx extended 17332501 29 58 00 00     # bare stop trigger
xvg camping status
metrics list v.e.hvac
```

Then stop transmitting and let the bus sleep. A wedged BAP channel clears when
KCAN idles (≥15 min) or on an ignition on/off cycle. Never retry harder into a
suspected wedge.

---

## Per-capture notes

Record in the capture `.md`: firmware version, ignition state, `xvg offline`
confirmed, run order with marker bytes, SoC before/after, verdict per run.
Register in `captures.tsv`. No VIN, plate, or location.
