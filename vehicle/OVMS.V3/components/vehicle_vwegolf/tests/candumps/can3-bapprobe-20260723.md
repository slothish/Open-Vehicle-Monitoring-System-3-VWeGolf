# can3-bapprobe-20260723 — BAP probe matrix (warm bus)

Shell-driven probe session per `docs/bap-probe-runbook.md`. Purpose: settle four
BAP protocol claims on the car before publishing them upstream (issue #690).
KCAN (can3) only. Ignition off, `xvg offline`, warm bus. `.crtd` gitignored (PII).

Each run preceded by a marker on `0x1FFFFF00` and an NM-alive rejoin
(`1B000067`), since `xvg offline` drops ring membership within ~1 min.

## Runs and verdicts

| run | byte4 | what | verdict (vs 0x03B5 ClimaRunning) |
|---|---|---|---|
| R0 | 0x01 | control start | first two attempts failed (scrambled paste + bus contention); succeeded on rejoin retry |
| R1 | 0x02 | control stop | stopped |
| E2a | 0x0a | GET, ASG-ID nibble 0 | FSG reply echoes `0x0a` **verbatim** (t+19 ms) |
| E2b | 0x2a | GET, ASG-ID nibble 2 | FSG reply echoes `0x2a` **verbatim** (t+20 ms) |
| E1a | 0x23 | START carrying the "stop" selector | clima **started**, 1.95 s lag |
| E1b | 0x22 | STOP carrying the "start" selector | clima **stopped**, 0.07 s, held ~46 s |
| E3a | 0x03 | channel-open (`19 42`/`19 41`) then start | **started**, same as a rejoin-only start |

## Claims settled

- **byte 4 is not a start/stop selector** — CONFIRMED. E1a/E1b: the two most
  adversarial values (`0x23` as start, `0x22` as stop) both worked; frame 3's
  last byte alone sets direction. Backs the array-header reading thomasakarlsen
  gave on PR #1430.
- **`| 0x80` echo is write-ACK-specific** — CONFIRMED. GET replies (E2a/E2b)
  echo byte 4 verbatim; the high-bit-set echo (16/16 in the corpus) is the
  SetGet-write ACK only. Kills the earlier "invalid ASG-ID nibble gets rewritten"
  guess.
- **channel-open unnecessary on a warm bus** — CONFIRMED (E3a vs rejoin-only
  start). Deep-sleep necessity untested.

## Claims NOT settled

- **NM-ring membership required** — UNSETTLED. Looked confirmed at the car (no-NM
  burst ignored, rejoin fixed it) but the capture won't support it: the no-NM
  failures coincided with a climbing TX-error count, and the failed bursts were
  partly scrambled by a wolfSSH paste truncation — "no ring" and "garbled/
  contended frames" can't be separated. Needs a clean well-formed no-NM burst on
  a verified-quiet bus.

## Caveats for re-analysis

- `crtd.py` drops all TX (`3T`) records — recover our frames and the markers by
  grep (`1FFFFF00`, `3T29 17332501`), not `load()`.
- This capture has an unusually high, TX-specific logger drop rate: the middle
  (continuation) frame of a burst is absent in 4/7 bursts, uncorrelated with
  outcome (5/7 wide-gap bursts still succeeded). Treat "missing frame 2" as a
  logger artifact, not an on-wire gap.
- The marker labeled R0-OK in the runbook's map was in fact a second failed
  attempt; the real R0 success was an unmarked rejoin retry ~75 s later.
