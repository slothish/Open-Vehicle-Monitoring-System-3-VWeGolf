# CLAUDE.md

Build/framework/capture → `docs/dev-guide.md`.

## Project

OVMS3 fork, VW e-Golf. ESP32 telemetry/ctrl via CAN. Focus: `vehicle/OVMS.V3/components/vehicle_vwegolf/`.

**App** = OVMS Connect by CrashOverride2 (iOS/Android). Metric names, cmd behavior, timing → target this app.

**OBD port** = Diag CAN, ign-on only. Remote (clima/wake/lock/charge) needs KCAN via J533 harness. e-Golf via J533, not direct ECU.

**BAP** = VW *Bedien-/Anzeigeprotokoll*. KCAN, not ISO-TP/UDS. No std poller — direct frame build. Refs: `vehicle/OVMS.V3/components/vehicle_vwegolf/docs/vw-bap-protocol.md`, `…/docs/clima-control-bap.md`.

**Ext RE:** https://github.com/thomasakarlsen/e-golf-comfort-can — primary KCAN/BAP ref.

**AI warn:** upstream rejects unvalidated AI code. Validate vs real CAN before upstream PR.

## e-Golf bus map

- CAN2 (`m_can2`, FCAN): powertrain — gear, BMS, VIN
- CAN3 (`m_can3`, KCAN): convenience — speed, SoC, charge, body, clima
- Prefix: `xvg` (CLI/cfg/metrics)
- KCAN/BAP capable: wake · clima start/stop · temp/dur · min charge · profiles · horn/indicator

## Frame decode docs

**Single source:** `vehicle/OVMS.V3/components/vehicle_vwegolf/docs/vwegolf.dbc`. All signal definitions (CAN ID, bit pos, scaling, units, sender/receiver, comments). Other docs (PROJECT.md, captures.tsv, per-cap .md, BAP refs) reference signal names — DO NOT restate decode details. Re-statement = drift = bugs.

When a capture confirms a new signal: add to `vwegolf.dbc`, point cap notes at it.

## Code navigation

Use `rg` (ripgrep) for symbol/string lookup — fast, no token overhead:
```bash
rg 'IncomingFrameCan3' vehicle/OVMS.V3/components/vehicle_vwegolf/
```

Key entry points: `IncomingFrameCan3()` · `IncomingFrameCan2()` · `make_vehicle()` · `test_clima_all()`.

## CRTD captures — NEVER read whole

`.crtd` files = 10k–60k+ lines each. Reading whole = context burn for nothing.

**Use:** `vehicle/OVMS.V3/components/vehicle_vwegolf/tests/analysis/crtd.py` (in `.venv`).

API: `load(path, bus=N)` → `Capture` · `load_multi(path)` mixed-bus · `iter_frames`/`iter_crtd` stream · `trajectory_*`/`byte_stats`/`hexdump_window` · `gear_timeline`/`gear_windows` FCAN drive. README = mislabeled-bus list (`bus=N` explicit).

**Stimulus hunt helpers:**
- `transitions(records, off, mask=0xFF, shift=0)` → `[(t,prev,curr)]` on change. Bit/nibble/byte edges.
- `frame_rate(records)` → `FrameRate(count, span_s, mean_hz, median_gap_s, max_gap_s, long_gaps)`. Catch stale-rebroadcast / bus-sleep.
- `diff_window(cap, t_event, pre, post, ids=None)` → ranked `DiffEntry`s. Replaces apr7_*/cap_*/ack_check_recent boilerplate.
- `mux_split(records, off=0, mask=0xFF, shift=0)` → `{mux: records}`. 0x1A5554A8: `mask=0xF0, shift=4`.

**DBC decode:** `load_dbc()` → cantools DB · `cap.decode(dbc, fid, sig=None)` → `[(t,val),…]`. `fid`=hex str/int. `sig=None` → dict all sigs. Short/sentinel frames silently skipped. Hex key fmt: 3-char `<=0x7FF`, 8-char ext. DBC loaded `strict=False` (ChargeType/Port overlap intentional).

Pattern: throwaway script `tests/analysis/scratch/`, `.venv`, read output not frames.

## DBC regression test

`tests/analysis/test_dbc_decode.py` = pytest pinning per-cap `.md` ground truth (wheelspeed scale, gear nibble, SoC range, peak power) vs `vwegolf.dbc`. Silent DBC break → red test.

Run: `make -C vehicle/OVMS.V3/components/vehicle_vwegolf/tests pytest` (need `.venv` w/ cantools+pytest). Separate from C++ `make test` — C++ stays the push-gate, pytest is dev-only until cantools mandatory.

New cap-confirmed signal → add to `vwegolf.dbc` AND add assertion to `test_dbc_decode.py` citing `.md` section in docstring.

## Style

Google, 4sp, 100col. `.clang-format` in component dir.
```bash
clang-format -i vehicle/OVMS.V3/components/vehicle_vwegolf/src/*.{cpp,h}
```

## Naming

| Element | Pattern | Ex |
|---|---|---|
| Vehicle ID | 2+ UPPER | `VWEG` |
| Log tag | `v-` lower-dash | `v-vwegolf` |
| Cfg/metric prefix | `x` + 2+ lower | `xvg` |
| CLI ns | = prefix | `xvg` |
| Cfg instance | all lower | `cc-temp` |
| CLI subcmd | all lower | `xvg climate` |

## Workflow

Per fix/feat: branch → test → build → OTA flash to-the-module → verify on car → upstream PR. One thing/branch.

- `master` — tracks upstream, no direct commits. Keep synced: `git fetch <upstream> master:master`, push to `origin` + `caps`
- `vwegolf` — active dev branch. All changes land here first, flashed + verified on the car
- `fix/*` `feat/*` — one PR each, branched **off `master`**, never off `vwegolf`

**Branch model — PR branches come off master, not vwegolf:**

`vwegolf` carries car-verified work but also 140+ commits upstream never sees. A PR branched off it drags all of them in. So per PR: branch fresh off current `master`, then cherry-pick (or rewrite) just that change out of `vwegolf`. Verify with `git diff --stat master...HEAD` — only the intended files, nothing else.

Ignoring this closed + refiled #1459/#1460. Stale PR branches also drift: #1403/#1404 sat on a 15-commit-old master and needed a rebase before merge.

Hooks: `fix/*`/`feat/*` push blocked unless native tests pass. Bypass `--no-verify` emergencies only.

Upstream PRs: small focused per `PROJECT.md` order. Each: pass native tests · ref RE notes in `…/vehicle_vwegolf/docs/` · EN strings · single tag `v-vwegolf` · NO metric defaults in ctor.

## Maintainer review (dexterbg PR #1327)

Violation = PR returned.

- **State:** no static module-lvl vars — all in class members
- **Metrics:** never set default in ctor (breaks persistent metrics). Set only on real decode
- **Cfg:** instance names all lower (case-sens), self-doc, every param in user guide, no unused
- **Cmds:** CLI ns = cfg prefix (`xvg`), all lower, EN only — no German user-visible
- **Log:** single tag `v-vwegolf`, dashes not underscores
- **Stubs:** OK if commented w/ plan. No silent dead code
- **User guide:** every merged module → `docs/index.rst` w/ HW wiring, J533, all params/cmds
