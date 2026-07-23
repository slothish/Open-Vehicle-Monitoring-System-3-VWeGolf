#!/usr/bin/env python
"""make_synthetic_crtd.py — DBC-driven generator for the committed CRTD fixtures.

Regenerates:
  tests/candumps/kcan-synthetic.crtd  (bus 2 only — feeds test_crtd_replay,
                                        the FCAN-tagged replay. Despite the
                                        "kcan-" name this is the FCAN/bus-2
                                        fixture; do not rename the file, see
                                        tests/test_crtd_replay.cpp.)
  tests/candumps/can3-synthetic.crtd  (bus 3 only — feeds
                                        test_crtd_replay_kcan, the KCAN replay)

Every frame is packed straight from `docs/vwegolf.dbc` signal metadata
(start bit, length, scale, offset) via `crtd.load_dbc()` — no CAN bit
position, bit length, scale, or offset appears as a literal here. Only
frame IDs, signal names, physical setpoints, and periods do.

cantools' `Message.encode()` cannot be used for every message on this DBC:
ChargeType/ChargePort double-claim bits 42-43 on 0x594 (documented in the
DBC CM_ BO_ 1428 comment). `pack_frame()` below hand-packs from signal
metadata instead, and asserts the requested signals' bit masks are
disjoint before writing — so a DBC overlap defect fails loudly here
instead of silently producing a corrupt fixture. (0x5F5's former
RangeIdeal bit-8-10 overlap was fixed in WI-DBC-1, 2026-07-23 — the
other signal there was a disproven ghost, see DBC CM_ BO_ 1525 — so the
self-test below now exercises the 0x594 alias instead.)

Run (from tests/analysis/, with the project .venv active — needs cantools):
    ../../../../.venv/bin/python make_synthetic_crtd.py
or via the Makefile target:
    make -C .. fixtures

Deterministic: re-running produces byte-identical output (fixed epoch base,
no wall-clock or RNG input) — `git status` shows no diff on a second run.
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from crtd import load_dbc  # noqa: E402

CANDUMPS_DIR = os.path.normpath(os.path.join(HERE, "..", "candumps"))

# Fixed fictional epoch — never derived from wall-clock, so re-runs are
# byte-identical. 2024-01-15 12:00:00 UTC, arbitrary.
BASE_TS = 1705316400.0

# PII guard: frame IDs that must never appear in a generated fixture.
# 0x6B4 = VIN broadcast. 0x486 = GPS position — no assert in
# test_crtd_replay.cpp needs it, and test_can_decode.cpp already unit-covers
# that decoder directly, so omitting it here removes the "did someone paste
# real coordinates" review surface entirely.
DENYLIST_FRAME_IDS = {0x6B4, 0x486}


class OverlapError(AssertionError):
    """Raised when two requested signals in the same frame share a bit."""


def pack_frame(dbc, frame_id: int, signal_values: dict) -> bytes:
    """Hand-pack `signal_values` (name -> physical value) into a data frame.

    Reads start bit, length, scale, and offset from the DBC message's signal
    metadata — the caller supplies only signal names and physical setpoints.
    Asserts the requested signals occupy disjoint bit ranges; a defect like
    the 0x594 ChargeType/ChargePort overlap raises `OverlapError` instead of
    silently producing a corrupt frame.

    Only little-endian (Intel, "@1") unsigned signals are supported — every
    signal used by the fixtures below is one of those.
    """
    assert frame_id not in DENYLIST_FRAME_IDS, f"frame 0x{frame_id:X} is denylisted (PII)"

    msg = dbc.get_message_by_frame_id(frame_id)
    used_mask = 0
    frame_int = 0

    for name, phys in signal_values.items():
        sig = msg.get_signal_by_name(name)
        assert sig.byte_order == "little_endian", (
            f"{msg.name}.{name}: only little-endian signals are supported by this generator"
        )
        assert not sig.is_signed, (
            f"{msg.name}.{name}: only unsigned signals are supported by this generator"
        )

        raw = round((phys - sig.offset) / sig.scale)
        max_raw = (1 << sig.length) - 1
        assert 0 <= raw <= max_raw, (
            f"{msg.name}.{name}: physical value {phys} -> raw {raw} out of range [0, {max_raw}]"
        )

        sig_mask = ((1 << sig.length) - 1) << sig.start
        if used_mask & sig_mask:
            raise OverlapError(
                f"{msg.name}: signal {name!r} (bits {sig.start}..{sig.start + sig.length - 1}) "
                f"overlaps a bit range already packed in this frame "
                f"(used_mask=0x{used_mask:016X}, sig_mask=0x{sig_mask:016X}) — "
                f"this is a DBC signal-overlap defect, not a generator bug"
            )
        used_mask |= sig_mask
        frame_int |= (raw << sig.start) & sig_mask

    dlc = msg.length
    return bytes((frame_int >> (i * 8)) & 0xFF for i in range(dlc))


def _self_test_overlap_detection(dbc) -> None:
    """Prove the disjointness assert actually fires on the known 0x594 alias.

    ChargeType (42|2) and ChargePort (42|2) share the same bits on
    ChargeManagement (0x594) — a genuine, still-live same-bits case
    documented in the DBC CM_ BO_ 1428 comment (out of scope for WI-DBC-1;
    do not touch it here). Supplying both must raise OverlapError. Runs on
    every invocation so the guarantee never silently rots.

    (Formerly exercised the 0x5F5 RangeIdeal bit-8-10 overlap — fixed in
    WI-DBC-1, 2026-07-23: the other signal there was a disproven ghost,
    see DBC CM_ BO_ 1525. Repointed here since RangeIdeal/RangeEst no
    longer overlap anything.)
    """
    try:
        pack_frame(dbc, 0x594, {"ChargeType": 1, "ChargePort": 1})
    except OverlapError as e:
        print(f"[self-test] disjointness assert fired as expected: {e}")
        return
    raise AssertionError(
        "0x594 ChargeType/ChargePort overlap did NOT raise — "
        "either the DBC overlap was fixed (update this self-test) or the "
        "disjointness check regressed (do not silently drop it)"
    )


def _crtd_line(ts: float, bus: int, frame_id: int, data: bytes) -> str:
    hex_id = f"{frame_id:03X}" if frame_id <= 0x7FF else f"{frame_id:08X}"
    byte_str = " ".join(f"{b:02x}" for b in data)
    return f"{ts:.6f} {bus}R11 {hex_id} {byte_str}"


def _schedule(frame_id: int, values: dict, count: int, period: float,
              phase: float = 0.0) -> list[tuple[float, int, dict]]:
    """N identical frames of one ID, spaced `period` seconds apart from BASE_TS+phase."""
    return [(BASE_TS + phase + i * period, frame_id, values) for i in range(count)]


def _build_fixture_a(dbc) -> list[str]:
    """Bus 2 (FCAN) fixture feeding test_crtd_replay(). Driving IDs asserted
    there: 0x131 (SoC, 0.8 ratio) and 0x187 (gear, 0.8 ratio) — both must be
    present. 0x191 is bonus coverage only (no assert in test_crtd_replay.cpp
    reads it), included to prove the d[2]!=0xFF sentinel guard per the
    generator's PII/sentinel review requirements.
    """
    events: list[tuple[float, int, dict]] = []

    # 0x131 BMS_SoC — SoC=60.5% every frame (d[3]=0x79, matches the test's
    # own comment "SoC: 0x131 d[3]=0x79 -> 60.5%"). No 0xFE sentinel ever.
    events += _schedule(0x131, {"SoC": 60.5}, count=20, period=0.5, phase=0.05)

    # 0x187 GearSelector — nibble=2 (Park) every frame -> ms_v_env_gear=0.
    events += _schedule(0x187, {"GearPosition": 2}, count=20, period=0.5, phase=0.20)

    # 0x191 BMS_PowerBus — plausible current/voltage, d[2] far from the 0xFF
    # startup sentinel.
    events += _schedule(0x191, {"BatCurrent": 10.0, "BatVoltage": 380.0},
                        count=10, period=1.0, phase=0.35)

    lines = []
    for ts, frame_id, values in sorted(events, key=lambda e: e[0]):
        data = pack_frame(dbc, frame_id, values)
        lines.append(_crtd_line(ts, 2, frame_id, data))
    return lines


def _build_fixture_b(dbc) -> list[str]:
    """Bus 3 (KCAN) fixture feeding test_crtd_replay_kcan(). Driving IDs
    asserted there (all 1.0 ratio): 0x0FD, 0x5F5, 0x65A, 0x6B7 — all four
    must be present. 0x5EA and 0x594 are bonus coverage (no assert reads
    them), included to prove the ClimaCabinTemp sentinel guard and the
    ChargeType/ChargePort overlap avoidance called out in the work item.
    """
    events: list[tuple[float, int, dict]] = []

    # 0x0FD ESP_Speed — plausible, well under the 250 km/h ceiling.
    events += _schedule(0x0FD, {"Speed": 42.5}, count=20, period=0.5, phase=0.05)

    # 0x5F5 Instruments_Range — RangeEst + RangeIdeal, both 11-bit, disjoint.
    events += _schedule(0x5F5, {"RangeEst": 120, "RangeIdeal": 90},
                        count=20, period=0.5, phase=0.20)

    # 0x65A BCM_01 — hood closed.
    events += _schedule(0x65A, {"HoodOpen": 0}, count=20, period=0.5, phase=0.35)

    # 0x6B7 OdoTempPark — Odometer + OutsideTemp. ParkTime intentionally
    # left unset: its DBC bit layout is not fully confirmed (see the DBC
    # CM_ SG_ 1719 comment) and no test asserts on it.
    events += _schedule(0x6B7, {"Odometer": 12345, "OutsideTemp": 15.0},
                        count=20, period=0.5, phase=0.45)

    # 0x5EA ClimaECU_Status — ClimaCabinTemp well under the 1020 raw
    # "not available" sentinel (20.0 degC -> raw 600).
    events += _schedule(0x5EA, {"ClimaCabinTemp": 20.0}, count=10, period=1.0, phase=0.60)

    # 0x594 ChargeManagement — ChargeType ONLY (AC_Type2). Do NOT add
    # ChargePort here: it double-claims bits 42-43 with ChargeType (see the
    # DBC CM_ BO_ 1428 comment — this is the documented, not new, overlap).
    events += _schedule(0x594, {"ChargeType": 1}, count=10, period=1.0, phase=0.75)

    lines = []
    for ts, frame_id, values in sorted(events, key=lambda e: e[0]):
        data = pack_frame(dbc, frame_id, values)
        lines.append(_crtd_line(ts, 3, frame_id, data))
    return lines


def _write_fixture(path: str, header: list[str], lines: list[str]) -> None:
    with open(path, "w") as f:
        for h in header:
            f.write(f"# {h}\n" if h else "#\n")
        for line in lines:
            f.write(line + "\n")


def main() -> None:
    dbc = load_dbc()

    _self_test_overlap_detection(dbc)

    fixture_a = _build_fixture_a(dbc)
    fixture_b = _build_fixture_b(dbc)

    for frame_id in DENYLIST_FRAME_IDS:
        hex_id = f"{frame_id:03X}" if frame_id <= 0x7FF else f"{frame_id:08X}"
        for line in fixture_a + fixture_b:
            assert f" {hex_id} " not in line, f"denylisted frame 0x{hex_id} leaked into fixture"

    a_path = os.path.join(CANDUMPS_DIR, "kcan-synthetic.crtd")
    _write_fixture(a_path, [
        "kcan-synthetic.crtd -- synthetic FCAN (bus 2) fixture.",
        "Despite the 'kcan-' name this is the FCAN/bus-2 fixture feeding",
        "test_crtd_replay() in tests/test_crtd_replay.cpp (the file predates",
        "the bus-3 fixture and keeps its name to avoid disturbing that",
        "test's existing candidate-path chain). Do not rename.",
        "",
        "Synthetic. No real vehicle data. Generated by",
        "tests/analysis/make_synthetic_crtd.py -- re-run to regenerate.",
        "",
        "Bus 2 (FCAN) only. Frame IDs: 0x131 (SoC), 0x187 (gear), 0x191",
        "(battery current/voltage, bonus sentinel coverage).",
    ], fixture_a)

    b_path = os.path.join(CANDUMPS_DIR, "can3-synthetic.crtd")
    _write_fixture(b_path, [
        "can3-synthetic.crtd -- synthetic KCAN (bus 3) fixture, feeding",
        "test_crtd_replay_kcan() in tests/test_crtd_replay.cpp.",
        "",
        "Synthetic. No real vehicle data. Generated by",
        "tests/analysis/make_synthetic_crtd.py -- re-run to regenerate.",
        "",
        "Bus 3 (KCAN) only. Frame IDs: 0x0FD (speed), 0x5F5 (range),",
        "0x65A (hood), 0x6B7 (odo/parktime/outside temp), 0x5EA (clima",
        "cabin temp, bonus), 0x594 (charge type, bonus).",
    ], fixture_b)

    print(f"wrote {a_path} ({len(fixture_a)} frames, {os.path.getsize(a_path)} bytes)")
    print(f"wrote {b_path} ({len(fixture_b)} frames, {os.path.getsize(b_path)} bytes)")


if __name__ == "__main__":
    main()
