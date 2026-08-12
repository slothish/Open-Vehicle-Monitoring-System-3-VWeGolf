"""Regression tests: BAP confirmation-signal semantics (WI-CLIMA-CONF-1).

Pins the claims in `docs/clima-control-bap.md` under "Confirmation-Signal
Semantics" against the whole real capture corpus (`tests/candumps/*.crtd`
symlinks; the two committed `*-synthetic.crtd` fixtures are excluded — they
model DBC signal decode, not BAP transaction pairing).

Deliberately does NOT use `crtd.py`'s `load()`/`iter_frames()`: those only
recognize `NRxx` (RX) record types (see `crtd.py` module docstring / the
`capture-analysis` skill, rule 7) and silently drop OVMS's own `NTxx` (TX)
lines and `NCER TX_Fail` records. A BAP confirmation is a TX/RX pair — the
command is a TX — so this file parses the raw `.crtd` lines directly.

Run with:
    .venv/bin/pytest vehicle/OVMS.V3/components/vehicle_vwegolf/tests/analysis/
"""

from __future__ import annotations

import glob
import os
import re

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
CANDUMPS = os.path.normpath(os.path.join(HERE, "..", "candumps"))

ID_CMD = "17332501"   # ASG -> FSG (our commands), also carries other ASGs' traffic
ID_STA = "17332510"   # FSG -> ASG (status/ACK), shared by all ASGs

# Correlation window for "was this reply preceded by our own command".
# Measured start-from-off latency tops out at 5.38s across the corpus
# (see docs/clima-control-bap.md); 25s leaves ample margin without being
# so wide it would paper over a real regression.
CORRELATION_WINDOW_S = 25.0

# Measured 49 59 TID-matched latency across the corpus is 0.023-2.612s
# (docs/clima-control-bap.md). Bound with headroom, not the raw max, so a
# legitimately slower FSG reply on a future capture doesn't false-fail.
LATENCY_49_59_MAX_S = 5.0

_R_TYPE = re.compile(r"^(\d)R(11|29)$")
_T_TYPE = re.compile(r"^(\d)T(11|29)$")
_TXFAIL_WTYPE = re.compile(r"^T(11|29)$")


def _parse_raw(path):
    """Yield (t_unix, kind, id_hex, data) for RX / TX / TX_Fail records.

    kind: 'RX' | 'TX' | 'TXFAIL'. Bus number is not tracked — the two BAP
    CAN ids are unique across buses in this corpus, and matching on id_hex
    alone also tolerates the known mislabeled-bus captures (see
    tests/analysis/README.md) without needing per-file bus overrides.
    """
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 3:
                continue
            typ = parts[1]
            m = _R_TYPE.match(typ)
            if m:
                try:
                    ts = float(parts[0])
                    data = [int(x, 16) for x in parts[3:11]]
                except ValueError:
                    continue
                yield (ts, "RX", parts[2].upper(), data)
                continue
            m = _T_TYPE.match(typ)
            if m:
                try:
                    ts = float(parts[0])
                    data = [int(x, 16) for x in parts[3:11]]
                except ValueError:
                    continue
                yield (ts, "TX", parts[2].upper(), data)
                continue
            if typ == "3CER" or (len(typ) == 4 and typ.endswith("CER")):
                if len(parts) >= 5 and parts[2] == "TX_Fail":
                    wtyp = parts[3]
                    if _TXFAIL_WTYPE.match(wtyp):
                        try:
                            ts = float(parts[0])
                            data = [int(x, 16) for x in parts[5:13]]
                        except ValueError:
                            continue
                        yield (ts, "TXFAIL", parts[4].upper(), data)
                continue


def _real_captures():
    """The symlinked real corpus (excludes the 2 committed synthetic fixtures)."""
    return sorted(p for p in glob.glob(os.path.join(CANDUMPS, "*.crtd"))
                  if os.path.islink(p))


@pytest.fixture(scope="module")
def corpus_events():
    """{path: [(t, kind, id_hex, data), ...] sorted by t} for every real capture."""
    caps = _real_captures()
    assert len(caps) >= 20, (
        f"expected the real capture corpus to be symlinked into tests/candumps/, "
        f"found only {len(caps)} symlinked .crtd files — corpus missing?")
    return {p: sorted(_parse_raw(p), key=lambda e: e[0]) for p in caps}


# ---------------------------------------------------------------------------
# (a) every `49 58` reply on 0x17332510 is a reply to our own `29 58 00`
# ---------------------------------------------------------------------------

def test_4958_always_command_correlated(corpus_events):
    """Every `49 58 <flag>` short frame traces to a preceding own `29 58 00
    <flag>` (TX or TX_Fail) within CORRELATION_WINDOW_S. Never spontaneous.

    docs/clima-control-bap.md "(a)" — 27 occurrences across 9 captures,
    all correlated, measured max latency 5.380s.
    """
    total = 0
    for path, evs in corpus_events.items():
        cmds = [(t, data) for t, kind, idh, data in evs
                if kind in ("TX", "TXFAIL") and idh == ID_CMD and len(data) >= 4
                and data[0] == 0x29 and data[1] == 0x58 and data[2] == 0x00]
        replies = [(t, data) for t, kind, idh, data in evs
                   if kind == "RX" and idh == ID_STA and len(data) >= 3
                   and data[0] == 0x49 and data[1] == 0x58]
        for t_r, data_r in replies:
            total += 1
            preceding = [t_c for t_c, _ in cmds if t_c <= t_r]
            assert preceding, (
                f"{os.path.basename(path)} t={t_r:.3f}: 49 58 reply with "
                f"no preceding 29 58 command at all")
            dt = t_r - max(preceding)
            assert dt <= CORRELATION_WINDOW_S, (
                f"{os.path.basename(path)} t={t_r:.3f}: nearest preceding "
                f"29 58 is {dt:.1f}s away (window={CORRELATION_WINDOW_S}s)")
    assert total >= 20, f"expected the corpus to contain multiple 49 58 replies, found {total}"


# ---------------------------------------------------------------------------
# (c) TID-matched `49 59` latency + TXFAIL never gets a matched reply
# ---------------------------------------------------------------------------

def test_4959_tid_matched_latency_bound(corpus_events):
    """Every TID-matched `49 59` (TID = ours | 0x80) reply latency is within
    LATENCY_49_59_MAX_S, and a burst whose transmit itself failed (TX_Fail)
    never gets a matched reply.

    docs/clima-control-bap.md "(c)" — measured range 0.023-2.612s over 23
    matched bursts (out of 29 electrically-successful sends); 4 TX_Fail
    bursts, 0 matched.
    """
    n_matched = 0
    n_txfail_matched = 0
    for path, evs in corpus_events.items():
        starts = [(t, kind, data) for t, kind, idh, data in evs
                  if idh == ID_CMD and len(data) >= 5
                  and data[0] == 0x80 and data[1] == 0x08
                  and data[2] == 0x29 and data[3] == 0x59]
        replies = [(t, data) for t, kind, idh, data in evs
                   if kind == "RX" and idh == ID_STA and len(data) >= 5
                   and data[0] == 0x80 and data[1] == 0x0A
                   and data[2] == 0x49 and data[3] == 0x59]
        for t_s, kind, data_s in starts:
            want = (data_s[4] | 0x80) & 0xFF
            cands = [t_r for t_r, data_r in replies
                     if data_r[4] == want and t_r >= t_s
                     and (t_r - t_s) <= CORRELATION_WINDOW_S]
            if not cands:
                continue
            dt = min(cands) - t_s
            if kind == "TXFAIL":
                n_txfail_matched += 1
                continue  # counted below as a hard failure
            n_matched += 1
            assert 0.0 < dt <= LATENCY_49_59_MAX_S, (
                f"{os.path.basename(path)} t={t_s:.3f} tid=0x{data_s[4]:02x}: "
                f"49 59 latency {dt:.3f}s outside (0, {LATENCY_49_59_MAX_S}]s")
    assert n_txfail_matched == 0, (
        f"{n_txfail_matched} TX_Fail burst(s) matched a 49 59 reply — a frame "
        f"that never reached the bus cannot have been confirmed")
    assert n_matched >= 10, f"expected multiple TID-matched 49 59 replies, found {n_matched}"


# ---------------------------------------------------------------------------
# (d) zero BAP opcode-7 (Error) elements anywhere in the corpus
# ---------------------------------------------------------------------------

def test_no_opcode7_error_elements(corpus_events):
    """No BAP opcode-7 (Error) element on 0x17332501 or 0x17332510, in
    either short-message or long-message-start form, anywhere in the corpus.

    Short message: byte0 bit7=0, opcode=bits[6:4] -> byte0 in 0x70..0x7F.
    Long message start: byte0 top nibble in {0x8,0x9,0xA,0xB}, index 0;
    the opcode/lsg header is byte2, same 0x70..0x7F test.
    See docs/vw-bap-protocol.md "Frame Encoding" / "OpCodes".

    docs/clima-control-bap.md "(d)" — 0 found across the 40-file corpus.
    """
    hits = []
    for path, evs in corpus_events.items():
        for t, kind, idh, data in evs:
            if idh not in (ID_CMD, ID_STA) or not data:
                continue
            b0 = data[0]
            if (b0 & 0x80) == 0 and (b0 & 0x70) == 0x70:
                hits.append((path, t, "short", data))
                continue
            if (b0 & 0xF0) in (0x80, 0x90, 0xA0, 0xB0) and (b0 & 0x0F) == 0 and len(data) >= 3:
                b2 = data[2]
                if (b2 & 0x80) == 0 and (b2 & 0x70) == 0x70:
                    hits.append((path, t, "long-start", data))
    assert not hits, f"opcode-7 (Error) element(s) found: {hits}"
