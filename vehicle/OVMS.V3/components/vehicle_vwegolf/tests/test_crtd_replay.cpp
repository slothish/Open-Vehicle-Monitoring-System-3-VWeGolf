// test_crtd_replay.cpp — Feed a real CRTD capture through the decode pipeline.
//
// Replays candumps/kcan-capture.crtd (relative to the tests/ directory) if
// present — a developer-local real capture, gitignored — falling back to the
// committed synthetic fixture (candumps/kcan-synthetic.crtd) so the suite is
// still deterministic in CI without it. Set VWEGOLF_CRTD=<path> to override
// with a specific capture. Builds CAN_frame_t objects from each data line and
// dispatches them to the vehicle module exactly as the OVMS runtime would.
// After the replay we check that the metrics hold the expected values.

#include "mock/mock_ovms.hpp"
#include "../src/vehicle_vwegolf.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

// Declared in test_can_decode.cpp
extern int tests_run;
extern int tests_passed;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else       { printf("  FAIL: %s\n", msg); } \
} while(0)

// Catches "set once, never updates" regressions — the stuck-range_est class.
// A pure value check passes when a metric was written once at boot from a
// persistent store, even if the decoder never ran again. Asserting a write
// floor against the frame count exposes the gap.
//
// The floor is `ratio * count(id)`, not a magic constant: `id` is the
// driving CAN ID named at the call site (so a reader doesn't need to chase
// a comment to know which frame feeds which metric), and `counts` is the
// per-message-ID dispatch tally `replay_crtd()` fills in for the fixture
// actually loaded — so the floor scales with whatever capture is in hand.
// frames(id)==0 means the driving ID never showed up in the replayed
// fixture at all — floor = ratio * 0 = 0, so any write count (including 0)
// would otherwise pass vacuously. That is not "the decoder didn't run", it
// is "the fixture doesn't exercise this assertion" — fail loudly and name
// the missing ID so a reader doesn't go chasing the decoder for nothing.
#define CHECK_WRITES_PROPORTIONAL(name, id, counts, ratio, msg) do { \
    tests_run++; \
    int w = g_metrics.write_count(name); \
    int frames = (counts).count(id) ? (counts).at(id) : 0; \
    if (frames == 0) { \
        printf("  FAIL: %s (fixture has ZERO frames of 0x%03X — assertion" \
               " has nothing to test, not a decoder result)\n", \
               msg, (unsigned)(id)); \
    } else { \
        int floor = (int)((ratio) * frames); \
        if (w >= floor) { \
            tests_passed++; \
            printf("  PASS: %s (writes=%d, frames(0x%03X)=%d, floor=%d)\n", \
                   msg, w, (unsigned)(id), frames, floor); \
        } else { \
            printf("  FAIL: %s (writes=%d, frames(0x%03X)=%d, floor=%d)\n", \
                   msg, w, (unsigned)(id), frames, floor); \
        } \
    } \
} while(0)

static bool near_f(float a, float b, float tol = 0.1f) {
    return std::fabs(a - b) < tol;
}

// ---------------------------------------------------------------------------
// CRTD parser
// ---------------------------------------------------------------------------

// Returns the number of frames dispatched, or -1 on file-open failure.
// `id_counts` is filled in with a per-message-ID dispatch tally, so callers
// can size write-floor assertions off the actual fixture rather than a
// constant baked in from one offline capture.
static int replay_crtd(OvmsVehicleVWeGolf* v, const char* path,
                        std::map<uint32_t, int>& id_counts) {
    FILE* f = fopen(path, "r");
    if (!f) return -1;

    char line[512];
    int dispatched = 0;

    while (fgets(line, sizeof(line), f)) {
        // Strip trailing newline
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';

        // Fields: <timestamp> <type> [<id> <b0> <b1> ...]
        char ts[32], rtype[16];
        if (sscanf(line, "%31s %15s", ts, rtype) < 2) continue;

        // Only handle standard and extended receive frames.
        // Bus number is the first character of the type field (1=can1, 2=can2, 3=can3).
        int bus = rtype[0] - '0';
        bool is_11bit = (rtype[1] == 'R' && rtype[2] == '1' && rtype[3] == '1');
        bool is_29bit = (rtype[1] == 'R' && rtype[2] == '2' && rtype[3] == '9');
        if (!is_11bit && !is_29bit) continue;

        // Parse the rest of the line: <id_hex> <b0> <b1> ...
        const char* p = line;
        // skip timestamp
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
        // skip type
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;

        // parse ID
        char id_str[16];
        if (sscanf(p, "%15s", id_str) < 1) continue;
        uint32_t msg_id = (uint32_t)strtoul(id_str, nullptr, 16);

        // advance past ID field
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;

        // parse up to 8 data bytes
        CAN_frame_t frame{};
        frame.MsgID = msg_id;
        frame.FIR.B.FF = is_29bit ? CAN_frame_ext : CAN_frame_std;
        uint8_t dlc = 0;
        while (dlc < 8 && *p) {
            char byte_str[4];
            if (sscanf(p, "%3s", byte_str) < 1) break;
            frame.data.u8[dlc++] = (uint8_t)strtoul(byte_str, nullptr, 16);
            while (*p && *p != ' ') p++;
            while (*p == ' ') p++;
        }
        frame.FIR.B.DLC = dlc;

        // Dispatch by the CRTD bus tag. Captures pre-dating the bus-filter
        // fix could mis-tag KCAN frames as bus 2; those captures are now
        // purged (commit 0b9fd9211). New captures tag each bus correctly,
        // so IncomingFrameCanN matches the physical bus the firmware sees.
        if (bus == 2)      v->IncomingFrameCan2(&frame);
        else if (bus == 3) v->IncomingFrameCan3(&frame);
        else               continue;
        dispatched++;
        id_counts[msg_id]++;
    }

    fclose(f);
    return dispatched;
}

// ---------------------------------------------------------------------------
// Test
// ---------------------------------------------------------------------------

void test_crtd_replay() {
    printf("\ntest_crtd_replay (FCAN-tagged capture, exercises IncomingFrameCan2)\n");

    g_metrics = MetricStore{};
    auto* v = new OvmsVehicleVWeGolf();

    // kcan-capture.crtd dates from before commit 9fdbf8b7 fixed the
    // bus-filter bug, so every frame is bus-tagged 2 regardless of physical
    // origin. The replay dispatches it through IncomingFrameCan2, which
    // exercises only the FCAN-side decoders. KCAN-routed metrics (range,
    // speed, charging, doors, cabin temp) are covered by the second
    // replay below against a properly-tagged bus-3 capture.
    //
    // Candidate order is deliberate: an explicit VWEGOLF_CRTD override first,
    // then the developer-local real capture above (gitignored, may be absent),
    // then the committed synthetic fixture as a deterministic backstop — so
    // the suite still runs the same way in CI as it does with the real capture
    // on hand.
    const char* env_path = getenv("VWEGOLF_CRTD");
    const char* candidates[] = {
        env_path ? env_path : "candumps/kcan-capture.crtd",
        "candumps/kcan-capture.crtd",
        "candumps/kcan-synthetic.crtd",
    };
    const char* used_path = nullptr;
    int n = -1;
    std::map<uint32_t, int> counts;
    for (const char* p : candidates) {
        counts.clear();
        n = replay_crtd(v, p, counts);
        if (n >= 0) { used_path = p; break; }
    }

    if (n < 0) {
        printf("  SKIP: candumps/kcan-capture.crtd not found\n");
        delete v;
        return;
    }
    printf("  replayed %d frames from %s\n", n, used_path);

    // --- SoC: 0x131 d[3]=0x79 → 60.5%. Decoded in IncomingFrameCan2
    //     (line 135 — added in commit fc4de583b alongside the can3 decoder).
    float soc = StandardMetrics.ms_v_bat_soc->AsFloat();
    CHECK(near_f(soc, 60.5f, 1.0f), "SoC ~60.5% (d[3]=0x79 from kcan-capture)");

    // --- Gear: 0x187 byte2=0x12, nibble=2 → Park → gear=0 ---
    int gear = StandardMetrics.ms_v_env_gear->AsValue();
    CHECK(gear == 0, "Gear = 0 (Park)");

    // --- Write floors for FCAN-side decoders. Asserting the metric was
    //     written at least a set fraction of the frames-in-capture (per-ID
    //     count, ratio at each call site below) catches the "set once at
    //     boot, decoder never fires again" class of bug. A pure value
    //     check passes when a persistent metric is restored to its
    //     last-known value at boot — the stuck-range_est class.

    // 0x131 SoC: IncomingFrameCan2 case 0x0131 discards d[3]==0xFE (BMS
    // "not ready" sentinel). This capture happens to carry none, but the
    // ratio must hold for any capture, including ones that do — 0.8, not 1.0.
    CHECK_WRITES_PROPORTIONAL("ms_v_bat_soc", 0x131, counts, 0.8,
                              "SoC decoded ~every 0x131 frame");
    // 0x187 gear: the decoder only writes for nibble in {2..6} (P/R/N/D/B);
    // any other nibble value is silently dropped — same class of gap as a
    // sentinel filter, so 0.8 rather than assuming every frame decodes.
    CHECK_WRITES_PROPORTIONAL("ms_v_env_gear", 0x187, counts, 0.8,
                              "Gear decoded ~every 0x187 frame");

    delete v;
}

// ---------------------------------------------------------------------------
// Second replay — a properly-tagged bus-3 KCAN capture. Exercises
// IncomingFrameCan3 and the KCAN-side decoders. Catches the stuck-update
// class of bug (range_est, speed, charging frames, cabin temp, doors)
// that the FCAN-tagged replay can't reach.
// ---------------------------------------------------------------------------

void test_crtd_replay_kcan() {
    printf("\ntest_crtd_replay_kcan (bus-3 tagged, exercises IncomingFrameCan3)\n");

    g_metrics = MetricStore{};
    auto* v = new OvmsVehicleVWeGolf();

    // 20260428-095821: 23.5 s clean KCAN capture, 182 IDs, every frame
    // tagged bus 3. Spans bus-up, frames flow into IncomingFrameCan3.
    //
    // Candidate order mirrors test_crtd_replay above: an explicit
    // VWEGOLF_CRTD_KCAN override first, then the developer-local real
    // capture (gitignored, may be absent on a fresh clone), then the
    // committed synthetic fixture — generated from docs/vwegolf.dbc — as a
    // deterministic backstop, so the suite still runs the same way in CI
    // as it does with the real capture on hand.
    const char* env_path = getenv("VWEGOLF_CRTD_KCAN");
    const char* candidates[] = {
        env_path ? env_path : "candumps/can3-1aec82f33_ota_0_edge-20260428-095821.crtd",
        "candumps/can3-1aec82f33_ota_0_edge-20260428-095821.crtd",
        "candumps/can3-synthetic.crtd",
    };
    const char* used_path = nullptr;
    int n = -1;
    std::map<uint32_t, int> counts;
    for (const char* p : candidates) {
        counts.clear();
        n = replay_crtd(v, p, counts);
        if (n >= 0) { used_path = p; break; }
    }

    if (n < 0) {
        printf("  SKIP: can3-1aec82f33_ota_0_edge-20260428-095821.crtd not found\n");
        delete v;
        return;
    }
    printf("  replayed %d frames from %s\n", n, used_path);

    // --- Plausibility on a couple of KCAN-sourced metrics. Range_est is
    //     the specific bug class this test exists to surface ---
    float range_est = StandardMetrics.ms_v_bat_range_est->AsFloat();
    CHECK(range_est > 0.0f && range_est < 500.0f,
          "range_est plausible (0..500 km)");

    float speed = StandardMetrics.ms_v_pos_speed->AsFloat();
    CHECK(speed >= 0.0f && speed < 250.0f,
          "speed plausible (0..250 km/h)");

    // --- Write floors for KCAN-side decoders (per-ID count and ratio
    //     justified at each call site below).

    // 0x0FD speed: IncomingFrameCan3 case 0x00FD writes unconditionally,
    // no sentinel/range filter — 1.0 holds.
    CHECK_WRITES_PROPORTIONAL("ms_v_pos_speed", 0x0FD, counts, 1.0,
                              "Speed decoded ~every 0x0FD frame");
    // 0x5F5 range_est: case 0x05F5 writes range_est unconditionally every
    // frame, no filtering — 1.0 holds.
    CHECK_WRITES_PROPORTIONAL("ms_v_bat_range_est", 0x5F5, counts, 1.0,
                              "range_est decoded ~every 0x5F5 frame "
                              "(the stuck-at-boot regression class)");
    // 0x65A hood: case 0x065A writes unconditionally, no filtering — 1.0 holds.
    CHECK_WRITES_PROPORTIONAL("ms_v_door_hood", 0x65A, counts, 1.0,
                              "hood door decoded ~every 0x65A frame");
    // 0x6B7 outside temp: case 0x06B7 writes odo/parktime/temp unconditionally
    // every frame, no filtering — 1.0 holds.
    CHECK_WRITES_PROPORTIONAL("ms_v_env_temp", 0x6B7, counts, 1.0,
                              "outside temp decoded ~every 0x6B7 frame");

    delete v;
}
