/*
;    Project:       Open Vehicle Monitor System
;	 Subproject:    Integrate VW e-Golf
;
;    Changes:
;    February 7 2026: Initial Implementation
;
;    (C) 2026  Erick Fuentes <fuentes.erick@gmail.com>
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "vehicle_vwegolf.h"

#include "mcp2515.h"

#undef TAG
#define TAG "v-vwegolf"

// UDS BMS poll addresses/DIDs — VW ECU 8C "hybrid battery management". Taken from
// vehicle_vweup (vweup_obd.h, same VW EV platform family); MUST be validated on the
// e-Golf before trusting values. Polled on can1 = OBD/diag CAN at the J533 (gateway
// routes the request to the BMS on FCAN); wiring per docs/j533_to_ovms.svg.
#define VWEGOLF_BMS_TX 0x7E5      // tester -> BMS request CAN ID
#define VWEGOLF_BMS_RX 0x7ED      // BMS -> tester response CAN ID
#define VWEGOLF_BMS_DID_U 0x1E3B  // measured pack voltage, uint16 / 4.0 V
#define VWEGOLF_BMS_DID_I 0x1E3D  // measured pack current, (uint16 - 2044) / 4.0 A
// SoH — same DID, TX/RX as vweup_obd.cpp VWUP_BAT_MGMT_SOH_CAC, but NOT vweup's scaling:
// e-Golf raw runs 10x the pre-2020 vweup scale (on-car 2026-07-10: raw 43177, vweup /50
// would give 863%). Divisor 500 is provisional; CAC scaling unknown, not decoded.
#define VWEGOLF_BMS_DID_SOH_CAC 0x74CB  // uint16 @offset0: SoH = raw/500.0 %

// Poll intervals in seconds per state {OFF, AWAKE, CHARGING, ON}. Diag CAN sleeps
// with the car -> OFF/AWAKE are 0. CHARGING feeds the bat_* metrics (0x191 is silent
// during DC and reads 0 A during AC). ON polls slowly and logs only — the on-car
// validation path: compare log values against live 0x191 metrics while driving.
// Voltage entry MUST precede current: the current handler derives power from the
// voltage metric, so voltage has to be fresh within the same poll round.
static const OvmsPoller::poll_pid_t vwegolf_polls[] = {{VWEGOLF_BMS_TX,
                                                        VWEGOLF_BMS_RX,
                                                        VEHICLE_POLL_TYPE_READDATA,
                                                        VWEGOLF_BMS_DID_U,
                                                        {0, 0, 3, 10},
                                                        0,
                                                        ISOTP_STD},
                                                       {VWEGOLF_BMS_TX,
                                                        VWEGOLF_BMS_RX,
                                                        VEHICLE_POLL_TYPE_READDATA,
                                                        VWEGOLF_BMS_DID_I,
                                                        {0, 0, 3, 10},
                                                        0,
                                                        ISOTP_STD},
                                                       // SoH/CAC are slow-changing (capacity
                                                       // fade, not an instantaneous reading) —
                                                       // poll at a slow interval, and in AWAKE/
                                                       // CHARGING/ON alike (unlike U/I above,
                                                       // not gated to CHARGING-only downstream).
                                                       {VWEGOLF_BMS_TX,
                                                        VWEGOLF_BMS_RX,
                                                        VEHICLE_POLL_TYPE_READDATA,
                                                        VWEGOLF_BMS_DID_SOH_CAC,
                                                        {0, 600, 600, 600},
                                                        0,
                                                        ISOTP_STD},
                                                       POLL_LIST_END};

// ---------------------------------------------------------------------------
// Module registration
// ---------------------------------------------------------------------------

OvmsVehicleVWeGolf::OvmsVehicleVWeGolf() {
    ESP_LOGI(TAG, "Start vehicle module: VW e-Golf");

    MyConfig.RegisterParam("xvg", "VW e-Golf", true, true);

    // KCAN (CAN3) carries comfort, body, and clima frames via the J533 gateway.
    // FCAN (CAN2) is the powertrain bus (BMS, motor controller, VIN).
    // CAN1 (OBD) is diagnostic-only and inaccessible while the car is asleep.
    //
    // FCAN is listen-only: we read gear and VIN but never transmit on this bus.
    // Active mode would require the ESP32 CAN controller to ACK every received
    // frame; its ACK timing on a bus already managed by native ECUs produces
    // spurious ECC TX-direction errors (ecc != 0 → CAN_logerror every ~200 ms)
    // even though rxerr/txerr stay at zero. Listen-only eliminates this entirely.
    RegisterCanBus(2, CAN_MODE_LISTEN, CAN_SPEED_500KBPS);  // FCAN — powertrain (read-only)
    RegisterCanBus(3, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);  // KCAN — comfort / clima

    // OBD/diag CAN at the J533 — tester bus, gateway routes UDS to target ECUs.
    // ACTIVE: we must transmit poll requests here (unlike listen-only FCAN).
    RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);  // diag CAN — UDS polling

    // UDS BMS polling (see vwegolf_polls above). Poll state is driven in Ticker1
    // from charge_inprogress (0x594) and bus liveness; starts in OFF (state 0).
    PollSetPidList(m_can1, vwegolf_polls);

    // During charging the OBC floods FCAN at ~860 frames/s which overflows the MCP2515
    // RX buffers causing a continuous ISR storm that starves the Events task → TWDT.
    // Hardware acceptance filter passes only the 5 FCAN IDs we decode; everything else
    // is dropped in hardware before reaching the RX buffers.
    // Disable (cfg "fcan-filter" no / "xvg fcanfilter off") only for frame discovery
    // captures — running unfiltered during charging risks the TWDT storm above.
    SetFcanFilter(MyConfig.GetParamValueBool("xvg", "fcan-filter", true));

    // NOTE: no metric writes in the constructor — that defeats metric persistence and is
    // a maintainer-review blocker. ms_v_env_cabintemp's sentinel (0x05EA raw >= 1020, the
    // ECU's "not available" value) is rejected at the decode site (IncomingFrameCan3 case
    // 0x05EA) so it never reaches the metric in the first place; nothing bad gets
    // persisted to clear at boot.

    OvmsCommand* cmd_xvg = MyCommandApp.RegisterCommand("xvg", "VW e-Golf controls");

    // Diagnostic: manually stop OCU keepalive and leave the NM ring.
    cmd_xvg->RegisterCommand("offline", "Stop sending OCU keepalive (diagnostic)", [this](...) {
        m_ocu_active = false;
        ESP_LOGI(TAG, "OCU keepalive stopped");
    });

    cmd_xvg->RegisterCommand("fold_mirrors", "Fold mirrors in",
                             [this](...) { CommandMirrorFoldIn(); });

    // FCAN hardware acceptance filter toggle. Persists via config "xvg fcan-filter"
    // and applies to the MCP2515 immediately (no module reload needed).
    OvmsCommand* cmd_filter =
        cmd_xvg->RegisterCommand("fcanfilter", "FCAN hardware acceptance filter");
    cmd_filter->RegisterCommand(
        "on", "Enable filter (normal operation)",
        [this](int, OvmsWriter* writer, OvmsCommand*, int, const char* const*) {
            MyConfig.SetParamValueBool("xvg", "fcan-filter", true);
            SetFcanFilter(true);
            writer->puts("FCAN filter enabled (5 decoded IDs only)");
        });
    cmd_filter->RegisterCommand(
        "off", "Disable filter (full-bus capture; ISR storm risk while charging)",
        [this](int, OvmsWriter* writer, OvmsCommand*, int, const char* const*) {
            MyConfig.SetParamValueBool("xvg", "fcan-filter", false);
            SetFcanFilter(false);
            writer->puts(
                "FCAN filter disabled - all FCAN frames pass; "
                "re-enable after capture to avoid ISR storm while charging");
        });
    cmd_filter->RegisterCommand(
        "status", "Show filter state",
        [this](int, OvmsWriter* writer, OvmsCommand*, int, const char* const*) {
            writer->printf("FCAN filter: %s\n",
                           MyConfig.GetParamValueBool("xvg", "fcan-filter", true)
                               ? "enabled (5 decoded IDs only)"
                               : "disabled (all frames pass)");
        });

    // Camping mode: overnight cabin-temperature thermostat. Holds cabin temp within
    // [cc-camp-tmin, cc-camp-tmax] by tripping clima start/stop, guarded by a SoC floor
    // and a max-run-hours backstop. Logic lives in StartCamping/StopCamping and Ticker1.
    OvmsCommand* cmd_camping =
        cmd_xvg->RegisterCommand("camping", "Overnight cabin climate thermostat");
    cmd_camping->RegisterCommand(
        "on", "Start camping mode (maintain cabin temperature band)",
        [this](int, OvmsWriter* writer, OvmsCommand*, int, const char* const*) {
            StartCamping();
            writer->printf("Camping mode on: hold %d-%d C, SoC floor %d%%, max %d h\n",
                           MyConfig.GetParamValueInt("xvg", "cc-camp-tmin", 15),
                           MyConfig.GetParamValueInt("xvg", "cc-camp-tmax", 26),
                           MyConfig.GetParamValueInt("xvg", "cc-camp-socfloor", 30),
                           MyConfig.GetParamValueInt("xvg", "cc-camp-maxhours", 8));
        });
    cmd_camping->RegisterCommand(
        "off", "Stop camping mode",
        [this](int, OvmsWriter* writer, OvmsCommand*, int, const char* const*) {
            StopCamping("cli");
            writer->puts("Camping mode off");
        });
    cmd_camping->RegisterCommand(
        "status", "Show camping mode state",
        [this](int, OvmsWriter* writer, OvmsCommand*, int, const char* const*) {
            if (!m_camping_active) {
                writer->puts("Camping mode: off");
                return;
            }
            float cabin = StandardMetrics.ms_v_env_cabintemp->AsFloat();
            bool stale = StandardMetrics.ms_v_env_cabintemp->IsStale();
            writer->printf(
                "Camping mode: on (%lu min)\n"
                "  cabin %.1f C%s, band %d-%d C\n"
                "  SoC %.0f%% (floor %d%%), hvac %s\n",
                (unsigned long)(m_camping_secs / 60), cabin, stale ? " STALE" : "",
                MyConfig.GetParamValueInt("xvg", "cc-camp-tmin", 15),
                MyConfig.GetParamValueInt("xvg", "cc-camp-tmax", 26),
                StandardMetrics.ms_v_bat_soc->AsFloat(),
                MyConfig.GetParamValueInt("xvg", "cc-camp-socfloor", 30),
                StandardMetrics.ms_v_env_hvac->AsBool() ? "running" : "idle");
        });

#ifdef CONFIG_OVMS_COMP_WEBSERVER
    WebInit();
#endif
}

// Apply or clear the FCAN (can2) MCP2515 hardware acceptance filter.
//
// .u32 encoding: SetAcceptanceFilter sends bytes u8[3..0] in order as MCP2515
// registers SIDH, SIDL, EID8, EID0.  .b.sid stores the SID in bits [10:0] (LSB),
// which lands in EID8/EID0 — wrong.  Use u32 = (SID>>3)<<24 | (SID&7)<<21 instead.
//
// disable: an all-zero config means every mask bit is don't-care, so the MCP2515
// accepts every frame (promiscuous) — used for frame-discovery captures.  The driver
// stores the config and re-applies it on bus restart, so either state survives
// `can can2 stop/start`.
void OvmsVehicleVWeGolf::SetFcanFilter(bool enable) {
    mcp2515_filter_config_t f = {};
    if (enable) {
        auto mcp_sid = [](uint16_t sid) -> uint32_t {
            return (static_cast<uint32_t>(sid >> 3) << 24) |
                   (static_cast<uint32_t>(sid & 0x7) << 21);
        };
        // MCP2515 has 2 RX buffers: RXB0 uses mask[0] + filter[0..1] (2 slots);
        // RXB1 uses mask[1] + filter[2..5] (4 slots). We need 5 unique IDs total;
        // filter[5] duplicates filter[4] (0x6B4) so the unused slot does not
        // accidentally accept id 0 with the all-don't-care default.
        f.mask[0].u32 = mcp_sid(0x7FF);
        f.mask[1].u32 = mcp_sid(0x7FF);
        f.filter[0].u32 = mcp_sid(0x131);  // SoC (BMS)              -> RXB0
        f.filter[1].u32 = mcp_sid(0x187);  // gear selector          -> RXB0
        f.filter[2].u32 = mcp_sid(0x191);  // BMS current/voltage    -> RXB1
        f.filter[3].u32 = mcp_sid(0x2AF);  // trip energy            -> RXB1
        f.filter[4].u32 = mcp_sid(0x6B4);  // VIN                    -> RXB1
        f.filter[5].u32 = mcp_sid(0x6B4);  // (duplicate of filter[4] to fill unused slot)
    }
    if (static_cast<mcp2515*>(m_can2)->SetAcceptanceFilter(f) != ESP_OK)
        ESP_LOGE(TAG, "FCAN acceptance filter %s failed", enable ? "enable" : "disable");
    else
        ESP_LOGI(TAG, "FCAN acceptance filter %s", enable ? "enabled" : "disabled");
}

OvmsVehicleVWeGolf::~OvmsVehicleVWeGolf() {
#ifdef CONFIG_OVMS_COMP_WEBSERVER
    WebDeInit();
#endif
    MyCommandApp.UnregisterCommand("xvg");
    ESP_LOGI(TAG, "Stop vehicle module: VW e-Golf");
}

class OvmsVehicleVWeGolfInit {
 public:
    OvmsVehicleVWeGolfInit();
} MyOvmsVehicleVWeGolfInit __attribute__((init_priority(9000)));

OvmsVehicleVWeGolfInit::OvmsVehicleVWeGolfInit() {
    ESP_LOGI(TAG, "Registering Vehicle: VW e-Golf (9000)");
    MyVehicleFactory.RegisterVehicle<OvmsVehicleVWeGolf>("VWEG", "VW e-Golf");
}

// ---------------------------------------------------------------------------
// UDS BMS polling (can1 = OBD/diag CAN)
// ---------------------------------------------------------------------------

// Derive the poll state each second (called from Ticker1).
// CHARGING: 0x594 charge gate (authoritative, KCAN). ON: FCAN alive = powertrain
// awake (driving/ready — charging is caught first, so no overlap). AWAKE: only
// KCAN alive. OFF: all buses silent — the diag CAN is asleep too, polls would
// only accumulate TX errors.
void OvmsVehicleVWeGolf::UpdatePollState() {
    uint8_t want = VWEGOLF_OFF;
    if (StandardMetrics.ms_v_charge_inprogress->AsBool())
        want = VWEGOLF_CHARGING;
    else if (m_fcan_idle_ticks < VWEGOLF_BUS_TIMEOUT_SECS)
        want = VWEGOLF_ON;
    else if (m_bus_idle_ticks < VWEGOLF_BUS_TIMEOUT_SECS)
        want = VWEGOLF_AWAKE;

    if (want != m_poll_state) {
        ESP_LOGI(TAG, "Poll state %u -> %u", m_poll_state, want);
        PollSetState(want);
    }
}

void OvmsVehicleVWeGolf::IncomingPollReply(const OvmsPoller::poll_job_t& job, uint8_t* data,
                                           uint8_t length) {
    // BMS measured pack voltage/current (UDS, ECU 8C). Scaling from vehicle_vweup —
    // validate on car before trusting (see vwegolf_polls above).
    // Metrics are written only while CHARGING: 0x191 owns bat_* when driving and is
    // silent (DC) / 0 A (AC) during charge. ON state logs only, for on-car validation
    // against live 0x191 values.
    if (job.moduleid_rec != VWEGOLF_BMS_RX || length < 2) return;
    const uint16_t raw = ((uint16_t)data[0] << 8) | data[1];

    switch (job.pid) {
        case VWEGOLF_BMS_DID_U: {
            const float volts = raw / 4.0f;
            ESP_LOGD(TAG, "UDS BMS U raw=%u -> %.2fV", raw, volts);
            if (m_poll_state == VWEGOLF_CHARGING) {
                StandardMetrics.ms_v_bat_voltage->SetValue(volts);
                // Charge metrics are pack-side: 0x0569 is NOT the OBC AC inlet (see that
                // case in IncomingFrameCan3), so charge voltage/current/power are derived
                // from the BMS pack measurement, the only frame that tracks the real rate
                // on both AC and DC. Folds in the small inlet->pack loss.
                StandardMetrics.ms_v_charge_voltage->SetValue(volts);
            }
            break;
        }
        case VWEGOLF_BMS_DID_I: {
            // 2044 offset, 0.25 A/bit. ECU sign: negative = current out of the battery;
            // OVMS wants discharge positive -> negate (same convention as the 0x191 path,
            // so charge shows as negative current and negative power).
            const float amps = ((float)raw - 2044.0f) / 4.0f * -1.0f;
            ESP_LOGD(TAG, "UDS BMS I raw=%u -> %.2fA", raw, amps);
            if (m_poll_state == VWEGOLF_CHARGING) {
                StandardMetrics.ms_v_bat_current->SetValue(amps);
                const float bat_power =
                    StandardMetrics.ms_v_bat_voltage->AsFloat() * amps / 1000.0f;
                StandardMetrics.ms_v_bat_power->SetValue(bat_power);
                // Charging: bat current/power are negative (into the pack). Charge metrics
                // use the positive-magnitude convention, so negate. Pack-side source — see
                // the DID_U case above and the 0x0569 note in IncomingFrameCan3.
                StandardMetrics.ms_v_charge_current->SetValue(-amps);
                StandardMetrics.ms_v_charge_power->SetValue(-bat_power);
            }
            break;
        }
        case VWEGOLF_BMS_DID_SOH_CAC: {
            // Battery SoH — e-Golf raw is 10x the pre-2020 vweup scale (observed raw 43177
            // vs vweup /50): divisor 500 gives 86.35% on this pack (101 Mm), plausible.
            // Provisional until cross-checked against a diag readout. CAC not written:
            // vweup's raw/100 gave 431.77 Ah here (nominal ~111 Ah) — scaling unknown,
            // needs re-derivation from a capture before ms_v_bat_cac gets a value.
            // Slow-changing, not gated to CHARGING like U/I: written whenever polled.
            const float soh = raw / 500.0f;
            ESP_LOGD(TAG, "UDS BMS SOH_CAC raw=%u -> SoH=%.2f%%", raw, soh);
            StandardMetrics.ms_v_bat_soh->SetValue(soh);
            break;
        }
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// FCAN (CAN2) — powertrain bus: BMS, gear selector, VIN
// ---------------------------------------------------------------------------

void OvmsVehicleVWeGolf::IncomingFrameCan2(CAN_frame_t* p_frame) {
    uint8_t* d = p_frame->data.u8;
    float f;
    uint16_t u16;

    // FCAN liveness counter for the poll-state machine (see UpdatePollState).
    m_fcan_idle_ticks = 0;

    switch (p_frame->MsgID) {
        case 0x0131: {
            // State of charge. Single byte in d[3], factor 0.5 %.
            // 0xFE is the ECU's "not ready" sentinel (decodes to 127.0%) — discard it.
            // Decoded on BOTH buses on purpose: FCAN stops broadcasting 0x131 during CCS DC
            // charge (SoC would pin), so IncomingFrameCan3 also decodes the J533-bridged
            // copy. Same value/scaling → no flapping. Do not "dedupe" by removing either.
            if (d[3] == 0xFE) break;
            f = d[3] * 0.5f;
            StandardMetrics.ms_v_bat_soc->SetValue(f);
            ESP_LOGV(TAG, "0x0131 soc=%.1f%%", f);
            break;
        }
        case 0x0191: {
            // BMS current and voltage; power is derived.
            // d[2]==0xFF is the startup sentinel (all-ones current field → ~2047A, ~1023V).
            if (d[2] == 0xFF) break;
            // Current: 12-bit, factor 1 A, offset -2047 A (raw=2047 → 0 A).
            // NOTE: during AC charging this field reads 0A — 0x0191 appears to carry
            // inverter/motor current only. Charging current comes from a different frame
            // not yet identified. TODO: capture charging session and identify the frame.
            u16 = ((uint16_t)(d[1] & 0xF0) >> 4) | ((uint16_t)(d[2]) << 4);
            // Negated: OVMS convention is discharge=positive (battery outputs current),
            // charge=negative — same as the UDS DID_I path and the vweup reference. Raw
            // field is charge-positive (raw>2047 while charging), so flip it here.
            float current = -(u16 * 1.0f - 2047.0f);
            StandardMetrics.ms_v_bat_current->SetValue(current);

            // Voltage: 12-bit, factor 0.25 V.
            u16 = (uint16_t)(d[3]) | ((uint16_t)(d[4] & 0x0F) << 8);
            float voltage = u16 * 0.25f;
            StandardMetrics.ms_v_bat_voltage->SetValue(voltage);

            // Current already carries the OVMS sign, so power follows directly:
            // drive=positive, charge/regen=negative.
            StandardMetrics.ms_v_bat_power->SetValue((voltage * current) / 1000.0f);
            ESP_LOGV(TAG, "0x0191 raw=%02x%02x%02x%02x%02x I=%.1fA V=%.2fV", d[1], d[2], d[3], d[4],
                     d[5], current, voltage);
            break;
        }
        case 0x02AF: {
            // Trip energy: regeneration recovered and total consumed.
            // Both are 15-bit, factor 10 Ws; convert to kWh.
            u16 = (uint16_t)(d[4]) | ((uint16_t)(d[5] & 0x7F) << 8);
            StandardMetrics.ms_v_bat_energy_recd->SetValue(u16 * 10.0f / 3600000.0f);

            u16 = (uint16_t)(d[6]) | ((uint16_t)(d[7] & 0x7F) << 8);
            StandardMetrics.ms_v_bat_energy_used->SetValue(u16 * 10.0f / 3600000.0f);
            break;
        }
        case 0x187: {
            // Gear selector position. Nibble at d[2][3:0]: 2=P, 3=R, 4=N, 5=D, 6=B.
            // OVMS convention: negative=reverse, 0=neutral/park, positive=forward.
            // drivemode 1 signals B-mode (increased regen).
            const uint8_t gear = d[2] & 0x0F;
            if (gear == 2) {
                StandardMetrics.ms_v_env_gear->SetValue(0);
                StandardMetrics.ms_v_env_drivemode->SetValue(0);
            } else if (gear == 3) {
                StandardMetrics.ms_v_env_gear->SetValue(-1);
                StandardMetrics.ms_v_env_drivemode->SetValue(0);
            } else if (gear == 4) {
                StandardMetrics.ms_v_env_gear->SetValue(0);
                StandardMetrics.ms_v_env_drivemode->SetValue(0);
            } else if (gear == 5) {
                StandardMetrics.ms_v_env_gear->SetValue(1);
                StandardMetrics.ms_v_env_drivemode->SetValue(0);
            } else if (gear == 6) {
                StandardMetrics.ms_v_env_gear->SetValue(1);
                StandardMetrics.ms_v_env_drivemode->SetValue(1);
            }
            ESP_LOGV(TAG, "0x187 gear nibble=%u", gear);
            break;
        }
        case 0x6B4: {
            // VIN is broadcast in three frames distinguished by data[0] (0, 1, 2).
            // We assemble all three before committing — a partial VIN is meaningless.
            // Once all three are received the VIN is immutable; further frames are ignored.
            uint8_t frame_idx = d[0];
            if (m_vin_parts_received == 0x07) break;

            if (frame_idx == 0) {
                m_vin_buf[0] = d[5];
                m_vin_buf[1] = d[6];
                m_vin_buf[2] = d[7];
                m_vin_parts_received |= 0x01;
            } else if (frame_idx == 1) {
                memcpy(&m_vin_buf[3], &d[1], 7);
                m_vin_parts_received |= 0x02;
            } else if (frame_idx == 2) {
                memcpy(&m_vin_buf[10], &d[1], 7);
                m_vin_parts_received |= 0x04;
            }

            if (m_vin_parts_received == 0x07) {
                m_vin_buf[17] = '\0';
                StandardMetrics.ms_v_vin->SetValue(m_vin_buf);
                ESP_LOGI(TAG, "VIN: %s", m_vin_buf);
            }
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// KCAN (CAN3) — convenience bus: body, BMS, clima, GPS, charging
// ---------------------------------------------------------------------------

void OvmsVehicleVWeGolf::IncomingFrameCan3(CAN_frame_t* p_frame) {
    m_bus_idle_ticks = 0;

    // Send OCU keepalive at ~5Hz while active. VW OSEK NM requires keepalives at
    // ~200ms intervals; Ticker1 alone (1Hz) is too slow for the ECU to stay in network.
    // SendOcuHeartbeat self-throttles (180ms min) against TX queue overflow on bus bursts.
    if (m_ocu_active) {
        SendOcuHeartbeat();
    }

    uint8_t* d = p_frame->data.u8;

    // Track OEM OCU activity: any non-zero 0x5A7 means the car's OCU is still active.
    // Reset idle counter so we don't try to wake while it's conflicting with our heartbeat.
    if (p_frame->MsgID == 0x5A7) {
        if (d[0] | d[1] | d[2] | d[3] | d[4] | d[5] | d[6] | d[7]) {
            m_oem_ocu_idle_ticks = 0;
        }
    }
    float f = 0.0f;
    uint16_t u16;
    uint32_t u32;

    switch (p_frame->MsgID) {
        case 0x00FD: {
            // Vehicle speed from ESP (electronic stability program) module.
            // 16-bit little-endian in d[4:5], factor 0.01 km/h.
            f = ((uint16_t)(d[4]) | ((uint16_t)(d[5]) << 8)) * 0.01f;
            StandardMetrics.ms_v_pos_speed->SetValue(f);
            ESP_LOGV(TAG, "0x00FD speed=%.2f km/h", f);
            break;
        }
        case 0x0131: {
            // KCAN-bridged BMS state of charge (J533 forwards 0x131 from FCAN).
            // Intentional twin of the IncomingFrameCan2 0x131 path — keeps SoC live during
            // CCS DC charge when FCAN goes quiet. See that case for the full rationale.
            // d[3] x 0.5 %; 0xFE is the BMS "not ready" sentinel.
            if (d[3] == 0xFE) break;
            f = d[3] * 0.5f;
            StandardMetrics.ms_v_bat_soc->SetValue(f);
            ESP_LOGV(TAG, "0x0131 soc=%.1f%%", f);
            break;
        }
        case 0x0486: {
            // GPS position. Latitude: bits 0-26 (27 bits), longitude: bits 27-54 (28 bits),
            // factor 1e-6 degrees. Sign bits inferred from bit layout (55 bits used out of
            // 64): bit 55 (d[6] MSB) = lat sign, bit 56 (d[7] bit 0) = lon sign.
            // Confirmed consistent with known N/E location (Norway). Needs a S/W hemisphere
            // capture to verify. Only write metrics when values are within valid range —
            // prevents sentinel frames (all 0xFF → 134°/268°) from overwriting good position.
            u32 = (uint32_t)(d[0]) | ((uint32_t)(d[1]) << 8) | ((uint32_t)(d[2]) << 16) |
                  ((uint32_t)(d[3] & 0x07) << 24);
            float lat = u32 * 0.000001f;
            if ((d[6] >> 7) & 1) lat = -lat;  // Southern hemisphere

            u32 = ((uint32_t)(d[3] & 0xF8) >> 3) | ((uint32_t)(d[4]) << 5) |
                  ((uint32_t)(d[5]) << 13) | ((uint32_t)(d[6] & 0x7F) << 21);
            float lon = u32 * 0.000001f;
            if ((d[7] >> 0) & 1) lon = -lon;  // Western hemisphere

            bool valid = (lat > -91.0f && lat < 91.0f && lon > -181.0f && lon < 181.0f);
            StandardMetrics.ms_v_pos_gpslock->SetValue(valid);
            if (valid) {
                StandardMetrics.ms_v_pos_latitude->SetValue(lat);
                StandardMetrics.ms_v_pos_longitude->SetValue(lon);
            }
            ESP_LOGV(TAG, "0x0486 lat=%.6f lon=%.6f valid=%d", lat, lon, valid);
            break;
        }
        case 0x0583: {
            // ZV_02: central locking and door open states.
            // d[2] bit 1: locked externally. d[3] bits 4:0: trunk, rr, rl, fr, fl (1=open).
            StandardMetrics.ms_v_env_locked->SetValue((d[2] & 0x02) >> 1);
            StandardMetrics.ms_v_door_fl->SetValue((d[3] & 0x01) >> 0);
            StandardMetrics.ms_v_door_fr->SetValue((d[3] & 0x02) >> 1);
            StandardMetrics.ms_v_door_rl->SetValue((d[3] & 0x04) >> 2);
            StandardMetrics.ms_v_door_rr->SetValue((d[3] & 0x08) >> 3);
            StandardMetrics.ms_v_door_trunk->SetValue((d[3] & 0x10) >> 4);
            ESP_LOGV(TAG, "0x0583 locked=%u fl=%u fr=%u rl=%u rr=%u trunk=%u", (d[2] & 0x02) >> 1,
                     d[3] & 0x01, (d[3] & 0x02) >> 1, (d[3] & 0x04) >> 2, (d[3] & 0x08) >> 3,
                     (d[3] & 0x10) >> 4);
            break;
        }
        case 0x0569: {
            // NOT the OBC AC inlet. Earlier read decoded d[4] as AC line voltage and d[5]
            // as charge current, but capture review disproves it:
            //   - d[4] is a constant 0xF8 in every state (parked+clima, idle, AC, and CCS
            //     DC) — it is a status byte, not a voltage. "248 V" was a coincidental
            //     match of 0xF8 to NZ mains.
            //   - d[5] is an auxiliary/DC-DC-converter current: 0 A idle, ~9 A during
            //     clima-while-parked, ~13 A through a 33 kW CCS DC session. d[4]*d[5] is
            //     therefore a flat ~3.2 kW during a 33 kW charge — it does not track the
            //     charge rate. (caps: all-dc583be4a-…-132333, all-168388a82, clima_control)
            // Charge voltage/current/power now come from the BMS pack UDS poll instead
            // (see the DID_U/DID_I handlers in IncomingPollReply). Decoded here as a
            // diagnostic log only; no metric is driven from this frame.
            ESP_LOGV(TAG, "0x0569 d[1]=%02x d[4]=%02x aux_current=%uA (not OBC AC inlet)", d[1],
                     d[4], d[5]);
            break;
        }
        case 0x0594: {
            // HV charge management: AC/DC type, timer, plug, cabin setpoint.

            // Time to full charge: 9-bit, factor 5 min.
            u16 = ((uint16_t)(d[1] & 0xF0) >> 4) | ((uint16_t)(d[2] & 0x1F) << 4);
            StandardMetrics.ms_v_charge_duration_full->SetValue(u16 * 5, Minutes);

            // Charge timer enabled when bits [6:5] of d[2] == 0x01.
            StandardMetrics.ms_v_charge_timermode->SetValue(((d[2] & 0x60) >> 5) == 0x01);

            // Charging in progress: bit 5 of d[3].
            {
                bool was_charging = StandardMetrics.ms_v_charge_inprogress->AsBool();
                bool is_charging = (d[3] & 0x20) != 0;
                StandardMetrics.ms_v_charge_inprogress->SetValue(is_charging);
                StandardMetrics.ms_v_charge_state->SetValue(is_charging ? "charging" : "stopped");
                if (is_charging != was_charging) {
                    if (is_charging) {
                        NotifyChargeStart();
                    } else {
                        NotifyChargeStopped();
                        // Charge metrics are fed from the BMS UDS poll, which stops once
                        // the poll state leaves CHARGING — without this they would freeze
                        // at the last sampled value. Zero them on charge end.
                        StandardMetrics.ms_v_charge_current->SetValue(0);
                        StandardMetrics.ms_v_charge_power->SetValue(0);
                    }
                }
            }

            // Charge type from bits [3:2] of d[5]: 0=none, 1=AC Type2, 3=cable connected
            // (charge not needed — e.g. 100% SOC). CCS DC keeps KCAN silent so case 2
            // is not observable. Do not update in the default case — leave last value.
            // Charge port open = cable physically present (ChargeType != 0).
            // The framework's status display gates on ms_v_door_chargeport — without it,
            // the "Not charging" fallback always shows regardless of charge_inprogress.
            {
                uint8_t charge_type = (d[5] & 0x0C) >> 2;
                switch (charge_type) {
                    case 0:
                        StandardMetrics.ms_v_door_chargeport->SetValue(false);
                        break;
                    case 1:
                        StandardMetrics.ms_v_charge_type->SetValue("type2");
                        StandardMetrics.ms_v_door_chargeport->SetValue(true);
                        break;
                    case 3:
                        // Cable connected, charge complete or not needed.
                        StandardMetrics.ms_v_door_chargeport->SetValue(true);
                        break;
                    default:
                        break;
                }
            }

            // Cabin temperature setpoint: 5-bit, factor 0.5°C, offset +15.5°C.
            f = (d[7] & 0x1F) * 0.5f + 15.5f;
            StandardMetrics.ms_v_env_cabinsetpoint->SetValue(f);

            ESP_LOGV(TAG, "0x0594 charging=%d timer=%d type=%s d[3]=%02x d[5]=%02x setpoint=%.1f°C",
                     StandardMetrics.ms_v_charge_inprogress->AsBool(),
                     StandardMetrics.ms_v_charge_timermode->AsBool(),
                     StandardMetrics.ms_v_charge_type->AsString().c_str(), d[3], d[5], f);
            break;
        }
        case 0x059E: {
            // BMS battery pack temperature. Factor 0.5°C, offset -40°C.
            // d[2]==0xFE is the startup sentinel (decodes to 87°C).
            if (d[2] == 0xFE) break;
            f = d[2] * 0.5f - 40.0f;
            StandardMetrics.ms_v_bat_temp->SetValue(f);
            ESP_LOGV(TAG, "0x059E bat_temp=%.1f°C", f);
            break;
        }
        case 0x05CA: {
            // HV battery REMAINING energy content (tracks SoC: 5 kWh observed at 18%),
            // not pack capacity — writing it to ms_v_bat_capacity was a bug (upstream
            // still has it; fix rides the charging PR). Log-only until OVMS grows a
            // remaining-energy home for it. 11-bit, factor 50 Wh → kWh.
            // d[2]==0xFF is the startup sentinel (near-max field; real values ≤40 kWh).
            if (d[2] == 0xFF) break;
            u16 = ((uint16_t)(d[1] & 0xF0) >> 4) | ((uint16_t)(d[2] & 0x7F) << 4);
            f = u16 * 50.0f / 1000.0f;
            ESP_LOGV(TAG, "0x05CA bat energy remaining=%.1f kWh (log-only)", f);
            break;
        }
        case 0x05EA: {
            // Clima ECU status broadcast.
            // remote_mode reflects "clima ECU energized" — true whenever ignition/ACC is on
            // OR a remote session keeps the ECU awake — NOT cabin conditioning. Capture
            // all-168388a82-…-224827 showed it pinned with ignition off and asserted by
            // radio/ACC mode, so ms_v_env_hvac is driven from 0x03B5 ClimaRunning instead.
            // remote_mode stays log-only.
            //
            // ClimaCabinTemp (this field) is the PRIMARY ms_v_env_cabintemp source
            // (WI-cabintemp-1, 2026-07-16): 0x066E d[4] is permanently 0xFE ("not ready")
            // on this car across all 22 captures on record, so that setter never fires.
            // (Raw ≥ 1020 → ≥ 62°C is the ECU's cabin-temp "not available" sentinel.)
            u16 = ((uint16_t)(d[6] & 0xFC) >> 2) | ((uint16_t)(d[7] & 0x0F) << 6);
            uint8_t remote_mode = ((d[3] & 0xC0) >> 6) | ((d[4] & 0x01) << 2);
            (void)remote_mode;  // log-only; ESP_LOGV compiles out in the native test build
            if (u16 < 1020) {
                f = u16 * 0.1f - 40.0f;
                StandardMetrics.ms_v_env_cabintemp->SetValue(f);
                ESP_LOGV(TAG, "0x05EA clima_cabin=%.1f°C remote_mode=%u", f, remote_mode);
            } else {
                ESP_LOGV(TAG, "0x05EA clima_cabin=n/a remote_mode=%u", remote_mode);
            }
            break;
        }
        case 0x03B5: {
            // ClimaRunning: d[0] bit7 = blower actively conditioning the cabin (0x80=on).
            // Authoritative ms_v_env_hvac run-state — reflects real conditioning regardless
            // of trigger (remote, schedule, or driving), unlike 0x05EA remote_mode which only
            // tracks ECU power. Capture all-168388a82-…-224827.
            if (d[0] & 0x80) {
                // Running evidence — refresh the hold timer. Suppress turning the metric back
                // on during the spin-down right after our own stop command (a stop sets false
                // immediately for responsive UX); if 0x03B5 still reports running past the
                // suppress window, the stop didn't take, so trust it.
                m_clima_run_secs = 0;
                if (m_hvac_stop_secs >= VWEGOLF_HVAC_STOP_SUPPRESS_SECS) {
                    StandardMetrics.ms_v_env_hvac->SetValue(true);
                }
            }
            ESP_LOGV(TAG, "0x03B5 clima_running=%u", (d[0] >> 7) & 1);
            break;
        }
        case 0x17332510: {
            // BAP status/ACK from clima ECU (node 0x25), extended 29-bit frame.
            // Pattern `49 58 XX` (DLC=3): port 0x18 (start/stop trigger) ACK. This is a
            // transactional ACK, NOT a run-state — it emits a spurious 49 58 00 mid-session
            // (capture all-168388a82-…-221008) — so it does NOT drive ms_v_env_hvac
            // (0x03B5 ClimaRunning does). We use it only to stand down the OCU ring once the
            // transaction has been acknowledged.
            if (p_frame->FIR.B.DLC == 3 && d[0] == 0x49 && d[1] == 0x58) {
                ESP_LOGI(TAG, "0x17332510 BAP port 0x18 ACK: d2=0x%02X", d[2]);
                // Arm grace: transaction is done, keep the ring warm briefly then stand down.
                m_ocu_grace_secs = 0;
            }
            break;
        }
        case 0x05F5: {
            // Range estimates from the instrument cluster.
            // Estimated range (displayed on dash): 11-bit, factor 1 km.
            u16 = ((uint16_t)(d[3] & 0xE0) >> 5) | ((uint16_t)(d[4]) << 3);
            StandardMetrics.ms_v_bat_range_est->SetValue((float)u16);

            // Ideal range (BMS model lower bound, typically lower than estimated).
            u16 = (uint16_t)(d[0]) | ((uint16_t)(d[1] & 0x07) << 8);
            StandardMetrics.ms_v_bat_range_ideal->SetValue((float)u16);

            ESP_LOGV(TAG, "0x05F5 range_est=%u range_ideal=%u km",
                     StandardMetrics.ms_v_bat_range_est->AsInt(),
                     StandardMetrics.ms_v_bat_range_ideal->AsInt());
            break;
        }
        case 0x065A: {
            // BCM_01: hood open indicator (d[4] bit 0).
            StandardMetrics.ms_v_door_hood->SetValue(d[4] & 0x01);
            ESP_LOGV(TAG, "0x065A hood=%u", d[4] & 0x01);
            break;
        }
        case 0x066E: {
            // InnenTemp: cabin interior temperature sensor. Log-only (WI-cabintemp-1,
            // 2026-07-16): d[4] is permanently 0xFE ("not ready" sentinel, decodes to 77°C)
            // on this car — disproven across all 22 captures on record, never observed
            // non-sentinel. ms_v_env_cabintemp is now driven from 0x05EA ClimaCabinTemp.
            if (d[4] == 0xFE) break;
            f = d[4] * 0.5f - 50.0f;
            ESP_LOGV(TAG, "0x066E cabin_temp=%.1f°C (log-only)", f);
            break;
        }
        case 0x06B0: {
            // FS temperature sensor (windshield/front area).
            // Logged for reference; not yet mapped to a standard metric.
            f = d[4] * 0.5f - 40.0f;
            ESP_LOGV(TAG, "0x06B0 fs_temp=%.1f°C", f);
            break;
        }
        case 0x06B5: {
            // Ambient temperature from two sensors: solar sensor and outside air.
            // d[6]==0xFE is the startup sentinel (both sensors report max-range garbage).
            if (d[6] == 0xFE) break;
            f = ((uint16_t)(d[6]) | ((uint16_t)(d[7] & 0x07) << 8)) * 0.1f - 40.0f;
            ESP_LOGV(TAG, "0x06B5 solar_sensor=%.1f°C", f);
            f = ((uint16_t)(d[2]) | ((uint16_t)(d[3] & 0x03) << 8)) * 0.1f - 40.0f;
            ESP_LOGV(TAG, "0x06B5 air_sensor=%.1f°C", f);
            break;
        }
        case 0x06B7: {
            // Odometer, parking time, and filtered outside temperature.

            // Odometer: 20-bit, factor 1 km.
            uint32_t odo =
                (uint32_t)(d[0]) | ((uint32_t)(d[1]) << 8) | ((uint32_t)(d[2] & 0x0F) << 16);
            StandardMetrics.ms_v_pos_odometer->SetValue(odo);

            // Park time: 17-bit field at bit offset 20, factor 1 s.
            // d[2] bits [7:4] → result bits [3:0], d[3] → [11:4], d[4] bits [4:0] → [16:12].
            // The field saturates at its 17-bit max (0x1FFFF ≈ 36.5 h); ignore that clamped
            // value so v.e.parktime falls back to OVMS's native (uncapped) counter.
            // (Upstream c51e6aac1 — corrects the low-nibble shift and adds the saturation guard.)
            u32 = ((uint32_t)(d[2] & 0xF0) >> 4) | ((uint32_t)(d[3]) << 4) |
                  ((uint32_t)(d[4] & 0x1F) << 12);
            if (u32 != 0x1FFFF) {
                StandardMetrics.ms_v_env_parktime->SetValue(u32);
            }

            // Outside temperature (filtered): factor 0.5°C, offset -50°C.
            f = d[7] * 0.5f - 50.0f;
            StandardMetrics.ms_v_env_temp->SetValue(f);

            ESP_LOGV(TAG, "0x06B7 odo=%u km outside=%.1f°C", odo, f);
            break;
        }
        case 0x0391:  // OBD_01: drivetrain READY status
        {
            // d[7] bit 5 is OBD_Driving_Cycle: it goes high only once the drivetrain is fully
            // up and the car is ready to drive; it stays clear during charging, remote climate
            // and while the ignition is merely on but not yet READY. The frame keeps
            // broadcasting after the ignition goes off and the bit stays latched high for
            // several seconds into the power-down, so it is combined with KL_15 for v.e.on
            // rather than used on its own. if only ignition is turned on again this bit is cleared.
            // (d[5] carries the accelerator pedal position, OBD_Abs_Pedal_Pos - not mapped.)
            m_drivetrain_ready = (d[7] & 0x20) != 0;
            StandardMetrics.ms_v_env_on->SetValue(m_kl15_on && m_drivetrain_ready);
            ESP_LOGV(TAG, "0x0391 READY=%u", m_drivetrain_ready);
            break;
        }
        case 0x03C0:  // clamp status received
        {
            // the following are from d[2]
            // KL_S Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_15 Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_X Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_50 Startanforderung Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // Remotestart Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_Infotainment Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // Remotestart_KL15_Anf Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // Remotestart_Motor_Start Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_15 (terminal 15 = ignition) means the car is awake and switched on by the
            // user; drivable (v.e.on) additionally requires the drivetrain to report READY.
            m_kl15_on = (d[2] & 0x02) != 0;
            StandardMetrics.ms_v_env_awake->SetValue(m_kl15_on);
            StandardMetrics.ms_v_env_on->SetValue(m_kl15_on && m_drivetrain_ready);
            ESP_LOGV(TAG, "0x03C0 KL_15=%u KL_S=%u", m_kl15_on, d[2] & 0x01);
            break;
        }
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Periodic tickers
// ---------------------------------------------------------------------------

void OvmsVehicleVWeGolf::Ticker1(uint32_t ticker) {
    OvmsVehicle::Ticker1(ticker);

    // Count consecutive seconds of KCAN silence. IncomingFrameCan3 resets this to 0
    // whenever a frame arrives, so it measures how long since the last activity.
    if (m_bus_idle_ticks < 254) m_bus_idle_ticks++;
    if (m_fcan_idle_ticks < 254) m_fcan_idle_ticks++;
    if (m_oem_ocu_idle_ticks < 254) m_oem_ocu_idle_ticks++;
    if (m_clima_run_secs < 255) m_clima_run_secs++;
    if (m_hvac_stop_secs < 255) m_hvac_stop_secs++;

    UpdatePollState();

    // ms_v_env_hvac (driven by 0x03B5 ClimaRunning + commands) clears once no "running"
    // evidence has arrived for the hold window. Bridges blower thermostat cycling and clears
    // the metric when the car sleeps. A stop command already set it false immediately, so this
    // governs only autonomous/timer stops (≈hold-second linger).
    if (m_clima_run_secs >= VWEGOLF_HVAC_RUN_HOLD_SECS && StandardMetrics.ms_v_env_hvac->AsBool()) {
        StandardMetrics.ms_v_env_hvac->SetValue(false);
    }

    bool bus_alive = m_bus_idle_ticks < VWEGOLF_BUS_TIMEOUT_SECS;
    bool just_went_idle = (m_bus_idle_ticks == VWEGOLF_BUS_TIMEOUT_SECS);
    ESP_LOGV(TAG, "Ticker1: bus_idle=%u alive=%d ocu=%d", m_bus_idle_ticks, bus_alive,
             m_ocu_active);

    // KCAN gone silent → car is asleep: clear awake/on as a backstop in case the terminal
    // frames (0x03C0/0x0391) stopped before signalling the off transition. Exception to the
    // "decoders own metrics" rule below: asleep really does mean not-awake/not-on, and unlike
    // charge_inprogress this holds during CCS DC too (car charging is not "on"). From PR #1453.
    if (just_went_idle) {
        m_kl15_on = false;
        m_drivetrain_ready = false;
        StandardMetrics.ms_v_env_awake->SetValue(false);
        StandardMetrics.ms_v_env_on->SetValue(false);
    }

    OcuTick(bus_alive, just_went_idle);
    ClimaTick(bus_alive);
    CampingTick();
}
