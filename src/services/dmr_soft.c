/*
 * Copyright (C) 2026 Junqiang Shi, VA3BPF / BG8AM
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * 4. Use of this source code or binary releases for commercial purposes is strictly forbidden without the written
 *    permission of the copyright holder. This includes, without limitation, incorporation in a commercial product or
 *    incorporation into a product or project which allows commercial use.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * OpenDMR, service: the DMR receiver in software (dmr_soft.h, dmr_soft/README.md).
 *
 * As the NXDN and M17 receivers (afrx.c runs it): the modem records the FD6818's discriminator, dmr_demod.c finds
 * the TDMA time line and gives each 30 ms burst; dmr_l2.c decodes its fields. Each burst's timeslot: on a BS signal
 * from the CACH before it (TC: the channel of the burst that follows), in TDMA direct mode from its SYNC, an MS
 * signal (direct mode, one transmitter) taken as the channel's timeslot. Per timeslot:
 *
 * - a data burst (its Slot Type, colour code): a voice LC header starts the call (its LC through the DMR filters,
 *   dmr_tg_filter()), a terminator with LC ends it.
 * - a voice burst: A has the voice SYNC; B - F follow it 60 ms apart (also an A whose SYNC was missed), each with its
 *   EMB (colour code; two bad in a row: the superframe lost) and the embedded LC's fragments: a whole LC starts the
 *   call (late entry), or brings its talker alias and GPS position. The call's voice: the 3 AMBE+2 frames of each burst
 *   through our vocoder into the playback (voice_play.c), one call played at a time.
 * - no voice for CALL_TIMEOUT: the call over.
 *
 * The calls are shown as the engine's (dmr.c: the last heard list, the talker alias). The PTT: the recording off, the
 * HR_C7000 engine transmits (dmr_engine_transmit()), then the recording again. The demodulator's state and the voice
 * queue live after the vocoder's RAM (ckcpu.ld .dmrs_rx, where NXDN's is when NXDN runs).
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "systime.h"
#include "vocoder.h"
#include "p25_phy.h"
#include "dmr_compat.h"
#include "hr_c7000.h"
#include "dmr.h"
#include "afrx.h"
#include "voice_play.h"
#include "dmr_demod.h"
#include "dmr_l2.h"
#include "dmr_soft.h"
#include "ambe.h"

#define PCM_FRAMES    16  // 320 ms of decoded voice
#define PLAY_START    5   // frames queued before the playback starts (they come 3 at a time every 60 ms)
#define CALL_TIMEOUT  400 // ms without a voice burst on the call's timeslot: the call over
#define LOST_TIMEOUT  150 // ... its voice superframe no longer followed (no terminator came)
#define TAP_SAMPLES   2048 // the last samples recorded, for tools over the cable (256 ms)
#define BURST_LOG     32   // the last bursts, for tools over the cable
#define A_MSE_MAX     77   // an A without its SYNC taken (the sync missed) when its symbols fit under 0.30 level^2
                           // (Q8): noise is 0.31 - 0.45, a signal at -116 dBm 0.12 - 0.2
#define TX_END_MS     3000 // the engine's transmission ending (a repeater's hang time) at most, before receiving again

#define OVERLAY_BSS __attribute__((section(".dmrs_bss"))) // after the vocoder's RAM (ckcpu.ld), not zeroed

volatile uint32_t dmr_soft_enabled = 1;
volatile dmr_soft_stats_t dmr_soft_stats;
// tests against known content (over the cable, the transmitter's vocoder_tx_test): not 0, each AMBE+2 frame played
// judged: by its place in the burst, frames, bad (C0 + C1 corrections > 3), the tone frame after the FEC
volatile uint32_t dmr_soft_tone_check;
volatile uint32_t dmr_soft_tone_stats[3][3];

static dmr_demod dm OVERLAY_BSS;
static int16_t pcm[PCM_FRAMES][VOICE_PLAY_SAMPLES] OVERLAY_BSS;
// the recording (ring, dmr_soft_tap_n samples so far; dmr_soft_tap_hold 1: kept as it is, more: samples still to take
// before that)
int16_t dmr_soft_tap[TAP_SAMPLES] OVERLAY_BSS;
volatile uint32_t dmr_soft_tap_n, dmr_soft_tap_hold;
// the last bursts: the 33 bytes, the sync kind, the timeslot, the mse (Q4 level^2, at most 255), slots since a sync
uint8_t dmr_soft_burst_log[BURST_LOG][37] OVERLAY_BSS;
volatile uint32_t dmr_soft_burst_log_n;

typedef struct {
    bool active;      // a call followed: its LC taken
    int pos;          // the last voice burst's place in the superframe: 0 A .. 5 F; -1 none followed
    int bad_emb;      // bursts in a row without a good EMB
    int cc;           // the colour code of the voice followed (its first good EMB; -1 not yet)
    dmr_emb_lc emb;
    TickType_t last;  // the last voice burst or header of the call
} slot_t;

static slot_t slots[2];
static uint32_t burst_n;
static int phase;      // the timeslot of burst n: (n + phase) & 1
static int playing;    // the timeslot played (-1 none)

/* ---- the calls ---- */

static void call_end(int ts)
{
    slot_t *s = &slots[ts];
    s->active = false;
    s->pos = -1;
    if (playing == ts) {
        playing = -1;
        voice_play_end(); // what is queued played out, then the speaker off
        dmr_call_over();
    }
}

// a voice call's LC (a header's, an embedded one): the call on ts followed if the filters take it
static void call_lc(int ts, const uint8_t lc[9])
{
    dmr_lc_t p;
    dmr_lc_parse(lc, &p);
    slot_t *s = &slots[ts];
    if ((p.flco != DMR_FLCO_GROUP) && (p.flco != DMR_FLCO_UNIT)) {
        if (s->active && (playing == ts) && (p.flco >= 4) && (p.flco <= 8)) {
            dmr_lc_heard(lc, (uint8_t)ts); // a talker alias block, a GPS position
        }
        return;
    }
    uint32_t tg_or_pc = (p.flco == DMR_FLCO_UNIT) ? (((uint32_t)PC_CALL_FLAG << 24) | p.dst) : p.dst;
    if (!dmr_tg_filter(tg_or_pc, p.src)) {
        dmr_soft_stats.filtered++;
        if (s->active) {
            call_end(ts);
        }
        return;
    }
    s->last = xTaskGetTickCount();
    if (!s->active) {
        s->active = true;
        dmr_soft_stats.calls++;
        if (playing < 0) {
            playing = ts;
            vocoderReset();
        }
    }
    if (playing == ts) {
        dmr_lc_heard(lc, (uint8_t)ts);
    }
}

static void voice(const uint8_t b[DMR_BURST_BYTES])
{
    uint8_t ambe[3][DMR_AMBE_BYTES];
    dmr_voice_frames(b, ambe);
    uint32_t t0 = systime_get_us();
    for (int f = 0; f < 3; f++) {
        int16_t *out = voice_play_frame();
        if (!out) { // the playback fell behind: the newest frame dropped
            dmr_soft_stats.pcm_drops++;
            continue;
        }
        vocoderDecodeFrame(ambe[f], out);
        voice_play_put();
        if (dmr_soft_tone_check) {
            static const uint8_t tone[DMR_AMBE_BYTES] = {0xCE, 0xA8, 0xFE, 0x83, 0xAC, 0xC4, 0x58, 0x20, 0x0A};
            uint8_t d[49], ref[49];
            int e = ambe_decode_frame(ambe[f], d);
            ambe_decode_frame(tone, ref);
            dmr_soft_tone_stats[f][0]++;
            dmr_soft_tone_stats[f][1] += (e > 3);
            dmr_soft_tone_stats[f][2] += !memcmp(d, ref, sizeof(d));
        }
    }
    uint32_t took = systime_get_us() - t0;
    if (took > dmr_soft_stats.dec_us_max) {
        dmr_soft_stats.dec_us_max = took;
    }
}

/* ---- the bursts ---- */

static bool cc_ok(uint8_t cc)
{
    if ((nonVolatileSettings.dmrCcTsFilter & DMR_CC_FILTER_PATTERN) && (cc != trxGetDMRColourCode())) {
        dmr_soft_stats.other_cc++;
        return false;
    }
    return true;
}

static bool ts_ok(int ts)
{
    return !(nonVolatileSettings.dmrCcTsFilter & DMR_TS_FILTER_PATTERN) || (ts == trxGetDMRTimeSlot());
}

static void log_burst(const dmr_burst_t *b, int ts)
{
    uint8_t *e = dmr_soft_burst_log[dmr_soft_burst_log_n % BURST_LOG];
    memcpy(e, b->bits, DMR_BURST_BYTES);
    e[33] = (uint8_t)b->sync;
    e[34] = (uint8_t)ts;
    int32_t m = b->mse >> 4;
    e[35] = (uint8_t)((m > 255) ? 255 : m);
    e[36] = (uint8_t)((b->since_sync > 255) ? 255 : b->since_sync);
    dmr_soft_burst_log_n++;
}

static void on_burst(void *ctx, const dmr_burst_t *b)
{
    (void)ctx;
    uint32_t n = burst_n++;
    // the timeslot
    if (DMR_SYNC_IS_BS(b->sync)) {
        uint8_t at, tc, lcss;
        dmr_tact(b->cach, &at, &tc, &lcss);
        phase = (int)((tc - n) & 1);
    } else if ((b->sync == DMR_SYNC_TS1_VOICE) || (b->sync == DMR_SYNC_TS1_DATA)) {
        phase = (int)((0 - n) & 1);
    } else if ((b->sync == DMR_SYNC_TS2_VOICE) || (b->sync == DMR_SYNC_TS2_DATA)) {
        phase = (int)((1 - n) & 1);
    } else if (b->sync != DMR_SYNC_NONE) { // MS: one transmitter, on the channel's timeslot
        phase = (int)(((uint32_t)trxGetDMRTimeSlot() - n) & 1);
    }
    int ts = (int)((n + (uint32_t)phase) & 1);
    slot_t *s = &slots[ts];
    log_burst(b, ts);
    if (!ts_ok(ts)) {
        return;
    }
    if (DMR_SYNC_IS_DATA(b->sync)) {
        uint8_t cc, dt, lc[12];
        s->pos = -1;
        if (!dmr_slot_type(b->bits, &cc, &dt) || !cc_ok(cc)) {
            return;
        }
        if ((dt == DMR_DT_VOICE_LC_HEADER) && dmr_bptc196(b->bits, lc) && dmr_full_lc_check(lc, DMR_MASK_VOICE_LC_HEADER)) {
            dmr_soft_stats.headers++;
            call_lc(ts, lc);
        } else if ((dt == DMR_DT_TERMINATOR_LC) && dmr_bptc196(b->bits, lc) &&
                   dmr_full_lc_check(lc, DMR_MASK_TERMINATOR_LC)) {
            dmr_soft_stats.terminators++;
            if (s->active) {
                call_end(ts);
            }
        }
        return;
    }
    // a voice burst: A (its SYNC), or the next of the superframe followed
    if (DMR_SYNC_IS_VOICE(b->sync)) {
        if (s->pos < 0) {
            s->cc = -1;
        }
        s->pos = 0;
        s->bad_emb = 0;
    } else if (s->pos >= 0) {
        s->pos = (s->pos + 1) % 6;
    } else {
        return;
    }
    if (s->pos > 0) {
        uint8_t cc, pi, lcss, frag[4], lc[9];
        // a good EMB of the voice's colour code (a random word passes the QR (16,7,6) one time in 4)
        bool good = dmr_emb(b->bits, &cc, &pi, &lcss) && ((s->cc < 0) || (cc == s->cc));
        if (good) {
            dmr_soft_stats.emb_ok++;
            s->bad_emb = 0;
            if (!cc_ok(cc)) {
                s->pos = -1;
                return;
            }
            s->cc = cc;
            dmr_embedded(b->bits, frag);
            if (dmr_emb_lc_add(&s->emb, lcss, frag, lc)) {
                dmr_soft_stats.emb_lcs++;
                call_lc(ts, lc);
            }
        } else {
            dmr_soft_stats.emb_bad++;
            if (++s->bad_emb >= 2) { // the voice is over (or too weak to follow)
                s->pos = -1;
                return;
            }
        }
    } else if ((b->sync == DMR_SYNC_NONE) && (s->bad_emb || (b->mse > A_MSE_MAX))) {
        s->pos = -1; // an A without its SYNC: not a voice burst seen (the voice over)
        return;
    }
    dmr_soft_stats.voice_bursts++;
    if (s->active) {
        s->last = xTaskGetTickCount();
        if (playing == ts) {
            voice(b->bits);
        }
    }
}

/* ---- the playback (voice_play.c) ---- */

static const voice_play_cfg_t play_cfg = {
    .frames = pcm,
    .count = PCM_FRAMES,
    .start = PLAY_START,
    .underruns = &dmr_soft_stats.underruns,
};

/* ---- receiving (afrx.c runs it) ---- */

static void rx_init(void)
{
    dmr_demod_init(&dm, on_burst, NULL);
    memset(slots, 0, sizeof(slots));
    for (int ts = 0; ts < 2; ts++) {
        slots[ts].pos = -1;
        slots[ts].cc = -1;
        dmr_emb_lc_reset(&slots[ts].emb);
    }
    playing = -1;
    burst_n = 0;
    phase = 0;
    vocoderReset();
}

static void samples(int16_t *x, int n)
{
    for (int i = 0; (i < n) && (dmr_soft_tap_hold != 1); i++) {
        dmr_soft_tap[dmr_soft_tap_n % TAP_SAMPLES] = x[i];
        dmr_soft_tap_n++;
        if (dmr_soft_tap_hold > 1) {
            dmr_soft_tap_hold--;
        }
    }
    dmr_demod_samples(&dm, x, n);
    dmr_soft_stats.bursts = dm.bursts;
    dmr_soft_stats.syncs = dm.syncs;
    dmr_soft_stats.lost = dm.lost;
    dmr_soft_stats.relocks = dm.relocks;
}

static void poll(void)
{
    TickType_t now = xTaskGetTickCount();
    for (int ts = 0; ts < 2; ts++) {
        uint32_t timeout = (slots[ts].pos < 0) ? LOST_TIMEOUT : CALL_TIMEOUT;
        if (slots[ts].active && ((now - slots[ts].last) >= pdMS_TO_TICKS(timeout))) {
            call_end(ts);
        }
    }
}

static void end_call(void)
{
    for (int ts = 0; ts < 2; ts++) {
        if (slots[ts].active) {
            call_end(ts);
        }
    }
}

// The PTT held: the recording and the playback are off (afrx.c); the engine transmits from its receive state from
// scratch, until the PTT is released and its transmission (a repeater's hang time) is over
static void transmit(void)
{
    dmr_engine_resume();
    dmr_engine_transmit(true);
    while (afrx_ptt_held()) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    dmr_engine_transmit(false);
    TickType_t t0 = xTaskGetTickCount();
    while (dmr_engine_transmitting() && ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(TX_END_MS))) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static const afrx_ops_t ops = {
    .bw = AFREC_BW_FLAT, // no band-pass, no de-emphasis (the RRC is ours)
    .overlay = OVERLAY_AMBE,
    .play = &play_cfg,
    .rx_init = rx_init,
    .samples = samples,
    .poll = poll,
    .end_call = end_call,
    .transmit = transmit,
    .stats = {
        .halves = &dmr_soft_stats.halves,
        .missed = &dmr_soft_stats.missed_halves,
        .busy_us_max = &dmr_soft_stats.busy_us_max,
    },
};

bool dmr_soft_wanted(void)
{
    return dmr_soft_enabled && (settingsUsbMode != USB_MODE_HOTSPOT);
}

void dmr_soft_start(void)
{
    afrx_start(&ops);
}

void dmr_soft_stop(void)
{
    if (afrx_running(&ops)) {
        afrx_stop();
        dmr_call_over();
    }
}

bool dmr_soft_running(void)
{
    return afrx_running(&ops);
}

void dmr_soft_ptt(bool on)
{
    afrx_ptt(on);
}

bool dmr_soft_transmitting(void)
{
    return afrx_running(&ops) && afrx_transmitting();
}

bool dmr_soft_has_sync(void)
{
    return afrx_running(&ops) && dm.locked;
}

bool dmr_soft_busy(void)
{
    return afrx_running(&ops) && (slots[0].active || slots[1].active);
}
