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
 * OpenDMR, service: the M17 receiver (m17_radio.h, m17/README.md).
 *
 * As the NXDN receiver (nxdn_radio.c): the modem records its AF input (the FD6818's discriminator, 8 kHz, here without
 * the de-emphasis) in halves of 256 samples; the task reads each half as the modem finishes it into m17_demod.c (4FSK
 * in software: the syncs found by correlation, the timing, gain and DC from each), whose dibits go into m17_core.c
 * (frames, FEC, LSF / LICH). Each stream frame's two Codec 2 3200 frames are decoded (c2dec.c) into a queue of 20 ms
 * frames; a second, small task feeds them into the prompt FIFO (8 kHz, the beeps' path), which plays while the modem
 * records.
 *
 * The demodulator's and the receiver's state, the decoder and the task's stack live in the M17 overlay RAM (ckcpu.ld):
 * the task is created when the receiver starts, after the overlay is loaded, and deleted when it stops.
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "hw.h"
#include "leds.h"
#include "modem.h"
#include "systime.h"
#include "vocoder.h"
#include "radio.h"
#include "radio_hooks.h"
#include "p25_phy.h"
#include "c2dec.h"
#include "m17_core.h"
#include "m17_demod.h"
#include "m17_radio.h"
#include "ui.h"

#define FM_BW         0x0F // the recording's FM_BANDWIDTH: no band-pass, no de-emphasis (NXDN: 0x1F)
#define PCM_FRAMES    16   // 320 ms of decoded voice
#define PLAY_START    4    // frames queued before the playback starts (they come two at a time every 40 ms)
#define CALL_TIMEOUT  400  // ms without a stream frame: the call ended
#define AMP_START_MS  75   // the amplifier on before the voice (sound.c)
#define POLL_MS       4    // the recording looked at this often (a half is 32 ms)
#define VOICE_METRIC  24   // a stream frame with more coded bits wrong: its voice not played (silence)
#define STACK_WORDS   1536 // 6 KB: c2dec ~3.5 KB, the Viterbi 0.5 KB

#define OVERLAY_BSS __attribute__((section(".m17_bss"))) // in the M17 overlay, zeroed by its load

volatile m17_radio_stats_t m17_radio_stats;

/* in the overlay */
static m17_demod dm OVERLAY_BSS;
static m17_rx rx OVERLAY_BSS;
static c2dec_t dec OVERLAY_BSS;
static int16_t pcm[PCM_FRAMES][C2DEC_SAMPLES] OVERLAY_BSS;
static int16_t half[256] OVERLAY_BSS;
static StackType_t task_stack[STACK_WORDS] OVERLAY_BSS;
// for tools over the cable (m17/py/m17_monitor.py): the dibits given to m17_core.c (5: the demodulator lost the
// signal), the last ones; the recording, the last M17_RAW_TAP samples, held RAW_HOLD samples after a call began (the
// call's start in it)
#define M17_TAP     4096
#define M17_RAW_TAP 12000
#define RAW_HOLD    4000
uint8_t m17_tap[M17_TAP] OVERLAY_BSS;
uint32_t m17_tap_n OVERLAY_BSS;
int16_t m17_raw_tap[M17_RAW_TAP] OVERLAY_BSS;
uint32_t m17_raw_tap_n OVERLAY_BSS;
uint32_t m17_raw_hold OVERLAY_BSS; // samples still to take (0: taking all the time; set once a call began)
volatile uint32_t m17_raw_rearm; // over the cable: not 0, the recording tap taking again

static volatile uint32_t pcm_in, pcm_out; // frames put in / taken out (the feeder's)
static uint32_t pcm_pos;                  // samples of pcm[pcm_out] already in the FIFO (the feeder's)

static StaticTask_t task_tcb, feed_tcb;
static StackType_t feed_stack[192];
static TaskHandle_t task, feeder;
static volatile bool running;
static volatile bool busy; // the task is using the overlay

static m17_call_t call;
static TickType_t last_frame;
static volatile TickType_t heard_at;
static volatile bool heard;

/* the playback, the feeder's: 0 off, 1 the amplifier starting, 2 playing */
static volatile int play_state;
static TickType_t play_since;
static volatile bool play_wanted;

/* ---- the call ---- */

static void call_publish(void)
{
    ui_event_t ev = {.type = UI_EVENT_DATA, .what = UI_DATA_SIGNAL};

    ui_post(&ev);
}

static void call_begin(void)
{
    if (!call.active) {
        memset(&call, 0, sizeof(call));
        call.active = true;
        c2dec_init(&dec);
        m17_radio_stats.calls++;
        if (!m17_raw_hold) {
            m17_raw_hold = RAW_HOLD;
        }
        call_publish();
    }
}

static void call_end(void)
{
    call.active = false;
    play_wanted = false; // the feeder plays out what is queued, then stops
    call_publish();
}

// the LSF (from the frame or the LICH): the caller, and whether it is voice we can play
static void lsf(void)
{
    char src[10], dst[10];
    m17_address(rx.lsf + 6, src);
    m17_address(rx.lsf, dst);
    rx.lsf_valid = false;
    if (!call.active) {
        return;
    }
    bool muted = !m17_lsf_voice(rx.lsf);
    if (strcmp(src, call.src) || strcmp(dst, call.dst) || (muted != call.muted)) {
        memcpy(call.src, src, sizeof(src));
        memcpy(call.dst, dst, sizeof(dst));
        call.muted = muted;
        call_publish();
    }
}

/* ---- the voice ---- */

static void voice(const uint8_t stream[M17_STREAM_BYTES], bool good)
{
    uint32_t t0 = systime_get_us();
    for (int h = 0; h < 2; h++) {
        if ((pcm_in - pcm_out) >= PCM_FRAMES) { // the feeder fell behind: the newest frame dropped
            m17_radio_stats.pcm_drops++;
            continue;
        }
        int16_t *out = pcm[pcm_in % PCM_FRAMES];
        if (good && !call.muted) {
            c2dec_decode(&dec, stream + C2DEC_BYTES * h, out);
        } else {
            memset(out, 0, sizeof(pcm[0]));
        }
        pcm_in++;
        m17_radio_stats.c2_frames++;
    }
    uint32_t us = systime_get_us() - t0;
    if (us > m17_radio_stats.decode_us_max) {
        m17_radio_stats.decode_us_max = us;
    }
    if ((pcm_in - pcm_out) >= PLAY_START) {
        play_wanted = true;
    }
}

static void on_dibit(void *ctx, uint8_t d)
{
    (void)ctx;
    m17_tap[m17_tap_n % M17_TAP] = d;
    m17_tap_n++;
    if (d == M17_DEMOD_LOST) {
        m17_rx_reset(&rx);
        return;
    }
    switch (m17_rx_dibit(&rx, d)) {
    case M17_RX_LSF:
        m17_radio_stats.lsfs++;
        call_begin();
        last_frame = xTaskGetTickCount();
        lsf();
        break;
    case M17_RX_STREAM:
        m17_radio_stats.streams++;
        if (rx.metric > VOICE_METRIC) {
            m17_radio_stats.bad_frames++;
            if (!call.active) {
                break; // noise taken for a frame
            }
        }
        last_frame = xTaskGetTickCount();
        heard_at = last_frame;
        heard = true;
        call_begin();
        if (rx.lsf_valid) {
            lsf();
        }
        voice(rx.stream, rx.metric <= VOICE_METRIC);
        if (rx.eos) {
            call_end();
        }
        break;
    case M17_RX_EOT:
        m17_radio_stats.eots++;
        if (call.active) {
            call_end();
        }
        break;
    default:
        break;
    }
}

/* ---- the playback: the prompt FIFO (8 kHz, 16 bits little endian, MODEM_BEEP_BLOCK bytes a write) ---- */

static void play_on(bool on)
{
    if (on) {
        radio_beep_path(true);
        modem_beep(true);
        radio_dmr_audio(true);
        leds_green(true);
    } else {
        radio_dmr_audio(false);
        modem_beep(false);
        radio_beep_path(false);
        leds_green(false);
    }
}

static void feed_task(void *arg)
{
    uint8_t block[MODEM_BEEP_BLOCK];

    (void)arg;
    for (;;) {
        vTaskDelay(1);
        int state = play_state;
        if (state == 0) {
            if (running && play_wanted) {
                play_on(true);
                play_since = xTaskGetTickCount();
                play_state = 1;
            }
            continue;
        }
        if (state == 1) { // the amplifier starting: silence meanwhile, the voice kept queued
            if (!modem_beep_fifo_full()) {
                memset(block, 0, sizeof(block));
                modem_beep_write(block);
            }
            if ((xTaskGetTickCount() - play_since) >= pdMS_TO_TICKS(AMP_START_MS)) {
                play_state = 2;
            }
            continue;
        }
        while (running && !modem_beep_fifo_full()) {
            if (pcm_in == pcm_out) {
                if (!play_wanted) {
                    break;
                }
                m17_radio_stats.underruns++;
                memset(block, 0, sizeof(block));
                modem_beep_write(block);
                break;
            }
            const int16_t *frame = pcm[pcm_out % PCM_FRAMES];
            for (int i = 0; i < (MODEM_BEEP_BLOCK / 2); i++) {
                int16_t s = frame[pcm_pos + (uint32_t)i];
                block[2 * i] = (uint8_t)s;
                block[2 * i + 1] = (uint8_t)((uint16_t)s >> 8);
            }
            modem_beep_write(block);
            pcm_pos += MODEM_BEEP_BLOCK / 2;
            if (pcm_pos >= C2DEC_SAMPLES) {
                pcm_pos = 0;
                pcm_out++;
            }
        }
        if (!running || (!play_wanted && (pcm_in == pcm_out))) {
            play_on(false);
            pcm_pos = 0;
            if (!running) {
                pcm_out = pcm_in;
            }
            play_state = 0;
        }
    }
}

/* ---- the receive task ---- */

static void task_main(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        busy = true;
        if (!running) {
            busy = false;
            continue;
        }
        while (running && nxdn_fm_read(half)) {
            uint32_t t0 = systime_get_us();
            m17_radio_stats.halves++;
            if (m17_raw_rearm) {
                m17_raw_rearm = 0;
                m17_raw_hold = 0;
            }
            for (int k = 0; (k < 256) && (m17_raw_hold != 1); k++) {
                m17_raw_tap[m17_raw_tap_n % M17_RAW_TAP] = half[k];
                m17_raw_tap_n++;
                if (m17_raw_hold > 1) {
                    m17_raw_hold--;
                }
            }
            m17_demod_samples(&dm, half, 256);
            m17_radio_stats.syncs = dm.syncs;
            m17_radio_stats.lost = dm.lost;
            uint32_t took = systime_get_us() - t0;
            if (took > m17_radio_stats.busy_us_max) {
                m17_radio_stats.busy_us_max = took;
            }
        }
        if (call.active && ((xTaskGetTickCount() - last_frame) >= pdMS_TO_TICKS(CALL_TIMEOUT))) {
            call_end();
        }
        busy = false;
    }
}

void m17_radio_start(void)
{
    if (running) {
        return;
    }
    if (!feeder) {
        feeder = xTaskCreateStatic(feed_task, "m17a", sizeof(feed_stack) / sizeof(feed_stack[0]), NULL,
                                   tskIDLE_PRIORITY + 4, feed_stack, &feed_tcb);
    }
    if (!nxdn_fm_start_bw(FM_BW)) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    overlay_use(OVERLAY_M17);
    m17_demod_init(&dm, on_dibit, NULL);
    m17_rx_init(&rx);
    c2dec_init(&dec);
    memset(&call, 0, sizeof(call));
    pcm_in = pcm_out = 0;
    pcm_pos = 0;
    play_wanted = false;
    heard = false;
    running = true;
    task = xTaskCreateStatic(task_main, "m17", STACK_WORDS, NULL, tskIDLE_PRIORITY + 3, task_stack, &task_tcb);
}

void m17_radio_stop(void)
{
    if (!running) {
        return;
    }
    running = false;
    while (busy || (play_state != 0)) { // the tasks done with the overlay and the prompt path
        vTaskDelay(1);
    }
    TaskHandle_t t = task;
    task = NULL;
    vTaskDelete(t);
    nxdn_fm_stop();
    call.active = false;
    call_publish();
}

bool m17_radio_call(m17_call_t *out)
{
    *out = call;
    return call.active;
}

bool m17_radio_heard_within(uint32_t ms)
{
    return heard && ((xTaskGetTickCount() - heard_at) < pdMS_TO_TICKS(ms));
}
