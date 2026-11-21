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
 * OpenDMR, service: the voice playback of the NXDN and M17 receivers (voice_play.h).
 *
 * The feeder task, every tick (1 ms): off until a call begins (voice_play_prepare()) or enough frames are queued (start
 * frames, start_ms after the call's first: the bursts come with a jitter of the recording's 16 ms quarters); then the
 * speaker on as sound.c turns it on
 * for a beep: the audio switch to the baseband and the prompt path first, silence for PATH_SETTLE_MS, then the amplifier (and the
 * green LED), silence for AMP_START_MS while it starts; then one block (16 samples: twice real time) a tick while the
 * FIFO has room, silence until the voice is enough (the speaker's start while the first frames come: 85 ms less
 * delay, 2026-11-20, tools/debug/voice_delay.py). One block only: the FIFO's full flag comes late, several at once
 * overran it and samples were lost
 * (heard as the voice shaking, as pops; NXDN and M17 both, 2026-11). Out of voice while the call goes on: silence; the
 * call over and the queue played out: the amplifier off first, AMP_STOP_MS, then the path back. All at once (path and
 * amplifier together) the amplifier caught the switching: a pop at each call's start (2026-11-20).
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "leds.h"
#include "modem.h"
#include "p25_radio.h"
#include "radio_hooks.h"
#include "voice_play.h"

#define PATH_SETTLE_MS 10 // the audio path switched, silence into the FIFO, before the amplifier goes on (sound.c)
#define AMP_START_MS   75 // the amplifier on before the voice
#define AMP_STOP_MS    20 // the amplifier off, before the audio path switches back

enum { OFF, PATH, AMP, PLAY, STOP }; // the feeder's states

static StaticTask_t feed_tcb;
static StackType_t feed_stack[192];
static TaskHandle_t feeder;

static voice_play_cfg_t cfg;
static volatile bool enabled;           // a receiver's playback allowed
static volatile bool wanted;            // enough queued: play (until the call's end)
static volatile bool call_on;           // a call begun (voice_play_prepare()): the speaker on, waiting for its voice
static volatile bool have_first;        // the call's first frame queued, at first_at
static volatile TickType_t first_at;
static volatile uint32_t pcm_in, pcm_out; // frames put in / taken out (the feeder's)
static uint32_t pcm_pos;                // samples of frames[pcm_out] already in the FIFO (the feeder's)
static volatile int state;              // the feeder's (OFF ... STOP)
static TickType_t since;
static bool in_underrun; // the delay log: an underrun going on
static bool first_block; // the delay log (p25_radio.c): the first voice block of a playback not yet written

// the audio switch and the prompt path
static void path(bool on)
{
    if (on) {
        radio_beep_path(true);
        modem_beep(true);
    } else {
        modem_beep(false);
        radio_beep_path(false);
    }
}

// the amplifier (and the LED)
static void amp(bool on)
{
    radio_dmr_audio(on);
    leds_green(on);
}

// a new state from now
static void go(int next)
{
    state = next;
    since = xTaskGetTickCount();
}

static bool waited(uint32_t ms)
{
    return (xTaskGetTickCount() - since) >= pdMS_TO_TICKS(ms);
}

static void silence(uint8_t *block)
{
    if (!modem_beep_fifo_full()) {
        memset(block, 0, MODEM_BEEP_BLOCK);
        modem_beep_write(block);
    }
}

static void count(volatile uint32_t *counter)
{
    if (counter) {
        (*counter)++;
    }
}

static void feed_task(void *arg)
{
    uint8_t block[MODEM_BEEP_BLOCK];

    (void)arg;
    for (;;) {
        vTaskDelay(1);
        if (!wanted && have_first && ((pcm_in - pcm_out) >= cfg.start) &&
            ((xTaskGetTickCount() - first_at) >= pdMS_TO_TICKS(cfg.start_ms))) {
            p25_ev_log(50, (uint16_t)(pcm_in - pcm_out)); // the delay log (p25_radio.c): enough queued
            wanted = true;
        }
        if (state == OFF) {
            if (enabled && (wanted || call_on)) {
                path(true);
                go(PATH);
            }
            continue;
        }
        if (state == STOP) { // the amplifier off: then the path back
            if (waited(AMP_STOP_MS)) {
                path(false);
                pcm_pos = 0;
                if (!enabled) {
                    pcm_out = pcm_in;
                }
                state = OFF;
            }
            continue;
        }
        if (!enabled) { // stopped while starting or playing
            amp(false);
            go(STOP);
            continue;
        }
        if (state == PATH) { // the path settling, the voice kept queued
            silence(block);
            if (waited(PATH_SETTLE_MS)) {
                p25_ev_log(51, (uint16_t)(pcm_in - pcm_out)); // the amplifier on (arg: frames queued)
                amp(true);
                go(AMP);
            }
            continue;
        }
        if (state == AMP) { // the amplifier starting
            silence(block);
            if (waited(AMP_START_MS)) {
                if (wanted) {
                    first_block = true;
                    go(PLAY);
                } else if (!call_on) { // the call over before its voice was enough
                    amp(false);
                    go(STOP);
                }
            }
            continue;
        }
        if (enabled && !modem_beep_fifo_full()) {
            if (pcm_in == pcm_out) {
                if (wanted) { // out of voice, the call going on
                    if (!in_underrun) {
                        in_underrun = true;
                        p25_ev_log(53, (uint16_t)pcm_out); // the delay log (p25_radio.c): out of voice (arg: frames
                    }                                      // played so far)
                    count(cfg.underruns);
                    memset(block, 0, sizeof(block));
                    modem_beep_write(block);
                }
            } else {
                if (first_block) {
                    first_block = false;
                    p25_ev_log(52, (uint16_t)(pcm_in - pcm_out)); // the first voice to the speaker (arg: queued)
                }
                in_underrun = false;
                const int16_t *frame = &cfg.frames[pcm_out % cfg.count][pcm_pos];
                for (int i = 0; i < (MODEM_BEEP_BLOCK / 2); i++) {
                    block[2 * i] = (uint8_t)frame[i];
                    block[2 * i + 1] = (uint8_t)((uint16_t)frame[i] >> 8);
                }
                modem_beep_write(block);
                if (cfg.tap) {
                    cfg.tap(frame, MODEM_BEEP_BLOCK / 2);
                }
                count(cfg.writes);
                pcm_pos += MODEM_BEEP_BLOCK / 2;
                if (pcm_pos >= VOICE_PLAY_SAMPLES) {
                    pcm_pos = 0;
                    pcm_out++;
                }
            }
        }
        if (!wanted && (pcm_in == pcm_out)) { // the call over, played out
            amp(false);
            go(STOP);
        }
    }
}

void voice_play_start(const voice_play_cfg_t *c)
{
    if (!feeder) {
        feeder = xTaskCreateStatic(feed_task, "voice", sizeof(feed_stack) / sizeof(feed_stack[0]), NULL,
                                   tskIDLE_PRIORITY + 4, feed_stack, &feed_tcb);
    }
    voice_play_stop();
    cfg = *c;
    pcm_in = pcm_out = 0;
    pcm_pos = 0;
    wanted = false;
    call_on = false;
    have_first = false;
    enabled = true;
}

void voice_play_stop(void)
{
    enabled = false;
    wanted = false;
    call_on = false;
    have_first = false;
    while (state != OFF) { // the feeder off the prompt path
        vTaskDelay(1);
    }
    pcm_out = pcm_in;
}

int16_t *voice_play_frame(void)
{
    if (!enabled || ((pcm_in - pcm_out) >= cfg.count)) {
        return NULL;
    }
    return cfg.frames[pcm_in % cfg.count];
}

void voice_play_prepare(void)
{
    call_on = true;
}

void voice_play_put(void)
{
    pcm_in++;
    if (!have_first) { // the call's first frame: the feeder starts the playback once enough came (start, start_ms)
        first_at = xTaskGetTickCount();
        have_first = true;
    }
}

void voice_play_end(void)
{
    wanted = false;
    call_on = false;
    have_first = false;
}
