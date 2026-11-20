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

// M17 4FSK demodulation from FM discriminator audio at 8 kHz (the HR_C7000's recording, as NXDN's): no OS, no
// hardware, all integer; also built on the PC (m17/c). Dibits out as on the air (+3 1, +1 0, -1 2, -3 3) for
// m17_core.c.
#ifndef M17_DEMOD_H
#define M17_DEMOD_H

#include <stdbool.h>
#include <stdint.h>

#define M17_DEMOD_RING 2048 // samples kept (256 ms; a frame is 320)
#define M17_DEMOD_LOST 5    // given instead of a dibit: the signal lost, the frame receiver to start over

typedef void (*m17_demod_cb)(void *ctx, uint8_t dibit);

// Times in samples Q12, from the start (a symbol is 5/3 samples)
typedef struct {
    int32_t x[M17_DEMOD_RING]; // the samples, x[n % M17_DEMOD_RING]
    uint32_t n;                // samples so far
    int32_t dc_slow;           // the input's long-term mean, Q8 (hunting: the signs of a sync)
    uint32_t hunt_t;           // hunting: the next start time to try (Q12)
    // hunting: the best sync within a few candidates
    bool cand;
    uint32_t cand_t;
    int32_t cand_err, cand_g, cand_dc; // its error (Q8, level^2), gain (Q8, per symbol level), DC
    // locked
    bool locked;
    uint32_t t0;               // the frame's start (its sync's first symbol, Q12)
    int i;                     // the next symbol of the frame (0 .. 191)
    int32_t g, dc;             // gain (Q8, per symbol level: +3 is 3 g), DC
    int misses;                // frames in a row without their sync
    bool confirmed;            // a sync found where the lock said it would be (else a miss ends the lock at once)
    m17_demod_cb cb;
    void *ctx;
    uint32_t syncs, resyncs, lost, eots; // counters
    int32_t last_err;          // the last sync's error (Q8)
} m17_demod;

void m17_demod_init(m17_demod *d, m17_demod_cb cb, void *ctx);
void m17_demod_samples(m17_demod *d, const int16_t *x, int count);

#endif
