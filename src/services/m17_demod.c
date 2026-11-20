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
 * OpenDMR, service: M17 demodulation (m17_demod.h) from the FM discriminator at 8 kHz: 4800 symbols/s, a symbol every
 * 5/3 samples, each taken by cubic (Catmull-Rom) interpolation at its time (the recording is already band-limited:
 * measured, no receive filter does better, m17/README.md).
 *
 * Hunting: every third of a sample a start is tried: the 8 symbols of the LSF or the stream sync burst (all outer:
 * +-3) by their signs at the nearest samples first, then a least squares fit of gain and DC to the interpolated values
 * and its error; the best of the candidates close together, refined to a sixth of a sample, is the frame's start.
 * Locked: the frame's 192 symbols sliced with that gain and DC (thresholds 0, +-2 gain); the next sync looked for
 * within 2 samples of where it should be (a sixth of a sample apart), the timing, gain and DC taken from it; up to
 * MAX_MISSES frames without it are sliced where they should be. An EoT sync ends the lock.
 */

#include <string.h>

#include "m17_demod.h"

#define Q           12
#define ONE         (1 << Q)
#define FRAME       192
#define FRAME_T     1310720U            // 192 symbols, Q12 (192 * 5/3 samples)
#define HUNT_STEP   1365                // a third of a sample, Q12
#define FINE_STEP   683                 // a sixth
#define TRACK       12                  // the next sync looked for within +-TRACK sixths of a sample (2 samples)
#define MAX_MISSES  3
#define G_MIN       (100 << 8)          // gain at least (Q8, per symbol level; samples halved: an outer +-3 ~ 3800)
#define ERR_MAX     128                 // the fit's mean squared error at most, Q8 level^2 (0.5)
#define HUNT_ERR    48                  // the same hunting (0.19: noise found syncs at 0.5; a signal's are 0.03 - 0.15)
#define REBASE_AT   (1U << 19)          // samples: the Q12 times kept within 32 bits
#define REBASE_BY   (1U << 18)

static const int8_t LSF[8] = {3, 3, 3, 3, -3, -3, 3, -3};    // 0x55F7; the stream sync 0xFF5D is its negative
static const int8_t STREAM[8] = {-3, -3, -3, -3, 3, 3, -3, 3};
static const int8_t EOT[8] = {3, 3, 3, 3, 3, 3, -3, 3};      // 0x555D

// the time of symbol i of a frame starting at t (Q12)
static uint32_t sym_t(uint32_t t, int i)
{
    return t + ((uint32_t)i * 20480U + 1U) / 3U;
}

static int32_t at(const m17_demod *d, uint32_t n)
{
    return d->x[n % M17_DEMOD_RING];
}

// the signal at time t (Q12), cubic interpolation; needs samples up to t + 2
static int32_t sample(const m17_demod *d, uint32_t t)
{
    uint32_t i = t >> Q;
    int32_t f = (int32_t)(t & (ONE - 1));
    int32_t p0 = at(d, i - 1), p1 = at(d, i), p2 = at(d, i + 1), p3 = at(d, i + 2);
    int32_t a = (p2 - p0) / 2;
    int32_t b = p0 - (5 * p1) / 2 + 2 * p2 - p3 / 2;
    int32_t c = (3 * (p1 - p2) + p3 - p0) / 2;
    return p1 + ((((((c * f) >> Q) + b) * f) >> Q) + a) * f / ONE;
}

static bool available(const m17_demod *d, uint32_t t)
{
    return ((t >> Q) + 3) <= d->n;
}

// Gain (Q8) and DC by least squares of the 8 symbols at t to the pattern, the error (Q8, mean squared, in levels^2);
// false: no fit (too weak, or the error too large)
static bool fit(const m17_demod *d, uint32_t t, const int8_t *p, int32_t *g, int32_t *dc, int32_t *err, int32_t max)
{
    int32_t v[8], sp = 0, sv = 0, spv = 0;
    for (int k = 0; k < 8; k++) {
        v[k] = sample(d, sym_t(t, k));
        sp += p[k];
        sv += v[k];
        spv += p[k] * v[k];
    }
    int32_t det = 8 * 72 - sp * sp;
    int32_t gq = (int32_t)(((int64_t)(8 * spv - sp * sv) << 8) / det);
    if (gq < G_MIN) {
        return false;
    }
    int32_t dcv = (int32_t)(((int64_t)sv * 256 - (int64_t)gq * sp) / (8 * 256));
    int64_t e2 = 0;
    for (int k = 0; k < 8; k++) {
        int32_t e = v[k] - ((gq * p[k]) >> 8) - dcv;
        e2 += (int64_t)e * e;
    }
    // (e2 / 8) / (gq / 256)^2, Q8
    int64_t gg = ((int64_t)gq * gq) >> 8;
    int32_t en = (int32_t)((e2 * 256 * 32) / (gg ? gg : 1));
    *g = gq;
    *dc = dcv;
    *err = en;
    return en < max;
}

static uint8_t slice(const m17_demod *d, int32_t v)
{
    int32_t two = d->g >> 7; // 2 gain
    v -= d->dc;
    return (v >= two) ? 1 : (v >= 0) ? 0 : (v >= -two) ? 2 : 3;
}

void m17_demod_init(m17_demod *d, m17_demod_cb cb, void *ctx)
{
    memset(d, 0, sizeof(*d));
    d->cb = cb;
    d->ctx = ctx;
    d->hunt_t = 4U << Q;
}

static void lock(m17_demod *d, uint32_t t, int32_t g, int32_t dc)
{
    d->locked = true;
    d->t0 = t;
    d->i = 0;
    d->g = g;
    d->dc = dc;
    d->misses = 0;
    d->confirmed = false;
    d->cand = false;
    d->syncs++;
}

static void unlock(m17_demod *d, uint32_t from)
{
    d->locked = false;
    d->cand = false;
    d->hunt_t = from;
}

// hunting: the candidates whose symbols are all in
static void hunt(m17_demod *d)
{
    while (!d->locked && available(d, sym_t(d->hunt_t, 7))) {
        uint32_t t = d->hunt_t;
        d->hunt_t += HUNT_STEP;
        // the signs at the nearest samples against the LSF pattern (the stream sync's are the opposite)
        int32_t ref = d->dc_slow >> 8;
        int same = 0;
        for (int k = 0; k < 8; k++) {
            uint32_t s = (sym_t(t, k) + (ONE / 2)) >> Q;
            same += ((at(d, s) > ref) == (LSF[k] > 0));
        }
        const int8_t *p = (same >= 7) ? LSF : (same <= 1) ? STREAM : NULL;
        int32_t g, dc, err;
        if (p && fit(d, t, p, &g, &dc, &err, HUNT_ERR)) {
            if (!d->cand || err < d->cand_err) {
                d->cand = true;
                d->cand_t = t;
                d->cand_err = err;
                d->cand_g = g;
                d->cand_dc = dc;
            }
        }
        if (d->cand && ((int32_t)(t - d->cand_t) > 2 * ONE)) { // the best of those close together: refined
            const int8_t *best_p = NULL;
            for (int s = -1; s <= 1; s += 2) {
                uint32_t tt = d->cand_t + (uint32_t)(s * FINE_STEP);
                for (int q = 0; q < 2; q++) {
                    const int8_t *pp = q ? STREAM : LSF;
                    if (fit(d, tt, pp, &g, &dc, &err, HUNT_ERR) && (err < d->cand_err)) {
                        d->cand_t = tt;
                        d->cand_err = err;
                        d->cand_g = g;
                        d->cand_dc = dc;
                        best_p = pp;
                    }
                }
            }
            (void)best_p;
            d->last_err = d->cand_err;
            lock(d, d->cand_t, d->cand_g, d->cand_dc);
        }
    }
}

// locked: the symbols in; at a frame's end the next one's sync
static void track(m17_demod *d)
{
    while (d->locked) {
        if (d->i < FRAME) {
            uint32_t t = sym_t(d->t0, d->i);
            if (!available(d, t)) {
                return;
            }
            d->cb(d->ctx, slice(d, sample(d, t)));
            d->i++;
            continue;
        }
        uint32_t t1 = d->t0 + FRAME_T;
        if (!available(d, sym_t(t1 + (uint32_t)(TRACK * FINE_STEP), 7))) {
            return;
        }
        int32_t best_err = ERR_MAX, bg = 0, bdc = 0;
        uint32_t bt = 0;
        const int8_t *bp = NULL;
        for (int s = -TRACK; s <= TRACK; s++) {
            uint32_t tt = t1 + (uint32_t)(s * FINE_STEP);
            for (int q = 0; q < 2; q++) {
                const int8_t *pp = q ? EOT : STREAM;
                int32_t g, dc, err;
                if (fit(d, tt, pp, &g, &dc, &err, ERR_MAX) && (err < best_err)) {
                    best_err = err;
                    bt = tt;
                    bg = g;
                    bdc = dc;
                    bp = pp;
                }
            }
        }
        if (bp) {
            d->t0 = bt;
            d->g = (d->g + bg) / 2;
            d->dc = (d->dc + bdc) / 2;
            d->misses = 0;
            d->confirmed = true;
            d->resyncs++;
            d->last_err = best_err;
            if (bp == EOT) { // its 8 symbols out (the frame receiver ends the stream), then hunting
                for (int k = 0; k < 8; k++) {
                    d->cb(d->ctx, slice(d, sample(d, sym_t(bt, k))));
                }
                d->eots++;
                unlock(d, sym_t(bt, 8));
                return;
            }
        } else {
            d->t0 = t1;
            if (!d->confirmed || (++d->misses > MAX_MISSES)) {
                d->lost++;
                d->cb(d->ctx, M17_DEMOD_LOST);
                unlock(d, t1);
                return;
            }
        }
        d->i = 0;
    }
}

void m17_demod_samples(m17_demod *d, const int16_t *x, int count)
{
    for (int k = 0; k < count; k++) {
        int32_t s = x[k] >> 1; // halved: the interpolation's products within 32 bits
        d->x[d->n % M17_DEMOD_RING] = s;
        d->n++;
        d->dc_slow += ((s << 8) - d->dc_slow) >> 10;
    }
    // the hunt never looks further back than the ring holds
    uint32_t oldest = (d->n > M17_DEMOD_RING - 8) ? ((d->n - M17_DEMOD_RING + 8) << Q) : 0;
    if (!d->locked && ((int32_t)(d->hunt_t - oldest) < 0)) {
        d->hunt_t = oldest;
        d->cand = false;
    }
    hunt(d);
    track(d);
    hunt(d); // after an unlock
    if (d->n >= REBASE_AT) {
        d->n -= REBASE_BY;
        d->hunt_t -= REBASE_BY << Q;
        d->t0 -= REBASE_BY << Q;
        d->cand_t -= REBASE_BY << Q;
    }
}
