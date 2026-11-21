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
 * OpenDMR, service: P25 Phase 1 C4FM demodulation (p25_demod.h), as worked out on the radio's recordings (p25_soft/py
 * demod.py: our transmitter's test frames, 0 symbols wrong of 6912 with a strong signal).
 *
 * - the samples as recorded (no filter), into fsk4.c's ring; a symbol every 5/3 samples, read by cubic interpolation.
 * - hunting: every third of a sample, the frame sync's 24 symbols (0x5575F5FF77FF, both polarities) first by their signs
 *   at the nearest samples, then a least squares fit of gain and DC; the best of those close together, refined to a
 *   sixth of a sample, starts the time line at its first symbol.
 * - locked: each symbol from the time line, scaled by the gain and DC, decided, out. The last 24 dibits out the sync
 *   (2 wrong at most): it fitted again within a sample of where it is (the timing, gain and DC from it). No sync for
 *   LOST_SYMS: lost. The hunt goes on while locked once no sync came for HUNT_SYMS (another transmitter, its timing).
 */

#include <string.h>

#include "p25_demod.h"

#define Q          FSK4_Q
#define ONE        FSK4_ONE
#define HUNT_STEP  1365              // a third of a sample, Q12
#define FINE_STEP  683               // a sixth
#define TRACK      6                 // a sync refined within +-TRACK sixths of a sample
#define G_MIN      (80 << 8)         // gain at least (Q8 a level)
#define HUNT_ERR   154               // the fit's mean squared error under 0.6 level^2 (Q8) hunting ...
#define TRACK_ERR  200               // ... 0.78 where the dibits say a sync is
#define HUNT_SIGNS 21                // of the 24 symbols' signs at least (or at most 24 - this: the other polarity)
#define SYNC_WRONG 2                 // dibits of the sync wrong at most, locked
#define HUNT_SYMS  900               // locked without a sync this long: hunting too (longer than an LDU, 864)
#define LOST_SYMS  2000              // ... and lost
#define REBASE_AT  (1U << 18)        // samples: the Q12 times kept within int32
#define REBASE_BY  (1U << 17)        // a multiple of the ring
#define SYNC_WORD  0x5575F5FF77FFULL // 24 dibits, the first sent the highest

static int8_t pattern[24];       // the sync's levels
static uint64_t sign_mask[3], sign_positive[3];
static bool ready;

static int8_t level_of(unsigned dibit)
{
    static const int8_t lv[4] = {1, 3, -1, -3}; // 00 +1, 01 +3, 10 -1, 11 -3
    return lv[dibit & 3];
}

static void make_pattern(void)
{
    if (ready) {
        return;
    }
    for (int k = 0; k < 24; k++) {
        pattern[k] = level_of((unsigned)(SYNC_WORD >> (46 - 2 * k)));
    }
    for (int ph = 0; ph < 3; ph++) {
        for (int k = 0; k < 24; k++) {
            uint64_t bit = 1ULL << ((ph + 5 * k + 1) / 3);
            sign_mask[ph] |= bit;
            if (pattern[k] > 0) {
                sign_positive[ph] |= bit;
            }
        }
    }
    ready = true;
}

static int32_t sym_t(int32_t t, int k)
{
    return fsk4_sym_t(t, k, FSK4_SPS_4800_NUM, FSK4_SPS_DEN);
}

void p25_demod_init(p25_demod *d, p25_demod_cb cb, void *ctx)
{
    make_pattern();
    memset(d, 0, sizeof(*d));
    fsk4_ring_init(&d->x, d->buf, P25_DEMOD_RING);
    d->cb = cb;
    d->ctx = ctx;
    d->hunt_t = 8 << Q;
    d->sign = 1;
}

// the sync at t (its first symbol) fitted with sign; true: under max
static bool fit_sync(const p25_demod *d, int32_t t, int sign, int32_t max, fsk4_fit_t *f)
{
    int32_t v[24];
    int8_t p[24];
    for (int k = 0; k < 24; k++) {
        v[k] = fsk4_at(&d->x, sym_t(t, k));
        p[k] = (int8_t)(sign * pattern[k]);
    }
    return fsk4_fit(v, p, 24, G_MIN, max, f);
}

// 64 of the samples' sign bits from sample i on (bit k: sample i + k)
static uint64_t sign_window(const p25_demod *d, uint32_t i)
{
    const uint32_t words = P25_DEMOD_RING / 32;
    uint32_t w = (i & d->x.mask) >> 5, b = i & 31;
    uint64_t lo = d->signs[w] | ((uint64_t)d->signs[(w + 1) % words] << 32);
    if (!b) {
        return lo;
    }
    return (lo >> b) | ((uint64_t)d->signs[(w + 2) % words] << (64 - b));
}

static int32_t oldest_t(const p25_demod *d)
{
    return (d->x.n > P25_DEMOD_RING - 16) ? (int32_t)((d->x.n - P25_DEMOD_RING + 16) << Q) : 0;
}

// a time line from a sync starting at t. Locked already: the dibits go on from where they are (none twice, none
// skipped: the receiver needs them in a row), on the new line
static void lock_at(p25_demod *d, int32_t t, int sign, const fsk4_fit_t *f)
{
    int k = 0;
    if (d->locked) {
        d->relocks++;
        int64_t ahead = (int64_t)t - sym_t(d->t, d->k); // the sync's start from the next symbol due, Q12
        k = -(int)((ahead * FSK4_SPS_DEN + (ahead >= 0 ? FSK4_SPS_4800_NUM / 2 : -FSK4_SPS_4800_NUM / 2)) /
                   FSK4_SPS_4800_NUM);
    }
    d->locked = true;
    d->t = t;
    d->k = k;
    d->sign = sign;
    d->g = f->g;
    d->dc = f->dc;
    d->since_sync = 0;
    d->syncs++;
}

// hunting (not locked, or locked without a sync for HUNT_SYMS): the candidates whose symbols are all in
static void hunt(p25_demod *d)
{
    while (fsk4_have(&d->x, sym_t(d->hunt_t, 24) + FINE_STEP)) {
        if (d->locked && (d->since_sync < HUNT_SYMS)) {
            d->cand = false;
            d->hunt_t = sym_t(d->t, d->k); // kept up with the time line
            return;
        }
        int32_t t = d->hunt_t;
        uint32_t i0 = (uint32_t)(t >> Q);
        int ph = ((t & (ONE - 1)) + HUNT_STEP / 2) / HUNT_STEP;
        if (ph >= 3) {
            i0++;
            ph = 0;
        }
        d->hunt_t = (ph == 2) ? (int32_t)((i0 + 1) << Q) : (int32_t)((i0 << Q) + (uint32_t)(ph + 1) * HUNT_STEP);
        uint64_t win = sign_window(d, i0) & sign_mask[ph];
        int same = 24 - __builtin_popcountll(win ^ sign_positive[ph]);
        int sign = (same >= HUNT_SIGNS) ? 1 : (same <= 24 - HUNT_SIGNS) ? -1 : 0;
        fsk4_fit_t f;
        if (sign && fit_sync(d, t, sign, HUNT_ERR, &f) && (!d->cand || (f.err < d->cand_fit.err))) {
            d->cand = true;
            d->cand_t = t;
            d->cand_sign = sign;
            d->cand_fit = f;
        }
        if (d->cand && ((t - d->cand_t) > 2 * ONE)) { // the best of those close together, refined
            int32_t c = d->cand_t;
            for (int s = -2; s <= 2; s++) {
                fsk4_fit_t g;
                if (s && fit_sync(d, c + s * FINE_STEP / 2, d->cand_sign, d->cand_fit.err, &g)) {
                    d->cand_t = c + s * FINE_STEP / 2;
                    d->cand_fit = g;
                }
            }
            d->cand = false;
            lock_at(d, d->cand_t, d->cand_sign, &d->cand_fit);
        }
    }
}

static unsigned decide(int32_t z) // z: Q8 levels
{
    return (z >= 512) ? 1 : (z >= 0) ? 0 : (z >= -512) ? 2 : 3;
}

// locked: the symbols whose samples are in, out
static void track(p25_demod *d)
{
    while (d->locked && fsk4_have(&d->x, sym_t(d->t, d->k + 1) + TRACK * FINE_STEP)) {
        int32_t gs = (d->g >> 8) ? (d->g >> 8) : 1;
        int32_t z = d->sign * (fsk4_at(&d->x, sym_t(d->t, d->k)) - d->dc) * 256 / gs;
        unsigned db = decide(z);
        d->k++;
        d->since_sync++;
        d->symbols++;
        d->last = (d->last << 2) | db;
        d->cb(d->ctx, (int)db);
        // the sync just out: its place and the levels again from it
        uint64_t diff = (d->last ^ SYNC_WORD) & 0xFFFFFFFFFFFFULL;
        int wrong = 0;
        for (int i = 0; (i < 24) && (wrong <= SYNC_WRONG); i++) {
            wrong += ((diff >> (2 * i)) & 3) != 0;
        }
        if ((wrong <= SYNC_WRONG) && (d->since_sync >= 24)) {
            int32_t due = sym_t(d->t, d->k - 24), best_t = 0, best_err = TRACK_ERR;
            fsk4_fit_t best = {0, 0, 0};
            for (int s = -TRACK; s <= TRACK; s++) {
                fsk4_fit_t f;
                if (fit_sync(d, due + s * FINE_STEP, d->sign, best_err, &f)) {
                    best_err = f.err;
                    best = f;
                    best_t = due + s * FINE_STEP;
                }
            }
            if (best_err < TRACK_ERR) {
                d->t = best_t;
                d->k = 24;
                d->g = best.g;
                d->dc = best.dc;
                d->since_sync = 0;
                d->syncs++;
            }
        }
        if (d->since_sync >= LOST_SYMS) {
            d->locked = false;
            d->lost++;
            d->hunt_t = sym_t(d->t, d->k);
            d->cb(d->ctx, P25_DEMOD_LOST);
        }
    }
}

void p25_demod_samples(p25_demod *d, const int16_t *x, int count)
{
    for (int k = 0; k < count; k++) {
        int32_t s = x[k];
        uint32_t i = d->x.n & d->x.mask;
        if (s > (d->dc_slow >> 8)) {
            d->signs[i >> 5] |= 1U << (i & 31);
        } else {
            d->signs[i >> 5] &= ~(1U << (i & 31));
        }
        fsk4_put(&d->x, s);
        d->dc_slow += ((s << 8) - d->dc_slow) >> 10;
    }
    int32_t oldest = oldest_t(d);
    if (d->hunt_t < oldest) {
        d->hunt_t = oldest;
        d->cand = false;
    }
    if (d->locked && (sym_t(d->t, d->k) < oldest)) { // fell behind (the task late): hunting again
        d->locked = false;
        d->lost++;
        d->hunt_t = oldest;
        d->cb(d->ctx, P25_DEMOD_LOST);
    }
    hunt(d);
    track(d);
    hunt(d); // after a loss, or once no sync came for a while
    if (d->x.n >= REBASE_AT) {
        fsk4_rebase(&d->x, REBASE_BY);
        int32_t by = (int32_t)(REBASE_BY << Q);
        d->hunt_t -= by;
        d->t -= by;
        d->cand_t -= by;
    }
}
