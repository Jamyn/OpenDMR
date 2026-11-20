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
 * OpenDMR, service: M17 receive (m17_core.h), from the M17 specification (m17/README.md): the sync bursts (LSF 0x55F7,
 * stream 0xFF5D, EoT 0x555D), the randomizer, the QPP interleaver (its own inverse), the K=5 rate 1/2 convolutional
 * code (G1 = 1 + D^3 + D^4, G2 = 1 + D + D^2 + D^4) punctured with P1 (LSF) or P2 (stream), the LICH's extended
 * Golay(24, 12) and the CRC (0x5935, from 0xFFFF).
 *
 * Hunting, a frame starts at an exact sync burst; a stream frame found that way is taken if its Viterbi metric says it
 * is one. From an LSF or a stream frame on the receiver is locked: the next frame is expected 192 dibits on, its sync
 * may have bit errors or erasures, and up to MAX_MISSES frames without their sync are still decoded where they should
 * be. The LSF is rebuilt from the LICH chunks too (the receiver may start after the LSF, and the HR_C7000 loses most of
 * it when it first finds a signal).
 */

#include <string.h>

#include "m17_core.h"

#define LSF_SYNC    0x55F7
#define STREAM_SYNC 0xFF5D
#define EOT_SYNC    0x555D
#define PAYLOAD     184 // dibits after the sync
#define BITS        368
#define MAX_MISSES  3   // frames without a sync before the lock is given up
#define SYNC_ERRORS 3   // bits of a sync wrong at most where one is expected
#define HUNT_METRIC 24  // a stream frame found hunting: coded bits wrong at most (of 272)

static const uint8_t randomizer[46] = {
    0xd6, 0xb5, 0xe2, 0x30, 0x82, 0xff, 0x84, 0x62, 0xba, 0x4e, 0x96, 0x90, 0xd8, 0x98, 0xdd, 0x5d,
    0x0c, 0xc8, 0x52, 0x43, 0x91, 0x1d, 0xf8, 0x6e, 0x68, 0x2f, 0x35, 0xda, 0x14, 0xea, 0xcd, 0x76,
    0x19, 0x8d, 0xd5, 0x80, 0xd1, 0x33, 0x87, 0x13, 0x57, 0x18, 0x2d, 0x29, 0x78, 0xc3,
};

// Golay(24, 12): the check bits of each data bit (the first data bit first; from g(x) = 0xC75, then the parity bit),
// and the transpose, the inverse (the code is self-dual)
static const uint16_t golay_p[12] = {0xc75, 0x63b, 0xf68, 0x7b4, 0x3da, 0xd99, 0x6cd, 0x367, 0xdc6, 0xa97, 0x93e, 0x8eb};
static const uint16_t golay_pt[12] = {0xa4f, 0xf68, 0x7b4, 0x3da, 0x1ed, 0xab9, 0xf13, 0xdc6, 0x6e3, 0x93e, 0x49f, 0xc75};

static unsigned weight(uint32_t v)
{
    return (unsigned)__builtin_popcount(v);
}

static uint16_t golay_mul(const uint16_t m[12], uint16_t v)
{
    uint16_t out = 0;
    for (int i = 0; i < 12; i++) {
        if ((v >> (11 - i)) & 1) {
            out ^= m[i];
        }
    }
    return out;
}

// up to 3 errors put right; false: more
static bool golay_decode(uint32_t cw, uint16_t *data)
{
    uint16_t d = (uint16_t)(cw >> 12), s = (uint16_t)((cw & 0xFFF) ^ golay_mul(golay_p, d));
    if (weight(s) <= 3) {
        *data = d;
        return true;
    }
    for (int i = 0; i < 12; i++) {
        if (weight(s ^ golay_p[i]) <= 2) {
            *data = d ^ (uint16_t)(1U << (11 - i));
            return true;
        }
    }
    uint16_t s2 = golay_mul(golay_pt, s);
    if (weight(s2) <= 3) {
        *data = d ^ s2;
        return true;
    }
    for (int i = 0; i < 12; i++) {
        if (weight(s2 ^ golay_pt[i]) <= 2) {
            *data = d ^ s2 ^ golay_pt[i];
            return true;
        }
    }
    return false;
}

static uint16_t crc16(const uint8_t *data, unsigned n)
{
    uint16_t crc = 0xFFFF;
    for (unsigned i = 0; i < n; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int b = 0; b < 8; b++) {
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x5935 : 0));
        }
    }
    return crc;
}

// coded bit k of the frame (the interleaver and the randomizer undone); -1 erased
static int coded_bit(const uint8_t *payload, unsigned k)
{
    unsigned a = (45 * k + 92 * k * k) % BITS;
    uint8_t d = payload[a >> 1];
    if (d > 3) {
        return -1;
    }
    unsigned bit = (a & 1) ? (d & 1) : (d >> 1);
    return (int)(bit ^ ((randomizer[a >> 3] >> (7 - (a & 7))) & 1));
}

static unsigned parity(unsigned v)
{
    return (unsigned)__builtin_parity(v);
}

// The Viterbi decoder: length data bits (and 4 flush bits) from the coded bits from first on, punctured with P1 (the
// LSF) or P2 (a stream frame); punctured and erased bits cost nothing. Returns the path metric.
static uint16_t viterbi(const uint8_t *payload, unsigned first, uint8_t *out, unsigned length, bool stream)
{
    uint16_t trace[244];
    uint16_t cost[16], next[16];
    unsigned k = first, steps = length + 4;

    cost[0] = 0;
    for (int s = 1; s < 16; s++) {
        cost[s] = 0x4000;
    }
    for (unsigned step = 0; step < steps; step++) {
        int obs[2];
        for (int o = 0; o < 2; o++) {
            unsigned c = 2 * step + (unsigned)o;
            bool kept = stream ? (c % 12 != 11) : ((c % 61) % 4 != 2); // P2: 11 of 12; P1: 1, then 1 0 1 1 ...
            obs[o] = kept ? coded_bit(payload, k++) : -1;
        }
        uint16_t t = 0;
        for (unsigned s = 0; s < 16; s++) {
            unsigned lo = s >> 1, hi = lo | 8; // the two states before: the oldest bit 0 or 1
            unsigned g1 = parity(s & 0x19), g2 = parity(s & 0x17); // the outputs from lo; from hi both inverted
            unsigned clo = cost[lo], chi = cost[hi];
            if (obs[0] >= 0) {
                clo += g1 ^ (unsigned)obs[0];
                chi += g1 ^ 1 ^ (unsigned)obs[0];
            }
            if (obs[1] >= 0) {
                clo += g2 ^ (unsigned)obs[1];
                chi += g2 ^ 1 ^ (unsigned)obs[1];
            }
            if (chi < clo) {
                t |= (uint16_t)(1U << s);
                next[s] = (uint16_t)chi;
            } else {
                next[s] = (uint16_t)clo;
            }
        }
        trace[step] = t;
        memcpy(cost, next, sizeof(cost));
    }
    memset(out, 0, (length + 7) / 8);
    unsigned s = 0; // the flush bits end in state 0
    for (unsigned step = steps; step-- > 0;) {
        if (step < length) {
            out[step >> 3] |= (uint8_t)((s & 1) << (7 - (step & 7)));
        }
        s = (s >> 1) | (((trace[step] >> s) & 1) << 3);
    }
    return cost[0];
}

// the LICH: 4 Golay words in the coded bits 0 .. 95: 40 bits of the LSF and the chunk number (0 .. 5)
static void lich(m17_rx *r)
{
    uint8_t bits[6] = {0};
    for (int w = 0; w < 4; w++) {
        uint32_t cw = 0;
        for (int i = 0; i < 24; i++) {
            int b = coded_bit(r->payload, (unsigned)(24 * w + i));
            cw = (cw << 1) | (uint32_t)(b > 0);
        }
        uint16_t d;
        if (!golay_decode(cw, &d)) {
            r->lich_bad++;
            return;
        }
        if (w & 1) { // the second 12 bits of a pair of words: the low half of byte 1, byte 2
            bits[3 * (w / 2) + 1] |= (uint8_t)(d >> 8);
            bits[3 * (w / 2) + 2] = (uint8_t)d;
        } else {
            bits[3 * (w / 2)] = (uint8_t)(d >> 4);
            bits[3 * (w / 2) + 1] = (uint8_t)(d << 4);
        }
    }
    r->lich_ok++;
    unsigned n = bits[5] >> 5;
    if (n > 5) {
        return;
    }
    memcpy(&r->lich_lsf[5 * n], bits, 5);
    r->lich_got |= (uint8_t)(1U << n);
}

void m17_rx_init(m17_rx *r)
{
    memset(r, 0, sizeof(*r));
}

void m17_rx_reset(m17_rx *r)
{
    r->locked = false;
    r->pos = 0;
    r->since = 0;
    r->misses = 0;
    r->lich_got = 0;
}

// a whole frame: decoded
static m17_rx_event_t frame(m17_rx *r)
{
    if (r->kind == 1) {
        uint8_t lsf[M17_LSF_BYTES];
        r->metric = viterbi(r->payload, 0, lsf, 240, false);
        if (crc16(lsf, sizeof(lsf)) != 0) {
            r->lsf_bad++;
            return M17_RX_NONE;
        }
        r->lsfs++;
        memcpy(r->lsf, lsf, sizeof(lsf));
        r->lsf_valid = true;
        r->locked = true;
        r->misses = 0;
        r->lich_got = 0; // a new transmission
        return M17_RX_LSF;
    }
    uint8_t data[18];
    r->metric = viterbi(r->payload, 96, data, 144, true);
    if (!r->locked && r->metric > HUNT_METRIC) {
        r->stream_bad++;
        return M17_RX_NONE;
    }
    r->locked = true;
    r->streams++;
    r->fn = (uint16_t)(((data[0] & 0x7F) << 8) | data[1]);
    r->eos = (data[0] & 0x80) != 0;
    memcpy(r->stream, data + 2, M17_STREAM_BYTES);
    uint8_t had = r->lich_got;
    lich(r);
    if ((r->lich_got == 0x3F) && (had != 0x3F) && (crc16(r->lich_lsf, M17_LSF_BYTES) == 0)) {
        memcpy(r->lsf, r->lich_lsf, M17_LSF_BYTES);
        r->lsf_valid = true;
    }
    if (r->eos) {
        r->locked = false;
        r->lich_got = 0;
    }
    return M17_RX_STREAM;
}

m17_rx_event_t m17_rx_dibit(m17_rx *r, uint8_t dibit)
{
    bool erased = dibit > 3;
    r->sync = (r->sync << 2) | (erased ? 0U : dibit);
    r->sync_valid = erased ? 0 : (uint8_t)((r->sync_valid < 255) ? r->sync_valid + 1 : 255);

    if (r->locked) {
        r->since++;
        if (r->since <= 8) { // the next frame's sync
            if (r->since == 8) {
                uint16_t s = (uint16_t)r->sync;
                unsigned valid = (r->sync_valid < 8) ? r->sync_valid : 8; // the last dibits not erased
                uint16_t mask = (uint16_t)((valid >= 8) ? 0xFFFF : ((1U << (2 * valid)) - 1));
                if ((valid >= 4) && (weight((s ^ EOT_SYNC) & mask) <= SYNC_ERRORS)) {
                    r->eots++;
                    m17_rx_reset(r);
                    return M17_RX_EOT;
                }
                if (weight((s ^ STREAM_SYNC) & mask) > SYNC_ERRORS) {
                    if (++r->misses > MAX_MISSES) {
                        r->syncs_lost++;
                        m17_rx_reset(r);
                        return M17_RX_NONE;
                    }
                } else {
                    r->misses = 0;
                }
                r->kind = 2;
                r->pos = 0;
            }
            return M17_RX_NONE;
        }
        r->payload[r->pos++] = dibit;
        if (r->pos < PAYLOAD) {
            return M17_RX_NONE;
        }
        r->since = 0;
        r->pos = 0;
        return frame(r);
    }

    if (r->pos) { // collecting a frame found hunting
        r->payload[r->pos - 1] = dibit;
        if (++r->pos <= PAYLOAD) {
            return M17_RX_NONE;
        }
        r->pos = 0;
        r->since = 0;
        return frame(r);
    }
    if (r->sync_valid >= 8) {
        uint16_t s = (uint16_t)r->sync;
        if (s == LSF_SYNC) {
            r->kind = 1;
            r->pos = 1;
        } else if (s == STREAM_SYNC) {
            r->kind = 2;
            r->pos = 1;
        } else if (s == EOT_SYNC) {
            r->eots++;
            return M17_RX_EOT;
        }
    }
    return M17_RX_NONE;
}

void m17_address(const uint8_t addr[6], char out[10])
{
    static const char alphabet[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/.";
    uint64_t v = 0;
    for (int i = 0; i < 6; i++) {
        v = (v << 8) | addr[i];
    }
    if (v == 0xFFFFFFFFFFFFULL) {
        strcpy(out, "ALL");
        return;
    }
    if ((v == 0) || (v >= 0xEE6B28000000ULL)) {
        strcpy(out, v ? "#EXT" : "");
        return;
    }
    int n = 0;
    while (v && (n < 9)) {
        out[n++] = alphabet[v % 40];
        v /= 40;
    }
    while (n && (out[n - 1] == ' ')) {
        n--;
    }
    out[n] = 0;
}

bool m17_lsf_voice(const uint8_t lsf[M17_LSF_BYTES])
{
    return (lsf[13] & 0x1F) == 0x05; // stream, voice (3200), not encrypted
}
