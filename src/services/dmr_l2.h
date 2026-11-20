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

// DMR layer 2 receive (ETSI TS 102 361-1): a burst's fields and their FEC, our own implementation from the standard.
// No OS, no hardware; also built on the PC (dmr_soft/c). A burst is its 264 bits, 33 bytes, the first on the air the
// MSB of the first byte (as MMDVM and the hotspot test files keep them).
#ifndef DMR_L2_H
#define DMR_L2_H

#include <stdbool.h>
#include <stdint.h>

#define DMR_BURST_BYTES 33
#define DMR_AMBE_BYTES  9

// The SYNC patterns (Table 9.2); voice and data are each other's symbol-wise complement
enum {
    DMR_SYNC_NONE = 0,
    DMR_SYNC_BS_VOICE, DMR_SYNC_BS_DATA,
    DMR_SYNC_MS_VOICE, DMR_SYNC_MS_DATA,
    DMR_SYNC_TS1_VOICE, DMR_SYNC_TS1_DATA, // TDMA direct mode
    DMR_SYNC_TS2_VOICE, DMR_SYNC_TS2_DATA,
    DMR_SYNC_KINDS
};
extern const uint64_t dmr_sync_word[DMR_SYNC_KINDS]; // 48 bits ([0] unused)
#define DMR_SYNC_IS_VOICE(k) (((k) != DMR_SYNC_NONE) && ((k) & 1))
#define DMR_SYNC_IS_DATA(k)  (((k) != DMR_SYNC_NONE) && !((k) & 1))
#define DMR_SYNC_IS_BS(k)    (((k) == DMR_SYNC_BS_VOICE) || ((k) == DMR_SYNC_BS_DATA))

// Data Type (Table 9.22)
enum {
    DMR_DT_PI_HEADER = 0, DMR_DT_VOICE_LC_HEADER = 1, DMR_DT_TERMINATOR_LC = 2, DMR_DT_CSBK = 3,
    DMR_DT_MBC_HEADER = 4, DMR_DT_MBC_CONT = 5, DMR_DT_DATA_HEADER = 6, DMR_DT_RATE_12 = 7, DMR_DT_RATE_34 = 8,
    DMR_DT_IDLE = 9, DMR_DT_RATE_1 = 10, DMR_DT_USBD = 11
};

// LCSS (9.3.3)
enum { DMR_LCSS_SINGLE = 0, DMR_LCSS_FIRST = 1, DMR_LCSS_LAST = 2, DMR_LCSS_CONT = 3 };

// the burst's middle 48 bits (bits 108 - 155)
uint64_t dmr_burst_middle(const uint8_t b[DMR_BURST_BYTES]);
// the nearest SYNC pattern within max_errors bits of w (DMR_SYNC_NONE: none)
int dmr_sync_kind(uint64_t w, int max_errors);

// Slot Type (9.1.3, Golay (20,8)): colour code and data type; false: more errors than it corrects (3)
bool dmr_slot_type(const uint8_t b[DMR_BURST_BYTES], uint8_t *cc, uint8_t *data_type);
// EMB (9.1.2, QR (16,7,6)): colour code, PI, LCSS; false: more errors than it corrects (2)
bool dmr_emb(const uint8_t b[DMR_BURST_BYTES], uint8_t *cc, uint8_t *pi, uint8_t *lcss);
// the 32 embedded signalling bits of a voice burst B - F, 4 bytes
void dmr_embedded(const uint8_t b[DMR_BURST_BYTES], uint8_t out[4]);
// a voice burst's 3 AMBE+2 frames (72 bits each, as on the air: what our vocoder decodes)
void dmr_voice_frames(const uint8_t b[DMR_BURST_BYTES], uint8_t out[3][DMR_AMBE_BYTES]);

// BPTC (196,96) (B.1.1) of a data burst: the 96 information bits, 12 bytes; false: not corrected (a row or column with
// more errors than its Hamming code corrects, after a few rounds)
bool dmr_bptc196(const uint8_t b[DMR_BURST_BYTES], uint8_t out[12]);
// Full LC of a voice LC header or a terminator (12 bytes after the BPTC): its RS (12,9) parity with the data type's
// CRC mask (B.3.12) checked (one wrong byte corrected); false: not a valid LC
#define DMR_MASK_VOICE_LC_HEADER 0x969696
#define DMR_MASK_TERMINATOR_LC   0x999999
bool dmr_full_lc_check(uint8_t lc[12], uint32_t mask);

// TACT (9.1.4, Hamming (7,4)) of a CACH (its 24 bits as sent): AT, TC (the TDMA channel of the burst that follows: 0
// timeslot 1), LCSS; false: two or more errors seen
bool dmr_tact(uint32_t cach, uint8_t *at, uint8_t *tc, uint8_t *lcss);

// The embedded LC (7.1.3, B.2.1): the fragments of bursts B - E (LCSS first, continuation, continuation, last) gathered;
// dmr_emb_lc_add() true once the 4th is in and the LC (9 bytes) decodes with its checksum
typedef struct {
    uint8_t bits[16]; // the transmit matrix, 128 bits as received
    int n;            // fragments in so far (-1: waiting for a first)
} dmr_emb_lc;
void dmr_emb_lc_reset(dmr_emb_lc *e);
bool dmr_emb_lc_add(dmr_emb_lc *e, uint8_t lcss, const uint8_t frag[4], uint8_t lc[9]);

// A Full LC's fields (ETSI TS 102 361-2 7.1.1: group voice FLCO 0, unit to unit 3)
#define DMR_FLCO_GROUP 0
#define DMR_FLCO_UNIT  3
typedef struct {
    uint8_t flco, fid, options;
    uint32_t dst, src;
} dmr_lc_t;
void dmr_lc_parse(const uint8_t lc[9], dmr_lc_t *out);

#endif
