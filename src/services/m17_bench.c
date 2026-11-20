#ifdef OPENDMR_DEBUG

#include "c2dec.h"
#include "systime.h"
#include "vocoder.h"

#define M17_BSS __attribute__((section(".m17_bss"))) // in the M17 overlay (ckcpu.ld), zeroed by its load

static c2dec_t bench_dec M17_BSS;
static int16_t bench_pcm[32 * C2DEC_SAMPLES] M17_BSS; // the last batch's output, read over the cable


// The Codec 2 3200 decoder's time for `frames` frames (8 bytes each) at input, us (m17/py/codec2_radio_bench.py);
// frames 0: the M17 overlay loaded (the decoder from the RAM), the decoder reset
uint32_t m17_codec2_bench(uint32_t frames, const uint8_t *input)
{
    if (frames == 0) {
        overlay_use(OVERLAY_M17);
        c2dec_init(&bench_dec);
        return 0;
    }
    if (frames > 32) {
        return UINT32_MAX;
    }
    uint32_t start = systime_get_us();
    for (uint32_t n = 0; n < frames; n++) {
        c2dec_decode(&bench_dec, input + n * C2DEC_BYTES, bench_pcm + n * C2DEC_SAMPLES);
    }
    return systime_get_us() - start;
}

#endif
