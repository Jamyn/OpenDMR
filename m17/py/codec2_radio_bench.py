"""Run src/services/c2dec.c on a DM-32 (debug build, src/build's ELF): decode Codec 2 3200 frames, time them, and read
the PCM back (to compare with the PC build of the same code).

  python m17/py/codec2_radio_bench.py in.c2 --port COM23 [--out radio.raw]
"""

import argparse
import struct
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "vocoder" / "py"))
import radio_link  # noqa: E402
from radio_link import Link  # noqa: E402

BATCH = 32
# The calls' big stack: the top of the overlay RAM, above the M17 overlay (its end in the ELF: __m17_ram_end); the
# debug build leaves too little below the vocoder (radio_link.STACK_TOP)
radio_link.STACK_TOP = 0x00056E00


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="raw Codec 2 3200 frames (8 bytes each)")
    parser.add_argument("--port", required=True)
    parser.add_argument("--out", type=Path, help="the radio's PCM (raw 16-bit 8 kHz)")
    parser.add_argument("--profile", action="store_true", help="the time of each step (c2dec_prof)")
    args = parser.parse_args()
    data = args.input.read_bytes()
    frames = len(data) // 8
    link = Link(args.port, ("m17_codec2_bench", "bench_pcm", "__m17_ram_end"))
    assert link.s["__m17_ram_end"] + 0x2000 <= radio_link.STACK_TOP
    fn = link.s["m17_codec2_bench"]
    pcm_addr = link.s["bench_pcm"]
    times, pcm = [], b""
    prof = radio_link.symbols(("c2dec_prof",)).get("c2dec_prof") if args.profile else None
    try:
        link.big_stack_call(fn, 0, 0)
        if prof:
            link.write(prof, bytes(24))
        for start in range(0, frames, BATCH):
            n = min(BATCH, frames - start)
            link._write_long(link.scratch, data[8 * start:8 * (start + n)])
            us = link.big_stack_call(fn, n, link.scratch)
            if us == 0xFFFFFFFF:
                raise RuntimeError("the firmware rejected the batch")
            times.append(us / n)
            if args.out:
                pcm += link._read_long(pcm_addr, 320 * n)
        if prof:
            us = struct.unpack("<6I", link.read(prof, 24))
            print("per frame, ms: " + ", ".join("%s %.2f" % (name, t / frames / 1000) for name, t in zip(
                ("LPC + W", "spectrum", "amplitudes", "phases", "oscillators", "overlap add"), us)))
    finally:
        link.p.close()
    print("%d frames: %.2f ms a frame on average, %.2f ms the slowest batch's average (a frame is 20 ms)"
          % (frames, sum(times) / len(times) / 1000, max(times) / 1000))
    if args.out:
        args.out.write_bytes(pcm)


if __name__ == "__main__":
    main()
