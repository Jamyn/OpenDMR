#!/usr/bin/env python3
"""The way back from a forgotten PIN (General options -> Power-on PIN / Lock PIN): the radio's user data erased.

  py tools/debug/pin_wipe.py --port COM6 --yes

While the radio waits for its PIN (or is locked with the lock PIN) the cable link answers nothing but this request,
'D' 'E' "WIPE!" (src/services/cable_link.c). The radio turns the receiver off and erases the first 1 MB of its data
flash window: the codeplug (channels, zones, contacts, RX groups...), the settings and the PIN (src/services/
security.c). Then it restarts empty, as a new radio: write the codeplug again with the CPS (keep a backup of it).
The factory calibration and the Chinese font are not touched; the firmware stays.

Only when a PIN is on, not while transmitting (the radio refuses). The erase takes some seconds.
"""
import argparse
import sys
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', required=True, help='the programming cable, e.g. COM6')
    parser.add_argument('--yes', action='store_true', help='erase the codeplug, the settings and the PIN')
    args = parser.parse_args()
    if not args.yes:
        sys.exit('This erases the codeplug, the settings and the PIN of the radio. Run again with --yes.')

    p = serial.Serial(args.port, 115200, timeout=0.5)
    time.sleep(0.2)
    p.reset_input_buffer()
    p.write(b'DEWIPE!')
    buf = b''
    end = time.time() + 3.0
    while time.time() < end:
        buf += p.read(max(1, p.in_waiting))
        i = buf.find(b'DE')
        if i >= 0 and len(buf) - i >= 3:
            if buf[i + 2] != 1:
                sys.exit('the radio refused: no PIN is on, or it is transmitting')
            print('erasing; the radio restarts empty in some seconds')
            return
    sys.exit('no answer: is the radio on, with this firmware, and the cable in?')


if __name__ == '__main__':
    main()
