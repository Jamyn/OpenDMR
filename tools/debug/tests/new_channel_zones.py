"""A new channel and the other zones' channel (va3bpf/OpenDMR#211). Where each zone is stays in RAM until the power-off
save; VFO --> New Chan read the codeplug again, and a zone not on the screen came back on the channel of the last power
off. On a test zone of four channels (codeplug_zone.py: the radio's own zones and channels saved, and put back after):
Test zone on another channel than the one saved, All Channels, VFO --> New Chan, Test zone: still on that channel.

    py tools/debug/tests/new_channel_zones.py PORT
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from radio import Radio  # noqa: E402
from codeplug_zone import TestZone, go_to, new_channel, pick_zone  # noqa: E402

if len(sys.argv) != 2:
    sys.exit(__doc__)
radio = Radio(sys.argv[1])
zone = TestZone(radio, 'new_channel_zones-saved.bin')
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print('%-66s %s %s' % (name, 'OK  ' if ok else 'FAIL', detail))


try:
    zone.write(1, 2, 3, 4)
    saved = zone.saved_position(0)
    start = saved if saved < 4 else 0  # where Test zone opens (channel_screen.c zone_start_position())
    at = 2 if start != 2 else 3        # another position
    want = 'Test %d' % (at + 1)
    s = pick_zone(radio, 'Test zone')
    check('setup: Test zone opens on the saved Test %d' % (start + 1), s.get('name') == 'Test %d' % (start + 1),
          s.get('name'))

    go_to(radio, want)
    pick_zone(radio, 'All Channels')
    new_channel(radio)
    check('setup: New channel 5 written', zone.in_use(5))
    s = pick_zone(radio, 'Test zone')
    check('Test zone after a new channel elsewhere: %s' % want, (s.get('name') == want) and (s.get('zone') == '0'),
          '%s, zone %s' % (s.get('name'), s.get('zone')))
finally:
    zone.restore()
    print('\n%d of %d passed' % (sum(1 for _, ok in results if ok), len(results)))
sys.exit(0 if results and all(ok for _, ok in results) else 1)
