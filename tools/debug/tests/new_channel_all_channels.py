"""A new channel and "All Channels" (va3bpf/OpenDMR#213). There the screen keeps a position among the channels in use,
and VFO --> New Chan takes the lowest free number: one below the channel shown moved the screen to the channel before
it. On a test zone of channels 1, 3, 5, 7 and 8 (codeplug_zone.py: the radio's own zones and channels saved, and put
back after):
  1. All Channels on Test 8, VFO --> New Chan (number 2, below it): still on Test 8
  2. on Test 1, VFO --> New Chan (number 4, above it): still on Test 1
  3. Test zone on Test 8, VFO --> New Chan (number 6): still on Test 8 (a zone's own list: never moved)

    py tools/debug/tests/new_channel_all_channels.py PORT
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from radio import Radio  # noqa: E402
from codeplug_zone import TestZone, go_to, new_channel, pick_zone  # noqa: E402

if len(sys.argv) != 2:
    sys.exit(__doc__)
radio = Radio(sys.argv[1])
zone = TestZone(radio, 'new_channel_all_channels-saved.bin')
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print('%-66s %s %s' % (name, 'OK  ' if ok else 'FAIL', detail))


try:
    zone.write(1, 3, 5, 7, 8)
    # the zone list: Test zone (0), All Channels (1)
    for step, index, shown, number in (('1.', 1, 'Test 8', 2), ('2.', None, 'Test 1', 4), ('3.', 0, 'Test 8', 6)):
        if index is not None:
            pick_zone(radio, ('Test zone', 'All Channels')[index])
            zone_index = index
        go_to(radio, shown)
        s = new_channel(radio)
        check('setup: New channel %d written' % number, zone.in_use(number))
        check('%s %s on %s, a new channel (%d): still %s' % (step, ('Test zone', 'All Channels')[zone_index], shown,
                                                              number, shown),
              (s.get('name') == shown) and (s.get('zone') == str(zone_index)), '%s, zone %s' % (s.get('name'),
                                                                                                 s.get('zone')))
finally:
    zone.restore()
    print('\n%d of %d passed' % (sum(1 for _, ok in results if ok), len(results)))
sys.exit(0 if results and all(ok for _, ok in results) else 1)
