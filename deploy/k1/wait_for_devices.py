#!/usr/bin/env python3
"""Sleep on missing hardware instead of repeatedly importing robot runtimes."""
import argparse
import os
from pathlib import Path
import re
import time


def ready(profile, root=Path('/')):
    paths = {'navigation': ('dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00',
                            'dev/wheeltec_lidar'),
             'voice': ('dev/rvclaw-m260c-control',)}[profile]
    for name in paths:
        path = root / name
        if not path.is_char_device() or not os.access(path, os.R_OK | os.W_OK):
            return False
    if profile == 'voice':
        try:
            cards = (root / 'proc/asound/cards').read_text()
        except OSError:
            return False
        if 'XFMDPV0018' not in cards or not re.search(r'\[\s*Device\s*\]', cards):
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', choices=('navigation', 'voice'))
    args = parser.parse_args()
    announced = False
    while not ready(args.profile):
        if not announced:
            print('hardware_wait: ' + args.profile + ' devices absent; sleeping', flush=True)
            announced = True
        time.sleep(2)
    print('hardware_wait: ' + args.profile + ' devices ready', flush=True)


if __name__ == '__main__':
    main()
