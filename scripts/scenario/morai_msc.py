#!/usr/bin/env python3
"""Bounded MORAI MSC settings client (MORAI-Autonomous/MSC protocol).

Does not equate command acknowledgement with verified sensor/control readiness.
"""
import argparse
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=['status', 'network', 'sensor', 'scenario', 'save-scenario'])
    parser.add_argument('name', nargs='?', default='')
    args = parser.parse_args()
    if args.kind != 'status' and not args.name:
        parser.error('a settings/scenario name is required')
    if args.kind == 'scenario':
        fields = args.name.split('/')
        if len(fields) != 2 or not fields[1].isdigit() or not 0 <= int(fields[1]) <= 63:
            parser.error('scenario requires an explicit name/flags; verify flags for your simulator')
    command = {'network': 0x11, 'sensor': 0x12, 'scenario': 0x13, 'save-scenario': 0x14}.get(args.kind)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        rx.bind(('127.0.0.1', 10326))
        rx.settimeout(1)
        if command is not None:
            option = (args.name+'$').encode()
            if len(option) > 100:
                parser.error('MSC option exceeds 100 bytes')
            tx.sendto(b'#SimControl$'+struct.pack('<i5iBH', 103, 0, 0, 0, 0, 0, 2, command)
                      + option.ljust(100, b'\0') + b'\r\n', ('127.0.0.1', 10508))
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            try:
                data, _ = rx.recvfrom(65535)
            except socket.timeout:
                continue
            if data[:11] != b'#SimStatus$' or len(data) < 143:
                continue
            platform, stage, status, cmd_platform, cmd = struct.unpack('<BBHBH', data[35:42])
            option = data[42:142].rstrip(b'\0').decode(errors='replace').rstrip('$')
            result = data[142]
            if command is None:
                print(platform, stage, status, cmd_platform, cmd, option, result)
                return
            accepted_names = {args.name}
            if args.kind == 'save-scenario' and not args.name.endswith('.json'):
                accepted_names.add(args.name + '.json')
            if cmd_platform == 2 and cmd == command and option in accepted_names and status < 0x1000:
                if result == 1:
                    print('ACKNOWLEDGED', args.kind, args.name)
                    return
                if result:
                    raise RuntimeError('MORAI rejected command: result=%d' % result)
        raise TimeoutError('No matching successful MORAI acknowledgement')


if __name__ == '__main__':
    main()
