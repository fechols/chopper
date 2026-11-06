#!/usr/bin/env python3
"""Drive the C64 Ultimate for MODE 7: run the PRG, upload the REU image, read the status block, hold a
frame and dump it for `chopper compare`. Device class (REST + DMA socket) from ../talonhd/tools/c64u.py,
itself adapted from impact/tools/c64u_device.py (MIT).

    python tools/c64u.py config [config/chopper.cfg | config/chopper-internal-speaker.cfg]
    python tools/c64u.py run dist/chopper.prg
    python tools/c64u.py upload-reu dist/chopper.reu [--no-reset]
    python tools/c64u.py status
    python tools/c64u.py mode full|doubled|adaptive
    python tools/c64u.py pop PIXELS
    python tools/c64u.py dump dist/dump [--hold]

`config` applies chopper's settings (NTSC, 64 MHz turbo, the 16 MB REU without a preload image, the
Ultimate Audio sampler mapped at $DF20 and audible) to the running configuration, not to flash: run it
before `upload-reu`/`run` when another project may have left its own on the machine.
`config/chopper-internal-speaker.cfg` is the same plus the Speaker Mixer: the game's sound on the C64U's
internal speaker too, for players without HDMI audio.

Host from --host or C64U_HOST (default 192.168.66.178); password (if any) from C64U_PASSWORD.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import struct
import time
import urllib.parse
import urllib.request

STATUS = 0xCA80
SIZE = 28   # layout::status::SIZE
BUFFERS = [0x6000]


class Device:
    def __init__(self, host, timeout=20):
        self.base = host.rstrip('/') if '://' in host else 'http://' + host
        self.host = urllib.parse.urlsplit(self.base).hostname
        self.timeout = timeout

    def request(self, method, path, body=None, binary=False, **query):
        url = self.base + path
        if query:
            url += '?' + urllib.parse.urlencode(query)
        headers = {}
        if os.environ.get('C64U_PASSWORD'):
            headers['X-Password'] = os.environ['C64U_PASSWORD']
        if isinstance(body, dict):
            body = json.dumps(body).encode()
            headers['Content-Type'] = 'application/json'
        elif body is not None:
            headers['Content-Type'] = 'application/octet-stream'
        req = urllib.request.Request(url, method=method, data=body, headers=headers)
        with urllib.request.urlopen(req, timeout=self.timeout) as response:
            payload = response.read()
        if binary:
            return payload
        result = json.loads(payload) if payload else {}
        if result.get('errors'):
            raise RuntimeError(result['errors'])
        return result

    def read(self, address, length):
        data = self.request('GET', '/v1/machine:readmem', binary=True,
                            address=f'{address:04x}', length=length)
        if len(data) != length:
            raise RuntimeError(f'short read at ${address:04x}: {len(data)} of {length}')
        return data

    def write(self, address, data):
        self.request('PUT', '/v1/machine:writemem', address=f'{address:04x}', data=data.hex())

    def pause(self):
        self.request('PUT', '/v1/machine:pause')

    def resume(self):
        self.request('PUT', '/v1/machine:resume')

    def reset(self):
        self.request('PUT', '/v1/machine:reset')

    def run_prg(self, path):
        self.request('POST', '/v1/runners:run_prg', Path(path).read_bytes())

    def write_reu(self, data, address=0):
        """REUWRITE (FF07) over the DMA socket, then IDENTIFY as a barrier."""
        def receive(sock, size):
            out = bytearray()
            while len(out) < size:
                block = sock.recv(size - len(out))
                if not block:
                    raise RuntimeError('DMA socket closed early')
                out.extend(block)
            return bytes(out)
        with socket.create_connection((self.host, 64), timeout=self.timeout) as sock:
            password = os.environ.get('C64U_PASSWORD')
            if password:
                secret = password.encode()
                sock.sendall(struct.pack('<HH', 0xff1f, len(secret)) + secret)
                if receive(sock, 1) != b'\x01':
                    raise RuntimeError('DMA authentication failed')
            # The old header is wiped first, then the chunk holding the new one (its magic, `format::HEADER`)
            # goes last: a booting PRG must never see a valid header before the rest has arrived (it would
            # load a scene not yet there, or, with an image left from the same build, start at once and
            # run its REU DMAs while these writes land: found on the C64U, a corrupted IRQ vector).
            wipe = (address + HEADER).to_bytes(3, 'little') + bytes(64)
            sock.sendall(struct.pack('<HH', 0xff07, len(wipe)) + wipe)
            offsets = list(range(0, len(data), 32768))
            last = [o for o in offsets if o <= HEADER < o + 32768] or offsets[:1]
            for offset in [o for o in offsets if o not in last] + last:
                chunk = data[offset:offset + 32768]
                payload = (address + offset).to_bytes(3, 'little') + chunk
                sock.sendall(struct.pack('<HH', 0xff07, len(payload)) + payload)
            sock.sendall(struct.pack('<HH', 0xff0e, 0))
            receive(sock, receive(sock, 1)[0])


# `format::HEADER`: the image header's REU address (chopper's is in bank 86).
HEADER = 86 << 16


def status(dev):
    st = dev.read(STATUS, SIZE)
    return dict(magic=f'${st[0]:02X}', front=st[1], tick=st[2] | st[3] << 8 | st[21] << 16 | st[22] << 24,
                render_us=(st[4] | st[5] << 8) * 4, floor_us=(st[23] | st[24] << 8) * 4,
                draw_us=(st[25] | st[26] << 8) * 4, hold=st[6], mode='doubled' if st[7] & 1 else 'adaptive' if st[7] & 4 else 'full',
                frames=st[8] | st[9] << 8, fps=st[10], probes=st[12] | st[13] << 8,
                setup_us=st[14] | st[15] << 8, geom_us=st[16] | st[17] << 8, tris=st[18], objects=st[19], pop=st[20])


def read_cfg(path):
    """A C64U config file as the device's own loader reads it (F1 > load from USB): `[Section]` headers,
    `Item=Value` lines, the value everything after the first `=` with only the line end taken off. So a
    leading space stays: the device spells 0 dB ` 0 dB`, written `Vol UltiSid 1= 0 dB` (no quotes: the
    menu takes quotes literally and rejects the value). Blank lines and `;`/`#` comments are skipped; a
    setting before any section, or a quoted value, stops with an error (neither would load on the
    device). A byte-order mark is allowed."""
    settings, section = {}, None
    for n, line in enumerate(Path(path).read_text(encoding='utf-8-sig').splitlines(), 1):
        if not line.strip() or line.lstrip().startswith((';', '#')):
            continue
        if line.startswith('[') and line.rstrip().endswith(']'):
            section = settings.setdefault(line.strip()[1:-1], {})
        elif '=' in line:
            item, value = line.split('=', 1)
            if section is None:
                raise SystemExit(f'{path}:{n}: {item.strip()!r} before any [Section]')
            if len(value.strip()) >= 2 and value.strip()[0] == value.strip()[-1] == '"':
                raise SystemExit(f'{path}:{n}: {item.strip()} is quoted; the device takes quotes literally '
                                 f'(write a leading space as is: `{item.strip()}={value.strip()[1:-1]}`)')
            section[item.strip()] = value.rstrip()
    return settings


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=os.environ.get('C64U_HOST', '192.168.66.178'))
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('config').add_argument('cfg', nargs='?', default=str(Path(__file__).resolve().parent.parent / 'config/chopper.cfg'))
    sub.add_parser('run').add_argument('prg')
    u = sub.add_parser('upload-reu')
    u.add_argument('reu')
    u.add_argument('--no-reset', action='store_true',
                   help='write into the running program (by default the C64 is reset first)')
    sub.add_parser('status')
    sub.add_parser('mode').add_argument('mode', choices=['full', 'doubled', 'adaptive'])
    sub.add_parser('pop').add_argument('pixels', type=int, help='pop-in size: pixels of bounding radius (0 = default 6)')
    d = sub.add_parser('dump')
    d.add_argument('dir')
    d.add_argument('--hold', action='store_true', help='freeze the path tick first (status HOLD)')
    args = ap.parse_args()
    dev = Device(args.host)
    if args.cmd == 'config':
        settings = read_cfg(args.cfg)
        dev.request('POST', '/v1/configs', settings)
        for section, items in settings.items():
            now = dev.request('GET', '/v1/configs/' + urllib.parse.quote(section)).get(section, {})
            for item, value in items.items():
                if str(now.get(item)).strip() != value.strip():
                    raise SystemExit(f'{section} / {item}: {now.get(item)!r}, not {value!r}')
        print(f'{sum(map(len, settings.values()))} settings from {args.cfg}')
    elif args.cmd == 'run':
        dev.run_prg(args.prg)
    elif args.cmd == 'upload-reu':
        data = Path(args.reu).read_bytes()
        t = time.time()
        if not args.no_reset:
            # Reset first: a running program (an earlier build streaming its state or sound through the
            # REU) would otherwise DMA into the image while it arrives. After a reset only the KERNAL and
            # BASIC run, and neither touches the REU. (The header wipe and header-last order stay too.)
            dev.reset()
        dev.write_reu(data)
        print(f'{len(data)} bytes in {time.time() - t:.1f} s')
    elif args.cmd == 'status':
        print(json.dumps(status(dev)))
    elif args.cmd == 'mode':
        dev.write(STATUS + 7, bytes([{'full': 0, 'doubled': 1, 'adaptive': 4}[args.mode]]))
    elif args.cmd == 'pop':
        dev.write(STATUS + 20, bytes([args.pixels]))
    elif args.cmd == 'dump':
        if args.hold:
            dev.write(STATUS + 6, bytes([1]))  # HOLD: any nonzero value freezes the path tick
            time.sleep(0.5)  # let the frames in flight finish and flip
        out = Path(args.dir)
        out.mkdir(parents=True, exist_ok=True)
        # Pause while reading: even with the path held the C64 keeps drawing and flipping, and a buffer
        # can be redrawn during the 8000-byte read (seen as hundreds of "mismatched" bytes).
        dev.pause()
        try:
            st = status(dev)
            bitmap = dev.read(BUFFERS[st['front']], 8000)
        finally:
            dev.resume()
        (out / 'bitmap.bin').write_bytes(bitmap)
        (out / 'status.json').write_text(json.dumps(st))
        print(json.dumps(st))


if __name__ == '__main__':
    main()
