#!/usr/bin/env python3
"""Drive the C64 Ultimate for MODE 7: run the PRG, upload the REU image, read the status block, hold a
frame and dump it for `chopper compare`. Device class (REST + DMA socket) from ../talonhd/tools/c64u.py,
itself adapted from impact/tools/c64u_device.py (MIT).

    python tools/c64u.py run dist/chopper.prg
    python tools/c64u.py upload-reu dist/chopper.reu
    python tools/c64u.py status
    python tools/c64u.py mode full|doubled
    python tools/c64u.py pop PIXELS
    python tools/c64u.py dump dist/dump [--hold]

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
            # The first chunk (the image header, its magic) goes last: a booting PRG must never see a
            # header before the rest has arrived (it would find the image incomplete and erase the magic).
            offsets = list(range(0, len(data), 32768))
            for offset in offsets[1:] + offsets[:1]:
                chunk = data[offset:offset + 32768]
                payload = (address + offset).to_bytes(3, 'little') + chunk
                sock.sendall(struct.pack('<HH', 0xff07, len(payload)) + payload)
            sock.sendall(struct.pack('<HH', 0xff0e, 0))
            receive(sock, receive(sock, 1)[0])


def status(dev):
    st = dev.read(STATUS, 21)
    return dict(magic=f'${st[0]:02X}', front=st[1], tick=st[2] | st[3] << 8,
                render_us=st[4] | st[5] << 8, hold=st[6], mode='doubled' if st[7] & 1 else 'full',
                frames=st[8] | st[9] << 8, fps=st[10], probes=st[12] | st[13] << 8,
                setup_us=st[14] | st[15] << 8, geom_us=st[16] | st[17] << 8, tris=st[18], objects=st[19], pop=st[20])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=os.environ.get('C64U_HOST', '192.168.66.178'))
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('run').add_argument('prg')
    sub.add_parser('upload-reu').add_argument('reu')
    sub.add_parser('status')
    sub.add_parser('mode').add_argument('mode', choices=['full', 'doubled'])
    sub.add_parser('pop').add_argument('pixels', type=int, help='pop-in size: pixels of bounding radius (0 = default 6)')
    d = sub.add_parser('dump')
    d.add_argument('dir')
    d.add_argument('--hold', action='store_true', help='freeze the path tick first (status HOLD)')
    args = ap.parse_args()
    dev = Device(args.host)
    if args.cmd == 'run':
        dev.run_prg(args.prg)
    elif args.cmd == 'upload-reu':
        data = Path(args.reu).read_bytes()
        t = time.time()
        dev.write_reu(data)
        print(f'{len(data)} bytes in {time.time() - t:.1f} s')
    elif args.cmd == 'status':
        print(json.dumps(status(dev)))
    elif args.cmd == 'mode':
        dev.write(STATUS + 7, bytes([1 if args.mode == 'doubled' else 0]))
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
