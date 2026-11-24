#!/usr/bin/env python3
"""A camera on the desk: an RTSP server with a test pattern, for the FPV view.

    python3 tools/rtsp-sim.py [--port 8554] [--path /main.264] [--codec h264|h265]
                              [--size 1280x720] [--fps 30] [--udp-only] [--loss PERCENT]
                              [--reorder] [--no-sprop] [--file stream.h264] [--drop-after S]

The pattern (ffmpeg's testsrc2: a moving gradient, a counter) is encoded once by the
ffmpeg command and looped, without B frames and with a key frame a second, as a SIYI
camera sends it. RTP over TCP interleaved; --udp-only answers 461 to it, so that the
client asks for UDP. --loss / --reorder act on the UDP packets. In SYS CONFIG -> SIYI
CAMERA STREAM set the IP ADDRESS to 127.0.0.1 and press the FPV button.
"""
import argparse
import base64
import os
import random
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time


def encode(codec, size, fps):
    out = os.path.join(tempfile.mkdtemp(prefix="rtsp-sim-"), "stream." + codec)
    enc = (["-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency", "-x264-params", "keyint=%d:bframes=0" % fps]
           if codec == "h264" else
           ["-c:v", "libx265", "-preset", "ultrafast", "-tune", "zerolatency", "-x265-params",
            "keyint=%d:bframes=0:log-level=error" % fps])
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "lavfi", "-i", "testsrc2=size=%s:rate=%d" % (size, fps), "-t", "4",
                    "-pix_fmt", "yuv420p", *enc, "-f", "hevc" if codec == "h265" else "h264", out], check=True)
    return open(out, "rb").read()


def nals(data):
    """The NAL units of an Annex-B stream."""
    out, i, n = [], 0, len(data)
    starts = []
    while True:
        k = data.find(b"\0\0\1", i)
        if k < 0:
            break
        starts.append(k)
        i = k + 3
    for a, b in zip(starts, starts[1:] + [n]):
        nal = data[a + 3:b]
        while nal.endswith(b"\0"):
            nal = nal[:-1]
        if nal:
            out.append(nal)
    return out


class Stream:
    def __init__(self, data, hevc):
        self.hevc = hevc
        self.params, self.units, unit, has_vcl = [], [], [], False
        for nal in nals(data):
            t = self.type(nal)
            if t in ((32, 33, 34) if hevc else (7, 8)):
                if not any(self.type(p) == t for p in self.params):
                    self.params.append(nal)
            if t in ((39, 40, 35) if hevc else (6, 9)):
                continue   # SEI, AUD
            # The first slice of a picture, or parameter sets after one: a new access unit
            vcl = t < 32 if hevc else 1 <= t <= 5
            first = vcl and nal[2 if hevc else 1] & 0x80
            if has_vcl and (first or not vcl):
                self.units.append(unit)
                unit, has_vcl = [], False
            unit.append(nal)
            has_vcl = has_vcl or vcl
        if has_vcl:
            self.units.append(unit)

    def type(self, nal):
        return (nal[0] >> 1) & 0x3F if self.hevc else nal[0] & 0x1F

    def payloads(self, nal, mtu=1400):
        if len(nal) <= mtu:
            return [nal]
        out = []
        if self.hevc:
            t = self.type(nal)
            head = bytes([(nal[0] & 0x81) | (49 << 1), nal[1]])
            body = nal[2:]
        else:
            t = self.type(nal)
            head = bytes([(nal[0] & 0xE0) | 28])
            body = nal[1:]
        parts = [body[i:i + mtu] for i in range(0, len(body), mtu)]
        for i, p in enumerate(parts):
            fu = t | (0x80 if i == 0 else 0) | (0x40 if i == len(parts) - 1 else 0)
            out.append(head + bytes([fu]) + p)
        return out


class Session(threading.Thread):
    def __init__(self, sock, addr, opts, stream):
        super().__init__(daemon=True)
        self.s, self.addr, self.o, self.stream = sock, addr, opts, stream
        self.playing, self.udp, self.lock = False, None, threading.Lock()

    def reply(self, cseq, code="200 OK", headers=(), body=b""):
        msg = "RTSP/1.0 %s\r\nCSeq: %s\r\n" % (code, cseq)
        for k, v in headers:
            msg += "%s: %s\r\n" % (k, v)
        if body:
            msg += "Content-Length: %d\r\n" % len(body)
        with self.lock:
            self.s.sendall(msg.encode() + b"\r\n" + body)

    def run(self):
        buf = b""
        base = "rtsp://%s:%d%s" % (self.s.getsockname()[0], self.o.port, self.o.path)
        try:
            while True:
                d = self.s.recv(4096)
                if not d:
                    break
                buf += d
                while b"\r\n\r\n" in buf:
                    head, buf = buf.split(b"\r\n\r\n", 1)
                    lines = head.decode().split("\r\n")
                    method = lines[0].split(" ")[0]
                    h = {k.strip().lower(): v.strip() for k, v in (line.split(":", 1) for line in lines[1:] if ":" in line)}
                    cseq = h.get("cseq", "0")
                    print(self.addr, lines[0])
                    if method == "OPTIONS":
                        self.reply(cseq, headers=[("Public", "OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER")])
                    elif method == "DESCRIBE":
                        if not lines[0].split(" ")[1].endswith(self.o.path):
                            self.reply(cseq, "404 Not Found")
                            continue
                        b64 = [base64.b64encode(p).decode() for p in self.stream.params]
                        sdp = "v=0\r\no=- 0 0 IN IP4 127.0.0.1\r\ns=sim\r\nt=0 0\r\na=control:*\r\nm=video 0 RTP/AVP 96\r\n"
                        sdp += "a=rtpmap:96 %s/90000\r\n" % ("H265" if self.stream.hevc else "H264")
                        if not self.o.no_sprop:
                            if self.stream.hevc:
                                sdp += "a=fmtp:96 sprop-vps=%s;sprop-sps=%s;sprop-pps=%s\r\n" % tuple(b64)
                            else:
                                sdp += "a=fmtp:96 packetization-mode=1;sprop-parameter-sets=%s\r\n" % ",".join(b64)
                        sdp += "a=control:track1\r\n"
                        self.reply(cseq, headers=[("Content-Base", base + "/"), ("Content-Type", "application/sdp")], body=sdp.encode())
                    elif method == "SETUP":
                        tr = h.get("transport", "")
                        if "TCP" in tr:
                            if self.o.udp_only:
                                self.reply(cseq, "461 Unsupported Transport")
                                continue
                            self.reply(cseq, headers=[("Transport", tr), ("Session", "42;timeout=60")])
                        else:
                            port = int(tr.split("client_port=")[1].split("-")[0])
                            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                            self.udp.bind(("", 0))
                            self.udp_to = (self.addr[0], port)
                            self.reply(cseq, headers=[("Transport", tr + ";server_port=%d" % self.udp.getsockname()[1]), ("Session", "42;timeout=60")])
                    elif method == "PLAY":
                        self.reply(cseq, headers=[("Session", "42")])
                        self.playing = True
                        threading.Thread(target=self.feed, daemon=True).start()
                    elif method == "TEARDOWN":
                        self.playing = False
                        self.reply(cseq)
                        return
                    else:
                        self.reply(cseq)
        except OSError:
            pass
        finally:
            self.playing = False
            self.s.close()
            print(self.addr, "closed")

    def feed(self):
        seq, start, n, held = random.randrange(65536), time.time(), 0, []
        try:
            while self.playing:
                unit = self.stream.units[n % len(self.stream.units)]
                ts = int(n * 90000 / self.o.fps) & 0xFFFFFFFF
                pk = [p for nal in unit for p in self.stream.payloads(nal)]
                for i, p in enumerate(pk):
                    pkt = struct.pack("!BBHII", 0x80, 96 | (0x80 if i == len(pk) - 1 else 0), seq, ts, 0x1234) + p
                    seq = (seq + 1) & 0xFFFF
                    if self.udp:
                        if random.random() * 100 < self.o.loss:
                            continue
                        held.append(pkt)
                        if self.o.reorder and len(held) < 4:
                            continue
                        if self.o.reorder:
                            random.shuffle(held)
                        for q in held:
                            self.udp.sendto(q, self.udp_to)
                        held = []
                    else:
                        with self.lock:
                            self.s.sendall(b"$\0" + struct.pack("!H", len(pkt)) + pkt)
                n += 1
                if self.o.drop_after and time.time() - start > self.o.drop_after:
                    print("dropping the connection")
                    self.s.shutdown(socket.SHUT_RDWR)
                    return
                time.sleep(max(0, start + n / self.o.fps - time.time()))
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8554)
    ap.add_argument("--path", default="/main.264")
    ap.add_argument("--codec", choices=["h264", "h265"], default="h264")
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--file", help="an Annex-B file instead of the pattern")
    ap.add_argument("--udp-only", action="store_true")
    ap.add_argument("--loss", type=float, default=0)
    ap.add_argument("--reorder", action="store_true")
    ap.add_argument("--no-sprop", action="store_true", help="parameter sets in the stream only")
    ap.add_argument("--drop-after", type=float, default=0, help="close the connection after this many seconds")
    o = ap.parse_args()
    data = open(o.file, "rb").read() if o.file else encode(o.codec, o.size, o.fps)
    stream = Stream(data, o.codec == "h265")
    print("%d pictures, parameter sets %s" % (len(stream.units), [len(p) for p in stream.params]))
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", o.port))
    srv.listen()
    print("rtsp://127.0.0.1:%d%s" % (o.port, o.path), flush=True)
    while True:
        s, addr = srv.accept()
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        Session(s, addr, o, stream).start()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
