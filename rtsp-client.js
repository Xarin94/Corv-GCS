/**
 * rtsp-client.js - Minimal RTSP/RTP client for the FPV camera (main process)
 *
 * Pulls the camera's H.264 / H.265 stream over RTSP and reassembles the RTP
 * payloads into Annex-B access units that the renderer decodes in hardware
 * with WebCodecs.
 *
 * This replaces the VLC pipeline for the common case. VLC decoded the H.264
 * stream and re-encoded every frame to MJPEG only so the renderer could show
 * JPEGs, which cost a whole extra process transcoding at the stream rate plus
 * a JPEG decode per frame in the renderer. Here the main process only moves
 * bytes: parsing a few Mbit/s of RTP is negligible.
 *
 * Scope: RTSP 1.0 without authentication, one video track. RTP is requested
 * over TCP (interleaved on the RTSP connection, immune to loss on the radio
 * link) and over UDP when the server refuses that. fpv-manager.js falls back
 * to VLC when the camera cannot be reached this way or uses a codec the
 * renderer cannot decode.
 */

const net = require('net');
const dgram = require('dgram');
const { EventEmitter } = require('events');

const START_CODE = Buffer.from([0, 0, 0, 1]);
// RTP packets held while waiting for a missing one before declaring it lost
// (UDP can reorder; over TCP packets always arrive in sequence).
const REORDER_WINDOW = 32;

// ============== H.264 / H.265 DEPACKETIZER ==============

/**
 * Reassembles RTP payloads (RFC 6184 for H.264, RFC 7798 for H.265) into
 * Annex-B access units. Emits { data, key, timestamp } per access unit.
 */
class Depacketizer {
  /**
   * @param {'H264'|'H265'} codec
   * @param {(au: {data: Buffer, key: boolean, rtpTimestamp: number}) => void} onAccessUnit
   */
  constructor(codec, onAccessUnit) {
    this.hevc = codec === 'H265';
    this.onAccessUnit = onAccessUnit;
    this.nals = [];            // NAL units of the access unit being assembled
    this.auTimestamp = null;
    this.fragments = null;     // FU being reassembled: array of Buffers, or null
    this.broken = false;       // a packet of this access unit was lost
    this.needKey = true;       // after a loss, drop until the next key frame
    // Parameter sets, from the SDP or seen in-band; prepended to key frames that
    // do not carry their own (the decoder needs them with every IDR in Annex B).
    this.paramSets = new Map(); // nal type -> Buffer
  }

  nalType(nal) {
    return this.hevc ? (nal[0] >> 1) & 0x3f : nal[0] & 0x1f;
  }

  isParamSet(type) {
    return this.hevc ? (type >= 32 && type <= 34) : (type === 7 || type === 8);
  }

  isKey(type) {
    return this.hevc ? (type >= 16 && type <= 21) : type === 5;
  }

  /** Parameter sets announced in the SDP (sprop-*) */
  addParamSet(nal) {
    if (nal && nal.length) this.paramSets.set(this.nalType(nal), Buffer.from(nal));
  }

  /** Call on an RTP sequence gap: the current access unit cannot be decoded. */
  markLoss() {
    this.broken = true;
    this.fragments = null;
  }

  push(payload, rtpTimestamp, marker) {
    if (this.auTimestamp !== null && rtpTimestamp !== this.auTimestamp) {
      this.flush(); // marker of the previous access unit was lost
    }
    this.auTimestamp = rtpTimestamp;

    if (payload.length < (this.hevc ? 3 : 2)) return;
    if (this.hevc) this.pushH265(payload); else this.pushH264(payload);

    if (marker) this.flush();
  }

  pushH264(p) {
    const type = p[0] & 0x1f;
    if (type >= 1 && type <= 23) {
      this.addNal(p);
    } else if (type === 24) {            // STAP-A: [size16 NAL]...
      let off = 1;
      while (off + 2 <= p.length) {
        const size = p.readUInt16BE(off);
        off += 2;
        if (size === 0 || off + size > p.length) break;
        this.addNal(p.subarray(off, off + size));
        off += size;
      }
    } else if (type === 28) {            // FU-A
      const fuHeader = p[1];
      const start = fuHeader & 0x80, end = fuHeader & 0x40;
      if (start) {
        this.fragments = [Buffer.from([(p[0] & 0xe0) | (fuHeader & 0x1f)]), p.subarray(2)];
      } else if (this.fragments) {
        this.fragments.push(p.subarray(2));
      }
      if (end && this.fragments) {
        this.addNal(Buffer.concat(this.fragments));
        this.fragments = null;
      }
    }
    // STAP-B / MTAP / FU-B are not used by cameras in non-interleaved mode
  }

  pushH265(p) {
    const type = (p[0] >> 1) & 0x3f;
    if (type === 48) {                   // Aggregation packet: [size16 NAL]...
      let off = 2;
      while (off + 2 <= p.length) {
        const size = p.readUInt16BE(off);
        off += 2;
        if (size === 0 || off + size > p.length) break;
        this.addNal(p.subarray(off, off + size));
        off += size;
      }
    } else if (type === 49) {            // Fragmentation unit
      const fuHeader = p[2];
      const start = fuHeader & 0x80, end = fuHeader & 0x40;
      if (start) {
        const nalType = fuHeader & 0x3f;
        const header = Buffer.from([(p[0] & 0x81) | (nalType << 1), p[1]]);
        this.fragments = [header, p.subarray(3)];
      } else if (this.fragments) {
        this.fragments.push(p.subarray(3));
      }
      if (end && this.fragments) {
        this.addNal(Buffer.concat(this.fragments));
        this.fragments = null;
      }
    } else if (type !== 50) {            // 50 = PACI, not used by cameras
      this.addNal(p);
    }
  }

  addNal(nal) {
    const type = this.nalType(nal);
    if (this.isParamSet(type)) this.paramSets.set(type, Buffer.from(nal));
    this.nals.push(nal);
  }

  flush() {
    const nals = this.nals;
    const broken = this.broken;
    this.nals = [];
    this.broken = false;
    this.fragments = null;
    const rtpTimestamp = this.auTimestamp;
    this.auTimestamp = null;
    if (nals.length === 0) return;

    let key = false;
    const present = new Set();
    for (const nal of nals) {
      const type = this.nalType(nal);
      present.add(type);
      if (this.isKey(type)) key = true;
    }

    if (broken) { this.needKey = true; return; }
    if (this.needKey && !key) return;
    this.needKey = false;

    const parts = [];
    if (key) {
      for (const [type, nal] of this.paramSets) {
        if (!present.has(type)) parts.push(START_CODE, nal);
      }
    }
    for (const nal of nals) parts.push(START_CODE, nal);
    this.onAccessUnit({ data: Buffer.concat(parts), key, rtpTimestamp });
  }

  /** WebCodecs codec string derived from the SPS, or null until one is known. */
  codecString() {
    if (this.hevc) {
      const sps = this.paramSets.get(33);
      return sps ? hevcCodecString(sps) : null;
    }
    const sps = this.paramSets.get(7);
    if (!sps || sps.length < 4) return null;
    const hex = (b) => b.toString(16).padStart(2, '0');
    return `avc1.${hex(sps[1])}${hex(sps[2])}${hex(sps[3])}`;
  }
}

/** Strip emulation-prevention bytes (00 00 03 → 00 00). */
function unescapeRbsp(nal) {
  const out = [];
  for (let i = 0; i < nal.length; i++) {
    if (i >= 2 && nal[i] === 3 && nal[i - 1] === 0 && nal[i - 2] === 0) continue;
    out.push(nal[i]);
  }
  return Buffer.from(out);
}

/**
 * RFC 6381 / ISO 14496-15 codec string for an H.265 SPS, e.g. hev1.1.6.L93.B0.
 * The profile_tier_level structure sits at a fixed offset right after the
 * 2-byte NAL header and one byte of VPS id / sub-layer count.
 */
function hevcCodecString(sps) {
  const b = unescapeRbsp(sps);
  if (b.length < 15) return null;
  const ptl = 3;
  const space = b[ptl] >> 6;
  const tier = (b[ptl] >> 5) & 1;
  const profile = b[ptl] & 0x1f;
  const compat = b.readUInt32BE(ptl + 1);
  let reversed = 0;
  for (let i = 0; i < 32; i++) if (compat & (1 << i)) reversed |= 1 << (31 - i);
  const constraints = Array.from(b.subarray(ptl + 5, ptl + 11));
  while (constraints.length && constraints[constraints.length - 1] === 0) constraints.pop();
  const level = b[ptl + 11];
  let s = `hev1.${['', 'A', 'B', 'C'][space]}${profile}.${(reversed >>> 0).toString(16)}.${tier ? 'H' : 'L'}${level}`;
  for (const c of constraints) s += '.' + c.toString(16).toUpperCase();
  return s;
}

// ============== SDP ==============

/** First video media of an SDP: payload type, codec, clock rate, control, parameter sets. */
function parseSdp(sdp) {
  const lines = sdp.split(/\r?\n/);
  let media = null;
  let sessionControl = null;
  for (const line of lines) {
    if (line.startsWith('m=')) {
      if (media && media.video) break;
      const parts = line.slice(2).split(' ');
      media = { video: parts[0] === 'video', payloadType: Number(parts[3]), attrs: [] };
      continue;
    }
    if (!line.startsWith('a=')) continue;
    if (!media) {
      if (line.startsWith('a=control:')) sessionControl = line.slice(10).trim();
      continue;
    }
    media.attrs.push(line.slice(2));
  }
  if (!media || !media.video) return null;

  const out = { payloadType: media.payloadType, codec: null, clockRate: 90000, control: null, paramSets: [], sessionControl };
  for (const a of media.attrs) {
    if (a.startsWith('rtpmap:')) {
      const m = /^rtpmap:(\d+)\s+([\w-]+)\/(\d+)/.exec(a);
      if (m && Number(m[1]) === out.payloadType) {
        out.codec = m[2].toUpperCase() === 'HEVC' ? 'H265' : m[2].toUpperCase();
        out.clockRate = Number(m[3]);
      }
    } else if (a.startsWith('control:')) {
      out.control = a.slice(8).trim();
    } else if (a.startsWith('fmtp:')) {
      const params = a.slice(a.indexOf(' ') + 1).split(';');
      for (const kv of params) {
        const eq = kv.indexOf('=');
        if (eq < 0) continue;
        const k = kv.slice(0, eq).trim().toLowerCase();
        const v = kv.slice(eq + 1).trim();
        if (k === 'sprop-parameter-sets') {
          for (const b64 of v.split(',')) if (b64) out.paramSets.push(Buffer.from(b64, 'base64'));
        } else if (k === 'sprop-vps' || k === 'sprop-sps' || k === 'sprop-pps') {
          for (const b64 of v.split(',')) if (b64) out.paramSets.push(Buffer.from(b64, 'base64'));
        }
      }
    }
  }
  return out;
}

function resolveControl(base, control) {
  if (!control || control === '*') return base;
  if (/^rtsp:\/\//i.test(control)) return control;
  return base.endsWith('/') ? base + control : base + '/' + control;
}

// ============== RTSP SESSION ==============

class RtspError extends Error {
  constructor(message, code) { super(message); this.code = code; }
}

/**
 * Events:
 *   'config'  { codec }                        codec string known (re-sent if the SPS changes)
 *   'frame'   { data, key, timestamp }         Annex-B access unit, timestamp in µs
 *   'close'   (error|null)                     connection ended
 */
class RtspClient extends EventEmitter {
  constructor(url, { timeoutMs = 5000, supportedCodecs = ['H264', 'H265'] } = {}) {
    super();
    this.url = url;
    this.timeoutMs = timeoutMs;
    this.supportedCodecs = supportedCodecs;
    this.socket = null;
    this.buf = Buffer.alloc(0);
    this.cseq = 0;
    this.pending = new Map();   // cseq -> { resolve, reject, timer }
    this.session = null;
    this.keepAlive = null;
    this.closed = false;
    this.depack = null;         // set once the SDP is known
    this.expectedSeq = null;    // next RTP sequence number to deliver
    this.held = new Map();      // seq -> packet waiting for a gap to fill
  }

  async start() {
    const u = new URL(this.url);
    const host = u.hostname;
    const port = Number(u.port) || 554;

    await new Promise((resolve, reject) => {
      const s = net.connect({ host, port }, resolve);
      s.setNoDelay(true);
      s.setTimeout(this.timeoutMs, () => s.destroy(new Error('RTSP connect timeout')));
      s.once('error', reject);
      this.socket = s;
    });
    this.socket.setTimeout(0);
    this.socket.on('data', (chunk) => this.onData(chunk));
    this.socket.on('error', (err) => this.finish(err));
    this.socket.on('close', () => this.finish(null));

    await this.request('OPTIONS', this.url);
    const desc = await this.request('DESCRIBE', this.url, { Accept: 'application/sdp' });
    const sdp = parseSdp(desc.body);
    if (!sdp || !sdp.codec) throw new RtspError('No video track in SDP', 'NO_VIDEO');
    if (!this.supportedCodecs.includes(sdp.codec)) {
      throw new RtspError(`Codec ${sdp.codec} not decodable here`, 'CODEC');
    }
    this.sdp = sdp;

    const base = desc.headers['content-base'] || desc.headers['content-location'] || this.url;
    const trackUrl = resolveControl(resolveControl(base, sdp.sessionControl), sdp.control);

    this.depack = new Depacketizer(sdp.codec, (au) => this.onAccessUnit(au));
    for (const ps of sdp.paramSets) this.depack.addParamSet(ps);
    this.expectedSeq = null;
    this.held = new Map();

    let setup;
    try {
      setup = await this.request('SETUP', trackUrl, {
        Transport: 'RTP/AVP/TCP;unicast;interleaved=0-1'
      });
      this.transport = 'tcp';
      const tr = /interleaved=(\d+)/.exec(setup.headers.transport || '');
      this.rtpChannel = tr ? Number(tr[1]) : 0;
    } catch (err) {
      if (!(err instanceof RtspError) || typeof err.code !== 'number') throw err;
      // 461 Unsupported Transport (or a 4xx from servers that do not say so)
      const port = await this.openUdp();
      setup = await this.request('SETUP', trackUrl, {
        Transport: `RTP/AVP;unicast;client_port=${port}-${port + 1}`
      });
      this.transport = 'udp';
      this.rtpChannel = -1;
    }
    const session = setup.headers.session || '';
    this.session = session.split(';')[0].trim();
    const timeout = /timeout=(\d+)/.exec(session);

    this.tsBase = null;
    this.tsLast = 0;
    this.tsWraps = 0;
    this.codec = null;

    await this.request('PLAY', resolveControl(base, sdp.sessionControl), { Range: 'npt=0.000-' });

    // Many servers drop a session that sends nothing for its timeout, even
    // while it is streaming over the same connection.
    const every = Math.max(5, Math.min(30, (timeout ? Number(timeout[1]) : 60) / 2)) * 1000;
    this.keepAlive = setInterval(() => {
      this.request('GET_PARAMETER', this.url).catch(() => {});
    }, every);
  }

  /** Bind an even RTP port and the RTCP port above it; returns the RTP port. */
  async openUdp() {
    const bind = (sock, port) => new Promise((resolve, reject) => {
      sock.once('error', reject);
      sock.bind(port, () => { sock.removeListener('error', reject); resolve(); });
    });
    for (let attempt = 0; attempt < 20; attempt++) {
      const port = 50000 + 2 * Math.floor(Math.random() * 5000);
      const rtp = dgram.createSocket('udp4');
      const rtcp = dgram.createSocket('udp4');
      try {
        await bind(rtp, port);
        await bind(rtcp, port + 1);
      } catch (_) {
        try { rtp.close(); } catch (e) {}
        try { rtcp.close(); } catch (e) {}
        continue;
      }
      // Key frames arrive as bursts of a few hundred packets
      try { rtp.setRecvBufferSize(4 * 1024 * 1024); } catch (_) {}
      rtp.on('message', (pkt) => this.onRtp(pkt));
      rtcp.on('message', () => {});
      this.udpSockets = [rtp, rtcp];
      return port;
    }
    throw new RtspError('No free UDP port pair for RTP', 'UDP');
  }

  stop() {
    if (this.closed) return;
    if (this.session && this.socket && !this.socket.destroyed) {
      try { this.write('TEARDOWN', this.url, {}); } catch (_) {}
    }
    this.finish(null);
  }

  finish(err) {
    if (this.closed) return;
    this.closed = true;
    clearInterval(this.keepAlive);
    for (const p of this.pending.values()) {
      clearTimeout(p.timer);
      p.reject(err || new Error('RTSP connection closed'));
    }
    this.pending.clear();
    if (this.socket && !this.socket.destroyed) this.socket.destroy();
    for (const s of this.udpSockets || []) { try { s.close(); } catch (_) {} }
    this.udpSockets = null;
    this.emit('close', err);
  }

  write(method, url, headers) {
    const cseq = ++this.cseq;
    let msg = `${method} ${url} RTSP/1.0\r\nCSeq: ${cseq}\r\nUser-Agent: CORV-GCS\r\n`;
    if (this.session) msg += `Session: ${this.session}\r\n`;
    for (const [k, v] of Object.entries(headers)) msg += `${k}: ${v}\r\n`;
    this.socket.write(msg + '\r\n');
    return cseq;
  }

  request(method, url, headers = {}) {
    return new Promise((resolve, reject) => {
      if (this.closed) { reject(new Error('RTSP connection closed')); return; }
      const cseq = this.write(method, url, headers);
      const timer = setTimeout(() => {
        this.pending.delete(cseq);
        reject(new RtspError(`${method} timed out`, 'TIMEOUT'));
      }, this.timeoutMs);
      this.pending.set(cseq, {
        timer,
        resolve: (res) => {
          if (res.status >= 200 && res.status < 300) resolve(res);
          else reject(new RtspError(`${method} failed: ${res.status} ${res.reason}`, res.status));
        },
        reject
      });
    });
  }

  onData(chunk) {
    this.buf = this.buf.length ? Buffer.concat([this.buf, chunk]) : chunk;
    const buf = this.buf;
    let off = 0;
    while (off < buf.length) {
      if (buf[off] === 0x24) {                       // '$' interleaved binary frame
        if (buf.length - off < 4) break;
        const channel = buf[off + 1];
        const len = buf.readUInt16BE(off + 2);
        if (buf.length - off < 4 + len) break;
        if (channel === this.rtpChannel) this.onRtp(buf.subarray(off + 4, off + 4 + len));
        off += 4 + len;                              // RTCP (odd channel) ignored
      } else if (buf[off] === 0x52) {                // 'R' — RTSP/1.0 response
        const headerEnd = buf.indexOf('\r\n\r\n', off);
        if (headerEnd === -1) break;
        const head = buf.toString('latin1', off, headerEnd);
        const cl = /\r\ncontent-length:\s*(\d+)/i.exec(head);
        const bodyLen = cl ? Number(cl[1]) : 0;
        if (buf.length < headerEnd + 4 + bodyLen) break;
        const body = buf.toString('utf8', headerEnd + 4, headerEnd + 4 + bodyLen);
        off = headerEnd + 4 + bodyLen;
        this.onResponse(head, body);
      } else {
        // Out of sync (or a server request we do not handle): skip to the next frame
        const next = buf.indexOf(0x24, off + 1);
        off = next === -1 ? buf.length : next;
      }
    }
    this.buf = off >= buf.length ? Buffer.alloc(0) : buf.subarray(off);
  }

  onResponse(head, body) {
    const lines = head.split('\r\n');
    const m = /^RTSP\/1\.\d\s+(\d+)\s*(.*)$/.exec(lines[0]);
    if (!m) return;
    const headers = {};
    for (const line of lines.slice(1)) {
      const i = line.indexOf(':');
      if (i > 0) headers[line.slice(0, i).trim().toLowerCase()] = line.slice(i + 1).trim();
    }
    const cseq = Number(headers.cseq);
    const p = this.pending.get(cseq);
    if (!p) return;
    this.pending.delete(cseq);
    clearTimeout(p.timer);
    p.resolve({ status: Number(m[1]), reason: m[2], headers, body });
  }

  onRtp(pkt) {
    if (this.closed || !this.depack) return;
    if (pkt.length < 12 || (pkt[0] >> 6) !== 2) return;
    const payloadType = pkt[1] & 0x7f;
    if (payloadType !== this.sdp.payloadType) return;
    let off = 12 + (pkt[0] & 0x0f) * 4;
    if (pkt[0] & 0x10) {                             // header extension
      if (pkt.length < off + 4) return;
      off += 4 + pkt.readUInt16BE(off + 2) * 4;
    }
    let end = pkt.length;
    if (pkt[0] & 0x20) end -= pkt[pkt.length - 1];   // padding
    if (off >= end) return;
    const p = {
      seq: pkt.readUInt16BE(2),
      ts: pkt.readUInt32BE(4),
      marker: (pkt[1] & 0x80) !== 0,
      payload: pkt.subarray(off, end)
    };

    if (this.expectedSeq === null) this.expectedSeq = p.seq;
    const ahead = (p.seq - this.expectedSeq) & 0xffff;
    if (ahead === 0) {
      this.deliver(p);
      this.drainHeld();
    } else if (ahead < 0x8000) {
      // A packet is missing: hold this one until the gap fills or the window
      // overflows, then give the missing ones up as lost.
      this.held.set(p.seq, p);
      if (this.held.size > REORDER_WINDOW) {
        let next = null, best = Infinity;
        for (const seq of this.held.keys()) {
          const d = (seq - this.expectedSeq) & 0xffff;
          if (d < best) { best = d; next = seq; }
        }
        this.depack.markLoss();
        this.expectedSeq = next;
        this.drainHeld();
      }
    }
    // else: late duplicate of a packet already delivered or given up — drop
  }

  deliver(p) {
    this.expectedSeq = (p.seq + 1) & 0xffff;
    this.depack.push(p.payload, p.ts, p.marker);
  }

  drainHeld() {
    let p;
    while (this.held.size && (p = this.held.get(this.expectedSeq))) {
      this.held.delete(p.seq);
      this.deliver(p);
    }
  }

  onAccessUnit(au) {
    const codec = this.depack.codecString();
    if (!codec) return;                              // no SPS yet: nothing decodable
    if (codec !== this.codec) {
      this.codec = codec;
      this.emit('config', { codec });
    }
    // 32-bit RTP timestamp → monotonic microseconds
    const ts = au.rtpTimestamp >>> 0;
    if (this.tsBase === null) this.tsBase = ts;
    if (ts < this.tsLast && this.tsLast - ts > 0x80000000) this.tsWraps++;
    this.tsLast = ts;
    const ticks = ts + this.tsWraps * 0x100000000 - this.tsBase;
    const timestamp = Math.round(ticks * 1e6 / this.sdp.clockRate);
    this.emit('frame', { data: au.data, key: au.key, timestamp });
  }
}

module.exports = { RtspClient, RtspError, Depacketizer, parseSdp, hevcCodecString };
