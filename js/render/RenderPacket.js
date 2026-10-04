import { RENDER_SCHEMA_VERSION } from './RenderWorld.js';

// CRVG + protocol version + UTF-8 manifest bytes + binary payload bytes.
// All header words and numeric attachments are little endian, aligned to 8.
const MAGIC = 0x47565243, HEADER = 16, MAX_BYTES = 512 * 1024 * 1024;
const TYPES = { i16: Int16Array, u8: Uint8Array, u16: Uint16Array, u32: Uint32Array, f32: Float32Array, f64: Float64Array };
const LITTLE_ENDIAN = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;
const align8 = n => Math.ceil(n / 8) * 8;

/** A snapshot uses active spans, not entire point-cloud allocations. No GPU objects. */
export function renderDocument(world, { includeTextures = false } = {}) {
    const c = world.camera;
    return {
        schemaVersion: world.schemaVersion, axes: world.axes, units: world.units,
        origin: { ...world.origin }, timeSeconds: performance.now() / 1000,
        camera: { revision: c.revision, position: [...c.position], quaternion: [...c.quaternion],
            projection: c.projection, view: c.view, world: c.world,
            viewport: [...c.viewport], near: c.near, far: c.far, fov: c.fov },
        terrainStyle: world.terrainStyle,
        terrain: [...world.terrain.chunks.values()].map(r => ({
            ...r, appearance: includeTextures ? r.appearance : (r.appearance ? { kind: r.appearance.kind, omitted: true } : null)
        })),
        pointClouds: [...world.pointClouds.entries()].map(([id, p]) => ({
            id, revision: p.revision, generation: p.generation, origin: p.origin,
            visible: p.visible, colorMode: p.colorMode, pointSize: p.pointSize, overTerrain: p.overTerrain,
            mapTransform: p.mapTransform, liveTransform: p.liveTransform, heightRange: p.heightRange,
            chunks: p.chunks.map(b => ({ id: b.id, revision: b.revision, count: b.count,
                positions: b.positions.subarray(0, b.count * 3), intensity: b.intensity.subarray(0, b.count) })),
            live: { count: p.live.count, head: p.live.head, frame: p.live.frame, ttl: p.live.ttl,
                positions: p.live.positions.subarray(0, p.live.count * 3),
                intensity: p.live.intensity.subarray(0, p.live.count), birth: p.live.birth.subarray(0, p.live.count) }
        }))
    };
}

/** On-demand export makes ONE copy of attachments, never detaches live buffers. */
export function encodeRenderPacket(document) {
    if (document.schemaVersion !== RENDER_SCHEMA_VERSION) throw new Error('Unsupported render schema');
    const attachments = [], sources = [], seen = new Map();
    let bytes = 0;
    function pack(value) {
        if (ArrayBuffer.isView(value)) {
            const type = Object.keys(TYPES).find(key => value.constructor === TYPES[key]);
            if (!type) throw new Error('Unsupported render buffer type');
            if (seen.has(value)) return { $buffer: seen.get(value) };
            const index = sources.length;
            sources.push(value); seen.set(value, index);
            bytes = align8(bytes);
            attachments.push({ type, offset: bytes, byteLength: value.byteLength }); bytes += value.byteLength;
            if (bytes > MAX_BYTES) throw new Error('Render packet too large');
            return { $buffer: index };
        }
        if (Array.isArray(value)) return value.map(pack);
        if (value && typeof value === 'object') {
            if (Object.getPrototypeOf(value) !== Object.prototype) throw new Error('Render contract requires plain data');
            const result = {};
            for (const [key, v] of Object.entries(value)) if (v !== undefined) result[key] = pack(v);
            return result;
        }
        if (typeof value === 'number' && !Number.isFinite(value)) throw new Error('Non-finite render value');
        if (value === null || ['string', 'number', 'boolean'].includes(typeof value)) return value;
        throw new Error('Unsupported render value');
    }
    const data = pack(document);
    const manifest = new TextEncoder().encode(JSON.stringify({ data, attachments }));
    const payloadOffset = align8(HEADER + manifest.byteLength), length = payloadOffset + bytes;
    if (manifest.byteLength > 16 * 1024 * 1024 || length > MAX_BYTES) throw new Error('Render packet too large');
    const buffer = new ArrayBuffer(length), header = new DataView(buffer), out = new Uint8Array(buffer);
    header.setUint32(0, MAGIC, true); header.setUint32(4, RENDER_SCHEMA_VERSION, true);
    header.setUint32(8, manifest.byteLength, true); header.setUint32(12, bytes, true);
    out.set(manifest, HEADER);
    sources.forEach((source, index) => {
        const offset = payloadOffset + attachments[index].offset;
        out.set(new Uint8Array(source.buffer, source.byteOffset, source.byteLength), offset);
        if (!LITTLE_ENDIAN && source.BYTES_PER_ELEMENT > 1) {
            for (let i = 0; i < source.byteLength; i += source.BYTES_PER_ELEMENT) {
                out.subarray(offset + i, offset + i + source.BYTES_PER_ELEMENT).reverse();
            }
        }
    });
    return buffer;
}

export function decodeRenderPacket(buffer) {
    if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < HEADER || buffer.byteLength > MAX_BYTES) throw new Error('Invalid render packet length');
    const header = new DataView(buffer);
    if (header.getUint32(0, true) !== MAGIC || header.getUint32(4, true) !== RENDER_SCHEMA_VERSION) throw new Error('Unsupported render packet');
    const manifestLength = header.getUint32(8, true), payloadLength = header.getUint32(12, true);
    const offset = align8(HEADER + manifestLength);
    if (manifestLength > 16 * 1024 * 1024 || offset + payloadLength !== buffer.byteLength) throw new Error('Truncated render packet');
    const manifest = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, HEADER, manifestLength)));
    if (!Array.isArray(manifest.attachments)) throw new Error('Missing render attachments');
    const arrays = manifest.attachments.map(a => {
        const Type = TYPES[a.type];
        if (!Type || !Number.isSafeInteger(a.offset) || !Number.isSafeInteger(a.byteLength) || a.offset < 0 || a.byteLength < 0 ||
            a.offset % 8 || a.byteLength % Type.BYTES_PER_ELEMENT || a.offset + a.byteLength > payloadLength) throw new Error('Invalid render attachment');
        const copy = buffer.slice(offset + a.offset, offset + a.offset + a.byteLength);
        if (!LITTLE_ENDIAN && Type.BYTES_PER_ELEMENT > 1) {
            const bytes = new Uint8Array(copy);
            for (let i = 0; i < bytes.length; i += Type.BYTES_PER_ELEMENT) bytes.subarray(i, i + Type.BYTES_PER_ELEMENT).reverse();
        }
        return new Type(copy);
    });
    function unpack(value) {
        if (value && typeof value === 'object' && Object.hasOwn(value, '$buffer')) {
            if (!Number.isInteger(value.$buffer) || !arrays[value.$buffer]) throw new Error('Missing render buffer');
            return arrays[value.$buffer];
        }
        if (Array.isArray(value)) return value.map(unpack);
        if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value).map(([k, v]) => [k, unpack(v)]));
        return value;
    }
    const result = unpack(manifest.data);
    if (result.schemaVersion !== RENDER_SCHEMA_VERSION || result.axes !== 'east-up-south' || result.units !== 'metres') throw new Error('Unsupported render coordinates');
    return result;
}

export function encodeRenderWorld(world, options) { return encodeRenderPacket(renderDocument(world, options)); }
