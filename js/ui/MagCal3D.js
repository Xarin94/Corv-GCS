/**
 * MagCal3D.js - 3D view of the magnetometers during compass calibration
 *
 * What Mission Planner's live compass calibration shows: every magnetometer
 * sample as a point in the vehicle's body frame. Turning the vehicle turns the
 * Earth's field around it, so the points paint a sphere — offset and squashed
 * before calibration, which is exactly what the calibration measures. On top:
 *   - a sphere fitted to each compass's points, which recentres and scales
 *     them, so every compass sits on the same unit sphere;
 *   - the 80 sections of the firmware's completion mask (GeodesicGrid.js),
 *     green once covered, so the operator sees which sides are still missing;
 *   - the latest sample as a white marker, and the nearest section still
 *     missing in orange: turn the vehicle until the marker reaches it.
 *
 * Rendering is on demand (a new sample, a new mask, a drag), so nothing runs
 * while the page is hidden.
 */

import { SECTIONS, SECTION_COUNT, sectionOf, maskHas } from './GeodesicGrid.js';

export const MAG_COLORS = ['#00d2ff', '#ffa020', '#ff4fd8'];

const MAX_POINTS = 2500;      // per compass, oldest dropped first
const MIN_STEP = 0.02;        // a sample within 2 % of the radius of the last kept one adds nothing
const FIT_MIN_POINTS = 30;
const FIT_MIN_SECTIONS = 10;  // a fit through one patch of the sphere is a guess, not a sphere

const COLOR_IDLE = new THREE.Color('#24384c');      // no calibration running
const COLOR_MISSING = new THREE.Color('#3a2a36');   // not covered yet
const COLOR_COVERED = new THREE.Color('#1fbf62');
const COLOR_TARGET = new THREE.Color('#ffa020');

// Body frame (x nose, y right, z down) <-> scene (x right, y up, z towards the tail)
const toScene = (x, y, z) => [y, -z, -x];
const toBody = (x, y, z) => [-z, x, -y];

const DEFAULT_VIEW = { yaw: 0.65, pitch: 0.42, dist: 4.4 };

export class MagCal3D {
    /** @param {HTMLCanvasElement} canvas */
    constructor(canvas) {
        this.canvas = canvas;
        this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
        this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;   // see js/core/three.js
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.setClearColor(0x000000, 0);
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(36, 1, 0.05, 50);
        this.view = { ...DEFAULT_VIEW };

        this.focus = 0;          // the compass the sphere, marker and coverage follow
        this.mask = null;        // its MAG_CAL_PROGRESS completion_mask, null = not calibrating
        this.target = -1;        // section to aim for
        this.current = -1;       // section the latest sample is in

        this.clouds = [0, 1, 2].map(i => this._makeCloud(i));
        this._buildSphere();
        this._buildAxes();
        this._buildMarker();
        this._bindControls();

        this._renderPending = false;
        this._fitTimer = null;
        new ResizeObserver(() => this._resize()).observe(canvas);
        this._resize();
    }

    // ── Data in ─────────────────────────────────────────────────────────────

    /**
     * One magnetometer sample, body frame, any unit (RAW_IMU sends mGauss).
     * @param {number} compass - 0..2, priority order (RAW_IMU, SCALED_IMU2, SCALED_IMU3)
     */
    addSample(compass, x, y, z) {
        const c = this.clouds[compass];
        if (!c || !Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return;
        if (!x && !y && !z) return;                       // compass absent: the fields stay 0
        const p = toScene(x, y, z);
        c.latest = p;

        const r = c.fit ? c.fit.r : Math.hypot(p[0], p[1], p[2]);
        const moved = !c.last || Math.hypot(p[0] - c.last[0], p[1] - c.last[1], p[2] - c.last[2]) >= MIN_STEP * r;
        if (moved) {
            c.positions.set(p, c.next * 3);
            c.next = (c.next + 1) % MAX_POINTS;
            c.count = Math.min(c.count + 1, MAX_POINTS);
            c.last = p;
            c.geom.setDrawRange(0, c.count);
            c.geom.attributes.position.needsUpdate = true;
            this._scheduleFit();
        }
        if (!c.fit) this._applyFit(c);          // the provisional frame follows the latest sample
        if (compass === this.focus) this._updateMarker();
        this.requestRender();
    }

    /** Completion mask (10 bytes) of the focused compass; null when not calibrating */
    setCoverage(mask) {
        this.mask = mask ? Array.from(mask) : null;
        this._updateMarker();
        this._paintSections();
        this.requestRender();
    }

    /** Which compass the sphere, marker and coverage refer to */
    setFocus(compass) {
        if (!this.clouds[compass]) return;
        this.focus = compass;
        this.clouds.forEach((c, i) => { c.points.material.opacity = i === compass ? 0.95 : 0.35; });
        this._updateMarker();
        this._paintSections();
        this.requestRender();
    }

    /** Forget every sample (a new calibration starts from an empty sphere) */
    clear() {
        for (const c of this.clouds) {
            c.count = 0; c.next = 0; c.last = null; c.latest = null; c.fit = null;
            c.geom.setDrawRange(0, 0);
            this._applyFit(c);
        }
        this.mask = null;
        this._updateMarker();
        this._paintSections();
        this.requestRender();
    }

    resetView() {
        this.view = { ...DEFAULT_VIEW };
        this.requestRender();
    }

    /** What the toolbar shows under the view */
    stats() {
        const c = this.clouds[this.focus];
        let covered = 0;
        if (this.mask) for (let s = 0; s < SECTION_COUNT; s++) if (maskHas(this.mask, s)) covered++;
        return {
            points: this.clouds.map(k => k.count),
            field: c.fit ? c.fit.r : (c.latest ? Math.hypot(...c.latest) : null),
            fitted: !!c.fit,
            covered: this.mask ? covered : null
        };
    }

    // ── Scene ───────────────────────────────────────────────────────────────

    _makeCloud(i) {
        const positions = new Float32Array(MAX_POINTS * 3);
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geom.setDrawRange(0, 0);
        const points = new THREE.Points(geom, new THREE.PointsMaterial({
            color: MAG_COLORS[i], size: 3, sizeAttenuation: false,
            transparent: true, opacity: i === 0 ? 0.95 : 0.35, depthWrite: false
        }));
        points.frustumCulled = false;   // the bounding sphere is never recomputed
        points.renderOrder = 2;
        const group = new THREE.Group();
        group.add(points);
        this.scene.add(group);
        return { group, points, geom, positions, count: 0, next: 0, last: null, latest: null, fit: null };
    }

    /** The 80 sections as a translucent sphere: back faces, then front faces */
    _buildSphere() {
        const N = 4;   // each section split N×N times so it bulges with the sphere
        const perSection = N * N * 3;
        const pos = new Float32Array(SECTION_COUNT * perSection * 3);
        const col = new Float32Array(SECTION_COUNT * perSection * 3);
        const lerp = (a, b, c, u, v) => {
            const w = 1 - u - v;
            const x = a[0] * w + b[0] * u + c[0] * v, y = a[1] * w + b[1] * u + c[1] * v, z = a[2] * w + b[2] * u + c[2] * v;
            const l = Math.hypot(x, y, z);
            return toScene(x / l, y / l, z / l);
        };
        let k = 0;
        for (let s = 0; s < SECTION_COUNT; s++) {
            const { a, b, c } = SECTIONS[s];
            for (let i = 0; i < N; i++) {
                for (let j = 0; j < N - i; j++) {
                    const tris = [[[i, j], [i + 1, j], [i, j + 1]]];
                    if (j < N - i - 1) tris.push([[i + 1, j], [i + 1, j + 1], [i, j + 1]]);
                    for (const tri of tris) {
                        for (const [u, v] of tri) { pos.set(lerp(a, b, c, u / N, v / N), k * 3); k++; }
                    }
                }
            }
        }
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
        geom.setAttribute('color', new THREE.BufferAttribute(col, 3));
        this.sphereGeom = geom;
        this.sphereVertsPerSection = perSection;

        // SECTIONS wind outwards and toScene is a rotation, so front faces face out:
        // the far half is drawn as back faces first, faint, the near half last
        const back = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({
            vertexColors: true, transparent: true, opacity: 0.16, side: THREE.BackSide, depthWrite: false
        }));
        back.renderOrder = 1;
        const front = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({
            vertexColors: true, transparent: true, opacity: 0.38, side: THREE.FrontSide, depthWrite: false
        }));
        front.renderOrder = 3;
        this.scene.add(back, front);

        // Section outlines along great circles
        const seg = [];
        const arc = (p, q) => {
            for (let t = 0; t < 6; t++) {
                for (const f of [t / 6, (t + 1) / 6]) {
                    const x = p[0] + (q[0] - p[0]) * f, y = p[1] + (q[1] - p[1]) * f, z = p[2] + (q[2] - p[2]) * f;
                    const l = Math.hypot(x, y, z) / 1.002;
                    seg.push(...toScene(x / l, y / l, z / l));
                }
            }
        };
        for (const { a, b, c } of SECTIONS) { arc(a, b); arc(b, c); arc(c, a); }
        const lineGeom = new THREE.BufferGeometry();
        lineGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(seg), 3));
        const lines = new THREE.LineSegments(lineGeom, new THREE.LineBasicMaterial({
            color: 0x9fd8ff, transparent: true, opacity: 0.13, depthWrite: false
        }));
        lines.renderOrder = 3;
        this.scene.add(lines);
        this._paintSections();
    }

    _paintSections() {
        if (!this.sphereGeom) return;
        const col = this.sphereGeom.attributes.color;
        for (let s = 0; s < SECTION_COUNT; s++) {
            let c = COLOR_IDLE;
            if (this.mask) c = s === this.target ? COLOR_TARGET : (maskHas(this.mask, s) ? COLOR_COVERED : COLOR_MISSING);
            const start = s * this.sphereVertsPerSection;
            for (let v = 0; v < this.sphereVertsPerSection; v++) col.setXYZ(start + v, c.r, c.g, c.b);
        }
        col.needsUpdate = true;
    }

    _buildAxes() {
        const axes = [
            { dir: [1, 0, 0], color: '#ff5d5d', label: 'NOSE' },
            { dir: [0, 1, 0], color: '#5ddc6a', label: 'RIGHT' },
            { dir: [0, 0, 1], color: '#5d9dff', label: 'DOWN' },
        ];
        for (const { dir, color, label } of axes) {
            const end = toScene(dir[0] * 1.45, dir[1] * 1.45, dir[2] * 1.45);
            const g = new THREE.BufferGeometry();
            g.setAttribute('position', new THREE.BufferAttribute(new Float32Array([0, 0, 0, ...end]), 3));
            const line = new THREE.Line(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.85 }));
            line.renderOrder = 4;
            this.scene.add(line);
            const sprite = makeLabel(label, color);
            sprite.position.set(end[0] * 1.12, end[1] * 1.12, end[2] * 1.12);
            this.scene.add(sprite);
        }
        // The vehicle at the centre: an arrowhead pointing at the nose
        const tri = [[0.3, 0, 0], [-0.18, -0.17, 0], [-0.18, 0.17, 0]].flatMap(p => toScene(...p));
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(tri), 3));
        const vehicle = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
            color: 0xdfe8f2, side: THREE.DoubleSide, transparent: true, opacity: 0.85
        }));
        vehicle.renderOrder = 4;
        this.scene.add(vehicle);
    }

    _buildMarker() {
        this.marker = new THREE.Mesh(
            new THREE.SphereGeometry(0.045, 16, 12),
            new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false })
        );
        this.marker.renderOrder = 5;
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
        this.markerLine = new THREE.Line(g, new THREE.LineBasicMaterial({
            color: 0xffffff, transparent: true, opacity: 0.55, depthTest: false
        }));
        this.markerLine.renderOrder = 5;
        this.marker.visible = this.markerLine.visible = false;
        this.scene.add(this.marker, this.markerLine);
    }

    /** Marker on the latest sample of the focused compass; the section to aim for */
    _updateMarker() {
        const c = this.clouds[this.focus];
        this.current = -1;
        this.target = -1;
        if (!c.latest) {
            this.marker.visible = this.markerLine.visible = false;
            return;
        }
        const { center, r } = this._frame(c);
        const m = [(c.latest[0] - center[0]) / r, (c.latest[1] - center[1]) / r, (c.latest[2] - center[2]) / r];
        this.marker.position.set(m[0], m[1], m[2]);
        const lp = this.markerLine.geometry.attributes.position;
        lp.setXYZ(1, m[0], m[1], m[2]);
        lp.needsUpdate = true;
        this.marker.visible = this.markerLine.visible = true;

        const body = toBody(...m);
        this.current = sectionOf(body);
        if (this.mask) {
            const l = Math.hypot(...body) || 1;
            let best = -Infinity;
            for (let s = 0; s < SECTION_COUNT; s++) {
                if (maskHas(this.mask, s)) continue;
                const k = SECTIONS[s].center;
                const d = (k[0] * body[0] + k[1] * body[1] + k[2] * body[2]) / l;
                if (d > best) { best = d; this.target = s; }
            }
        }
        this._paintSections();
    }

    // ── Sphere fit ──────────────────────────────────────────────────────────

    /** Centre and radius that put a compass's points on the unit sphere */
    _frame(c) {
        if (c.fit) return c.fit;
        // Before a fit: around the origin — RAW_IMU fields already carry the
        // stored offsets, so that is where an existing calibration says it is.
        const r = c.latest ? Math.hypot(...c.latest) : 1;
        return { center: [0, 0, 0], r: r || 1 };
    }

    _scheduleFit() {
        if (this._fitTimer) return;
        this._fitTimer = setTimeout(() => {
            this._fitTimer = null;
            for (const c of this.clouds) {
                c.fit = fitSphere(c.positions, c.count);
                this._applyFit(c);
            }
            this._updateMarker();
            this.requestRender();
        }, 400);
    }

    _applyFit(c) {
        const { center, r } = this._frame(c);
        c.group.scale.setScalar(1 / r);
        c.group.position.set(-center[0] / r, -center[1] / r, -center[2] / r);
    }

    // ── Camera, rendering ───────────────────────────────────────────────────

    _bindControls() {
        const el = this.canvas;
        let drag = null;
        el.addEventListener('pointerdown', (e) => {
            drag = { x: e.clientX, y: e.clientY };
            el.setPointerCapture(e.pointerId);
        });
        el.addEventListener('pointermove', (e) => {
            if (!drag) return;
            this.view.yaw -= (e.clientX - drag.x) * 0.008;
            this.view.pitch = Math.max(-1.45, Math.min(1.45, this.view.pitch + (e.clientY - drag.y) * 0.008));
            drag = { x: e.clientX, y: e.clientY };
            this.requestRender();
        });
        const end = () => { drag = null; };
        el.addEventListener('pointerup', end);
        el.addEventListener('pointercancel', end);
        el.addEventListener('wheel', (e) => {
            e.preventDefault();
            this.view.dist = Math.max(2.2, Math.min(10, this.view.dist * (1 + e.deltaY * 0.001)));
            this.requestRender();
        }, { passive: false });
        el.addEventListener('dblclick', () => this.resetView());
    }

    _resize() {
        const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
        if (!w || !h) return;
        this.renderer.setSize(w, h, false);
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.requestRender();
    }

    requestRender() {
        if (this._renderPending) return;
        this._renderPending = true;
        requestAnimationFrame(() => {
            this._renderPending = false;
            if (!this.canvas.clientWidth) return;          // page hidden
            const { yaw, pitch, dist } = this.view;
            this.camera.position.set(
                dist * Math.cos(pitch) * Math.sin(yaw),
                dist * Math.sin(pitch),
                dist * Math.cos(pitch) * Math.cos(yaw)
            );
            this.camera.lookAt(0, 0, 0);
            this.renderer.render(this.scene, this.camera);
        });
    }
}

/**
 * Least-squares sphere through the points: |p|² = 2 c·p + d, linear in (c, d),
 * r² = d + |c|². Rejected when the points cover too little of the sphere or
 * scatter too far from it to say where its centre is.
 * @returns {{center:number[], r:number}|null}
 */
function fitSphere(pos, n) {
    if (n < FIT_MIN_POINTS) return null;
    const A = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    const B = [0, 0, 0, 0];
    for (let i = 0; i < n; i++) {
        const x = pos[i * 3], y = pos[i * 3 + 1], z = pos[i * 3 + 2];
        const row = [2 * x, 2 * y, 2 * z, 1];
        const rhs = x * x + y * y + z * z;
        for (let j = 0; j < 4; j++) {
            B[j] += row[j] * rhs;
            for (let k = 0; k < 4; k++) A[j][k] += row[j] * row[k];
        }
    }
    const sol = solve4(A, B);
    if (!sol) return null;
    const center = [sol[0], sol[1], sol[2]];
    const r2 = sol[3] + center[0] ** 2 + center[1] ** 2 + center[2] ** 2;
    if (!(r2 > 0)) return null;
    const r = Math.sqrt(r2);

    const sections = new Set();
    let sq = 0;
    for (let i = 0; i < n; i++) {
        const d = [pos[i * 3] - center[0], pos[i * 3 + 1] - center[1], pos[i * 3 + 2] - center[2]];
        sq += (Math.hypot(...d) - r) ** 2;
        sections.add(sectionOf(toBody(...d)));
    }
    if (sections.size < FIT_MIN_SECTIONS || Math.sqrt(sq / n) > 0.25 * r) return null;
    return { center, r };
}

/** Gaussian elimination with partial pivoting; null when singular */
function solve4(A, B) {
    const m = A.map((row, i) => [...row, B[i]]);
    for (let c = 0; c < 4; c++) {
        let p = c;
        for (let r = c + 1; r < 4; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
        if (Math.abs(m[p][c]) < 1e-9) return null;
        [m[c], m[p]] = [m[p], m[c]];
        for (let r = 0; r < 4; r++) {
            if (r === c) continue;
            const f = m[r][c] / m[c][c];
            for (let k = c; k < 5; k++) m[r][k] -= f * m[c][k];
        }
    }
    return m.map((row, i) => row[4] / row[i][i]);
}

function makeLabel(text, color) {
    const cv = document.createElement('canvas');
    cv.width = 128; cv.height = 48;
    const ctx = cv.getContext('2d');
    ctx.font = 'bold 26px "Rajdhani", "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = color;
    ctx.fillText(text, 64, 24);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
        map: new THREE.CanvasTexture(cv), transparent: true, depthTest: false
    }));
    sprite.scale.set(0.5, 0.19, 1);
    sprite.renderOrder = 6;
    return sprite;
}
