#!/usr/bin/env node
/**
 * test-radio-link.js - offline checks for js/mission/RadioLink.js
 *
 * The renderer modules are ES modules in a CommonJS package, so they are
 * copied to a temp folder as .mjs and imported from there. Terrain is a
 * synthetic function: flat ground, a ridge, a hole — enough to check the
 * free-space budget, the Fresnel clearance rule and the knife-edge loss.
 *
 *   node scripts/test-radio-link.js
 */

const fs = require('fs');
const os = require('os');
const path = require('path');

const root = path.join(__dirname, '..');
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-radio-'));
for (const name of ['RouteModel', 'RadioLink']) {
    const src = fs.readFileSync(path.join(root, 'js', 'mission', `${name}.js`), 'utf8').replace(/from '\.\/(\w+)\.js'/g, "from './$1.mjs'");
    fs.writeFileSync(path.join(tmp, `${name}.mjs`), src);
}

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}
const near = (a, b, tol) => Math.abs(a - b) <= tol;

(async () => {
    const RL = await import(path.join(tmp, 'RadioLink.mjs'));
    const RM = await import(path.join(tmp, 'RouteModel.mjs'));
    const { LINK } = RL;

    const radio = { ...RM.defaultRadio(), preset: 'rfd900x', ...RM.RADIO_PRESETS.rfd900x, losses: 1, minMargin: 10, groundHeight: 2 };
    const gsPos = { lat: 45.0, lng: 9.0 };
    const frame = RM.localFrame(gsPos);
    const at = (east, north) => frame.toLL({ x: east, y: north });

    // ---------- 1. Primitives ----------
    check('FSPL 5 km @ 915 MHz', near(RL.fspl(5000, 915), 105.65, 0.05), RL.fspl(5000, 915).toFixed(2));
    check('FSPL floors at 1 m', RL.fspl(0, 915) === RL.fspl(1, 915));
    check('J(0) = 6 dB grazing', near(RL.knifeEdgeLoss(0), 6.0, 0.1), RL.knifeEdgeLoss(0).toFixed(2));
    check('J(ν) = 0 below −0.78', RL.knifeEdgeLoss(-1) === 0 && RL.knifeEdgeLoss(-0.79) === 0);
    check('J(ν) grows with ν', RL.knifeEdgeLoss(3) > RL.knifeEdgeLoss(1) && RL.knifeEdgeLoss(1) > RL.knifeEdgeLoss(0));
    const r1 = RL.fresnelRadius(2500, 2500, 915);
    check('F1 radius mid-path 5 km @ 915 MHz ≈ 20.2 m', near(r1, 20.24, 0.1), r1.toFixed(2));
    const range0 = RL.freeSpaceRange(radio, 0);
    check('free-space range RFD900x (138 dB budget) ≈ 207 km', near(range0 / 1000, 207, 3), (range0 / 1000).toFixed(1));
    check('range with margin is shorter', RL.freeSpaceRange(radio, 10) < range0);

    // ---------- 2. Flat ground ----------
    const flat = () => 100;
    const gs = RL.groundStation(radio, gsPos, flat);
    check('ground station antenna MSL', gs.elev === 100 && gs.antennaMsl === 102);
    const p5 = at(0, 5000);
    let ev = RL.evaluateLink(radio, gs, { ...p5, altMsl: 200 }, flat);
    check('flat 5 km: RSSI ≈ −72.7 dBm', near(ev.rssi, -72.65, 0.1), ev.rssi.toFixed(2));
    check('flat 5 km: margin ≈ 32 dB', near(ev.margin, 32.35, 0.1), ev.margin.toFixed(2));
    check('flat 5 km: Fresnel clear, GOOD', ev.fresnelClear && !ev.obstructed && ev.clazz === LINK.GOOD, `nu=${ev.nu.toFixed(2)}`);

    // Antenna on the ground: the first Fresnel zone grazes the ground next to the mast
    const low = RL.groundStation({ ...radio, groundHeight: 0.3 }, gsPos, flat);
    ev = RL.evaluateLink(radio, low, { ...p5, altMsl: 200 }, flat);
    check('antenna at 0.3 m: Fresnel intruded → DEGRADED', ev.clazz === LINK.DEGRADED && ev.loss > 0 && !ev.obstructed, `nu=${ev.nu.toFixed(2)} loss=${ev.loss.toFixed(1)}`);

    // Far away in free space: the margin shrinks below the safety margin, then under the sensitivity
    const p150 = at(150000, 0);
    ev = RL.evaluateLink(radio, gs, { ...p150, altMsl: 5000 }, flat, { maxSamples: 200 });
    check('150 km, 5 km up: MARGINAL', ev.clazz === LINK.MARGINAL, `margin=${ev.margin.toFixed(1)}`);
    const p400 = at(0, -400000);
    ev = RL.evaluateLink(radio, gs, { ...p400, altMsl: 20000 }, flat, { maxSamples: 200 });
    check('400 km: NONE', ev.clazz === LINK.NONE, `margin=${ev.margin.toFixed(1)}`);

    // ---------- 3. A ridge between operator and aircraft ----------
    const ridge = (lat, lng) => {
        const q = frame.toXY({ lat, lng });
        return Math.abs(q.y - 2500) < 250 && Math.abs(q.x) < 3000 ? 300 : 100;
    };
    ev = RL.evaluateLink(radio, gs, { ...p5, altMsl: 200 }, ridge);
    check('ridge 300 m: LOS blocked', ev.obstructed && ev.nu > 5, `nu=${ev.nu.toFixed(2)} loss=${ev.loss.toFixed(1)}`);
    check('ridge 300 m: heavy loss → NONE or MARGINAL', ev.clazz === LINK.NONE || ev.clazz === LINK.MARGINAL, `margin=${ev.margin.toFixed(1)}`);
    check('ridge: worst sample on the ridge', ev.worst && near(ev.worst.dist, 2500, 260) && ev.worst.elev === 300, JSON.stringify(ev.worst));
    // Fly higher: the line clears the ridge but the Fresnel zone does not
    ev = RL.evaluateLink(radio, gs, { ...p5, altMsl: 555 }, ridge);
    check('ridge, aircraft 555 m: LOS clear, Fresnel intruded → DEGRADED', !ev.obstructed && !ev.fresnelClear && ev.clazz === LINK.DEGRADED, `nu=${ev.nu.toFixed(2)} loss=${ev.loss.toFixed(1)}`);
    ev = RL.evaluateLink(radio, gs, { ...p5, altMsl: 700 }, ridge);
    check('ridge, aircraft 700 m: GOOD', ev.clazz === LINK.GOOD, `nu=${ev.nu.toFixed(2)}`);

    // Earth bulge: 40 km over flat ground at low altitude — the bulge (≈47 m mid-path) eats the clearance
    const p40 = at(40000, 0);
    ev = RL.evaluateLink(radio, gs, { ...p40, altMsl: 130 }, flat, { maxSamples: 400 });
    check('40 km at 30 m AGL: curvature blocks LOS', ev.obstructed, `nu=${ev.nu.toFixed(2)}`);
    ev = RL.evaluateLink(radio, gs, { ...p40, altMsl: 400 }, flat, { maxSamples: 400 });
    check('40 km at 300 m AGL: LOS clear but the zone grazes the ground by the mast → DEGRADED', !ev.obstructed && ev.clazz === LINK.DEGRADED, `nu=${ev.nu.toFixed(2)}`);
    ev = RL.evaluateLink(radio, gs, { ...p40, altMsl: 1100 }, flat, { maxSamples: 400 });
    check('40 km at 1000 m AGL: GOOD', ev.clazz === LINK.GOOD, `nu=${ev.nu.toFixed(2)}`);

    // No terrain data
    const hole = (lat, lng) => (frame.toXY({ lat, lng }).y > 1000 ? null : 100);
    ev = RL.evaluateLink(radio, gs, { ...p5, altMsl: 200 }, hole);
    check('missing terrain → UNKNOWN', ev.clazz === LINK.UNKNOWN);

    // ---------- 4. Route analysis ----------
    const nav = [
        { lat: gsPos.lat, lng: gsPos.lng, dist: 0, altMsl: 100, isHome: true },
        { ...at(0, 1000), dist: 1000, altMsl: 200, segId: 'a' },
        { ...at(0, 5000), dist: 5000, altMsl: 200, segId: 'a' },
        { ...at(3500, 5000), dist: 8500, altMsl: 200, segId: 'b' },
    ];
    const an = RL.analyzeRoute(radio, gs, nav, ridge, { stepM: 50 });
    check('analysis: samples cover the route', an && an.samples.length > 100 && near(an.samples[an.samples.length - 1].dist, 8500, 1));
    check('analysis: starts GOOD before the ridge', an.samples[3].clazz === LINK.GOOD);
    check('analysis: shadowed behind the ridge', an.lengths[LINK.NONE] + an.lengths[LINK.MARGINAL] > 1000, JSON.stringify(an.lengths));
    check('analysis: worst is the lowest class', an.worst && an.worst.clazz === Math.min(...an.samples.filter(s => s.clazz !== LINK.UNKNOWN).map(s => s.clazz)));
    check('analysis: runs are continuous', an.runs.every((r, i) => i === 0 || r.points[0][0] === an.runs[i - 1].points[an.runs[i - 1].points.length - 1][0]));
    check('summary text', /no link|marginal/.test(RL.summarizeLink(an)), RL.summarizeLink(an));

    // ---------- 5. Coverage raster ----------
    const t0 = Date.now();
    const cov = await RL.computeCoverage(radio, gs, t => t + 100, ridge, { range: 6000, rays: 180, maxCount: 120, budgetMs: 1000 });
    const dt = Date.now() - t0;
    check('coverage: shape', cov.rays === 180 && cov.count === 120 && cov.classes.length === 180 * 120, `${dt} ms`);
    check('coverage: GOOD south of the operator', RL.coverageClassAt(cov, 0, -3000) === LINK.GOOD);
    const behind = RL.coverageClassAt(cov, 0, 5000);
    check('coverage: shadow behind the ridge', behind === LINK.NONE || behind === LINK.MARGINAL, `class=${behind}`);
    check('coverage: outside the range is UNKNOWN', RL.coverageClassAt(cov, 7000, 0) === LINK.UNKNOWN);
    const evN = RL.evaluateLink(radio, gs, { ...at(0, -3000), altMsl: 200 }, ridge);
    check('coverage agrees with evaluateLink', RL.coverageClassAt(cov, 0, -3000) === evN.clazz);
    const token = { cancelled: true };
    const cancelled = await RL.computeCoverage(radio, gs, t => t + 100, flat, { range: 6000, rays: 720, budgetMs: 0, token });
    check('coverage: cancellation returns null', cancelled === null);

    // ---------- 6. Timing of the full-size raster ----------
    const t1 = Date.now();
    const full = await RL.computeCoverage(radio, gs, t => t + 100, ridge, { range: 15000, budgetMs: 1e9 });
    const dt1 = Date.now() - t1;
    check('full raster 720 × 400 completes', full && full.count === 400, `${dt1} ms`);

    fs.rmSync(tmp, { recursive: true, force: true });
    console.log(failures ? `\n${failures} FAILED` : '\nall passed');
    process.exit(failures ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
