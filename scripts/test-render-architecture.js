#!/usr/bin/env node
// Domain tests run with no Three/DOM; adapter checks then use the vendored math
// and resource classes, plus a renderer stub (actual GPU checks live in smoke).
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const root = path.join(__dirname, '..');
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-architecture-'));
fs.writeFileSync(path.join(tmp, 'package.json'), '{"type":"module"}');
for (const relative of ['js/render', 'js/engine/three', 'js/engine/Layers.js', 'js/core/constants.js',
    'js/core/utils.js', 'js/terrain/SatelliteDetail.js', 'vendor/three/three.core.js']) {
    fs.mkdirSync(path.dirname(path.join(tmp, relative)), { recursive: true });
    fs.cpSync(path.join(root, relative), path.join(tmp, relative), { recursive: true });
}
const load = relative => import(pathToFileURL(path.join(tmp, relative)));
const near = (a, b, epsilon = 1e-7) => assert(Math.abs(a - b) <= epsilon, `${a} != ${b}`);

(async () => {
    try {
        const { RenderWorld } = await load('js/render/RenderWorld.js');
        const { PointCloudData, setYXZQuaternion } = await load('js/render/PointCloudData.js');
        const { renderDocument, encodeRenderWorld, encodeRenderPacket, decodeRenderPacket } = await load('js/render/RenderPacket.js');
        assert.equal(global.THREE, undefined);
        const world = new RenderWorld({ lat: 47.2603, lon: 11.3439 });
        const item = { chunkKey: '47_11_10_20', latBase: 47, lonBase: 11, cx: 10, cy: 20, size: 3601, vertsPerChunk: 120 };
        const heights = new Int16Array(16 * 16).fill(1200); heights[7] = -32768;
        const record = world.terrain.put(item, 8, heights, -32768, 1200);
        assert.equal(record.heights, heights, 'domain takes ownership without copying');
        assert.equal(record.width, 16);
        near(record.bounds[0] - record.bounds[1], 1 / 30);
        const edge = world.terrain.put({ ...item, chunkKey: 'adjacent', cx: 11 }, 8, heights.slice(), -32768, 1200);
        near(record.grid[0] + 15 * record.grid[2], edge.grid[0], 1e-6);
        assert.throws(() => world.terrain.put(item, 8, new Int16Array(1), 0, 0), /Invalid/);
        console.log('PASS terrain ownership, axis convention, neighbouring chunk seam and validation');

        const points = new PointCloudData({ chunkSize: 2, liveCapacity: 3, maxPoints: 4 });
        points.setOrigin({ epoch: 1, lat: 47, lon: 11, alt: 100, mPerLat: 111320, mPerLon: 75000 });
        let updates = points.appendMapped({ enu: new Float32Array([1,2,3, 4,5,6, 7,8,9, 10,11,12, 13,14,15]), epoch: 1 });
        assert.deepEqual(updates.map(u => [u.start,u.count]), [[0,2],[0,2]]);
        assert.equal(points.total, 4); assert.deepEqual(Array.from(points.chunks[0].positions), [1,3,-2,4,6,-5]);
        assert.equal(points.appendMapped({ enu: new Float32Array([1,2,3]), epoch: 0 }).length, 0);
        const liveUpdates = points.appendLive({ xyz: new Float32Array([1,2,3, 4,5,6, 7,8,9, 10,11,12]) }, 42);
        assert.deepEqual(liveUpdates, [{ start:0,count:3 }, { start:0,count:1 }]);
        assert.equal(points.live.count, 3); assert.equal(points.live.head, 1);
        assert.deepEqual(Array.from(points.live.positions.slice(0,3)), [11,-12,-10]);
        assert.equal(points.live.birth[0], 42);
        world.pointClouds.set('livox', points);
        const packet = encodeRenderWorld(world);
        const outputIndex = process.argv.indexOf('--packet-out');
        if (outputIndex >= 0) {
            assert(process.argv[outputIndex + 1], '--packet-out requires a filename');
            fs.writeFileSync(path.resolve(process.argv[outputIndex + 1]), Buffer.from(packet));
        }
        const restored = decodeRenderPacket(packet);
        assert.deepEqual(Array.from(restored.terrain[0].heights), Array.from(heights));
        assert.equal(restored.pointClouds[0].chunks[0].positions.length, 6);
        assert.deepEqual(Array.from(restored.pointClouds[0].live.positions), Array.from(points.live.positions));
        assert.equal(heights.byteLength, 512, 'export must never detach renderer buffers');
        assert.throws(() => decodeRenderPacket(packet.slice(0, packet.byteLength - 1)), /Truncated/);
        const badVersion = packet.slice(0); new DataView(badVersion).setUint32(4, 99, true);
        assert.throws(() => decodeRenderPacket(badVersion), /Unsupported/);
        assert.throws(() => encodeRenderPacket({ schemaVersion:1, leak: new Date() }), /plain data/);
        points.setOrigin({ epoch: 2 }); assert.equal(points.total, 0); assert.equal(points.generation, 1);
        console.log('PASS point block boundaries, live ring wrap, stale epoch, snapshot round trip and ownership');

        const THREE = await load('vendor/three/three.core.js');
        global.THREE = THREE;
        const { captureThreeCamera } = await load('js/engine/three/CameraAdapter.js');
        const camera = new THREE.PerspectiveCamera(60, 16 / 9, 1, 300000);
        const parent = new THREE.Group(); parent.position.set(8,4,-3); parent.rotation.y = 0.2; parent.add(camera);
        camera.position.set(120,100,80); camera.rotation.set(0.2,-0.7,0.1,'YXZ');
        camera.setViewOffset(1920,1080,200,0,1720,1080);
        const viewport = { domElement:{width:1800,height:1050}, getPixelRatio:() => 1.5 };
        captureThreeCamera(camera, world.camera, viewport);
        const frustum = new THREE.Frustum().setFromProjectionMatrix(new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix,camera.matrixWorldInverse));
        for (let i = 0; i < 300; i++) {
            const sphere = { x:Math.sin(i)*15000, y:Math.cos(i)*12000, z:Math.sin(i*2)*15000, r: 10 + i*30 };
            assert.equal(world.camera.intersectsSphere(sphere), frustum.intersectsSphere(new THREE.Sphere(new THREE.Vector3(sphere.x,sphere.y,sphere.z),sphere.r)));
        }
        const worldQ = camera.getWorldQuaternion(new THREE.Quaternion());
        world.camera.quaternion.forEach((v,i) => near(v, worldQ.toArray()[i]));
        const q = [0,0,0,1]; setYXZQuaternion(q, .4, -.8, .3);
        const expected = new THREE.Quaternion().setFromEuler(new THREE.Euler(.4,-.8,.3,'YXZ')).toArray();
        q.forEach((v,i) => near(v, expected[i]));
        console.log('PASS backend-independent frustum against Three, camera parent/view offset and body quaternion');

        const { createThreeTerrainRenderer } = await load('js/engine/three/TerrainRenderer.js');
        const { createThreePointCloudRenderer } = await load('js/engine/three/PointCloudRenderer.js');
        const canvas = new EventTarget(); canvas.width=1800; canvas.height=1050;
        const renderer = { domElement:canvas, capabilities:{maxTextureSize:8192,getMaxAnisotropy:() => 8},
            getContext:() => ({getExtension:() => ({})}), initTexture(){}, getPixelRatio:() => 1.5 };
        const scene = new THREE.Scene();
        let disposals = 0;
        const terrain = createThreeTerrainRenderer({three:THREE,scene,renderer,world,loadTileImage(){},onTextureDisposed:() => disposals++});
        const mesh = terrain.createChunk(record, {});
        assert.equal(mesh.data.lodStep,record.step);
        assert.equal(mesh.isMesh,undefined,'terrain controller receives only a logical handle');
        const texture = scene.children.find(o => o.isInstancedMesh).material.userData.terrain.uHeights.value;
        assert.equal(texture.image.data[7], -32768);
        const version = texture.version; canvas.dispatchEvent(new Event('webglcontextrestored'));
        assert(texture.version > version);
        const bc1 = terrain.makeCompressedTexture({width:4,height:4,padWidth:4,padHeight:4,mips:[{width:4,height:4,data:new Uint8Array(8)}]});
        terrain.setMap(mesh,bc1); assert.equal(world.terrain.chunks.get(record.id).appearance.kind,'bc1');
        assert(scene.children.some(o => o.isMesh && o.material.map === bc1));
        const lodHeights = new Int16Array(31*31).fill(1300);
        const lod = world.terrain.put(item,4,lodHeights,1300,1300);
        terrain.updateHeights(mesh,lod); assert(scene.children.some(o => o.material?.map === bc1));
        assert.equal(terrain.getStats().heightLayers,1);
        terrain.removeChunk(mesh); world.terrain.remove(record.id);
        assert.equal(disposals,1); assert.equal(terrain.getStats().heightLayers,0);
        terrain.dispose();
        const model = new PointCloudData({chunkSize:2,liveCapacity:3}); model.setOrigin({epoch:1}); model.visible=true;
        const sensor = createThreePointCloudRenderer({three:THREE,scene,renderer,model});
        const changed = model.appendMapped({enu:new Float32Array([1,2,3]),epoch:1}); sensor.applyMapped(changed);
        sensor.updateFrame();
        const group = scene.getObjectByName('lidarCloud'), attr = group.children[0].geometry.getAttribute('position');
        assert.equal(attr.array, model.chunks[0].positions, 'GPU attribute shares domain allocation');
        assert.deepEqual(attr.updateRanges,[{start:0,count:3}]);
        model.pointSize=3; sensor.updateFrame(); assert.equal(group.children[0].material.uniforms.uSize.value,4.5);
        model.setOrigin({epoch:2}); sensor.updateFrame(); assert.equal(group.children.length,0);
        sensor.dispose(); assert.equal(scene.getObjectByName('lidarCloud'),undefined);
        console.log('PASS adapters: context restore, LOD with satellite map, resource disposal and partial point uploads');
    } finally {
        delete global.THREE; fs.rmSync(tmp,{recursive:true,force:true});
    }
})().catch(error => { console.error(error); process.exitCode=1; });
