# roslib 2.1.0 — single-file ES module

`roslib.esm.min.js` is [roslib](https://github.com/RobotWebTools/roslibjs) 2.1.0
(BSD-2-Clause, see `LICENSE`) bundled with its dependencies into one ES module,
so the renderer can load it offline from a module Web Worker
(`js/ros/RosWorker.js`). The npm package itself imports bare specifiers
(`eventemitter3`, `cbor2`, ...) that a browser cannot resolve without a bundler,
and its own import map points at unpkg.

Bundled: roslib 2.1.0, eventemitter3 5.0.4, uuid 13.0.2, bson 7.3.3 (Apache-2.0),
cbor2 2.4.0, fast-png 8.0.0, fflate 0.8.3, iobuffer 6.0.1, @xmldom/xmldom 0.9.12,
@cto.af/wtf8 0.0.6 (all MIT unless noted); their license headers are kept
inline in the bundle.

`ws` is replaced by `ws-stub.js`, which re-exports the global `WebSocket`:
roslib only uses `ws` where there is no global `WebSocket` (plain Node older
than 22), but its transport module imports it statically, and a bare `"ws"`
import left in the bundle would stop the module from loading in a browser.
The bundle has no imports at all.

Rebuild:

```
npm install roslib@2.1.0 esbuild@0.25
echo "export { Ros, Topic, Service } from 'roslib';" > entry.js
npx esbuild entry.js --bundle --format=esm --platform=browser --target=chrome120 \
    --alias:ws=./ws-stub.js --minify --legal-comments=inline --outfile=roslib.esm.min.js
```
