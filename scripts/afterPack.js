/**
 * afterPack.js - electron-builder hook
 * Replaces the Linux-rebuilt serialport native binary with the correct
 * platform prebuilt, since node-gyp-build checks build/Release/ FIRST.
 */

const fs = require('fs');
const path = require('path');

exports.default = async function afterPack(context) {
    const platform = context.electronPlatformName; // 'win32', 'linux', 'darwin'
    const arch = context.arch === 1 ? 'x64' : context.arch === 3 ? 'arm64' : 'x64';

    const platformMap = {
        win32: `win32-${arch}`,
        darwin: `darwin-${arch}`,
        linux:  `linux-${arch}`,
    };

    const prebuildTag = platformMap[platform];
    if (!prebuildTag) return;

    const unpacked = path.join(
        context.appOutDir, 'resources', 'app.asar.unpacked',
        'node_modules', '@serialport', 'bindings-cpp'
    );

    const destBinary = path.join(unpacked, 'build', 'Release', 'bindings.node');
    const srcBinary  = path.join(unpacked, 'prebuilds', prebuildTag, '@serialport+bindings-cpp.node');

    if (!fs.existsSync(srcBinary)) {
        console.warn(`[afterPack] Prebuilt not found: ${srcBinary}`);
        return;
    }

    fs.mkdirSync(path.dirname(destBinary), { recursive: true });
    fs.copyFileSync(srcBinary, destBinary);
    console.log(`[afterPack] Installed ${prebuildTag} serialport binary → build/Release/bindings.node`);
};
