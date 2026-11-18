#!/usr/bin/env bash
# Linux AppImage of CORV GCS (guide, "Build, pacchetti e CI": AppImage and deb, as today).
#
#   RUNTIME=<type-2 runtime or any type-2 AppImage> tools/build-appimage.sh
#
# CORV_VERSION_LABEL (the release tag, e.g. 2.0.0-alpha.8) is the version the
# program reports and the update check compares, and names the file; without
# it the build says <version>-dev.
#
# Builds linux-release, lays out an AppDir with the Qt of QT_DIR (default
# ~/Qt/6.12.0/gcc_64): the Qt libraries the program and its plugins need, the
# platform (xcb, wayland), image, icon, SQL, TLS and portal plugins, and only
# the QML modules qmlimportscanner finds in libs/ui/qml. Then squashfs (gzip)
# behind the AppImage runtime: dist/CORV-GCS-<version>-x86_64.AppImage.
#
# Nothing is downloaded. RUNTIME is the AppImage type-2 runtime
# (https://github.com/AppImage/type2-runtime/releases), or an existing type-2
# AppImage whose runtime is cut out (the ELF part before its squashfs).
# Default: build/appimage-tools/runtime-x86_64.
#
# The AppImage runs on distributions with a glibc at least as new as the build
# machine's: release builds come from the oldest Ubuntu LTS to support (the CI's
# ubuntu-22.04). Inside it the program folder is read-only, so the data root is
# the per-user folder of the Electron version (~/.config/CORV GCS/data); the .glb
# models travel in models/ next to the executable.
set -euo pipefail

cd "$(dirname "$0")/.."
QT_DIR="${QT_DIR:-$HOME/Qt/6.12.0/gcc_64}"
BUILD=build/linux-release
APPDIR=build/AppDir
RUNTIME="${RUNTIME:-build/appimage-tools/runtime-x86_64}"
VERSION="$(sed -n 's/^project(CorvGcs VERSION \([0-9.]*\).*/\1/p' CMakeLists.txt)"
LABEL="${CORV_VERSION_LABEL#v}"
OUT="dist/CORV-GCS-${LABEL:-$VERSION}-x86_64.AppImage"

[ -x "$QT_DIR/bin/qmake" ] || { echo "Qt not found in $QT_DIR (set QT_DIR)" >&2; exit 1; }
[ -f "$RUNTIME" ] || { echo "No AppImage runtime at $RUNTIME (set RUNTIME)" >&2; exit 1; }
command -v mksquashfs >/dev/null || { echo "mksquashfs not found (squashfs-tools)" >&2; exit 1; }
export PATH="$HOME/Qt/Tools/Ninja:$QT_DIR/bin:$PATH"

# 1. Build
if [ ! -f "$BUILD/build.ninja" ] || [ -n "$LABEL" ]; then
    cmake --preset linux-release -DCMAKE_PREFIX_PATH="$QT_DIR" -DCORV_VERSION_LABEL="$LABEL" ${CORV_CMAKE_ARGS:-}
fi
cmake --build --preset linux-release

# 2. AppDir: Qt's own layout under usr/, so the RUNPATHs of its plugins hold
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin" "$APPDIR/usr/lib" "$APPDIR/usr/plugins" "$APPDIR/usr/qml" "$APPDIR/usr/share/doc/corv-gcs"
cp "$BUILD/app/corv-gcs" "$APPDIR/usr/bin/"
# RelWithDebInfo: the symbols stay in the build tree, not in the package
strip --strip-unneeded "$APPDIR/usr/bin/corv-gcs"
cp -r resources/models "$APPDIR/usr/bin/models"
# The bundled LTE cells (CC BY-SA 4.0, see celldb/README.md)
cp -r resources/celldb "$APPDIR/usr/bin/celldb"
cat > "$APPDIR/usr/bin/qt.conf" <<'CONF'
[Paths]
Prefix = ..
Libraries = lib
Plugins = plugins
QmlImports = qml
CONF

for p in platforms/libqxcb.so platforms/libqwayland.so platforms/libqoffscreen.so \
         xcbglintegrations wayland-decoration-client wayland-graphics-integration-client wayland-shell-integration \
         imageformats iconengines sqldrivers/libqsqlite.so tls platforminputcontexts \
         platformthemes/libqxdgdesktopportal.so egldeviceintegrations; do
    [ -e "$QT_DIR/plugins/$p" ] || continue
    mkdir -p "$APPDIR/usr/plugins/$(dirname "$p")"
    cp -r "$QT_DIR/plugins/$p" "$APPDIR/usr/plugins/$(dirname "$p")/"
done

# QML modules the interface imports, each without the modules nested in it
"$QT_DIR/libexec/qmlimportscanner" -rootPath libs/ui/qml -importPath "$QT_DIR/qml" > build/qmlimports.json
python3 -I - "$QT_DIR/qml" "$APPDIR/usr/qml" build/qmlimports.json <<'PY'
import json, os, shutil, sys
src, dst, listing = sys.argv[1], sys.argv[2], sys.argv[3]
mods = [m for m in json.load(open(listing)) if m.get('type') == 'module' and m.get('path', '').startswith(src)]
for m in mods:
    rel = os.path.relpath(m['path'], src)
    for root, dirs, files in os.walk(m['path']):
        # a sub-folder with its own qmldir is another module: copied only if imported
        dirs[:] = [d for d in dirs if not os.path.exists(os.path.join(root, d, 'qmldir'))]
        out = os.path.join(dst, rel, os.path.relpath(root, m['path']))
        os.makedirs(out, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root, f), out)
print(len(mods), 'QML modules')
PY

# Qt libraries: everything from QT_DIR/lib the binaries reach (system libraries stay out)
export LD_LIBRARY_PATH="$QT_DIR/lib"
find "$APPDIR/usr/bin/corv-gcs" "$APPDIR/usr/plugins" "$APPDIR/usr/qml" -type f \( -name 'corv-gcs' -o -name '*.so*' \) -print0 |
    xargs -0 ldd 2>/dev/null | awk -v q="$QT_DIR/lib/" 'index($3, q) == 1 { print $3 }' | sort -u |
    while read -r lib; do cp -L "$lib" "$APPDIR/usr/lib/"; done
unset LD_LIBRARY_PATH

# Launcher, desktop entry, icon
cat > "$APPDIR/AppRun" <<'RUN'
#!/bin/sh
HERE="$(dirname "$(readlink -f "$0")")"
export LD_LIBRARY_PATH="$HERE/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$HERE/usr/bin/corv-gcs" "$@"
RUN
chmod +x "$APPDIR/AppRun"
cp packaging/linux/corv-gcs.desktop "$APPDIR/"
cp resources/icons/icon-256x256.png "$APPDIR/corv-gcs.png"
ln -sf corv-gcs.png "$APPDIR/.DirIcon"
# LGPLv3: the licence texts, and where Qt's sources are
cp LICENSE "$APPDIR/usr/share/doc/corv-gcs/LICENSE"
cat > "$APPDIR/usr/share/doc/corv-gcs/THIRD-PARTY.txt" <<TXT
CORV GCS $VERSION is Apache-2.0 (LICENSE).
It uses Qt $(basename "$(dirname "$QT_DIR")") under the LGPLv3, linked dynamically: the Qt libraries in
usr/lib can be replaced. Qt sources: https://download.qt.io/official_releases/qt/
MAVLink generated headers: MIT.
TXT

# 3. squashfs behind the runtime; the type-2 magic "AI\x02" at offset 8
mkdir -p dist
mksquashfs "$APPDIR" build/corv.squashfs -root-owned -noappend -comp gzip -quiet
python3 -I - "$RUNTIME" build/runtime.bin <<'PY'
import struct, sys
data = open(sys.argv[1], 'rb').read()
assert data[:4] == b'\x7fELF', 'not an ELF runtime'
shoff = struct.unpack_from('<Q', data, 0x28)[0]
shentsize, shnum = struct.unpack_from('<HH', data, 0x3A)
size = shoff + shentsize * shnum
rt = bytearray(data[:size] if len(data) > size else data)
rt[8:11] = b'AI\x02'
open(sys.argv[2], 'wb').write(rt)
print('runtime', len(rt), 'bytes')
PY
cat build/runtime.bin build/corv.squashfs > "$OUT"
chmod +x "$OUT"
rm -f build/corv.squashfs build/runtime.bin
du -h "$OUT"
