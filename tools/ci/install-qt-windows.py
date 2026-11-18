"""Install Qt for Windows MSVC from the official online repository, for CI.

Qt 6.12 moved its Windows packages to one repository per toolchain
(online/qtsdkrepository/windows_x86/desktop/qt6_6120/qt6_6120_msvc2022_64/),
a layout aqtinstall 3.3 does not know. This reads that repository's
Updates.xml, downloads the archives of qtbase / svg / declarative and of the
requested add-on modules, checks every archive against the .sha1 published
next to it (the download stops on a mismatch) and extracts them with 7-Zip.

Usage: install-qt-windows.py VERSION ARCH OUTDIR [MODULE ...]
  e.g. install-qt-windows.py 6.12.0 win64_msvc2022_64 C:/Qt qtserialport qtshadertools
Prints the Qt prefix (OUTDIR/VERSION/msvc2022_64) on the last line.
"""
import hashlib
import os
import subprocess
import sys
import tempfile
import urllib.request
import xml.etree.ElementTree as ET

BASE = 'https://download.qt.io/online/qtsdkrepository/windows_x86/desktop'


def fetch(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'corv-gcs-ci'})
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def main():
    version, arch, outdir, modules = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
    vnum = version.replace('.', '')                      # 6.12.0 -> 6120
    toolchain = arch.replace('win64_', '')               # msvc2022_64
    repo = f'{BASE}/qt6_{vnum}/qt6_{vnum}_{toolchain}'
    root = ET.fromstring(fetch(f'{repo}/Updates.xml'))
    packages = {p.findtext('Name'): p for p in root.findall('PackageUpdate')}

    wanted = [f'qt.qt6.{vnum}.{arch}'] + [f'qt.qt6.{vnum}.addons.{m}.{arch}' for m in modules]
    prefix = os.path.join(outdir, version, toolchain)
    os.makedirs(prefix, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        for name in wanted:
            p = packages.get(name)
            if p is None:
                sys.exit(f'package {name} not in {repo}/Updates.xml')
            pkg_version = p.findtext('Version')
            archives = [a.strip() for a in (p.findtext('DownloadableArchives') or '').split(',') if a.strip()]
            for a in archives:
                if a.startswith('qtdoc'):
                    continue                              # documentation: not needed to build
                url = f'{repo}/{name}/{pkg_version}{a}'
                data = fetch(url)
                expected = fetch(url + '.sha1').decode().split()[0].strip().lower()
                got = hashlib.sha1(data).hexdigest()
                if got != expected:
                    sys.exit(f'checksum mismatch for {a}: {got} != {expected}')
                path = os.path.join(tmp, a)
                with open(path, 'wb') as f:
                    f.write(data)
                print(f'{name}: {a} ({len(data) // 1024} KiB, sha1 ok)', flush=True)
                subprocess.run(['7z', 'x', '-y', f'-o{prefix}', path], check=True, stdout=subprocess.DEVNULL)
    # The archives carry the build machine's prefix in qt.conf-less tools; a
    # qt.conf next to the binaries makes qmake / qtpaths report this one.
    with open(os.path.join(prefix, 'bin', 'qt.conf'), 'w') as f:
        f.write('[Paths]\nPrefix = ..\n')
    print(prefix)


if __name__ == '__main__':
    main()
