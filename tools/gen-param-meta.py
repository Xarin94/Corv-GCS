#!/usr/bin/env python3
"""Parameter metadata bundled with the GCS (SETUP -> PARAMETERS: descriptions,
units, ranges, value lists, bitmasks), one gzip file per vehicle in
resources/params/<Firmware>.pdef.json.gz. The CORV fork's HPF_ parameters
are left out.

The files are ArduPilot's apm.pdef.json (Tools/autotest/param_metadata/
param_parse.py --format json) with the whitespace and the source paths
removed. Built from an ArduPilot tree, so the metadata matches the firmware
that tree builds (the CORV fork has its own parameters):

    tools/gen-param-meta.py ~/ardupilot-corv

or from apm.pdef.json files already made (e.g. downloaded from
https://autotest.ardupilot.org/Parameters/<Firmware>/apm.pdef.json):

    tools/gen-param-meta.py --from-json ArduPlane=plane.json ArduCopter=copter.json

A vehicle on a stable release has parameters that the tree no longer knows
(renamed or dropped: SRn_*, ARMING_CHECK, WPNAV_*, SYSID_THISMAV...). Their
documentation comes from the apm.pdef.xml of those releases
(https://autotest.ardupilot.org/Parameters/versioned/<Vehicle>/stable-<x.y.z>/):
the names missing from the table go into a group "stable <x.y.z>", newest
release first. Alone it completes the tables already bundled:

    tools/gen-param-meta.py --stable ArduCopter:4.6.3=Copter-4.6.3.xml ArduPlane:4.6.3=Plane-4.6.3.xml
"""
import argparse
import gzip
import json
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ElementTree

FIRMWARES = ["ArduPlane", "ArduCopter", "Rover", "ArduSub"]
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resources", "params")


# Left out of the bundled metadata: the CORV fork's HPF_ group and the
# AHRS_EKF_TYPE value that selects it
PRIVATE_PREFIXES = ("HPF_",)
PRIVATE_VALUES = {"AHRS_EKF_TYPE": ("20",)}


def slim(pdef):
    """Drop what the GCS does not show (source paths) or does not publish, and
    keep everything else."""
    out = {}
    for group, params in pdef.items():
        if group == "json" or not isinstance(params, dict):
            out[group] = params
            continue
        if group.startswith(PRIVATE_PREFIXES):
            continue
        out[group] = {}
        for name, info in params.items():
            if name.startswith(PRIVATE_PREFIXES):
                continue
            if isinstance(info, dict):
                info = {k: v for k, v in info.items() if k != "path"}
                hidden = PRIVATE_VALUES.get(name, ())
                if hidden and isinstance(info.get("Values"), dict):
                    info["Values"] = {k: v for k, v in info["Values"].items() if k not in hidden}
            out[group][name] = info
    return out


STABLE = "stable "


def read_xml(path):
    """apm.pdef.xml (the format of the versioned releases) as {name: entry of apm.pdef.json}."""
    out = {}
    for p in ElementTree.parse(path).getroot().iter("param"):
        name = p.get("name", "").split(":")[-1]
        if not name or name.startswith(PRIVATE_PREFIXES):
            continue
        info = {"DisplayName": p.get("humanName", ""), "Description": p.get("documentation", "")}
        if p.get("user"):
            info["User"] = p.get("user")
        for f in p.findall("field"):
            key, text = f.get("name"), (f.text or "").strip()
            if key == "Range":
                lohi = text.split()
                if len(lohi) == 2:
                    info["Range"] = {"low": lohi[0], "high": lohi[1]}
            elif key in ("Units", "Increment", "RebootRequired", "ReadOnly"):
                info[key] = text
        for tag, item, key in (("values", "value", "Values"), ("bitmask", "bit", "Bitmask")):
            items = {v.get("code", "").strip(): (v.text or "").strip() for v in p.findall(f"{tag}/{item}")}
            if items:
                info[key] = items
        out[name] = info
    return out


def add_stable(pdef, releases):
    """The parameters only older releases document, each under the newest release that has it."""
    for group in [g for g in pdef if g.startswith(STABLE)]:
        del pdef[group]
    known = {n.split(":")[-1] for g, params in pdef.items() if g != "json" for n in params}
    for version, path in sorted(releases, key=lambda r: [int(x) for x in r[0].split(".")], reverse=True):
        new = {n: i for n, i in read_xml(path).items() if n not in known}
        known.update(new)
        if new:
            pdef[STABLE + version] = new
        print(f"  {STABLE}{version}: {len(new)} more")


def bundled(firmware):
    with gzip.open(os.path.join(OUT_DIR, firmware + ".pdef.json.gz")) as f:
        return json.load(f)


def write(firmware, pdef, source, releases=()):
    pdef = slim(pdef)
    pdef.setdefault("json", {})["source"] = source
    if releases:
        add_stable(pdef, releases)
    raw = json.dumps(pdef, separators=(",", ":"), sort_keys=True).encode()
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, firmware + ".pdef.json.gz")
    # mtime 0: the same metadata gives the same bytes
    with open(path, "wb") as f, gzip.GzipFile(fileobj=f, mode="wb", compresslevel=9, mtime=0, filename="") as gz:
        gz.write(raw)
    count = sum(len(v) for k, v in pdef.items() if k != "json")
    print(f"{firmware}: {count} parameters, {len(raw)} B -> {os.path.getsize(path)} B ({path})")


def describe(tree):
    try:
        sha = subprocess.run(["git", "-C", tree, "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
        return f"{os.path.basename(os.path.normpath(tree))} {sha}"
    except (OSError, subprocess.CalledProcessError):
        return os.path.basename(os.path.normpath(tree))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tree", nargs="?", help="ArduPilot source tree")
    ap.add_argument("--from-json", nargs="+", metavar="FIRMWARE=FILE", help="apm.pdef.json files already made")
    ap.add_argument("--stable", nargs="+", default=[], metavar="FIRMWARE:VERSION=FILE", help="apm.pdef.xml of stable releases")
    args = ap.parse_args()
    stable = {f: [] for f in FIRMWARES}
    for item in args.stable:
        head, _, path = item.partition("=")
        firmware, _, version = head.partition(":")
        if firmware not in FIRMWARES or not version or not path:
            sys.exit(f"expected one of {FIRMWARES}:VERSION=FILE, got {item}")
        stable[firmware].append((version, path))
    if args.stable and not args.tree and not args.from_json:
        for firmware, releases in stable.items():
            pdef = bundled(firmware)
            if releases:
                write(firmware, pdef, pdef.get("json", {}).get("source", "bundled"), releases)
        return
    if args.from_json:
        for item in args.from_json:
            firmware, _, path = item.partition("=")
            if firmware not in FIRMWARES or not path:
                sys.exit(f"expected one of {FIRMWARES}=FILE, got {item}")
            with open(path, encoding="utf-8") as f:
                write(firmware, json.load(f), os.path.basename(path), stable[firmware])
        return
    if not args.tree:
        ap.error("an ArduPilot tree or --from-json")
    script = os.path.join(args.tree, "Tools", "autotest", "param_metadata", "param_parse.py")
    if not os.path.isfile(script):
        sys.exit(f"no {script}")
    source = describe(args.tree)
    for firmware in FIRMWARES:
        with tempfile.TemporaryDirectory() as tmp:
            r = subprocess.run([sys.executable, script, "--vehicle", firmware, "--format", "json"], cwd=tmp, capture_output=True, text=True)
            out = os.path.join(tmp, "apm.pdef.json")
            if r.returncode != 0 or not os.path.isfile(out):
                sys.exit(f"{firmware}: param_parse failed\n{r.stderr[-2000:]}")
            with open(out, encoding="utf-8") as f:
                write(firmware, json.load(f), source, stable[firmware])


if __name__ == "__main__":
    main()
