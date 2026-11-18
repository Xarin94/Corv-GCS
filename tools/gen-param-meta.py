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
"""
import argparse
import gzip
import json
import os
import subprocess
import sys
import tempfile

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


def write(firmware, pdef, source):
    pdef = slim(pdef)
    pdef.setdefault("json", {})["source"] = source
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
    args = ap.parse_args()
    if args.from_json:
        for item in args.from_json:
            firmware, _, path = item.partition("=")
            if firmware not in FIRMWARES or not path:
                sys.exit(f"expected one of {FIRMWARES}=FILE, got {item}")
            with open(path, encoding="utf-8") as f:
                write(firmware, json.load(f), os.path.basename(path))
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
                write(firmware, json.load(f), source)


if __name__ == "__main__":
    main()
