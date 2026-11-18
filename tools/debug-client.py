#!/usr/bin/env python3
"""Send one request to the debug server of a running CORV GCS (--debug-port PORT).

    python tools/debug-client.py 47800 stats
    python tools/debug-client.py 47800 set '{"object":"scene","property":"orbitDistance","value":300}'
    python tools/debug-client.py 47800 call '{"target":"Gcs.setMode","args":["FBWA"]}'
    python tools/debug-client.py 47800 screenshot '{"file":"C:/tmp/view.png"}'
    python tools/debug-client.py 47800 script '{"line":"emit toggleSatellite"}'

Prints the JSON response; exits 1 on an error response. Loopback only, as the server.
"""
import json
import socket
import sys


def request(port, method, params=None, timeout=30.0):
    msg = {"jsonrpc": "2.0", "id": 1, "method": method}
    if params is not None:
        msg["params"] = params
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
        s.sendall((json.dumps(msg) + "\n").encode())
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
    return json.loads(buf.decode())


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    port, method = int(sys.argv[1]), sys.argv[2]
    params = json.loads(sys.argv[3]) if len(sys.argv) > 3 else None
    resp = request(port, method, params)
    result = resp.get("result")
    if isinstance(result, dict) and "png" in result:
        result = dict(result, png="<%d base64 chars>" % len(result["png"]))
        resp["result"] = result
    print(json.dumps(resp, indent=2))
    return 1 if "error" in resp else 0


if __name__ == "__main__":
    sys.exit(main())
