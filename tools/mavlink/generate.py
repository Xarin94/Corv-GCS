"""Generate the MAVLink C headers in third_party/mavlink from tools/mavlink/corv.xml.

    python -m pip install pymavlink
    python tools/mavlink/generate.py

The output is committed: the build needs no Python. Run again after changing corv.xml
or updating pymavlink, and check that CORV_CELL_INFO keeps CRC_EXTRA 119.
"""
import os
import re
import shutil
import sys

from pymavlink.generator import mavgen

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
OUT = os.path.join(ROOT, 'third_party', 'mavlink')
DEFS = os.path.join(os.path.dirname(mavgen.__file__), '..', 'message_definitions', 'v1.0')

work = os.path.join(OUT, '_defs')
shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(work)
for name in os.listdir(DEFS):
    if name.endswith('.xml'):
        shutil.copy(os.path.join(DEFS, name), work)
shutil.copy(os.path.join(HERE, 'corv.xml'), work)

opts = mavgen.Opts(OUT, wire_protocol='2.0', language='C', validate=True, strict_units=False)
if not mavgen.mavgen(opts, [os.path.join(work, 'corv.xml')]):
    sys.exit('mavgen failed')
shutil.rmtree(work)

hdr = open(os.path.join(OUT, 'corv', 'mavlink_msg_corv_cell_info.h'), encoding='utf8').read()
crc = int(re.search(r'MAVLINK_MSG_ID_CORV_CELL_INFO_CRC (\d+)', hdr).group(1))
length = int(re.search(r'MAVLINK_MSG_ID_CORV_CELL_INFO_LEN (\d+)', hdr).group(1))
print(f'CORV_CELL_INFO: {length} bytes, CRC_EXTRA {crc}')
if crc != 119 or length != 63:
    sys.exit('CORV_CELL_INFO does not match the LTEtelem definition (63 bytes, CRC_EXTRA 119)')
