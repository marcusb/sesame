# Unpacks Aladdin Connect OTA image files (.uif) into individual partition binaries

import struct
import sys


def err(s):
    print(f"error: {s}", file=sys.stderr)
    sys.exit(-1)


filename = sys.argv[1]

part_info_t = struct.Struct("!8s16slll")
    
with open(filename, "rb") as f:
    buf = f.read()
    if buf[:8] != b"UIFFWIMG":
        err("magic not found")
    n_parts, = struct.unpack("!l", buf[40:44])
    pos = 44
    for i in range(n_parts):
        name, _, crc, ofs, sz = part_info_t.unpack_from(buf, pos)
        name = name.decode("utf-8").rstrip("\x00")
        print(f"{i}: {name:9s} ofs={ofs:6d} len={sz}", file=sys.stderr)
        pos += part_info_t.size

        with open(f"{filename}.{name}.bin", "wb") as p:
            p.write(buf[ofs:ofs + sz])
