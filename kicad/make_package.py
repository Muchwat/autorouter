"""Build a zip of the plugin for KiCad's Plugin and Content Manager
("Install from File..."):

    python3 kicad/make_package.py        writes kicad/dist/autorouter-kicad-VERSION.zip

The version is the crate's, from Cargo.toml.
"""

import json
import os
import re
import struct
import zipfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PLUGIN = os.path.join(HERE, "autorouter_plugin")


def version():
    with open(os.path.join(REPO, "Cargo.toml")) as f:
        return re.search(r'^version\s*=\s*"([^"]+)"', f.read(), re.M).group(1)


def icon64():
    """The toolbar icon (24 x 24) drawn again at 64 x 64, for the package list."""
    with open(os.path.join(PLUGIN, "icon.png"), "rb") as f:
        data = f.read()
    w, h = struct.unpack(">II", data[16:24])
    # (the icon is written by this repository: 8-bit RGBA, no interlace, one IDAT)
    idat = b"".join(data[i + 8 : i + 8 + struct.unpack(">I", data[i : i + 4])[0]]
                    for i in [m.start() - 4 for m in re.finditer(b"IDAT", data)])
    raw = zlib.decompress(idat)
    rows = [raw[y * (w * 4 + 1) + 1 : (y + 1) * (w * 4 + 1)] for y in range(h)]
    n = 64
    out = b""
    for y in range(n):
        row = rows[y * h // n]
        out += b"\x00" + b"".join(row[(x * w // n) * 4 : (x * w // n) * 4 + 4] for x in range(n))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", n, n, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(out, 9)) + chunk(b"IEND", b""))


def main():
    v = version()
    meta = {
        "$schema": "https://go.kicad.org/pcm/schemas/v1",
        "name": "Autorouter",
        "description": "Route a board's unjoined nets, keeping to its own design rules",
        "description_full": (
            "Runs the autorouter (a grid router written in Rust) on the open board and puts "
            "the tracks and vias it lays on the board as one undoable step. It keeps to the "
            "board's net classes and custom rules, net pair by net pair, and handles "
            "differential pairs, length matching, per-layer widths, blind and buried vias, "
            "fan-out for BGAs and fine-pitch parts, and joining pads to inner planes.\n\n"
            "Needs the autorouter program itself: cargo install --path . in its repository."
        ),
        "identifier": "com.github.muchwat.autorouter",
        "type": "plugin",
        "author": {"name": "Muchwat", "contact": {"github": "https://github.com/Muchwat"}},
        "license": "MIT",
        "resources": {"homepage": "https://github.com/Muchwat/autorouter"},
        "versions": [{"version": v, "status": "testing", "kicad_version": "10.0"}],
    }
    dist = os.path.join(HERE, "dist")
    os.makedirs(dist, exist_ok=True)
    path = os.path.join(dist, "autorouter-kicad-%s.zip" % v)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("metadata.json", json.dumps(meta, indent=2) + "\n")
        for name in sorted(os.listdir(PLUGIN)):
            if name.endswith((".py", ".png")):
                z.write(os.path.join(PLUGIN, name), "plugins/" + name)
        z.writestr("resources/icon.png", icon64())
    print(path)


if __name__ == "__main__":
    main()
