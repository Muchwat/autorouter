"""Build a zip of the plugin for KiCad's Plugin and Content Manager
("Install from File..."):

    python3 kicad/make_package.py        writes kicad/dist/autorouter-kicad-VERSION.zip

The version is the crate's, from Cargo.toml.
"""

import json
import os
import re
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PLUGIN = os.path.join(HERE, "autorouter_plugin")


def version():
    with open(os.path.join(REPO, "Cargo.toml")) as f:
        return re.search(r'^version\s*=\s*"([^"]+)"', f.read(), re.M).group(1)


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
            if name.endswith(".py") or name == "icon.png":
                z.write(os.path.join(PLUGIN, name), "plugins/" + name)
        # (the package list's icon: the same A, drawn at 64 px by make_icon.py)
        z.write(os.path.join(PLUGIN, "icon-64.png"), "resources/icon.png")
    print(path)


if __name__ == "__main__":
    main()
