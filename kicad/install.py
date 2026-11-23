"""Install the Autoroute plugin into KiCad's plugin folder.

    python3 kicad/install.py            copy it into the newest KiCad found
    python3 kicad/install.py --link     link it instead (changes here show in KiCad)
    python3 kicad/install.py --kicad 9.0
    python3 kicad/install.py --remove

Then restart the PCB editor, or use Tools > External Plugins > Refresh Plugins.
The plugin runs the autorouter program, which it looks for on PATH and in
~/.cargo/bin; install that with `cargo install --path .` in this repository.
"""

import argparse
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NAME = "autorouter_plugin"


def kicad_roots():
    """Where KiCad keeps its per-version user folders, on this system."""
    home = os.path.expanduser("~")
    if sys.platform.startswith("linux"):
        data = os.environ.get("XDG_DATA_HOME", os.path.join(home, ".local", "share"))
        return [os.path.join(data, "kicad")]
    return [os.path.join(home, "Documents", "KiCad")]


def versions():
    """(version, plugin folder) for each KiCad version that has a user folder, newest first."""
    found = []
    for root in kicad_roots():
        if not os.path.isdir(root):
            continue
        for v in os.listdir(root):
            try:
                key = tuple(int(x) for x in v.split("."))
            except ValueError:
                continue
            found.append((key, v, os.path.join(root, v, "scripting", "plugins")))
    found.sort(reverse=True)
    return [(v, p) for _, v, p in found]


def main():
    ap = argparse.ArgumentParser(description="Install the Autoroute plugin into KiCad.")
    ap.add_argument("--kicad", help="the KiCad version to install for, such as 10.0 (default: the newest found)")
    ap.add_argument("--link", action="store_true", help="link to this folder instead of copying")
    ap.add_argument("--remove", action="store_true", help="take the plugin out again")
    a = ap.parse_args()
    all_ = versions()
    if a.kicad:
        all_ = [x for x in all_ if x[0] == a.kicad]
    if not all_:
        sys.exit("no KiCad user folder found%s; start KiCad once, or give --kicad" % (
            " for " + a.kicad if a.kicad else ""))
    version, folder = all_[0]
    target = os.path.join(folder, NAME)
    if os.path.islink(target) or os.path.isfile(target):
        os.remove(target)
    elif os.path.isdir(target):
        shutil.rmtree(target)
    if a.remove:
        print("removed from KiCad %s (%s)" % (version, target))
        return
    os.makedirs(folder, exist_ok=True)
    source = os.path.join(HERE, NAME)
    if a.link:
        os.symlink(source, target, target_is_directory=True)
    else:
        shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    print("installed for KiCad %s: %s" % (version, target))
    exe = "autorouter.exe" if sys.platform == "win32" else "autorouter"
    cargo = os.path.join(os.path.expanduser("~"), ".cargo", "bin", exe)
    if not (shutil.which("autorouter") or os.path.isfile(cargo)):
        print("note: the autorouter program is not on PATH or in ~/.cargo/bin yet. Install it with\n"
              "  cargo install --path \"%s\"\nor set its path in the plugin's dialog." % os.path.dirname(HERE))
    print("Restart the PCB editor (or Tools > External Plugins > Refresh Plugins); then Tools > External Plugins > Autoroute.")


if __name__ == "__main__":
    main()
