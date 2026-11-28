"""Try the plugin's work on a board, outside the editor, and check the result
with KiCad's DRC. Run it with KiCad's own Python (it needs pcbnew):

    macOS:   /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \\
                 kicad/test_plugin.py BOARD.kicad_pcb [--rip] [--option value ...]
    Linux:   python3 kicad/test_plugin.py ...       (the system Python KiCad uses)
    Windows: "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" kicad\\test_plugin.py ...

The board is loaded, routed and saved as BOARD-plugin.kicad_pcb beside it (the
board itself is not touched); then kicad-cli checks the copy. Any setting of
the dialog can be given as --name value (--via_spans F.Cu-In1.Cu, --grid 0.1).
"""

import os
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pcbnew  # noqa: E402

from antnest_plugin import core  # noqa: E402


def kicad_cli():
    for p in (
        os.environ.get("KICAD_CLI", ""),
        shutil.which("kicad-cli") or "",
        "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
        r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe",
    ):
        if p and os.path.isfile(p):
            return p
    return None


def main():
    args = sys.argv[1:]
    if not args or args[0].startswith("-"):
        sys.exit(__doc__)
    path, rest = args[0], args[1:]
    kw = {}
    i = 0
    while i < len(rest):
        name = rest[i].lstrip("-").replace("-", "_")
        if i + 1 < len(rest) and not rest[i + 1].startswith("--"):
            kw[name], i = rest[i + 1], i + 2
        else:
            kw[name], i = True, i + 1
    settings = core.Settings(**{k: v for k, v in kw.items() if hasattr(core.Settings(), k)})
    board = pcbnew.LoadBoard(path)
    result = core.route(board, settings, on_line=print)
    try:
        applied = core.apply(board, result.report)
    finally:
        result.clean()
    print()
    print(core.summary(result, applied))
    stem = os.path.splitext(path)[0]
    out = stem + "-plugin.kicad_pcb"
    pcbnew.SaveBoard(out, board, True)
    for ext in (".kicad_pro", ".kicad_dru"):
        if os.path.isfile(stem + ext):
            shutil.copy(stem + ext, os.path.splitext(out)[0] + ext)
    cli = kicad_cli()
    if not cli:
        print("saved %s (kicad-cli not found: no DRC)" % out)
        return
    rpt = os.path.splitext(out)[0] + "-drc.rpt"
    subprocess.run([cli, "pcb", "drc", "--severity-error", "--units", "mm", "-o", rpt, out],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(rpt) as f:
        found = [l.strip() for l in f if l.startswith("** Found")]
    print("saved %s; KiCad DRC: %s" % (out, "; ".join(found)))


if __name__ == "__main__":
    main()
