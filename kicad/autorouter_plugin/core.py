"""The plugin's work, without any window: run the autorouter on a copy of the
open board, then put what it laid on the board itself.

The router never touches the open board or its file. The board as it stands in
the editor (saved or not) is written to a scratch folder with the project's
rules beside it, routed there, and the router's report lists the tracks and
vias it laid and the nets whose old tracks it took up. Those are applied to the
open board, so the whole run is one step that Undo takes back.

Kept free of wx so it can be tried headless, with KiCad's own Python:

    from autorouter_plugin import core
    board = pcbnew.LoadBoard("my.kicad_pcb")
    result = core.route(board, core.Settings())
    core.apply(board, result.report)
"""

import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading

import pcbnew

SETTINGS_FILE = "autorouter.json"


class Settings:
    """What the dialog asks, and where the router program is."""

    def __init__(self, **kw):
        self.program = ""  # empty: look for it
        self.nets = ""  # only these nets (wildcards), comma-separated; empty: all
        self.skip = ""  # never these nets
        self.rip = False  # take up the existing unlocked tracks of the nets routed
        self.grid = 0.25
        self.diff_pairs = True
        self.fanout = True
        self.plane_vias = True
        self.tracks_on_planes = False
        self.include_poured = False
        self.via_spans = ""  # blind and buried vias, as F.Cu-In1.Cu,In2.Cu-B.Cu
        self.extra = ""  # more options, as typed on the command line
        for k, v in kw.items():
            if hasattr(self, k):
                setattr(self, k, v)

    def arguments(self):
        """The router's command-line options for these settings."""
        a = ["--grid", "%g" % float(self.grid)]
        if self.nets.strip():
            a += ["--nets", self.nets.strip()]
        if self.skip.strip():
            a += ["--skip", self.skip.strip()]
        if self.rip:
            a.append("--rip")
        if not self.diff_pairs:
            a.append("--no-diff-pairs")
        if not self.fanout:
            a.append("--no-fanout")
        if not self.plane_vias:
            a.append("--no-plane-vias")
        if self.tracks_on_planes:
            a.append("--tracks-on-planes")
        if self.include_poured:
            a.append("--include-poured")
        if self.via_spans.strip():
            a += ["--via-spans", self.via_spans.strip().replace(" ", "")]
        a += shlex.split(self.extra or "")
        return a

    # remembered between runs, beside KiCad's own settings
    @staticmethod
    def path():
        try:
            folder = pcbnew.SETTINGS_MANAGER.GetUserSettingsPath()
        except Exception:
            folder = os.path.expanduser("~")
        return os.path.join(folder, SETTINGS_FILE)

    @classmethod
    def load(cls):
        try:
            with open(cls.path()) as f:
                return cls(**json.load(f))
        except Exception:
            return cls()

    def save(self):
        try:
            with open(self.path(), "w") as f:
                json.dump(self.__dict__, f, indent=1)
        except Exception:
            pass  # a setting not remembered is no reason to stop


def find_program(settings):
    """The autorouter program: as set, $AUTOROUTER, on PATH, or where cargo
    installs it (KiCad started from the Dock or Start menu has a short PATH)."""
    exe = "autorouter.exe" if sys.platform == "win32" else "autorouter"
    places = [settings.program, os.environ.get("AUTOROUTER", ""), shutil.which("autorouter") or ""]
    places.append(os.path.join(os.path.expanduser("~"), ".cargo", "bin", exe))
    if sys.platform != "win32":
        places += ["/opt/homebrew/bin/autorouter", "/usr/local/bin/autorouter"]
    for p in places:
        if p and os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


class Result:
    def __init__(self, status, output, report, folder):
        self.status = status  # the router's exit status (0: every net joined)
        self.output = output  # what it printed
        self.report = report  # its JSON report, or None
        self.folder = folder  # the scratch folder (removed by apply(), or by clean())

    def clean(self):
        shutil.rmtree(self.folder, ignore_errors=True)


class Cancelled(Exception):
    pass


def route(board, settings, on_line=None, cancelled=None):
    """Route a copy of `board` with `settings`. `on_line(text)` is called with
    each line the router prints; `cancelled()`, if given, is asked now and then
    and stops the router when it says yes (raising Cancelled)."""
    return run(prepare(board, settings), on_line, cancelled)


class Job:
    """A copy of the board in a scratch folder, and the command to route it."""

    def __init__(self, folder, cmd, report):
        self.folder, self.cmd, self.report = folder, cmd, report


def prepare(board, settings):
    """Write the board and its rules to a scratch folder. (Touches the board:
    call it from KiCad's own thread, not from a worker.)"""
    program = find_program(settings)
    if not program:
        raise RuntimeError(
            "the autorouter program was not found. Install it with "
            "'cargo install --path .' in the autorouter folder, or set its path in this dialog"
        )
    folder = tempfile.mkdtemp(prefix="autorouter-")
    source = board.GetFileName()
    stem = os.path.splitext(os.path.basename(source))[0] if source else "board"
    copy = os.path.join(folder, stem + ".kicad_pcb")
    # the board as it is in the editor, unsaved changes and all; the rules from
    # the project's files (true skips writing project settings, which would
    # point the project at the scratch folder)
    if not pcbnew.SaveBoard(copy, board, True):
        shutil.rmtree(folder, ignore_errors=True)
        raise RuntimeError("could not write a copy of the board to " + folder)
    if source:
        for ext in (".kicad_pro", ".kicad_dru"):
            rules = os.path.splitext(source)[0] + ext
            if os.path.isfile(rules):
                shutil.copy(rules, os.path.join(folder, stem + ext))
    report = os.path.join(folder, "report.json")
    cmd = [program, copy, "-o", os.path.join(folder, "routed.kicad_pcb"), "--report", report]
    cmd += settings.arguments()
    return Job(folder, cmd, report)


def run(job, on_line=None, cancelled=None):
    """Run the router on a prepared copy. (Touches no board: safe in a worker.)"""
    folder, cmd, report = job.folder, job.cmd, job.report
    flags = 0x08000000 if sys.platform == "win32" else 0  # CREATE_NO_WINDOW
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        cwd=folder,
        universal_newlines=True,
        creationflags=flags,
    )
    lines = []

    def read():
        for line in proc.stdout:
            lines.append(line.rstrip("\n"))
            if on_line:
                on_line(lines[-1])

    reader = threading.Thread(target=read, daemon=True)
    reader.start()
    while True:
        try:
            proc.wait(timeout=0.1)
            break
        except subprocess.TimeoutExpired:
            if cancelled and cancelled():
                proc.kill()
                proc.wait()
                reader.join(1)
                shutil.rmtree(folder, ignore_errors=True)
                raise Cancelled()
    reader.join(5)
    data = None
    if os.path.isfile(report):
        with open(report) as f:
            data = json.load(f)
    if proc.returncode == 2 or data is None:
        out = "\n".join(lines)
        shutil.rmtree(folder, ignore_errors=True)
        raise RuntimeError("the router stopped: " + (out.strip() or "exit status %d" % proc.returncode))
    return Result(proc.returncode, "\n".join(lines), data, folder)


def _nm(v):
    return pcbnew.FromMM(float(v))


def _point(p):
    return pcbnew.VECTOR2I(_nm(p[0]), _nm(p[1]))


def _via_type(board, top, bottom):
    outer = (board.GetLayerID("F.Cu"), board.GetLayerID("B.Cu"))
    a, b = top in outer, bottom in outer
    if a and b:
        return pcbnew.VIATYPE_THROUGH
    if a or b:
        return pcbnew.VIATYPE_BLIND
    # KiCad 10 has a word for a via between inner layers only; before, one word did for both
    return getattr(pcbnew, "VIATYPE_BURIED", None) or getattr(pcbnew, "VIATYPE_BLIND_BURIED")


def apply(board, report, refill=True):
    """Put the router's result on `board`: take up the tracks it took up, add
    what it laid, and pour the zones again. Returns (removed, tracks, vias)."""
    nets = {}

    def net(name):
        if name not in nets:
            n = board.FindNet(name)
            if n is None:
                raise RuntimeError("the board has no net " + name + " (was it changed while routing?)")
            nets[name] = n
        return nets[name]

    # --rip: every unlocked track, arc and via of those nets, as the router took them up
    ripped = set(report.get("ripped_nets", []))
    removed = 0
    if ripped:
        for item in list(board.GetTracks()):
            if not item.IsLocked() and item.GetNetname() in ripped:
                board.Remove(item)
                # (inside the editor KiCad deletes it with the undo step; outside it,
                # in a script, Python would own it but cannot free it, and doing so
                # corrupts its view of the board: leave it to C++)
                item.thisown = 0
                removed += 1
    tracks = 0
    for t in report.get("new_tracks", []):
        layer = board.GetLayerID(t["layer"])
        if layer < 0:
            raise RuntimeError("the board has no layer " + t["layer"])
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(_point(t["start"]))
        track.SetEnd(_point(t["end"]))
        track.SetWidth(_nm(t["width"]))
        track.SetLayer(layer)
        board.Add(track)
        track.SetNet(net(t["net"]))
        tracks += 1
    vias = 0
    for v in report.get("new_vias", []):
        top, bottom = board.GetLayerID(v["layers"][0]), board.GetLayerID(v["layers"][1])
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(_point(v["at"]))
        via.SetViaType(_via_type(board, top, bottom))
        via.SetLayerPair(top, bottom)
        via.SetWidth(_nm(v["diameter"]))
        via.SetDrill(_nm(v["drill"]))
        board.Add(via)
        via.SetNet(net(v["net"]))
        vias += 1
    if refill and (tracks or vias or removed):
        zones = board.Zones()
        if len(zones):
            pcbnew.ZONE_FILLER(board).Fill(zones)
    return removed, tracks, vias


def summary(result, applied):
    """A few lines on what was done, for the person at the board."""
    r = result.report
    removed, tracks, vias = applied
    lines = []
    lines.append(
        "%d of %d nets joined in %.1f s: %d tracks and %d vias added%s."
        % (
            r["joined"],
            r["nets"],
            r["seconds"],
            tracks,
            vias,
            ", %d old tracks and vias taken up" % removed if removed else "",
        )
    )
    for u in r.get("unrouted", []):
        lines.append("Not joined: %s (%s)" % (u["net"], "; ".join(u["left"])))
    for w in r.get("warnings", []):
        lines.append("Note: " + w)
    for c in r.get("pour_cuts", []):
        lines.append(
            "The %s pour may be cut: %s off the main piece" % (c["net"], ", ".join(c["cut_off"]))
        )
    lines.append("")
    lines.append("Zones were poured again. Run the DRC before you plot; Undo takes the whole run back.")
    return "\n".join(lines)
