"""The plugin's windows: the options, the run, the result."""

import threading

import pcbnew
import wx

from . import core


class OptionsDialog(wx.Dialog):
    def __init__(self, parent, settings, board):
        super().__init__(parent, title="Antnest", style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.settings = settings
        s = settings
        pad = 6
        box = wx.BoxSizer(wx.VERTICAL)
        grid = wx.FlexGridSizer(cols=2, vgap=pad, hgap=pad * 2)
        grid.AddGrowableCol(1)

        def text(label, value, tip):
            grid.Add(wx.StaticText(self, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            t = wx.TextCtrl(self, value=value)
            t.SetToolTip(tip)
            grid.Add(t, 1, wx.EXPAND)
            return t

        self.nets = text("Only these nets", s.nets, "Comma-separated, wildcards allowed (/SPI_*). Empty: every net not yet joined.")
        self.skip = text("Never these nets", s.skip, "Comma-separated, wildcards allowed.")
        self.grid = text("Grid (mm)", "%g" % s.grid, "0.25 by default; 0.254 suits 0.1 inch through-hole parts; finer is slower.")
        layers = ", ".join(board.GetLayerName(l) for l in board.GetEnabledLayers().CuStack())
        self.spans = text(
            "Blind and buried vias",
            s.via_spans,
            "The vias your board house makes, each by the two layers it joins: F.Cu-In1.Cu,In2.Cu-B.Cu. "
            "Empty: through vias only. This board's copper layers: " + layers,
        )
        self.extra = text("More options", s.extra, "Any other option of the antnest command, as typed on its command line.")
        box.Add(grid, 0, wx.EXPAND | wx.ALL, pad * 2)

        def check(label, value, tip):
            c = wx.CheckBox(self, label=label)
            c.SetValue(value)
            c.SetToolTip(tip)
            box.Add(c, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, pad * 2)
            return c

        self.rip = check("Take up the existing tracks of these nets first", s.rip, "Locked tracks and vias stay.")
        self.pairs = check("Route differential pairs side by side", s.diff_pairs, "Nets named D+/D-, _P/_N, DP/DM.")
        self.fanout = check("Fan out BGAs and fine-pitch parts", s.fanout, "An escape track or a dog-bone via for pads the grid cannot reach.")
        self.planes = check("Join pads to their inner plane with vias", s.plane_vias, "For nets with a plane on an inner layer, such as GND.")
        self.on_planes = check("Lay tracks on inner plane layers too", s.tracks_on_planes, "By default such a layer is kept as a whole plane.")
        self.poured = check("Also route nets that have a copper pour", s.include_poured, "By default they are left to the pour.")

        prog = wx.BoxSizer(wx.HORIZONTAL)
        prog.Add(wx.StaticText(self, label="Router program"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, pad * 2)
        found = core.find_program(s) or ""
        self.program = wx.FilePickerCtrl(self, path=s.program or found, message="The antnest program")
        self.program.SetToolTip("Found by itself on PATH or in ~/.cargo/bin; set it here if it is elsewhere.")
        prog.Add(self.program, 1, wx.EXPAND)
        box.Add(prog, 0, wx.EXPAND | wx.ALL, pad * 2)

        buttons = self.CreateStdDialogButtonSizer(wx.OK | wx.CANCEL)
        self.FindWindowById(wx.ID_OK).SetLabel("Route")
        box.Add(buttons, 0, wx.EXPAND | wx.ALL, pad * 2)
        self.SetSizerAndFit(box)
        self.SetMinSize((560, -1))
        self.Fit()
        self.CentreOnParent()

    def read(self):
        """The settings as filled in."""
        s = self.settings
        s.nets, s.skip = self.nets.GetValue(), self.skip.GetValue()
        try:
            s.grid = float(self.grid.GetValue())
        except ValueError:
            pass
        s.via_spans, s.extra = self.spans.GetValue(), self.extra.GetValue()
        s.rip, s.diff_pairs, s.fanout = self.rip.GetValue(), self.pairs.GetValue(), self.fanout.GetValue()
        s.plane_vias, s.tracks_on_planes = self.planes.GetValue(), self.on_planes.GetValue()
        s.include_poured = self.poured.GetValue()
        path = self.program.GetPath()
        s.program = "" if path == (core.find_program(core.Settings()) or "") else path
        return s


def run(parent):
    board = pcbnew.GetBoard()
    settings = core.Settings.load()
    dlg = OptionsDialog(parent, settings, board)
    if dlg.ShowModal() != wx.ID_OK:
        dlg.Destroy()
        return
    settings = dlg.read()
    dlg.Destroy()
    settings.save()

    # the router runs in its own process; this window shows its last line meanwhile
    progress = wx.ProgressDialog(
        "Antnest",
        "Starting the router...",
        parent=parent,
        style=wx.PD_APP_MODAL | wx.PD_CAN_ABORT | wx.PD_ELAPSED_TIME,
    )
    last = ["Starting the router..."]
    stop = threading.Event()
    done = {}
    try:
        job = core.prepare(board, settings)
    except Exception as e:
        progress.Destroy()
        wx.MessageBox(str(e), "Antnest", wx.OK | wx.ICON_ERROR, parent)
        return

    def work():
        try:
            done["result"] = core.run(job, on_line=lambda l: last.__setitem__(0, l), cancelled=stop.is_set)
        except Exception as e:  # (Cancelled too)
            done["error"] = e

    worker = threading.Thread(target=work, daemon=True)
    worker.start()
    while worker.is_alive():
        keep, _ = progress.Pulse(last[0][:100])
        if not keep:
            stop.set()
        wx.MilliSleep(100)
        wx.Yield()
    progress.Destroy()

    if isinstance(done.get("error"), core.Cancelled):
        return
    if "error" in done:
        wx.MessageBox(str(done["error"]), "Antnest", wx.OK | wx.ICON_ERROR, parent)
        return
    result = done["result"]
    try:
        applied = core.apply(board, result.report)
    except Exception as e:
        wx.MessageBox("The route could not be put on the board: %s" % e, "Antnest", wx.OK | wx.ICON_ERROR, parent)
        return
    finally:
        result.clean()
    pcbnew.Refresh()
    text = core.summary(result, applied)
    wx.MessageBox(
        text,
        "Antnest",
        wx.OK | (wx.ICON_INFORMATION if result.status == 0 else wx.ICON_WARNING),
        parent,
    )
