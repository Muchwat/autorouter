"""KiCad action plugin: route the open board with the autorouter.

Appears in the PCB editor under Tools > External Plugins > Autoroute, and as a
toolbar button. See the README in this folder.
"""

import os

import pcbnew


class Autoroute(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "Autoroute"
        self.category = "Routing"
        self.description = "Route the nets not yet joined, keeping to the board's own design rules"
        self.show_toolbar_button = True
        self.icon_file_name = os.path.join(os.path.dirname(__file__), "icon.png")

    def Run(self):
        import wx

        from . import dialog

        parent = next((w for w in wx.GetTopLevelWindows() if w.GetTitle().lower().find("pcb editor") >= 0), None)
        dialog.run(parent)


try:
    Autoroute().register()
except Exception:
    pass  # imported outside the PCB editor (a script, a test): nothing to register with
