# Autoroute plugin for KiCad

Runs the [autorouter](../README.md) on the board open in KiCad's PCB editor and puts the tracks and vias it lays on that board, as one step that Undo takes back.

## Install

1. The router itself, from this repository or straight from GitHub:

   ```bash
   cargo install --git https://github.com/Muchwat/autorouter
   ```

   It goes into `~/.cargo/bin`, where the plugin looks for it. If it is somewhere else, set its path in the plugin's dialog, or set `AUTOROUTER` to it.

2. The plugin, into the newest KiCad found on this computer:

   ```bash
   python3 kicad/install.py
   ```

   `--kicad 9.0` picks another version, `--link` links to this folder instead of copying (for working on the plugin), and `--remove` takes it out. You can also copy `kicad/autorouter_plugin` by hand into KiCad's `scripting/plugins` folder (Preferences > Configure Paths shows where that is).

   Or build a package for KiCad's Plugin and Content Manager and use its **Install from File...**:

   ```bash
   python3 kicad/make_package.py
   ```

3. Restart the PCB editor, or use **Tools > External Plugins > Refresh Plugins**.

## Use

**Tools > External Plugins > Autoroute**, or the toolbar button. The dialog asks:

| Field | Router option |
|---|---|
| Only these nets, Never these nets | `--nets`, `--skip` (comma-separated, wildcards such as `/SPI_*`) |
| Grid | `--grid` |
| Blind and buried vias | `--via-spans`, such as `F.Cu-In1.Cu,In2.Cu-B.Cu`; empty for through vias only |
| More options | anything else, as typed on the command line, such as `--skew 0.1` |
| Take up the existing tracks of these nets first | `--rip` (locked tracks and vias stay) |
| Route differential pairs side by side | off: `--no-diff-pairs` |
| Fan out BGAs and fine-pitch parts | off: `--no-fanout` |
| Join pads to their inner plane with vias | off: `--no-plane-vias` |
| Lay tracks on inner plane layers too | `--tracks-on-planes` |
| Also route nets that have a copper pour | `--include-poured` |

The settings are remembered, in `autorouter.json` beside KiCad's own settings.

What happens when you press Route:

1. The board as it is in the editor, unsaved changes and all, is written to a scratch folder with the project's `.kicad_pro` and `.kicad_dru` beside it. The board's own file is not touched.
2. The router routes that copy. Its last line shows while it works; Cancel stops it and leaves the board as it was.
3. The tracks and vias it laid are added to the open board (and with "take up" ticked, the unlocked tracks and vias of the nets it routed are taken out first, as the router did). The zones are poured again.
4. A summary says what was joined, what was not and why, and any pour the new tracks may have cut. Run the DRC before you plot. Ctrl+Z takes the whole run back.

## Try it from a script

`core.py` does the work and needs no window, so the plugin can be tried on a board without the editor, with KiCad's own Python:

```bash
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 kicad/test_plugin.py my-board.kicad_pcb --rip
```

It routes the board, saves the result as `my-board-plugin.kicad_pcb` beside it, and checks that with `kicad-cli`. Any setting of the dialog can be given as `--name value` (`--via_spans F.Cu-In1.Cu`, `--grid 0.1`).

## Limits

- Written and tried with KiCad 10.0. It uses KiCad's Python scripting interface (SWIG), which KiCad has said it will replace with its IPC interface.
- The rules come from the project's files on disk: save net class and rule changes before routing.
- The editor waits while the router runs.
