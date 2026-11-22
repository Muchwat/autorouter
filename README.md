# autorouter

A grid autorouter for KiCad boards, written in Rust. It reads a `.kicad_pcb`, routes the nets that are not yet joined, and writes the board back with the new tracks and vias. It keeps to the board's own design rules: the net classes in the `.kicad_pro` and the custom rules in the `.kicad_dru`, net pair by net pair. That is what lets it route a mains board, where mains must stay 8 mm from low voltage while two signals sit 0.2 mm apart.

```bash
cargo install --git https://github.com/Muchwat/autorouter
autorouter my-board.kicad_pcb --drc
```

It has no dependencies beyond the Rust standard library, and it runs without KiCad. KiCad's `kicad-cli` is used only for `--drc`, to check the result.

## What it does

- **Every net at once.** Nets are routed together and bargain for space (negotiated congestion, the PathFinder scheme of McMurchie and Ebeling, 1995). A spot two nets both want gets dearer each pass, and only the nets still in conflict are routed again. So the order nets come in does not decide whether a board finishes.
- **Always legal.** If the passes run out with nets still in conflict, those nets are routed once more with everyone else's copper as walls. A net may be left unjoined, but tracks never overlap or break a clearance the router knows about.
- **KiCad's rules, per net pair.** Clearance between two items is worked out the way KiCad's DRC does it: the larger net-class clearance, overridden by the last custom rule that matches, never below the board minimum.
- **Existing copper is kept.** Pads, tracks and vias of other nets are obstacles. A net's own copper already joins what it touches, so only the missing links are routed. Locked items stay even with `--rip`.
- **The real board shape.** The outline (lines, arcs, circles, rectangles, polygons, and edge cuts inside footprints) and keep-out areas are walls, so slots and odd shapes work.
- **Any number of copper layers,** with through vias. Every track starts and ends on its pad's centre.
- **KiCad 7 to 10 files,** both the numbered nets of KiCad 7 to 9 and the named nets of KiCad 10. Only new items are added and replaced ones cut out; the rest of the file stays byte for byte as KiCad wrote it.

## Rules it reads

| Where | What |
|---|---|
| `.kicad_pro` net classes | clearance, track width, via diameter and drill; assignments and wildcard patterns |
| `.kicad_pro` board rules | minimum clearance, copper-to-edge clearance |
| `.kicad_dru` constraints | `clearance`, `track_width` (opt, else min), `edge_clearance`, `disallow via` |
| `.kicad_dru` conditions | `A/B.NetName`, `A/B.NetClass`, `A/B.Type`, `==` and `!=` with wildcards, `&&`, `\|\|`, `!`, `hasNetclass()`, `intersectsCourtyard()` and `insideCourtyard()` for pads |

Conditions are evaluated with three values: true, false and "can't tell". Some tests can't be answered while routing, such as whether a track will cross a courtyard, or any function the router does not know. A rule whose condition might hold is applied, so the router errs on the safe side, and it prints a note naming the rule. Layer-limited rules are applied on every layer.

## Usage

```
autorouter BOARD.kicad_pcb [options]

  -o, --output FILE      where to write the routed board (default BOARD-routed.kicad_pcb)
      --nets A,B         route only these nets (wildcards: '/SPI_*')
      --skip A,B         never route these nets
      --include-poured   also route nets that have a copper pour (left to the pour by default)
      --rip              remove the existing tracks and vias of the nets being routed first
      --layers F.Cu,B.Cu copper layers to use (default: all)
      --grid MM          grid pitch (default 0.25; 0.254 suits 0.1 inch through-hole parts)
      --margin MM        extra gap on every clearance for the grid's corners (default 0.05)
      --width MM         one track width for every net (default: from the rules)
      --via D/DRILL      one via size for every net, e.g. 0.6/0.3 (default: from the rules)
      --no-vias          route without vias
      --via-cost N       what a via costs, in grid steps (default 12)
      --passes N         most negotiation passes (default 40)
      --drc              check the result with KiCad's DRC (kicad-cli), custom rules included
  -q, --quiet            only the summary
```

The exit status is 0 when every net is joined (and, with `--drc`, KiCad finds no errors), 1 when something is left, and 2 on bad input. `--drc` looks for `kicad-cli` in `KICAD_CLI`, the usual install places and `PATH`, and checks a copy of the output beside the board's own `.kicad_pro` and `.kicad_dru`.

Nets with a copper pour (GND, usually) are left to the pour unless you pass `--include-poured`. New tracks can cut a pour into islands. With `--drc`, KiCad reports any island that nothing joins; add stitching vias, or route the poured net too.

### As a library

```rust
use autorouter::{board::Board, rules::Rules, router::{route, Options}};

let board = Board::parse(std::fs::read_to_string("my.kicad_pcb")?)?;
let rules = Rules::for_board("my.kicad_pcb", &board)?;
let result = route(&board, &rules, &Options::default());
std::fs::write("my-routed.kicad_pcb", autorouter::write::with_routes(&board, &result, &[]))?;
```

`cargo run --example pads -- board.kicad_pcb` prints every pad as the router reads it, which helps when something looks wrong.

## How well it works

These are measured on two real boards, checked by KiCad 10's own DRC with each board's custom rules. Times are the routing alone, on a 2019 Intel MacBook Pro.

| Board | Job | Result | Time |
|---|---|---|---|
| A reflow-hotplate controller, 110 x 88 mm, 2 layers, 240 V mains | all 27 low-voltage nets, hand-routed mains kept | 27 of 27 joined, 0 errors, 0 unconnected | 3 s |
| The same board | the whole board from nothing, mains included (8 mm mains to low voltage, 2.5 mm between mains nets, 2 mm heater tracks, no vias on mains) | 34 of 34 joined, 0 errors, 0 unconnected | 4 to 6 s |
| A dispenser controller, 176 x 69 mm, 2 layers, 164 parts, fine-pitch ESP32-S3 and USB-C | all 101 nets including ground | 101 of 101 joined, 0 errors, 0 unconnected | 15 s |

The mains board is the case per-pair rules are for. A net class has one clearance, and it can't be 8 mm to low voltage and 2.5 mm to the other mains nets at the same time.

## Limits

- **A grid router.** Tracks run on a grid (0.25 mm by default) in 45-degree steps. There is no push-and-shove, no smoothing of the result, no differential pairs or length tuning.
- **Pad shapes.** Custom pad shapes are taken as their bounding box, and trapezoids as their enclosing rectangle. That is safe but can block more than it needs to.
- **Through vias only.** No blind or buried vias, and no micro vias.
- **Rules on the safe side.** A condition the router can't settle makes the rule apply everywhere. A rule that exempts an area, like a courtyard, is honoured for the pads in that area but not for tracks near them.
- **Check the result.** Run KiCad's DRC (or `--drc`) before you order a board. The router's own checks are on its grid; KiCad has the final word.

## How it works

1. **Read** the board into pads, tracks, vias, zones and outline (`board.rs`), and the rules (`rules.rs`).
2. **Group** each net's pads and the copper already joining them into pieces; a net with two or more pieces is a job.
3. **Walls** for each job: the outline and keep-outs, and every other net's pads, holes and existing copper, each grown by the clearance that pair of nets needs plus half the job's track width.
4. **Route** each job as a tree, piece by piece, with A* on the grid. Other jobs' new copper costs more where it is crowded, and more again where it has been fought over before.
5. **Negotiate.** Find the jobs whose copper sits inside another's clearance, raise the costs, and route only those again, until none are left or progress stops.
6. **Settle** anything still in conflict with the others as walls, then write the tracks and vias.

Nets that every rule treats alike (same net class and same answer to every net-name test in the rules) share one occupancy map, so a board with many nets stays fast while each pair of nets still gets its own clearance.

## Credits

Inspired by imsaiguy's [Visible Autorouter for KiCad](https://github.com/imsaiguy/autorouter), which showed how far negotiated routing goes on hobby boards. This is a separate implementation in Rust, written around KiCad's per-pair design rules.

## Licence

Licensed under either of the [Apache License, Version 2.0](LICENSE-APACHE) or the [MIT licence](LICENSE-MIT), at your option. Contributions are taken under the same terms unless you say otherwise.
