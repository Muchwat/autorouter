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
- **Existing copper is kept.** Pads, tracks and vias of other nets are obstacles. A net's own copper already joins what it reaches, counted the way KiCad counts it (a track's end, or a via's or pad's centre, on the other item), so only the missing links are routed. Locked items stay even with `--rip`.
- **Tracks finish on the pad's centre.** The last piece of track into a pad is checked against every clearance exactly, not on the grid, and a pad too small for a grid point is entered from just beside it.
- **Wide tracks narrow to reach small pads.** Where a power track is too wide to get between a pad's neighbours, it stops where it still fits and a short, straight, narrower piece finishes the job (`--neck`, `--neck-length`). Every such pad is listed in the summary.
- **Kept fan-outs are used.** Where a net already has a track with a loose end (a locked fan-out, a half-drawn track), the new track ends on that end and nothing is left dangling.
- **Vias stay off pads.** No via is put on a surface-mount pad of its own net (the open hole drains the solder joint) unless you pass `--via-in-pad`, and none in a through-hole pad or on another via.
- **Wide tracks get two vias.** A track of 0.8 mm or more gets a second via beside each via, joined on both layers, so one small hole is not the weak point of a power path (`--double-vias`). Where there is no room, the via is listed.
- **Differential pairs.** Nets named as a pair the way KiCad reads names (`D+`/`D-`, `CLK_P`/`CLK_N`, `TXP`/`TXN`, with any trailing digits), or as `DP`/`DM`, are routed side by side at a constant gap, with a via each where they change layer and a short piece each into their pads. A pair is done stretch by stretch between its parts: a stretch that cannot be had as a pair (the two nets the other way round at its ends, a part that splits them) is routed one net at a time, and the summary says which and why. Width and gap come from the net class's pair settings, or `--pair W/GAP`; `--no-diff-pairs` turns it off. No length matching yet.
- **It says why.** A piece left unjoined comes with a reason: no clear way in at that width, shut in by other copper with so many square millimetres of room, or no path between other nets' copper.
- **The real board shape.** The outline (lines, arcs, circles, rectangles, polygons, Bezier curves, and edge cuts inside footprints), non-plated holes and milled slots, and keep-out areas are walls. An outline that is not closed is reported, and the board is routed without it.
- **Any number of copper layers,** with through vias. An inner layer that a pour covers is treated as a plane and kept free of tracks, unless you pass `--tracks-on-planes`. Every track starts and ends on its pad's centre.
- **KiCad 7 to 10 files,** both the numbered nets of KiCad 7 to 9 and the named nets of KiCad 10. Only new items are added and replaced ones cut out; the rest of the file stays byte for byte as KiCad wrote it.

## Rules it reads

| Where | What |
|---|---|
| `.kicad_pro` net classes | clearance, track width, via diameter and drill, differential pair width, gap and via gap; assignments and wildcard patterns |
| `.kicad_pro` board rules | minimum clearance, copper-to-edge clearance, hole clearance, hole-to-hole gap |
| `.kicad_dru` constraints | `clearance`, `track_width` (`opt` sets the width, `min` is a floor, `max` a cap), `edge_clearance`, `disallow via` |
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
      --margin MM        least extra gap on every clearance (default 0.05; a coarse grid
                         raises it by what tracks between grid points need)
      --width MM         one track width for every net (default: from the rules)
      --via D/DRILL      one via size for every net, e.g. 0.6/0.3 (default: from the rules)
      --tracks-on-planes also lay tracks on inner layers that a pour covers (by default
                         such a layer is kept as a whole plane)
      --no-vias          route without vias
      --via-in-pad       let vias sit on surface-mount pads of their own net
      --neck MM          width a track narrows to where it is too wide to reach a pad
                         (default: the Default class's width, or the rules' minimum if more)
      --neck-length MM   longest narrowed piece (default 3; 0: never narrow)
      --no-diff-pairs    route nets named as pairs one by one, not as pairs
      --diff-pairs A,B   only these nets as pairs (wildcards; either half's name)
      --pair W/GAP       width of a pair's tracks and the gap between them, e.g. 0.2/0.15
                         (default: the net class's pair settings, else its track width
                         and the clearance between the two nets)
      --double-vias MM   tracks this wide or wider get a second via beside each via
                         (default 0.8; 0: never)
      --report FILE      write what was done as JSON (for scripts and benchmarks)
      --via-cost N       what a via costs, in grid steps (default 12)
      --passes N         most negotiation passes (default 40)
      --drc              refill the zones and check the result with KiCad's DRC (kicad-cli),
                         custom rules included; the refilled board is what gets written
      --keep-fills       leave the zones' old fills in the output (they are removed by
                         default, being stale once tracks change; --drc pours them again)
  -q, --quiet            only the summary
```

The exit status is 0 when every net is joined (and, with `--drc`, KiCad finds no errors), 1 when something is left, and 2 on bad input. `--drc` looks for `kicad-cli` in `KICAD_CLI`, the usual install places and `PATH`, and checks a copy of the output beside the board's own `.kicad_pro` and `.kicad_dru`.

**Copper pours.** A zone's stored fill is only right for the copper that was there when it was poured, so once tracks change, the old fill would lie across the new tracks. The output therefore has the fills taken out: pour them again in KiCad (press B) before you plot. With `--drc`, KiCad pours the zones, checks the board and saves it, so the file you get is the file that was checked. `--keep-fills` leaves the old fills in.

Nets with a copper pour (GND, usually) are left to the pour unless you pass `--include-poured`. New tracks can cut a pour into islands. The router estimates this on a fine grid and warns when pads that the pour joined before routing are cut off after it; with `--drc`, KiCad's own fill decides. Add stitching vias, or route the poured net too.

**Grid.** Walls and conflicts are tested at grid points, so a track between two points can pass a little nearer to other copper than the points do. Every clearance carries a margin for that, worked out from the grid: 0.05 mm at the default 0.25 mm, more on a coarser grid (about 0.45 mm at 1 mm). A coarse grid is safe but packs tracks loosely.

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

Five real boards, each with its tracks ripped up and routed again from nothing, then checked by KiCad 10's own DRC with the board's own rules. `autorouter-bench` produced this table (see [bench/](bench/README.md)); times are the routing alone, on one core of a 2019 Intel MacBook Pro.

| Board | Nets | Joined | Vias | Track | Time | DRC errors | Unconnected | Same on repeat |
|---|---|---|---|---|---|---|---|---|
| Reflow-hotplate controller, 110 x 88 mm, 2 layers, 240 V mains (hand-routed mains kept) | 27 | 27 | 8 | 1198 mm | 1.1 s | 0 | 0 | yes |
| Dispenser controller, 176 x 69 mm, 2 layers, 164 parts, ESP32-S3 and USB-C | 100 | 100 | 141 | 3563 mm | 8.3 s | 0 | 0 | yes |
| The same, with 0.5 to 1.6 mm power and mains widths as rules | 100 | 100 | 161 | 3663 mm | 14.7 s | 0 | 0 | yes |
| Battery-management board, 2 layers, four pours | 42 | 42 | 37 | 779 mm | 1.0 s | 0 | 1 | yes |
| Vehicle tracker, 4 layers with two inner planes, 187 nets | 187 | 187 | 274 | 5286 mm | 49.1 s | 0 | 0 | yes |
| **All five** | **456** | **456** | | | | **0** | **1** | |

The one unconnected item is a pour, not a track: on the battery board the new tracks cut one pad off its pour. The router's own pour check said so before KiCad did. Nets left to a pour are the open problem here (see Limits).

On the dispenser board with real widths, three pads of the 1.2 mm supply are reached by a narrowed piece, and 14 of the 19 vias on wide tracks got a second via; the other five had no room and are listed.

The mains board is the case per-pair rules are for. Routed whole, mains included, it needs 8 mm from mains to low voltage and 2.5 mm between mains nets at once, which one net-class clearance cannot say. That run also comes out with 0 errors and 0 unconnected.

## Limits

- **A grid router.** Tracks run on a grid (0.25 mm by default) in 45-degree steps. There is no push-and-shove, no smoothing of the result, no differential pairs or length tuning.
- **Pad shapes.** Custom pad shapes are taken as their bounding box, and trapezoids as their enclosing rectangle. That is safe but can block more than it needs to.
- **Through vias only.** No blind or buried vias, and no micro vias.
- **Rules on the safe side.** A condition the router can't settle makes the rule apply everywhere. A rule that exempts an area, like a courtyard, is honoured for the pads in that area but not for tracks near them.
- **Fine-pitch parts.** A narrowed piece is one straight line of up to `--neck-length`. A pad that even that cannot reach legally is left unjoined and reported, not forced. Fan such pads out by hand, lock those tracks, and the router carries on from their ends.
- **Plated holes of other nets** are kept clear by their pad's copper, not by a hole rule of their own. Non-plated holes and slots keep the board's hole clearance.
- **Pours on outer layers are not planned for.** Inner planes are left whole, but on a layer it routes on the router does not steer tracks to keep a pour in one piece; it warns after the fact.
- **Speed.** One core. A dense 4-layer board of 190 nets takes a few minutes.
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
