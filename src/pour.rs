//! An estimate of whether new tracks cut a copper pour into islands.
//!
//! A net left to its pour is joined by copper the router never lays, and a new
//! track of another net can cut that copper in two. This works out, on a grid,
//! where each pour can reach once other nets' copper is kept clear of it, and
//! says which pads of the pour's net end up on a piece apart from the rest.
//!
//! It is an estimate: KiCad's own fill has the final word (`--drc`). To keep
//! false alarms down, only pads that were joined before routing and are not
//! after it are reported.

use crate::board::Board;
use crate::geom::{self, Shape, P};
use crate::router::{NewVia, Routed};
use crate::rules::{Item, Rules};

#[derive(Clone, Debug)]
pub struct Cut {
    pub net: String,
    /// Pads of the net in all.
    pub pads: usize,
    /// Pads the new copper cut off from the largest piece.
    pub cut_off: Vec<String>,
}

struct Grid {
    x0: f64,
    y0: f64,
    g: f64,
    w: usize,
    h: usize,
}

impl Grid {
    fn xy(&self, c: usize) -> P {
        (
            self.x0 + (c % self.w) as f64 * self.g,
            self.y0 + (c / self.w) as f64 * self.g,
        )
    }

    fn cell(&self, p: P) -> usize {
        let i = ((p.0 - self.x0) / self.g).round().clamp(0.0, (self.w - 1) as f64) as usize;
        let j = ((p.1 - self.y0) / self.g).round().clamp(0.0, (self.h - 1) as f64) as usize;
        j * self.w + i
    }

    fn each(&self, shape: &Shape, r: f64, mut f: impl FnMut(usize)) {
        let bb = shape.bbox();
        let lo = |v: f64, o: f64| (((v - r - o) / self.g).floor().max(0.0)) as usize;
        let hi = |v: f64, o: f64, m: usize| (((v + r - o) / self.g).ceil().max(0.0) as usize).min(m - 1);
        let (i0, j0, i1, j1) = (
            lo(bb.0, self.x0),
            lo(bb.1, self.y0),
            hi(bb.2, self.x0, self.w),
            hi(bb.3, self.y0, self.h),
        );
        if i0 > i1 || j0 > j1 {
            return;
        }
        for j in j0..=j1 {
            for i in i0..=i1 {
                let c = j * self.w + i;
                if shape.dist(self.xy(c)) <= r {
                    f(c);
                }
            }
        }
    }
}

/// Copper as (net, shape, layers).
type Copper = (Option<String>, Shape, Vec<usize>);

fn copper(board: &Board, routed: Option<&Routed>) -> Vec<Copper> {
    let nl = board.copper.len();
    let all: Vec<usize> = (0..nl).collect();
    let mut out: Vec<Copper> = Vec::new();
    for p in &board.pads {
        out.push((p.net.clone(), p.shape, p.layers.clone()));
    }
    for t in &board.tracks {
        let shape = Shape::Capsule {
            a: t.a,
            b: t.b,
            r: t.width / 2.0,
        };
        out.push((t.net.clone(), shape, vec![t.layer]));
    }
    for v in &board.vias {
        let shape = Shape::Circle {
            c: v.at,
            r: v.diameter / 2.0,
        };
        out.push((v.net.clone(), shape, (v.layers.0..=v.layers.1).collect()));
    }
    for &h in &board.holes {
        out.push((None, h, all.clone()));
    }
    for a in &board.art {
        out.push((a.net.clone(), a.shape, a.layers.clone()));
    }
    if let Some(r) = routed {
        for t in &r.tracks {
            let shape = Shape::Capsule {
                a: t.a,
                b: t.b,
                r: t.width / 2.0,
            };
            out.push((Some(t.net.clone()), shape, vec![t.layer]));
        }
        for v in &r.vias {
            let shape = Shape::Circle {
                c: v.at,
                r: v.diameter / 2.0,
            };
            out.push((Some(v.net.clone()), shape, (v.layers.0..=v.layers.1).collect()));
        }
    }
    out
}

/// For one poured net: which piece each of its pads is on (None: on no copper
/// the grid can see), and the size of each piece.
fn pieces(board: &Board, rules: &Rules, net: &str, items: &[Copper], gr: &Grid) -> (Vec<Option<u32>>, Vec<u32>) {
    let (of_pad, sizes, _) = labelled(board, rules, net, items, gr, true);
    (of_pad, sizes)
}

/// As `pieces`, and third: the piece each grid point is on, layer by layer
/// (u32::MAX: none of the net's copper there). `pad_barrels`: a drilled pad
/// joins the layers it reaches, as a via does; without, only vias do, and the
/// copper round a drilled pad on each layer has to reach the rest by itself
/// (KiCad wants a pad's thermal spokes on copper that goes somewhere).
fn labelled(
    board: &Board,
    rules: &Rules,
    net: &str,
    items: &[Copper],
    gr: &Grid,
    pad_barrels: bool,
) -> (Vec<Option<u32>>, Vec<u32>, Vec<u32>) {
    let nl = board.copper.len();
    let n = gr.w * gr.h;
    // 1: copper of the net (pour or its own tracks and pads)
    let mut on = vec![0u8; n * nl];
    for z in board
        .zones
        .iter()
        .filter(|z| z.keepout.is_none() && z.net.as_deref() == Some(net))
    {
        let (mut x0, mut y0, mut x1, mut y1) = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in &z.outline {
            x0 = x0.min(p.0);
            y0 = y0.min(p.1);
            x1 = x1.max(p.0);
            y1 = y1.max(p.1);
        }
        let i0 = (((x0 - gr.x0) / gr.g).floor().max(0.0)) as usize;
        let j0 = (((y0 - gr.y0) / gr.g).floor().max(0.0)) as usize;
        let i1 = ((((x1 - gr.x0) / gr.g).ceil().max(0.0)) as usize).min(gr.w - 1);
        let j1 = ((((y1 - gr.y0) / gr.g).ceil().max(0.0)) as usize).min(gr.h - 1);
        for j in j0..=j1 {
            for i in i0..=i1 {
                let c = j * gr.w + i;
                if geom::in_polygon(gr.xy(c), &z.outline) {
                    for &l in &z.layers {
                        on[l * n + c] = 1;
                    }
                }
            }
        }
    }
    // other nets' copper pushes the pour back by the clearance and half its least width
    let zone_clr = board
        .zones
        .iter()
        .filter(|z| z.keepout.is_none() && z.net.as_deref() == Some(net))
        .map(|z| z.clearance)
        .fold(0.0, f64::max);
    let thin = board
        .zones
        .iter()
        .filter(|z| z.keepout.is_none() && z.net.as_deref() == Some(net))
        .map(|z| z.min_thickness)
        .fold(0.0, f64::max);
    for (onet, shape, layers) in items {
        if onet.as_deref() == Some(net) {
            continue;
        }
        let other = Item {
            net: onet.as_deref(),
            kind: None,
            shape: Some(shape),
            layer: None,
            paired: None,
        };
        let clr = rules.clearance(&Item::track(net), &other).max(zone_clr);
        for &l in layers {
            let layer = &mut on[l * n..(l + 1) * n];
            // (less half a cell: a neck the grid's points straddle is still a neck, and
            // a false alarm costs more trust than a miss that --drc catches anyway)
            gr.each(shape, (clr + thin / 2.0 - gr.g / 2.0).max(0.0), |c| layer[c] = 0);
        }
    }
    // the net's own copper conducts wherever it is; through pads and vias join
    // the layers they reach. up[l * n + c]: layer l is joined to the next one down at c
    let mut up = vec![false; n * nl];
    // (`copper` lists the board's pads first)
    for (at, (onet, shape, layers)) in items.iter().enumerate() {
        if onet.as_deref() != Some(net) {
            continue;
        }
        for &l in layers {
            let layer = &mut on[l * n..(l + 1) * n];
            gr.each(shape, 0.0, |c| layer[c] = 1);
            layer[gr.cell(shape.centre())] = 1;
        }
        if !pad_barrels && at < board.pads.len() {
            continue;
        }
        if let (Some(&a), Some(&b)) = (layers.iter().min(), layers.iter().max()) {
            let c = gr.cell(shape.centre());
            for l in a..b {
                up[l * n + c] = true;
            }
        }
    }
    let mut label = vec![u32::MAX; n * nl];
    let mut sizes: Vec<u32> = Vec::new();
    let mut stack: Vec<usize> = Vec::new();
    for start in 0..n * nl {
        if on[start] == 0 || label[start] != u32::MAX {
            continue;
        }
        let id = sizes.len() as u32;
        let mut size = 0u32;
        label[start] = id;
        stack.push(start);
        while let Some(s) = stack.pop() {
            size += 1;
            let (l, c) = (s / n, s % n);
            let (i, j) = (c % gr.w, c / gr.w);
            let mut go = |t: usize| {
                if on[t] != 0 && label[t] == u32::MAX {
                    label[t] = id;
                    stack.push(t);
                }
            };
            if i > 0 {
                go(s - 1);
            }
            if i + 1 < gr.w {
                go(s + 1);
            }
            if j > 0 {
                go(s - gr.w);
            }
            if j + 1 < gr.h {
                go(s + gr.w);
            }
            // along the barrel, both ways, to every layer it reaches
            let mut l2 = l;
            while l2 + 1 < nl && up[l2 * n + c] {
                l2 += 1;
                go(l2 * n + c);
            }
            let mut l2 = l;
            while l2 > 0 && up[(l2 - 1) * n + c] {
                l2 -= 1;
                go(l2 * n + c);
            }
        }
        sizes.push(size);
    }
    let of_pad = board
        .pads
        .iter()
        .filter(|p| p.net.as_deref() == Some(net))
        .map(|p| {
            let c = gr.cell(p.shape.centre());
            p.layers.iter().map(|&l| label[l * n + c]).find(|&id| id != u32::MAX)
        })
        .collect();
    (of_pad, sizes, label)
}

/// For each pad of `net`, in the board's order: whether the net's copper as
/// it stands (pours, tracks, vias) joins it to the plane the net has on the
/// layers `planes`. An estimate, like the rest of this file.
pub fn on_plane(board: &Board, rules: &Rules, net: &str, planes: &[usize], grid: f64) -> Vec<bool> {
    let pads = board.pads.iter().filter(|p| p.net.as_deref() == Some(net)).count();
    let gr = match fine_grid(board, grid) {
        Some(g) => g,
        None => return vec![false; pads],
    };
    let (of_pad, sizes, label) = labelled(board, rules, net, &copper(board, None), &gr, true);
    // the plane: the piece with the most copper on those layers
    let n = gr.w * gr.h;
    let mut there = vec![0u32; sizes.len()];
    for &l in planes {
        for id in label[l * n..(l + 1) * n].iter().filter(|id| **id != u32::MAX) {
            there[*id as usize] += 1;
        }
    }
    let plane = there
        .iter()
        .enumerate()
        .filter(|(_, k)| **k > 0)
        .max_by_key(|(i, k)| (**k, std::cmp::Reverse(*i)))
        .map(|(i, _)| i as u32);
    of_pad.iter().map(|id| id.is_some() && *id == plane).collect()
}

/// For each pad of `net`, in the board's order: whether it is on the largest
/// piece of the net's copper as it stands (pours, tracks, vias), the piece the
/// pour is taken to join.
pub fn on_main(board: &Board, rules: &Rules, net: &str, grid: f64) -> Vec<bool> {
    let pads = board.pads.iter().filter(|p| p.net.as_deref() == Some(net)).count();
    let gr = match fine_grid(board, grid) {
        Some(g) => g,
        None => return vec![false; pads],
    };
    let (of_pad, sizes) = pieces(board, rules, net, &copper(board, None), &gr);
    let main = sizes.iter().enumerate().max_by_key(|(_, s)| **s).map(|(i, _)| i as u32);
    of_pad.iter().map(|id| id.is_some() && *id == main).collect()
}

/// The grid the pours are worked out on.
fn fine_grid(board: &Board, grid: f64) -> Option<Grid> {
    let (bx0, by0, bx1, by1) = board.extent()?;
    let g = (grid / 2.0).clamp(0.05, 0.1); // fine enough to see the narrow necks a pour squeezes through
    let x0 = ((bx0 - 1.0) / g).floor() * g;
    let y0 = ((by0 - 1.0) / g).floor() * g;
    Some(Grid {
        x0,
        y0,
        g,
        w: (((bx1 + 1.0) - x0) / g).ceil() as usize + 1,
        h: (((by1 + 1.0) - y0) / g).ceil() as usize + 1,
    })
}

/// Pads of poured nets that the routed copper cut off from their pour.
/// `nets`: only these (None: every net with a pour).
pub fn cuts(board: &Board, rules: &Rules, routed: &Routed, grid: f64, nets: Option<&[String]>) -> Vec<Cut> {
    let gr = match fine_grid(board, grid) {
        Some(g) => g,
        None => return Vec::new(),
    };
    let before = copper(board, None);
    let after = copper(board, Some(routed));
    let mut out = Vec::new();
    for net in board.poured_nets() {
        if let Some(only) = nets {
            if !only.contains(&net) {
                continue;
            }
        }
        let names: Vec<String> = board
            .pads
            .iter()
            .filter(|p| p.net.as_deref() == Some(net.as_str()))
            .map(|p| p.name())
            .collect();
        if names.len() < 2 {
            continue;
        }
        let (was, was_sizes) = pieces(board, rules, &net, &before, &gr);
        let (now, now_sizes) = pieces(board, rules, &net, &after, &gr);
        let biggest = |sizes: &[u32]| sizes.iter().enumerate().max_by_key(|(_, s)| **s).map(|(i, _)| i as u32);
        let (main_was, main_now) = (biggest(&was_sizes), biggest(&now_sizes));
        let cut_off: Vec<String> = (0..names.len())
            .filter(|&k| was[k].is_some() && was[k] == main_was && now[k] != main_now)
            .map(|k| names[k].clone())
            .collect();
        if !cut_off.is_empty() {
            out.push(Cut {
                net,
                pads: names.len(),
                cut_off,
            });
        }
    }
    out
}

/// A via put in to join a piece of a pour, cut off by new copper, to the rest.
#[derive(Clone, Debug)]
pub struct Stitch {
    pub net: String,
    pub at: P,
    /// The pads it joined to the rest again (none: a piece with no pad on it).
    pub pads: Vec<String>,
}

/// The grid points inside the board's outline (all of them, if it has none).
fn inside(board: &Board, gr: &Grid) -> Vec<bool> {
    let n = gr.w * gr.h;
    if board.edges.is_empty() {
        return vec![true; n];
    }
    let mut wall = vec![false; n];
    for &(a, b) in &board.edges {
        gr.each(&Shape::Capsule { a, b, r: 0.0 }, gr.g * 0.75, |c| wall[c] = true);
    }
    // flood from the grid's border, stopped by the edge lines
    let mut out = vec![false; n];
    let mut stack: Vec<usize> = Vec::new();
    for i in 0..gr.w {
        stack.push(i);
        stack.push((gr.h - 1) * gr.w + i);
    }
    for j in 0..gr.h {
        stack.push(j * gr.w);
        stack.push(j * gr.w + gr.w - 1);
    }
    while let Some(c) = stack.pop() {
        if out[c] || wall[c] {
            continue;
        }
        out[c] = true;
        let (i, j) = (c % gr.w, c / gr.w);
        if i > 0 {
            stack.push(c - 1);
        }
        if i + 1 < gr.w {
            stack.push(c + 1);
        }
        if j > 0 {
            stack.push(c - gr.w);
        }
        if j + 1 < gr.h {
            stack.push(c + gr.w);
        }
    }
    (0..n).map(|c| !out[c] && !wall[c]).collect()
}

/// Join again what the new copper cut off: where a piece of a pour no longer
/// reaches the rest of its net, and the net has a pour on another layer that
/// does, a via of the net is put where the two lie one over the other. The vias
/// are added to `routed`.
///
/// A drilled pad does not count as joining its layers here: the pour round it
/// on each layer has to reach the rest of the net by pour or by a via, or
/// KiCad finds its thermal spokes on an island.
///
/// A piece is joined if it has a pad of the net on it, or if the zone keeps
/// pieces without pads when it is filled and the piece is a square millimetre
/// or more. Only what was whole before routing is mended; a piece that was
/// apart already is left as the board's own business. Like the rest of this
/// file it works on an estimate of the fill, and each via is then checked
/// exactly against every clearance.
pub fn stitch(board: &Board, rules: &Rules, routed: &mut Routed, grid: f64, via_in_pad: bool) -> Vec<Stitch> {
    let mut done: Vec<Stitch> = Vec::new();
    let nl = board.copper.len();
    let gr = match fine_grid(board, grid) {
        Some(g) if nl >= 2 => g,
        _ => return done,
    };
    let n = gr.w * gr.h;
    let within = inside(board, &gr);
    let before = copper(board, None);
    let biggest = |sizes: &[u32]| sizes.iter().enumerate().max_by_key(|(_, s)| **s).map(|(i, _)| i as u32);
    for net in board.poured_nets() {
        let (d, drill) = match rules.via(&net) {
            Some(v) => v,
            None => continue,
        };
        let pads: Vec<&crate::board::Pad> = board
            .pads
            .iter()
            .filter(|p| p.net.as_deref() == Some(net.as_str()))
            .collect();
        let keeps = board
            .zones
            .iter()
            .any(|z| z.keepout.is_none() && z.net.as_deref() == Some(net.as_str()) && z.keeps_islands);
        let (_, was_sizes, was) = labelled(board, rules, &net, &before, &gr, false);
        let main_was = match biggest(&was_sizes) {
            Some(m) => m,
            None => continue,
        };
        let edge = rules.edge_clearance(&net) + d / 2.0;
        // (a board is not stitched without end: each round adds one via)
        for _ in 0..64 {
            let after = copper(board, Some(routed));
            let (_, now_sizes, now) = labelled(board, rules, &net, &after, &gr, false);
            let main_now = match biggest(&now_sizes) {
                Some(m) => m,
                None => break,
            };
            // the pieces the new copper cut off the main one
            let mut cut = vec![false; now_sizes.len()];
            for s in 0..n * nl {
                if now[s] != u32::MAX && now[s] != main_now && was[s] == main_was {
                    cut[now[s] as usize] = true;
                }
            }
            let fits = |p: P, piece: u32| -> bool {
                // in the pour on both layers, all of the via's own copper
                let mut solid = [false, false];
                for l in 0..nl {
                    let layer = &now[l * n..(l + 1) * n];
                    let (mut on_piece, mut on_main, mut any) = (true, true, false);
                    gr.each(&Shape::Circle { c: p, r: 0.0 }, d / 2.0, |c| {
                        any = true;
                        on_piece &= layer[c] == piece;
                        on_main &= layer[c] == main_now;
                    });
                    solid[0] |= any && on_piece;
                    solid[1] |= any && on_main;
                }
                if !solid[0] || !solid[1] {
                    return false;
                }
                if board.edges.iter().any(|&(a, b)| geom::seg_dist(p, a, b) < edge - 1e-6) {
                    return false;
                }
                for z in &board.zones {
                    if z.keepout.as_ref().map(|k| k.vias).unwrap_or(false)
                        && (geom::in_polygon(p, &z.outline) || geom::polygon_edge_dist(p, &z.outline) < d / 2.0 + 1e-6)
                    {
                        return false;
                    }
                }
                // its own pads: no hole in a surface-mount one (unless asked), none too near a drilled one
                for pad in &pads {
                    let gap = pad.shape.dist(p);
                    if pad.through && gap < rules.hole_to_hole + drill / 2.0 {
                        return false;
                    }
                    if !pad.through && !via_in_pad && gap < drill / 2.0 + 0.05 {
                        return false;
                    }
                }
                // holes keep their distance from each other, whatever the net
                let holes = board
                    .vias
                    .iter()
                    .map(|v| (v.at, v.drill))
                    .chain(routed.vias.iter().map(|v| (v.at, v.drill)));
                for (at, h) in holes {
                    if geom::dist(p, at) < rules.hole_to_hole + (drill + h) / 2.0 - 1e-6 {
                        return false;
                    }
                }
                // other nets' copper, and holes
                for (onet, shape, _) in &after {
                    if onet.as_deref() == Some(net.as_str()) {
                        continue;
                    }
                    let bb = shape.bbox();
                    if p.0 < bb.0 - 10.0 || p.0 > bb.2 + 10.0 || p.1 < bb.1 - 10.0 || p.1 > bb.3 + 10.0 {
                        continue;
                    }
                    let other = Item {
                        net: onet.as_deref(),
                        kind: None,
                        shape: Some(shape),
                        layer: None,
                        paired: None,
                    };
                    let mut clr = rules.clearance(&Item::via(&net), &other);
                    if onet.is_none() {
                        clr = clr.max(rules.hole_clearance);
                    }
                    if shape.dist(p) < clr + d / 2.0 - 1e-6 {
                        return false;
                    }
                }
                true
            };
            let mut added: Option<Stitch> = None;
            for piece in 0..now_sizes.len() as u32 {
                if piece == main_now || !cut[piece as usize] {
                    continue;
                }
                // its pads that were on the main piece and are on this one now (a drilled
                // pad layer by layer: its copper on each has to reach the rest)
                let lost: Vec<usize> = (0..pads.len())
                    .filter(|&k| {
                        let c = gr.cell(pads[k].shape.centre());
                        pads[k]
                            .layers
                            .iter()
                            .any(|&l| now[l * n + c] == piece && was[l * n + c] == main_was)
                    })
                    .collect();
                let area = now_sizes[piece as usize] as f64 * gr.g * gr.g;
                if lost.is_empty() && !(keeps && area >= 1.0) {
                    continue;
                }
                // where the piece lies over (or under) the main one, nearest its pads first
                let mut spots: Vec<(f64, usize)> = (0..n)
                    .filter(|&c| {
                        within[c]
                            && (0..nl).any(|l| now[l * n + c] == piece)
                            && (0..nl).any(|l| now[l * n + c] == main_now)
                    })
                    .map(|c| {
                        let p = gr.xy(c);
                        let near = lost
                            .iter()
                            .map(|&k| geom::dist(p, pads[k].shape.centre()))
                            .fold(f64::INFINITY, f64::min);
                        (if near.is_finite() { near } else { 0.0 }, c)
                    })
                    .collect();
                spots.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
                // (to the micrometre: the grid's sums leave dust in the last places)
                let at = |c: usize| {
                    let p = gr.xy(c);
                    ((p.0 * 1e6).round() / 1e6, (p.1 * 1e6).round() / 1e6)
                };
                if let Some(&(_, c)) = spots.iter().find(|&&(_, c)| fits(at(c), piece)) {
                    added = Some(Stitch {
                        net: net.clone(),
                        at: at(c),
                        pads: lost.iter().map(|&k| pads[k].name()).collect(),
                    });
                    break;
                }
            }
            match added {
                Some(s) => {
                    routed.vias.push(NewVia {
                        net: net.clone(),
                        at: s.at,
                        diameter: d,
                        drill,
                        layers: (0, nl - 1),
                    });
                    done.push(s);
                }
                None => break,
            }
        }
    }
    done
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::router::NewTrack;

    #[test]
    fn a_track_across_a_pour_cuts_it() {
        // a GND pour on the front with a pad at each end; a new track runs right across it
        let t = r#"(kicad_pcb (version 20260206) (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
          (footprint "P" (at 3 5) (property "Reference" "A") (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "GND")))
          (footprint "P" (at 17 5) (property "Reference" "B") (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "GND")))
          (zone (net "GND") (layer "F.Cu") (connect_pads (clearance 0.3)) (min_thickness 0.25)
            (polygon (pts (xy 0.5 0.5) (xy 19.5 0.5) (xy 19.5 9.5) (xy 0.5 9.5))))
          (gr_rect (start 0 0) (end 20 10) (layer "Edge.Cuts")))"#;
        let b = Board::parse(t.to_string()).unwrap();
        let rules = Rules::default();
        let mut r = Routed::default();
        assert!(
            cuts(&b, &rules, &r, 0.25, None).is_empty(),
            "nothing added, nothing cut"
        );
        // a track that stops short leaves a way round
        r.tracks.push(NewTrack {
            net: "S".into(),
            layer: 0,
            a: (10.0, 0.0),
            b: (10.0, 6.0),
            width: 0.25,
        });
        assert!(cuts(&b, &rules, &r, 0.25, None).is_empty());
        // right across: the pour is in two, and one pad is off the larger piece
        r.tracks[0].b = (10.0, 10.0);
        let c = cuts(&b, &rules, &r, 0.25, None);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].net, "GND");
        assert_eq!(c[0].cut_off.len(), 1);
        // on the back there is no pour to cut
        r.tracks[0].layer = 1;
        assert!(cuts(&b, &rules, &r, 0.25, None).is_empty());
    }

    #[test]
    fn a_cut_pour_is_joined_again_through_the_other_layer() {
        // GND is poured on both layers and joined by a via at the left; a new track
        // right across the front cuts pad B's half off
        let board = |back: bool| {
            let t = format!(
                r#"(kicad_pcb (version 20260206) (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
          (footprint "P" (at 3 5) (property "Reference" "A") (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "GND")))
          (footprint "P" (at 17 5) (property "Reference" "B") (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "GND")))
          (via (at 3 8) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "GND"))
          (zone (net "GND") (layer "F.Cu") (connect_pads (clearance 0.3)) (min_thickness 0.25)
            (polygon (pts (xy 0.5 0.5) (xy 19.5 0.5) (xy 19.5 9.5) (xy 0.5 9.5))))
          {}
          (gr_rect (start 0 0) (end 20 10) (layer "Edge.Cuts")))"#,
                if back {
                    r#"(zone (net "GND") (layer "B.Cu") (connect_pads (clearance 0.3)) (min_thickness 0.25)
            (polygon (pts (xy 0.5 0.5) (xy 19.5 0.5) (xy 19.5 9.5) (xy 0.5 9.5))))"#
                } else {
                    ""
                }
            );
            Board::parse(t).unwrap()
        };
        let rules = Rules::default();
        let across = NewTrack {
            net: "S".into(),
            layer: 0,
            a: (10.0, 0.0),
            b: (10.0, 10.0),
            width: 0.25,
        };
        let b = board(true);
        let mut r = Routed::default();
        assert!(
            stitch(&b, &rules, &mut r, 0.25, false).is_empty(),
            "nothing cut, nothing to mend"
        );
        r.tracks.push(across.clone());
        assert_eq!(cuts(&b, &rules, &r, 0.25, None).len(), 1);
        let done = stitch(&b, &rules, &mut r, 0.25, false);
        assert_eq!(done.len(), 1, "{:?}", done);
        assert_eq!(done[0].pads, vec!["B.1".to_string()]);
        assert_eq!(r.vias.len(), 1);
        let v = &r.vias[0];
        assert_eq!((v.net.as_str(), v.layers), ("GND", (0, 1)));
        // on pad B's side, clear of the track that did the cutting, off the pad, inside the board
        let clr = rules.clearance(&Item::via("GND"), &Item::track("S"));
        assert!(v.at.0 - 10.0 >= 0.125 + clr + v.diameter / 2.0 - 1e-9, "{:?}", v.at);
        assert!(b.pads[1].shape.dist(v.at) > v.drill / 2.0);
        assert!(v.at.0 < 19.5 && v.at.1 > 0.5 && v.at.1 < 9.5, "{:?}", v.at);
        assert!(cuts(&b, &rules, &r, 0.25, None).is_empty(), "the pour is whole again");
        // and once is enough
        assert!(stitch(&b, &rules, &mut r, 0.25, false).is_empty());
        // with no pour on the back there is nothing to join the piece to: it stays reported
        let b = board(false);
        let mut r = Routed::default();
        r.tracks.push(across);
        assert!(stitch(&b, &rules, &mut r, 0.25, false).is_empty());
        assert!(r.vias.is_empty());
        assert_eq!(cuts(&b, &rules, &r, 0.25, None).len(), 1);
    }
    #[test]
    fn the_pour_round_a_drilled_pad_is_joined_again() {
        // GND on both layers, joined by a via; a drilled GND pad in the middle. New
        // tracks on the back box the pad in: its copper there is an island that only
        // the pad's own barrel joins to the rest, which KiCad does not take for joined
        let t = r#"(kicad_pcb (version 20260206) (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
          (footprint "P" (at 15 5) (property "Reference" "J") (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu") (net "GND")))
          (footprint "P" (at 3 5) (property "Reference" "A") (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "GND")))
          (via (at 3 8) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "GND"))
          (zone (net "GND") (layer "F.Cu") (connect_pads (clearance 0.3)) (min_thickness 0.25)
            (polygon (pts (xy 0.5 0.5) (xy 29.5 0.5) (xy 29.5 9.5) (xy 0.5 9.5))))
          (zone (net "GND") (layer "B.Cu") (connect_pads (clearance 0.3)) (min_thickness 0.25)
            (polygon (pts (xy 0.5 0.5) (xy 29.5 0.5) (xy 29.5 9.5) (xy 0.5 9.5))))
          (gr_rect (start 0 0) (end 30 10) (layer "Edge.Cuts")))"#;
        let b = Board::parse(t.to_string()).unwrap();
        let rules = Rules::default();
        let mut r = Routed::default();
        let corners = [(12.5, 2.5), (17.5, 2.5), (17.5, 7.5), (12.5, 7.5)];
        for k in 0..4 {
            r.tracks.push(NewTrack {
                net: "S".into(),
                layer: 1,
                a: corners[k],
                b: corners[(k + 1) % 4],
                width: 0.25,
            });
        }
        let done = stitch(&b, &rules, &mut r, 0.25, false);
        assert_eq!(done.len(), 1, "{:?}", done);
        assert_eq!(done[0].pads, vec!["J.1".to_string()]);
        // inside the box, clear of the pad's hole and of the tracks
        let v = &r.vias[0];
        assert!(
            v.at.0 > 12.5 && v.at.0 < 17.5 && v.at.1 > 2.5 && v.at.1 < 7.5,
            "{:?}",
            v.at
        );
        assert!(b.pads[0].shape.dist(v.at) >= rules.hole_to_hole + v.drill / 2.0 - 1e-9);
        let clr = rules.clearance(&Item::via("GND"), &Item::track("S"));
        for t in &r.tracks {
            assert!(geom::seg_dist(v.at, t.a, t.b) >= 0.125 + clr + v.diameter / 2.0 - 1e-9);
        }
        assert!(stitch(&b, &rules, &mut r, 0.25, false).is_empty(), "once is enough");
        // the same box on the front, where the pad has no hold on the back's pour to lose
        // either: it is boxed in there instead, and mended the same way
        let mut r = Routed::default();
        for k in 0..4 {
            r.tracks.push(NewTrack {
                net: "S".into(),
                layer: 0,
                a: corners[k],
                b: corners[(k + 1) % 4],
                width: 0.25,
            });
        }
        assert_eq!(stitch(&b, &rules, &mut r, 0.25, false).len(), 1);
    }
}
