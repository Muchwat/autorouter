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
use crate::router::Routed;
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
        out.push((v.net.clone(), shape, all.clone()));
    }
    for &h in &board.holes {
        out.push((None, h, all.clone()));
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
            out.push((Some(v.net.clone()), shape, all.clone()));
        }
    }
    out
}

/// For one poured net: which piece each of its pads is on (None: on no copper
/// the grid can see), and the size of each piece.
fn pieces(board: &Board, rules: &Rules, net: &str, items: &[Copper], gr: &Grid) -> (Vec<Option<u32>>, Vec<u32>) {
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
        };
        let clr = rules.clearance(&Item::track(net), &other).max(zone_clr);
        for &l in layers {
            let layer = &mut on[l * n..(l + 1) * n];
            // (less half a cell: a neck the grid's points straddle is still a neck, and
            // a false alarm costs more trust than a miss that --drc catches anyway)
            gr.each(shape, (clr + thin / 2.0 - gr.g / 2.0).max(0.0), |c| layer[c] = 0);
        }
    }
    // the net's own copper conducts wherever it is; through pads and vias join the layers
    let mut joins: Vec<usize> = Vec::new();
    for (onet, shape, layers) in items {
        if onet.as_deref() != Some(net) {
            continue;
        }
        for &l in layers {
            let layer = &mut on[l * n..(l + 1) * n];
            gr.each(shape, 0.0, |c| layer[c] = 1);
            layer[gr.cell(shape.centre())] = 1;
        }
        if layers.len() > 1 {
            joins.push(gr.cell(shape.centre()));
        }
    }
    let mut join = vec![false; n];
    for c in joins {
        join[c] = true;
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
            if join[c] {
                for l2 in 0..nl {
                    if l2 != l {
                        go(l2 * n + c);
                    }
                }
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
    (of_pad, sizes)
}

/// Pads of poured nets that the routed copper cut off from their pour.
/// `nets`: only these (None: every net with a pour).
pub fn cuts(board: &Board, rules: &Rules, routed: &Routed, grid: f64, nets: Option<&[String]>) -> Vec<Cut> {
    let (bx0, by0, bx1, by1) = match board.extent() {
        Some(e) => e,
        None => return Vec::new(),
    };
    let g = (grid / 2.0).clamp(0.05, 0.1); // fine enough to see the narrow necks a pour squeezes through
    let x0 = ((bx0 - 1.0) / g).floor() * g;
    let y0 = ((by0 - 1.0) / g).floor() * g;
    let gr = Grid {
        x0,
        y0,
        g,
        w: (((bx1 + 1.0) - x0) / g).ceil() as usize + 1,
        h: (((by1 + 1.0) - y0) / g).ceil() as usize + 1,
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
}
