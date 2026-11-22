//! Fan-out: a way out for pads the grid cannot reach.
//!
//! The inner balls of a BGA and the pads of a tight fine-pitch part have no
//! grid point near them that a track may start from, or only ones in a pocket
//! shut in by the neighbouring pads; and in a row of pads closer together
//! than tracks can run on the grid, the pads fight over the same few points.
//! For each such pad this finds, by exact geometry (not on the grid):
//!
//! * an escape on the pad's own layer: a straight piece out of the pad and,
//!   if need be, a second one, ending on a free grid point with room around
//!   it; or
//! * a dog-bone: a short track to a via that fits between the neighbours (in
//!   the middle of four balls, say), from where the net goes on on another
//!   layer.
//!
//! The copper found is handed to the router as if the board already had it,
//! and the router carries on from its far end. Fan-outs it does not use in the
//! end are not written.

use super::*;

/// A pad's way out.
#[derive(Clone, Debug)]
pub(super) struct Fan {
    pub net: String,
    /// The pad, as "U1.B2".
    pub pad: String,
    /// The pad's layer.
    pub layer: usize,
    /// Pieces of track on the pad's layer: (from, to, width).
    pub tracks: Vec<(P, P, f64)>,
    /// A via at the far end: (at, diameter, drill, span).
    pub via: Option<(P, f64, f64, Span)>,
    /// Where the router carries on from: the last track's end, or the via.
    pub end: P,
}

/// Fan-out copper already decided, which later fan-outs keep clear of.
struct Placed {
    net: String,
    shape: Shape,
    layers: Span,
    /// The drill, for a via.
    hole: Option<f64>,
    /// Not copper but the room kept for the router's own track where it takes
    /// over (its half width and margin are in the shape already).
    landing: bool,
}

/// The part a pad belongs to: its middle, and how far its pads reach from it.
#[derive(Clone, Copy)]
struct Part {
    middle: P,
    radius: f64,
    /// The box round its pads' centres.
    bbox: (f64, f64, f64, f64),
}

/// What the flood needs that asks whether a grid point leads out from under
/// a part: walls around the fan-outs made so far, per layer, and its marks.
struct Flood {
    walls: Vec<Vec<u8>>,
    mark: Vec<u32>,
    gen: u32,
    stack: Vec<usize>,
}

/// How far apart the tries along a direction are, in mm.
const STEP: f64 = 0.05;
/// What a via costs against going further on the pad's own layer, in mm.
const VIA_COST: f64 = 0.75;
/// A via is nudged until it has this much to spare, where there is that much
/// (between the balls of a BGA there is not, and it ends up in the middle).
const SPARE: f64 = 0.1;
/// The gap a dog-bone's via keeps from its own pad, so that solder mask can
/// lie between the two (where there is no room for that, the hole alone stays
/// off the pad).
const MASK: f64 = 0.05;

impl Router<'_> {
    /// The pads that have no way out on the grid, and a way out for each one
    /// that has one. Second: (pad, net, why) for each pad that has none.
    pub(super) fn fanout(&mut self) -> (Vec<Fan>, Vec<(String, String, String)>) {
        let mut fans: Vec<Fan> = Vec::new();
        let mut notes: Vec<(String, String, String)> = Vec::new();
        let mut placed: Vec<Placed> = Vec::new();
        let mut seen: std::collections::HashSet<(String, String)> = std::collections::HashSet::new();
        // each part's middle and reach: its pads are fanned outwards, and a
        // fan-out must lead out from under it
        let mut sums: HashMap<String, (f64, f64, usize)> = HashMap::new();
        for p in &self.board.pads {
            let c = p.shape.centre();
            let e = sums.entry(p.reference.clone()).or_insert((0.0, 0.0, 0));
            *e = (e.0 + c.0, e.1 + c.1, e.2 + 1);
        }
        let mut parts: HashMap<String, Part> = sums
            .into_iter()
            .map(|(r, (sx, sy, k))| {
                (
                    r,
                    Part {
                        middle: (sx / k as f64, sy / k as f64),
                        radius: 0.0,
                        bbox: (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                    },
                )
            })
            .collect();
        for p in &self.board.pads {
            if let Some(part) = parts.get_mut(&p.reference) {
                let b = p.shape.bbox();
                let far = [(b.0, b.1), (b.0, b.3), (b.2, b.1), (b.2, b.3)]
                    .iter()
                    .map(|&q| geom::dist(q, part.middle))
                    .fold(0.0, f64::max);
                part.radius = part.radius.max(far);
                let c = p.shape.centre();
                part.bbox = (
                    part.bbox.0.min(c.0),
                    part.bbox.1.min(c.1),
                    part.bbox.2.max(c.0),
                    part.bbox.3.max(c.1),
                );
            }
        }
        let mut flood = Flood {
            walls: vec![vec![0u8; self.gr.n]; self.gr.nl],
            mark: vec![0u32; self.gr.n],
            gen: 0,
            stack: Vec::new(),
        };
        // the pads to bring out: (part, how deep inside it, job, terminal,
        // whether the grid has no way in at all)
        let mut todo: Vec<(String, f64, usize, usize, bool)> = Vec::new();
        for j in 0..self.jobs.len() {
            if self.jobs[j].pair.is_some() {
                continue;
            }
            let net = self.jobs[j].net.clone();
            let mut alone: Vec<usize> = Vec::new();
            for t in 0..self.jobs[j].terminals.len() {
                let term = &self.jobs[j].terminals[t];
                // a surface-mount pad with nothing joined to it yet
                let bare = term.parts.len() == 1 && term.parts[0].2 && term.parts[0].1.len() == 1;
                if bare && seen.insert((net.clone(), term.label.clone())) {
                    alone.push(t);
                }
            }
            if alone.is_empty() {
                continue;
            }
            let entries = self.entries(j);
            let room = self.room(j);
            let roomy = room.iter().cloned().max().unwrap_or(0) / 20;
            let (block, _) = self.hard(j);
            let n = self.gr.n;
            for t in alone {
                let term = &self.jobs[j].terminals[t];
                let reference = term.label.split('.').next().unwrap_or("").to_string();
                let part = parts.get(&reference).cloned();
                // the grid has a way in if a track can start by the pad and has room
                // to go on, by a via if need be. Where the net may use blind vias, a
                // pad that only a via would get out from under its part (a ball inside
                // a BGA) is fanned out all the same: its dog-bone is given the layer
                // with nothing in its way, which the grid search would not think of
                let layered = self.jobs[j].spans.len() > 1;
                let open = entries[t].iter().any(|e| {
                    let (l, c) = (e.0 as usize / n, e.0 as usize % n);
                    room[e.0 as usize] >= roomy && (!layered || self.leads_out(&mut flood, &block[l], l, c, part))
                });
                // (a pad in a pocket together with another piece of its net needs no
                // way out: the two are joined inside it. Regions of one size are one region)
                let together = entries[t].iter().any(|e| {
                    room[e.0 as usize] < roomy
                        && entries
                            .iter()
                            .enumerate()
                            .any(|(t2, es)| t2 != t && es.iter().any(|e2| room[e2.0 as usize] == room[e.0 as usize]))
                });
                // a row of pads closer together than tracks can run on the grid:
                // each gets a landing of its own, or they fight over the same few points
                let (shape, layers) = (term.parts[0].0, &term.parts[0].1);
                let pitch = {
                    let w = self.jobs[j].widths[layers[0]];
                    let need = w + self.rules.class_of(&net).clearance + self.jobs[j].margin;
                    (need / self.gr.g - 1e-9).ceil() * self.gr.g
                };
                let here = shape.centre();
                let crowded = self.board.pads.iter().any(|p| {
                    p.reference == reference
                        && p.layers.contains(&layers[0])
                        && p.net.as_deref() != Some(net.as_str())
                        && {
                            let d = geom::dist(p.shape.centre(), here);
                            d > 1e-6 && d < pitch - 1e-6
                        }
                });
                if (open || together) && !crowded {
                    continue;
                }
                // how deep inside the part it lies: 0 for the outer row or ring
                let deep = part
                    .map(|p| {
                        let c = term.centre;
                        (c.0 - p.bbox.0)
                            .min(p.bbox.2 - c.0)
                            .min(c.1 - p.bbox.1)
                            .min(p.bbox.3 - c.1)
                            .max(0.0)
                    })
                    .unwrap_or(0.0);
                todo.push((reference, (deep / 0.2).round(), j, t, !(open || together)));
            }
        }
        // a part's outer pads first (the ones further in must get past their
        // fan-outs), and each row from one end to the other (so that no pad is
        // boxed in by a neighbour on each side)
        todo.sort_by(|a, b| {
            let (ta, tb) = (&self.jobs[a.2].terminals[a.3], &self.jobs[b.2].terminals[b.3]);
            a.0.cmp(&b.0)
                .then(a.1.partial_cmp(&b.1).unwrap())
                .then(ta.centre.partial_cmp(&tb.centre).unwrap())
                .then(ta.label.cmp(&tb.label))
                .then(a.2.cmp(&b.2))
        });
        for (reference, _, j, t, shut) in todo {
            let label = self.jobs[j].terminals[t].label.clone();
            let net = self.jobs[j].net.clone();
            let part = parts.get(&reference).cloned();
            match self.fan_pad(j, t, part, &placed, &mut flood) {
                Ok((fan, copper)) => {
                    // walls for the pads still to come: what a track of a like net keeps from this
                    let clr = self.rules.class_of(&net).clearance;
                    let m = self.jobs[j].margin;
                    for p in &copper {
                        for l in p.layers.0..=p.layers.1.min(self.gr.nl - 1) {
                            let grow = clr + self.jobs[j].widths[l] / 2.0 + if p.landing { 0.0 } else { m };
                            let layer = &mut flood.walls[l];
                            self.gr.each(&p.shape, grow, |c| layer[c] = 1);
                        }
                    }
                    placed.extend(copper);
                    fans.push(fan);
                }
                // (a pad the grid can reach after all is left to it)
                Err(why) => {
                    if shut {
                        notes.push((label, net, why))
                    }
                }
            }
        }
        (fans, notes)
    }

    /// Whether a track on layer l can get from grid point `start` out from
    /// under the part, past the job's own walls (`block`) and the fan-outs
    /// made so far.
    fn leads_out(&self, fl: &mut Flood, block: &[u8], l: usize, start: usize, part: Option<Part>) -> bool {
        let part = match part {
            Some(p) => p,
            None => return true,
        };
        let (w, h) = (self.gr.w, self.gr.h);
        let out = part.radius + 1.0;
        fl.gen += 1;
        fl.stack.clear();
        if block[start] != 0 || fl.walls[l][start] != 0 {
            return false;
        }
        fl.mark[start] = fl.gen;
        fl.stack.push(start);
        while let Some(c) = fl.stack.pop() {
            if geom::dist(self.gr.xy(c), part.middle) > out {
                return true;
            }
            let (i, j) = (c % w, c / w);
            let go = |t: usize, fl: &mut Flood| {
                if block[t] == 0 && fl.walls[l][t] == 0 && fl.mark[t] != fl.gen {
                    fl.mark[t] = fl.gen;
                    fl.stack.push(t);
                }
            };
            if i > 0 {
                go(c - 1, fl);
            }
            if i + 1 < w {
                go(c + 1, fl);
            }
            if j > 0 {
                go(c - w, fl);
            }
            if j + 1 < h {
                go(c + w, fl);
            }
        }
        false
    }

    /// The clearance between new copper of `net` (a track, or a via) and a
    /// piece of fan-out copper of another net.
    fn fan_clearance(&self, net: &str, via: bool, other: &Placed) -> f64 {
        let me = if via { Item::via(net) } else { Item::track(net) };
        let it = Item {
            net: Some(other.net.as_str()),
            kind: Some(if other.hole.is_some() { "Via" } else { "Track" }),
            shape: Some(&other.shape),
            layer: None,
            paired: None,
        };
        self.rules.clearance(&me, &it)
    }

    /// A way out for the pad that is terminal t of job j, and the copper it
    /// takes up; or why there is none. `part`: the part the pad is of.
    fn fan_pad(
        &mut self,
        j: usize,
        t: usize,
        part: Option<Part>,
        placed: &[Placed],
        fl: &mut Flood,
    ) -> Result<(Fan, Vec<Placed>), String> {
        let (nl, n, g) = (self.gr.nl, self.gr.n, self.gr.g);
        let (shape, layers, _) = self.jobs[j].terminals[t].parts[0].clone();
        let (l0, c) = (layers[0], shape.centre());
        // the way that leads out of the part
        let away = part.and_then(|p| {
            let d = geom::dist(p.middle, c);
            if d > 1e-6 {
                Some(((c.0 - p.middle.0) / d, (c.1 - p.middle.1) / d))
            } else {
                None
            }
        });
        let label = self.jobs[j].terminals[t].label.clone();
        let net = self.jobs[j].net.clone();
        let widths = self.jobs[j].widths.clone();
        let (via, spans, m) = (self.jobs[j].via, self.jobs[j].spans.clone(), self.jobs[j].margin);
        let edge = self.rules.edge_clearance(&net);
        let (block, _) = self.hard(j);
        let room = self.room(j);
        let roomy = room.iter().cloned().max().unwrap_or(0) / 20;
        let reach = self.opt.fanout_length;
        let beside = 1.5 * g;
        let (vd, drill) = via.unwrap_or((0.0, 0.0));
        let vr = vd / 2.0;
        let bb = shape.bbox();
        let big = (bb.0 - reach, bb.1 - reach, bb.2 + reach, bb.3 + reach);
        let near = self.near(j, big, (self.jobs[j].width / 2.0).max(vr) + beside + 0.5);
        // what a via of this net owes each of them
        let via_clr: Vec<f64> = near
            .iter()
            .map(|&(o, _)| {
                let ob = &self.obstacles[o];
                let it = Item {
                    net: ob.net.as_deref(),
                    kind: Some(ob.kind),
                    shape: Some(&ob.shape),
                    layer: None,
                    paired: None,
                };
                self.rules.clearance(&Item::via(&net), &it).max(ob.least)
            })
            .collect();
        // the net's own copper nearby: no via on its pads or its other vias
        let own: Vec<usize> = (0..self.obstacles.len())
            .filter(|&o| {
                let ob = &self.obstacles[o];
                let b = ob.shape.bbox();
                ob.net.as_deref() == Some(net.as_str())
                    && b.0 <= big.2 + vd
                    && b.2 >= big.0 - vd
                    && b.1 <= big.3 + vd
                    && b.3 >= big.1 - vd
            })
            .collect();
        // fan-out copper of other pads within reach, with what is owed to it
        let by: Vec<(&Placed, f64, f64)> = placed
            .iter()
            .filter(|p| {
                let b = p.shape.bbox();
                b.0 <= big.2 + 1.0 && b.2 >= big.0 - 1.0 && b.1 <= big.3 + 1.0 && b.3 >= big.1 - 1.0
            })
            .map(|p| {
                if p.net == net {
                    (p, 0.0, 0.0)
                } else {
                    (p, self.fan_clearance(&net, false, p), self.fan_clearance(&net, true, p))
                }
            })
            .collect();
        let on = |l: usize, (a, b): Span| a <= l && l <= b;
        let keepouts: Vec<&crate::board::Zone> = self.board.zones.iter().filter(|z| z.keepout.is_some()).collect();
        // a piece of track on the pad's layer, clear of everything
        let track_ok = |a: P, b: P, w: f64| -> bool {
            if !self.stub_ok(w / 2.0, a, b, l0, &near, edge) {
                return false;
            }
            for (p, clr, _) in &by {
                if p.net != net && on(l0, p.layers) && geom::seg_shape_dist(a, b, &p.shape) < clr + w / 2.0 - 1e-6 {
                    return false;
                }
            }
            let steps = ((geom::dist(a, b) / (g / 2.0)).ceil() as usize).max(1);
            for z in &keepouts {
                if !z.keepout.as_ref().map(|k| k.tracks).unwrap_or(false) || !z.layers.contains(&l0) {
                    continue;
                }
                for s in 0..=steps {
                    let f = s as f64 / steps as f64;
                    let p = (a.0 + f * (b.0 - a.0), a.1 + f * (b.1 - a.1));
                    if geom::in_polygon(p, &z.outline) || geom::polygon_edge_dist(p, &z.outline) < w / 2.0 {
                        return false;
                    }
                }
            }
            true
        };
        // a grid point the router can carry on from, on layer l: free, with
        // room around it, and far enough from the other fan-outs to stay so
        let landing = |l: usize, x: usize| -> bool {
            if block[l][x] != 0 || room[l * n + x] < roomy {
                return false;
            }
            let p = self.gr.xy(x);
            by.iter().all(|(q, clr, _)| {
                q.net == net || !on(l, q.layers) || q.shape.dist(p) >= clr + widths[l] / 2.0 + m - 1e-6
            })
        };
        // (as far from its own pad as from the others: the middle of four balls)
        let own_gap = (vr + self.rules.class_of(&net).clearance)
            .max(drill / 2.0 + self.rules.hole_clearance)
            .max(vr + MASK);
        // how much a via at e has to spare (below 0: it does not fit). `apart`:
        // its copper is to stay off the net's own surface-mount pads
        let via_spare = |e: P, span: Span, apart: bool| -> f64 {
            if self.outside[self.gr.cell(e)] != 0 {
                return -1.0;
            }
            let mut s = f64::INFINITY;
            for (i, &(o, _)) in near.iter().enumerate() {
                let ob = &self.obstacles[o];
                if ob.layers.iter().any(|&l| on(l, span)) {
                    let need = (via_clr[i] + vr).max(drill / 2.0 + self.rules.hole_clearance);
                    s = s.min(ob.shape.dist(e) - need);
                }
            }
            for &(a, b) in &self.board.edges {
                s = s.min(geom::seg_dist(e, a, b) - (edge + vr));
            }
            for &o in &own {
                let ob = &self.obstacles[o];
                if !ob.layers.iter().any(|&l| on(l, span)) {
                    continue;
                }
                match (ob.kind, ob.layers.len() == nl) {
                    ("Pad", true) | ("Via", _) => s = s.min(ob.shape.dist(e) - vr),
                    ("Pad", false) if !self.opt.via_in_pad => {
                        s = s.min(ob.shape.dist(e) - if apart { own_gap } else { drill / 2.0 + 0.05 })
                    }
                    _ => {}
                }
            }
            for v in &self.board.vias {
                if v.layers.0 <= span.1 && span.0 <= v.layers.1 {
                    s = s.min(geom::dist(e, v.at) - (drill / 2.0 + v.drill / 2.0 + self.rules.hole_to_hole));
                }
            }
            for z in &keepouts {
                if z.keepout.as_ref().map(|k| k.vias).unwrap_or(false) && z.layers.iter().any(|&l| on(l, span)) {
                    if geom::in_polygon(e, &z.outline) {
                        return -1.0;
                    }
                    s = s.min(geom::polygon_edge_dist(e, &z.outline) - vr);
                }
            }
            for (p, _, clr) in &by {
                if p.layers.0 > span.1 || span.0 > p.layers.1 {
                    continue;
                }
                if p.net != net {
                    s = s.min(p.shape.dist(e) - (clr + vr));
                }
                if let Some(d) = p.hole {
                    s = s.min(geom::dist(e, p.shape.centre()) - (drill / 2.0 + d / 2.0 + self.rules.hole_to_hole));
                }
            }
            s
        };
        // how many fan-out vias of the same part stand on layer l between a via
        // at e and the way out of the part
        let in_the_way = |e: P, l: usize| -> usize {
            let part = match part {
                Some(p) => p,
                None => return 0,
            };
            let far = geom::dist(e, part.middle);
            if far < 1e-6 {
                return 0;
            }
            let out = ((e.0 - part.middle.0) / far, (e.1 - part.middle.1) / far);
            // (those made so far, and the board's own: among them the fan-outs of
            // an earlier round)
            placed
                .iter()
                .filter(|p| p.hole.is_some() && on(l, p.layers))
                .map(|p| p.shape.centre())
                .chain(self.board.vias.iter().filter(|v| on(l, v.layers)).map(|v| v.at))
                .filter(|&q| {
                    let d = geom::dist(q, e);
                    let from_middle = geom::dist(q, part.middle);
                    from_middle <= part.radius + 1.0
                        && from_middle > far + 0.1
                        && d > 1e-6
                        && ((q.0 - e.0) * out.0 + (q.1 - e.1) * out.1) / d >= 0.5
                })
                .count()
        };
        // where the net might go on from a via at e: layers and grid points,
        // the nearest few on each layer
        let exits = |e: P, span: Span| -> Vec<(usize, usize)> {
            let mut found = Vec::new();
            for l in (span.0..=span.1).filter(|&l| l != l0 && self.layers_ok(l)) {
                let mut cells: Vec<(f64, usize)> = Vec::new();
                self.gr.each(&Shape::Circle { c: e, r: 0.0 }, vr.max(beside), |x| {
                    cells.push((0.0, x));
                });
                for x in cells.iter_mut() {
                    x.0 = geom::dist(self.gr.xy(x.1), e);
                }
                cells.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
                let mut k = 0;
                for (_, x) in cells {
                    if k < 3 && landing(l, x) && self.stub_ok(widths[l] / 2.0, self.gr.xy(x), e, l, &near, edge) {
                        found.push((l, x));
                        k += 1;
                    }
                }
            }
            found
        };

        // the ways to try, nearest and most outward first
        let base = match shape {
            Shape::Box { deg, .. } => deg.to_radians(),
            Shape::Capsule { a, b, .. } => (b.1 - a.1).atan2(b.0 - a.0),
            Shape::Circle { .. } => 0.0,
        };
        let dirs: Vec<P> = (0..16)
            .map(|k| {
                let a = base + k as f64 * std::f64::consts::PI / 8.0;
                (a.cos(), a.sin())
            })
            .collect();
        let steps = (reach / STEP).round() as usize;
        // (cost, with a via, direction, steps out)
        let mut tries: Vec<(f64, bool, usize, usize)> = Vec::new();
        for (d, u) in dirs.iter().enumerate() {
            let turn = away.map(|a| (1.0 - (u.0 * a.0 + u.1 * a.1)) / 2.0).unwrap_or(0.0);
            for s in 1..=steps {
                let cost = s as f64 * STEP + turn;
                tries.push((cost, false, d, s));
                if via.is_some() {
                    tries.push((cost + VIA_COST, true, d, s));
                }
            }
        }
        if via.is_some() && self.opt.via_in_pad {
            tries.push((VIA_COST, true, 0, 0));
        }
        tries.sort_by(|a, b| a.partial_cmp(b).unwrap());
        // full width first; a narrower piece where the net may neck down
        let mut ws = vec![widths[l0]];
        if let Some(nw) = self.jobs[j].neck.filter(|nw| *nw < widths[l0] - 1e-9) {
            ws.push(nw);
        }
        // how far the best try with a via got: 1 the via fits, 2 a track
        // reaches it, 3 there is a free grid point by it on another layer
        let mut got = 0;
        // first with every via's copper clear of the net's own pads; failing
        // that, with only its hole off them
        let rounds: &[bool] = if via.is_some() && !self.opt.via_in_pad {
            &[true, false]
        } else {
            &[false]
        };
        for (round, &apart) in rounds.iter().enumerate() {
            for &(_, with_via, d, s) in &tries {
                let u = dirs[d];
                let len = s as f64 * STEP;
                let e = (c.0 + len * u.0, c.1 + len * u.1);
                if !with_via && round > 0 {
                    continue; // (tried already)
                }
                if !with_via {
                    for &w in &ws {
                        if !track_ok(c, e, w) {
                            continue;
                        }
                        let mut cells: Vec<(f64, usize)> = Vec::new();
                        self.gr
                            .each(&Shape::Circle { c: e, r: 0.0 }, beside, |x| cells.push((0.0, x)));
                        for x in cells.iter_mut() {
                            x.0 = geom::dist(self.gr.xy(x.1), e);
                        }
                        cells.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
                        for (far, x) in cells {
                            let p = self.gr.xy(x);
                            if !landing(l0, x)
                                || (far > 1e-9 && !track_ok(e, p, w))
                                || !self.leads_out(fl, &block[l0], l0, x, part)
                            {
                                continue;
                            }
                            let mut tracks = vec![(c, e, w)];
                            if far > 1e-9 {
                                tracks.push((e, p, w));
                            }
                            let mut copper: Vec<Placed> = tracks
                                .iter()
                                .map(|&(a, b, w)| Placed {
                                    net: net.clone(),
                                    shape: Shape::Capsule { a, b, r: w / 2.0 },
                                    layers: (l0, l0),
                                    hole: None,
                                    landing: false,
                                })
                                .collect();
                            // (and the room the router's own track needs at the landing)
                            copper.push(Placed {
                                net: net.clone(),
                                shape: Shape::Circle {
                                    c: p,
                                    r: widths[l0] / 2.0 + m,
                                },
                                layers: (l0, l0),
                                hole: None,
                                landing: true,
                            });
                            return Ok((
                                Fan {
                                    net,
                                    pad: label,
                                    layer: l0,
                                    tracks,
                                    via: None,
                                    end: p,
                                },
                                copper,
                            ));
                        }
                    }
                    continue;
                }
                // each span the via may have, and each layer the net could go on from
                // on: (vias in its way out on that layer, span, via, track width, layer, grid point)
                let mut ways: Vec<(usize, usize, P, f64, usize, usize)> = Vec::new();
                for (si, &span) in spans.iter().enumerate().filter(|(_, sp)| on(l0, **sp)) {
                    let spots: Vec<P> = if via_spare(e, span, apart) < 0.0 {
                        Vec::new()
                    } else {
                        // nudged to where it has the most to spare (the middle of four balls)
                        let mut at = e;
                        let mut spare = via_spare(at, span, apart);
                        for nudge in [0.02, 0.01, 0.005] {
                            let mut moved = true;
                            while moved && spare < SPARE {
                                moved = false;
                                for k in 0..8 {
                                    let a = k as f64 * std::f64::consts::FRAC_PI_4;
                                    let q = (at.0 + nudge * a.cos(), at.1 + nudge * a.sin());
                                    let sq = via_spare(q, span, apart);
                                    if sq > spare + 1e-9 {
                                        at = q;
                                        spare = sq;
                                        moved = true;
                                    }
                                }
                            }
                        }
                        vec![at, e]
                    };
                    if spots.is_empty() {
                        continue;
                    }
                    got = got.max(1);
                    for at in spots {
                        let far = geom::dist(c, at);
                        let w = match ws.iter().find(|&&w| far < 1e-9 || track_ok(c, at, w)) {
                            Some(w) => *w,
                            None => continue,
                        };
                        got = got.max(2);
                        let found = exits(at, span);
                        if found.is_empty() {
                            continue;
                        }
                        got = got.max(3);
                        for (l, x) in found {
                            ways.push((in_the_way(at, l), si, at, w, l, x));
                        }
                        break;
                    }
                }
                // the layer with the fewest vias between it and the way out first: so the
                // rings of a BGA go one to a layer, where the vias may be blind
                ways.sort_by(|a, b| (a.0, a.1, a.4, a.5).cmp(&(b.0, b.1, b.4, b.5)));
                for (_, si, at, w, xl, x) in ways {
                    if !self.leads_out(fl, &block[xl], xl, x, part) {
                        continue;
                    }
                    let span = spans[si];
                    let far = geom::dist(c, at);
                    let tracks = if far > 1e-9 { vec![(c, at, w)] } else { Vec::new() };
                    let mut copper: Vec<Placed> = tracks
                        .iter()
                        .map(|&(a, b, w)| Placed {
                            net: net.clone(),
                            shape: Shape::Capsule { a, b, r: w / 2.0 },
                            layers: (l0, l0),
                            hole: None,
                            landing: false,
                        })
                        .collect();
                    copper.push(Placed {
                        net: net.clone(),
                        shape: Shape::Circle { c: at, r: vr },
                        layers: span,
                        hole: Some(drill),
                        landing: false,
                    });
                    copper.push(Placed {
                        net: net.clone(),
                        shape: Shape::Circle {
                            c: self.gr.xy(x),
                            r: widths[xl] / 2.0 + m,
                        },
                        layers: (xl, xl),
                        hole: None,
                        landing: true,
                    });
                    return Ok((
                        Fan {
                            net,
                            pad: label,
                            layer: l0,
                            tracks,
                            via: Some((at, vd, drill, span)),
                            end: at,
                        },
                        copper,
                    ));
                }
            }
        }
        let own = format!("no way out on its own layer within {} mm", reach);
        Err(match (via, got) {
            (None, _) => format!("{}, and the net may have no vias", own),
            (Some(_), 0) => format!("{}, and no room for a {} mm via beside it", own, vd),
            (Some(_), 1) => format!("{}, and no room for a track to where a via would fit", own),
            (Some(_), 2) => format!("{}, and no free grid point by a via on another layer", own),
            _ => format!(
                "{}, and the layers a via reaches are shut too, by the pads and vias around it",
                own
            ),
        })
    }
}
