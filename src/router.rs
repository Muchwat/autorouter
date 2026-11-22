//! The router: a grid A* per connection inside a negotiated-congestion loop.
//!
//! Every net is routed; the copper of other routed nets is a cost, not a wall.
//! A cell wanted by more than one net gets dearer each pass (the present
//! cost) and remembers that it was fought over (the history cost), and only
//! the nets still in conflict are routed again. When the passes run out or
//! stop helping, any net still in conflict is routed once more with the
//! others as walls, so what comes back never breaks a clearance the router
//! knows about. Walls from the start: the board edge and keep-outs, and other
//! nets' pads, holes and existing copper.

use crate::board::Board;
use crate::geom::{self, Shape, P};
use crate::rules::{Item, Rules};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

#[derive(Clone, Debug)]
pub struct Options {
    /// Grid pitch in mm.
    pub grid: f64,
    /// Extra gap added to every clearance, for the grid's corners.
    pub margin: f64,
    pub passes: usize,
    /// Stop bargaining after this many passes without fewer cells in conflict.
    pub stall: usize,
    /// Cost of a via, in grid steps.
    pub via_cost: f64,
    /// Search effort per connection (cells expanded) before giving up on it.
    pub limit: usize,
    /// Route only these nets (wildcards allowed).
    pub nets: Option<Vec<String>>,
    /// Never route these nets (wildcards allowed).
    pub skip: Vec<String>,
    /// Also route nets that have a copper pour (they are left to the pour by default).
    pub include_poured: bool,
    /// Copper layers the router may use (indices into Board::copper); None: all.
    pub layers: Option<Vec<usize>>,
    /// Override every net's track width.
    pub track_width: Option<f64>,
    /// Override every net's via (diameter, drill).
    pub via: Option<(f64, f64)>,
    /// No vias at all.
    pub no_vias: bool,
    /// Let a via sit on a surface-mount pad of its own net (kept off by default:
    /// an open hole in a pad drains the solder joint).
    pub via_in_pad: bool,
    /// Where a track is too wide to reach a pad, finish with a straight piece
    /// this wide (None: the Default class's width), no thinner than the rules allow.
    pub neck: Option<f64>,
    /// The longest such narrow piece, in mm (0: never narrow).
    pub neck_length: f64,
    /// Tracks this wide or wider get a second via beside each via, to share the
    /// current (0: never).
    pub double_vias: f64,
    /// Also lay tracks on inner layers that a pour covers (by default such a
    /// layer is a plane, and stays whole).
    pub tracks_on_planes: bool,
    /// Print progress to stderr.
    pub progress: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            grid: 0.25,
            margin: 0.05,
            passes: 40,
            stall: 5,
            via_cost: 12.0,
            limit: 2_000_000,
            nets: None,
            skip: Vec::new(),
            include_poured: false,
            layers: None,
            track_width: None,
            via: None,
            no_vias: false,
            via_in_pad: false,
            neck: None,
            neck_length: 3.0,
            double_vias: 0.8,
            tracks_on_planes: false,
            progress: false,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct NewTrack {
    pub net: String,
    pub layer: usize,
    pub a: P,
    pub b: P,
    pub width: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NewVia {
    pub net: String,
    pub at: P,
    pub diameter: f64,
    pub drill: f64,
}

#[derive(Clone, Debug)]
pub struct NetReport {
    pub net: String,
    pub width: f64,
    /// Pads reached with a narrowed piece of track, and its width.
    pub necked: Vec<(String, f64)>,
    /// Separate pieces to join (pads, or pads already joined by copper).
    pub pieces: usize,
    /// Pieces left unjoined, by a pad in each.
    pub unrouted: Vec<String>,
}

#[derive(Clone, Debug, Default)]
pub struct Routed {
    pub tracks: Vec<NewTrack>,
    pub vias: Vec<NewVia>,
    pub nets: Vec<NetReport>,
    pub passes: usize,
    /// Nets left alone, and why.
    pub skipped: Vec<(String, String)>,
    /// Second vias put beside the vias of wide tracks.
    pub doubled: usize,
    /// Vias of wide tracks with no room for a second one: (net, where).
    pub single: Vec<(String, P)>,
    /// Things the user should know about the board or the result.
    pub warnings: Vec<String>,
    pub log: Vec<String>,
}

impl Routed {
    pub fn complete(&self) -> bool {
        self.nets.iter().all(|n| n.unrouted.is_empty())
    }
}

// ------------------------------------------------------------------ grid

struct Grid {
    x0: f64,
    y0: f64,
    g: f64,
    w: usize,
    h: usize,
    n: usize,
    nl: usize,
}

impl Grid {
    fn xy(&self, idx: usize) -> P {
        (
            self.x0 + (idx % self.w) as f64 * self.g,
            self.y0 + (idx / self.w) as f64 * self.g,
        )
    }

    fn cell(&self, p: P) -> usize {
        let i = ((p.0 - self.x0) / self.g).round().clamp(0.0, (self.w - 1) as f64) as usize;
        let j = ((p.1 - self.y0) / self.g).round().clamp(0.0, (self.h - 1) as f64) as usize;
        j * self.w + i
    }

    fn range(&self, bb: (f64, f64, f64, f64), r: f64) -> (usize, usize, usize, usize) {
        let i0 = (((bb.0 - r - self.x0) / self.g).floor().max(0.0)) as usize;
        let j0 = (((bb.1 - r - self.y0) / self.g).floor().max(0.0)) as usize;
        let i1 = (((bb.2 + r - self.x0) / self.g).ceil().max(0.0) as usize).min(self.w - 1);
        let j1 = (((bb.3 + r - self.y0) / self.g).ceil().max(0.0) as usize).min(self.h - 1);
        (i0, j0, i1, j1)
    }

    /// Call f(cell) for every cell within r of the shape.
    fn each(&self, shape: &Shape, r: f64, mut f: impl FnMut(usize)) {
        let (i0, j0, i1, j1) = self.range(shape.bbox(), r);
        if i0 > i1 || j0 > j1 {
            return;
        }
        for j in j0..=j1 {
            let y = self.y0 + j as f64 * self.g;
            for i in i0..=i1 {
                let x = self.x0 + i as f64 * self.g;
                if shape.dist((x, y)) <= r {
                    f(j * self.w + i);
                }
            }
        }
    }
}

// --------------------------------------------------------------- inputs

struct Obstacle {
    net: Option<String>,
    kind: &'static str,
    shape: Shape,
    layers: Vec<usize>,
    /// A clearance this item is owed whatever the nets (a drilled hole's own rule).
    least: f64,
    /// Obstacles of one group are alike to every rule: nets the rules treat
    /// alike (same class, same answers to every net-name test), the same kind,
    /// and the same answers to the rules' tests of place.
    group: usize,
}

struct Terminal {
    label: String,
    centre: P,
    /// Shapes of this piece's copper: (shape, layers, is a pad).
    parts: Vec<(Shape, Vec<usize>, bool)>,
}

struct Job {
    net: String,
    width: f64,
    via: Option<(f64, f64)>,
    /// Extra gap on every clearance of this job: walls and conflicts are tested
    /// at grid points, and a track between two points can pass this much nearer.
    margin: f64,
    /// The width a track may narrow to at a pad it cannot otherwise reach.
    neck: Option<f64>,
    terminals: Vec<Terminal>,
    sig: usize,
    span: f64,
}

/// A cell a job's track may end on to reach a piece: its A* state and, when
/// the track is to be finished at an exact point (a pad's centre, the loose
/// end of a track), that point and the width of the last piece.
type Entry2 = (u32, Option<(P, f64)>);

/// The gap the grid costs: a track between two grid points that are each at
/// least r from some copper passes no nearer than sqrt(r^2 - (step / 2)^2),
/// so r has to be this much more than the clearance wanted. It also keeps two
/// tracks from crossing between each other's grid points.
fn grid_margin(r0: f64, grid: f64) -> f64 {
    (r0 * r0 + grid * grid / 2.0).sqrt() - r0
}

/// KiCad joins two items of a net where an anchor of one (a track's ends, the
/// centre of a via or pad) lies on the other's copper; overlapping is not enough.
fn joined(a: &(Shape, Vec<usize>, bool, String), b: &(Shape, Vec<usize>, bool, String)) -> bool {
    fn anchors(s: &Shape, is_pad: bool) -> Vec<P> {
        match *s {
            Shape::Capsule { a, b, .. } if !is_pad => vec![a, b],
            _ => vec![s.centre()],
        }
    }
    if !a.1.iter().any(|l| b.1.contains(l)) {
        return false;
    }
    anchors(&a.0, a.2).iter().any(|p| b.0.dist(*p) <= 1e-6) || anchors(&b.0, b.2).iter().any(|p| a.0.dist(*p) <= 1e-6)
}

/// One net's routing, as A* states (layer * n + cell) and copper.
#[derive(Default, Clone)]
struct Laid {
    paths: Vec<Vec<u32>>,
    /// (layer, points, width)
    tracks: Vec<(usize, Vec<P>, f64)>,
    vias: Vec<P>,
    unrouted: Vec<String>,
    necked: Vec<(String, f64)>,
}

// ------------------------------------------------------------------- A*

#[derive(PartialEq)]
struct Entry(f64, f64, u32);

impl Eq for Entry {}

impl Ord for Entry {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.partial_cmp(&self.0)
            .unwrap_or(Ordering::Equal)
            .then_with(|| o.1.partial_cmp(&self.1).unwrap_or(Ordering::Equal))
            .then_with(|| o.2.cmp(&self.2))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

struct Work {
    best: Vec<f64>,
    came: Vec<u32>,
    seen: Vec<u32>,
    target: Vec<u32>,
    gen: u32,
    heap: BinaryHeap<Entry>,
    mark: Vec<u32>,
    mgen: u32,
}

impl Work {
    fn new(states: usize) -> Work {
        Work {
            best: vec![0.0; states],
            came: vec![u32::MAX; states],
            seen: vec![0; states],
            target: vec![0; states],
            gen: 0,
            heap: BinaryHeap::new(),
            mark: vec![0; states],
            mgen: 0,
        }
    }

    fn next_gen(&mut self) -> u32 {
        self.gen = self.gen.wrapping_add(1);
        if self.gen == 0 {
            self.seen.iter_mut().for_each(|v| *v = 0);
            self.target.iter_mut().for_each(|v| *v = 0);
            self.gen = 1;
        }
        self.gen
    }

    fn next_mark(&mut self) -> u32 {
        self.mgen = self.mgen.wrapping_add(1);
        if self.mgen == 0 {
            self.mark.iter_mut().for_each(|v| *v = 0);
            self.mgen = 1;
        }
        self.mgen
    }
}

const SQRT2: f64 = std::f64::consts::SQRT_2;
const DIRS: [(i64, i64, f64); 8] = [
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, SQRT2),
    (1, -1, SQRT2),
    (-1, 1, SQRT2),
    (-1, -1, SQRT2),
];

struct Search<'a> {
    block: &'a [Vec<u8>],
    vblock: Option<&'a [u8]>,
    soft: &'a [Vec<u16>],
    soft_via: &'a [u32],
    hist: &'a [Vec<f32>],
    pres: f64,
    via_cost: f64,
    limit: usize,
}

fn astar(gr: &Grid, wk: &mut Work, s: &Search, sources: &[u32], targets: &[u32], goal: P) -> Option<Vec<u32>> {
    let (w, h, n, nl) = (gr.w as i64, gr.h as i64, gr.n, gr.nl);
    let gen = wk.next_gen();
    wk.heap.clear();
    for &t in targets {
        wk.target[t as usize] = gen;
    }
    let gi = ((goal.0 - gr.x0) / gr.g).round() as i64;
    let gj = ((goal.1 - gr.y0) / gr.g).round() as i64;
    let heur = |st: u32| -> f64 {
        let idx = (st as usize % n) as i64;
        let dx = (idx % w - gi).abs();
        let dy = (idx / w - gj).abs();
        (dx + dy) as f64 - (2.0 - SQRT2) * dx.min(dy) as f64
    };
    for &st in sources {
        wk.seen[st as usize] = gen;
        wk.best[st as usize] = 0.0;
        wk.came[st as usize] = u32::MAX;
        wk.heap.push(Entry(heur(st), 0.0, st));
    }
    let mut expanded = 0usize;
    while let Some(Entry(_, gc, st)) = wk.heap.pop() {
        let su = st as usize;
        if wk.seen[su] == gen && gc > wk.best[su] {
            continue;
        }
        if wk.target[su] == gen {
            let mut path = vec![st];
            let mut cur = st;
            while wk.came[cur as usize] != u32::MAX {
                cur = wk.came[cur as usize];
                path.push(cur);
            }
            path.reverse();
            return Some(path);
        }
        expanded += 1;
        if expanded > s.limit {
            return None;
        }
        let l = su / n;
        let idx = su % n;
        let (i, j) = ((idx as i64) % w, (idx as i64) / w);
        let (m, o, hl) = (&s.block[l], &s.soft[l], &s.hist[l]);
        let relax = |wk: &mut Work, ns: usize, ng: f64| {
            if wk.seen[ns] != gen || ng < wk.best[ns] {
                wk.seen[ns] = gen;
                wk.best[ns] = ng;
                wk.came[ns] = st;
                wk.heap.push(Entry(ng + heur(ns as u32), ng, ns as u32));
            }
        };
        for &(di, dj, c) in DIRS.iter() {
            let (ni, nj) = (i + di, j + dj);
            if ni < 0 || ni >= w || nj < 0 || nj >= h {
                continue;
            }
            let nidx = (nj * w + ni) as usize;
            if m[nidx] != 0 {
                continue;
            }
            if di != 0 && dj != 0 && (m[(j * w + ni) as usize] != 0 || m[(nj * w + i) as usize] != 0) {
                continue; // no squeezing between two blocked cells
            }
            let ng = gc + c * (1.0 + hl[nidx] as f64) * (1.0 + s.pres * o[nidx] as f64);
            relax(wk, l * n + nidx, ng);
        }
        if let Some(vb) = s.vblock {
            if vb[idx] == 0 {
                let hsum: f64 = (0..nl).map(|k| s.hist[k][idx] as f64).sum();
                let ng = gc + s.via_cost * (1.0 + hsum) * (1.0 + s.pres * s.soft_via[idx] as f64);
                for l2 in 0..nl {
                    if l2 != l && s.block[l2][idx] == 0 {
                        relax(wk, l2 * n + idx, ng);
                    }
                }
            }
        }
    }
    None
}

// --------------------------------------------------------------- router

struct Router<'a> {
    board: &'a Board,
    rules: &'a Rules,
    opt: &'a Options,
    gr: Grid,
    obstacles: Vec<Obstacle>,
    outside: Vec<u8>,
    jobs: Vec<Job>,
    hard: Vec<Option<(Vec<Vec<u8>>, Option<Vec<u8>>)>>,
    /// job -> terminal -> the cells a track may end on
    entries: Vec<Option<Vec<Vec<Entry2>>>>,
    /// job -> state -> size of the free region it is in
    rooms: Vec<Option<std::rc::Rc<Vec<u32>>>>,
    /// job -> obstacle -> the clearance the job's tracks owe it (worked out once, in hard())
    owed: Vec<Vec<f32>>,
    laid: Vec<Option<Laid>>,
    /// job -> radius (um) -> cells per layer
    contrib: Vec<HashMap<i64, Vec<Vec<u32>>>>,
    /// (signature, radius um) -> per-layer counts of that signature's routed copper
    occ: HashMap<(usize, i64), Vec<Vec<u8>>>,
    hist: Vec<Vec<f32>>,
    work: Work,
    clr_cache: HashMap<(String, &'static str, Option<String>, &'static str, Option<u64>), f64>,
    cache_ok: bool,
    ngroups: usize,
    /// Inner layers given over to a pour: no tracks there, so the plane stays whole.
    planes: Vec<usize>,
    log: Vec<String>,
}

fn um(r: f64) -> i64 {
    (r * 1000.0).round() as i64
}

fn matches(list: &[String], net: &str) -> bool {
    list.iter().any(|p| p == net || crate::rules::wildcard(p, net))
}

impl<'a> Router<'a> {
    fn say(&mut self, s: String) {
        if self.opt.progress {
            eprintln!("{}", s);
        }
        self.log.push(s);
    }

    /// The clearance a owes b, remembered by net and kind and by what the
    /// rules' tests of place say of b (a, new copper, has no place yet).
    fn clearance(&mut self, a: Item, b: Item) -> f64 {
        if !self.cache_ok {
            return self.rules.clearance(&a, &b);
        }
        let key = (
            a.net.unwrap_or("").to_string(),
            a.kind.unwrap_or(""),
            b.net.map(|s| s.to_string()),
            b.kind.unwrap_or(""),
            b.shape.map(|s| self.rules.place(s)),
        );
        if let Some(v) = self.clr_cache.get(&key) {
            return *v;
        }
        let v = self.rules.clearance(&a, &b);
        self.clr_cache.insert(key, v);
        v
    }

    fn layers_ok(&self, l: usize) -> bool {
        !self.planes.contains(&l) && self.opt.layers.as_ref().map(|v| v.contains(&l)).unwrap_or(true)
    }

    /// Walls for job j: (per-layer track walls, via walls).
    fn hard(&mut self, j: usize) -> (Vec<Vec<u8>>, Option<Vec<u8>>) {
        if let Some(h) = &self.hard[j] {
            return h.clone();
        }
        let (nl, n) = (self.gr.nl, self.gr.n);
        let net = self.jobs[j].net.clone();
        let hw = self.jobs[j].width / 2.0;
        let vr = self.jobs[j].via.map(|v| v.0 / 2.0);
        let m = self.jobs[j].margin;
        let mut block: Vec<Vec<u8>> = (0..nl).map(|_| self.outside.clone()).collect();
        let mut vblock: Option<Vec<u8>> = vr.map(|_| self.outside.clone());
        for l in 0..nl {
            if !self.layers_ok(l) {
                block[l] = vec![1; n];
            }
        }
        let ec = self.rules.edge_clearance(&net);
        for &(a, b) in &self.board.edges {
            let s = Shape::Capsule { a, b, r: 0.0 };
            for l in 0..nl {
                let bl = &mut block[l];
                self.gr.each(&s, ec + hw + m, |c| bl[c] = 1);
            }
            if let (Some(vb), Some(vr)) = (vblock.as_mut(), vr) {
                self.gr.each(&s, ec + vr + m, |c| vb[c] = 1);
            }
        }
        for z in &self.board.zones {
            let k = match &z.keepout {
                Some(k) => k,
                None => continue,
            };
            let (mut x0, mut y0, mut x1, mut y1) = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for p in &z.outline {
                x0 = x0.min(p.0);
                y0 = y0.min(p.1);
                x1 = x1.max(p.0);
                y1 = y1.max(p.1);
            }
            let reach = hw.max(vr.unwrap_or(0.0)) + m;
            let (i0, j0, i1, j1) = self.gr.range((x0, y0, x1, y1), reach);
            for jj in j0..=j1 {
                for ii in i0..=i1 {
                    let c = jj * self.gr.w + ii;
                    let p = self.gr.xy(c);
                    let inside = geom::in_polygon(p, &z.outline);
                    let d = if inside {
                        0.0
                    } else {
                        geom::polygon_edge_dist(p, &z.outline)
                    };
                    if k.tracks && d <= hw + m {
                        for &l in &z.layers {
                            block[l][c] = 1;
                        }
                    }
                    if let (true, Some(vb), Some(vr)) = (k.vias, vblock.as_mut(), vr) {
                        if d <= vr + m {
                            vb[c] = 1;
                        }
                    }
                }
            }
        }
        let mut owed = vec![0f32; self.obstacles.len()];
        // the rules are asked once for each group of like obstacles: (for a track, for a via)
        let mut asked: Vec<Option<(f64, f64)>> = vec![None; self.ngroups];
        for (o, ob) in self.obstacles.iter().enumerate() {
            if ob.net.as_deref() == Some(net.as_str()) {
                continue;
            }
            let (ct, cv) = match asked[ob.group] {
                Some(v) => v,
                None => {
                    let other = Item {
                        net: ob.net.as_deref(),
                        kind: Some(ob.kind),
                        shape: Some(&ob.shape),
                    };
                    let v = (
                        self.rules.clearance(&Item::track(&net), &other),
                        if vr.is_some() {
                            self.rules.clearance(&Item::via(&net), &other)
                        } else {
                            0.0
                        },
                    );
                    asked[ob.group] = Some(v);
                    v
                }
            };
            let clr = ct.max(ob.least);
            owed[o] = clr as f32;
            let rt = clr + hw + m;
            for &l in &ob.layers {
                let bl = &mut block[l];
                self.gr.each(&ob.shape, rt, |c| bl[c] = 1);
            }
            if let (Some(vr), Some(vb)) = (vr, vblock.as_mut()) {
                self.gr.each(&ob.shape, cv.max(ob.least) + vr + m, |c| vb[c] = 1);
            }
        }
        self.owed[j] = owed;
        // vias and the net's own copper: none in a through-hole pad or on another
        // via (two holes too close), and none on a surface-mount pad unless asked
        if let (Some(vb), Some((vd, drill))) = (vblock.as_mut(), self.jobs[j].via) {
            for t in &self.jobs[j].terminals {
                for (shape, layers, is_pad) in &t.parts {
                    let through = layers.len() == nl;
                    let r = match (*is_pad, through, shape) {
                        (true, true, _) => vd / 2.0,
                        (true, false, _) if !self.opt.via_in_pad => drill / 2.0 + 0.05,
                        (false, true, Shape::Circle { .. }) => vd / 2.0,
                        _ => continue,
                    };
                    self.gr.each(shape, r, |c| vb[c] = 1);
                }
            }
        }
        self.hard[j] = Some((block.clone(), vblock.clone()));
        (block, vblock)
    }

    /// Whether a track of job j from a to b on layer l keeps every clearance to
    /// the copper in `near` and to the board edge, measured exactly.
    fn stub_ok(&self, hw: f64, a: P, b: P, l: usize, near: &[(usize, f64)], edge: f64) -> bool {
        let (x0, y0, x1, y1) = (a.0.min(b.0), a.1.min(b.1), a.0.max(b.0), a.1.max(b.1));
        for &(o, clr) in near {
            let ob = &self.obstacles[o];
            if !ob.layers.contains(&l) {
                continue;
            }
            let need = clr + hw - 1e-6;
            let bb = ob.shape.bbox();
            if bb.0 - x1 > need || x0 - bb.2 > need || bb.1 - y1 > need || y0 - bb.3 > need {
                continue;
            }
            if geom::seg_shape_dist(a, b, &ob.shape) < need {
                return false;
            }
        }
        let need = edge + hw - 1e-6;
        self.board.edges.iter().all(|&(c, d)| {
            (c.0.min(d.0) - x1 > need
                || x0 - c.0.max(d.0) > need
                || c.1.min(d.1) - y1 > need
                || y0 - c.1.max(d.1) > need)
                || geom::seg_seg_dist(a, b, c, d) >= need
        })
    }

    /// Other nets' copper within `reach` of a shape, with the clearance job j owes each.
    fn near(&mut self, j: usize, bbox: (f64, f64, f64, f64), reach: f64) -> Vec<(usize, f64)> {
        if self.owed[j].is_empty() {
            self.hard(j);
        }
        let net = self.jobs[j].net.as_str();
        let owed = &self.owed[j];
        let mut near: Vec<(usize, f64)> = Vec::new();
        for (o, ob) in self.obstacles.iter().enumerate() {
            if ob.net.as_deref() == Some(net) {
                continue;
            }
            let bb = ob.shape.bbox();
            let gap = (bb.0 - bbox.2).max(bbox.0 - bb.2).max(bb.1 - bbox.3).max(bbox.1 - bb.3);
            let clr = owed[o] as f64;
            if gap <= clr + reach + 0.01 {
                near.push((o, clr));
            }
        }
        near
    }

    /// For each state (layer * n + cell) a track of job j may be on, the size of
    /// the free region it lies in: the states it can walk to, vias included. A
    /// small number is a pocket, shut in by other nets' pads and copper.
    fn room(&mut self, j: usize) -> std::rc::Rc<Vec<u32>> {
        if let Some(r) = &self.rooms[j] {
            return r.clone();
        }
        let (nl, n, w, h) = (self.gr.nl, self.gr.n, self.gr.w, self.gr.h);
        let (block, vblock) = self.hard(j);
        let mut label = vec![u32::MAX; n * nl];
        let mut sizes: Vec<u32> = Vec::new();
        let mut stack: Vec<usize> = Vec::new();
        for start in 0..n * nl {
            if label[start] != u32::MAX || block[start / n][start % n] != 0 {
                continue;
            }
            let id = sizes.len() as u32;
            let mut size = 0u32;
            label[start] = id;
            stack.push(start);
            while let Some(s) = stack.pop() {
                size += 1;
                let (l, c) = (s / n, s % n);
                let (i, jj) = (c % w, c / w);
                let go = |t: usize, label: &mut Vec<u32>, stack: &mut Vec<usize>| {
                    if label[t] == u32::MAX && block[t / n][t % n] == 0 {
                        label[t] = id;
                        stack.push(t);
                    }
                };
                if i > 0 {
                    go(s - 1, &mut label, &mut stack);
                }
                if i + 1 < w {
                    go(s + 1, &mut label, &mut stack);
                }
                if jj > 0 {
                    go(s - w, &mut label, &mut stack);
                }
                if jj + 1 < h {
                    go(s + w, &mut label, &mut stack);
                }
                if vblock.as_ref().map(|v| v[c] == 0).unwrap_or(false) {
                    for l2 in 0..nl {
                        if l2 != l {
                            go(l2 * n + c, &mut label, &mut stack);
                        }
                    }
                }
            }
            sizes.push(size);
        }
        let room: Vec<u32> = label
            .iter()
            .map(|&id| if id == u32::MAX { 0 } else { sizes[id as usize] })
            .collect();
        let room = std::rc::Rc::new(room);
        self.rooms[j] = Some(room.clone());
        room
    }

    /// The cells job j's tracks may end on, per terminal.
    ///
    /// On a pad: a free cell on it (or, for a pad no free grid point falls on,
    /// just beside it) from which a track to the pad's centre keeps every
    /// clearance; the track is finished there, since KiCad joins a track to a
    /// pad at its centre or where the track ends on it. Where the track is too
    /// wide to get that near, or gets there only inside a pocket it cannot
    /// leave, also the nearest cells with room around them from which a
    /// straight, narrower piece reaches the centre.
    ///
    /// On existing copper: where a track of the piece has a loose end (a
    /// fan-out, a half-drawn track), at that end, so it is not left dangling;
    /// otherwise any free cell on the copper.
    fn entries(&mut self, j: usize) -> Vec<Vec<Entry2>> {
        if let Some(e) = &self.entries[j] {
            return e.clone();
        }
        let (n, g) = (self.gr.n, self.gr.g);
        let (block, _) = self.hard(j);
        let net = self.jobs[j].net.clone();
        let (width, neck) = (self.jobs[j].width, self.jobs[j].neck);
        let hw = width / 2.0;
        let edge = self.rules.edge_clearance(&net);
        let beside = 1.5 * g;
        let neck_len = if neck.is_some() { self.opt.neck_length } else { 0.0 };
        let room = self.room(j);
        let roomy = room.iter().cloned().max().unwrap_or(0) / 20; // a twentieth of the largest region
        let mut all: Vec<Vec<Entry2>> = Vec::new();
        for t in 0..self.jobs[j].terminals.len() {
            let parts = self.jobs[j].terminals[t].parts.clone();
            // loose ends of the piece's tracks: (point, layer, track width)
            let mut loose: Vec<(P, usize, f64)> = Vec::new();
            for (i, (shape, layers, is_pad)) in parts.iter().enumerate() {
                if let (Shape::Capsule { a, b, r }, false) = (shape, *is_pad) {
                    for end in [*a, *b] {
                        let held = parts.iter().enumerate().any(|(k, (other, ol, _))| {
                            k != i && layers.iter().any(|l| ol.contains(l)) && other.dist(end) <= 1e-6
                        });
                        if !held {
                            loose.push((end, layers[0], 2.0 * r));
                        }
                    }
                }
            }
            let mut st: Vec<Entry2> = Vec::new();
            for &(end, l, _) in &loose {
                if !self.layers_ok(l) {
                    continue;
                }
                let near = self.near(j, (end.0, end.1, end.0, end.1), hw + beside);
                let mut cells: Vec<usize> = Vec::new();
                let bl = &block[l];
                self.gr.each(&Shape::Circle { c: end, r: 0.0 }, beside, |c| {
                    if bl[c] == 0 {
                        cells.push(c);
                    }
                });
                for c in cells {
                    if self.stub_ok(hw, self.gr.xy(c), end, l, &near, edge) {
                        st.push(((l * n + c) as u32, Some((end, width))));
                    }
                }
            }
            if !st.is_empty() {
                st.sort_by(|a, b| a.0.cmp(&b.0));
                st.dedup_by(|a, b| a.0 == b.0);
                all.push(st);
                continue;
            }
            for (shape, layers, is_pad) in &parts {
                if !*is_pad {
                    for &l in layers {
                        if self.layers_ok(l) {
                            let bl = &block[l];
                            self.gr.each(shape, 0.0, |c| {
                                if bl[c] == 0 {
                                    st.push(((l * n + c) as u32, None));
                                }
                            });
                        }
                    }
                    continue;
                }
                let near = self.near(j, shape.bbox(), hw + beside.max(neck_len));
                let centre = shape.centre();
                for &l in layers {
                    if !self.layers_ok(l) {
                        continue;
                    }
                    // at full width: on the pad, else just beside it
                    let before = st.len();
                    let open = |st: &Vec<Entry2>| st[before..].iter().any(|e| room[e.0 as usize] >= roomy);
                    for r in [0.0, beside] {
                        let mut cells: Vec<usize> = Vec::new();
                        let bl = &block[l];
                        self.gr.each(shape, r, |c| {
                            if bl[c] == 0 {
                                cells.push(c);
                            }
                        });
                        for c in cells {
                            if self.stub_ok(hw, self.gr.xy(c), centre, l, &near, edge) {
                                st.push(((l * n + c) as u32, Some((centre, width))));
                            }
                        }
                        if open(&st) {
                            break;
                        }
                    }
                    if open(&st) {
                        continue;
                    }
                    // too wide to get there, or only into a pocket: the nearest cells with
                    // room around them that a narrower piece reaches from
                    if let Some(nw) = neck {
                        let mut cells: Vec<(f64, usize)> = Vec::new();
                        let bl = &block[l];
                        self.gr.each(shape, neck_len, |c| {
                            if bl[c] == 0 && room[l * n + c] >= roomy {
                                cells.push((0.0, c));
                            }
                        });
                        for e in cells.iter_mut() {
                            e.0 = geom::dist(self.gr.xy(e.1), centre);
                        }
                        cells.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
                        let mut nearest: Option<f64> = None;
                        for (d, c) in cells {
                            if nearest.map(|m| d > m + 2.0 * g).unwrap_or(false) {
                                break;
                            }
                            if self.stub_ok(nw / 2.0, self.gr.xy(c), centre, l, &near, edge) {
                                nearest.get_or_insert(d);
                                st.push(((l * n + c) as u32, Some((centre, nw))));
                            }
                        }
                    }
                }
            }
            st.sort_by(|a, b| a.0.cmp(&b.0));
            st.dedup_by(|a, b| a.0 == b.0);
            all.push(st);
        }
        self.entries[j] = Some(all.clone());
        all
    }

    /// Radius around a job of signature `sig` that keeps job j's tracks (via=false) or vias out.
    fn radius(&mut self, j: usize, sig: usize, via: bool) -> Option<f64> {
        let rep = self
            .jobs
            .iter()
            .enumerate()
            .find(|(k, jb)| *k != j && jb.sig == sig)
            .map(|(_, jb)| jb.net.clone())?;
        let net = self.jobs[j].net.clone();
        let me = if via { Item::via(&net) } else { Item::track(&net) };
        let mut c = self
            .clearance(me, Item::track(&rep))
            .max(self.clearance(me, Item::via(&rep)));
        if via {
            // two vias: their holes keep the board's hole-to-hole gap as well
            let other = self.jobs.iter().find(|jb| jb.net == rep).and_then(|jb| jb.via);
            if let (Some((d1, h1)), Some((d2, h2))) = (self.jobs[j].via, other) {
                c = c.max(self.rules.hole_to_hole + (h1 + h2) / 2.0 - (d1 + d2) / 2.0);
            }
        }
        let half = if via {
            self.jobs[j].via?.0 / 2.0
        } else {
            self.jobs[j].width / 2.0
        };
        Some(c + half + self.jobs[j].margin)
    }

    fn contribution(&mut self, k: usize, r: f64) -> Vec<Vec<u32>> {
        let key = um(r);
        if let Some(c) = self.contrib[k].get(&key) {
            return c.clone();
        }
        let (nl, n) = (self.gr.nl, self.gr.n);
        let mark = self.work.next_mark();
        let mut out: Vec<Vec<u32>> = vec![Vec::new(); nl];
        if let Some(laid) = &self.laid[k] {
            for (l, pts, w) in &laid.tracks {
                for s in pts.windows(2) {
                    let sh = Shape::Capsule {
                        a: s[0],
                        b: s[1],
                        r: w / 2.0,
                    };
                    let (mk, o) = (&mut self.work.mark, &mut out[*l]);
                    self.gr.each(&sh, r, |c| {
                        if mk[*l * n + c] != mark {
                            mk[*l * n + c] = mark;
                            o.push(c as u32);
                        }
                    });
                }
            }
            if let Some((d, _)) = self.jobs[k].via {
                for &p in &laid.vias {
                    let sh = Shape::Circle { c: p, r: d / 2.0 };
                    for l in 0..nl {
                        let (mk, o) = (&mut self.work.mark, &mut out[l]);
                        self.gr.each(&sh, r, |c| {
                            if mk[l * n + c] != mark {
                                mk[l * n + c] = mark;
                                o.push(c as u32);
                            }
                        });
                    }
                }
            }
        }
        self.contrib[k].insert(key, out.clone());
        out
    }

    fn occupancy(&mut self, sig: usize, r: f64) -> (usize, i64) {
        let key = (sig, um(r));
        if !self.occ.contains_key(&key) {
            let (nl, n) = (self.gr.nl, self.gr.n);
            let mut arr: Vec<Vec<u8>> = vec![vec![0u8; n]; nl];
            for k in 0..self.jobs.len() {
                if self.jobs[k].sig == sig && self.laid[k].is_some() {
                    let c = self.contribution(k, r);
                    for l in 0..nl {
                        for &cell in &c[l] {
                            arr[l][cell as usize] = arr[l][cell as usize].saturating_add(1);
                        }
                    }
                }
            }
            self.occ.insert(key, arr);
        }
        key
    }

    fn add(&mut self, k: usize, sign: i32) {
        let sig = self.jobs[k].sig;
        let keys: Vec<(usize, i64)> = self.occ.keys().filter(|(s, _)| *s == sig).cloned().collect();
        for key in keys {
            let c = self.contribution(k, key.1 as f64 / 1000.0);
            let arr = self.occ.get_mut(&key).unwrap();
            for (l, cells) in c.iter().enumerate() {
                for &cell in cells {
                    let v = &mut arr[l][cell as usize];
                    *v = if sign > 0 {
                        v.saturating_add(1)
                    } else {
                        v.saturating_sub(1)
                    };
                }
            }
        }
    }

    fn rip(&mut self, k: usize) {
        if self.laid[k].is_some() {
            self.add(k, -1);
            self.laid[k] = None;
            self.contrib[k].clear();
        }
    }

    fn signatures(&self) -> Vec<usize> {
        let mut s: Vec<usize> = self.jobs.iter().map(|j| j.sig).collect();
        s.sort();
        s.dedup();
        s
    }

    /// Soft costs for job j: other jobs' copper near each cell, for tracks per layer and for vias.
    fn soft(&mut self, j: usize) -> (Vec<Vec<u16>>, Vec<u32>) {
        let (nl, n) = (self.gr.nl, self.gr.n);
        let mut soft: Vec<Vec<u16>> = vec![vec![0u16; n]; nl];
        let mut sv: Vec<u32> = vec![0u32; n];
        for sig in self.signatures() {
            if let Some(rt) = self.radius(j, sig, false) {
                let key = self.occupancy(sig, rt);
                let arr = &self.occ[&key];
                for l in 0..nl {
                    for (d, s) in soft[l].iter_mut().zip(arr[l].iter()) {
                        *d += *s as u16;
                    }
                }
            }
            if let Some(rv) = self.radius(j, sig, true) {
                let key = self.occupancy(sig, rv);
                let arr = &self.occ[&key];
                for l in 0..nl {
                    for (d, s) in sv.iter_mut().zip(arr[l].iter()) {
                        *d += *s as u32;
                    }
                }
            }
        }
        if self.laid[j].is_some() {
            // job j's own copper is in its signature's pile: take it out again
            let sig = self.jobs[j].sig;
            if let Some(rt) = self.radius(j, sig, false) {
                let c = self.contribution(j, rt);
                for l in 0..nl {
                    for &cell in &c[l] {
                        soft[l][cell as usize] = soft[l][cell as usize].saturating_sub(1);
                    }
                }
            }
            if let Some(rv) = self.radius(j, sig, true) {
                let c = self.contribution(j, rv);
                for l in 0..nl {
                    for &cell in &c[l] {
                        sv[cell as usize] = sv[cell as usize].saturating_sub(1);
                    }
                }
            }
        }
        (soft, sv)
    }

    fn conflicts(&mut self, j: usize) -> Vec<u32> {
        let laid = match &self.laid[j] {
            Some(l) => l.clone(),
            None => return Vec::new(),
        };
        let (soft, sv) = self.soft(j);
        let n = self.gr.n;
        let mut bad = Vec::new();
        for p in &laid.paths {
            for &s in p {
                let (l, c) = (s as usize / n, s as usize % n);
                if soft[l][c] > 0 {
                    bad.push(s);
                }
            }
        }
        for &v in &laid.vias {
            let c = self.gr.cell(v);
            if sv[c] > 0 {
                bad.push(c as u32);
            }
        }
        bad
    }

    /// Put a second via beside each via of job j, joined to the first on every
    /// layer the net has a track at it: one small via is the weak point of a wide
    /// track. Returns (added, vias left single).
    fn double_vias(&mut self, j: usize) -> (usize, Vec<P>) {
        let drill = match self.jobs[j].via {
            Some(v) => v.1,
            None => return (0, Vec::new()),
        };
        let laid = match &self.laid[j] {
            Some(l) if !l.vias.is_empty() => l.clone(),
            _ => return (0, Vec::new()),
        };
        let (nl, g, w) = (self.gr.nl, self.gr.g, self.gr.w as i64);
        let width = self.jobs[j].width;
        let (block, vblock) = self.hard(j);
        let vblock = match vblock {
            Some(v) => v,
            None => return (0, Vec::new()),
        };
        let (soft, sv) = self.soft(j);
        // two holes no nearer than the board's hole-to-hole gap, edge to edge
        let apart = drill + self.rules.hole_to_hole;
        let mut holes: Vec<P> = laid.vias.clone();
        for t in &self.jobs[j].terminals {
            for (shape, layers, is_pad) in &t.parts {
                if layers.len() == nl && (*is_pad || matches!(shape, Shape::Circle { .. })) {
                    holes.push(shape.centre());
                }
            }
        }
        let straight = (apart / g - 1e-9).ceil().max(1.0) as i64;
        let diagonal = (apart / (g * SQRT2) - 1e-9).ceil().max(1.0) as i64;
        let mut steps: Vec<(i64, i64, i64)> = Vec::new(); // (di, dj, cells)
        for &(di, dj, _) in DIRS.iter() {
            steps.push((di, dj, if di != 0 && dj != 0 { diagonal } else { straight }));
        }
        let mut new_tracks: Vec<(usize, Vec<P>, f64)> = Vec::new();
        let mut new_vias: Vec<P> = Vec::new();
        let mut single: Vec<P> = Vec::new();
        for &p in &laid.vias {
            let c = self.gr.cell(p) as i64;
            // the layers the net's tracks meet this via on
            let layers: Vec<usize> = (0..nl)
                .filter(|&l| {
                    laid.tracks
                        .iter()
                        .any(|(tl, pts, _)| *tl == l && pts.iter().any(|q| geom::dist(*q, p) < 1e-6))
                })
                .collect();
            let mut placed = false;
            for &(di, dj, k) in &steps {
                let (i, jj) = (c % w + di * k, c / w + dj * k);
                if i < 0 || i >= w || jj < 0 || jj >= self.gr.h as i64 {
                    continue;
                }
                let c2 = (jj * w + i) as usize;
                if vblock[c2] != 0 || sv[c2] != 0 {
                    continue;
                }
                let p2 = self.gr.xy(c2);
                if holes.iter().any(|h| geom::dist(*h, p2) < apart - 1e-9) {
                    continue;
                }
                // the link between the two, on each layer, over free cells only
                let clear = layers.iter().all(|&l| {
                    (0..=k).all(|t| {
                        let cc = ((c / w + dj * t) * w + (c % w + di * t)) as usize;
                        block[l][cc] == 0 && soft[l][cc] == 0
                    })
                });
                if !clear {
                    continue;
                }
                for &l in &layers {
                    new_tracks.push((l, vec![p, p2], width));
                }
                new_vias.push(p2);
                holes.push(p2);
                placed = true;
                break;
            }
            if !placed {
                single.push(p);
            }
        }
        let added = new_vias.len();
        if added > 0 {
            self.add(j, -1);
            if let Some(l) = self.laid[j].as_mut() {
                l.tracks.extend(new_tracks);
                l.vias.extend(new_vias);
            }
            self.contrib[j].clear();
            self.add(j, 1);
        }
        (added, single)
    }

    /// The jobs whose copper job j's track cells or vias are too close to. A
    /// conflict is found from j's side only (its grid points against the other's
    /// copper), so the other party would otherwise never be asked to move.
    fn offenders(&mut self, j: usize) -> Vec<usize> {
        let laid = match &self.laid[j] {
            Some(l) => l.clone(),
            None => return Vec::new(),
        };
        let (nl, n) = (self.gr.nl, self.gr.n);
        let mut mine = vec![0u8; n * nl]; // 1: a track cell of j, 2: a via of j
        for p in &laid.paths {
            for &s in p {
                mine[s as usize] |= 1;
            }
        }
        for &v in &laid.vias {
            let c = self.gr.cell(v);
            for l in 0..nl {
                mine[l * n + c] |= 2;
            }
        }
        let mut out = Vec::new();
        for k in 0..self.jobs.len() {
            if k == j || self.laid[k].is_none() {
                continue;
            }
            let sig = self.jobs[k].sig;
            let mut hit = false;
            for (via, bit) in [(false, 1u8), (true, 2u8)] {
                if hit {
                    break;
                }
                if let Some(r) = self.radius(j, sig, via) {
                    let c = self.contribution(k, r);
                    hit = c
                        .iter()
                        .enumerate()
                        .any(|(l, cells)| cells.iter().any(|&cell| mine[l * n + cell as usize] & bit != 0));
                }
            }
            if hit {
                out.push(k);
            }
        }
        out
    }

    fn route_job(&mut self, j: usize, pres: f64, via_cost: f64, limit: usize, walls: bool) {
        let (nl, n) = (self.gr.nl, self.gr.n);
        let (mut block, vhard) = self.hard(j);
        let (soft, sv) = self.soft(j);
        let mut vblock = vhard;
        if walls {
            for l in 0..nl {
                for c in 0..n {
                    if soft[l][c] > 0 {
                        block[l][c] = 1;
                    }
                }
            }
            if let Some(vb) = vblock.as_mut() {
                for c in 0..n {
                    if sv[c] > 0 {
                        vb[c] = 1;
                    }
                }
            }
        }
        // the cells each terminal can be reached on (fewer when others' copper is a wall)
        let entries: Vec<Vec<Entry2>> = self
            .entries(j)
            .into_iter()
            .map(|t| {
                t.into_iter()
                    .filter(|e| block[e.0 as usize / n][e.0 as usize % n] == 0)
                    .collect()
            })
            .collect();
        let states: Vec<Vec<u32>> = entries.iter().map(|t| t.iter().map(|e| e.0).collect()).collect();
        let centre_of = |t: usize, s: u32| -> Option<(P, f64)> {
            entries[t]
                .binary_search_by(|e| e.0.cmp(&s))
                .ok()
                .and_then(|k| entries[t][k].1)
        };
        let room = self.room(j);
        let terms = self.jobs[j].terminals.len();
        let centre = |t: usize| -> P { self.jobs[j].terminals[t].centre };
        let mut todo: Vec<usize> = (0..terms).collect();
        let first = {
            let cx = todo.iter().map(|&t| centre(t).0).sum::<f64>() / terms as f64;
            let cy = todo.iter().map(|&t| centre(t).1).sum::<f64>() / terms as f64;

            (0..terms)
                .filter(|&t| !states[t].is_empty())
                .min_by(|&a, &b| {
                    let da = geom::dist(centre(a), (cx, cy));
                    let db = geom::dist(centre(b), (cx, cy));
                    da.partial_cmp(&db).unwrap().then(a.cmp(&b))
                })
                .unwrap_or(0)
        };
        todo.retain(|&t| t != first);
        let mut tree: Vec<u32> = states[first].clone();
        // tree cells that lie on a pad already joined, and whether its last bit of track is laid
        let mut tree_pad: HashMap<u32, ((P, f64), bool)> = HashMap::new();
        for e in &entries[first] {
            if let Some(c) = e.1 {
                tree_pad.insert(e.0, (c, false));
            }
        }
        let width = self.jobs[j].width;
        let first_label = self.jobs[j].terminals[first].label.clone();
        let mut near: Vec<f64> = (0..terms).map(|t| geom::dist(centre(t), centre(first))).collect();
        let mut laid = Laid::default();
        let search_via = if vblock.is_some() { via_cost } else { 0.0 };
        let hist = std::mem::take(&mut self.hist);
        while !todo.is_empty() {
            todo.sort_by(|&a, &b| near[a].partial_cmp(&near[b]).unwrap().then(a.cmp(&b)));
            let next = todo.remove(0);
            let label = self.jobs[j].terminals[next].label.clone();
            if states[next].is_empty() {
                laid.unrouted
                    .push(format!("{}: no clear way in for a {} mm track", label, width));
                continue;
            }
            let s = Search {
                block: &block,
                vblock: vblock.as_deref(),
                soft: &soft,
                soft_via: &sv,
                hist: &hist,
                pres,
                via_cost: search_via,
                limit,
            };
            match astar(&self.gr, &mut self.work, &s, &tree, &states[next], centre(next)) {
                None => {
                    // say which side is shut in, when one is
                    let most = |st: &[u32]| st.iter().map(|&s| room[s as usize]).max().unwrap_or(0);
                    let (here, there) = (most(&states[next]), most(&tree));
                    let whole = room.iter().cloned().max().unwrap_or(0);
                    let why = if here < whole / 20 {
                        format!(
                            "shut in: only {:.1} mm2 of room around it",
                            here as f64 * self.gr.g * self.gr.g
                        )
                    } else if there < whole / 20 {
                        "the rest of the net is shut in".to_string()
                    } else {
                        "no path between other nets' copper".to_string()
                    };
                    laid.unrouted.push(format!("{}: {}", label, why));
                }
                Some(path) => {
                    for t in 0..terms {
                        near[t] = near[t].min(geom::dist(centre(t), centre(next)));
                    }
                    tree.extend_from_slice(&states[next]);
                    tree.extend_from_slice(&path);
                    tree.sort();
                    tree.dedup();
                    // finish on the pads' centres: where the path starts, unless an earlier
                    // path already did, and where it ends
                    let from = match tree_pad.get_mut(&path[0]) {
                        Some((c, done)) if !*done => {
                            *done = true;
                            Some(*c)
                        }
                        _ => None,
                    };
                    let last = path[path.len() - 1];
                    let to = centre_of(next, last);
                    for e in &entries[next] {
                        if let Some(c) = e.1 {
                            tree_pad.entry(e.0).or_insert((c, e.0 == last));
                        }
                    }
                    if to.is_some() {
                        if let Some(v) = tree_pad.get_mut(&last) {
                            v.1 = true;
                        }
                    }
                    for (end, name) in [(from, &first_label), (to, &label)] {
                        if let Some((_, w)) = end {
                            if w < width - 1e-9 && !laid.necked.iter().any(|(n, _)| n == name) {
                                laid.necked.push((name.clone(), w));
                            }
                        }
                    }
                    self.geometry(&path, width, from, to, &mut laid);
                    laid.paths.push(path);
                }
            }
        }
        self.hist = hist;
        self.laid[j] = Some(laid);
        self.contrib[j].clear();
        self.add(j, 1);
    }

    /// Turn a path of states into tracks and vias. `from` and `to` are the
    /// points to start and finish on (a pad's centre, a track's loose end) and
    /// the width of that piece, when the path's ends call for them.
    fn geometry(&self, path: &[u32], width: f64, from: Option<(P, f64)>, to: Option<(P, f64)>, laid: &mut Laid) {
        let n = self.gr.n;
        let mut runs: Vec<(usize, Vec<P>)> = Vec::new();
        for (k, &s) in path.iter().enumerate() {
            let (l, c) = (s as usize / n, s as usize % n);
            let p = self.gr.xy(c);
            match runs.last_mut() {
                Some((rl, pts)) if *rl == l => pts.push(p),
                _ => {
                    if k > 0 {
                        laid.vias.push(p);
                    }
                    runs.push((l, vec![p]));
                }
            }
        }
        // (a path of one cell, where the tree already crosses the pad, is still
        // finished on the pad's centre.) A narrowed piece is a track of its own.
        let last = runs.len() - 1;
        for (end, at_start) in [(from, true), (to, false)] {
            if let Some((pc, w)) = end {
                let (l, pts) = &mut runs[if at_start { 0 } else { last }];
                let cell = if at_start { pts[0] } else { pts[pts.len() - 1] };
                if w < width - 1e-9 {
                    if geom::dist(cell, pc) > 1e-6 {
                        laid.tracks.push((*l, vec![cell, pc], w));
                    }
                } else if at_start {
                    pts.insert(0, pc);
                } else {
                    pts.push(pc);
                }
            }
        }
        for (l, pts) in runs {
            let mut simp: Vec<P> = Vec::new();
            for p in pts {
                if simp.last().map(|q: &P| geom::dist(*q, p) < 1e-6).unwrap_or(false) {
                    continue;
                }
                if simp.len() >= 2 {
                    let (a, b) = (simp[simp.len() - 2], simp[simp.len() - 1]);
                    if ((b.0 - a.0) * (p.1 - b.1) - (b.1 - a.1) * (p.0 - b.0)).abs() < 1e-9
                        && (b.0 - a.0) * (p.0 - b.0) + (b.1 - a.1) * (p.1 - b.1) >= 0.0
                    {
                        simp.pop();
                    }
                }
                simp.push(p);
            }
            if simp.len() >= 2 {
                laid.tracks.push((l, simp, width));
            }
        }
    }
}

/// Inner layers where copper pours cover at least half the board: planes.
pub fn plane_layers(board: &Board) -> Vec<usize> {
    let nl = board.copper.len();
    let (x0, y0, x1, y1) = match board.extent() {
        Some(e) => e,
        None => return Vec::new(),
    };
    let whole = (x1 - x0) * (y1 - y0);
    (1..nl.saturating_sub(1))
        .filter(|&l| {
            let poured: f64 = board
                .zones
                .iter()
                .filter(|z| z.keepout.is_none() && z.net.is_some() && z.layers.contains(&l))
                .map(|z| {
                    let p = &z.outline;
                    (0..p.len())
                        .map(|i| {
                            let (a, b) = (p[i], p[(i + 1) % p.len()]);
                            a.0 * b.1 - b.0 * a.1
                        })
                        .sum::<f64>()
                        .abs()
                        / 2.0
                })
                .sum();
            whole > 0.0 && poured >= 0.5 * whole
        })
        .collect()
}

/// Whether `route` would take on this net (the same test it uses), for
/// deciding which existing copper to rip up first.
pub fn wanted(board: &Board, opt: &Options, net: &str) -> bool {
    if net.starts_with("unconnected-") {
        return false;
    }
    if let Some(only) = &opt.nets {
        if !matches(only, net) {
            return false;
        }
    }
    if matches(&opt.skip, net) {
        return false;
    }
    opt.include_poured || !board.poured_nets().iter().any(|p| p == net)
}

/// Route a board. The board is not changed; the result says what to add.
pub fn route(board: &Board, rules: &Rules, opt: &Options) -> Routed {
    let mut out = Routed::default();
    let nl = board.copper.len();
    let (bx0, by0, bx1, by1) = match board.extent() {
        Some(e) => e,
        None => {
            out.log
                .push("nothing to route: the board has no outline and no pads".into());
            return out;
        }
    };
    let g = opt.grid;
    let x0 = ((bx0 - 1.0) / g).floor() * g;
    let y0 = ((by0 - 1.0) / g).floor() * g;
    let w = (((bx1 + 1.0) - x0) / g).ceil() as usize + 1;
    let h = (((by1 + 1.0) - y0) / g).ceil() as usize + 1;
    let gr = Grid {
        x0,
        y0,
        g,
        w,
        h,
        n: w * h,
        nl,
    };

    // outside the outline: flood from the grid's border, stopped by the edge lines
    let mut outside = vec![0u8; gr.n];
    if !board.edges.is_empty() {
        let mut wall = vec![0u8; gr.n];
        for &(a, b) in &board.edges {
            gr.each(&Shape::Capsule { a, b, r: 0.0 }, g * 0.75, |c| wall[c] = 1);
        }
        let mut stack: Vec<usize> = Vec::new();
        for i in 0..w {
            stack.push(i);
            stack.push((h - 1) * w + i);
        }
        for j in 0..h {
            stack.push(j * w);
            stack.push(j * w + w - 1);
        }
        while let Some(c) = stack.pop() {
            if outside[c] != 0 || wall[c] != 0 {
                continue;
            }
            outside[c] = 1;
            let (i, j) = (c % w, c / w);
            if i > 0 {
                stack.push(c - 1);
            }
            if i + 1 < w {
                stack.push(c + 1);
            }
            if j > 0 {
                stack.push(c - w);
            }
            if j + 1 < h {
                stack.push(c + w);
            }
        }
    }

    // an outline with a gap in it lets the flood in, and then nothing could be routed
    if !board.pads.is_empty() {
        let lost = board
            .pads
            .iter()
            .filter(|p| outside[gr.cell(p.shape.centre())] != 0)
            .count();
        if lost * 2 > board.pads.len() {
            out.warnings.push(format!(
                "the board outline (Edge.Cuts) is not a closed shape: {} of {} pads count as outside it. \
                 Routing without it; tracks still keep clear of the lines that are there",
                lost,
                board.pads.len()
            ));
            outside = vec![0u8; gr.n];
        }
    }

    // what is in the way
    let mut obstacles: Vec<Obstacle> = Vec::new();
    for p in &board.pads {
        obstacles.push(Obstacle {
            net: p.net.clone(),
            kind: "Pad",
            shape: p.shape,
            layers: p.layers.clone(),
            least: 0.0,
            group: 0,
        });
    }
    for t in &board.tracks {
        obstacles.push(Obstacle {
            net: t.net.clone(),
            kind: "Track",
            shape: Shape::Capsule {
                a: t.a,
                b: t.b,
                r: t.width / 2.0,
            },
            layers: vec![t.layer],
            least: 0.0,
            group: 0,
        });
    }
    for v in &board.vias {
        obstacles.push(Obstacle {
            net: v.net.clone(),
            kind: "Via",
            shape: Shape::Circle {
                c: v.at,
                r: v.diameter / 2.0,
            },
            layers: (0..nl).collect(),
            least: 0.0,
            group: 0,
        });
    }
    for &shape in &board.holes {
        obstacles.push(Obstacle {
            net: None,
            kind: "Pad",
            shape,
            layers: (0..nl).collect(),
            least: rules.hole_clearance,
            group: 0,
        });
    }
    let mut groups: HashMap<(Option<String>, &'static str, u64), usize> = HashMap::new();
    let mut net_sig: HashMap<String, String> = HashMap::new();
    for ob in obstacles.iter_mut() {
        let sig = ob
            .net
            .as_ref()
            .map(|n| net_sig.entry(n.clone()).or_insert_with(|| rules.signature(n)).clone());
        let next = groups.len();
        ob.group = *groups.entry((sig, ob.kind, rules.place(&ob.shape))).or_insert(next);
    }
    let ngroups = groups.len();

    // what to join: per net, the pieces its pads and copper already form
    let poured = board.poured_nets();
    let mut by_net: BTreeMap<String, Vec<(Shape, Vec<usize>, bool, String)>> = BTreeMap::new();
    for p in &board.pads {
        if let Some(n) = &p.net {
            by_net
                .entry(n.clone())
                .or_default()
                .push((p.shape, p.layers.clone(), true, p.name()));
        }
    }
    for t in &board.tracks {
        if let Some(n) = &t.net {
            if let Some(v) = by_net.get_mut(n) {
                v.push((
                    Shape::Capsule {
                        a: t.a,
                        b: t.b,
                        r: t.width / 2.0,
                    },
                    vec![t.layer],
                    false,
                    String::new(),
                ));
            }
        }
    }
    for v in &board.vias {
        if let Some(n) = &v.net {
            if let Some(list) = by_net.get_mut(n) {
                list.push((
                    Shape::Circle {
                        c: v.at,
                        r: v.diameter / 2.0,
                    },
                    (0..nl).collect(),
                    false,
                    String::new(),
                ));
            }
        }
    }
    let mut jobs: Vec<Job> = Vec::new();
    let mut sigs: Vec<String> = Vec::new();
    let smallest = rules.smallest_clearance();
    for (net, items) in by_net {
        let pads = items.iter().filter(|i| i.2).count();
        if pads < 2 {
            continue;
        }
        if net.starts_with("unconnected-") {
            continue;
        }
        if let Some(only) = &opt.nets {
            if !matches(only, &net) {
                continue;
            }
        }
        if matches(&opt.skip, &net) {
            out.skipped.push((net, "skipped as asked".into()));
            continue;
        }
        if poured.contains(&net) && !opt.include_poured {
            out.skipped.push((net, "has a copper pour".into()));
            continue;
        }
        // union-find over copper that KiCad would call joined
        let k = items.len();
        let mut parent: Vec<usize> = (0..k).collect();
        fn find(p: &mut Vec<usize>, x: usize) -> usize {
            let mut r = x;
            while p[r] != r {
                r = p[r];
            }
            let mut y = x;
            while p[y] != r {
                let nx = p[y];
                p[y] = r;
                y = nx;
            }
            r
        }
        for a in 0..k {
            for b in a + 1..k {
                if joined(&items[a], &items[b]) {
                    let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                    if ra != rb {
                        parent[ra] = rb;
                    }
                }
            }
        }
        let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for i in 0..k {
            let r = find(&mut parent, i);
            groups.entry(r).or_default().push(i);
        }
        let terminals: Vec<Terminal> = groups
            .values()
            .filter(|g| g.iter().any(|&i| items[i].2))
            .map(|g| {
                let pads: Vec<usize> = g.iter().cloned().filter(|&i| items[i].2).collect();
                let centre = items[pads[0]].0.centre();
                Terminal {
                    label: items[pads[0]].3.clone(),
                    centre,
                    parts: g
                        .iter()
                        .map(|&i| (items[i].0, items[i].1.clone(), items[i].2))
                        .collect(),
                }
            })
            .collect();
        if terminals.len() < 2 {
            continue; // already joined
        }
        let width = opt.track_width.unwrap_or_else(|| rules.track_width(&net));
        let via = if opt.no_vias {
            None
        } else {
            opt.via.or_else(|| rules.via(&net))
        };
        let sig_s = rules.signature(&net);
        let sig = match sigs.iter().position(|s| *s == sig_s) {
            Some(i) => i,
            None => {
                sigs.push(sig_s);
                sigs.len() - 1
            }
        };
        let (mut sx0, mut sy0, mut sx1, mut sy1) = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for t in &terminals {
            sx0 = sx0.min(t.centre.0);
            sy0 = sy0.min(t.centre.1);
            sx1 = sx1.max(t.centre.0);
            sy1 = sy1.max(t.centre.1);
        }
        let margin = opt.margin.max(grid_margin(width / 2.0 + smallest, g));
        let neck = opt
            .neck
            .unwrap_or(rules.classes[0].track_width)
            .max(rules.track_floor(&net));
        let neck = if opt.neck_length > 0.0 && neck < width - 1e-9 {
            Some(neck)
        } else {
            None
        };
        jobs.push(Job {
            net,
            width,
            via,
            margin,
            neck,
            terminals,
            sig,
            span: (sx1 - sx0) + (sy1 - sy0),
        });
    }
    // short nets first: they have the fewest ways round
    jobs.sort_by(|a, b| a.span.partial_cmp(&b.span).unwrap().then(a.net.cmp(&b.net)));
    let nj = jobs.len();
    let cache_ok = rules.place_is_whole();
    let planes = if opt.tracks_on_planes {
        Vec::new()
    } else {
        plane_layers(board)
    };
    if !planes.is_empty() {
        out.log.push(format!(
            "keeping tracks off the plane layer(s) {} (--tracks-on-planes to use them)",
            planes
                .iter()
                .map(|&l| board.copper[l].clone())
                .collect::<Vec<String>>()
                .join(", ")
        ));
        if opt.progress {
            eprintln!("{}", out.log[out.log.len() - 1]);
        }
    }
    let mut r = Router {
        planes,
        ngroups,
        board,
        rules,
        opt,
        obstacles,
        outside,
        hard: (0..nj).map(|_| None).collect(),
        entries: (0..nj).map(|_| None).collect(),
        rooms: (0..nj).map(|_| None).collect(),
        owed: vec![Vec::new(); nj],
        laid: vec![None; nj],
        contrib: vec![HashMap::new(); nj],
        occ: HashMap::new(),
        hist: vec![vec![0f32; gr.n]; nl],
        work: Work::new(gr.n * nl),
        clr_cache: HashMap::new(),
        cache_ok,
        log: Vec::new(),
        jobs,
        gr,
    };
    r.say(format!(
        "grid {} x {} x {} layers at {} mm; {} nets to route, {} left alone",
        r.gr.w,
        r.gr.h,
        nl,
        g,
        nj,
        out.skipped.len()
    ));
    let widest = r.jobs.iter().map(|j| j.margin).fold(0.0, f64::max);
    if widest > opt.margin + 1e-9 {
        r.say(format!(
            "  the {} mm grid needs up to {:.3} mm of margin on clearances (asked: {} mm); a finer grid packs tracks closer",
            g, widest, opt.margin
        ));
    }

    let mut pres = 0.5;
    let mut todo: Vec<usize> = (0..nj).collect();
    let mut best: Option<usize> = None;
    let mut since = 0;
    let mut conflicts: BTreeMap<usize, Vec<u32>> = BTreeMap::new();
    for pass in 1..=opt.passes {
        for &j in &todo {
            r.rip(j);
            r.route_job(j, pres, opt.via_cost, opt.limit, false);
            if !r.laid[j].as_ref().unwrap().unrouted.is_empty() {
                r.rip(j);
                r.route_job(j, pres, opt.via_cost / 2.0, opt.limit * 5, false);
            }
        }
        conflicts.clear();
        for j in 0..nj {
            let b = r.conflicts(j);
            if !b.is_empty() {
                conflicts.insert(j, b);
            }
        }
        let cells: usize = conflicts.values().map(|v| v.len()).sum();
        let vias: usize = r.laid.iter().flatten().map(|l| l.vias.len()).sum();
        r.say(format!(
            "  pass {:2}: routed {:3} nets, {:3} in conflict over {:5} cells, {} vias",
            pass,
            todo.len(),
            conflicts.len(),
            cells,
            vias
        ));
        out.passes = pass;
        if conflicts.is_empty() {
            break;
        }
        match best {
            Some(b) if cells >= b => {
                since += 1;
                if since >= opt.stall {
                    break;
                }
            }
            _ => {
                best = Some(cells);
                since = 0;
            }
        }
        for bad in conflicts.values() {
            for &s in bad {
                let (l, c) = (s as usize / r.gr.n, s as usize % r.gr.n);
                r.hist[l][c] += 0.5;
            }
        }
        // those in conflict, and those they are in conflict with
        todo = conflicts.keys().cloned().collect();
        for j in conflicts.keys().cloned().collect::<Vec<usize>>() {
            todo.extend(r.offenders(j));
        }
        todo.sort();
        todo.dedup();
        pres *= 1.6;
    }
    // what is still in conflict is routed with the others as walls, until nothing is;
    // a net that cannot be made legal even so is taken off
    for round in 0..6 {
        if conflicts.is_empty() {
            break;
        }
        let mut order: Vec<(usize, usize)> = conflicts.iter().map(|(j, v)| (*j, v.len())).collect();
        order.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        for (j, _) in order {
            if r.conflicts(j).is_empty() {
                continue;
            }
            let others = r.offenders(j);
            r.rip(j);
            if round < 5 {
                r.route_job(j, pres, opt.via_cost, opt.limit * 4, true);
                // squeezed out? Then it goes first and the ones in its way go round it,
                // if that leaves less unjoined
                let left = |r: &Router, k: usize| r.laid[k].as_ref().map(|l| l.unrouted.len()).unwrap_or(0);
                if left(&r, j) > 0 && !others.is_empty() {
                    let mut group = vec![j];
                    group.extend(&others);
                    let before: usize = group.iter().map(|&k| left(&r, k)).sum();
                    let saved: Vec<Option<Laid>> = group.iter().map(|&k| r.laid[k].clone()).collect();
                    for &k in &group {
                        r.rip(k);
                    }
                    for &k in &group {
                        r.route_job(k, pres, opt.via_cost, opt.limit * 4, true);
                    }
                    let after: usize = group.iter().map(|&k| left(&r, k)).sum();
                    if after >= before {
                        for &k in &group {
                            r.rip(k);
                        }
                        for (&k, old) in group.iter().zip(saved) {
                            r.laid[k] = old;
                            r.contrib[k].clear();
                            r.add(k, 1);
                        }
                    }
                }
            } else {
                let all: Vec<String> = r.jobs[j].terminals.iter().skip(1).map(|t| t.label.clone()).collect();
                r.laid[j] = Some(Laid {
                    unrouted: all,
                    ..Laid::default()
                });
            }
            let left = r.laid[j].as_ref().unwrap().unrouted.len();
            let msg = format!(
                "  settled {} with the others as walls{}",
                r.jobs[j].net,
                if left > 0 {
                    format!(": {} piece(s) left unjoined", left)
                } else {
                    String::new()
                }
            );
            r.say(msg);
        }
        conflicts.clear();
        for j in 0..nj {
            let b = r.conflicts(j);
            if !b.is_empty() {
                conflicts.insert(j, b);
            }
        }
    }

    // wide tracks: a second via beside each via
    if opt.double_vias > 0.0 {
        for j in 0..nj {
            if r.jobs[j].width >= opt.double_vias - 1e-9 {
                let (added, single) = r.double_vias(j);
                out.doubled += added;
                let net = r.jobs[j].net.clone();
                out.single.extend(single.into_iter().map(|p| (net.clone(), p)));
            }
        }
        if out.doubled > 0 || !out.single.is_empty() {
            r.say(format!(
                "  second vias on tracks of {} mm and wider: {} added, {} with no room",
                opt.double_vias,
                out.doubled,
                out.single.len()
            ));
        }
    }

    for j in 0..nj {
        let job = &r.jobs[j];
        let laid = r.laid[j].clone().unwrap_or_default();
        for (l, pts, w) in &laid.tracks {
            for s in pts.windows(2) {
                out.tracks.push(NewTrack {
                    net: job.net.clone(),
                    layer: *l,
                    a: s[0],
                    b: s[1],
                    width: *w,
                });
            }
        }
        if let Some((d, drill)) = job.via {
            for &p in &laid.vias {
                out.vias.push(NewVia {
                    net: job.net.clone(),
                    at: p,
                    diameter: d,
                    drill,
                });
            }
        }
        out.nets.push(NetReport {
            net: job.net.clone(),
            width: job.width,
            necked: laid.necked.clone(),
            pieces: job.terminals.len(),
            unrouted: laid.unrouted.clone(),
        });
    }
    out.log.append(&mut r.log);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::Board;

    const HEAD: &str =
        "(kicad_pcb (version 20260206) (layers (0 \"F.Cu\" signal) (2 \"B.Cu\" signal) (25 \"Edge.Cuts\" user))\n";

    /// A surface-mount pad on the front, as its own footprint.
    fn smd(reference: &str, net: &str, x: f64, y: f64, w: f64, h: f64) -> String {
        format!(
            "(footprint \"P\" (at {} {}) (property \"Reference\" \"{}\") (pad \"1\" smd rect (at 0 0) (size {} {}) (layers \"F.Cu\") (net \"{}\")))\n",
            x, y, reference, w, h, net
        )
    }

    fn rules() -> Rules {
        let mut r = Rules::default();
        r.edge_clearance = 0.2;
        r.hole_to_hole = 0.5;
        r
    }

    /// The gap between two pieces of copper (below 0 when they overlap).
    fn gap(a: &Shape, b: &Shape) -> f64 {
        match (*a, *b) {
            (Shape::Capsule { a: p, b: q, r }, other) | (other, Shape::Capsule { a: p, b: q, r }) => {
                geom::seg_shape_dist(p, q, &other) - r
            }
            (Shape::Circle { c, r }, other) | (other, Shape::Circle { c, r }) => other.dist(c) - r,
            _ => f64::INFINITY, // two pads: the board's own business
        }
    }

    /// (net, layer, shape) of every pad and of everything the router added.
    fn copper(b: &Board, r: &Routed) -> Vec<(String, usize, Shape)> {
        let mut out = Vec::new();
        for t in &r.tracks {
            out.push((
                t.net.clone(),
                t.layer,
                Shape::Capsule {
                    a: t.a,
                    b: t.b,
                    r: t.width / 2.0,
                },
            ));
        }
        for v in &r.vias {
            for l in 0..b.copper.len() {
                out.push((
                    v.net.clone(),
                    l,
                    Shape::Circle {
                        c: v.at,
                        r: v.diameter / 2.0,
                    },
                ));
            }
        }
        for p in &b.pads {
            for &l in &p.layers {
                out.push((p.net.clone().unwrap_or_default(), l, p.shape));
            }
        }
        out
    }

    /// The smallest gap between copper of two different nets on one layer.
    fn smallest_gap(b: &Board, r: &Routed) -> f64 {
        let c = copper(b, r);
        let mut worst = f64::INFINITY;
        for i in 0..c.len() {
            for k in i + 1..c.len() {
                if c[i].0 != c[k].0 && c[i].1 == c[k].1 {
                    worst = worst.min(gap(&c[i].2, &c[k].2));
                }
            }
        }
        worst
    }

    /// Whether KiCad would call every pad of `net` joined: pads, new tracks and
    /// vias linked where a track's end or a via's or pad's centre lies on the other.
    fn kicad_joined(b: &Board, r: &Routed, net: &str) -> bool {
        let all: Vec<usize> = (0..b.copper.len()).collect();
        let mut items: Vec<(Shape, Vec<usize>, bool, String)> = Vec::new();
        for p in b.pads.iter().filter(|p| p.net.as_deref() == Some(net)) {
            items.push((p.shape, p.layers.clone(), true, p.name()));
        }
        let pads = items.len();
        for t in r.tracks.iter().filter(|t| t.net == net) {
            let shape = Shape::Capsule {
                a: t.a,
                b: t.b,
                r: t.width / 2.0,
            };
            items.push((shape, vec![t.layer], false, String::new()));
        }
        for v in r.vias.iter().filter(|v| v.net == net) {
            let shape = Shape::Circle {
                c: v.at,
                r: v.diameter / 2.0,
            };
            items.push((shape, all.clone(), false, String::new()));
        }
        let mut group: Vec<usize> = (0..items.len()).collect();
        loop {
            let mut changed = false;
            for i in 0..items.len() {
                for k in i + 1..items.len() {
                    if group[i] != group[k] && joined(&items[i], &items[k]) {
                        let (from, to) = (group[k].max(group[i]), group[k].min(group[i]));
                        group.iter_mut().filter(|g| **g == from).for_each(|g| *g = to);
                        changed = true;
                    }
                }
            }
            if !changed {
                break;
            }
        }
        (0..pads).all(|i| group[i] == group[0])
    }

    /// Four nets that must cross on a two-layer board, front-only pads against the edges.
    fn crossing() -> Board {
        let mut s = String::from(HEAD);
        let pads = [
            ("H1", 1.0, 6.0),
            ("H1", 29.0, 6.0),
            ("H2", 1.0, 10.0),
            ("H2", 29.0, 10.0),
            ("V1", 10.0, 1.0),
            ("V1", 10.0, 19.0),
            ("V2", 20.0, 1.0),
            ("V2", 20.0, 19.0),
        ];
        for (k, (net, x, y)) in pads.iter().enumerate() {
            s += &smd(&format!("J{}", k), net, *x, *y, 0.8, 0.8);
        }
        s += "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        Board::parse(s).unwrap()
    }

    #[test]
    fn crossing_nets_route_with_vias_and_keep_clear() {
        let b = crossing();
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(r.vias.len() >= 2, "crossings need vias");
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        for net in ["H1", "H2", "V1", "V2"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined as KiCad counts it", net);
        }
        // no via on a surface-mount pad of its own net (the hole would drain the joint)
        for v in &r.vias {
            for p in &b.pads {
                assert!(
                    p.shape.dist(v.at) > v.drill / 2.0,
                    "a via of {} sits on {}",
                    v.net,
                    p.name()
                );
            }
        }
    }

    #[test]
    fn a_coarse_grid_still_keeps_tracks_apart() {
        // at 1 mm, two tracks could once cross between each other's grid points
        let b = crossing();
        let opt = Options {
            grid: 1.0,
            ..Options::default()
        };
        let r = route(&b, &rules(), &opt);
        assert!(!r.tracks.is_empty());
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        for n in r.nets.iter().filter(|n| n.unrouted.is_empty()) {
            assert!(kicad_joined(&b, &r, &n.net), "{} is reported joined but is not", n.net);
        }
    }

    #[test]
    fn a_pad_the_track_already_crosses_is_still_finished_at_its_centre() {
        // T and A are joined first, by a track that runs across a corner of the tall
        // pad C, far from C's centre: C must still get a track to its centre
        let s = String::from(HEAD)
            + &smd("A", "N", 2.0, 5.0, 1.0, 1.0)
            + &smd("T", "N", 9.5, 5.0, 1.0, 1.0)
            + &smd("C", "N", 6.0, 12.0, 3.0, 15.0)
            + "(gr_rect (start 0 0) (end 12 22) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(
            kicad_joined(&b, &r, "N"),
            "C is crossed by the track but not joined at its centre"
        );
    }

    #[test]
    fn tracks_go_round_a_milled_slot() {
        // a 1 x 8 mm non-plated slot stands between the two pads
        let s = String::from(HEAD)
            + &smd("A", "N", 2.0, 5.0, 1.0, 1.0)
            + &smd("B", "N", 18.0, 5.0, 1.0, 1.0)
            + "(footprint \"S\" (at 10 5) (property \"Reference\" \"H1\") (pad \"\" np_thru_hole oval (at 0 0) (size 1 8) (drill oval 1 8) (layers \"*.Cu\" \"*.Mask\")))\n"
            + "(gr_rect (start 0 -5) (end 20 15) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        let slot = b.holes[0];
        for t in &r.tracks {
            let d = geom::seg_shape_dist(t.a, t.b, &slot) - t.width / 2.0;
            assert!(d >= 0.25 - 1e-6, "a track passes {:.3} mm from the slot", d);
        }
    }

    #[test]
    fn an_open_outline_is_reported_not_fatal() {
        // three sides of a rectangle: the flood gets in, and once nothing routed
        let s = String::from(HEAD)
            + &smd("A", "N", 2.0, 5.0, 1.0, 1.0)
            + &smd("B", "N", 18.0, 5.0, 1.0, 1.0)
            + "(gr_line (start 0 0) (end 20 0) (layer \"Edge.Cuts\"))\n"
            + "(gr_line (start 20 0) (end 20 10) (layer \"Edge.Cuts\"))\n"
            + "(gr_line (start 20 10) (end 0 10) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(
            r.warnings.iter().any(|w| w.contains("not a closed shape")),
            "{:?}",
            r.warnings
        );
    }

    #[test]
    fn existing_copper_counts_as_joined() {
        let mut s = String::from(HEAD);
        for (k, x) in [2.0, 8.0, 14.0].iter().enumerate() {
            s += &smd(&format!("J{}", k), "A", *x, 5.0, 1.0, 1.0);
        }
        s += "(segment (start 2 5) (end 8 5) (width 0.25) (layer \"F.Cu\") (net \"A\")))";
        let b = Board::parse(s).unwrap();
        let r = route(&b, &Rules::default(), &Options::default());
        assert_eq!(r.nets.len(), 1);
        assert_eq!(r.nets[0].pieces, 2, "J0 and J1 are already joined");
        assert!(r.complete());
    }

    #[test]
    fn tracks_that_only_cross_are_not_joined() {
        // two tracks of one net cross in an X with no end on each other: KiCad does
        // not join them, so neither may the router
        let s = String::from(HEAD)
            + &smd("A", "N", 2.0, 2.0, 1.0, 1.0)
            + &smd("B", "N", 8.0, 8.0, 1.0, 1.0)
            + "(segment (start 2 2) (end 6 6) (width 0.25) (layer \"F.Cu\") (net \"N\"))\n"
            + "(segment (start 8 8) (end 8 4) (width 0.25) (layer \"F.Cu\") (net \"N\"))\n"
            + "(segment (start 8 4) (end 3 4) (width 0.25) (layer \"F.Cu\") (net \"N\")))";
        let b = Board::parse(s).unwrap();
        let r = route(&b, &Rules::default(), &Options::default());
        assert_eq!(r.nets.len(), 1, "the net still needs joining");
        assert_eq!(r.nets[0].pieces, 2);
        assert!(r.complete());
    }

    #[test]
    fn wide_tracks_get_a_second_via() {
        // one wide net that has to change layer under a wall of another net's track
        let s = String::from(HEAD)
            + &smd("A", "P", 3.0, 10.0, 2.0, 2.0)
            + &smd("B", "P", 27.0, 10.0, 2.0, 2.0)
            + &smd("C", "S", 15.0, 1.0, 0.8, 0.8)
            + &smd("D", "S", 15.0, 19.0, 0.8, 0.8)
            + "(segment (start 15 1) (end 15 19) (width 0.25) (layer \"F.Cu\") (net \"S\"))\n"
            + "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let opt = Options {
            track_width: Some(1.2),
            via: Some((0.6, 0.3)),
            ..Options::default()
        };
        let r = route(&b, &rules(), &opt);
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.vias.len(), 4, "two crossings of the layer, each with a second via");
        assert_eq!(r.doubled, 2);
        assert!(smallest_gap(&b, &r) >= 0.2 - 1e-6);
        // the pairs are joined, and their holes keep apart
        assert!(kicad_joined(&b, &r, "P"));
        for (i, v) in r.vias.iter().enumerate() {
            for u in &r.vias[i + 1..] {
                assert!(
                    geom::dist(v.at, u.at) >= 0.3 + 0.5 - 1e-9,
                    "two holes {:.3} mm apart",
                    geom::dist(v.at, u.at)
                );
            }
        }
        // and none when it is turned off
        let off = route(
            &b,
            &rules(),
            &Options {
                double_vias: 0.0,
                ..opt
            },
        );
        assert_eq!(off.vias.len(), 2);
    }

    #[test]
    fn the_same_board_routes_the_same_way_every_time() {
        let b = crossing();
        let (one, two) = (
            route(&b, &rules(), &Options::default()),
            route(&b, &rules(), &Options::default()),
        );
        assert_eq!(one.tracks, two.tracks);
        assert_eq!(one.vias, two.vias);
    }

    #[test]
    fn a_wide_track_narrows_to_reach_a_small_pad() {
        // a 1.2 mm track cannot get between the neighbours of the middle pin; a short
        // 0.25 mm piece can
        let mut s = String::from(HEAD);
        for (k, net) in ["X", "P", "Y"].iter().enumerate() {
            s += &smd(&format!("U{}", k), net, 10.0 + 0.65 * k as f64, 5.0, 0.35, 1.4);
        }
        s += &smd("C", "P", 10.65, 12.0, 2.0, 2.0);
        s += "(gr_rect (start 0 0) (end 22 16) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let wide = Options {
            track_width: Some(1.2),
            ..Options::default()
        };
        let r = route(&b, &rules(), &wide);
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.nets[0].necked.len(), 1, "{:?}", r.nets[0].necked);
        assert!(r.tracks.iter().any(|t| (t.width - 0.25).abs() < 1e-9));
        assert!(r.tracks.iter().any(|t| (t.width - 1.2).abs() < 1e-9));
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        assert!(kicad_joined(&b, &r, "P"));
        // and without narrowing, the pad is reported, not forced
        let stiff = route(
            &b,
            &rules(),
            &Options {
                neck_length: 0.0,
                ..wide
            },
        );
        assert!(!stiff.complete());
        assert!(smallest_gap(&b, &stiff) >= 0.2 - 1e-6);
    }

    #[test]
    fn a_kept_fan_out_is_joined_at_its_loose_end() {
        // a locked stub leads away from pad A: the new track must end on the stub's end,
        // not start again at the pad and leave the stub dangling
        let s = String::from(HEAD)
            + &smd("A", "N", 3.0, 5.0, 1.0, 1.0)
            + &smd("B", "N", 17.0, 5.0, 1.0, 1.0)
            + "(segment (start 3 5) (end 3 8) (width 0.25) (locked yes) (layer \"F.Cu\") (net \"N\"))\n"
            + "(gr_rect (start 0 0) (end 20 12) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        let at_end = |p: P| geom::dist(p, (3.0, 8.0)) < 1e-6;
        assert!(
            r.tracks.iter().any(|t| at_end(t.a) || at_end(t.b)),
            "no track ends on the stub's end"
        );
        assert!(!r
            .tracks
            .iter()
            .any(|t| geom::dist(t.a, (3.0, 5.0)) < 1e-6 || geom::dist(t.b, (3.0, 5.0)) < 1e-6));
    }

    #[test]
    fn copper_keeps_the_hole_clearance_from_a_plain_hole() {
        // the board asks 0.4 mm from holes, more than the 0.2 mm between copper
        let s = String::from(HEAD)
            + &smd("A", "N", 2.0, 5.0, 1.0, 1.0)
            + &smd("B", "N", 18.0, 5.0, 1.0, 1.0)
            + "(footprint \"H\" (at 10 5) (property \"Reference\" \"H1\") (pad \"\" np_thru_hole circle (at 0 0) (size 2 2) (drill 2) (layers \"*.Cu\" \"*.Mask\")))\n"
            + "(gr_rect (start 0 2) (end 20 8) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let mut rules = rules();
        rules.hole_clearance = 0.4;
        let r = route(&b, &rules, &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        for t in &r.tracks {
            let d = geom::seg_shape_dist(t.a, t.b, &b.holes[0]) - t.width / 2.0;
            assert!(d >= 0.4 - 1e-6, "a track passes {:.3} mm from the hole", d);
        }
    }

    #[test]
    fn inner_planes_are_left_whole() {
        // four layers, In1 a GND plane: the crossing nets may use F, In2 and B, not In1
        let mut s = String::from(
            "(kicad_pcb (version 20260206) (layers (0 \"F.Cu\" signal) (4 \"In1.Cu\" signal) (6 \"In2.Cu\" signal) (2 \"B.Cu\" signal) (25 \"Edge.Cuts\" user))\n",
        );
        for (k, (net, x, y)) in [("H", 1.0, 10.0), ("H", 29.0, 10.0), ("V", 15.0, 1.0), ("V", 15.0, 19.0)]
            .iter()
            .enumerate()
        {
            s += &smd(&format!("J{}", k), net, *x, *y, 0.8, 0.8);
        }
        s += "(zone (net \"GND\") (layer \"In1.Cu\") (polygon (pts (xy 0.5 0.5) (xy 29.5 0.5) (xy 29.5 19.5) (xy 0.5 19.5))))\n";
        s += "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        assert_eq!(plane_layers(&b), vec![1]);
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(r.tracks.iter().all(|t| t.layer != 1), "a track was laid on the plane");
        assert!(!r.vias.is_empty());
    }

    #[test]
    fn the_grid_margin_covers_the_gap_between_grid_points() {
        // a point r from both ends of a diagonal step is r0 from the step's middle
        for (r0, g) in [(0.325, 0.25), (0.325, 1.0), (1.2, 0.5)] {
            let r = r0 + grid_margin(r0, g);
            let mid = (r * r - g * g / 2.0).sqrt();
            assert!((mid - r0).abs() < 1e-9);
            assert!(
                r > g * std::f64::consts::FRAC_1_SQRT_2,
                "two tracks could cross between grid points"
            );
        }
    }
}
