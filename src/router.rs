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
use crate::pair;
use crate::rules::{Item, Rules};
use crate::tune;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

mod fanout;
use fanout::Fan;

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
    /// Blind and buried vias the board house can make, each as (first, last)
    /// copper layer, counted from the top (indices into Board::copper). Through
    /// vias are always allowed; empty: through vias only.
    pub via_spans: Vec<(usize, usize)>,
    /// Bring out the pads the grid cannot reach (the inner balls of a BGA,
    /// tight fine-pitch pads) with an escape track or a dog-bone via, placed
    /// exactly.
    pub fanout: bool,
    /// Join the pads of a net that has an inner plane to that plane, with a
    /// via each (or a track to copper that is on it already). Off: such a net
    /// is left alone, as a net with a pour.
    pub plane_vias: bool,
    /// How far from its pad a fan-out may end, in mm.
    pub fanout_length: f64,
    /// Route nets named as differential pairs (`D+`/`D-`, `_P`/`_N`) as pairs.
    pub diff_pairs: bool,
    /// Only these nets as pairs (wildcards; either half's name will do).
    pub only_pairs: Option<Vec<String>>,
    /// Width of a pair's tracks and the gap between them (None: the net class's
    /// pair settings, else its track width and the clearance between the two nets).
    pub pair_width: Option<f64>,
    pub pair_gap: Option<f64>,
    /// Bring the two halves of each differential pair within this of each
    /// other in length (None: what a skew rule of the board says, if any).
    pub skew: Option<f64>,
    /// Groups of nets (wildcards) to bring to the length of the longest of the group.
    pub match_groups: Vec<Vec<String>>,
    /// The tallest bump of a meander, in mm.
    pub meander: f64,
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
            via_spans: Vec::new(),
            fanout: true,
            plane_vias: true,
            fanout_length: 2.0,
            diff_pairs: true,
            only_pairs: None,
            pair_width: None,
            pair_gap: None,
            skew: None,
            match_groups: Vec::new(),
            meander: 1.0,
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
    /// The first and last copper layer it joins (indices into Board::copper).
    pub layers: (usize, usize),
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

/// A net whose length was asked for: by a rule, to match the other half of
/// its pair, or to match its group.
#[derive(Clone, Debug)]
pub struct LengthReport {
    pub net: String,
    pub why: String,
    /// Its tracks' length as routed, the length asked for, and what it has now.
    pub before: f64,
    pub target: f64,
    pub after: f64,
}

/// How a differential pair came out.
#[derive(Clone, Debug)]
pub struct PairReport {
    pub positive: String,
    pub negative: String,
    /// False when no part of the two nets was routed side by side; `note` says why.
    pub as_pair: bool,
    pub note: String,
    /// The stretches between the pair's ends, and how many were routed side by
    /// side (the others, one net at a time).
    pub legs: usize,
    pub coupled_legs: usize,
    pub width: f64,
    pub gap: f64,
    /// Length routed side by side, and each net's whole length (the tracks the
    /// board already had and the new ones), in mm.
    pub coupled: f64,
    pub length_positive: f64,
    pub length_negative: f64,
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
    /// Differential pairs, routed as such or not.
    pub pairs: Vec<PairReport>,
    /// Nets with a length to meet, and how near they came.
    pub lengths: Vec<LengthReport>,
    /// Things the user should know about the board or the result.
    pub warnings: Vec<String>,
    /// Nets joined to an inner plane of their own: (net, the plane's layers,
    /// pieces that were not on it yet).
    pub planes: Vec<(String, Vec<usize>, usize)>,
    /// Pads brought out by a fan-out that the routing went on from.
    pub fanouts: Vec<FanReport>,
    /// Pads the grid could not reach and no fan-out fits either: (pad, net, why).
    pub shut_in: Vec<(String, String, String)>,
    pub log: Vec<String>,
}

/// A pad the grid could not reach, brought out to where it could.
#[derive(Clone, Debug)]
pub struct FanReport {
    pub net: String,
    /// The pad, as "U1.B2".
    pub pad: String,
    /// Length of its track, from the pad's centre.
    pub length: f64,
    /// The layers its via joins, if it ends in one.
    pub via: Option<(usize, usize)>,
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

#[derive(Clone)]
struct Terminal {
    label: String,
    centre: P,
    /// Shapes of this piece's copper: (shape, layers, is a pad).
    parts: Vec<(Shape, Vec<usize>, bool)>,
}

struct Job {
    net: String,
    /// The track width on each copper layer, and the widest of them.
    widths: Vec<f64>,
    /// The via spans it may use, shortest first (a through via among them, unless ruled out).
    spans: Vec<Span>,
    /// Inner layers the net has a plane on. Its first terminal is then that
    /// plane: a via that comes down on it anywhere joins it.
    plane: Vec<usize>,
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
    /// Some: this job routes two nets as a differential pair (`net` is the positive one).
    pair: Option<PairSpec>,
}

/// One end of a differential pair: where its two tracks finish.
#[derive(Clone)]
struct PairEnd {
    label: String,
    p: P,
    n: P,
    layers: Vec<usize>,
    /// Which piece of the positive net's job, and of the negative's, this end is.
    piece_p: usize,
    piece_n: usize,
}

#[derive(Clone)]
struct PairSpec {
    /// The jobs that route the two nets one by one, should the pair not work out.
    job_p: usize,
    job_n: usize,
    net_n: String,
    /// Width of each track and the gap between them: on the first layer (for
    /// reports), and on each layer.
    w: f64,
    #[allow(dead_code)]
    gap: f64,
    ws: Vec<f64>,
    gaps: Vec<f64>,
    /// One via of the two (diameter, drill), their least pitch, and how far to
    /// its side of the path each must stand for the other track to pass it.
    via: Option<(f64, f64)>,
    via_pitch: f64,
    via_side: f64,
    /// The least copper gap the two nets owe each other anywhere.
    pn: f64,
    /// The ends, in the order they are joined.
    ends: Vec<PairEnd>,
}

/// Stretches of a pair shorter than this (mm, between the middles of their
/// ends) are routed one net at a time.
const SHORTEST_PAIR: f64 = 4.0;

/// Loose ends of a piece's tracks: (point, layer, track width). A track end is
/// loose when no other copper of the piece holds it.
fn loose_ends(parts: &[(Shape, Vec<usize>, bool)]) -> Vec<(P, usize, f64)> {
    let mut loose = Vec::new();
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
    loose
}

/// Work out how two nets pair up: which piece of one goes with which of the
/// other, and in what order to join the ends.
fn pair_ends(p: &Job, n: &Job) -> Result<Vec<PairEnd>, String> {
    let attach = |t: &Terminal| -> (P, Vec<usize>) {
        match loose_ends(&t.parts).first() {
            Some(&(end, l, _)) => (end, vec![l]),
            None => {
                let layers = t.parts.iter().find(|x| x.2).map(|x| x.1.clone()).unwrap_or_default();
                (t.centre, layers)
            }
        }
    };
    if p.terminals.len() != n.terminals.len() {
        return Err(format!(
            "one net has {} pieces to join and the other {}",
            p.terminals.len(),
            n.terminals.len()
        ));
    }
    let mut used = vec![false; n.terminals.len()];
    let mut ends: Vec<PairEnd> = Vec::new();
    for (piece_p, t) in p.terminals.iter().enumerate() {
        let (pp, pl) = attach(t);
        let best = (0..n.terminals.len())
            .filter(|&k| !used[k])
            .map(|k| (geom::dist(pp, attach(&n.terminals[k]).0), k))
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)));
        let (d, k) = best.ok_or("no pads to pair up")?;
        if d > 10.0 {
            return Err(format!("{} has no pad of the other net within 10 mm", t.label));
        }
        used[k] = true;
        let (np, nlay) = attach(&n.terminals[k]);
        let layers: Vec<usize> = pl.iter().cloned().filter(|l| nlay.contains(l)).collect();
        if layers.is_empty() {
            return Err(format!(
                "{} and {} are on different layers",
                t.label, n.terminals[k].label
            ));
        }
        ends.push(PairEnd {
            label: format!("{} / {}", t.label, n.terminals[k].label),
            p: pp,
            n: np,
            layers,
            piece_p,
            piece_n: k,
        });
    }
    // a pair is a chain, not a tree: from the outermost end, nearest next
    let mid = |e: &PairEnd| ((e.p.0 + e.n.0) / 2.0, (e.p.1 + e.n.1) / 2.0);
    let c = (
        ends.iter().map(|e| mid(e).0).sum::<f64>() / ends.len() as f64,
        ends.iter().map(|e| mid(e).1).sum::<f64>() / ends.len() as f64,
    );
    let first = (0..ends.len())
        .max_by(|&a, &b| {
            geom::dist(mid(&ends[a]), c)
                .partial_cmp(&geom::dist(mid(&ends[b]), c))
                .unwrap()
                .then(b.cmp(&a))
        })
        .unwrap_or(0);
    let mut order = vec![ends.remove(first)];
    while !ends.is_empty() {
        let at = mid(&order[order.len() - 1]);
        let k = (0..ends.len())
            .min_by(|&a, &b| {
                geom::dist(mid(&ends[a]), at)
                    .partial_cmp(&geom::dist(mid(&ends[b]), at))
                    .unwrap()
                    .then(a.cmp(&b))
            })
            .unwrap();
        order.push(ends.remove(k));
    }
    Ok(order)
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
    vias: Vec<ViaAt>,
    unrouted: Vec<String>,
    necked: Vec<(String, f64)>,
    /// For a pair: the negative net's tracks, each net's real vias (true: the
    /// negative net's), and the length routed side by side.
    tracks_n: Vec<(usize, Vec<P>, f64)>,
    real_vias: Vec<(P, bool, Span)>,
    coupled: f64,
    leg: Option<pair::Leg>,
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
    /// How each state was reached: the via span used (an index into the job's
    /// spans), or NO_VIA for a step on its own layer.
    how: Vec<u8>,
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
            how: vec![NO_VIA; states],
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

const NO_VIA: u8 = u8::MAX;

/// A via's reach: the first and last copper layer it joins. A through via
/// spans them all; a blind or buried one fewer, and takes up room only on
/// the layers it spans.
type Span = (usize, usize);

/// A via laid: where, and the layers it spans.
type ViaAt = (P, Span);

struct Search<'a> {
    block: &'a [Vec<u8>],
    /// Per layer: where a via's copper may not be. None: no vias.
    vblock: Option<&'a [Vec<u8>]>,
    soft: &'a [Vec<u16>],
    /// Per layer: other jobs' copper within a via's reach.
    soft_via: &'a [Vec<u16>],
    /// The via spans this job may use, shortest first.
    spans: &'a [Span],
    /// How many steps a path must go on from one via before its next, where
    /// the two would share a layer (two holes want their distance).
    keep: usize,
    /// Per layer: a plane of the net's own. A path may come down on it or
    /// leave it by a via, but lays no track there.
    flat: &'a [bool],
    hist: &'a [Vec<f32>],
    pres: f64,
    via_cost: f64,
    limit: usize,
}

/// Whether the path to `st` came through a via, a few steps back at most, that
/// shares a layer with `span`.
fn recent_via(wk: &Work, s: &Search, mut st: usize, span: Span) -> bool {
    for _ in 0..s.keep {
        let k = wk.how[st];
        if k != NO_VIA {
            let (a, b) = s.spans[k as usize];
            if a <= span.1 && span.0 <= b {
                return true;
            }
        }
        match wk.came[st] {
            u32::MAX => return false,
            from => st = from as usize,
        }
    }
    false
}

/// The cheapest path from any source to any target: its states, and for each
/// the via span it was reached by (NO_VIA for a step along a layer).
fn astar(
    gr: &Grid,
    wk: &mut Work,
    s: &Search,
    sources: &[u32],
    targets: &[u32],
    goal: P,
) -> Option<(Vec<u32>, Vec<u8>)> {
    let (w, h, n) = (gr.w as i64, gr.h as i64, gr.n);
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
        wk.how[st as usize] = NO_VIA;
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
            let mut how = vec![wk.how[st as usize]];
            let mut cur = st;
            while wk.came[cur as usize] != u32::MAX {
                cur = wk.came[cur as usize];
                path.push(cur);
                how.push(wk.how[cur as usize]);
            }
            path.reverse();
            how.reverse();
            how[0] = NO_VIA;
            return Some((path, how));
        }
        expanded += 1;
        if expanded > s.limit {
            return None;
        }
        let l = su / n;
        let idx = su % n;
        let (i, j) = ((idx as i64) % w, (idx as i64) / w);
        let (m, o, hl) = (&s.block[l], &s.soft[l], &s.hist[l]);
        let relax = |wk: &mut Work, ns: usize, ng: f64, how: u8| {
            if wk.seen[ns] != gen || ng < wk.best[ns] {
                wk.seen[ns] = gen;
                wk.best[ns] = ng;
                wk.came[ns] = st;
                wk.how[ns] = how;
                wk.heap.push(Entry(ng + heur(ns as u32), ng, ns as u32));
            }
        };
        for &(di, dj, c) in DIRS.iter() {
            if s.flat[l] {
                break;
            }
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
            relax(wk, l * n + nidx, ng, NO_VIA);
        }
        if let Some(vb) = s.vblock {
            // a via here, of any span that takes in this layer and has room on all its layers
            for (k, &(a, b)) in s.spans.iter().enumerate() {
                if l < a || l > b || (a..=b).any(|x| vb[x][idx] != 0) {
                    continue;
                }
                if s.spans.len() > 1 && recent_via(wk, s, su, (a, b)) {
                    continue;
                }
                let hsum: f64 = (a..=b).map(|x| s.hist[x][idx] as f64).sum();
                let near: u32 = (a..=b).map(|x| s.soft_via[x][idx] as u32).sum();
                let ng = gc + s.via_cost * (1.0 + hsum) * (1.0 + s.pres * near as f64);
                for l2 in a..=b {
                    if l2 != l && s.block[l2][idx] == 0 {
                        relax(wk, l2 * n + idx, ng, k as u8);
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
    hard: Vec<Option<(Vec<Vec<u8>>, Option<Vec<Vec<u8>>>)>>,
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
    /// (job, signature, via, layer) -> how far that job's track (on that layer)
    /// or via must stay from a job of that signature's copper
    radii: HashMap<(usize, usize, bool, usize), Option<f64>>,
    /// Jobs in play. The jobs that route a stretch of a pair one net at a time
    /// sleep unless the pair cannot be had there.
    active: Vec<bool>,
    /// job -> the other jobs that share a net with it (stretches of one pair)
    sibs: Vec<Vec<usize>>,
    /// (positive net, why) for each stretch of a pair that was routed one by one
    pair_notes: Vec<(String, String)>,
    ngroups: usize,
    /// Inner layers given over to a pour: no tracks there, so the plane stays whole.
    planes: Vec<usize>,
    /// Fan-outs the board was given before this routing (as kept copper).
    fans: Vec<Fan>,
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

    /// A copper layer as the rules see it.
    fn layer(&self, l: usize) -> crate::rules::Layer<'a> {
        crate::rules::Layer {
            name: self.board.copper[l].as_str(),
            outer: l == 0 || l + 1 == self.board.copper.len(),
        }
    }

    /// Walls for job j: per layer, where a track's middle may not be, and
    /// where a via's may not (a via needs room only on the layers it spans).
    fn hard(&mut self, j: usize) -> (Vec<Vec<u8>>, Option<Vec<Vec<u8>>>) {
        if let Some(h) = &self.hard[j] {
            return h.clone();
        }
        let (nl, n) = (self.gr.nl, self.gr.n);
        let net = self.jobs[j].net.clone();
        // half the track's width, on each layer
        let hws: Vec<f64> = self.jobs[j].widths.iter().map(|w| w / 2.0).collect();
        let hw = self.jobs[j].width / 2.0;
        let layered = self.rules.layered();
        let vr = self.jobs[j].via.map(|v| v.0 / 2.0);
        let m = self.jobs[j].margin;
        let mut block: Vec<Vec<u8>> = (0..nl).map(|_| self.outside.clone()).collect();
        let mut vblock: Option<Vec<Vec<u8>>> = vr.map(|_| (0..nl).map(|_| self.outside.clone()).collect());
        for l in 0..nl {
            if !self.layers_ok(l) {
                block[l] = vec![1; n];
            }
        }
        // the net's own plane: a via may come down anywhere inside it, a via's
        // radius from its edge (no track is laid there: the search sees to that)
        for &l in &self.jobs[j].plane {
            block[l] = vec![1; n];
            let inset = vr.unwrap_or(0.0) + m;
            for z in self
                .board
                .zones
                .iter()
                .filter(|z| z.keepout.is_none() && z.net.as_deref() == Some(net.as_str()) && z.layers.contains(&l))
            {
                let (mut x0, mut y0, mut x1, mut y1) =
                    (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                for p in &z.outline {
                    x0 = x0.min(p.0);
                    y0 = y0.min(p.1);
                    x1 = x1.max(p.0);
                    y1 = y1.max(p.1);
                }
                let (i0, j0, i1, j1) = self.gr.range((x0, y0, x1, y1), 0.0);
                for jj in j0..=j1 {
                    for ii in i0..=i1 {
                        let c = jj * self.gr.w + ii;
                        let p = self.gr.xy(c);
                        if self.outside[c] == 0
                            && geom::in_polygon(p, &z.outline)
                            && geom::polygon_edge_dist(p, &z.outline) >= inset
                        {
                            block[l][c] = 0;
                        }
                    }
                }
            }
        }
        let ec = self.rules.edge_clearance(&net);
        for &(a, b) in &self.board.edges {
            let s = Shape::Capsule { a, b, r: 0.0 };
            for l in 0..nl {
                let bl = &mut block[l];
                self.gr.each(&s, ec + hws[l] + m, |c| bl[c] = 1);
            }
            if let (Some(vb), Some(vr)) = (vblock.as_mut(), vr) {
                for layer in vb.iter_mut() {
                    self.gr.each(&s, ec + vr + m, |c| layer[c] = 1);
                }
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
                    if k.tracks {
                        for &l in &z.layers {
                            if d <= hws[l] + m {
                                block[l][c] = 1;
                            }
                        }
                    }
                    if let (true, Some(vb), Some(vr)) = (k.vias, vblock.as_mut(), vr) {
                        if d <= vr + m {
                            for &l in &z.layers {
                                vb[l][c] = 1;
                            }
                        }
                    }
                }
            }
        }
        let mut owed = vec![0f32; self.obstacles.len()];
        // the rules are asked once for each group of like obstacles: (for a track, for a via)
        let mut asked: Vec<Option<(f64, f64)>> = vec![None; self.ngroups];
        let mut asked_on: HashMap<(usize, usize), f64> = HashMap::new();
        let me = if self.jobs[j].pair.is_some() {
            Item::track(&net).in_pair()
        } else {
            Item::track(&net)
        };
        // a pair keeps its coupled part off its own two nets' pads (they are reached
        // by a piece of track each, not by the pair as a whole)
        let pair = self.jobs[j].pair.as_ref().map(|p| (p.net_n.clone(), p.pn));
        for (o, ob) in self.obstacles.iter().enumerate() {
            let own = ob.net.as_deref() == Some(net.as_str())
                || pair
                    .as_ref()
                    .map(|(n, _)| ob.net.as_deref() == Some(n.as_str()))
                    .unwrap_or(false);
            if own && pair.is_none() {
                continue;
            }
            let (ct, cv) = match (own, &pair, asked[ob.group]) {
                (true, Some((_, pn)), _) => (*pn, *pn),
                (_, _, Some(v)) => v,
                _ => {
                    let other = Item {
                        net: ob.net.as_deref(),
                        kind: Some(ob.kind),
                        shape: Some(&ob.shape),
                        layer: None,
                        paired: None,
                    };
                    let v = (
                        self.rules.clearance(&me, &other),
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
            let mut most = ct.max(ob.least);
            for &l in &ob.layers {
                // where a rule tells layers apart, the clearance is this layer's own
                let clr = if layered && !(own && pair.is_some()) {
                    let key = (ob.group, l);
                    match asked_on.get(&key) {
                        Some(v) => *v,
                        None => {
                            let layer = Some(self.layer(l));
                            let other = Item {
                                net: ob.net.as_deref(),
                                kind: Some(ob.kind),
                                shape: Some(&ob.shape),
                                layer,
                                paired: None,
                            };
                            let v = self.rules.clearance(&me.on(layer), &other);
                            asked_on.insert(key, v);
                            v
                        }
                    }
                    .max(ob.least)
                } else {
                    ct.max(ob.least)
                };
                most = most.max(clr);
                let bl = &mut block[l];
                self.gr.each(&ob.shape, clr + hws[l] + m, |c| bl[c] = 1);
            }
            owed[o] = most as f32;
            if let (Some(vr), Some(vb)) = (vr, vblock.as_mut()) {
                for &l in &ob.layers {
                    let layer = &mut vb[l];
                    self.gr.each(&ob.shape, cv.max(ob.least) + vr + m, |c| layer[c] = 1);
                }
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
                        // (another via of the net, through or not: no new one on top of it)
                        (false, _, Shape::Circle { .. }) => vd / 2.0,
                        _ => continue,
                    };
                    for &l in layers {
                        let layer = &mut vb[l];
                        self.gr.each(shape, r, |c| layer[c] = 1);
                    }
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
        let partner = self.jobs[j].pair.as_ref().map(|p| p.net_n.as_str());
        let owed = &self.owed[j];
        let mut near: Vec<(usize, f64)> = Vec::new();
        for (o, ob) in self.obstacles.iter().enumerate() {
            if ob.net.as_deref() == Some(net) || (partner.is_some() && ob.net.as_deref() == partner) {
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
        let spans = self.jobs[j].spans.clone();
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
                if let Some(vb) = vblock.as_ref() {
                    for &(a, b) in &spans {
                        if l < a || l > b || (a..=b).any(|x| vb[x][c] != 0) {
                            continue;
                        }
                        for l2 in a..=b {
                            if l2 != l {
                                go(l2 * n + c, &mut label, &mut stack);
                            }
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
        let (widths, neck) = (self.jobs[j].widths.clone(), self.jobs[j].neck);
        let hw = self.jobs[j].width / 2.0; // (the widest, for how far to look)
        let edge = self.rules.edge_clearance(&net);
        let beside = 1.5 * g;
        let neck_len = if neck.is_some() { self.opt.neck_length } else { 0.0 };
        let room = self.room(j);
        let roomy = room.iter().cloned().max().unwrap_or(0) / 20; // a twentieth of the largest region
        let mut all: Vec<Vec<Entry2>> = Vec::new();
        for t in 0..self.jobs[j].terminals.len() {
            let parts = self.jobs[j].terminals[t].parts.clone();
            // a pad the fan-out brought to a via: the net goes on from that via, on
            // another layer (on the pad's own it was shut in)
            let fan_via = self.fans.iter().find_map(|f| match f.via {
                Some((at, d, _, span))
                    if f.net == net
                        && parts
                            .iter()
                            .any(|(s, _, pad)| !*pad && matches!(s, Shape::Circle { c, .. } if *c == at)) =>
                {
                    Some((at, d, span, f.layer))
                }
                _ => None,
            });
            if let Some((at, d, (a, b), pad_layer)) = fan_via {
                let mut st: Vec<Entry2> = Vec::new();
                for l in (a..=b).filter(|&l| l != pad_layer) {
                    if !self.layers_ok(l) {
                        continue;
                    }
                    let near = self.near(j, (at.0, at.1, at.0, at.1), hw + beside);
                    let mut cells: Vec<usize> = Vec::new();
                    let bl = &block[l];
                    self.gr
                        .each(&Shape::Circle { c: at, r: 0.0 }, (d / 2.0).max(beside), |c| {
                            if bl[c] == 0 {
                                cells.push(c);
                            }
                        });
                    for c in cells {
                        if self.stub_ok(widths[l] / 2.0, self.gr.xy(c), at, l, &near, edge) {
                            st.push(((l * n + c) as u32, Some((at, widths[l]))));
                        }
                    }
                }
                st.sort_by(|a, b| a.0.cmp(&b.0));
                st.dedup_by(|a, b| a.0 == b.0);
                all.push(st);
                continue;
            }
            let loose = loose_ends(&parts);
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
                    if self.stub_ok(widths[l] / 2.0, self.gr.xy(c), end, l, &near, edge) {
                        st.push(((l * n + c) as u32, Some((end, widths[l]))));
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
                            if self.stub_ok(widths[l] / 2.0, self.gr.xy(c), centre, l, &near, edge) {
                                st.push(((l * n + c) as u32, Some((centre, widths[l]))));
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
                    if let Some(nw) = neck.filter(|nw| *nw < widths[l] - 1e-9) {
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
    fn radius(&mut self, j: usize, sig: usize, via: bool, layer: usize) -> Option<f64> {
        // (a via is the same on every layer)
        let key = (j, sig, via, if via { 0 } else { layer });
        if let Some(r) = self.radii.get(&key) {
            return *r;
        }
        let r = self.radius_of(j, sig, via, layer);
        self.radii.insert(key, r);
        r
    }

    fn radius_of(&mut self, j: usize, sig: usize, via: bool, layer: usize) -> Option<f64> {
        let rep = self
            .jobs
            .iter()
            .enumerate()
            .find(|(k, jb)| *k != j && jb.sig == sig)
            .map(|(_, jb)| jb.net.clone())?;
        let net = self.jobs[j].net.clone();
        let mut c = if via || !self.rules.layered() {
            let me = if via { Item::via(&net) } else { Item::track(&net) };
            self.clearance(me, Item::track(&rep))
                .max(self.clearance(me, Item::via(&rep)))
        } else {
            // a rule tells layers apart: this layer's own clearance
            let on = Some(self.layer(layer));
            let me = Item::track(&net).on(on);
            self.rules
                .clearance(&me, &Item::track(&rep).on(on))
                .max(self.rules.clearance(&me, &Item::via(&rep).on(on)))
        };
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
            self.jobs[j].widths[layer] / 2.0
        };
        Some(c + half + self.jobs[j].margin)
    }

    /// The cells, per layer and each once, within r of job k's copper.
    fn contribution(&mut self, k: usize, r: f64) -> Vec<Vec<u32>> {
        self.contribution_of(k, r, 0)
    }

    /// The same for part of job k's copper: 0 all of it, 1 the tracks of its
    /// (positive) net, 2 the tracks of a pair's negative net, 3 its vias.
    fn contribution_of(&mut self, k: usize, r: f64, part: u8) -> Vec<Vec<u32>> {
        let key = um(r) * 4 + part as i64;
        if let Some(c) = self.contrib[k].get(&key) {
            return c.clone();
        }
        let (nl, n) = (self.gr.nl, self.gr.n);
        let mark = self.work.next_mark();
        let mut out: Vec<Vec<u32>> = vec![Vec::new(); nl];
        if let Some(laid) = &self.laid[k] {
            let none: Vec<(usize, Vec<P>, f64)> = Vec::new();
            let (first, second) = match part {
                0 => (&laid.tracks, &laid.tracks_n),
                1 => (&laid.tracks, &none),
                2 => (&laid.tracks_n, &none),
                _ => (&none, &none),
            };
            for (l, pts, w) in first.iter().chain(second.iter()) {
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
            if let (Some((d, _)), true) = (self.jobs[k].via, part == 0 || part == 3) {
                for &(p, (a, b)) in &laid.vias {
                    let sh = Shape::Circle { c: p, r: d / 2.0 };
                    for l in a..=b {
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

    /// The part of sibling k's copper that is job j's own net (see `sibs`), as
    /// (the part that is shared, the parts that are not). None: all of it is.
    fn shared_with(&self, j: usize, k: usize) -> Option<(u8, [u8; 2])> {
        match (&self.jobs[j].pair, &self.jobs[k].pair) {
            (None, Some(_)) => Some(if self.jobs[k].net == self.jobs[j].net {
                (1, [2, 3])
            } else {
                (2, [1, 3])
            }),
            _ => None,
        }
    }

    /// The cells within r of sibling k's copper that job j need not keep clear
    /// of, being its own net's: all of them, or for one net against a pair,
    /// those near its own net's track and not near the other's.
    fn own_in_sibling(&mut self, j: usize, k: usize, r: f64) -> Vec<Vec<u32>> {
        match self.shared_with(j, k) {
            None => self.contribution(k, r),
            Some((mine, others)) => {
                let (nl, n) = (self.gr.nl, self.gr.n);
                let mut theirs = vec![false; n * nl];
                for part in others {
                    for (l, cells) in self.contribution_of(k, r, part).iter().enumerate() {
                        for &c in cells {
                            theirs[l * n + c as usize] = true;
                        }
                    }
                }
                self.contribution_of(k, r, mine)
                    .into_iter()
                    .enumerate()
                    .map(|(l, cells)| cells.into_iter().filter(|&c| !theirs[l * n + c as usize]).collect())
                    .collect()
            }
        }
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

    /// Soft costs for job j: other jobs' copper near each cell, per layer, for
    /// a track of this job and for a via of it.
    fn soft(&mut self, j: usize) -> (Vec<Vec<u16>>, Vec<Vec<u16>>) {
        let (nl, n) = (self.gr.nl, self.gr.n);
        let mut soft: Vec<Vec<u16>> = vec![vec![0u16; n]; nl];
        let mut sv: Vec<Vec<u16>> = vec![vec![0u16; n]; nl];
        for sig in self.signatures() {
            for l in 0..nl {
                if let Some(rt) = self.radius(j, sig, false, l) {
                    let key = self.occupancy(sig, rt);
                    for (d, s) in soft[l].iter_mut().zip(self.occ[&key][l].iter()) {
                        *d = d.saturating_add(*s as u16);
                    }
                }
            }
            if let Some(rv) = self.radius(j, sig, true, 0) {
                let key = self.occupancy(sig, rv);
                let arr = &self.occ[&key];
                for l in 0..nl {
                    for (d, s) in sv[l].iter_mut().zip(arr[l].iter()) {
                        *d = d.saturating_add(*s as u16);
                    }
                }
            }
        }
        if self.laid[j].is_some() {
            // job j's own copper is in its signature's pile: take it out again
            let sig = self.jobs[j].sig;
            for l in 0..nl {
                if let Some(rt) = self.radius(j, sig, false, l) {
                    for &cell in &self.contribution(j, rt)[l] {
                        soft[l][cell as usize] = soft[l][cell as usize].saturating_sub(1);
                    }
                }
            }
            if let Some(rv) = self.radius(j, sig, true, 0) {
                let c = self.contribution(j, rv);
                for l in 0..nl {
                    for &cell in &c[l] {
                        sv[l][cell as usize] = sv[l][cell as usize].saturating_sub(1);
                    }
                }
            }
        }
        // other stretches of the same pair: copper of this job's own net is no obstacle
        for k in self.sibs[j].clone() {
            if self.laid[k].is_none() {
                continue;
            }
            let sig = self.jobs[k].sig;
            for l in 0..nl {
                if let Some(rt) = self.radius(j, sig, false, l) {
                    for &cell in &self.own_in_sibling(j, k, rt)[l] {
                        soft[l][cell as usize] = soft[l][cell as usize].saturating_sub(1);
                    }
                }
            }
            if let Some(rv) = self.radius(j, sig, true, 0) {
                let c = self.own_in_sibling(j, k, rv);
                for l in 0..nl {
                    for &cell in &c[l] {
                        sv[l][cell as usize] = sv[l][cell as usize].saturating_sub(1);
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
        for &(v, (a, b)) in &laid.vias {
            let c = self.gr.cell(v);
            if (a..=b).any(|l| sv[l][c] > 0) {
                bad.push((a * n + c) as u32);
            }
        }
        bad
    }

    /// The length of a net's tracks: those the board already had and those laid.
    fn net_length(&self, net: &str) -> f64 {
        let mut len: f64 = self
            .board
            .tracks
            .iter()
            .filter(|t| t.net.as_deref() == Some(net))
            .map(|t| geom::dist(t.a, t.b))
            .sum();
        for j in 0..self.jobs.len() {
            let laid = match (&self.laid[j], self.active[j]) {
                (Some(l), true) => l,
                _ => continue,
            };
            if self.jobs[j].net == net {
                len += laid.tracks.iter().map(|t| tune::length(&t.1)).sum::<f64>();
            }
            if self.jobs[j].pair.as_ref().map(|p| p.net_n == net).unwrap_or(false) {
                len += laid.tracks_n.iter().map(|t| tune::length(&t.1)).sum::<f64>();
            }
        }
        len
    }

    /// Lengthen a net's tracks by `extra` with meanders, wherever the jobs that
    /// carry it have room. Returns how much was added.
    fn lengthen(&mut self, net: &str, extra: f64) -> f64 {
        let mut left = extra;
        for j in 0..self.jobs.len() {
            if left <= 1e-6 {
                break;
            }
            if !self.active[j] || self.laid[j].is_none() {
                continue;
            }
            let negative = if self.jobs[j].net == net {
                false
            } else if self.jobs[j].pair.as_ref().map(|p| p.net_n == net).unwrap_or(false) {
                true
            } else {
                continue;
            };
            left -= self.lengthen_job(j, negative, left);
        }
        extra - left
    }

    /// Add meanders to job j's tracks (those of a pair's negative net, if
    /// `negative`), up to `extra` longer. Each bump is kept only if it has room:
    /// clear of other nets' copper by the job's own measure, a track's width or
    /// more from the rest of its own net, and for a pair, on the side away from
    /// the other half and clear of it.
    fn lengthen_job(&mut self, j: usize, negative: bool, extra: f64) -> f64 {
        let (gw, gh, g, x0, y0) = (self.gr.w, self.gr.h, self.gr.g, self.gr.x0, self.gr.y0);
        let spec = self.jobs[j].pair.clone();
        let (block, _) = self.hard(j);
        let (soft, _) = self.soft(j);
        let mut laid = self.laid[j].clone().expect("a routed job");
        let net = match (&spec, negative) {
            (Some(s), true) => s.net_n.clone(),
            _ => self.jobs[j].net.clone(),
        };
        // the track's width on each layer; and a pair is checked as the one wide
        // track it was routed as, about its middle: how far that is, on each layer
        let nl = self.gr.nl;
        let widths: Vec<f64> = match &spec {
            Some(s) => s.ws.clone(),
            None => self.jobs[j].widths.clone(),
        };
        let shifts: Vec<f64> = match &spec {
            Some(s) => (0..nl).map(|l| (s.ws[l] + s.gaps[l]) / 2.0).collect(),
            None => vec![0.0; nl],
        };
        let pn = spec.as_ref().map(|s| s.pn).unwrap_or(0.0);
        let own_clearance = self.rules.class_of(&net).clearance;
        let tallest = self.opt.meander;
        // the rest of this net's copper, and the other half's, here and in the other
        // stretches of the pair: (layer, a, b, width)
        let mut own: Vec<(usize, P, P, f64)> = Vec::new();
        let mut other: Vec<(usize, P, P, f64)> = Vec::new();
        let mut other_vias: Vec<(P, f64)> = Vec::new();
        for k in std::iter::once(j).chain(self.sibs[j].iter().cloned()) {
            let lk = match (&self.laid[k], self.active[k]) {
                (Some(l), true) => l,
                _ => continue,
            };
            let kn = self.jobs[k].pair.as_ref().map(|p| p.net_n.as_str());
            for (their, tracks) in [(Some(self.jobs[k].net.as_str()), &lk.tracks), (kn, &lk.tracks_n)] {
                let list = match their {
                    Some(t) if t == net => &mut own,
                    Some(_) => &mut other,
                    None => continue,
                };
                for (l, pts, w) in tracks.iter() {
                    list.extend(pts.windows(2).map(|s| (*l, s[0], s[1], *w)));
                }
            }
            if let Some(ks) = &self.jobs[k].pair {
                let d = ks.via.map(|v| v.0).unwrap_or(0.0);
                let theirs_negative = ks.net_n != net;
                other_vias.extend(lk.real_vias.iter().filter(|v| v.1 == theirs_negative).map(|v| (v.0, d)));
            }
        }
        let cells = |v: f64, origin: f64, most: usize| -> Vec<usize> {
            // the grid lines either side of v (the one it is on, if it is on one)
            let f = (v - origin) / g;
            let r = f.round();
            let list = if (f - r).abs() < 1e-6 {
                vec![r]
            } else {
                vec![f.floor(), f.floor() + 1.0]
            };
            list.into_iter()
                .filter(|c| *c >= 0.0 && (*c as usize) < most)
                .map(|c| c as usize)
                .collect()
        };
        let mut added_segs: Vec<(usize, P, P)> = Vec::new();
        let mut total = 0.0;
        let which = if negative { 1 } else { 0 };
        let count = if negative {
            laid.tracks_n.len()
        } else {
            laid.tracks.len()
        };
        for i in 0..count {
            if extra - total <= 1e-6 {
                break;
            }
            let (l, pts, w) = if which == 1 {
                laid.tracks_n[i].clone()
            } else {
                laid.tracks[i].clone()
            };
            let (width, shift) = (widths[l], shifts[l]);
            if (w - width).abs() > 1e-9 {
                continue; // a narrowed piece into a pad
            }
            let apart = width + width.max(own_clearance);
            let m = tune::Meander {
                spacing: apart,
                amplitude: tallest,
            };
            let mut ok = |b: &tune::Bump| -> bool {
                let out = {
                    let d = geom::dist(b[0], b[1]);
                    ((b[1].0 - b[0].0) / d, (b[1].1 - b[0].1) / d)
                };
                for s in b.windows(2) {
                    // room on the grid, by the job's own walls and the others' copper
                    let len = geom::dist(s[0], s[1]);
                    let steps = ((len / (g / 2.0)).ceil() as usize).max(1);
                    for t in 0..=steps {
                        let f = t as f64 / steps as f64;
                        let p = (
                            s[0].0 + f * (s[1].0 - s[0].0) - out.0 * shift,
                            s[0].1 + f * (s[1].1 - s[0].1) - out.1 * shift,
                        );
                        let (ci, cj) = (cells(p.0, x0, gw), cells(p.1, y0, gh));
                        if ci.is_empty() || cj.is_empty() {
                            return false;
                        }
                        for &jj in &cj {
                            for &ii in &ci {
                                let c = jj * gw + ii;
                                if block[l][c] != 0 || soft[l][c] != 0 {
                                    return false;
                                }
                            }
                        }
                    }
                    // the rest of its own net: not so near that the bump is shorted out
                    for &(ol, a, c, _) in &own {
                        if ol != l {
                            continue;
                        }
                        // (the side the bump stands on holds both its feet: that one is not "the rest")
                        if geom::seg_dist(b[0], a, c) < 1e-6 && geom::seg_dist(b[3], a, c) < 1e-6 {
                            continue;
                        }
                        if geom::seg_seg_dist(s[0], s[1], a, c) < apart - 1e-6 {
                            return false;
                        }
                    }
                    for &(al, a, c) in &added_segs {
                        if al == l
                            && geom::seg_seg_dist(s[0], s[1], a, c) < apart - 1e-6
                            && !(geom::dist(s[0], a) < 1e-9 || geom::dist(s[1], c) < 1e-9)
                        {
                            return false;
                        }
                    }
                    // the other half of the pair
                    for &(ol, a, c, ow) in &other {
                        if ol == l && geom::seg_seg_dist(s[0], s[1], a, c) < width / 2.0 + ow / 2.0 + pn - 1e-6 {
                            return false;
                        }
                    }
                    for &(v, d) in &other_vias {
                        if geom::seg_dist(v, s[0], s[1]) < d / 2.0 + width / 2.0 + pn - 1e-6 {
                            return false;
                        }
                    }
                }
                added_segs.extend(b.windows(2).map(|s| (l, s[0], s[1])));
                true
            };
            let (new, added) = tune::lengthen(&pts, extra - total, &m, &[1.0, -1.0], &mut ok);
            if added > 0.0 {
                total += added;
                if which == 1 {
                    laid.tracks_n[i].1 = new;
                } else {
                    laid.tracks[i].1 = new;
                }
            }
        }
        if total > 0.0 {
            self.add(j, -1);
            self.laid[j] = Some(laid);
            self.contrib[j].clear();
            self.add(j, 1);
        }
        total
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
        let widths = self.jobs[j].widths.clone();
        let (block, vblock) = self.hard(j);
        let vblock = match vblock {
            Some(v) => v,
            None => return (0, Vec::new()),
        };
        let (soft, sv) = self.soft(j);
        // two holes no nearer than the board's hole-to-hole gap, edge to edge
        let apart = drill + self.rules.hole_to_hole;
        let mut holes: Vec<P> = laid.vias.iter().map(|v| v.0).collect();
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
        let mut new_vias: Vec<ViaAt> = Vec::new();
        let mut single: Vec<P> = Vec::new();
        for &(p, (a, b)) in &laid.vias {
            let c = self.gr.cell(p) as i64;
            // the layers the net's tracks meet this via on
            let layers: Vec<usize> = (a..=b)
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
                if (a..=b).any(|l| vblock[l][c2] != 0 || sv[l][c2] != 0) {
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
                    new_tracks.push((l, vec![p, p2], widths[l]));
                }
                new_vias.push((p2, (a, b)));
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
        for &(v, (a, b)) in &laid.vias {
            let c = self.gr.cell(v);
            for l in a..=b {
                mine[l * n + c] |= 2;
            }
        }
        let mut out = Vec::new();
        for k in 0..self.jobs.len() {
            if k == j || self.laid[k].is_none() {
                continue;
            }
            let sig = self.jobs[k].sig;
            let sibling = self.sibs[j].contains(&k);
            if sibling && self.shared_with(j, k).is_none() {
                continue; // all of it is this job's own net
            }
            let mut hit = false;
            // (a track against the other's copper layer by layer, each at its own
            // width there; a via against all of it)
            let tries: Vec<(bool, u8, Option<usize>)> = (0..nl)
                .map(|l| (false, 1u8, Some(l)))
                .chain(std::iter::once((true, 2u8, None)))
                .collect();
            for (via, bit, only) in tries {
                if hit {
                    break;
                }
                if let Some(r) = self.radius(j, sig, via, only.unwrap_or(0)) {
                    let c = match (sibling, self.shared_with(j, k)) {
                        (true, Some((_, others))) => {
                            let mut both = self.contribution_of(k, r, others[0]);
                            for (l, more) in self.contribution_of(k, r, others[1]).into_iter().enumerate() {
                                both[l].extend(more);
                            }
                            both
                        }
                        _ => self.contribution(k, r),
                    };
                    hit = c.iter().enumerate().any(|(l, cells)| {
                        only.map(|o| o == l).unwrap_or(true)
                            && cells.iter().any(|&cell| mine[l * n + cell as usize] & bit != 0)
                    });
                }
            }
            if hit {
                out.push(k);
            }
        }
        out
    }

    /// Whether a leg's copper keeps every clearance: to other nets' copper
    /// (`near`) and the board edge, between the pair's two tracks, and from
    /// each track to the other net's own pads.
    fn pair_legal(
        &self,
        j: usize,
        spec: &PairSpec,
        leg: &pair::Leg,
        spans: &[Span],
        near: &[(usize, f64)],
    ) -> Result<(), String> {
        self.pair_legal_with(j, spec, leg, spans, near, 0.0)
    }

    /// `spans`: the layers each of the leg's vias joins, in the leg's order (a
    /// via not listed goes right through). `slack`: extra room to ask for around
    /// other nets' copper (when testing a place the tracks will only roughly keep to).
    fn pair_legal_with(
        &self,
        j: usize,
        spec: &PairSpec,
        leg: &pair::Leg,
        spans: &[Span],
        near: &[(usize, f64)],
        slack: f64,
    ) -> Result<(), String> {
        let through = (0, self.gr.nl - 1);
        let span_of = |i: usize| spans.get(i).copied().unwrap_or(through);
        let within = |layers: &[usize], (a, b): Span| layers.iter().any(|&l| a <= l && l <= b);
        let net_p = self.jobs[j].net.as_str();
        let edge = self.rules.edge_clearance(net_p);
        let vr = spec.via.map(|v| v.0 / 2.0).unwrap_or(0.0);
        for (lines, vias, other_net) in [(&leg.a, &leg.vias_a, spec.net_n.as_str()), (&leg.b, &leg.vias_b, net_p)] {
            for (l, pts) in lines.iter() {
                let hw = spec.ws[*l] / 2.0;
                for s in pts.windows(2) {
                    if !self.stub_ok(hw + slack, s[0], s[1], *l, near, edge) {
                        return Err("its tracks would come too close to other copper at a corner or an end".into());
                    }
                    // the other net of the pair: its pads and kept tracks
                    for ob in self.obstacles.iter().filter(|o| o.net.as_deref() == Some(other_net)) {
                        if ob.layers.contains(l)
                            && geom::seg_shape_dist(s[0], s[1], &ob.shape) < hw + slack + spec.pn - 1e-6
                        {
                            return Err("one track would come too close to the other net's pad".into());
                        }
                    }
                }
            }
            for (i, &v) in vias.iter().enumerate() {
                let span = span_of(i);
                for &(o, clr) in near {
                    let ob = &self.obstacles[o];
                    if within(&ob.layers, span) && ob.shape.dist(v) < clr + vr - 1e-6 {
                        return Err("its vias would come too close to other copper".into());
                    }
                }
                for ob in self.obstacles.iter().filter(|o| o.net.as_deref() == Some(other_net)) {
                    if within(&ob.layers, span) && ob.shape.dist(v) < vr + spec.pn - 1e-6 {
                        return Err("a via would come too close to the other net's pad".into());
                    }
                }
            }
        }
        for (la, pa) in &leg.a {
            for (lb, pb) in leg.b.iter().filter(|(lb, _)| lb == la) {
                let _ = lb;
                for s in pa.windows(2) {
                    for t in pb.windows(2) {
                        if geom::seg_seg_dist(s[0], s[1], t[0], t[1]) < spec.ws[*la] + spec.pn - 1e-6 {
                            return Err("its two tracks would come too close to each other at an end or a via".into());
                        }
                    }
                }
            }
        }
        // the other stretches of this pair: the corridor may share their space, so
        // each track is held off the other net's copper there, exactly
        for &k in &self.sibs[j] {
            let other = match &self.laid[k] {
                Some(l) => l,
                None => continue,
            };
            let kn = self.jobs[k].pair.as_ref().map(|p| p.net_n.as_str());
            let theirs: [(Option<&str>, &Vec<(usize, Vec<P>, f64)>); 2] =
                [(Some(self.jobs[k].net.as_str()), &other.tracks), (kn, &other.tracks_n)];
            for (mine, lines) in [(net_p, &leg.a), (spec.net_n.as_str(), &leg.b)] {
                for (their_net, tracks) in theirs.iter() {
                    if *their_net == Some(mine) || their_net.is_none() {
                        continue;
                    }
                    for (l, pts) in lines.iter() {
                        let hw = spec.ws[*l] / 2.0;
                        for s in pts.windows(2) {
                            for (tl, tp, tw) in tracks.iter() {
                                if tl != l {
                                    continue;
                                }
                                for t in tp.windows(2) {
                                    if geom::seg_seg_dist(s[0], s[1], t[0], t[1]) < hw + tw / 2.0 + spec.pn - 1e-6 {
                                        return Err("it would come too close to the other net where two stretches of the pair meet".into());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        for (vias, lines) in [(&leg.vias_a, &leg.b), (&leg.vias_b, &leg.a)] {
            for (i, &v) in vias.iter().enumerate() {
                let span = span_of(i);
                for (l, pts) in lines.iter().filter(|(l, _)| within(&[*l], span)) {
                    let hw = spec.ws[*l] / 2.0;
                    for s in pts.windows(2) {
                        if geom::seg_dist(v, s[0], s[1]) < vr + hw + spec.pn - 1e-6 {
                            return Err("a via would come too close to the other track".into());
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Where a pair's coupled part may begin or end, for each of its ends: free
    /// cells a little way off the two pads, from which a straight piece of
    /// track to each pad keeps every clearance.
    fn pair_gates(&mut self, j: usize) -> Vec<Vec<Entry2>> {
        if let Some(e) = &self.entries[j] {
            return e.clone();
        }
        let n = self.gr.n;
        let spec = self.jobs[j].pair.clone().expect("a pair job");
        let (block, _) = self.hard(j);
        let nl = self.gr.nl;
        let halves: Vec<f64> = (0..nl).map(|l| (spec.ws[l] + spec.gaps[l]) / 2.0).collect();
        let widest = halves.iter().cloned().fold(0.0, f64::max) * 2.0;
        let widest_track = spec.ws.iter().cloned().fold(0.0, f64::max);
        let mut all: Vec<Vec<Entry2>> = Vec::new();
        for e in &spec.ends {
            let mid = ((e.p.0 + e.n.0) / 2.0, (e.p.1 + e.n.1) / 2.0);
            let apart = geom::dist(e.p, e.n);
            let rmin = (0.5 * apart).max(1.5 * widest);
            let rmax = rmin + apart.max(3.0);
            let near = self.near(
                j,
                (mid.0 - rmax, mid.1 - rmax, mid.0 + rmax, mid.1 + rmax),
                widest_track,
            );
            let mut cells: Vec<usize> = Vec::new();
            self.gr.each(&Shape::Circle { c: mid, r: 0.0 }, rmax, |c| cells.push(c));
            let mut st: Vec<Entry2> = Vec::new();
            for c in cells {
                let at = self.gr.xy(c);
                let d = geom::dist(at, mid);
                if d < rmin {
                    continue;
                }
                // the two tracks arrive side by side, across the line from the pads
                let side = (-(at.1 - mid.1) / d, (at.0 - mid.0) / d);
                for &l in &e.layers {
                    if !self.layers_ok(l) || block[l][c] != 0 {
                        continue;
                    }
                    let half = halves[l];
                    let (q1, q2) = (
                        (at.0 + half * side.0, at.1 + half * side.1),
                        (at.0 - half * side.0, at.1 - half * side.1),
                    );
                    let (qp, qn) =
                        if geom::dist(e.p, q1) + geom::dist(e.n, q2) <= geom::dist(e.p, q2) + geom::dist(e.n, q1) {
                            (q1, q2)
                        } else {
                            (q2, q1)
                        };
                    let try_ = pair::Leg {
                        a: vec![(l, vec![e.p, qp])],
                        b: vec![(l, vec![e.n, qn])],
                        ..pair::Leg::default()
                    };
                    // (with room to spare: the tracks will arrive on the path's line, not
                    // exactly the one tried here)
                    if self.pair_legal_with(j, &spec, &try_, &[], &near, 0.1).is_ok() {
                        st.push(((l * n + c) as u32, None));
                    }
                }
            }
            st.sort_by(|a, b| a.0.cmp(&b.0));
            all.push(st);
        }
        self.entries[j] = Some(all.clone());
        all
    }

    /// Route a differential pair: one path down the middle for each leg of the
    /// chain, as wide as both tracks and their gap, then the two tracks either
    /// side of it. Any leg that cannot be done leaves the pair unrouted (and the
    /// caller routes the two nets one by one instead).
    fn route_pair(&mut self, j: usize, pres: f64, via_cost: f64, limit: usize, walls: bool) {
        let (nl, n) = (self.gr.nl, self.gr.n);
        let spec = self.jobs[j].pair.clone().expect("a pair job");
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
                for l in 0..nl {
                    for c in 0..n {
                        if sv[l][c] > 0 {
                            vb[l][c] = 1;
                        }
                    }
                }
            }
        }
        let gates: Vec<Vec<u32>> = self
            .pair_gates(j)
            .into_iter()
            .map(|t| {
                t.into_iter()
                    .map(|e| e.0)
                    .filter(|&s| block[s as usize / n][s as usize % n] == 0)
                    .collect()
            })
            .collect();
        let halves: Vec<f64> = (0..nl).map(|l| (spec.ws[l] + spec.gaps[l]) / 2.0).collect();
        let via_half = spec.via_pitch / 2.0;
        let reach = spec.ws.iter().cloned().fold(0.0, f64::max) + halves.iter().cloned().fold(0.0, f64::max);
        let mut laid = Laid::default();
        let search_via = if vblock.is_some() { via_cost } else { 0.0 };
        let spans = self.jobs[j].spans.clone();
        let keep = self.keep(j);
        let flat = vec![false; nl];
        let hist = std::mem::take(&mut self.hist);
        let mut stopped: Option<String> = None;
        for k in 0..spec.ends.len().saturating_sub(1) {
            let (ea, eb) = (&spec.ends[k], &spec.ends[k + 1]);
            if stopped.is_none() {
                for (e, g) in [(ea, &gates[k]), (eb, &gates[k + 1])] {
                    if g.is_empty() && stopped.is_none() {
                        stopped = Some(format!("no room near {} to bring the two tracks side by side", e.label));
                    }
                }
            }
            let mut done = false;
            if stopped.is_none() {
                let mid = ((eb.p.0 + eb.n.0) / 2.0, (eb.p.1 + eb.n.1) / 2.0);
                let search = Search {
                    block: &block,
                    vblock: vblock.as_deref(),
                    soft: &soft,
                    soft_via: &sv,
                    spans: &spans,
                    keep,
                    flat: &flat,
                    hist: &hist,
                    pres,
                    via_cost: search_via,
                    limit,
                };
                match astar(&self.gr, &mut self.work, &search, &gates[k], &gates[k + 1], mid) {
                    None => stopped = Some("no way through wide enough for both tracks".into()),
                    Some((path, how)) => {
                        // the layers each change of layer joins, in the path's order
                        let via_spans: Vec<Span> = (1..path.len())
                            .filter(|&i| path[i] as usize / n != path[i - 1] as usize / n)
                            .map(|i| spans[how[i] as usize])
                            .collect();
                        // the centreline, per layer, corners only
                        let mut runs: Vec<pair::Line> = Vec::new();
                        for &s in &path {
                            let (l, p) = (s as usize / n, self.gr.xy(s as usize % n));
                            match runs.last_mut() {
                                Some((rl, pts)) if *rl == l => {
                                    if pts.len() >= 2 {
                                        let (a, b) = (pts[pts.len() - 2], pts[pts.len() - 1]);
                                        if ((b.0 - a.0) * (p.1 - b.1) - (b.1 - a.1) * (p.0 - b.0)).abs() < 1e-9 {
                                            pts.pop();
                                        }
                                    }
                                    pts.push(p);
                                }
                                _ => {
                                    // the next run starts where this one ends, a layer away
                                    runs.push((l, vec![p]));
                                }
                            }
                        }
                        let mut bbox = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                        for q in runs
                            .iter()
                            .flat_map(|r| r.1.iter())
                            .chain([ea.p, ea.n, eb.p, eb.n].iter())
                        {
                            bbox = (bbox.0.min(q.0), bbox.1.min(q.1), bbox.2.max(q.0), bbox.3.max(q.1));
                        }
                        let near = self.near(j, bbox, spec.via_pitch + reach);
                        // the pair faces straight out from its first pads and straight in to its last
                        let mid_of = |e: &PairEnd| ((e.p.0 + e.n.0) / 2.0, (e.p.1 + e.n.1) / 2.0);
                        let (g0, g1) = (
                            self.gr.xy(path[0] as usize % n),
                            self.gr.xy(path[path.len() - 1] as usize % n),
                        );
                        let toward = |a: P, b: P| {
                            let d = geom::dist(a, b);
                            if d > 1e-9 {
                                Some(((b.0 - a.0) / d, (b.1 - a.1) / d))
                            } else {
                                None
                            }
                        };
                        let heading = (toward(mid_of(ea), g0), toward(g1, mid_of(eb)));
                        // which track runs on which side: try both; and if the path's first
                        // or last step is too far off the way the pair faces, let the path decide
                        let mut why = String::new();
                        let mut leg: Option<pair::Leg> = None;
                        'tries: for facing in [heading, (None, heading.1), (heading.0, None), (None, None)] {
                            for first in [1.0, -1.0] {
                                match pair::leg(
                                    &runs,
                                    &halves,
                                    via_half,
                                    spec.via_side,
                                    first,
                                    (ea.p, ea.n),
                                    (eb.p, eb.n),
                                    facing,
                                )
                                .and_then(|l| self.pair_legal(j, &spec, &l, &via_spans, &near).map(|_| l))
                                {
                                    Ok(l) => {
                                        leg = Some(l);
                                        break 'tries;
                                    }
                                    // (the last try is the most forgiving: its reason is the real one)
                                    Err(e) => why = e,
                                }
                            }
                        }
                        if why.contains("too close to each other") {
                            // both ways round were tried: the two nets do not line up
                            why = "the two nets are the other way round at one end from the other (the tracks would have to cross), or its pads leave them no room to come together".into();
                        }
                        match leg {
                            None => stopped = Some(why),
                            Some(l) => {
                                laid.tracks
                                    .extend(l.a.iter().map(|(ly, p)| (*ly, p.clone(), spec.ws[*ly])));
                                laid.tracks_n
                                    .extend(l.b.iter().map(|(ly, p)| (*ly, p.clone(), spec.ws[*ly])));
                                let through = (0, nl - 1);
                                let span = |i: usize| via_spans.get(i).copied().unwrap_or(through);
                                laid.real_vias
                                    .extend(l.vias_a.iter().enumerate().map(|(i, &v)| (v, false, span(i))));
                                laid.real_vias
                                    .extend(l.vias_b.iter().enumerate().map(|(i, &v)| (v, true, span(i))));
                                laid.vias
                                    .extend(l.centres.iter().enumerate().map(|(i, &c)| (c, span(i))));
                                laid.coupled += l.coupled;
                                laid.leg = Some(l.clone());
                                laid.paths.push(path);
                                done = true;
                            }
                        }
                    }
                }
            }
            if !done {
                laid.unrouted
                    .push(format!("{}: {}", eb.label, stopped.clone().unwrap_or_default()));
            }
        }
        self.hist = hist;
        self.laid[j] = Some(laid);
        self.contrib[j].clear();
        self.add(j, 1);
    }

    /// Give up on routing job j as a pair: wake the two jobs that route its nets
    /// one by one. Returns them, and why the pair did not work out.
    fn unpair(&mut self, j: usize) -> (usize, usize, String) {
        let why = self.laid[j]
            .as_ref()
            .and_then(|l| l.unrouted.first().cloned())
            .unwrap_or_default();
        self.rip(j);
        let spec = self.jobs[j].pair.clone().expect("a pair job");
        self.pair_notes.push((self.jobs[j].net.clone(), why.clone()));
        self.active[j] = false;
        self.active[spec.job_p] = true;
        self.active[spec.job_n] = true;
        (spec.job_p, spec.job_n, why)
    }

    fn route_job(&mut self, j: usize, pres: f64, via_cost: f64, limit: usize, walls: bool) {
        if self.jobs[j].pair.is_some() {
            return self.route_pair(j, pres, via_cost, limit, walls);
        }
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
                for l in 0..nl {
                    for c in 0..n {
                        if sv[l][c] > 0 {
                            vb[l][c] = 1;
                        }
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
        // a net with a plane starts from the plane (its first terminal)
        let plane = self.jobs[j].plane.clone();
        let flat: Vec<bool> = (0..nl).map(|l| plane.contains(&l)).collect();
        let first = if !plane.is_empty() {
            0
        } else {
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
        let widths = self.jobs[j].widths.clone();
        let first_label = self.jobs[j].terminals[first].label.clone();
        let mut near: Vec<f64> = (0..terms).map(|t| geom::dist(centre(t), centre(first))).collect();
        let mut laid = Laid::default();
        let search_via = if vblock.is_some() { via_cost } else { 0.0 };
        let spans = self.jobs[j].spans.clone();
        let keep = self.keep(j);
        let hole_room = self.jobs[j]
            .via
            .map(|(d, drill)| d.max(drill + self.rules.hole_to_hole))
            .unwrap_or(0.0);
        let hist = std::mem::take(&mut self.hist);
        while !todo.is_empty() {
            todo.sort_by(|&a, &b| near[a].partial_cmp(&near[b]).unwrap().then(a.cmp(&b)));
            let next = todo.remove(0);
            let label = self.jobs[j].terminals[next].label.clone();
            if states[next].is_empty() {
                laid.unrouted.push(format!(
                    "{}: no clear way in for a {} mm track",
                    label, self.jobs[j].width
                ));
                continue;
            }
            let s = Search {
                block: &block,
                vblock: vblock.as_deref(),
                soft: &soft,
                soft_via: &sv,
                spans: &spans,
                keep,
                flat: &flat,
                hist: &hist,
                pres,
                via_cost: search_via,
                limit,
            };
            let found = if plane.is_empty() {
                astar(&self.gr, &mut self.work, &s, &tree, &states[next], centre(next))
            } else {
                // from the copper joined so far, or from the plane itself: anywhere on it
                // near this piece, and further off if that fails
                let mut found = None;
                for reach in [4.0, 12.0] {
                    let mut from = tree.clone();
                    for &l in &plane {
                        let bl = &block[l];
                        self.gr.each(
                            &Shape::Circle {
                                c: centre(next),
                                r: 0.0,
                            },
                            reach,
                            |c| {
                                if bl[c] == 0 {
                                    from.push((l * n + c) as u32);
                                }
                            },
                        );
                    }
                    from.sort();
                    from.dedup();
                    found = astar(&self.gr, &mut self.work, &s, &from, &states[next], centre(next));
                    if found.is_some() {
                        break;
                    }
                }
                found
            };
            match found {
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
                Some((path, how)) => {
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
                    for (end, name, at) in [(from, &first_label, path[0]), (to, &label, last)] {
                        if let Some((_, w)) = end {
                            if w < widths[at as usize / n] - 1e-9 && !laid.necked.iter().any(|(n, _)| n == name) {
                                laid.necked.push((name.clone(), w));
                            }
                        }
                    }
                    let before = laid.vias.len();
                    self.geometry(&path, &how, &spans, &widths, from, to, &mut laid);
                    if spans.len() > 1 {
                        // a via that stops short of some layers: the net may go on from any
                        // layer it does reach, and wants no second hole of its own beside it
                        for &(p, (a, b)) in &laid.vias[before..] {
                            let c = self.gr.cell(p);
                            for l in a..=b {
                                if block[l][c] == 0 {
                                    tree.push((l * n + c) as u32);
                                }
                                if let Some(vb) = vblock.as_mut() {
                                    let layer = &mut vb[l];
                                    self.gr
                                        .each(&Shape::Circle { c: p, r: 0.0 }, hole_room, |x| layer[x] = 1);
                                }
                            }
                        }
                        tree.sort();
                        tree.dedup();
                    }
                    laid.paths.push(path);
                }
            }
        }
        self.hist = hist;
        self.laid[j] = Some(laid);
        self.contrib[j].clear();
        self.add(j, 1);
    }

    /// How many steps a path of job j must go on from one via before its next
    /// (where the two would share a layer).
    fn keep(&self, j: usize) -> usize {
        match self.jobs[j].via {
            Some((d, _)) => ((d + self.rules.hole_to_hole) / self.gr.g).ceil() as usize + 1,
            None => 0,
        }
    }

    /// Turn a path of states into tracks and vias. `how` says which of `spans`
    /// each change of layer was made by. `from` and `to` are the
    /// points to start and finish on (a pad's centre, a track's loose end) and
    /// the width of that piece, when the path's ends call for them.
    #[allow(clippy::too_many_arguments)]
    fn geometry(
        &self,
        path: &[u32],
        how: &[u8],
        spans: &[Span],
        widths: &[f64],
        from: Option<(P, f64)>,
        to: Option<(P, f64)>,
        laid: &mut Laid,
    ) {
        let n = self.gr.n;
        let mut runs: Vec<(usize, Vec<P>)> = Vec::new();
        for (k, &s) in path.iter().enumerate() {
            let (l, c) = (s as usize / n, s as usize % n);
            let p = self.gr.xy(c);
            match runs.last_mut() {
                Some((rl, pts)) if *rl == l => pts.push(p),
                _ => {
                    if k > 0 {
                        laid.vias.push((p, spans[how[k] as usize]));
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
                if w < widths[*l] - 1e-9 {
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
                laid.tracks.push((l, simp, widths[l]));
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
/// What a call of route_inner is for.
enum Stage<'f> {
    /// Route the board as it is.
    Plain,
    /// The board was given the fan-outs `have` as copper. Look for pads that
    /// (still) need one; if there are any, hand them back in `found` (with
    /// each pad that has none) and route nothing. Otherwise route.
    Fan {
        have: &'f [Fan],
        found: &'f mut (Vec<Fan>, Vec<(String, String, String)>),
    },
    /// Route a board that was given these fan-outs as copper.
    With(&'f [Fan]),
}

/// Route the board's unjoined nets.
pub fn route(board: &Board, rules: &Rules, opt: &Options) -> Routed {
    if !opt.fanout {
        return route_inner(board, rules, opt, Stage::Plain);
    }
    // fan-outs first, as copper the board already has: the router goes on from
    // their ends. One pad's fan-out can shut in another that had a way out
    // before, so look again until none is found
    let mut fans: Vec<Fan> = Vec::new();
    let mut notes: Vec<(String, String, String)> = Vec::new();
    let mut with: Option<Board> = None;
    let mut out = Routed::default();
    let mut routed = false;
    for _ in 0..3 {
        let mut found: (Vec<Fan>, Vec<(String, String, String)>) = (Vec::new(), Vec::new());
        out = route_inner(
            with.as_ref().unwrap_or(board),
            rules,
            opt,
            Stage::Fan {
                have: &fans,
                found: &mut found,
            },
        );
        notes = found.1;
        if found.0.is_empty() {
            routed = true;
            break;
        }
        let b = with.get_or_insert_with(|| board.clone());
        for f in &found.0 {
            for &(a, b2, width) in &f.tracks {
                b.tracks.push(crate::board::Track {
                    net: Some(f.net.clone()),
                    layer: f.layer,
                    a,
                    b: b2,
                    width,
                    locked: false,
                    span: (0, 0),
                });
            }
            if let Some((at, diameter, drill, layers)) = f.via {
                b.vias.push(crate::board::Via {
                    net: Some(f.net.clone()),
                    at,
                    diameter,
                    drill,
                    layers,
                    locked: false,
                    span: (0, 0),
                });
            }
        }
        fans.extend(found.0);
    }
    if !routed {
        out = route_inner(with.as_ref().unwrap_or(board), rules, opt, Stage::With(&fans));
    }
    if !fans.is_empty() {
        // the ones a new track touches (past the pad itself) are part of the
        // result; the rest are not written
        let planes = plane_layers(board);
        let used: Vec<&Fan> = fans
            .iter()
            .filter(|f| {
                // a via that comes down on the net's own plane has done its work
                let on_plane = f
                    .via
                    .map(|(at, _, _, (a, b))| {
                        opt.plane_vias
                            && board.zones.iter().any(|z| {
                                z.keepout.is_none()
                                    && z.net.as_deref() == Some(f.net.as_str())
                                    && z.layers.iter().any(|l| planes.contains(l) && a <= *l && *l <= b)
                                    && geom::in_polygon(at, &z.outline)
                            })
                    })
                    .unwrap_or(false);
                if on_plane {
                    return true;
                }
                let pad = f.tracks.first().map(|t| t.0).unwrap_or(f.end);
                out.tracks.iter().filter(|t| t.net == f.net).any(|t| {
                    [t.a, t.b].iter().any(|&p| {
                        let on_track = t.layer == f.layer
                            && geom::dist(p, pad) > 1e-6
                            && f.tracks
                                .iter()
                                .any(|&(a, b, w)| geom::seg_dist(p, a, b) <= w / 2.0 + 1e-6);
                        let on_via = f
                            .via
                            .map(|(at, d, _, (a, b))| {
                                a <= t.layer
                                    && t.layer <= b
                                    && t.layer != f.layer
                                    && geom::dist(p, at) <= d / 2.0 + 1e-6
                            })
                            .unwrap_or(false);
                        on_track || on_via
                    })
                })
            })
            .collect();
        for f in used {
            for &(a, b, width) in &f.tracks {
                out.tracks.push(NewTrack {
                    net: f.net.clone(),
                    layer: f.layer,
                    a,
                    b,
                    width,
                });
            }
            if let Some((at, diameter, drill, layers)) = f.via {
                out.vias.push(NewVia {
                    net: f.net.clone(),
                    at,
                    diameter,
                    drill,
                    layers,
                });
            }
            out.fanouts.push(FanReport {
                net: f.net.clone(),
                pad: f.pad.clone(),
                length: f.tracks.iter().map(|&(a, b, _)| geom::dist(a, b)).sum(),
                via: f.via.map(|v| v.3),
            });
        }
    }
    out.shut_in = notes;
    out
}

fn route_inner(board: &Board, rules: &Rules, opt: &Options, stage: Stage) -> Routed {
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
            // (a drilled pad's hole is in every layer, whatever copper it has)
            layers: if p.through { (0..nl).collect() } else { p.layers.clone() },
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
            layers: (v.layers.0..=v.layers.1).collect(),
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
                    (v.layers.0..=v.layers.1).collect(),
                    false,
                    String::new(),
                ));
            }
        }
    }
    let layer_of = |l: usize| crate::rules::Layer {
        name: board.copper[l].as_str(),
        outer: l == 0 || l + 1 == nl,
    };
    let mut jobs: Vec<Job> = Vec::new();
    let mut sigs: Vec<String> = Vec::new();
    let smallest = rules.smallest_clearance();
    let all_planes = plane_layers(board);
    for (net, items) in by_net {
        let pads = items.iter().filter(|i| i.2).count();
        // the inner planes of its own: its pads are joined to them, one by one
        let own_planes: Vec<usize> = if opt.plane_vias && !opt.no_vias {
            all_planes
                .iter()
                .cloned()
                .filter(|&l| {
                    board
                        .zones
                        .iter()
                        .any(|z| z.keepout.is_none() && z.net.as_deref() == Some(net.as_str()) && z.layers.contains(&l))
                })
                .collect()
        } else {
            Vec::new()
        };
        if pads < 2 && (own_planes.is_empty() || pads == 0) {
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
        if poured.contains(&net) && !opt.include_poured && own_planes.is_empty() {
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
        let mut terminals = terminals;
        if !own_planes.is_empty() {
            // which pads the net's copper joins to the plane as it stands: through
            // the plane itself, a via down to it, or a pour that is joined to it. (A
            // pad in a pour that is an island apart from the plane is not.)
            let there = crate::pour::on_plane(board, rules, &net, &own_planes, g);
            let pad_at = |at: P| {
                board
                    .pads
                    .iter()
                    .filter(|p| p.net.as_deref() == Some(net.as_str()))
                    .position(|p| p.shape.centre() == at)
            };
            let (on_plane, loose): (Vec<Terminal>, Vec<Terminal>) = terminals.into_iter().partition(|t| {
                t.parts
                    .iter()
                    .filter(|part| part.2)
                    .any(|part| pad_at(part.0.centre()).map(|k| there[k]).unwrap_or(false))
            });
            if loose.is_empty() {
                out.skipped.push((net, "has a copper pour".into()));
                continue;
            }
            out.planes.push((net.clone(), own_planes.clone(), loose.len()));
            // the plane, with the copper that is on it already, is the piece the rest join
            let the_plane = Terminal {
                label: format!(
                    "the {} plane",
                    own_planes
                        .iter()
                        .map(|&l| board.copper[l].as_str())
                        .collect::<Vec<&str>>()
                        .join(" / ")
                ),
                centre: loose[0].centre,
                parts: on_plane.into_iter().flat_map(|t| t.parts).collect(),
            };
            terminals = std::iter::once(the_plane).chain(loose).collect();
        }
        if terminals.len() < 2 {
            continue; // already joined
        }
        // (a rule may give a net one width on outer layers and another on inner ones)
        let widths: Vec<f64> = (0..nl)
            .map(|l| {
                opt.track_width
                    .unwrap_or_else(|| rules.track_width_on(&net, layer_of(l)))
            })
            .collect();
        let width = widths.iter().cloned().fold(0.0, f64::max);
        let narrowest = widths.iter().cloned().fold(f64::INFINITY, f64::min);
        let via = if opt.no_vias {
            None
        } else {
            opt.via.or_else(|| rules.via(&net))
        };
        // the vias it may use: right through, and the shorter ones the options
        // name, where its rules let it; shortest first
        let mut spans: Vec<Span> = vec![(0, nl - 1)];
        spans.extend(opt.via_spans.iter().cloned().filter(|&(a, b)| a < b && b < nl));
        spans.retain(|&(a, b)| {
            rules.via_allowed(
                &net,
                match (a == 0, b + 1 == nl) {
                    (true, true) => crate::rules::ViaKind::Through,
                    (false, false) => crate::rules::ViaKind::Buried,
                    _ => crate::rules::ViaKind::Blind,
                },
            )
        });
        spans.sort_by_key(|&(a, b)| (b - a, a));
        spans.dedup();
        let via = if spans.is_empty() { None } else { via };
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
        let margin = opt.margin.max(grid_margin(narrowest / 2.0 + smallest, g));
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
            widths,
            spans,
            plane: own_planes,
            width,
            via,
            margin,
            neck,
            terminals,
            sig,
            span: (sx1 - sx0) + (sy1 - sy0),
            pair: None,
        });
    }
    // short nets first: they have the fewest ways round
    jobs.sort_by(|a, b| a.span.partial_cmp(&b.span).unwrap().then(a.net.cmp(&b.net)));
    // differential pairs. A pair is a chain of ends (connector, protection part,
    // chip ...), and each stretch between two ends is a job of its own: routed as a
    // pair where that can be had, with a job for each net asleep behind it for where
    // it cannot. The jobs for the two whole nets are not used.
    let mut asleep: Vec<usize> = Vec::new();
    let mut pair_legs: Vec<(String, String, usize, f64, f64)> = Vec::new(); // (positive, negative, legs, width, gap)
                                                                            // pairs with a stretch long enough to route as a pair: real pairs, whose lengths matter
    let mut real_pairs: Vec<String> = Vec::new();
    let mut early_notes: Vec<(String, String)> = Vec::new();
    if opt.diff_pairs {
        let singles = jobs.len();
        for i in 0..singles {
            // this net as a positive half, and whichever net exists as its other half
            let k = match crate::rules::pair_partners(&jobs[i].net)
                .iter()
                .find_map(|name| jobs[..singles].iter().position(|jb| jb.net == *name))
            {
                Some(k) => k,
                None => continue,
            };
            // (a net joined to its plane is no half of a pair)
            if !jobs[i].plane.is_empty() || !jobs[k].plane.is_empty() {
                continue;
            }
            let other = jobs[k].net.clone();
            if let Some(only) = &opt.only_pairs {
                if !matches(only, &jobs[i].net) && !matches(only, &other) {
                    continue;
                }
            }
            let ends = match pair_ends(&jobs[i], &jobs[k]) {
                Ok(e) => e,
                Err(why) => {
                    out.pairs.push(PairReport {
                        positive: jobs[i].net.clone(),
                        negative: other.clone(),
                        as_pair: false,
                        note: why,
                        legs: 0,
                        coupled_legs: 0,
                        width: 0.0,
                        gap: 0.0,
                        coupled: 0.0,
                        length_positive: 0.0,
                        length_negative: 0.0,
                    });
                    continue;
                }
            };
            let class = rules.class_of(&jobs[i].net);
            let (ip, inn) = (Item::track(&jobs[i].net), Item::track(&other));
            let between = rules.clearance(&ip, &inn);
            // the pair's track width and gap on each layer: an option, a rule for that
            // layer, the class's pair settings, or the net's own width and clearance
            let ws: Vec<f64> = (0..nl)
                .map(|l| match (opt.pair_width, opt.track_width, class.pair_width > 0.0) {
                    (Some(w), _, _) => w,
                    (None, Some(w), false) => w,
                    _ => rules.pair_width_on(&jobs[i].net, layer_of(l)),
                })
                .collect();
            let gaps: Vec<f64> = (0..nl)
                .map(|l| {
                    opt.pair_gap
                        .unwrap_or_else(|| rules.pair_gap_on(&jobs[i].net, layer_of(l)).unwrap_or(between))
                })
                .collect();
            let (w, gap) = (ws[0], gaps[0]);
            let via = jobs[i].via;
            let widest_gap = gaps.iter().cloned().fold(0.0, f64::max);
            let via_gap = if class.pair_via_gap > 0.0 {
                class.pair_via_gap
            } else {
                widest_gap.max(between)
            };
            let via_pitch = via
                .map(|(d, h)| (d + via_gap).max(h + rules.hole_to_hole))
                .unwrap_or(0.0);
            let pn = gaps.iter().cloned().fold(between, f64::min);
            // a via stands at least this far to its side of the path, so the other
            // track (half a pitch to the other side) clears it; on a right-angle bend
            // that takes vias 1.41 times as far apart, which is what room is kept for
            let via_side = via
                .map(|(d, _)| {
                    (0..nl)
                        .map(|l| d / 2.0 + ws[l] / 2.0 + pn - (ws[l] + gaps[l]) / 2.0)
                        .fold(0.0, f64::max)
                })
                .unwrap_or(0.0);
            let via_room = via_pitch.max(2.0 * via_side * SQRT2);
            // as one wide track: both tracks and their gap, on each layer
            let fat: Vec<f64> = (0..nl).map(|l| 2.0 * ws[l] + gaps[l]).collect();
            let width = fat.iter().cloned().fold(0.0, f64::max);
            let narrowest = fat.iter().cloned().fold(f64::INFINITY, f64::min);
            let widest_half = (0..nl).map(|l| (ws[l] + gaps[l]) / 2.0).fold(0.0, f64::max);
            pair_legs.push((jobs[i].net.clone(), other.clone(), ends.len() - 1, w, gap));
            for leg in ends.windows(2) {
                // the stretch one net at a time: two jobs of two pieces each
                let one = |of: usize, a: usize, b: usize, jobs: &mut Vec<Job>| -> usize {
                    let terminals = vec![jobs[of].terminals[a].clone(), jobs[of].terminals[b].clone()];
                    let span = (terminals[0].centre.0 - terminals[1].centre.0).abs()
                        + (terminals[0].centre.1 - terminals[1].centre.1).abs();
                    let job = Job {
                        net: jobs[of].net.clone(),
                        widths: jobs[of].widths.clone(),
                        spans: jobs[of].spans.clone(),
                        plane: Vec::new(),
                        width: jobs[of].width,
                        via: jobs[of].via,
                        margin: jobs[of].margin,
                        neck: jobs[of].neck,
                        terminals,
                        sig: jobs[of].sig,
                        span,
                        pair: None,
                    };
                    jobs.push(job);
                    jobs.len() - 1
                };
                let job_p = one(i, leg[0].piece_p, leg[1].piece_p, &mut jobs);
                let job_n = one(k, leg[0].piece_n, leg[1].piece_n, &mut jobs);
                // a stretch of a few millimetres is not worth pairing (and the name
                // rule also catches nets that are no pair at all, such as the two
                // ends of a capacitor beside its chip): those go one net at a time
                let mid = |e: &PairEnd| ((e.p.0 + e.n.0) / 2.0, (e.p.1 + e.n.1) / 2.0);
                let far = geom::dist(mid(&leg[0]), mid(&leg[1]));
                if far < SHORTEST_PAIR {
                    early_notes.push((
                        jobs[i].net.clone(),
                        format!(
                            "{} to {}: {:.1} mm is too short a stretch to pair",
                            leg[0].label, leg[1].label, far
                        ),
                    ));
                    continue;
                }
                let mut terminals = jobs[job_p].terminals.clone();
                terminals.extend(jobs[job_n].terminals.iter().cloned());
                let span = jobs[job_p].span.max(jobs[job_n].span);
                let (net, sig) = (jobs[i].net.clone(), jobs[i].sig);
                jobs.push(Job {
                    net,
                    widths: fat.clone(),
                    plane: Vec::new(),
                    // (the vias both nets may use)
                    spans: jobs[i]
                        .spans
                        .iter()
                        .cloned()
                        .filter(|s| jobs[k].spans.contains(s))
                        .collect(),
                    width,
                    // both vias as one: a disc that holds them whichever way the pair is heading
                    via: via.map(|(d, h)| (via_room + d, h)),
                    // (and room for the outer track's corners: mitred at a right angle
                    // they stand a little outside the width the path was found for)
                    margin: opt.margin.max(grid_margin(narrowest / 2.0 + smallest, g)) + (SQRT2 - 1.0) * widest_half,
                    neck: None,
                    terminals,
                    sig,
                    span,
                    pair: Some(PairSpec {
                        job_p,
                        job_n,
                        net_n: other.clone(),
                        w,
                        gap,
                        ws: ws.clone(),
                        gaps: gaps.clone(),
                        via,
                        via_pitch,
                        via_side,
                        pn,
                        ends: leg.to_vec(),
                    }),
                });
                asleep.push(job_p);
                asleep.push(job_n);
                if !real_pairs.contains(&jobs[i].net) {
                    real_pairs.push(jobs[i].net.clone());
                }
            }
            asleep.push(i);
            asleep.push(k);
        }
    }
    // jobs that share a net (the stretches of a pair)
    let nets_of = |jb: &Job| -> Vec<String> {
        let mut v = vec![jb.net.clone()];
        if let Some(p) = &jb.pair {
            v.push(p.net_n.clone());
        }
        v
    };
    let sibs: Vec<Vec<usize>> = (0..jobs.len())
        .map(|a| {
            let mine = nets_of(&jobs[a]);
            (0..jobs.len())
                .filter(|&b| b != a && nets_of(&jobs[b]).iter().any(|n| mine.contains(n)))
                .collect()
        })
        .collect();
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
    let mut active = vec![true; nj];
    for &j in &asleep {
        active[j] = false;
    }
    let mut r = Router {
        planes,
        active,
        sibs,
        pair_notes: early_notes,
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
        radii: HashMap::new(),
        cache_ok,
        log: Vec::new(),
        fans: match &stage {
            Stage::With(f) => f.to_vec(),
            Stage::Fan { have, .. } => have.to_vec(),
            Stage::Plain => Vec::new(),
        },
        jobs,
        gr,
    };
    r.say(format!(
        "grid {} x {} x {} layers at {} mm; {} nets to route, {} left alone",
        r.gr.w,
        r.gr.h,
        nl,
        g,
        {
            let mut nets: Vec<&str> = r.jobs.iter().map(|jb| jb.net.as_str()).collect();
            nets.extend(
                r.jobs
                    .iter()
                    .filter_map(|jb| jb.pair.as_ref().map(|p| p.net_n.as_str())),
            );
            nets.sort();
            nets.dedup();
            nets.len()
        },
        out.skipped.len()
    ));
    if !pair_legs.is_empty() {
        r.say(format!(
            "  differential pairs: {}",
            pair_legs
                .iter()
                .map(|(p, n, legs, w, gap)| format!(
                    "{} / {} ({} mm tracks, {} mm apart, {} stretch{})",
                    p,
                    n,
                    w,
                    gap,
                    legs,
                    if *legs == 1 { "" } else { "es" }
                ))
                .collect::<Vec<String>>()
                .join("; ")
        ));
    }
    let widest = r.jobs.iter().map(|j| j.margin).fold(0.0, f64::max);
    if widest > opt.margin + 1e-9 {
        r.say(format!(
            "  the {} mm grid needs up to {:.3} mm of margin on clearances (asked: {} mm); a finer grid packs tracks closer",
            g, widest, opt.margin
        ));
    }

    // pads with no way out on the grid: a way out for each, and start again with those
    if let Stage::Fan { found, .. } = stage {
        *found = r.fanout();
        if !found.0.is_empty() {
            return out;
        }
    }

    let mut pres = 0.5;
    let mut todo: Vec<usize> = (0..nj).filter(|&j| r.active[j]).collect();
    let mut best: Option<usize> = None;
    let mut since = 0;
    let mut conflicts: BTreeMap<usize, Vec<u32>> = BTreeMap::new();
    for pass in 1..=opt.passes {
        let mut at = 0;
        while at < todo.len() {
            let j = todo[at];
            at += 1;
            if !r.active[j] {
                continue;
            }
            r.rip(j);
            r.route_job(j, pres, opt.via_cost, opt.limit, false);
            if !r.laid[j].as_ref().unwrap().unrouted.is_empty() {
                r.rip(j);
                r.route_job(j, pres, opt.via_cost / 2.0, opt.limit * 5, false);
            }
            // a pair that cannot be had even with nothing else in its way: the two nets
            // are routed one by one, further on in this pass
            if r.jobs[j].pair.is_some() && !r.laid[j].as_ref().unwrap().unrouted.is_empty() {
                let (p, n, why) = r.unpair(j);
                r.say(format!(
                    "  {} / {} not routed as a pair ({}); routing them one by one",
                    r.jobs[p].net, r.jobs[n].net, why
                ));
                todo.push(p);
                todo.push(n);
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
                if r.jobs[j].pair.is_some() && !r.laid[j].as_ref().unwrap().unrouted.is_empty() {
                    // squeezed out as a pair: the two nets go in one by one
                    let (p, n, why) = r.unpair(j);
                    r.say(format!(
                        "  {} / {} not routed as a pair ({}); routing them one by one",
                        r.jobs[p].net, r.jobs[n].net, why
                    ));
                    for k in [p, n] {
                        r.route_job(k, pres, opt.via_cost, opt.limit * 4, true);
                    }
                    continue;
                }
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

    // the stretches of a pair were each checked against the others as they stood
    // then; check them as they stand now, and route one by one any that fail
    loop {
        let mut failed: Option<usize> = None;
        for j in 0..nj {
            if !r.active[j] || r.jobs[j].pair.is_none() {
                continue;
            }
            let (spec, leg) = match (r.jobs[j].pair.clone(), r.laid[j].as_ref().and_then(|l| l.leg.clone())) {
                (Some(s), Some(l)) => (s, l),
                _ => continue,
            };
            let spans: Vec<Span> = r.laid[j].iter().flat_map(|l| l.vias.iter().map(|v| v.1)).collect();
            if r.pair_legal(j, &spec, &leg, &spans, &[]).is_err() {
                failed = Some(j);
                break;
            }
        }
        let j = match failed {
            Some(j) => j,
            None => break,
        };
        if let Some(l) = r.laid[j].as_mut() {
            l.unrouted
                .push("it came too close to the other net where two stretches of the pair meet".into());
        }
        let (p, n, _) = r.unpair(j);
        for k in [p, n] {
            r.route_job(k, pres, opt.via_cost, opt.limit * 4, true);
        }
    }

    // lengths: the halves of a pair to within their skew, nets up to the least
    // length a rule asks, and groups up to their longest
    {
        let mut nets: Vec<String> = Vec::new();
        for j in 0..nj {
            if r.active[j] {
                nets.push(r.jobs[j].net.clone());
                if let Some(p) = &r.jobs[j].pair {
                    nets.push(p.net_n.clone());
                }
            }
        }
        nets.sort();
        nets.dedup();
        let bring = |r: &mut Router, net: &str, target: f64, why: String, lengths: &mut Vec<LengthReport>| {
            let before = r.net_length(net);
            if target - before > 1e-6 {
                r.lengthen(net, target - before);
            }
            lengths.push(LengthReport {
                net: net.to_string(),
                why,
                before,
                target,
                after: r.net_length(net),
            });
        };
        for (p, n, _, _, _) in &pair_legs {
            // (not nets that are only named like a pair, such as a capacitor's two ends)
            if !real_pairs.contains(p) {
                continue;
            }
            let limit = match opt.skew.or_else(|| rules.skew(p)) {
                Some(l) => l,
                None => continue,
            };
            let (lp, ln) = (r.net_length(p), r.net_length(n));
            if (lp - ln).abs() <= limit {
                continue;
            }
            let (short, long, target) = if lp < ln { (p, n, ln) } else { (n, p, lp) };
            bring(
                &mut r,
                short,
                target,
                format!("to match {}, the other half of its pair", long),
                &mut out.lengths,
            );
        }
        for net in &nets {
            if let Some((Some(least), _)) = rules.length(net) {
                bring(&mut r, net, least, "a length rule".into(), &mut out.lengths);
            }
        }
        for group in &opt.match_groups {
            let members: Vec<&String> = nets.iter().filter(|n| matches(group, n)).collect();
            let longest = members
                .iter()
                .map(|n| (r.net_length(n), n.to_string()))
                .fold((0.0, String::new()), |a, b| if b.0 > a.0 { b } else { a });
            for net in members {
                if *net != longest.1 {
                    bring(
                        &mut r,
                        net,
                        longest.0,
                        format!("to match {}, the longest of its group", longest.1),
                        &mut out.lengths,
                    );
                }
            }
        }
        for net in &nets {
            if let Some((_, Some(most))) = rules.length(net) {
                let len = r.net_length(net);
                if len > most + 1e-6 {
                    out.warnings.push(format!(
                        "{} is {:.2} mm long, over the {:.2} mm a length rule allows (the router does not shorten tracks)",
                        net, len, most
                    ));
                }
            }
        }
        if !out.lengths.is_empty() {
            r.say(format!("  lengths tuned: {} net(s)", out.lengths.len()));
        }
    }

    // wide tracks: a second via beside each via
    if opt.double_vias > 0.0 {
        for j in 0..nj {
            if r.active[j] && r.jobs[j].pair.is_none() && r.jobs[j].width >= opt.double_vias - 1e-9 {
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

    let length = |tracks: &[(usize, Vec<P>, f64)]| -> f64 {
        tracks
            .iter()
            .map(|(_, p, _)| p.windows(2).map(|s| geom::dist(s[0], s[1])).sum::<f64>())
            .sum()
    };
    // a net may be the work of several jobs (the stretches of a pair): one report for it
    let mut reports: Vec<NetReport> = Vec::new();
    let mut lengths: BTreeMap<String, f64> = BTreeMap::new();
    let mut coupled: BTreeMap<String, (usize, f64)> = BTreeMap::new(); // positive net -> (stretches, mm)
    let report = |net: &str, width: f64, pieces: usize, laid: &Laid, reports: &mut Vec<NetReport>| match reports
        .iter_mut()
        .find(|n| n.net == net)
    {
        Some(n) => {
            n.pieces += pieces - 1;
            n.unrouted.extend(laid.unrouted.iter().cloned());
            n.necked.extend(laid.necked.iter().cloned());
        }
        None => reports.push(NetReport {
            net: net.to_string(),
            width,
            necked: laid.necked.clone(),
            pieces,
            unrouted: laid.unrouted.clone(),
        }),
    };
    for j in 0..nj {
        if !r.active[j] {
            continue;
        }
        let job = &r.jobs[j];
        let laid = r.laid[j].clone().unwrap_or_default();
        let partner = job.pair.as_ref().map(|p| p.net_n.clone());
        for (net, tracks) in [(Some(job.net.clone()), &laid.tracks), (partner.clone(), &laid.tracks_n)] {
            let net = match net {
                Some(n) => n,
                None => continue,
            };
            *lengths.entry(net.clone()).or_insert(0.0) += length(tracks);
            for (l, pts, w) in tracks.iter() {
                for s in pts.windows(2) {
                    out.tracks.push(NewTrack {
                        net: net.clone(),
                        layer: *l,
                        a: s[0],
                        b: s[1],
                        width: *w,
                    });
                }
            }
        }
        match &job.pair {
            None => {
                if let Some((d, drill)) = job.via {
                    for &(p, layers) in &laid.vias {
                        out.vias.push(NewVia {
                            net: job.net.clone(),
                            at: p,
                            diameter: d,
                            drill,
                            layers,
                        });
                    }
                }
                report(&job.net, job.width, job.terminals.len(), &laid, &mut reports);
            }
            Some(spec) => {
                if let Some((d, drill)) = spec.via {
                    for &(p, negative, layers) in &laid.real_vias {
                        out.vias.push(NewVia {
                            net: if negative { spec.net_n.clone() } else { job.net.clone() },
                            at: p,
                            diameter: d,
                            drill,
                            layers,
                        });
                    }
                }
                for net in [&job.net, &spec.net_n] {
                    report(net, spec.w, spec.ends.len(), &laid, &mut reports);
                }
                if laid.unrouted.is_empty() {
                    let e = coupled.entry(job.net.clone()).or_insert((0, 0.0));
                    e.0 += 1;
                    e.1 += laid.coupled;
                }
            }
        }
    }
    out.nets.extend(reports);
    for (positive, negative, legs, w, gap) in &pair_legs {
        let (done, mm) = coupled.get(positive).cloned().unwrap_or((0, 0.0));
        let mut why: Vec<String> = r
            .pair_notes
            .iter()
            .filter(|(p, _)| p == positive)
            .map(|(_, w)| w.clone())
            .collect();
        why.dedup();
        out.pairs.push(PairReport {
            positive: positive.clone(),
            negative: negative.clone(),
            as_pair: done > 0,
            note: why.join("; "),
            legs: *legs,
            coupled_legs: done,
            width: *w,
            gap: *gap,
            coupled: mm,
            length_positive: r.net_length(positive),
            length_negative: r.net_length(negative),
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
            assert!(v.layers.0 < v.layers.1 && v.layers.1 < b.copper.len());
            for l in v.layers.0..=v.layers.1 {
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
        (0..b.copper.len())
            .map(|l| smallest_gap_on(b, r, l))
            .fold(f64::INFINITY, f64::min)
    }

    /// The same, on one given layer.
    fn smallest_gap_on(b: &Board, r: &Routed, layer: usize) -> f64 {
        let c: Vec<(String, usize, Shape)> = copper(b, r).into_iter().filter(|x| x.1 == layer).collect();
        let mut worst = f64::INFINITY;
        for i in 0..c.len() {
            for k in i + 1..c.len() {
                if c[i].0 != c[k].0 {
                    worst = worst.min(gap(&c[i].2, &c[k].2));
                }
            }
        }
        worst
    }

    /// Whether KiCad would call every pad of `net` joined: pads, new tracks and
    /// vias linked where a track's end or a via's or pad's centre lies on the other.
    fn kicad_joined(b: &Board, r: &Routed, net: &str) -> bool {
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
            items.push((shape, (v.layers.0..=v.layers.1).collect(), false, String::new()));
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

    /// A two-layer board with a D+ / D- pair to join across it, 0.8 mm apart at each end.
    fn pair_board(extra: &str) -> Board {
        let s = String::from(HEAD)
            + &smd("J1", "D+", 3.0, 9.6, 0.5, 0.5)
            + &smd("J2", "D-", 3.0, 10.4, 0.5, 0.5)
            + &smd("U1", "D+", 27.0, 9.6, 0.5, 0.5)
            + &smd("U2", "D-", 27.0, 10.4, 0.5, 0.5)
            + extra
            + "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        Board::parse(s).unwrap()
    }

    #[test]
    fn a_differential_pair_runs_side_by_side() {
        // another net's pad stands in the straight line between the two ends
        let b = pair_board(&smd("X", "OTHER", 15.0, 10.0, 3.0, 3.0));
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.pairs.len(), 1);
        let p = &r.pairs[0];
        assert!(p.as_pair, "{}", p.note);
        assert_eq!((p.positive.as_str(), p.negative.as_str()), ("D+", "D-"));
        assert_eq!((p.width, p.gap), (0.25, 0.2));
        assert_eq!((p.legs, p.coupled_legs), (1, 1));
        assert_eq!(r.nets.len(), 2, "one report for each net: {:?}", r.nets);
        assert!(p.coupled > 15.0, "only {:.1} mm side by side", p.coupled);
        assert!((p.length_positive - p.length_negative).abs() < 3.0);
        for net in ["D+", "D-"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        // along the coupled part the two tracks are exactly a pitch apart, never nearer
        let (dp, dn): (Vec<&NewTrack>, Vec<&NewTrack>) = r.tracks.iter().partition(|t| t.net == "D+");
        let mut least = f64::INFINITY;
        for a in &dp {
            for c in &dn {
                if a.layer == c.layer {
                    least = least.min(geom::seg_seg_dist(a.a, a.b, c.a, c.b));
                }
            }
        }
        assert!(
            (least - 0.45).abs() < 1e-6,
            "the tracks' centre lines come {:.3} mm apart",
            least
        );
        assert!(r.vias.is_empty());
        // and it can be turned off
        let off = route(
            &b,
            &rules(),
            &Options {
                diff_pairs: false,
                ..Options::default()
            },
        );
        assert!(off.pairs.is_empty() && off.complete());
    }

    #[test]
    fn a_pair_changes_layer_with_a_via_each() {
        // a track of another net runs right across the front between the two ends
        let wall = smd("W1", "WALL", 15.0, 1.0, 0.8, 0.8)
            + &smd("W2", "WALL", 15.0, 19.0, 0.8, 0.8)
            + "(segment (start 15 1) (end 15 19) (width 0.25) (layer \"F.Cu\") (net \"WALL\"))\n";
        let b = pair_board(&wall);
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(r.pairs[0].as_pair, "{}", r.pairs[0].note);
        assert_eq!(r.vias.len(), 4, "down and up again, a via for each track each time");
        assert_eq!(r.vias.iter().filter(|v| v.net == "D+").count(), 2);
        for net in ["D+", "D-"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        // the two holes of a pair keep the board's hole-to-hole gap
        for (i, v) in r.vias.iter().enumerate() {
            for u in &r.vias[i + 1..] {
                assert!(geom::dist(v.at, u.at) >= 0.3 + 0.5 - 1e-9);
            }
        }
    }

    #[test]
    fn a_stretch_too_short_to_pair_is_routed_one_by_one() {
        // named like a pair, but 3 mm apart: the two ends of a capacitor beside its chip
        let s = String::from(HEAD)
            + &smd("C1", "CP", 10.0, 9.6, 0.5, 0.5)
            + &smd("C2", "CN", 10.0, 10.4, 0.5, 0.5)
            + &smd("U1", "CP", 13.0, 9.6, 0.5, 0.5)
            + &smd("U2", "CN", 13.0, 10.4, 0.5, 0.5)
            + "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.pairs.len(), 1);
        assert!(!r.pairs[0].as_pair);
        assert!(r.pairs[0].note.contains("too short"), "{}", r.pairs[0].note);
        for net in ["CP", "CN"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert!(smallest_gap(&b, &r) >= 0.2 - 1e-6);
    }

    #[test]
    fn nets_that_do_not_pair_up_are_routed_one_by_one() {
        // D- has a third pad that D+ has no partner for
        let b = pair_board(&smd("T", "D-", 15.0, 16.0, 0.5, 0.5));
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.pairs.len(), 1);
        assert!(!r.pairs[0].as_pair);
        assert!(r.pairs[0].note.contains("pieces"), "{}", r.pairs[0].note);
        assert_eq!((r.pairs[0].legs, r.pairs[0].coupled_legs), (0, 0));
        for net in ["D+", "D-"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert!(smallest_gap(&b, &r) >= 0.2 - 1e-6);
    }

    /// The length of a net's new tracks.
    fn routed_length(r: &Routed, net: &str) -> f64 {
        r.tracks
            .iter()
            .filter(|t| t.net == net)
            .map(|t| geom::dist(t.a, t.b))
            .sum()
    }

    #[test]
    fn a_group_of_nets_is_brought_to_the_length_of_its_longest() {
        // three nets across the board, the third with much further to go
        let s = String::from(HEAD)
            + &smd("A1", "D0", 3.0, 4.0, 0.8, 0.8)
            + &smd("A2", "D0", 20.0, 4.0, 0.8, 0.8)
            + &smd("B1", "D1", 3.0, 9.0, 0.8, 0.8)
            + &smd("B2", "D1", 24.0, 9.0, 0.8, 0.8)
            + &smd("C1", "D2", 3.0, 14.0, 0.8, 0.8)
            + &smd("C2", "D2", 27.0, 16.0, 0.8, 0.8)
            + &smd("X1", "OTHER", 12.0, 6.5, 0.8, 0.8)
            + "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let plain = route(&b, &rules(), &Options::default());
        assert!(plain.lengths.is_empty(), "nothing is tuned unless asked");
        let opt = Options {
            match_groups: vec![vec!["D?".to_string()]],
            ..Options::default()
        };
        let r = route(&b, &rules(), &opt);
        assert!(r.complete(), "{:?}", r.nets);
        let longest = routed_length(&plain, "D2");
        assert!(routed_length(&plain, "D0") < longest - 5.0);
        for net in ["D0", "D1", "D2"] {
            let len = routed_length(&r, net);
            assert!(
                (len - longest).abs() < 1e-6,
                "{} is {:.3} mm, not {:.3}",
                net,
                len,
                longest
            );
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert_eq!(r.lengths.len(), 2);
        assert!(r
            .lengths
            .iter()
            .all(|l| (l.after - l.target).abs() < 1e-6 && l.before < l.target));
        // the meanders keep clear of everything else
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        // and of the track they belong to: no bump shorted out by its neighbour
        for net in ["D0", "D1"] {
            let segs: Vec<&NewTrack> = r.tracks.iter().filter(|t| t.net == net).collect();
            for (i, a) in segs.iter().enumerate() {
                for c in &segs[i + 1..] {
                    let touching = [a.a, a.b]
                        .iter()
                        .any(|p| geom::dist(*p, c.a) < 1e-9 || geom::dist(*p, c.b) < 1e-9);
                    if !touching {
                        let d = geom::seg_seg_dist(a.a, a.b, c.a, c.b);
                        assert!(d >= 0.45 - 1e-6, "two parts of {} run {:.3} mm apart", net, d);
                    }
                }
            }
        }
    }

    #[test]
    fn the_halves_of_a_pair_are_brought_within_their_skew() {
        // the pair bends round a pad, so its outer track comes out longer
        let b = pair_board(&smd("X", "OTHER", 15.0, 10.0, 3.0, 3.0));
        let plain = route(&b, &rules(), &Options::default());
        let apart = (routed_length(&plain, "D+") - routed_length(&plain, "D-")).abs();
        assert!(
            apart > 0.05,
            "this board is meant to leave the pair uneven ({:.3} mm)",
            apart
        );
        let r = route(
            &b,
            &rules(),
            &Options {
                skew: Some(0.01),
                ..Options::default()
            },
        );
        assert!(r.complete(), "{:?}", r.nets);
        assert!(r.pairs[0].as_pair);
        let (lp, ln) = (routed_length(&r, "D+"), routed_length(&r, "D-"));
        assert!((lp - ln).abs() <= 0.01 + 1e-6, "still {:.3} mm apart", (lp - ln).abs());
        assert_eq!(r.lengths.len(), 1);
        for net in ["D+", "D-"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        // within the skew already: nothing is touched
        let loose = route(
            &b,
            &rules(),
            &Options {
                skew: Some(apart + 0.1),
                ..Options::default()
            },
        );
        assert!(loose.lengths.is_empty());
        assert_eq!(loose.tracks, plain.tracks);
    }

    #[test]
    fn a_net_with_no_room_for_meanders_says_how_short_it_is() {
        // boxed in by the board's edge on both sides
        let s = String::from(HEAD)
            + &smd("A1", "D0", 2.0, 1.0, 0.8, 0.8)
            + &smd("A2", "D0", 18.0, 1.0, 0.8, 0.8)
            + "(gr_rect (start 0 0) (end 20 2) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let mut rules = rules();
        rules
            .read_dru("(rule \"long\" (constraint length (min 40mm)) (condition \"A.NetName == 'D0'\"))")
            .unwrap();
        let r = route(&b, &rules, &Options::default());
        assert!(r.complete());
        assert_eq!(r.lengths.len(), 1);
        let l = &r.lengths[0];
        assert!((l.target - 40.0).abs() < 1e-9 && l.after < l.target, "{:?}", l);
        assert!(smallest_gap(&b, &r) >= 0.2 - 1e-6);
    }

    #[test]
    fn a_net_has_its_own_width_on_each_layer() {
        // the crossing nets change layer; a rule makes them wide on the front, narrow on the back
        let b = crossing();
        let mut rules = rules();
        rules
            .read_dru(
                "(rule \"front\" (layer \"F.Cu\") (constraint track_width (opt 0.5mm)))\n\
                 (rule \"back\" (layer \"B.Cu\") (constraint track_width (opt 0.2mm)))",
            )
            .unwrap();
        let r = route(&b, &rules, &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(r.tracks.iter().any(|t| t.layer == 0) && r.tracks.iter().any(|t| t.layer == 1));
        for t in &r.tracks {
            let want = if t.layer == 0 { 0.5 } else { 0.2 };
            assert!(
                (t.width - want).abs() < 1e-9,
                "a {} mm track on layer {}",
                t.width,
                t.layer
            );
        }
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        for net in ["H1", "H2", "V1", "V2"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
    }

    #[test]
    fn a_clearance_rule_for_one_layer_holds_there_only() {
        let b = crossing();
        let mut rules = rules();
        rules
            .read_dru("(rule \"back\" (layer \"B.Cu\") (constraint clearance (min 0.6mm)))")
            .unwrap();
        let r = route(&b, &rules, &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(
            r.tracks.iter().any(|t| t.layer == 1),
            "nothing went to the back to test"
        );
        assert!(
            smallest_gap_on(&b, &r, 1) >= 0.6 - 1e-6,
            "back copper {:.3} mm apart",
            smallest_gap_on(&b, &r, 1)
        );
        assert!(smallest_gap_on(&b, &r, 0) >= 0.2 - 1e-6);
        // and without the rule the back is as tight as the front is allowed to be
        let plain = route(&b, &self::rules(), &Options::default());
        assert!(smallest_gap_on(&b, &plain, 1) >= 0.2 - 1e-6);
    }

    #[test]
    fn a_pair_has_its_own_width_and_gap_on_each_layer() {
        // the pair has to go under a track that crosses the front
        let wall = smd("W1", "WALL", 15.0, 1.0, 0.8, 0.8)
            + &smd("W2", "WALL", 15.0, 19.0, 0.8, 0.8)
            + "(segment (start 15 1) (end 15 19) (width 0.25) (layer \"F.Cu\") (net \"WALL\"))\n";
        let b = pair_board(&wall);
        let mut rules = rules();
        rules
            .read_dru(
                "(rule \"pair front\" (layer \"F.Cu\") (constraint track_width (opt 0.3mm)) (constraint diff_pair_gap (opt 0.2mm)) (condition \"A.inDiffPair('*')\"))\n\
                 (rule \"pair back\" (layer \"B.Cu\") (constraint track_width (opt 0.15mm)) (constraint diff_pair_gap (opt 0.25mm)) (condition \"A.inDiffPair('*')\"))",
            )
            .unwrap();
        let r = route(&b, &rules, &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert!(r.pairs[0].as_pair, "{}", r.pairs[0].note);
        assert_eq!(r.vias.len(), 4);
        for (layer, width, pitch) in [(0usize, 0.3, 0.5), (1usize, 0.15, 0.4)] {
            let (dp, dn): (Vec<&NewTrack>, Vec<&NewTrack>) = r
                .tracks
                .iter()
                .filter(|t| t.layer == layer)
                .partition(|t| t.net == "D+");
            assert!(!dp.is_empty() && !dn.is_empty(), "the pair is not on layer {}", layer);
            let mut least = f64::INFINITY;
            for a in &dp {
                assert!(
                    (a.width - width).abs() < 1e-9,
                    "a {} mm track on layer {}",
                    a.width,
                    layer
                );
                for c in &dn {
                    least = least.min(geom::seg_seg_dist(a.a, a.b, c.a, c.b));
                }
            }
            assert!(
                (least - pitch).abs() < 1e-6,
                "on layer {} the tracks' centre lines come {:.3} mm apart, not {}",
                layer,
                least,
                pitch
            );
        }
        for net in ["D+", "D-"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
    }

    const HEAD4: &str = "(kicad_pcb (version 20260206) (layers (0 \"F.Cu\" signal) (4 \"In1.Cu\" signal) (6 \"In2.Cu\" signal) (2 \"B.Cu\" signal) (25 \"Edge.Cuts\" user))\n";

    /// Four layers. A wall of another net's copper runs right across the front
    /// between the pads of A, and one big pad of a third net takes up the back:
    /// a via cannot go right through anywhere.
    fn walled_in(pads: &str) -> Board {
        let s = String::from(HEAD4)
            + pads
            + &smd("W", "WALL", 15.0, 10.0, 1.0, 19.0)
            + "(footprint \"P\" (at 15 10) (property \"Reference\" \"X1\") (pad \"1\" smd rect (at 0 0) (size 29 19) (layers \"B.Cu\") (net \"X\")))\n"
            + "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        Board::parse(s).unwrap()
    }

    /// Holes of vias that share a layer, at least `apart` from each other (centre to centre).
    fn holes_apart(r: &Routed, apart: f64) {
        for (i, v) in r.vias.iter().enumerate() {
            for u in &r.vias[i + 1..] {
                if v.layers.0 <= u.layers.1 && u.layers.0 <= v.layers.1 {
                    let d = geom::dist(v.at, u.at);
                    assert!(
                        d >= apart - 1e-6,
                        "vias of {} and {} are {:.3} mm apart",
                        v.net,
                        u.net,
                        d
                    );
                }
            }
        }
    }

    #[test]
    fn a_blind_via_goes_where_a_through_via_cannot() {
        let b = walled_in(&(smd("J1", "A", 5.0, 10.0, 0.8, 0.8) + &smd("J2", "A", 25.0, 10.0, 0.8, 0.8)));
        // through vias only: no way past the wall
        let r = route(&b, &rules(), &Options::default());
        assert!(!r.complete(), "the back is one pad of another net: no through via fits");
        assert!(r.vias.is_empty());
        // with vias from the front to the first inner layer, there is
        let opt = Options {
            via_spans: vec![(0, 1)],
            ..Options::default()
        };
        let r = route(&b, &rules(), &opt);
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.vias.len(), 2, "down past the wall and up again");
        assert!(r.vias.iter().all(|v| v.layers == (0, 1)), "{:?}", r.vias);
        assert!(r.tracks.iter().all(|t| t.layer <= 1));
        assert!(kicad_joined(&b, &r, "A"));
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
    }

    #[test]
    fn a_rule_can_forbid_blind_vias() {
        let b = walled_in(&(smd("J1", "A", 5.0, 10.0, 0.8, 0.8) + &smd("J2", "A", 25.0, 10.0, 0.8, 0.8)));
        let mut rules = rules();
        rules
            .read_dru("(version 1)\n(rule \"no blind vias on A\"\n  (constraint disallow blind_via buried_via)\n  (condition \"A.NetName == 'A'\"))\n")
            .unwrap();
        let opt = Options {
            via_spans: vec![(0, 1), (1, 2)],
            ..Options::default()
        };
        let r = route(&b, &rules, &opt);
        assert!(!r.complete(), "the only way was a blind via, and the rule forbids it");
        assert!(r.vias.is_empty());
    }

    #[test]
    fn vias_on_two_layers_are_all_through_vias() {
        // naming the only span a two-layer board has changes nothing
        let b = crossing();
        let plain = route(&b, &rules(), &Options::default());
        let opt = Options {
            via_spans: vec![(0, 1)],
            ..Options::default()
        };
        let named = route(&b, &rules(), &opt);
        assert_eq!(plain.tracks, named.tracks);
        assert_eq!(plain.vias, named.vias);
        assert!(plain.vias.iter().all(|v| v.layers == (0, 1)));
    }

    #[test]
    fn blind_and_buried_vias_keep_every_clearance() {
        // four layers and four nets that must all cross, with every span allowed
        let mut s = String::from(HEAD4);
        let pads = [
            ("H1", 1.0, 6.0),
            ("H1", 29.0, 6.0),
            ("H2", 1.0, 10.0),
            ("H2", 29.0, 10.0),
            ("H3", 1.0, 14.0),
            ("H3", 29.0, 14.0),
            ("V1", 8.0, 1.0),
            ("V1", 8.0, 19.0),
            ("V2", 15.0, 1.0),
            ("V2", 15.0, 19.0),
            ("V3", 22.0, 1.0),
            ("V3", 22.0, 19.0),
        ];
        for (k, (net, x, y)) in pads.iter().enumerate() {
            s += &smd(&format!("J{}", k), net, *x, *y, 0.8, 0.8);
        }
        // the back is shut to all of them but for a strip down the middle
        s += "(footprint \"P\" (at 7 10) (property \"Reference\" \"X1\") (pad \"1\" smd rect (at 0 0) (size 13 19) (layers \"B.Cu\") (net \"X\")))\n";
        s += "(footprint \"P\" (at 23 10) (property \"Reference\" \"X2\") (pad \"1\" smd rect (at 0 0) (size 13 19) (layers \"B.Cu\") (net \"Y\")))\n";
        s += "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let opt = Options {
            via_spans: vec![(0, 1), (0, 2), (1, 2), (1, 3), (2, 3)],
            ..Options::default()
        };
        let r = route(&b, &rules(), &opt);
        assert!(r.complete(), "{:?}", r.nets);
        assert!(
            r.vias.iter().any(|v| v.layers != (0, 3)),
            "with the back mostly shut, some via stops short of it"
        );
        for l in 0..4 {
            assert!(
                smallest_gap_on(&b, &r, l) >= 0.2 - 1e-6,
                "on layer {} copper is only {:.3} mm apart",
                l,
                smallest_gap_on(&b, &r, l)
            );
        }
        // two holes that share a layer keep the board's hole-to-hole distance, edge to edge
        holes_apart(&r, 0.3 + 0.5);
        for net in ["H1", "H2", "H3", "V1", "V2", "V3"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined as KiCad counts it", net);
        }
        // every via has a track of its net on its first layer and on its last
        for v in &r.vias {
            for l in [v.layers.0, v.layers.1] {
                assert!(
                    r.tracks
                        .iter()
                        .any(|t| t.net == v.net && t.layer == l && geom::seg_dist(v.at, t.a, t.b) < 1e-6)
                        || b.pads.iter().any(|p| p.net.as_deref() == Some(v.net.as_str())
                            && p.layers.contains(&l)
                            && p.shape.dist(v.at) < 1e-6),
                    "the via of {} at {:?} joins layers {:?} but nothing of its net is on layer {}",
                    v.net,
                    v.at,
                    v.layers,
                    l
                );
            }
        }
    }

    #[test]
    fn a_pair_changes_layer_with_blind_vias() {
        let pads = smd("J1", "D+", 3.0, 9.6, 0.5, 0.5)
            + &smd("J2", "D-", 3.0, 10.4, 0.5, 0.5)
            + &smd("U1", "D+", 27.0, 9.6, 0.5, 0.5)
            + &smd("U2", "D-", 27.0, 10.4, 0.5, 0.5);
        let b = walled_in(&pads);
        let opt = Options {
            via_spans: vec![(0, 1)],
            ..Options::default()
        };
        let r = route(&b, &rules(), &opt);
        assert!(r.complete(), "{:?}", r.nets);
        assert!(r.pairs[0].as_pair, "{}", r.pairs[0].note);
        assert_eq!(r.vias.len(), 4, "down and up again, a via for each track each time");
        assert!(r.vias.iter().all(|v| v.layers == (0, 1)), "{:?}", r.vias);
        for net in ["D+", "D-"] {
            assert!(kicad_joined(&b, &r, net), "{} is not joined", net);
        }
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
    }

    /// Rules for fine work: 0.1 mm tracks and gaps, 0.45 mm vias.
    fn fine_rules() -> Rules {
        let mut r = rules();
        r.classes[0].clearance = 0.1;
        r.classes[0].track_width = 0.1;
        r.classes[0].via_diameter = 0.45;
        r.classes[0].via_drill = 0.2;
        r.hole_to_hole = 0.25;
        r.hole_clearance = 0.2;
        r
    }

    /// A 5 x 5 BGA of 0.4 mm balls at 0.8 mm, a little off the grid, each ball
    /// wired to a pad of a ring around the board. `extra`: more board.
    fn bga(extra: &str) -> Board {
        let mut s = String::from(HEAD);
        let (mid, pitch) = (10.1, 0.8);
        s += &format!(
            "(footprint \"BGA\" (at {} {}) (property \"Reference\" \"U1\")\n",
            mid, mid
        );
        for r in 0..5 {
            for c in 0..5 {
                s += &format!(
                    "(pad \"{}{}\" smd circle (at {} {}) (size 0.4 0.4) (layers \"F.Cu\") (net \"B{}{}\"))\n",
                    (b'A' + r as u8) as char,
                    c + 1,
                    (c as f64 - 2.0) * pitch,
                    (r as f64 - 2.0) * pitch,
                    r,
                    c
                );
            }
        }
        s += ")\n";
        // the ring, in the order the balls lie round the middle
        let mut balls: Vec<(usize, usize)> = (0..5).flat_map(|r| (0..5).map(move |c| (r, c))).collect();
        balls.sort_by(|a, b| {
            let angle = |x: &(usize, usize)| (x.0 as f64 - 2.0).atan2(x.1 as f64 - 2.0);
            angle(a).partial_cmp(&angle(b)).unwrap()
        });
        for (k, (r, c)) in balls.iter().enumerate() {
            let a = -std::f64::consts::PI + 2.0 * std::f64::consts::PI * (k as f64 + 0.5) / 25.0;
            let t = a.cos().abs().max(a.sin().abs());
            let (x, y) = (10.0 + 8.5 * a.cos() / t, 10.0 + 8.5 * a.sin() / t);
            s += &smd(
                &format!("J{}", k),
                &format!("B{}{}", r, c),
                (x * 4.0).round() / 4.0,
                (y * 4.0).round() / 4.0,
                0.6,
                0.6,
            );
        }
        s += extra;
        s += "(gr_rect (start 0 0) (end 20 20) (layer \"Edge.Cuts\")))";
        Board::parse(s).unwrap()
    }

    #[test]
    fn the_inner_balls_of_a_bga_get_dog_bones() {
        let b = bga("");
        let rules = fine_rules();
        // on the grid alone, no via fits between the balls: the nine inner ones are shut in
        let plain = route(
            &b,
            &rules,
            &Options {
                fanout: false,
                ..Options::default()
            },
        );
        assert_eq!(plain.nets.iter().filter(|n| !n.unrouted.is_empty()).count(), 9);
        let r = route(&b, &rules, &Options::default());
        assert!(
            r.complete(),
            "{:?}",
            r.nets.iter().filter(|n| !n.unrouted.is_empty()).collect::<Vec<_>>()
        );
        assert_eq!(r.fanouts.len(), 9, "{:?}", r.fanouts);
        assert!(r.shut_in.is_empty());
        for f in &r.fanouts {
            assert_eq!(f.via, Some((0, 1)), "{:?}", f);
            // the via sits in the middle of four balls, on the side away from the part's middle
            let pad = b.pads.iter().find(|p| p.name() == f.pad).unwrap().shape.centre();
            let via = r
                .vias
                .iter()
                .filter(|v| v.net == f.net)
                .min_by(|x, y| geom::dist(x.at, pad).partial_cmp(&geom::dist(y.at, pad)).unwrap())
                .unwrap();
            let (dx, dy) = (via.at.0 - pad.0, via.at.1 - pad.1);
            assert!(
                (dx.abs() - 0.4).abs() < 0.01 && (dy.abs() - 0.4).abs() < 0.01,
                "{}: its via is ({:.3}, {:.3}) from the ball",
                f.pad,
                dx,
                dy
            );
            let out = (pad.0 - 10.1, pad.1 - 10.1);
            assert!(dx * out.0 + dy * out.1 >= -1e-9, "{}: its via lies inwards", f.pad);
        }
        assert!(
            smallest_gap(&b, &r) >= 0.1 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        holes_apart(&r, 0.2 + 0.25);
        for n in &r.nets {
            assert!(
                kicad_joined(&b, &r, &n.net),
                "{} is not joined as KiCad counts it",
                n.net
            );
        }
    }

    #[test]
    fn a_fan_out_nobody_uses_is_not_written() {
        // the ring pad of the middle ball is walled in by another net: its net cannot be joined
        let ring = bga("");
        let far = ring
            .pads
            .iter()
            .find(|p| p.net.as_deref() == Some("B22") && p.reference != "U1")
            .unwrap()
            .shape
            .centre();
        let mut wall = String::new();
        for (k, (dx, dy, w, h)) in [
            (0.0, -0.65, 1.7, 0.4),
            (0.0, 0.65, 1.7, 0.4),
            (-0.65, 0.0, 0.4, 0.9),
            (0.65, 0.0, 0.4, 0.9),
        ]
        .iter()
        .enumerate()
        {
            wall += &smd(&format!("W{}", k), "WALL", far.0 + dx, far.1 + dy, *w, *h);
        }
        let b = bga(&wall);
        let r = route(&b, &fine_rules(), &Options::default());
        let left: Vec<&str> = r
            .nets
            .iter()
            .filter(|n| !n.unrouted.is_empty())
            .map(|n| n.net.as_str())
            .collect();
        assert_eq!(left, vec!["B22"]);
        assert_eq!(r.fanouts.len(), 8);
        assert!(
            r.tracks.iter().all(|t| t.net != "B22"),
            "a fan-out track of B22 was written"
        );
        assert!(
            r.vias.iter().all(|v| v.net != "B22"),
            "a fan-out via of B22 was written"
        );
    }

    #[test]
    fn pads_in_a_row_too_tight_for_the_grid_get_landings_of_their_own() {
        // eight pads 0.2 mm wide at 0.4 mm, with 0.15 mm tracks and gaps: tracks cannot
        // run that close on a 0.25 mm grid
        let mut s = String::from(HEAD);
        s += "(footprint \"ROW\" (at 10.05 6) (property \"Reference\" \"U1\")\n";
        for k in 0..8 {
            s += &format!(
                "(pad \"{}\" smd rect (at {} 0) (size 0.2 0.8) (layers \"F.Cu\") (net \"N{}\"))\n",
                k + 1,
                (k as f64 - 3.5) * 0.4,
                k
            );
        }
        // (the far side of the part: the pads face down the board)
        s += "(pad \"EP\" smd rect (at 0 -2) (size 3.2 2) (layers \"F.Cu\") (net \"GND\")))\n";
        for k in 0..8 {
            s += &smd(
                &format!("J{}", k),
                &format!("N{}", k),
                3.0 + 2.0 * k as f64,
                17.0,
                0.8,
                0.8,
            );
        }
        s += "(gr_rect (start 0 0) (end 20 20) (layer \"Edge.Cuts\")))";
        let b = Board::parse(s).unwrap();
        let mut rules = rules();
        rules.classes[0].clearance = 0.15;
        rules.classes[0].track_width = 0.15;
        rules.classes[0].via_diameter = 0.45;
        rules.classes[0].via_drill = 0.2;
        let left = |r: &Routed| r.nets.iter().filter(|n| !n.unrouted.is_empty()).count();
        let plain = route(
            &b,
            &rules,
            &Options {
                fanout: false,
                ..Options::default()
            },
        );
        let r = route(&b, &rules, &Options::default());
        // (at this pitch the grid cannot bring all eight out, with a landing each or
        // without: the fan-out gets more of them out)
        assert!(
            left(&r) < left(&plain),
            "{} left, {} without fan-out",
            left(&r),
            left(&plain)
        );
        assert!(left(&r) <= 1, "{:?} {:?}", r.fanouts, r.shut_in);
        assert!(!r.fanouts.is_empty());
        assert!(
            smallest_gap(&b, &r) >= 0.15 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        for n in r.nets.iter().filter(|n| n.unrouted.is_empty()) {
            assert!(kicad_joined(&b, &r, &n.net), "{} is not joined", n.net);
        }
        // and with ordinary parts nothing is fanned out
        let plain = route(&crossing(), &self::rules(), &Options::default());
        assert!(plain.fanouts.is_empty() && plain.shut_in.is_empty());
    }

    /// Four layers with a GND plane on In1 and three GND pads on the front, with
    /// two signal nets between them. `extra`: more board.
    fn plane_board(extra: &str) -> Board {
        let s = String::from(HEAD4)
            + &smd("C1", "GND", 5.0, 5.0, 0.8, 0.8)
            + &smd("C2", "GND", 15.0, 10.0, 0.8, 0.8)
            + &smd("C3", "GND", 25.0, 15.0, 0.8, 0.8)
            + &smd("J1", "A", 3.0, 10.0, 0.8, 0.8)
            + &smd("J2", "A", 27.0, 10.0, 0.8, 0.8)
            + "(zone (net \"GND\") (layer \"In1.Cu\") (polygon (pts (xy 0.5 0.5) (xy 29.5 0.5) (xy 29.5 19.5) (xy 0.5 19.5))))\n"
            + extra
            + "(gr_rect (start 0 0) (end 30 20) (layer \"Edge.Cuts\")))";
        Board::parse(s).unwrap()
    }

    /// Whether every pad of `net` is joined, by the net's tracks and vias, to a
    /// via that comes down on `layer`.
    fn on_the_plane(b: &Board, r: &Routed, net: &str, layer: usize) -> bool {
        let mut items: Vec<(Shape, Vec<usize>, bool, String)> = Vec::new();
        for p in b.pads.iter().filter(|p| p.net.as_deref() == Some(net)) {
            items.push((p.shape, p.layers.clone(), true, p.name()));
        }
        let pads = items.len();
        let kept = b
            .tracks
            .iter()
            .filter(|t| t.net.as_deref() == Some(net))
            .map(|t| (t.a, t.b, t.width, t.layer));
        let laid = r
            .tracks
            .iter()
            .filter(|t| t.net == net)
            .map(|t| (t.a, t.b, t.width, t.layer));
        for (a, z, w, layer) in kept.chain(laid) {
            // (a track joins layers no more than a pad does)
            items.push((Shape::Capsule { a, b: z, r: w / 2.0 }, vec![layer], true, String::new()));
        }
        let old = b
            .vias
            .iter()
            .filter(|v| v.net.as_deref() == Some(net))
            .map(|v| (v.at, v.diameter, v.layers));
        let new = r
            .vias
            .iter()
            .filter(|v| v.net == net)
            .map(|v| (v.at, v.diameter, v.layers));
        for (at, d, (a, z)) in old.chain(new) {
            items.push((
                Shape::Circle { c: at, r: d / 2.0 },
                (a..=z).collect(),
                false,
                String::new(),
            ));
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
        (0..pads).all(|i| (0..items.len()).any(|k| group[k] == group[i] && !items[k].2 && items[k].1.contains(&layer)))
    }

    #[test]
    fn pads_are_joined_to_their_plane_with_vias() {
        let b = plane_board("");
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.planes, vec![("GND".to_string(), vec![1], 3)]);
        assert_eq!(
            r.vias.iter().filter(|v| v.net == "GND").count(),
            3,
            "a via for each pad: {:?}",
            r.vias
        );
        assert!(on_the_plane(&b, &r, "GND", 1));
        assert!(r.tracks.iter().all(|t| t.layer != 1), "a track was laid on the plane");
        // no via in a pad, and each one inside the plane
        for v in r.vias.iter().filter(|v| v.net == "GND") {
            assert!(v.layers.0 == 0 && v.layers.1 >= 1);
            assert!(b.pads.iter().all(|p| p.shape.dist(v.at) > v.drill / 2.0));
        }
        assert!(
            smallest_gap(&b, &r) >= 0.2 - 1e-6,
            "copper only {:.3} mm apart",
            smallest_gap(&b, &r)
        );
        // switched off, the net is left to its pour as before
        let off = route(
            &b,
            &rules(),
            &Options {
                plane_vias: false,
                ..Options::default()
            },
        );
        assert!(off.planes.is_empty() && off.vias.iter().all(|v| v.net != "GND"));
        assert!(off.skipped.iter().any(|(n, why)| n == "GND" && why.contains("pour")));
    }

    #[test]
    fn a_pad_already_on_the_plane_gets_no_second_via() {
        // C1 has a track to a via that comes down on the plane; a fourth pad stands right beside it
        let extra = String::from("(segment (start 5 5) (end 6.5 5) (width 0.25) (layer \"F.Cu\") (net \"GND\"))\n")
            + "(via (at 6.5 5) (size 0.6) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net \"GND\"))\n"
            + &smd("C4", "GND", 5.0, 6.5, 0.8, 0.8);
        let b = plane_board(&extra);
        let r = route(&b, &rules(), &Options::default());
        assert!(r.complete(), "{:?}", r.nets);
        assert_eq!(r.planes[0].2, 3, "C1 is on the plane already: C2, C3 and C4 are not");
        assert!(on_the_plane(&b, &r, "GND", 1));
        // C4 is nearer to C1's copper than a via is worth: a track, no via
        assert_eq!(r.vias.iter().filter(|v| v.net == "GND").count(), 2, "{:?}", r.vias);
    }

    #[test]
    fn a_plane_via_is_blind_where_that_is_allowed() {
        let b = plane_board("");
        let opt = Options {
            via_spans: vec![(0, 1)],
            ..Options::default()
        };
        let r = route(&b, &rules(), &opt);
        assert!(r.complete(), "{:?}", r.nets);
        let gnd: Vec<&NewVia> = r.vias.iter().filter(|v| v.net == "GND").collect();
        assert_eq!(gnd.len(), 3);
        assert!(
            gnd.iter().all(|v| v.layers == (0, 1)),
            "from the front to the plane and no further: {:?}",
            gnd
        );
        assert!(on_the_plane(&b, &r, "GND", 1));
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
