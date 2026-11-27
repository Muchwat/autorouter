//! What the router needs from a `.kicad_pcb`: copper layers, nets, pads,
//! tracks, vias, zones and the board outline. KiCad 7 to 10 files.

use crate::geom::{self, rot, Shape, P};
use crate::sexpr::{self, Node};
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct Pad {
    pub reference: String,
    pub number: String,
    pub net: Option<String>,
    pub shape: Shape,
    /// Copper layer indices (into Board::copper).
    pub layers: Vec<usize>,
    pub through: bool,
}

impl Pad {
    pub fn name(&self) -> String {
        format!("{}.{}", self.reference, self.number)
    }
}

#[derive(Clone, Debug)]
pub struct Track {
    pub net: Option<String>,
    pub layer: usize,
    pub a: P,
    pub b: P,
    pub width: f64,
    pub locked: bool,
    /// Byte range of the item in the file; several segments share one arc's range.
    pub span: (usize, usize),
}

#[derive(Clone, Debug)]
pub struct Via {
    pub net: Option<String>,
    pub at: P,
    pub diameter: f64,
    pub drill: f64,
    /// The first and last copper layer it joins (indices into Board::copper):
    /// all of them for a through via, fewer for a blind or buried one.
    pub layers: (usize, usize),
    pub locked: bool,
    pub span: (usize, usize),
}

/// Copper that is drawn, not routed: text and graphics on a copper layer. It
/// has no net of its own (a footprint's graphic that touches one of its pads is
/// that pad's copper), and tracks keep their clearance from it like any copper.
#[derive(Clone, Debug)]
pub struct Art {
    pub net: Option<String>,
    pub shape: Shape,
    pub layers: Vec<usize>,
    pub text: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Keepout {
    pub tracks: bool,
    pub vias: bool,
}

#[derive(Clone, Debug)]
pub struct Zone {
    pub net: Option<String>,
    pub layers: Vec<usize>,
    pub outline: Vec<P>,
    /// Some for a rule area (keep-out).
    pub keepout: Option<Keepout>,
    /// The zone's own clearance to other nets' copper, and the least width of its copper.
    pub clearance: f64,
    pub min_thickness: f64,
    /// Whether pieces of the pour that reach no pad are kept when it is filled
    /// (KiCad's "remove islands" set to anything but "always").
    pub keeps_islands: bool,
}

/// How the file names a net inside an item.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NetStyle {
    /// KiCad 10: `(net "GND")`
    Name,
    /// KiCad 7 to 9: `(net 3)`, with a `(net 3 "GND")` table at the top
    Code,
}

#[derive(Clone, Debug)]
pub struct Board {
    pub text: String,
    pub version: u64,
    pub copper: Vec<String>,
    pub style: NetStyle,
    pub net_codes: BTreeMap<String, i64>,
    pub pads: Vec<Pad>,
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    pub zones: Vec<Zone>,
    /// Non-plated holes and slots, as the shape of what is drilled or milled away.
    pub holes: Vec<Shape>,
    /// Text and graphics drawn on copper layers.
    pub art: Vec<Art>,
    /// Byte ranges of the zones' stored fills (`filled_polygon`), which go
    /// stale the moment a track is added.
    pub fills: Vec<(usize, usize)>,
    /// The outline (Edge.Cuts), as zero-width segments.
    pub edges: Vec<(P, P)>,
    /// Footprint courtyards, for rules that ask about them: reference -> bbox.
    pub courtyards: BTreeMap<String, (f64, f64, f64, f64)>,
    /// Byte offset of the file's closing parenthesis, where new items go.
    pub end: usize,
}

fn xy(n: &Node) -> Option<P> {
    Some((n.num(0)?, n.num(1)?))
}

fn layer_order(name: &str) -> i64 {
    match name {
        "F.Cu" => 0,
        "B.Cu" => 10_000,
        _ => name
            .trim_start_matches("In")
            .trim_end_matches(".Cu")
            .parse::<i64>()
            .unwrap_or(5_000),
    }
}

impl Board {
    pub fn parse(text: String) -> Result<Board, String> {
        let root = sexpr::parse(&text).map_err(|e| e.to_string())?;
        if root.head() != Some("kicad_pcb") {
            return Err("not a KiCad board (no kicad_pcb at the top)".into());
        }
        let version = root.child_num("version").unwrap_or(0.0) as u64;
        let mut copper: Vec<String> = root
            .get("layers")
            .map(|l| {
                l.items()
                    .iter()
                    .filter_map(|n| n.arg(0))
                    .filter(|name| name.ends_with(".Cu"))
                    .map(|s| s.to_string())
                    .collect()
            })
            .unwrap_or_default();
        copper.sort_by_key(|n| layer_order(n));
        if copper.is_empty() {
            return Err("the board has no copper layers".into());
        }

        let mut net_codes = BTreeMap::new();
        let mut code_names = BTreeMap::new();
        for n in root.all("net") {
            if let (Some(code), Some(name)) = (n.num(0), n.arg(1)) {
                net_codes.insert(name.to_string(), code as i64);
                code_names.insert(code as i64, name.to_string());
            }
        }
        let style = if net_codes.is_empty() {
            NetStyle::Name
        } else {
            NetStyle::Code
        };
        let net_of = |n: &Node| -> Option<String> {
            let net = n.get("net")?;
            let name = match (net.items().len(), style) {
                // (net 3 "GND") in a pad, (net "GND") in KiCad 10, (net 3) in an item of a coded file
                (3, _) => net.arg(1).map(|s| s.to_string()),
                (2, NetStyle::Code) => net.num(0).and_then(|c| code_names.get(&(c as i64)).cloned()),
                (2, NetStyle::Name) => net.arg(0).map(|s| s.to_string()),
                _ => None,
            }
            .or_else(|| n.child_text("net_name").map(|s| s.to_string()))?;
            if name.is_empty() {
                None
            } else {
                Some(name)
            }
        };
        let layer_list = |n: &Node| -> Vec<usize> {
            let mut out = Vec::new();
            let mut names: Vec<String> = Vec::new();
            if let Some(l) = n.get("layers") {
                names.extend(l.args().iter().map(|s| s.to_string()));
            }
            if let Some(l) = n.get("layer") {
                names.extend(l.args().iter().map(|s| s.to_string()));
            }
            for name in names {
                match name.as_str() {
                    "*.Cu" => out.extend(0..copper.len()),
                    "F&B.Cu" => {
                        out.push(0);
                        out.push(copper.len() - 1);
                    }
                    other => {
                        if let Some(i) = copper.iter().position(|c| c == other) {
                            out.push(i);
                        }
                    }
                }
            }
            out.sort();
            out.dedup();
            out
        };

        let mut b = Board {
            version,
            copper: copper.clone(),
            style,
            net_codes: net_codes.clone(),
            pads: Vec::new(),
            tracks: Vec::new(),
            vias: Vec::new(),
            zones: Vec::new(),
            holes: Vec::new(),
            art: Vec::new(),
            fills: Vec::new(),
            edges: Vec::new(),
            courtyards: BTreeMap::new(),
            end: root.span().map(|s| s.1 - 1).unwrap_or(text.len()),
            text: String::new(),
        };

        for item in root.items() {
            match item.head() {
                Some("footprint") | Some("module") => b.footprint(item, &net_of, &layer_list),
                Some("segment") => {
                    let layer = layer_list(item);
                    if let (Some(s), Some(e), Some(&l)) = (
                        item.get("start").and_then(xy),
                        item.get("end").and_then(xy),
                        layer.first(),
                    ) {
                        b.tracks.push(Track {
                            net: net_of(item),
                            layer: l,
                            a: s,
                            b: e,
                            width: item.child_num("width").unwrap_or(0.25),
                            locked: item.flag("locked"),
                            span: item.span().unwrap(),
                        });
                    }
                }
                Some("arc") => {
                    let layer = layer_list(item);
                    let pts = (
                        item.get("start").and_then(xy),
                        item.get("mid").and_then(xy),
                        item.get("end").and_then(xy),
                    );
                    if let ((Some(s), Some(m), Some(e)), Some(&l)) = (pts, layer.first()) {
                        let pts = geom::arc_points(s, m, e, 0.2);
                        for w in pts.windows(2) {
                            b.tracks.push(Track {
                                net: net_of(item),
                                layer: l,
                                a: w[0],
                                b: w[1],
                                width: item.child_num("width").unwrap_or(0.25),
                                locked: item.flag("locked"),
                                span: item.span().unwrap(),
                            });
                        }
                    }
                }
                Some("via") => {
                    if let Some(at) = item.get("at").and_then(xy) {
                        let span = layer_list(item);
                        let layers = match (span.first(), span.last()) {
                            (Some(&a), Some(&z)) if a != z => (a, z),
                            _ => (0, copper.len() - 1),
                        };
                        b.vias.push(Via {
                            net: net_of(item),
                            layers,
                            at,
                            diameter: item.child_num("size").unwrap_or(0.8),
                            drill: item.child_num("drill").unwrap_or(0.4),
                            locked: item.flag("locked"),
                            span: item.span().unwrap(),
                        });
                    }
                }
                Some("zone") => b.zone(item, &net_of, &layer_list),
                Some(h) if h.starts_with("gr_") && item.child_text("layer") == Some("Edge.Cuts") => {
                    b.edge_item(item, (0.0, 0.0), 0.0);
                }
                Some(h) if h.starts_with("gr_") => {
                    let layers = layer_list(item);
                    if !layers.is_empty() {
                        b.art_item(item, (0.0, 0.0), 0.0, layers);
                    }
                }
                _ => {}
            }
        }
        b.text = text;
        Ok(b)
    }

    fn footprint(
        &mut self,
        fp: &Node,
        net_of: &dyn Fn(&Node) -> Option<String>,
        layer_list: &dyn Fn(&Node) -> Vec<usize>,
    ) {
        let at = fp.get("at");
        let origin = at.and_then(xy).unwrap_or((0.0, 0.0));
        let fdeg = at.and_then(|a| a.num(2)).unwrap_or(0.0);
        let reference = fp
            .all("property")
            .find(|p| p.arg(0) == Some("Reference"))
            .and_then(|p| p.arg(1))
            .or_else(|| {
                fp.all("fp_text")
                    .find(|t| t.arg(0) == Some("reference"))
                    .and_then(|t| t.arg(1))
            })
            .unwrap_or("")
            .to_string();
        let place = |p: P| geom::add(origin, rot(p, fdeg));
        let mut court: Option<(f64, f64, f64, f64)> = None;
        let (first_pad, first_art) = (self.pads.len(), self.art.len());
        for item in fp.items() {
            match item.head() {
                Some("pad") => self.pad(
                    item,
                    &reference,
                    place(item.get("at").and_then(xy).unwrap_or((0.0, 0.0))),
                    net_of,
                    layer_list,
                ),
                Some("zone") => self.zone(item, net_of, layer_list),
                Some(h) if h.starts_with("fp_") => {
                    let layer = item.child_text("layer").unwrap_or("");
                    if layer == "Edge.Cuts" {
                        self.edge_item(item, origin, fdeg);
                    }
                    let on_copper = layer_list(item);
                    if !on_copper.is_empty() {
                        self.art_item(item, origin, fdeg, on_copper);
                    }
                    if layer == "F.CrtYd" || layer == "B.CrtYd" {
                        for key in ["start", "end", "center", "mid"] {
                            if let Some(p) = item.get(key).and_then(xy) {
                                let q = place(p);
                                court = Some(match court {
                                    None => (q.0, q.1, q.0, q.1),
                                    Some(c) => (c.0.min(q.0), c.1.min(q.1), c.2.max(q.0), c.3.max(q.1)),
                                });
                            }
                        }
                        if let Some(pts) = item.get("pts") {
                            for p in pts.all("xy").filter_map(xy) {
                                let q = place(p);
                                court = Some(match court {
                                    None => (q.0, q.1, q.0, q.1),
                                    Some(c) => (c.0.min(q.0), c.1.min(q.1), c.2.max(q.0), c.3.max(q.1)),
                                });
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        if let Some(c) = court {
            if !reference.is_empty() {
                self.courtyards.insert(reference, c);
            }
        }
        // a footprint's copper graphic that touches one of its pads is that pad's
        // copper (an antenna, a net tie, a pad drawn larger): it takes the pad's net
        for k in first_art..self.art.len() {
            if self.art[k].text {
                continue;
            }
            let net = self.pads[first_pad..]
                .iter()
                .find(|p| {
                    p.layers.iter().any(|l| self.art[k].layers.contains(l)) && p.shape.touches(&self.art[k].shape)
                })
                .and_then(|p| p.net.clone());
            self.art[k].net = net;
        }
    }

    /// Text or a graphic on copper layers, as shapes tracks must keep clear of.
    /// Text is taken as the box it fills, a little on the generous side (the
    /// strokes themselves are not worked out); a filled polygon as the box
    /// round it.
    fn art_item(&mut self, item: &Node, origin: P, deg: f64, layers: Vec<usize>) {
        let place = |p: P| geom::add(origin, rot(p, deg));
        let get = |k: &str| item.get(k).and_then(xy).map(&place);
        let head = item.head().unwrap_or("");
        let kind = head.trim_start_matches("gr_").trim_start_matches("fp_");
        let half = item
            .get("stroke")
            .and_then(|s| s.child_num("width"))
            .or_else(|| item.child_num("width"))
            .unwrap_or(0.15)
            .max(0.0)
            / 2.0;
        let filled = matches!(item.child_text("fill"), Some("yes") | Some("solid"));
        let mut out: Vec<(Shape, bool)> = Vec::new();
        let chain = |pts: &[P], out: &mut Vec<(Shape, bool)>| {
            for w in pts.windows(2) {
                out.push((
                    Shape::Capsule {
                        a: w[0],
                        b: w[1],
                        r: half,
                    },
                    false,
                ));
            }
        };
        match kind {
            "line" => {
                if let (Some(a), Some(b)) = (get("start"), get("end")) {
                    chain(&[a, b], &mut out);
                }
            }
            "arc" => {
                if let (Some(s), Some(m), Some(e)) = (get("start"), get("mid"), get("end")) {
                    chain(&geom::arc_points(s, m, e, 0.1), &mut out);
                }
            }
            "circle" => {
                if let (Some(c), Some(e)) = (get("center"), get("end")) {
                    let r = geom::dist(c, e);
                    if filled {
                        out.push((Shape::Circle { c, r: r + half }, false));
                    } else {
                        let n = ((std::f64::consts::TAU * r / 0.1).ceil() as usize).max(12);
                        let pts: Vec<P> = (0..=n)
                            .map(|k| {
                                let a = std::f64::consts::TAU * k as f64 / n as f64;
                                (c.0 + r * a.cos(), c.1 + r * a.sin())
                            })
                            .collect();
                        chain(&pts, &mut out);
                    }
                }
            }
            "rect" | "text_box" => {
                if let (Some(a), Some(b)) = (item.get("start").and_then(xy), item.get("end").and_then(xy)) {
                    if filled || kind == "text_box" {
                        let c = place(((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0));
                        out.push((
                            Shape::Box {
                                c,
                                hw: (b.0 - a.0).abs() / 2.0 + half,
                                hh: (b.1 - a.1).abs() / 2.0 + half,
                                deg,
                                r: 0.0,
                            },
                            kind == "text_box",
                        ));
                    } else {
                        let c = [a, (b.0, a.1), b, (a.0, b.1), a].map(&place);
                        chain(&c, &mut out);
                    }
                }
            }
            "poly" | "curve" => {
                if let Some(pts) = item.get("pts") {
                    let mut p: Vec<P> = pts.all("xy").filter_map(xy).map(&place).collect();
                    if kind == "curve" && p.len() == 4 {
                        p = geom::bezier_points(p[0], p[1], p[2], p[3], 0.1);
                    } else if kind == "poly" && p.len() >= 3 {
                        if filled {
                            let (mut x0, mut y0, mut x1, mut y1) =
                                (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                            for q in &p {
                                x0 = x0.min(q.0);
                                y0 = y0.min(q.1);
                                x1 = x1.max(q.0);
                                y1 = y1.max(q.1);
                            }
                            out.push((
                                Shape::Box {
                                    c: ((x0 + x1) / 2.0, (y0 + y1) / 2.0),
                                    hw: (x1 - x0) / 2.0 + half,
                                    hh: (y1 - y0) / 2.0 + half,
                                    deg: 0.0,
                                    r: 0.0,
                                },
                                false,
                            ));
                        }
                        p.push(p[0]);
                    }
                    chain(&p, &mut out);
                }
            }
            "text" => {
                let effects = item.get("effects");
                if item.flag("hide") || effects.map(|e| e.flag("hide")).unwrap_or(false) {
                    return;
                }
                // (gr_text "words" ...), (fp_text user "words" ...)
                let words = if head == "fp_text" { item.arg(1) } else { item.arg(0) }.unwrap_or("");
                let words = words.replace("\\n", "\n");
                let lines: Vec<&str> = words.trim_end_matches('\n').split('\n').collect();
                let longest = lines.iter().map(|l| l.chars().count()).max().unwrap_or(0);
                if longest == 0 {
                    return;
                }
                let font = effects.and_then(|e| e.get("font"));
                let size = font.and_then(|f| f.get("size"));
                // (size height width)
                let (h, w) = (
                    size.and_then(|s| s.num(0)).unwrap_or(1.0),
                    size.and_then(|s| s.num(1)).unwrap_or(1.0),
                );
                let thick = font.and_then(|f| f.child_num("thickness")).unwrap_or(h / 8.0);
                // KiCad's own box for a line of its stroke font is up to about 1.1 widths
                // a letter and 1.7 heights; more lines stand 1.62 heights apart
                let wide = longest as f64 * w * 1.15 + thick;
                let more = (lines.len() - 1) as f64 * 1.62 * h;
                let tall = 1.7 * h + thick + more;
                let just = effects
                    .and_then(|e| e.get("justify"))
                    .map(|j| j.args())
                    .unwrap_or_default();
                let mirror = if just.contains(&"mirror") { -1.0 } else { 1.0 };
                let ox = mirror
                    * if just.contains(&"left") {
                        wide / 2.0
                    } else if just.contains(&"right") {
                        -wide / 2.0
                    } else {
                        0.0
                    };
                let oy = if just.contains(&"top") {
                    tall / 2.0
                } else if just.contains(&"bottom") {
                    -tall / 2.0
                } else {
                    0.0
                };
                let at = item.get("at");
                let angle = at.and_then(|a| a.num(2)).unwrap_or(0.0);
                let anchor = at.and_then(xy).unwrap_or((0.0, 0.0));
                let c = place(geom::add(anchor, rot((ox, oy), angle)));
                // (in a footprint the text's own angle and the footprint's do not simply
                // add up from one KiCad to the next: a square box holds either way; and
                // lines after the first may run on down or be centred on the anchor)
                let (hw, hh) = if deg != 0.0 || head == "fp_text" {
                    let m = (wide / 2.0).max(tall / 2.0);
                    (m, m)
                } else {
                    (wide / 2.0, tall / 2.0 + more / 2.0)
                };
                out.push((
                    Shape::Box {
                        c,
                        hw,
                        hh,
                        deg: angle,
                        r: 0.0,
                    },
                    true,
                ));
            }
            _ => {}
        }
        for (shape, text) in out {
            self.art.push(Art {
                net: None,
                shape,
                layers: layers.clone(),
                text,
            });
        }
    }

    fn pad(
        &mut self,
        pad: &Node,
        reference: &str,
        pos: P,
        net_of: &dyn Fn(&Node) -> Option<String>,
        layer_list: &dyn Fn(&Node) -> Vec<usize>,
    ) {
        let kind = pad.arg(1).unwrap_or("smd");
        let form = pad.arg(2).unwrap_or("rect");
        // the pad's angle in a board file is its total angle, footprint's included
        let deg = pad.get("at").and_then(|a| a.num(2)).unwrap_or(0.0);
        let size = pad.get("size").and_then(xy).unwrap_or((1.0, 1.0));
        let drill = pad.get("drill");
        if kind == "np_thru_hole" {
            // (drill 3.2) is round; (drill oval 1 3) is a slot, turned with the pad
            let nums: Vec<f64> = drill
                .map(|d| d.args().iter().filter_map(|a| a.parse::<f64>().ok()).collect())
                .unwrap_or_default();
            let (dw, dh) = match nums.len() {
                0 => (size.0, size.1),
                1 => (nums[0], nums[0]),
                _ => (nums[0], nums[1]),
            };
            let half = rot(
                if dw >= dh {
                    ((dw - dh) / 2.0, 0.0)
                } else {
                    (0.0, (dh - dw) / 2.0)
                },
                deg,
            );
            let r = dw.min(dh) / 2.0;
            self.holes.push(if (dw - dh).abs() < 1e-9 {
                Shape::Circle { c: pos, r }
            } else {
                Shape::Capsule {
                    a: (pos.0 - half.0, pos.1 - half.1),
                    b: (pos.0 + half.0, pos.1 + half.1),
                    r,
                }
            });
            return;
        }
        let offset = drill.and_then(|d| d.get("offset")).and_then(xy).unwrap_or((0.0, 0.0));
        let c = geom::add(pos, rot(offset, deg));
        let (w, h) = size;
        let shape = match form {
            "circle" => Shape::Circle { c, r: w / 2.0 },
            "oval" => {
                if (w - h).abs() < 1e-9 {
                    Shape::Circle { c, r: w / 2.0 }
                } else if w > h {
                    let d = rot(((w - h) / 2.0, 0.0), deg);
                    Shape::Capsule {
                        a: (c.0 - d.0, c.1 - d.1),
                        b: (c.0 + d.0, c.1 + d.1),
                        r: h / 2.0,
                    }
                } else {
                    let d = rot((0.0, (h - w) / 2.0), deg);
                    Shape::Capsule {
                        a: (c.0 - d.0, c.1 - d.1),
                        b: (c.0 + d.0, c.1 + d.1),
                        r: w / 2.0,
                    }
                }
            }
            "roundrect" => {
                let ratio = pad.child_num("roundrect_rratio").unwrap_or(0.25);
                Shape::Box {
                    c,
                    hw: w / 2.0,
                    hh: h / 2.0,
                    deg,
                    r: ratio * w.min(h),
                }
            }
            "trapezoid" => {
                let d = pad.get("rect_delta").and_then(xy).unwrap_or((0.0, 0.0));
                Shape::Box {
                    c,
                    hw: (w + d.1.abs()) / 2.0,
                    hh: (h + d.0.abs()) / 2.0,
                    deg,
                    r: 0.0,
                }
            }
            "custom" => {
                // the anchor plus the bounding box of the primitives, in the pad's own frame
                let (mut x0, mut y0, mut x1, mut y1) = (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0);
                if let Some(prims) = pad.get("primitives") {
                    for p in prims.items().iter().skip(1) {
                        let width = p.child_num("width").unwrap_or(0.0) / 2.0;
                        let mut pts: Vec<P> = Vec::new();
                        for key in ["start", "end", "center", "mid"] {
                            if let Some(q) = p.get(key).and_then(xy) {
                                pts.push(q);
                            }
                        }
                        if p.head() == Some("gr_circle") {
                            if let (Some(cc), Some(e)) = (p.get("center").and_then(xy), p.get("end").and_then(xy)) {
                                let r = geom::dist(cc, e);
                                pts.push((cc.0 - r, cc.1 - r));
                                pts.push((cc.0 + r, cc.1 + r));
                            }
                        }
                        if let Some(ps) = p.get("pts") {
                            pts.extend(ps.all("xy").filter_map(xy));
                        }
                        for q in pts {
                            x0 = x0.min(q.0 - width);
                            y0 = y0.min(q.1 - width);
                            x1 = x1.max(q.0 + width);
                            y1 = y1.max(q.1 + width);
                        }
                    }
                }
                let mid = rot(((x0 + x1) / 2.0, (y0 + y1) / 2.0), deg);
                Shape::Box {
                    c: geom::add(c, mid),
                    hw: (x1 - x0) / 2.0,
                    hh: (y1 - y0) / 2.0,
                    deg,
                    r: 0.0,
                }
            }
            _ => Shape::Box {
                c,
                hw: w / 2.0,
                hh: h / 2.0,
                deg,
                r: 0.0,
            },
        };
        let layers = layer_list(pad);
        if layers.is_empty() {
            return; // a pad on no copper layer (a paste or mask aperture)
        }
        self.pads.push(Pad {
            reference: reference.to_string(),
            number: pad.arg(0).unwrap_or("").to_string(),
            net: net_of(pad),
            shape,
            layers,
            through: kind == "thru_hole",
        });
    }

    fn zone(&mut self, z: &Node, net_of: &dyn Fn(&Node) -> Option<String>, layer_list: &dyn Fn(&Node) -> Vec<usize>) {
        for fill in z.all("filled_polygon").chain(z.all("fill_segments")) {
            if let Some(span) = fill.span() {
                self.fills.push(span);
            }
        }
        let layers = layer_list(z);
        let outline: Vec<P> = z
            .get("polygon")
            .and_then(|p| p.get("pts"))
            .map(|pts| pts.all("xy").filter_map(xy).collect())
            .unwrap_or_default();
        if outline.len() < 3 || layers.is_empty() {
            return;
        }
        let keepout = z.get("keepout").map(|k| Keepout {
            tracks: k.child_text("tracks") == Some("not_allowed"),
            vias: k.child_text("vias") == Some("not_allowed"),
        });
        if let Some(k) = &keepout {
            if !k.tracks && !k.vias {
                return; // a rule area that does not limit routing
            }
        }
        self.zones.push(Zone {
            net: net_of(z),
            layers,
            outline,
            keepout,
            clearance: z
                .get("connect_pads")
                .and_then(|c| c.child_num("clearance"))
                .unwrap_or(0.0),
            min_thickness: z.child_num("min_thickness").unwrap_or(0.25),
            keeps_islands: z
                .get("fill")
                .and_then(|f| f.child_num("island_removal_mode"))
                .map(|m| m != 0.0)
                .unwrap_or(false),
        });
    }

    fn edge_item(&mut self, item: &Node, origin: P, deg: f64) {
        let place = |p: P| geom::add(origin, rot(p, deg));
        let get = |k: &str| item.get(k).and_then(xy).map(&place);
        let head = item.head().unwrap_or("");
        let kind = head.trim_start_matches("gr_").trim_start_matches("fp_");
        match kind {
            "line" => {
                if let (Some(a), Some(b)) = (get("start"), get("end")) {
                    self.edges.push((a, b));
                }
            }
            "arc" => {
                if let (Some(s), Some(m), Some(e)) = (get("start"), get("mid"), get("end")) {
                    let pts = geom::arc_points(s, m, e, 0.1);
                    for w in pts.windows(2) {
                        self.edges.push((w[0], w[1]));
                    }
                }
            }
            "circle" => {
                if let (Some(c), Some(e)) = (get("center"), get("end")) {
                    let r = geom::dist(c, e);
                    let n = ((std::f64::consts::TAU * r / 0.1).ceil() as usize).max(12);
                    let pts: Vec<P> = (0..=n)
                        .map(|k| {
                            let a = std::f64::consts::TAU * k as f64 / n as f64;
                            (c.0 + r * a.cos(), c.1 + r * a.sin())
                        })
                        .collect();
                    for w in pts.windows(2) {
                        self.edges.push((w[0], w[1]));
                    }
                }
            }
            "rect" => {
                if let (Some(a), Some(b)) = (item.get("start").and_then(xy), item.get("end").and_then(xy)) {
                    let c = [a, (b.0, a.1), b, (a.0, b.1)].map(&place);
                    for k in 0..4 {
                        self.edges.push((c[k], c[(k + 1) % 4]));
                    }
                }
            }
            "curve" => {
                // a cubic Bezier: (pts (xy start) (xy control) (xy control) (xy end))
                if let Some(pts) = item.get("pts") {
                    let p: Vec<P> = pts.all("xy").filter_map(xy).map(&place).collect();
                    if p.len() == 4 {
                        let pts = geom::bezier_points(p[0], p[1], p[2], p[3], 0.1);
                        for w in pts.windows(2) {
                            self.edges.push((w[0], w[1]));
                        }
                    }
                }
            }
            "poly" => {
                if let Some(pts) = item.get("pts") {
                    let p: Vec<P> = pts.all("xy").filter_map(xy).map(place).collect();
                    for k in 0..p.len() {
                        self.edges.push((p[k], p[(k + 1) % p.len()]));
                    }
                }
            }
            _ => {}
        }
    }

    /// The board's extent: the outline's bounding box, or every pad's when there is no outline.
    pub fn extent(&self) -> Option<(f64, f64, f64, f64)> {
        let mut bb: Option<(f64, f64, f64, f64)> = None;
        let mut grow = |x0: f64, y0: f64, x1: f64, y1: f64| {
            bb = Some(match bb {
                None => (x0, y0, x1, y1),
                Some(b) => (b.0.min(x0), b.1.min(y0), b.2.max(x1), b.3.max(y1)),
            });
        };
        if !self.edges.is_empty() {
            for (a, b) in &self.edges {
                grow(a.0.min(b.0), a.1.min(b.1), a.0.max(b.0), a.1.max(b.1));
            }
        } else {
            for p in &self.pads {
                let (x0, y0, x1, y1) = p.shape.bbox();
                grow(x0, y0, x1, y1);
            }
        }
        bb
    }

    /// Nets that have a copper pour (not a keep-out) on some layer.
    pub fn poured_nets(&self) -> Vec<String> {
        let mut v: Vec<String> = self
            .zones
            .iter()
            .filter(|z| z.keepout.is_none())
            .filter_map(|z| z.net.clone())
            .collect();
        v.sort();
        v.dedup();
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KICAD10: &str = r#"(kicad_pcb (version 20260206)
      (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
      (footprint "R" (layer "F.Cu") (at 10 10 90)
        (property "Reference" "R1")
        (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu" "*.Mask") (net "A"))
        (pad "2" smd roundrect (at 2 0 90) (size 1 2) (layers "F.Cu" "F.Mask") (roundrect_rratio 0.25) (net "B")))
      (segment (start 0 0) (end 1 0) (width 0.25) (layer "B.Cu") (net "A") (uuid "x"))
      (via (at 5 5) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net "B"))
      (zone (net "A") (layer "F.Cu") (polygon (pts (xy 0 0) (xy 20 0) (xy 20 20))))
      (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))
    )"#;

    #[test]
    fn reads_a_kicad10_board() {
        let b = Board::parse(KICAD10.to_string()).unwrap();
        assert_eq!(b.copper, vec!["F.Cu", "B.Cu"]);
        assert_eq!(b.style, NetStyle::Name);
        assert_eq!(b.pads.len(), 2);
        assert_eq!(b.pads[0].layers, vec![0, 1]);
        assert_eq!(b.pads[1].layers, vec![0]);
        // local (2, 0) in a footprint turned 90 degrees: (10, 8)
        let c = b.pads[1].shape.centre();
        assert!((c.0 - 10.0).abs() < 1e-9 && (c.1 - 8.0).abs() < 1e-9, "{:?}", c);
        assert_eq!(b.tracks[0].net.as_deref(), Some("A"));
        assert_eq!(b.tracks[0].layer, 1);
        assert_eq!(b.vias[0].net.as_deref(), Some("B"));
        assert_eq!(b.vias[0].layers, (0, 1));
        assert_eq!(b.poured_nets(), vec!["A".to_string()]);
        assert_eq!(b.edges.len(), 4);
        assert_eq!(b.extent(), Some((0.0, 0.0, 20.0, 20.0)));
    }

    #[test]
    fn slots_curves_and_fills() {
        let t = r#"(kicad_pcb (version 20260206) (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
          (footprint "J" (at 10 10 90) (property "Reference" "J1")
            (pad "" np_thru_hole oval (at 0 0 90) (size 1 3) (drill oval 1 3) (layers "*.Cu" "*.Mask"))
            (pad "" np_thru_hole circle (at 5 0 90) (size 2 2) (drill 2) (layers "*.Cu" "*.Mask")))
          (zone (net "GND") (layer "F.Cu") (polygon (pts (xy 0 0) (xy 20 0) (xy 20 20)))
            (filled_polygon (layer "F.Cu") (pts (xy 0 0) (xy 20 0) (xy 20 20))))
          (gr_curve (pts (xy 0 0) (xy 0 10) (xy 10 20) (xy 20 20)) (layer "Edge.Cuts")))"#;
        let b = Board::parse(t.to_string()).unwrap();
        // the slot is 3 mm long in the pad's frame, which is turned 90 degrees: it runs along x
        match b.holes[0] {
            Shape::Capsule { a, b, r } => {
                assert!((r - 0.5).abs() < 1e-9);
                assert!(
                    (a.1 - 10.0).abs() < 1e-9 && (b.1 - 10.0).abs() < 1e-9,
                    "{:?} {:?}",
                    a,
                    b
                );
                assert!(((a.0 - b.0).abs() - 2.0).abs() < 1e-9);
            }
            other => panic!("the slot came out as {:?}", other),
        }
        assert_eq!(b.holes[1], Shape::Circle { c: (10.0, 5.0), r: 1.0 });
        assert_eq!(b.fills.len(), 1);
        assert!(b.text[b.fills[0].0..b.fills[0].1].starts_with("(filled_polygon"));
        // the curve is a chain of short segments from its first point to its last
        assert!(b.edges.len() > 20);
        assert_eq!(b.edges[0].0, (0.0, 0.0));
        let last = b.edges[b.edges.len() - 1].1;
        assert!((last.0 - 20.0).abs() < 1e-9 && (last.1 - 20.0).abs() < 1e-9);
    }

    #[test]
    fn reads_blind_and_buried_vias() {
        let t = r#"(kicad_pcb (version 20260206)
          (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal))
          (via (at 1 1) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "A"))
          (via blind (at 2 1) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") (net "A"))
          (via buried (at 3 1) (size 0.6) (drill 0.3) (layers "In1.Cu" "In2.Cu") (net "A"))
          (via blind (at 4 1) (size 0.6) (drill 0.3) (layers "In2.Cu" "B.Cu") (net "A"))
          (via micro locked (at 5 1) (size 0.3) (drill 0.1) (layers "F.Cu" "In1.Cu") (net "A")))"#;
        let b = Board::parse(t.to_string()).unwrap();
        let spans: Vec<(usize, usize)> = b.vias.iter().map(|v| v.layers).collect();
        assert_eq!(spans, vec![(0, 3), (0, 1), (1, 2), (2, 3), (0, 1)]);
        assert!(b.vias[4].locked && !b.vias[1].locked);
    }

    #[test]
    fn reads_net_codes() {
        let t = r#"(kicad_pcb (version 20240108) (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
          (net 0 "") (net 1 "GND")
          (footprint "X" (at 0 0) (fp_text reference "U1" (at 0 0))
            (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
          (segment (start 0 0) (end 1 0) (width 0.25) (layer "F.Cu") (net 1)))"#;
        let b = Board::parse(t.to_string()).unwrap();
        assert_eq!(b.style, NetStyle::Code);
        assert_eq!(b.pads[0].net.as_deref(), Some("GND"));
        assert_eq!(b.pads[0].reference, "U1");
        assert_eq!(b.tracks[0].net.as_deref(), Some("GND"));
        assert_eq!(b.net_codes.get("GND"), Some(&1));
    }
    #[test]
    fn text_and_graphics_on_copper_are_read() {
        // the box KiCad itself gives 'MOD1' at 109, 96 in a 1 mm font 0.2 thick is
        // 106.73 to 111.27 across and 95.06 to 96.94 down
        let t = r#"(kicad_pcb (version 20260206) (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user) (37 "F.SilkS" user))
          (gr_text "MOD1" (at 109 96 0) (layer "F.Cu") (effects (font (size 1 1) (thickness 0.2))))
          (gr_text "silk" (at 50 50 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.2))))
          (gr_text "unseen" (at 60 60 0) (layer "F.Cu") (effects (font (size 1 1) (thickness 0.2)) (hide yes)))
          (gr_text "AB" (at 20 20 0) (layer "B.Cu") (effects (font (size 2 1) (thickness 0.2)) (justify left mirror)))
          (gr_line (start 30 30) (end 40 30) (stroke (width 0.5) (type solid)) (layer "F.Cu"))
          (gr_rect (start 70 70) (end 74 72) (stroke (width 0.2) (type solid)) (fill yes) (layer "B.Cu"))
          (gr_rect (start 0 0) (end 120 120) (layer "Edge.Cuts")))"#;
        let b = Board::parse(t.to_string()).unwrap();
        assert_eq!(b.art.len(), 4, "{:?}", b.art);
        let mod1 = &b.art[0];
        assert!(mod1.text && mod1.net.is_none() && mod1.layers == vec![0]);
        match mod1.shape {
            Shape::Box { c, hw, hh, deg, .. } => {
                assert_eq!((c, deg), ((109.0, 96.0), 0.0));
                // it holds KiCad's box, and is no more than a sixth larger
                assert!(hw >= 2.27 && hw <= 2.27 * 1.17, "half width {}", hw);
                assert!(hh >= 0.94 && hh <= 0.94 * 1.17, "half height {}", hh);
            }
            other => panic!("{:?}", other),
        }
        // mirrored and held by its left end, the words run to the left of the anchor
        match b.art[1].shape {
            Shape::Box { c, hw, .. } => {
                assert_eq!(b.art[1].layers, vec![1]);
                assert!(c.0 < 20.0 && (c.0 + hw - 20.0).abs() < 1e-9, "{:?} {}", c, hw);
            }
            other => panic!("{:?}", other),
        }
        assert_eq!(
            b.art[2].shape,
            Shape::Capsule {
                a: (30.0, 30.0),
                b: (40.0, 30.0),
                r: 0.25
            }
        );
        assert!(!b.art[2].text);
        assert_eq!(
            b.art[3].shape,
            Shape::Box {
                c: (72.0, 71.0),
                hw: 2.1,
                hh: 1.1,
                deg: 0.0,
                r: 0.0
            }
        );
    }

    #[test]
    fn a_footprint_graphic_on_a_pad_takes_its_net() {
        let t = r#"(kicad_pcb (version 20260206) (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
          (footprint "ANT" (at 10 10) (property "Reference" "A")
            (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net "RF"))
            (fp_line (start 0 0) (end 5 0) (stroke (width 0.4) (type solid)) (layer "F.Cu"))
            (fp_line (start 0 4) (end 5 4) (stroke (width 0.4) (type solid)) (layer "F.Cu")))
          (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts")))"#;
        let b = Board::parse(t.to_string()).unwrap();
        assert_eq!(b.art.len(), 2);
        assert_eq!(b.art[0].net.as_deref(), Some("RF"), "it starts on the pad");
        assert_eq!(b.art[1].net, None, "it touches no pad");
        assert_eq!(
            b.art[1].shape,
            Shape::Capsule {
                a: (10.0, 14.0),
                b: (15.0, 14.0),
                r: 0.2
            }
        );
    }
}
