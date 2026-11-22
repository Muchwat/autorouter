//! KiCad's design rules, as far as routing needs them.
//!
//! * Net classes from the `.kicad_pro`: clearance, track width and via size,
//!   with net assignments and wildcard patterns.
//! * The board minimums from the `.kicad_pro`: clearance and copper-to-edge.
//! * Custom rules from the `.kicad_dru`: `clearance`, `track_width`,
//!   `edge_clearance` and `disallow via` constraints, with their conditions.
//!
//! Conditions are evaluated with three values: true, false and "can't tell"
//! (a test the router cannot answer, such as where a track will run). A rule
//! that might apply is treated as applying, so the router errs on the safe
//! side and KiCad's DRC has the final word.

use crate::board::Board;
use crate::geom::Shape;
use crate::json::{self, Json};
use crate::sexpr::{self, Node};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

#[derive(Clone, Debug)]
pub struct NetClass {
    pub name: String,
    pub clearance: f64,
    pub track_width: f64,
    pub via_diameter: f64,
    pub via_drill: f64,
}

/// What a rule can know about one of the two items it compares.
#[derive(Clone, Copy, Debug)]
pub struct Item<'a> {
    pub net: Option<&'a str>,
    /// "Pad", "Track" or "Via"; None when unknown.
    pub kind: Option<&'static str>,
    /// Where the item is, when it is fixed copper.
    pub shape: Option<&'a Shape>,
}

impl<'a> Item<'a> {
    pub fn track(net: &'a str) -> Item<'a> {
        Item {
            net: Some(net),
            kind: Some("Track"),
            shape: None,
        }
    }
    pub fn via(net: &'a str) -> Item<'a> {
        Item {
            net: Some(net),
            kind: Some("Via"),
            shape: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum T {
    Yes,
    No,
    Maybe,
}

fn not(t: T) -> T {
    match t {
        T::Yes => T::No,
        T::No => T::Yes,
        T::Maybe => T::Maybe,
    }
}

fn and(a: T, b: T) -> T {
    match (a, b) {
        (T::No, _) | (_, T::No) => T::No,
        (T::Yes, T::Yes) => T::Yes,
        _ => T::Maybe,
    }
}

fn or(a: T, b: T) -> T {
    match (a, b) {
        (T::Yes, _) | (_, T::Yes) => T::Yes,
        (T::No, T::No) => T::No,
        _ => T::Maybe,
    }
}

#[derive(Clone, Debug)]
enum Expr {
    Or(Box<Expr>, Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Not(Box<Expr>),
    Cmp(Val, bool, Val), // true: ==, false: !=
    Call(char, String, Vec<String>),
    Unknown,
}

#[derive(Clone, Debug)]
enum Val {
    Prop(char, String),
    Str(String),
    Num(f64),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Kind {
    Clearance,
    TrackWidth,
    EdgeClearance,
    DisallowVia,
}

#[derive(Clone, Debug)]
pub struct Constraint {
    pub kind: Kind,
    pub min: Option<f64>,
    pub opt: Option<f64>,
    pub max: Option<f64>,
}

#[derive(Clone, Debug)]
pub struct Rule {
    pub name: String,
    pub constraints: Vec<Constraint>,
    condition: Option<Expr>,
}

#[derive(Clone, Debug)]
pub struct Rules {
    pub classes: Vec<NetClass>,
    assignments: BTreeMap<String, String>,
    patterns: Vec<(String, String)>,
    pub min_clearance: f64,
    pub edge_clearance: f64,
    /// The least gap between the edges of two drilled holes.
    pub hole_to_hole: f64,
    /// The least gap between a hole's edge and other copper.
    pub hole_clearance: f64,
    pub rules: Vec<Rule>,
    courtyards: BTreeMap<String, (f64, f64, f64, f64)>,
    /// Things worth telling the user (a rule part this router cannot read, say).
    pub notes: Vec<String>,
}

impl Default for Rules {
    fn default() -> Self {
        Rules {
            classes: vec![NetClass {
                name: "Default".into(),
                clearance: 0.2,
                track_width: 0.25,
                via_diameter: 0.6,
                via_drill: 0.3,
            }],
            assignments: BTreeMap::new(),
            patterns: Vec::new(),
            min_clearance: 0.0,
            edge_clearance: 0.5,
            hole_to_hole: 0.25,
            hole_clearance: 0.25,
            rules: Vec::new(),
            courtyards: BTreeMap::new(),
            notes: Vec::new(),
        }
    }
}

/// KiCad's wildcard match: `*` any run, `?` any one character.
pub fn wildcard(pattern: &str, s: &str) -> bool {
    if !pattern.contains(['*', '?']) {
        return pattern == s; // the usual case, with nothing to allocate
    }
    let p: Vec<char> = pattern.chars().collect();
    let t: Vec<char> = s.chars().collect();
    let (mut i, mut j, mut star, mut mark) = (0usize, 0usize, None, 0usize);
    while j < t.len() {
        if i < p.len() && (p[i] == '?' || p[i] == t[j]) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some(i);
            mark = j;
            i += 1;
        } else if let Some(si) = star {
            i = si + 1;
            mark += 1;
            j = mark;
        } else {
            return false;
        }
    }
    while i < p.len() && p[i] == '*' {
        i += 1;
    }
    i == p.len()
}

/// "8mm", "10mil", "0.2" -> millimetres.
fn length(s: &str) -> Option<f64> {
    let s = s.trim();
    let (num, scale) = if let Some(n) = s.strip_suffix("mm") {
        (n, 1.0)
    } else if let Some(n) = s.strip_suffix("mil") {
        (n, 0.0254)
    } else if let Some(n) = s.strip_suffix("in") {
        (n, 25.4)
    } else if let Some(n) = s.strip_suffix("um") {
        (n, 0.001)
    } else {
        (s, 1.0)
    };
    num.trim().parse::<f64>().ok().map(|v| v * scale)
}

impl Rules {
    /// The rules for a board: its `.kicad_pro` and `.kicad_dru` beside it, when they exist.
    pub fn for_board(pcb_path: impl AsRef<Path>, board: &Board) -> Result<Rules, String> {
        let pcb = pcb_path.as_ref();
        let with = |ext: &str| -> PathBuf { pcb.with_extension(ext) };
        let mut r = Rules::default();
        if let Ok(text) = std::fs::read_to_string(with("kicad_pro")) {
            r.read_project(&text)?;
        }
        if let Ok(text) = std::fs::read_to_string(with("kicad_dru")) {
            r.read_dru(&text)?;
        }
        r.courtyards = board.courtyards.clone();
        Ok(r)
    }

    pub fn read_project(&mut self, text: &str) -> Result<(), String> {
        let j = json::parse(text).map_err(|e| format!(".kicad_pro: {}", e))?;
        if let Some(rules) = j.path(&["board", "design_settings", "rules"]) {
            if let Some(v) = rules.get("min_clearance").and_then(Json::as_f64) {
                self.min_clearance = v;
            }
            if let Some(v) = rules.get("min_copper_edge_clearance").and_then(Json::as_f64) {
                self.edge_clearance = v;
            }
            if let Some(v) = rules.get("min_hole_to_hole").and_then(Json::as_f64) {
                self.hole_to_hole = v;
            }
            if let Some(v) = rules.get("min_hole_clearance").and_then(Json::as_f64) {
                self.hole_clearance = v;
            }
        }
        if let Some(ns) = j.get("net_settings") {
            let builtin = self.classes[0].clone();
            let list = ns.get("classes").map(Json::as_arr).unwrap_or(&[]);
            // other classes may leave fields out: they take them from the project's Default
            let d = match list
                .iter()
                .find(|c| c.get("name").and_then(Json::as_str) == Some("Default"))
            {
                Some(c) => {
                    let f = |k: &str, dv: f64| c.get(k).and_then(Json::as_f64).unwrap_or(dv);
                    NetClass {
                        name: "Default".into(),
                        clearance: f("clearance", builtin.clearance),
                        track_width: f("track_width", builtin.track_width),
                        via_diameter: f("via_diameter", builtin.via_diameter),
                        via_drill: f("via_drill", builtin.via_drill),
                    }
                }
                None => builtin,
            };
            let mut classes: Vec<NetClass> = Vec::new();
            for c in list {
                let f = |k: &str, dv: f64| c.get(k).and_then(Json::as_f64).unwrap_or(dv);
                let name = c.get("name").and_then(Json::as_str).unwrap_or("").to_string();
                classes.push(NetClass {
                    clearance: f("clearance", d.clearance),
                    track_width: f("track_width", d.track_width),
                    via_diameter: f("via_diameter", d.via_diameter),
                    via_drill: f("via_drill", d.via_drill),
                    name,
                });
            }
            if let Some(pos) = classes.iter().position(|c| c.name == "Default") {
                let dflt = classes.remove(pos);
                classes.insert(0, dflt);
            } else {
                classes.insert(0, d);
            }
            self.classes = classes;
            if let Some(m) = ns.get("netclass_assignments").and_then(Json::as_obj) {
                for (net, cls) in m {
                    let name = match cls {
                        Json::Str(s) => Some(s.clone()),
                        Json::Arr(a) => a.iter().filter_map(Json::as_str).next().map(|s| s.to_string()),
                        _ => None,
                    };
                    if let Some(n) = name {
                        self.assignments.insert(net.clone(), n);
                    }
                }
            }
            for p in ns.get("netclass_patterns").map(Json::as_arr).unwrap_or(&[]) {
                if let (Some(pat), Some(cls)) = (
                    p.get("pattern").and_then(Json::as_str),
                    p.get("netclass").and_then(Json::as_str),
                ) {
                    self.patterns.push((pat.to_string(), cls.to_string()));
                }
            }
        }
        Ok(())
    }

    pub fn read_dru(&mut self, text: &str) -> Result<(), String> {
        let root = sexpr::parse(&format!("(dru {})", text)).map_err(|e| format!(".kicad_dru: {}", e))?;
        for rule in root.all("rule") {
            let name = rule.arg(0).unwrap_or("").to_string();
            let mut constraints = Vec::new();
            for c in rule.all("constraint") {
                let kind = match c.arg(0) {
                    Some("clearance") => Kind::Clearance,
                    Some("track_width") => Kind::TrackWidth,
                    Some("edge_clearance") => Kind::EdgeClearance,
                    Some("disallow") => {
                        if c.args().iter().skip(1).any(|a| *a == "via" || *a == "through_via") {
                            Kind::DisallowVia
                        } else {
                            continue;
                        }
                    }
                    _ => continue,
                };
                let val = |k: &str| c.get(k).and_then(|n: &Node| n.arg(0)).and_then(length);
                constraints.push(Constraint {
                    kind,
                    min: val("min"),
                    opt: val("opt"),
                    max: val("max"),
                });
            }
            if constraints.is_empty() {
                continue;
            }
            if rule.get("layer").is_some() {
                self.notes.push(format!(
                    "rule \"{}\" is limited to some layers; the router applies it on every layer",
                    name
                ));
            }
            let condition = match rule.child_text("condition") {
                Some(text) => {
                    let e = parse_expr(text);
                    if contains_unknown(&e) {
                        self.notes.push(format!(
                            "rule \"{}\": part of its condition is beyond this router, which applies the rule wherever it might",
                            name
                        ));
                    }
                    Some(e)
                }
                None => None,
            };
            self.rules.push(Rule {
                name,
                constraints,
                condition,
            });
        }
        Ok(())
    }

    /// Two nets with the same signature are treated alike by every rule (same
    /// net class, and the same answer to every net-name test in the rules), so
    /// the router may keep their copper in one pile.
    pub fn signature(&self, net: &str) -> String {
        let mut s = self.class_of(net).name.clone();
        let mut lits: Vec<&str> = Vec::new();
        for r in &self.rules {
            if let Some(e) = &r.condition {
                literals(e, &mut lits);
            }
        }
        for l in lits {
            s.push(if wildcard(l, net) || l.eq_ignore_ascii_case(net) {
                '1'
            } else {
                '0'
            });
        }
        s
    }

    pub fn class_of(&self, net: &str) -> &NetClass {
        let name: Option<&str> = self.assignments.get(net).map(|s| s.as_str()).or_else(|| {
            self.patterns
                .iter()
                .find(|(p, _)| p == net || wildcard(p, net))
                .map(|(_, c)| c.as_str())
        });
        name.and_then(|n| self.classes.iter().find(|c| c.name == n))
            .unwrap_or(&self.classes[0])
    }

    fn eval<'v>(&'v self, e: &'v Expr, a: &Item<'v>, b: &Item<'v>) -> T {
        match e {
            Expr::Or(x, y) => or(self.eval(x, a, b), self.eval(y, a, b)),
            Expr::And(x, y) => and(self.eval(x, a, b), self.eval(y, a, b)),
            Expr::Not(x) => not(self.eval(x, a, b)),
            Expr::Unknown => T::Maybe,
            Expr::Cmp(l, eq, r) => {
                let (l, r) = (self.value(l, a, b), self.value(r, a, b));
                match (l, r) {
                    (Some(l), Some(r)) => {
                        let m = wildcard(&r, &l) || wildcard(&l, &r) || l.eq_ignore_ascii_case(&r);
                        if m == *eq {
                            T::Yes
                        } else {
                            T::No
                        }
                    }
                    _ => T::Maybe,
                }
            }
            Expr::Call(side, func, args) => {
                let it = if *side == 'A' { a } else { b };
                match func.as_str() {
                    "intersectsCourtyard"
                    | "insideCourtyard"
                    | "intersectsFrontCourtyard"
                    | "intersectsBackCourtyard" => {
                        let shape = match it.shape {
                            Some(s) => s,
                            None => return T::Maybe,
                        };
                        let target = args.first().cloned().unwrap_or_default();
                        let (x0, y0, x1, y1) = shape.bbox();
                        let hit = self
                            .courtyards
                            .iter()
                            .filter(|(r, _)| wildcard(&target, r))
                            .any(|(_, c)| {
                                if func == "insideCourtyard" {
                                    x0 >= c.0 && y0 >= c.1 && x1 <= c.2 && y1 <= c.3
                                } else {
                                    !(x1 < c.0 || x0 > c.2 || y1 < c.1 || y0 > c.3)
                                }
                            });
                        if hit {
                            T::Yes
                        } else {
                            T::No
                        }
                    }
                    "hasNetclass" | "hasExactNetclass" => match it.net {
                        Some(n) => {
                            let target = args.first().cloned().unwrap_or_default();
                            if wildcard(&target, &self.class_of(n).name) {
                                T::Yes
                            } else {
                                T::No
                            }
                        }
                        None => T::No,
                    },
                    "isPlated" => match it.kind {
                        Some("Via") => T::Yes,
                        Some("Track") => T::No,
                        _ => T::Maybe,
                    },
                    _ => T::Maybe,
                }
            }
        }
    }

    /// The text a value stands for, borrowed where it can be (this runs for
    /// every comparison of every rule, so it must not copy strings).
    fn value<'v>(&'v self, v: &'v Val, a: &Item<'v>, b: &Item<'v>) -> Option<std::borrow::Cow<'v, str>> {
        use std::borrow::Cow;
        match v {
            Val::Str(s) => Some(Cow::Borrowed(s.as_str())),
            Val::Num(n) => Some(Cow::Owned(n.to_string())),
            Val::Prop(side, prop) => {
                let it = if *side == 'A' { a } else { b };
                if prop.eq_ignore_ascii_case("netname") {
                    Some(Cow::Borrowed(it.net.unwrap_or("")))
                } else if prop.eq_ignore_ascii_case("netclass") || prop.eq_ignore_ascii_case("net_class") {
                    Some(Cow::Borrowed(
                        it.net.map(|n| self.class_of(n).name.as_str()).unwrap_or(""),
                    ))
                } else if prop.eq_ignore_ascii_case("type") {
                    it.kind.map(Cow::Borrowed)
                } else {
                    None
                }
            }
        }
    }

    fn applies(&self, rule: &Rule, a: &Item, b: &Item, pair: bool) -> T {
        match &rule.condition {
            None => T::Yes,
            Some(e) => {
                let ab = self.eval(e, a, b);
                if pair {
                    or(ab, self.eval(e, b, a))
                } else {
                    ab
                }
            }
        }
    }

    /// The value of the last rule (in file order) that applies, a rule that
    /// might apply counting as the larger of itself and what lies beneath.
    fn constrained(
        &self,
        kind: Kind,
        a: &Item,
        b: &Item,
        pair: bool,
        base: f64,
        pick: fn(&Constraint) -> Option<f64>,
    ) -> f64 {
        let mut out = base;
        for rule in &self.rules {
            for c in rule.constraints.iter().filter(|c| c.kind == kind) {
                let v = match pick(c) {
                    Some(v) => v,
                    None => continue,
                };
                match self.applies(rule, a, b, pair) {
                    T::Yes => out = v,
                    T::Maybe => out = out.max(v),
                    T::No => {}
                }
            }
        }
        out
    }

    /// Copper-to-copper clearance between two items of different nets.
    pub fn clearance(&self, a: &Item, b: &Item) -> f64 {
        let ca = a
            .net
            .map(|n| self.class_of(n).clearance)
            .unwrap_or(self.classes[0].clearance);
        let cb = b
            .net
            .map(|n| self.class_of(n).clearance)
            .unwrap_or(self.classes[0].clearance);
        let v = self.constrained(Kind::Clearance, a, b, true, ca.max(cb), |c| c.min);
        v.max(self.min_clearance)
    }

    /// The width to route this net at: its class's width, or the `opt` of the
    /// last track_width rule that applies; never under a rule's `min` (a floor,
    /// not a target) and, where a `max` applies, not over it.
    pub fn track_width(&self, net: &str) -> f64 {
        let it = Item::track(net);
        let mut width = self.class_of(net).track_width;
        let (mut floor, mut cap) = (0.0f64, f64::INFINITY);
        for rule in &self.rules {
            for c in rule.constraints.iter().filter(|c| c.kind == Kind::TrackWidth) {
                let how = self.applies(rule, &it, &it, false);
                if how == T::No {
                    continue;
                }
                if let Some(v) = c.opt {
                    width = if how == T::Yes { v } else { width.max(v) };
                }
                if let Some(v) = c.min {
                    floor = floor.max(v);
                }
                if let (Some(v), T::Yes) = (c.max, how) {
                    cap = v;
                }
            }
        }
        width.min(cap).max(floor)
    }

    /// What the rules' tests of place say of a shape: one bit for each
    /// courtyard a rule names, set when the shape is in it. Two items of the
    /// same net and kind with the same answer here are owed the same clearance,
    /// so the answer can stand in for the shape when clearances are remembered.
    pub fn place(&self, shape: &Shape) -> u64 {
        let mut calls: Vec<(&str, &str)> = Vec::new();
        for r in &self.rules {
            if let Some(e) = &r.condition {
                yard_calls(e, &mut calls);
            }
        }
        calls.sort();
        calls.dedup();
        let (x0, y0, x1, y1) = shape.bbox();
        let mut bits = 0u64;
        for (k, (func, target)) in calls.iter().take(64).enumerate() {
            let hit = self
                .courtyards
                .iter()
                .filter(|(r, _)| wildcard(target, r))
                .any(|(_, c)| {
                    if *func == "insideCourtyard" {
                        x0 >= c.0 && y0 >= c.1 && x1 <= c.2 && y1 <= c.3
                    } else {
                        !(x1 < c.0 || x0 > c.2 || y1 < c.1 || y0 > c.3)
                    }
                });
            if hit {
                bits |= 1 << k;
            }
        }
        bits
    }

    /// Whether clearances may be remembered by net, kind and `place()`: not
    /// when the rules name more courtyards than `place()` has bits for.
    pub fn place_is_whole(&self) -> bool {
        let mut calls: Vec<(&str, &str)> = Vec::new();
        for r in &self.rules {
            if let Some(e) = &r.condition {
                yard_calls(e, &mut calls);
            }
        }
        calls.sort();
        calls.dedup();
        calls.len() <= 64
    }

    /// The least width the rules allow this net's tracks (0 when no rule says).
    pub fn track_floor(&self, net: &str) -> f64 {
        let it = Item::track(net);
        let mut floor = 0.0f64;
        for rule in &self.rules {
            for c in rule.constraints.iter().filter(|c| c.kind == Kind::TrackWidth) {
                if let (Some(v), true) = (c.min, self.applies(rule, &it, &it, false) != T::No) {
                    floor = floor.max(v);
                }
            }
        }
        floor
    }

    /// The smallest copper clearance any rule or class asks for: a lower bound
    /// on every pair's clearance, which the router sizes its grid margin by.
    pub fn smallest_clearance(&self) -> f64 {
        let mut v = self.classes.iter().map(|c| c.clearance).fold(f64::INFINITY, f64::min);
        for rule in &self.rules {
            for c in rule.constraints.iter().filter(|c| c.kind == Kind::Clearance) {
                if let Some(m) = c.min {
                    v = v.min(m);
                }
            }
        }
        if v.is_finite() {
            v.max(self.min_clearance).max(0.0)
        } else {
            self.min_clearance.max(0.0)
        }
    }

    /// (diameter, drill) of this net's vias, or None when its rules forbid them.
    pub fn via(&self, net: &str) -> Option<(f64, f64)> {
        let it = Item::via(net);
        for rule in &self.rules {
            if rule.constraints.iter().any(|c| c.kind == Kind::DisallowVia)
                && self.applies(rule, &it, &it, false) != T::No
            {
                return None;
            }
        }
        let c = self.class_of(net);
        Some((c.via_diameter, c.via_drill))
    }

    pub fn edge_clearance(&self, net: &str) -> f64 {
        let it = Item::track(net);
        self.constrained(Kind::EdgeClearance, &it, &it, false, self.edge_clearance, |c| c.min)
            .max(self.edge_clearance)
    }
}

// ------------------------------------------------------------ conditions

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Id(String),
    Str(String),
    Num(String),
    Op(&'static str),
    LParen,
    RParen,
    Comma,
}

fn lex(s: &str) -> Vec<Tok> {
    let c: Vec<char> = s.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < c.len() {
        let ch = c[i];
        if ch.is_whitespace() {
            i += 1;
        } else if ch == '\'' || ch == '"' {
            let q = ch;
            let mut t = String::new();
            i += 1;
            while i < c.len() && c[i] != q {
                t.push(c[i]);
                i += 1;
            }
            i += 1;
            out.push(Tok::Str(t));
        } else if ch == '(' {
            out.push(Tok::LParen);
            i += 1;
        } else if ch == ')' {
            out.push(Tok::RParen);
            i += 1;
        } else if ch == ',' {
            out.push(Tok::Comma);
            i += 1;
        } else {
            let two: String = c[i..(i + 2).min(c.len())].iter().collect();
            let op = match two.as_str() {
                "==" => Some("=="),
                "!=" => Some("!="),
                "&&" => Some("&&"),
                "||" => Some("||"),
                "<=" => Some("<="),
                ">=" => Some(">="),
                _ => None,
            };
            if let Some(o) = op {
                out.push(Tok::Op(o));
                i += 2;
            } else if ch == '!' || ch == '<' || ch == '>' {
                out.push(Tok::Op(match ch {
                    '!' => "!",
                    '<' => "<",
                    _ => ">",
                }));
                i += 1;
            } else if ch.is_ascii_digit() || (ch == '.' && i + 1 < c.len() && c[i + 1].is_ascii_digit()) {
                let mut t = String::new();
                while i < c.len() && (c[i].is_ascii_alphanumeric() || c[i] == '.') {
                    t.push(c[i]);
                    i += 1;
                }
                out.push(Tok::Num(t));
            } else {
                let mut t = String::new();
                while i < c.len() && (c[i].is_alphanumeric() || c[i] == '.' || c[i] == '_') {
                    t.push(c[i]);
                    i += 1;
                }
                if t.is_empty() {
                    i += 1; // a character the grammar does not know
                    out.push(Tok::Id("?".into()));
                } else {
                    out.push(Tok::Id(t));
                }
            }
        }
    }
    out
}

struct Parser {
    t: Vec<Tok>,
    i: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.t.get(self.i)
    }
    fn next(&mut self) -> Option<Tok> {
        let t = self.t.get(self.i).cloned();
        self.i += 1;
        t
    }
    fn or(&mut self) -> Expr {
        let mut e = self.and();
        while self.peek() == Some(&Tok::Op("||")) {
            self.i += 1;
            e = Expr::Or(Box::new(e), Box::new(self.and()));
        }
        e
    }
    fn and(&mut self) -> Expr {
        let mut e = self.unary();
        while self.peek() == Some(&Tok::Op("&&")) {
            self.i += 1;
            e = Expr::And(Box::new(e), Box::new(self.unary()));
        }
        e
    }
    fn unary(&mut self) -> Expr {
        if self.peek() == Some(&Tok::Op("!")) {
            self.i += 1;
            return Expr::Not(Box::new(self.unary()));
        }
        self.cmp()
    }
    fn cmp(&mut self) -> Expr {
        if self.peek() == Some(&Tok::LParen) {
            self.i += 1;
            let e = self.or();
            if self.peek() == Some(&Tok::RParen) {
                self.i += 1;
            }
            return self.maybe_cmp_tail(e);
        }
        let lhs = self.operand();
        match self.peek() {
            Some(Tok::Op(o)) if *o == "==" || *o == "!=" => {
                let eq = *o == "==";
                self.i += 1;
                let rhs = self.operand();
                match (lhs, rhs) {
                    (Ok(l), Ok(r)) => Expr::Cmp(l, eq, r),
                    _ => Expr::Unknown,
                }
            }
            Some(Tok::Op(o)) if ["<", ">", "<=", ">="].contains(o) => {
                self.i += 1;
                let _ = self.operand();
                Expr::Unknown
            }
            _ => match lhs {
                Err(Some(call)) => call,
                _ => Expr::Unknown,
            },
        }
    }
    fn maybe_cmp_tail(&mut self, e: Expr) -> Expr {
        e
    }
    /// Ok(value), or Err(Some(call)) for a function call, Err(None) for nonsense.
    fn operand(&mut self) -> Result<Val, Option<Expr>> {
        match self.next() {
            Some(Tok::Str(s)) => Ok(Val::Str(s)),
            Some(Tok::Num(n)) => Ok(match length(&n) {
                Some(v) => Val::Num(v),
                None => Val::Str(n),
            }),
            Some(Tok::Id(id)) => {
                let (side, name) = match id.split_once('.') {
                    Some((s, n)) if s == "A" || s == "B" => (s.chars().next().unwrap(), n.to_string()),
                    _ => ('?', id.clone()),
                };
                if self.peek() == Some(&Tok::LParen) {
                    self.i += 1;
                    let mut args = Vec::new();
                    while let Some(t) = self.next() {
                        match t {
                            Tok::RParen => break,
                            Tok::Str(s) | Tok::Num(s) | Tok::Id(s) => args.push(s),
                            _ => {}
                        }
                    }
                    if side == '?' {
                        return Err(Some(Expr::Unknown));
                    }
                    return Err(Some(Expr::Call(side, name, args)));
                }
                if side == '?' {
                    return Err(None);
                }
                Ok(Val::Prop(side, name))
            }
            _ => Err(None),
        }
    }
}

fn parse_expr(s: &str) -> Expr {
    let mut p = Parser { t: lex(s), i: 0 };
    let e = p.or();
    if p.i < p.t.len() {
        return Expr::Unknown;
    }
    e
}

/// The courtyard tests in a condition: (function, footprint named).
fn yard_calls<'a>(e: &'a Expr, out: &mut Vec<(&'a str, &'a str)>) {
    match e {
        Expr::Or(a, b) | Expr::And(a, b) => {
            yard_calls(a, out);
            yard_calls(b, out);
        }
        Expr::Not(a) => yard_calls(a, out),
        Expr::Call(_, func, args) if func.ends_with("Courtyard") => {
            out.push((func.as_str(), args.first().map(|s| s.as_str()).unwrap_or("")));
        }
        _ => {}
    }
}

fn literals<'a>(e: &'a Expr, out: &mut Vec<&'a str>) {
    match e {
        Expr::Or(a, b) | Expr::And(a, b) => {
            literals(a, out);
            literals(b, out);
        }
        Expr::Not(a) => literals(a, out),
        Expr::Cmp(l, _, r) => {
            for v in [l, r] {
                if let Val::Str(s) = v {
                    out.push(s.as_str());
                }
            }
        }
        _ => {}
    }
}

fn contains_unknown(e: &Expr) -> bool {
    match e {
        Expr::Unknown => true,
        Expr::Or(a, b) | Expr::And(a, b) => contains_unknown(a) || contains_unknown(b),
        Expr::Not(a) => contains_unknown(a),
        Expr::Call(_, f, _) => !matches!(
            f.as_str(),
            "intersectsCourtyard"
                | "insideCourtyard"
                | "intersectsFrontCourtyard"
                | "intersectsBackCourtyard"
                | "hasNetclass"
                | "hasExactNetclass"
                | "isPlated"
        ),
        Expr::Cmp(l, _, r) => {
            let known = |v: &Val| match v {
                Val::Prop(_, p) => matches!(
                    p.to_ascii_lowercase().as_str(),
                    "netname" | "netclass" | "net_class" | "type"
                ),
                _ => true,
            };
            !known(l) || !known(r)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DRU: &str = r#"(version 1)
(rule "mains to low voltage"
  (constraint clearance (min 8mm))
  (condition "(A.NetName == '/L' || A.NetName == '/N') && !(B.NetName == '/L' || B.NetName == '/N') && B.NetName != ''"))
(rule "between mains nets"
  (constraint clearance (min 2.5mm))
  (condition "(A.NetName == '/L' || A.NetName == '/N') && (B.NetName == '/L' || B.NetName == '/N') && A.NetName != B.NetName && !(A.Type == 'Pad' && B.Type == 'Pad') && !A.intersectsCourtyard('Q4') && !B.intersectsCourtyard('Q4')"))
(rule "mains are wide"
  (constraint track_width (min 2mm))
  (condition "A.NetName == '/L' || A.NetName == '/N'"))
(rule "no vias on mains"
  (constraint disallow via)
  (condition "A.NetName == '/L*'"))
"#;

    fn rules() -> Rules {
        let mut r = Rules::default();
        r.read_dru(DRU).unwrap();
        r.courtyards.insert("Q4".into(), (10.0, 10.0, 20.0, 20.0));
        r
    }

    #[test]
    fn net_pair_clearance() {
        let r = rules();
        assert_eq!(r.clearance(&Item::track("/L"), &Item::track("SDA")), 8.0);
        assert_eq!(
            r.clearance(&Item::track("SDA"), &Item::track("/N")),
            8.0,
            "conditions are tried both ways round"
        );
        assert_eq!(r.clearance(&Item::track("SDA"), &Item::track("SCL")), 0.2);
        // a track's place is unknown, so the Q4 exception might not hold: 2.5 mm
        assert_eq!(r.clearance(&Item::track("/L"), &Item::track("/N")), 2.5);
    }

    #[test]
    fn location_tests_on_fixed_pads() {
        let r = rules();
        let in_q4 = Shape::Circle {
            c: (15.0, 15.0),
            r: 0.8,
        };
        let pad = Item {
            net: Some("/N"),
            kind: Some("Pad"),
            shape: Some(&in_q4),
        };
        // the pad is inside Q4's courtyard, so the 2.5 mm rule does not apply to it
        assert_eq!(r.clearance(&Item::track("/L"), &pad), 0.2);
        let far = Shape::Circle {
            c: (50.0, 50.0),
            r: 0.8,
        };
        let pad = Item {
            net: Some("/N"),
            kind: Some("Pad"),
            shape: Some(&far),
        };
        assert_eq!(r.clearance(&Item::track("/L"), &pad), 2.5);
    }

    #[test]
    fn widths_and_vias() {
        let r = rules();
        assert_eq!(r.track_width("/N"), 2.0);
        assert_eq!(r.track_width("SDA"), 0.25);
        assert_eq!(r.via("/L"), None);
        assert!(r.via("/N").is_some());
    }

    #[test]
    fn net_classes_from_the_project() {
        let mut r = Rules::default();
        r.read_project(
            r#"{"board": {"design_settings": {"rules": {"min_clearance": 0.15, "min_copper_edge_clearance": 0.3}}},
                "net_settings": {"classes": [{"name": "Default", "clearance": 0.2, "track_width": 0.25, "via_diameter": 0.8, "via_drill": 0.4},
                                             {"name": "Power", "clearance": 0.3, "track_width": 1.0}],
                                 "netclass_assignments": {"VBUS": "Power"},
                                 "netclass_patterns": [{"netclass": "Power", "pattern": "+*V*"}]}}"#,
        )
        .unwrap();
        assert_eq!(r.class_of("VBUS").name, "Power");
        assert_eq!(r.class_of("+3V3").name, "Power");
        assert_eq!(r.class_of("SDA").name, "Default");
        assert_eq!(r.track_width("+12V"), 1.0);
        assert_eq!(r.via("+12V"), Some((0.8, 0.4)));
        assert_eq!(r.clearance(&Item::track("+12V"), &Item::track("SDA")), 0.3);
        assert_eq!(r.edge_clearance, 0.3);
    }

    #[test]
    fn a_minimum_width_is_a_floor_not_the_width() {
        let mut r = Rules::default();
        r.read_project(
            r#"{"net_settings": {"classes": [{"name": "Default", "clearance": 0.2, "track_width": 0.25},
                                             {"name": "Power", "clearance": 0.2, "track_width": 1.0}],
                                 "netclass_assignments": {"VBUS": "Power"}}}"#,
        )
        .unwrap();
        // the fab-house style of rule: no condition, a minimum only
        r.read_dru("(version 1)\n(rule \"Minimum Trace Width\" (constraint track_width (min 0.127mm)))")
            .unwrap();
        assert_eq!(r.track_width("VBUS"), 1.0);
        assert_eq!(r.track_width("SDA"), 0.25);
        // a floor above the class width lifts it; an opt sets it; a max caps it
        r.read_dru("(rule \"a\" (constraint track_width (min 0.4mm)) (condition \"A.NetName == 'SDA'\"))")
            .unwrap();
        assert_eq!(r.track_width("SDA"), 0.4);
        r.read_dru("(rule \"b\" (constraint track_width (min 0.2mm) (opt 0.6mm) (max 0.8mm)) (condition \"A.NetName == 'VBUS'\"))")
            .unwrap();
        assert_eq!(r.track_width("VBUS"), 0.6);
        r.read_dru("(rule \"c\" (constraint track_width (max 0.5mm)) (condition \"A.NetName == 'VBUS'\"))")
            .unwrap();
        assert_eq!(r.track_width("VBUS"), 0.5);
        assert_eq!(r.smallest_clearance(), 0.2);
    }

    #[test]
    fn wildcards() {
        assert!(wildcard("/MCU*", "/MCU_TX"));
        assert!(wildcard("?ND", "GND"));
        assert!(!wildcard("/MCU*", "/LED"));
    }
}
