//! Shapes, in board millimetres with y pointing down (KiCad's convention).

pub type P = (f64, f64);

/// KiCad's rotation of a point by `deg` (counter-clockwise on screen, y down).
pub fn rot(p: P, deg: f64) -> P {
    if deg == 0.0 {
        return p;
    }
    let (s, c) = deg.to_radians().sin_cos();
    (p.0 * c + p.1 * s, -p.0 * s + p.1 * c)
}

pub fn add(a: P, b: P) -> P {
    (a.0 + b.0, a.1 + b.1)
}

pub fn dist(a: P, b: P) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

/// Distance from p to the segment a-b.
pub fn seg_dist(p: P, a: P, b: P) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l = dx * dx + dy * dy;
    let t = if l == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l).clamp(0.0, 1.0)
    };
    let (qx, qy) = (a.0 + t * dx - p.0, a.1 + t * dy - p.1);
    (qx * qx + qy * qy).sqrt()
}

fn cross(o: P, a: P, b: P) -> f64 {
    (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
}

/// Shortest distance between segments a-b and c-d (0 when they cross).
pub fn seg_seg_dist(a: P, b: P, c: P, d: P) -> f64 {
    let (d1, d2, d3, d4) = (cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)) {
        return 0.0;
    }
    seg_dist(a, c, d)
        .min(seg_dist(b, c, d))
        .min(seg_dist(c, a, b))
        .min(seg_dist(d, a, b))
}

/// Copper (or an obstacle) as one of three primitives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Circle {
        c: P,
        r: f64,
    },
    /// A segment with round ends: tracks, oval pads. `r` is half the width.
    Capsule {
        a: P,
        b: P,
        r: f64,
    },
    /// A rectangle of half-sizes hw x hh turned by `deg`, corners rounded by r
    /// (r = 0: square corners). Rect, roundrect and the bounding box of odd shapes.
    Box {
        c: P,
        hw: f64,
        hh: f64,
        deg: f64,
        r: f64,
    },
}

impl Shape {
    /// Distance from p to the shape (0 inside).
    pub fn dist(&self, p: P) -> f64 {
        match *self {
            Shape::Circle { c, r } => (dist(p, c) - r).max(0.0),
            Shape::Capsule { a, b, r } => (seg_dist(p, a, b) - r).max(0.0),
            Shape::Box { c, hw, hh, deg, r } => {
                let l = rot((p.0 - c.0, p.1 - c.1), -deg);
                let qx = (l.0.abs() - (hw - r)).max(0.0);
                let qy = (l.1.abs() - (hh - r)).max(0.0);
                ((qx * qx + qy * qy).sqrt() - r).max(0.0)
            }
        }
    }

    /// (x0, y0, x1, y1)
    pub fn bbox(&self) -> (f64, f64, f64, f64) {
        match *self {
            Shape::Circle { c, r } => (c.0 - r, c.1 - r, c.0 + r, c.1 + r),
            Shape::Capsule { a, b, r } => (a.0.min(b.0) - r, a.1.min(b.1) - r, a.0.max(b.0) + r, a.1.max(b.1) + r),
            Shape::Box { c, hw, hh, deg, .. } => {
                let (s, co) = deg.to_radians().sin_cos();
                let ex = hw * co.abs() + hh * s.abs();
                let ey = hw * s.abs() + hh * co.abs();
                (c.0 - ex, c.1 - ey, c.0 + ex, c.1 + ey)
            }
        }
    }

    pub fn centre(&self) -> P {
        match *self {
            Shape::Circle { c, .. } | Shape::Box { c, .. } => c,
            Shape::Capsule { a, b, .. } => ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0),
        }
    }

    /// A conservative test of whether two shapes touch or overlap.
    pub fn touches(&self, o: &Shape) -> bool {
        match (*self, *o) {
            (Shape::Capsule { a, b, r }, Shape::Capsule { a: c, b: d, r: r2 }) => {
                seg_seg_dist(a, b, c, d) <= r + r2 + 1e-6
            }
            (Shape::Circle { c, r }, other) | (other, Shape::Circle { c, r }) => other.dist(c) <= r + 1e-6,
            (Shape::Capsule { a, b, r }, bx) | (bx, Shape::Capsule { a, b, r }) => {
                // along the segment, finely enough for pads and tracks
                let n = ((dist(a, b) / 0.05).ceil() as usize).clamp(1, 2000);
                (0..=n).any(|k| {
                    let t = k as f64 / n as f64;
                    bx.dist((a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1))) <= r + 1e-6
                })
            }
            (a @ Shape::Box { .. }, b @ Shape::Box { .. }) => {
                let (ax0, ay0, ax1, ay1) = a.bbox();
                let (bx0, by0, bx1, by1) = b.bbox();
                if ax1 < bx0 || bx1 < ax0 || ay1 < by0 || by1 < ay0 {
                    return false;
                }
                a.dist(b.centre()) <= 1e-6 || b.dist(a.centre()) <= 1e-6 || {
                    let (x0, y0, x1, y1) = (ax0.max(bx0), ay0.max(by0), ax1.min(bx1), ay1.min(by1));
                    let n = 12;
                    (0..=n).any(|i| {
                        (0..=n).any(|j| {
                            let p = (
                                x0 + (x1 - x0) * i as f64 / n as f64,
                                y0 + (y1 - y0) * j as f64 / n as f64,
                            );
                            a.dist(p) <= 1e-6 && b.dist(p) <= 1e-6
                        })
                    })
                }
            }
        }
    }
}

/// Points along a three-point arc (start, mid, end), at most `step` mm apart.
pub fn arc_points(s: P, m: P, e: P, step: f64) -> Vec<P> {
    let d = 2.0 * (s.0 * (m.1 - e.1) + m.0 * (e.1 - s.1) + e.0 * (s.1 - m.1));
    if d.abs() < 1e-12 {
        return vec![s, e];
    }
    let sq = |p: P| p.0 * p.0 + p.1 * p.1;
    let cx = (sq(s) * (m.1 - e.1) + sq(m) * (e.1 - s.1) + sq(e) * (s.1 - m.1)) / d;
    let cy = (sq(s) * (e.0 - m.0) + sq(m) * (s.0 - e.0) + sq(e) * (m.0 - s.0)) / d;
    let r = dist(s, (cx, cy));
    let ang = |p: P| (p.1 - cy).atan2(p.0 - cx);
    let (a0, am, a1) = (ang(s), ang(m), ang(e));
    let norm = |x: f64| {
        let t = std::f64::consts::TAU;
        ((x % t) + t) % t
    };
    // sweep from a0 through am to a1, in whichever direction passes am
    let ccw = norm(am - a0) < norm(a1 - a0);
    let sweep = if ccw { norm(a1 - a0) } else { -norm(a0 - a1) };
    let n = ((r * sweep.abs() / step).ceil() as usize).max(2);
    (0..=n)
        .map(|k| {
            let a = a0 + sweep * k as f64 / n as f64;
            (cx + r * a.cos(), cy + r * a.sin())
        })
        .collect()
}

/// Points along a cubic Bezier curve, about `step` mm apart.
pub fn bezier_points(p0: P, p1: P, p2: P, p3: P, step: f64) -> Vec<P> {
    let n = (((dist(p0, p1) + dist(p1, p2) + dist(p2, p3)) / step).ceil() as usize).clamp(2, 2000);
    (0..=n)
        .map(|k| {
            if k == n {
                return p3; // exactly, so the outline closes
            }
            let t = k as f64 / n as f64;
            let u = 1.0 - t;
            let (a, b, c, d) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
            (
                a * p0.0 + b * p1.0 + c * p2.0 + d * p3.0,
                a * p0.1 + b * p1.1 + c * p2.1 + d * p3.1,
            )
        })
        .collect()
}

/// The smallest distance from the segment a-b to a shape (0 when they touch),
/// to within a few micrometres.
pub fn seg_shape_dist(a: P, b: P, shape: &Shape) -> f64 {
    match *shape {
        Shape::Circle { c, r } => (seg_dist(c, a, b) - r).max(0.0),
        Shape::Capsule { a: c, b: d, r } => (seg_seg_dist(a, b, c, d) - r).max(0.0),
        Shape::Box { .. } => {
            let n = ((dist(a, b) / 0.01).ceil() as usize).clamp(1, 4000);
            (0..=n)
                .map(|k| {
                    let t = k as f64 / n as f64;
                    shape.dist((a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1)))
                })
                .fold(f64::INFINITY, f64::min)
        }
    }
}

/// Even-odd point-in-polygon.
pub fn in_polygon(p: P, poly: &[P]) -> bool {
    let mut inside = false;
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Distance from p to a polygon's outline.
pub fn polygon_edge_dist(p: P, poly: &[P]) -> f64 {
    let n = poly.len();
    (0..n)
        .map(|i| seg_dist(p, poly[i], poly[(i + 1) % n]))
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_distance_and_rotation() {
        let b = Shape::Box {
            c: (0.0, 0.0),
            hw: 2.0,
            hh: 1.0,
            deg: 0.0,
            r: 0.0,
        };
        assert!(b.dist((1.9, 0.9)) == 0.0);
        assert!((b.dist((3.0, 0.0)) - 1.0).abs() < 1e-9);
        let b90 = Shape::Box {
            c: (0.0, 0.0),
            hw: 2.0,
            hh: 1.0,
            deg: 90.0,
            r: 0.0,
        };
        assert!((b90.dist((0.0, 3.0)) - 1.0).abs() < 1e-9);
        assert!(b90.dist((0.0, 1.9)) == 0.0);
        let (x0, y0, x1, y1) = b90.bbox();
        assert!(
            (x0 + 1.0).abs() < 1e-9 && (y0 + 2.0).abs() < 1e-9 && (x1 - 1.0).abs() < 1e-9 && (y1 - 2.0).abs() < 1e-9
        );
    }

    #[test]
    fn rotation_matches_kicad() {
        // KiCad: a pad at local (1, 0) in a footprint turned 90 degrees sits at (0, -1)
        let p = rot((1.0, 0.0), 90.0);
        assert!((p.0).abs() < 1e-9 && (p.1 + 1.0).abs() < 1e-9);
    }

    #[test]
    fn arc_through_three_points() {
        let pts = arc_points((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), 0.1);
        assert!(pts.iter().all(|p| (dist(*p, (0.0, 0.0)) - 1.0).abs() < 1e-9));
        assert!(pts.iter().any(|p| p.1 > 0.9));
    }

    #[test]
    fn polygons() {
        let sq = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)];
        assert!(in_polygon((1.0, 1.0), &sq));
        assert!(!in_polygon((3.0, 1.0), &sq));
        assert!((polygon_edge_dist((1.0, 1.0), &sq) - 1.0).abs() < 1e-9);
    }
}
