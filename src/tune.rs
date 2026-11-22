//! Lengthening a track with meanders.
//!
//! A meander here is a square bump: the track leaves its line at a right
//! angle, runs alongside it for a little way, and comes back. A bump of height
//! A makes the track 2 A longer, whatever its width, and since the bump
//! replaces the piece of line under it there is no copper left to short across
//! it. Bumps go on the straight sides of a track, longest side first, each
//! one offered to the caller to check for room before it is kept.

use crate::geom::{self, P};

#[derive(Clone, Copy, Debug)]
pub struct Meander {
    /// How far a bump runs alongside the line, and the least gap between two
    /// bumps and between a bump and the end of its side.
    pub spacing: f64,
    /// The tallest bump.
    pub amplitude: f64,
}

/// One bump: where it leaves the line, its two outer corners, where it rejoins.
pub type Bump = [P; 4];

/// Lengthen a polyline by `extra`, as nearly as bumps that pass `ok` allow.
/// `sides`: which sides of the line a bump may go on (+1.0 its left as it
/// runs, -1.0 its right). Returns the new polyline and how much longer it is.
pub fn lengthen(pts: &[P], extra: f64, m: &Meander, sides: &[f64], ok: &mut dyn FnMut(&Bump) -> bool) -> (Vec<P>, f64) {
    let s = m.spacing;
    let mut left = extra;
    // the bumps chosen for each side of the polyline, by distance along it
    let mut chosen: Vec<Vec<(f64, f64, f64)>> = vec![Vec::new(); pts.len().saturating_sub(1)];
    let mut order: Vec<usize> = (0..pts.len().saturating_sub(1)).collect();
    order.sort_by(|&a, &b| {
        geom::dist(pts[b], pts[b + 1])
            .partial_cmp(&geom::dist(pts[a], pts[a + 1]))
            .unwrap()
            .then(a.cmp(&b))
    });
    for q in order {
        let (p0, p1) = (pts[q], pts[q + 1]);
        let len = geom::dist(p0, p1);
        if len < 3.0 * s {
            continue;
        }
        let d = ((p1.0 - p0.0) / len, (p1.1 - p0.1) / len);
        let n = (-d.1, d.0);
        let mut x = s;
        while x + 2.0 * s <= len + 1e-9 && left > 1e-6 {
            // as tall as is wanted, else half as tall; on whichever side has room
            let mut placed = false;
            'heights: for a in [(left / 2.0).min(m.amplitude), (left / 4.0).min(m.amplitude / 2.0)] {
                if a < 1e-6 {
                    continue;
                }
                for &side in sides {
                    let b0 = (p0.0 + d.0 * x, p0.1 + d.1 * x);
                    let b3 = (b0.0 + d.0 * s, b0.1 + d.1 * s);
                    let up = (side * a * n.0, side * a * n.1);
                    let bump: Bump = [b0, (b0.0 + up.0, b0.1 + up.1), (b3.0 + up.0, b3.1 + up.1), b3];
                    if ok(&bump) {
                        chosen[q].push((x, a, side));
                        left -= 2.0 * a;
                        placed = true;
                        break 'heights;
                    }
                }
            }
            x += if placed { 2.0 * s } else { s / 2.0 };
        }
        if left <= 1e-6 {
            break;
        }
    }
    let mut out: Vec<P> = Vec::with_capacity(pts.len() + 4 * chosen.iter().map(|c| c.len()).sum::<usize>());
    for q in 0..pts.len().saturating_sub(1) {
        let (p0, p1) = (pts[q], pts[q + 1]);
        out.push(p0);
        let len = geom::dist(p0, p1);
        if len < 1e-9 {
            continue;
        }
        let d = ((p1.0 - p0.0) / len, (p1.1 - p0.1) / len);
        let n = (-d.1, d.0);
        let mut bumps = chosen[q].clone();
        bumps.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        for (x, a, side) in bumps {
            let b0 = (p0.0 + d.0 * x, p0.1 + d.1 * x);
            let b3 = (b0.0 + d.0 * s, b0.1 + d.1 * s);
            let up = (side * a * n.0, side * a * n.1);
            out.extend([b0, (b0.0 + up.0, b0.1 + up.1), (b3.0 + up.0, b3.1 + up.1), b3]);
        }
    }
    if let Some(&last) = pts.last() {
        out.push(last);
    }
    (out, extra - left)
}

/// The length of a polyline.
pub fn length(pts: &[P]) -> f64 {
    pts.windows(2).map(|s| geom::dist(s[0], s[1])).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    const M: Meander = Meander {
        spacing: 0.6,
        amplitude: 1.0,
    };

    #[test]
    fn a_line_grows_by_exactly_what_is_asked() {
        let line = vec![(0.0, 0.0), (20.0, 0.0)];
        for extra in [0.3, 2.0, 7.3] {
            let (out, added) = lengthen(&line, extra, &M, &[1.0, -1.0], &mut |_| true);
            assert!((added - extra).abs() < 1e-9);
            assert!(
                (length(&out) - (20.0 + extra)).abs() < 1e-9,
                "{} for {}",
                length(&out),
                extra
            );
            assert_eq!((out[0], out[out.len() - 1]), ((0.0, 0.0), (20.0, 0.0)));
        }
    }

    #[test]
    fn bumps_keep_their_distance_and_their_height() {
        let line = vec![(0.0, 0.0), (20.0, 0.0)];
        let (out, _) = lengthen(&line, 7.3, &M, &[1.0], &mut |_| true);
        // every corner within the tallest bump of the line, on the one side allowed
        assert!(out.iter().all(|p| p.1 >= -1e-9 && p.1 <= 1.0 + 1e-9), "{:?}", out);
        // up-legs and down-legs: none nearer than the spacing to the next
        let legs: Vec<f64> = out
            .windows(2)
            .filter(|s| (s[0].0 - s[1].0).abs() < 1e-9)
            .map(|s| s[0].0)
            .collect();
        assert!(legs.len() >= 8);
        assert!(legs.windows(2).all(|w| w[1] - w[0] >= 0.6 - 1e-9), "{:?}", legs);
        // and the first and last stand off the ends
        assert!(legs[0] >= 0.6 - 1e-9 && legs[legs.len() - 1] <= 20.0 - 0.6 + 1e-9);
    }

    #[test]
    fn no_room_means_no_bump() {
        let line = vec![(0.0, 0.0), (20.0, 0.0)];
        // room only above the line, and only past x = 10
        let (out, added) = lengthen(&line, 3.0, &M, &[1.0, -1.0], &mut |b| {
            b.iter().all(|p| p.1 >= -1e-9 && p.0 >= 10.0)
        });
        assert!((added - 3.0).abs() < 1e-9);
        assert!(out.iter().all(|p| p.1 >= -1e-9));
        assert!(out.iter().filter(|p| p.1 > 1e-9).all(|p| p.0 >= 10.0));
        let (out, added) = lengthen(&line, 3.0, &M, &[1.0, -1.0], &mut |_| false);
        assert_eq!((out, added), (line.clone(), 0.0));
        // a side too short to stand a bump on is left alone
        let (_, added) = lengthen(&[(0.0, 0.0), (1.0, 0.0)], 3.0, &M, &[1.0], &mut |_| true);
        assert_eq!(added, 0.0);
    }

    #[test]
    fn the_longest_side_is_used_first() {
        let bent = vec![(0.0, 0.0), (3.0, 0.0), (3.0, 20.0)];
        let (out, added) = lengthen(&bent, 1.0, &M, &[1.0, -1.0], &mut |_| true);
        assert!((added - 1.0).abs() < 1e-9);
        // the short first side is untouched
        assert_eq!(&out[..2], &bent[..2]);
    }
}
