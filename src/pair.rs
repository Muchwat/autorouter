//! The geometry of a differential pair.
//!
//! The router finds one path for the pair, down the middle, treating the two
//! tracks and the gap between them as one wide track. This module turns that
//! centreline into the two real tracks: each a constant half-pitch to its side,
//! mitred at corners, with a via each where the path changes layer and a short
//! piece of its own from each end to its pad.

use crate::geom::{self, P};

/// One track of the pair on one layer: (layer, points).
pub type Line = (usize, Vec<P>);

/// A leg of a pair, between two ends: the two tracks' copper.
#[derive(Clone, Debug, Default)]
pub struct Leg {
    /// The track that starts at the first of the two start points, and its partner.
    pub a: Vec<Line>,
    pub b: Vec<Line>,
    pub vias_a: Vec<P>,
    pub vias_b: Vec<P>,
    /// Where the path changes layer (the middle of each pair of vias).
    pub centres: Vec<P>,
    /// Length of the part where the two run side by side, along the middle.
    pub coupled: f64,
}

fn length(lines: &[Line]) -> f64 {
    lines
        .iter()
        .map(|(_, p)| p.windows(2).map(|s| geom::dist(s[0], s[1])).sum::<f64>())
        .sum()
}

impl Leg {
    pub fn length_a(&self) -> f64 {
        length(&self.a)
    }
    pub fn length_b(&self) -> f64 {
        length(&self.b)
    }
}

fn unit(a: P, b: P) -> P {
    let d = geom::dist(a, b);
    ((b.0 - a.0) / d, (b.1 - a.1) / d)
}

/// The side of a direction: its normal.
fn side(d: P) -> P {
    (-d.1, d.0)
}

fn dot(a: P, b: P) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn push(pts: &mut Vec<P>, p: P) {
    if pts.last().map(|q| geom::dist(*q, p) > 1e-6).unwrap_or(true) {
        pts.push(p);
    }
}

/// The offset direction at a corner between two sides: their mitre, so both
/// tracks keep their distance from the centreline on both sides of the corner.
fn mitre(n1: P, n2: P) -> Result<P, String> {
    let c = dot(n1, n2);
    if c < -0.2 {
        return Err("it turns too sharply for two tracks to stay side by side".into());
    }
    Ok(((n1.0 + n2.0) / (1.0 + c), (n1.1 + n2.1) / (1.0 + c)))
}

/// Build a leg from the centreline.
///
/// `runs`: the centreline per layer, corners only, in order; consecutive runs
/// meet at the cell where the path changes layer. `halves`: half the pitch of
/// the two tracks (centre to centre) on each layer, by layer number. `via_half`: half the pitch of their two
/// vias, at least; `via_side`: how far to its side of the path a via must
/// stand for the other track to pass it (they are set wider apart, by as much
/// as it takes, where the path bends as it changes layer). `first`: +1.0 or -1.0, the side of the path track `a` starts on.
/// `start` and `end`: the points track a and track b come from and go to.
/// `heading`: the way the pair is facing as it leaves its start and as it
/// reaches its end (away from the first pads, toward the last), so the two
/// tracks line up across that line there whatever the path does next.
///
/// At a change of layer the two vias sit either side of the path, square to
/// the bend it makes there, and each track goes straight to its own via from a
/// little way back and straight on from it to a little way ahead, so neither
/// passes the other's via.
pub fn leg(
    runs: &[Line],
    halves: &[f64],
    via_half: f64,
    via_side: f64,
    first: f64,
    start: (P, P),
    end: (P, P),
    heading: (Option<P>, Option<P>),
) -> Result<Leg, String> {
    if runs.is_empty() {
        return Err("no path".into());
    }
    let no_room = || "it changes layer with no room to line the two tracks up".to_string();
    let mut out = Leg::default();
    let mut sigma = first;
    // the vias the next run's tracks start from
    let mut from: Option<(P, P)> = None;
    // how far from a via the tracks are back at their own pitch
    let reach = 2.0 * via_half;
    for (r, (layer, v)) in runs.iter().enumerate() {
        let (first_run, last_run) = (r == 0, r + 1 == runs.len());
        let half = *halves.get(*layer).or(halves.last()).ok_or("no pitch for the pair")?;
        let normals: Vec<P> = v.windows(2).map(|s| side(unit(s[0], s[1]))).collect();
        if normals.is_empty() && !first_run && !last_run {
            return Err(no_room()); // a layer it only passes through
        }
        let mut a: Vec<P> = Vec::new();
        let mut b: Vec<P> = Vec::new();
        match from {
            Some((va, vb)) => {
                a.push(va);
                b.push(vb);
            }
            None => {
                a.push(start.0);
                b.push(start.1);
            }
        }
        let offset = |at: P, m: P, sigma: f64, a: &mut Vec<P>, b: &mut Vec<P>| {
            push(a, (at.0 + sigma * half * m.0, at.1 + sigma * half * m.1));
            push(b, (at.0 - sigma * half * m.0, at.1 - sigma * half * m.1));
        };
        for k in 0..v.len() {
            if normals.is_empty() {
                break; // the path changes layer where it starts: pads straight to vias
            }
            let (at_via_start, at_via_end) = (k == 0 && !first_run, k == v.len() - 1 && !last_run);
            if at_via_start {
                // on from the vias: pick the pitch up a little way along the first side
                let d = geom::dist(v[0], v[1]);
                if d > reach + 1e-9 {
                    let t = reach / d;
                    offset(
                        (v[0].0 + t * (v[1].0 - v[0].0), v[0].1 + t * (v[1].1 - v[0].1)),
                        normals[0],
                        sigma,
                        &mut a,
                        &mut b,
                    );
                }
                continue;
            }
            if at_via_end {
                // in to the vias: leave the pitch a little way back along the last side
                let d = geom::dist(v[k - 1], v[k]);
                if d > reach + 1e-9 {
                    let t = 1.0 - reach / d;
                    offset(
                        (
                            v[k - 1].0 + t * (v[k].0 - v[k - 1].0),
                            v[k - 1].1 + t * (v[k].1 - v[k - 1].1),
                        ),
                        normals[k - 1],
                        sigma,
                        &mut a,
                        &mut b,
                    );
                }
                continue;
            }
            let m = if k == 0 {
                match heading.0 {
                    Some(d) => mitre(side(d), normals[0])?,
                    None => normals[0],
                }
            } else if k == v.len() - 1 {
                match heading.1 {
                    Some(d) => mitre(normals[k - 1], side(d))?,
                    None => normals[k - 1],
                }
            } else {
                mitre(normals[k - 1], normals[k])?
            };
            offset(v[k], m, sigma, &mut a, &mut b);
        }
        out.coupled += v.windows(2).map(|s| geom::dist(s[0], s[1])).sum::<f64>();
        let last = v[v.len() - 1];
        if !last_run {
            // the side of the path as it comes in, and as it goes on (on the next layer)
            let next = &runs[r + 1].1;
            let came = match (normals.last(), heading.0) {
                (Some(n), _) => Some(*n),
                (None, Some(d)) => Some(side(d)),
                (None, None) => None,
            };
            let mut onward = if next.len() >= 2 {
                side(unit(next[0], next[1]))
            } else {
                // it changes layer where it ends: on toward the last pads
                match (heading.1, came) {
                    (Some(d), _) => side(d),
                    (None, Some(n)) => n,
                    (None, None) => return Err(no_room()),
                }
            };
            let came = came.unwrap_or(onward);
            // each track carries on from its own via: if the path doubles back, the
            // track that was on the left of it is now on its right
            let mut flip = false;
            if dot(came, onward) < 0.0 {
                onward = (-onward.0, -onward.1);
                flip = true;
            }
            let across = (came.0 + onward.0, came.1 + onward.1);
            let len = (across.0 * across.0 + across.1 * across.1).sqrt();
            if len < 1e-6 {
                return Err(no_room());
            }
            let across = (across.0 / len, across.1 / len);
            // on a bend the vias stand skew to both sides of it: set them wider apart
            // then, so each is still far enough to its side for the other track to pass
            let wide = via_half.max(via_side / dot(came, across).max(0.5));
            let va = (last.0 + sigma * wide * across.0, last.1 + sigma * wide * across.1);
            let vb = (last.0 - sigma * wide * across.0, last.1 - sigma * wide * across.1);
            push(&mut a, va);
            push(&mut b, vb);
            out.vias_a.push(va);
            out.vias_b.push(vb);
            out.centres.push(last);
            from = Some((va, vb));
            if flip {
                sigma = -sigma;
            }
        } else {
            push(&mut a, end.0);
            push(&mut b, end.1);
        }
        if a.len() >= 2 {
            out.a.push((*layer, a));
        }
        if b.len() >= 2 {
            out.b.push((*layer, b));
        }
    }
    Ok(out)
}

/// The smallest distance between the centre lines of the two tracks on a
/// shared layer (infinite when they share none).
pub fn spacing(leg: &Leg) -> f64 {
    let mut least = f64::INFINITY;
    for (la, pa) in &leg.a {
        for (lb, pb) in &leg.b {
            if la != lb {
                continue;
            }
            for s in pa.windows(2) {
                for t in pb.windows(2) {
                    least = least.min(geom::seg_seg_dist(s[0], s[1], t[0], t[1]));
                }
            }
        }
    }
    least
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_tracks_stay_a_pitch_apart_round_a_corner() {
        // east 10 mm, then a 45 degree turn to the south-east
        let runs = vec![(0usize, vec![(0.0, 0.0), (10.0, 0.0), (15.0, 5.0)])];
        // sides of the start and the end, half a pitch out
        let n_end = side(unit((10.0, 0.0), (15.0, 5.0)));
        let h = 0.2;
        let start = ((0.0, h), (0.0, -h));
        let end = (
            (15.0 + h * n_end.0, 5.0 + h * n_end.1),
            (15.0 - h * n_end.0, 5.0 - h * n_end.1),
        );
        let l = leg(&runs, &[h, h], 0.45, 0.0, 1.0, start, end, (None, None)).unwrap();
        assert_eq!(l.a.len(), 1);
        assert!((spacing(&l) - 2.0 * h).abs() < 1e-9, "spacing {}", spacing(&l));
        assert!((l.coupled - (10.0 + 50f64.sqrt())).abs() < 1e-9);
        // the outer track is longer than the inner one, by the turn
        assert!((l.length_a() - l.length_b()).abs() > 0.1);
        assert!(l.vias_a.is_empty());
    }

    #[test]
    fn a_layer_change_gets_a_via_each() {
        let runs = vec![
            (0usize, vec![(0.0, 0.0), (5.0, 0.0)]),
            (1usize, vec![(5.0, 0.0), (10.0, 0.0)]),
        ];
        let h = 0.2;
        let l = leg(
            &runs,
            &[h, h],
            0.45,
            0.0,
            1.0,
            ((0.0, h), (0.0, -h)),
            ((10.0, h), (10.0, -h)),
            (None, None),
        )
        .unwrap();
        assert_eq!(l.vias_a, vec![(5.0, 0.45)]);
        assert_eq!(l.vias_b, vec![(5.0, -0.45)]);
        assert_eq!(l.centres, vec![(5.0, 0.0)]);
        // each track is on both layers, and they never cross
        assert_eq!((l.a.len(), l.b.len()), (2, 2));
        assert!(spacing(&l) >= 2.0 * h - 1e-9);
        // turning a right angle as it changes layer: the vias sit across the bend, the
        // tracks keep their distance, and neither runs past the other's via
        let turn = vec![
            (0usize, vec![(0.0, 0.0), (5.0, 0.0)]),
            (1usize, vec![(5.0, 0.0), (5.0, 5.0)]),
        ];
        let l = leg(
            &turn,
            &[0.175, 0.175],
            0.4,
            0.375,
            1.0,
            ((0.0, 0.175), (0.0, -0.175)),
            ((4.825, 5.0), (5.175, 5.0)),
            (None, None),
        )
        .unwrap();
        assert!(
            spacing(&l) >= 0.35 - 1e-9,
            "the tracks close in at the via: {}",
            spacing(&l)
        );
        for (via, lines) in [(l.vias_a[0], &l.b), (l.vias_b[0], &l.a)] {
            for (_, pts) in lines.iter() {
                for s in pts.windows(2) {
                    let d = geom::seg_dist(via, s[0], s[1]);
                    assert!(
                        d >= 0.55 - 1e-9,
                        "a track passes {:.3} mm from the other track's via",
                        d
                    );
                }
            }
        }
        // going back the way it came on the other layer: each track keeps its own side of the board
        let back = vec![
            (0usize, vec![(0.0, 0.0), (5.0, 0.0)]),
            (1usize, vec![(5.0, 0.0), (0.0, 0.0)]),
        ];
        let l = leg(
            &back,
            &[h, h],
            0.45,
            0.0,
            1.0,
            ((0.0, h), (0.0, -h)),
            ((0.0, h), (0.0, -h)),
            (None, None),
        )
        .unwrap();
        assert!(spacing(&l) >= 2.0 * h - 1e-9, "the tracks cross: {}", spacing(&l));
    }

    #[test]
    fn the_ends_line_up_across_the_way_the_pair_is_facing() {
        // the pair leaves its pads heading east, but the path's first step is south-east:
        // the tracks start mitred between the two, and a via at the very start works too
        let runs = vec![(0usize, vec![(0.0, 0.0), (5.0, 5.0)])];
        let l = leg(
            &runs,
            &[0.2, 0.2],
            0.45,
            0.0,
            -1.0,
            ((-2.0, -0.4), (-2.0, 0.4)),
            ((7.0, 4.6), (7.0, 5.4)),
            (Some((1.0, 0.0)), Some((1.0, 0.0))),
        )
        .unwrap();
        assert!(spacing(&l) >= 0.4 - 1e-9, "spacing {}", spacing(&l));
        let at_once = vec![(0usize, vec![(0.0, 0.0)]), (1usize, vec![(0.0, 0.0), (5.0, 0.0)])];
        // (the pads are far enough apart for a via each straight in front of them)
        let l = leg(
            &at_once,
            &[0.2, 0.2],
            0.45,
            0.0,
            -1.0,
            ((-2.0, -0.4), (-2.0, 0.4)),
            ((7.0, -0.4), (7.0, 0.4)),
            (Some((1.0, 0.0)), None),
        )
        .unwrap();
        assert_eq!(l.vias_a.len(), 1);
        assert!(spacing(&l) >= 0.4 - 1e-9, "spacing {}", spacing(&l));
    }

    #[test]
    fn a_hairpin_is_refused() {
        let runs = vec![(0usize, vec![(0.0, 0.0), (5.0, 0.0), (0.0, 0.5)])];
        assert!(leg(
            &runs,
            &[0.2, 0.2],
            0.45,
            0.0,
            1.0,
            ((0.0, 0.2), (0.0, -0.2)),
            ((0.0, 0.7), (0.0, 0.3)),
            (None, None)
        )
        .is_err());
    }
}
