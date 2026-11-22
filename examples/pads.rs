//! Print every pad as the router reads it: name, net, centre and bounding box.
//!
//!     cargo run --example pads -- board.kicad_pcb
use autorouter::board::Board;

fn main() {
    let path = std::env::args().nth(1).expect("usage: pads BOARD.kicad_pcb");
    let board = Board::parse(std::fs::read_to_string(&path).expect("cannot read the board")).expect("not a board");
    for p in &board.pads {
        let c = p.shape.centre();
        let (x0, y0, x1, y1) = p.shape.bbox();
        println!(
            "{}\t{}\t{:.4}\t{:.4}\t{:.4}\t{:.4}\t{:.4}\t{:.4}\t{:?}",
            p.name(),
            p.net.as_deref().unwrap_or(""),
            c.0,
            c.1,
            x0,
            y0,
            x1,
            y1,
            p.layers
        );
    }
    for z in &board.zones {
        let xs: Vec<f64> = z.outline.iter().map(|p| p.0).collect();
        let ys: Vec<f64> = z.outline.iter().map(|p| p.1).collect();
        let f = |v: &[f64], m: fn(f64, f64) -> f64| v.iter().cloned().fold(v[0], m);
        eprintln!(
            "zone {:?} keepout={} layers={:?} bbox {:.3} {:.3} {:.3} {:.3}",
            z.net,
            z.keepout.is_some(),
            z.layers,
            f(&xs, f64::min),
            f(&ys, f64::min),
            f(&xs, f64::max),
            f(&ys, f64::max)
        );
    }
    eprintln!(
        "{} pads, {} tracks, {} vias, {} zones, {} holes, {} edge segments",
        board.pads.len(),
        board.tracks.len(),
        board.vias.len(),
        board.zones.len(),
        board.holes.len(),
        board.edges.len()
    );
}
