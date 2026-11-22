//! Writing the result back: new items go in before the file's last
//! parenthesis, and removed items are cut out of the original text, so the
//! rest of the file stays byte for byte as KiCad wrote it.

use crate::board::{Board, NetStyle};
use crate::router::Routed;
use crate::sexpr::quote;
use std::time::{SystemTime, UNIX_EPOCH};

struct Ids(u64);

impl Ids {
    fn new() -> Ids {
        let t = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x9e37_79b9_7f4a_7c15);
        Ids(t ^ 0x2545_f491_4f6c_dd1d ^ (std::process::id() as u64) << 32)
    }

    fn next(&mut self) -> u64 {
        // xorshift64*
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    /// A random (version 4) UUID.
    fn uuid(&mut self) -> String {
        let (a, b) = (self.next(), self.next());
        let a = (a & 0xffff_ffff_ffff_0fff) | 0x0000_0000_0000_4000;
        let b = (b & 0x3fff_ffff_ffff_ffff) | 0x8000_0000_0000_0000;
        format!(
            "{:08x}-{:04x}-{:04x}-{:04x}-{:012x}",
            a >> 32,
            (a >> 16) & 0xffff,
            a & 0xffff,
            b >> 48,
            b & 0xffff_ffff_ffff
        )
    }
}

fn num(v: f64) -> String {
    let s = format!("{:.6}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    if s == "-0" {
        "0".into()
    } else {
        s
    }
}

fn net_token(board: &Board, net: &str) -> String {
    match board.style {
        NetStyle::Name => format!("(net {})", quote(net)),
        NetStyle::Code => format!("(net {})", board.net_codes.get(net).cloned().unwrap_or(0)),
    }
}

/// The board's text with `remove` (byte ranges of items) cut out and the routes added.
pub fn with_routes(board: &Board, routed: &Routed, remove: &[(usize, usize)]) -> String {
    let text = strip(&board.text, remove);
    let end = text.rfind(')').unwrap_or(text.len());
    let mut ids = Ids::new();
    let mut add = String::new();
    let uuid_key = if board.version >= 20231231 { "uuid" } else { "tstamp" };
    let id = |ids: &mut Ids| {
        if uuid_key == "uuid" {
            format!("(uuid {})", quote(&ids.uuid()))
        } else {
            format!("(tstamp {})", ids.uuid())
        }
    };
    for t in &routed.tracks {
        add.push_str(&format!(
            "\t(segment\n\t\t(start {} {})\n\t\t(end {} {})\n\t\t(width {})\n\t\t(layer {})\n\t\t{}\n\t\t{}\n\t)\n",
            num(t.a.0),
            num(t.a.1),
            num(t.b.0),
            num(t.b.1),
            num(t.width),
            quote(&board.copper[t.layer]),
            net_token(board, &t.net),
            id(&mut ids)
        ));
    }
    let (first, last) = (board.copper[0].clone(), board.copper[board.copper.len() - 1].clone());
    for v in &routed.vias {
        add.push_str(&format!(
            "\t(via\n\t\t(at {} {})\n\t\t(size {})\n\t\t(drill {})\n\t\t(layers {} {})\n\t\t{}\n\t\t{}\n\t)\n",
            num(v.at.0),
            num(v.at.1),
            num(v.diameter),
            num(v.drill),
            quote(&first),
            quote(&last),
            net_token(board, &v.net),
            id(&mut ids)
        ));
    }
    let mut out = String::with_capacity(text.len() + add.len());
    out.push_str(text[..end].trim_end());
    out.push('\n');
    out.push_str(&add);
    out.push_str(&text[end..]);
    out
}

/// The text with the given byte ranges cut out (with the line each one sat on).
pub fn strip(text: &str, remove: &[(usize, usize)]) -> String {
    let mut spans: Vec<(usize, usize)> = remove.to_vec();
    spans.sort();
    spans.dedup();
    let b = text.as_bytes();
    let mut out = String::with_capacity(text.len());
    let mut pos = 0;
    for (s, e) in spans {
        if s < pos {
            continue;
        }
        // take the indentation before the item and the newline after it as well
        let mut s2 = s;
        while s2 > pos && (b[s2 - 1] == b'\t' || b[s2 - 1] == b' ') {
            s2 -= 1;
        }
        let mut e2 = e;
        if e2 < b.len() && b[e2] == b'\n' {
            e2 += 1;
        }
        out.push_str(&text[pos..s2]);
        pos = e2;
    }
    out.push_str(&text[pos..]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::router::{NewTrack, NewVia, Routed};

    #[test]
    fn adds_items_in_the_files_style() {
        let text = "(kicad_pcb\n\t(version 20260206)\n\t(layers (0 \"F.Cu\" signal) (2 \"B.Cu\" signal))\n\t(segment (start 0 0) (end 1 0) (width 0.2) (layer \"F.Cu\") (net \"A\"))\n)\n";
        let b = Board::parse(text.to_string()).unwrap();
        let mut r = Routed::default();
        r.tracks.push(NewTrack {
            net: "A".into(),
            layer: 1,
            a: (1.0, 2.5),
            b: (3.0, 2.5),
            width: 0.25,
        });
        r.vias.push(NewVia {
            net: "A".into(),
            at: (1.0, 2.5),
            diameter: 0.6,
            drill: 0.3,
        });
        let spans: Vec<(usize, usize)> = b.tracks.iter().map(|t| t.span).collect();
        let out = with_routes(&b, &r, &spans);
        let back = Board::parse(out.clone()).unwrap();
        assert_eq!(back.tracks.len(), 1, "{}", out);
        assert_eq!(back.tracks[0].layer, 1);
        assert_eq!(back.tracks[0].net.as_deref(), Some("A"));
        assert_eq!(back.vias.len(), 1);
        assert!(out.contains("(net \"A\")"));
        assert!(out.ends_with(")\n"));
    }

    #[test]
    fn numbers_and_ids() {
        assert_eq!(num(1.250000), "1.25");
        assert_eq!(num(-0.0000001), "0");
        assert_eq!(num(100.0), "100");
        let mut ids = Ids::new();
        let u = ids.uuid();
        assert_eq!(u.len(), 36);
        assert_eq!(&u[14..15], "4");
        assert_ne!(u, ids.uuid());
    }
}
