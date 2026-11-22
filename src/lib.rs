//! A grid autorouter for KiCad boards.
//!
//! Every net is routed at once and nets bargain for space (negotiated
//! congestion, the PathFinder scheme), so the order nets come in does not
//! decide whether a board finishes. Clearance comes from KiCad's own rules:
//! net classes in the `.kicad_pro` and clearance rules in the `.kicad_dru`,
//! net pair by net pair.
//!
//! ```no_run
//! use autorouter::{board::Board, rules::Rules, router::{route, Options}};
//! let board = Board::parse(std::fs::read_to_string("my.kicad_pcb").unwrap()).unwrap();
//! let rules = Rules::for_board("my.kicad_pcb", &board).unwrap();
//! let result = route(&board, &rules, &Options::default());
//! let text = autorouter::write::with_routes(&board, &result, &[]);
//! std::fs::write("my-routed.kicad_pcb", text).unwrap();
//! ```

pub mod board;
pub mod geom;
pub mod json;
pub mod pair;
pub mod pour;
pub mod router;
pub mod rules;
pub mod sexpr;
pub mod write;
