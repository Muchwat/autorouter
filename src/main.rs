use autorouter::board::Board;
use autorouter::router::{self, Options};
use autorouter::rules::Rules;
use autorouter::write;
use autorouter::{json, pour};
use std::path::{Path, PathBuf};
use std::process::{exit, Command};
use std::time::Instant;

const USAGE: &str = "\
autorouter: route a KiCad board, keeping to its own design rules

usage: autorouter BOARD.kicad_pcb [options]

  -o, --output FILE      where to write the routed board (default BOARD-routed.kicad_pcb)
      --nets A,B         route only these nets (wildcards: '/SPI_*')
      --skip A,B         never route these nets
      --include-poured   also route nets that have a copper pour (left to the pour by default)
      --rip              remove the existing tracks and vias of the nets being routed first
                         (locked ones stay)
      --layers F.Cu,B.Cu copper layers to use (default: all)
      --grid MM          grid pitch (default 0.25; 0.254 suits 0.1 inch through-hole parts)
      --margin MM        least extra gap on every clearance (default 0.05; a coarse grid
                         raises it by what tracks between grid points need)
      --width MM         one track width for every net (default: from the rules)
      --via D/DRILL      one via size for every net, e.g. 0.6/0.3 (default: from the rules)
      --tracks-on-planes also lay tracks on inner layers that a pour covers (by default
                         such a layer is kept as a whole plane)
      --no-vias          route without vias
      --via-spans A-B,.. blind and buried vias the board house can make, each as the two
                         layers it joins: F.Cu-In1.Cu,In2.Cu-B.Cu (default: none; vias
                         go right through the board)
      --blind-vias       any blind or buried via: every pair of layers
      --via-in-pad       let vias sit on surface-mount pads of their own net
      --neck MM          width a track narrows to where it is too wide to reach a pad
                         (default: the Default class's width, or the rules' minimum if more)
      --neck-length MM   longest narrowed piece (default 3; 0: never narrow)
      --no-diff-pairs    route nets named as pairs (D+/D-, _P/_N) one by one, not as pairs
      --diff-pairs A,B   only these nets as pairs (wildcards; either half's name)
      --pair W/GAP       width of a pair's tracks and the gap between them, e.g. 0.2/0.15
                         (default: the net class's pair settings, else its track width
                         and the clearance between the two nets)
      --skew MM          lengthen the shorter half of each pair until the two are within
                         this of each other (default: a skew rule of the board, if any)
      --match-length A,B bring these nets (wildcards) to the length of the longest of
                         them, with meanders; may be given more than once, a group each
      --meander MM       the tallest bump of a meander (default 1)
      --double-vias MM   tracks this wide or wider get a second via beside each via
                         (default 0.8; 0: never)
      --report FILE      write what was done as JSON (for scripts and benchmarks)
      --via-cost N       what a via costs, in grid steps (default 12)
      --passes N         most negotiation passes (default 40)
      --drc              refill the zones and check the result with KiCad's DRC (kicad-cli),
                         custom rules included; the refilled board is what gets written
      --keep-fills       leave the zones' old fills in the output (they are removed by
                         default, being stale once tracks change; --drc pours them again)
  -q, --quiet            only the summary
  -h, --help             this text

Clearances, widths and via sizes come from the net classes in BOARD.kicad_pro and the
custom rules in BOARD.kicad_dru. Exit status: 0 when every net is joined (and, with --drc,
KiCad finds no errors), 1 when something is left, 2 on bad input.";

fn die(msg: &str) -> ! {
    eprintln!("autorouter: {}", msg);
    exit(2)
}

fn list(v: &str) -> Vec<String> {
    v.split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect()
}

fn mm(v: &str, what: &str) -> f64 {
    v.parse::<f64>()
        .ok()
        .filter(|x| *x > 0.0)
        .unwrap_or_else(|| die(&format!("{} needs a number of millimetres, not '{}'", what, v)))
}

fn kicad_cli() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("KICAD_CLI") {
        return Some(PathBuf::from(p));
    }
    let candidates = [
        "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
        "C:\\Program Files\\KiCad\\10.0\\bin\\kicad-cli.exe",
        "C:\\Program Files\\KiCad\\9.0\\bin\\kicad-cli.exe",
        "C:\\Program Files\\KiCad\\8.0\\bin\\kicad-cli.exe",
    ];
    for c in candidates {
        if Path::new(c).exists() {
            return Some(PathBuf::from(c));
        }
    }
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|d| d.join("kicad-cli"))
        .find(|p| p.exists())
}

/// "** Found N DRC violations **" -> N
fn found(report: &str, what: &str) -> Option<usize> {
    report
        .lines()
        .find(|l| l.starts_with("** Found") && l.contains(what))
        .and_then(|l| l.split_whitespace().nth(2))
        .and_then(|n| n.parse().ok())
}

struct Drc {
    errors: usize,
    unconnected: usize,
    /// The board as KiCad saved it after pouring the zones again, when it could.
    refilled: Option<String>,
}

/// KiCad's DRC on `pcb`, zones poured again first, with the rules from beside `rules_from`.
fn drc(pcb: &Path, rules_from: &Path) -> Result<Drc, String> {
    let cli = kicad_cli().ok_or("kicad-cli not found (set KICAD_CLI to its path)")?;
    // in a folder of its own, named like the original so its .kicad_pro and .kicad_dru apply
    let dir = std::env::temp_dir().join(format!("autorouter-drc-{}", std::process::id()));
    let result = drc_in(&cli, &dir, pcb, rules_from);
    let _ = std::fs::remove_dir_all(&dir);
    result
}

fn drc_in(cli: &Path, dir: &Path, pcb: &Path, rules_from: &Path) -> Result<Drc, String> {
    std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    let stem = rules_from.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
    let board = dir.join(format!("{}.kicad_pcb", stem));
    for ext in ["kicad_pro", "kicad_dru"] {
        let src = rules_from.with_extension(ext);
        if src.exists() {
            std::fs::copy(&src, dir.join(format!("{}.{}", stem, ext))).map_err(|e| e.to_string())?;
        }
    }
    let report = dir.join("drc.rpt");
    // newest first: pour and save; then pour only; then as the board is (older kicad-cli)
    let tries: [&[&str]; 3] = [&["--refill-zones", "--save-board"], &["--refill-zones"], &[]];
    let mut last = String::new();
    for extra in tries {
        std::fs::copy(pcb, &board).map_err(|e| e.to_string())?;
        let _ = std::fs::remove_file(&report);
        let out = Command::new(cli)
            .args(["pcb", "drc", "--severity-error", "--units", "mm"])
            .args(extra)
            .arg("-o")
            .arg(&report)
            .arg(&board)
            .output()
            .map_err(|e| format!("could not run {}: {}", cli.display(), e))?;
        let text = match std::fs::read_to_string(&report) {
            Ok(t) => t,
            Err(_) => {
                last = String::from_utf8_lossy(&out.stderr).trim().to_string();
                continue;
            }
        };
        // no count line is no answer: never read it as "none found"
        let (errors, unconnected) = match (found(&text, "DRC violations"), found(&text, "unconnected")) {
            (Some(e), Some(u)) => (e, u),
            _ => return Err("could not read the counts in kicad-cli's report".into()),
        };
        let refilled = if extra.contains(&"--save-board") {
            std::fs::read_to_string(&board)
                .ok()
                .filter(|t| t.starts_with("(kicad_pcb"))
        } else {
            None
        };
        return Ok(Drc {
            errors,
            unconnected,
            refilled,
        });
    }
    Err(format!("kicad-cli wrote no report: {}", last))
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || args.iter().any(|a| a == "-h" || a == "--help") {
        println!("{}", USAGE);
        exit(if args.is_empty() { 2 } else { 0 });
    }
    let mut opt = Options {
        progress: true,
        ..Options::default()
    };
    let (mut input, mut output, mut rip, mut check, mut quiet, mut keep_fills) =
        (None::<PathBuf>, None::<PathBuf>, false, false, false, false);
    let mut report: Option<PathBuf> = None;
    let mut layer_names: Option<Vec<String>> = None;
    let mut span_names: Vec<String> = Vec::new();
    let mut any_span = false;
    let mut i = 0;
    let value = |i: &mut usize, name: &str| -> String {
        *i += 1;
        args.get(*i)
            .cloned()
            .unwrap_or_else(|| die(&format!("{} needs a value", name)))
    };
    while i < args.len() {
        let a = args[i].clone();
        match a.as_str() {
            "-o" | "--output" => output = Some(PathBuf::from(value(&mut i, &a))),
            "--nets" => opt.nets = Some(list(&value(&mut i, &a))),
            "--skip" => opt.skip = list(&value(&mut i, &a)),
            "--include-poured" | "--include-zone-nets" => opt.include_poured = true,
            "--rip" => rip = true,
            "--layers" => layer_names = Some(list(&value(&mut i, &a))),
            "--via-spans" => span_names.extend(list(&value(&mut i, &a))),
            "--blind-vias" => any_span = true,
            "--grid" => opt.grid = mm(&value(&mut i, &a), &a),
            "--margin" => {
                opt.margin = value(&mut i, &a)
                    .parse::<f64>()
                    .ok()
                    .filter(|m| *m >= 0.0)
                    .unwrap_or_else(|| die("--margin needs a number of millimetres, 0 or more"))
            }
            "--width" | "--track-width" => opt.track_width = Some(mm(&value(&mut i, &a), &a)),
            "--via" => {
                let v = value(&mut i, &a);
                let (d, h) = v
                    .split_once('/')
                    .unwrap_or_else(|| die("--via takes DIAMETER/DRILL, e.g. 0.6/0.3"));
                opt.via = Some((mm(d, "--via"), mm(h, "--via")));
            }
            "--no-vias" => opt.no_vias = true,
            "--tracks-on-planes" => opt.tracks_on_planes = true,
            "--via-in-pad" => opt.via_in_pad = true,
            "--no-diff-pairs" => opt.diff_pairs = false,
            "--skew" => {
                opt.skew = Some(
                    value(&mut i, &a)
                        .parse::<f64>()
                        .ok()
                        .filter(|m| *m >= 0.0)
                        .unwrap_or_else(|| die("--skew needs a number of millimetres, 0 or more")),
                )
            }
            "--match-length" => opt.match_groups.push(list(&value(&mut i, &a))),
            "--meander" => opt.meander = mm(&value(&mut i, &a), &a),
            "--diff-pairs" => opt.only_pairs = Some(list(&value(&mut i, &a))),
            "--pair" => {
                let v = value(&mut i, &a);
                let (w, gap) = v
                    .split_once('/')
                    .unwrap_or_else(|| die("--pair takes WIDTH/GAP, e.g. 0.2/0.15"));
                opt.pair_width = Some(mm(w, "--pair"));
                opt.pair_gap = Some(mm(gap, "--pair"));
            }
            "--neck" => opt.neck = Some(mm(&value(&mut i, &a), &a)),
            "--report" => report = Some(PathBuf::from(value(&mut i, &a))),
            "--double-vias" => {
                opt.double_vias = value(&mut i, &a)
                    .parse::<f64>()
                    .ok()
                    .filter(|m| *m >= 0.0)
                    .unwrap_or_else(|| die("--double-vias needs a width in millimetres, 0 or more"))
            }
            "--neck-length" => {
                opt.neck_length = value(&mut i, &a)
                    .parse::<f64>()
                    .ok()
                    .filter(|m| *m >= 0.0)
                    .unwrap_or_else(|| die("--neck-length needs a number of millimetres, 0 or more"))
            }
            "--keep-fills" => keep_fills = true,
            "--via-cost" => {
                opt.via_cost = value(&mut i, &a)
                    .parse::<f64>()
                    .ok()
                    .filter(|c| *c > 0.0)
                    .unwrap_or_else(|| die("--via-cost needs a number above 0"))
            }
            "--passes" => {
                opt.passes = value(&mut i, &a)
                    .parse()
                    .unwrap_or_else(|_| die("--passes needs a whole number"))
            }
            "--drc" => check = true,
            "-q" | "--quiet" => quiet = true,
            s if s.starts_with('-') => die(&format!("unknown option {} (see --help)", s)),
            _ => {
                if input.is_some() {
                    die("one board at a time");
                }
                input = Some(PathBuf::from(&a));
            }
        }
        i += 1;
    }
    opt.progress = !quiet;
    let input = input.unwrap_or_else(|| die("no board given (see --help)"));
    let text =
        std::fs::read_to_string(&input).unwrap_or_else(|e| die(&format!("cannot read {}: {}", input.display(), e)));
    let mut board = Board::parse(text).unwrap_or_else(|e| die(&format!("{}: {}", input.display(), e)));
    if let Some(names) = layer_names {
        let idx: Vec<usize> = names
            .iter()
            .map(|n| {
                board.copper.iter().position(|c| c == n).unwrap_or_else(|| {
                    die(&format!(
                        "no copper layer {} (the board has {})",
                        n,
                        board.copper.join(", ")
                    ))
                })
            })
            .collect();
        opt.layers = Some(idx);
    }
    for name in &span_names {
        let ends: Vec<usize> = name
            .split('-')
            .map(|n| {
                board.copper.iter().position(|c| c == n.trim()).unwrap_or_else(|| {
                    die(&format!(
                        "--via-spans {}: no copper layer {} (the board has {})",
                        name,
                        n,
                        board.copper.join(", ")
                    ))
                })
            })
            .collect();
        if ends.len() != 2 || ends[0] == ends[1] {
            die(&format!(
                "--via-spans {}: a via is named by the two layers it joins, as in F.Cu-In1.Cu",
                name
            ));
        }
        opt.via_spans.push((ends[0].min(ends[1]), ends[0].max(ends[1])));
    }
    if any_span {
        let nl = board.copper.len();
        opt.via_spans = (0..nl).flat_map(|a| (a + 1..nl).map(move |b| (a, b))).collect();
    }
    if (any_span || !span_names.is_empty()) && board.copper.len() < 3 {
        eprintln!("note: a board of two layers has no blind or buried vias: every via goes right through");
    }
    let rules = Rules::for_board(&input, &board).unwrap_or_else(|e| die(&e));
    for n in &rules.notes {
        eprintln!("note: {}", n);
    }

    let mut removed = 0;
    if rip {
        let mut spans: Vec<(usize, usize)> = Vec::new();
        for t in &board.tracks {
            if !t.locked
                && t.net
                    .as_deref()
                    .map(|n| router::wanted(&board, &opt, n))
                    .unwrap_or(false)
            {
                spans.push(t.span);
            }
        }
        for v in &board.vias {
            if !v.locked
                && v.net
                    .as_deref()
                    .map(|n| router::wanted(&board, &opt, n))
                    .unwrap_or(false)
            {
                spans.push(v.span);
            }
        }
        spans.sort();
        spans.dedup();
        removed = spans.len();
        let stripped = write::strip(&board.text, &spans);
        board = Board::parse(stripped).unwrap_or_else(|e| die(&format!("after ripping up: {}", e)));
    }

    let t0 = Instant::now();
    let result = router::route(&board, &rules, &opt);
    let secs = t0.elapsed().as_secs_f64();

    let output = output.unwrap_or_else(|| {
        let stem = input.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
        input.with_file_name(format!("{}-routed.kicad_pcb", stem))
    });
    // the zones' stored fills are stale once copper changes: out they go, to be poured again
    let changed = removed > 0 || !result.tracks.is_empty() || !result.vias.is_empty();
    let stale: &[(usize, usize)] = if changed && !keep_fills { &board.fills } else { &[] };
    let text = write::with_routes(&board, &result, stale);
    std::fs::write(&output, text).unwrap_or_else(|e| die(&format!("cannot write {}: {}", output.display(), e)));
    for w in &result.warnings {
        eprintln!("warning: {}", w);
    }

    let left: Vec<&router::NetReport> = result.nets.iter().filter(|n| !n.unrouted.is_empty()).collect();
    if !quiet {
        for (net, why) in &result.skipped {
            eprintln!("  left alone: {} ({})", net, why);
        }
    }
    let bottom = board.copper.len() - 1;
    let blind = result
        .vias
        .iter()
        .filter(|v| (v.layers.0 == 0) != (v.layers.1 == bottom))
        .count();
    let buried = result
        .vias
        .iter()
        .filter(|v| v.layers.0 != 0 && v.layers.1 != bottom)
        .count();
    println!(
        "{}: {} of {} nets joined in {:.1} s ({} passes); {} tracks, {} vias added{}{}",
        output.display(),
        result.nets.len() - left.len(),
        result.nets.len(),
        secs,
        result.passes,
        result.tracks.len(),
        result.vias.len(),
        if blind + buried > 0 {
            format!(" ({} blind, {} buried)", blind, buried)
        } else {
            String::new()
        },
        if rip {
            format!("; {} old items removed", removed)
        } else {
            String::new()
        }
    );
    for n in &left {
        println!(
            "  UNROUTED {}: {} of {} pieces left ({})",
            n.net,
            n.unrouted.len(),
            n.pieces,
            n.unrouted.join(", ")
        );
    }
    let necked: Vec<String> = result
        .nets
        .iter()
        .flat_map(|n| {
            n.necked
                .iter()
                .map(move |(pad, w)| format!("{} at {} ({} mm, not {} mm)", n.net, pad, w, n.width))
        })
        .collect();
    if !necked.is_empty() {
        println!("  narrowed to reach {} pad(s): {}", necked.len(), necked.join("; "));
    }
    for p in &result.pairs {
        if p.as_pair {
            println!(
                "  pair {} / {}: {:.1} mm side by side ({} of {} stretch{}) at {} mm with a {} mm gap; lengths {:.1} and {:.1} mm ({:.2} mm apart){}",
                p.positive,
                p.negative,
                p.coupled,
                p.coupled_legs,
                p.legs,
                if p.legs == 1 { "" } else { "es" },
                p.width,
                p.gap,
                p.length_positive,
                p.length_negative,
                (p.length_positive - p.length_negative).abs(),
                if p.note.is_empty() {
                    String::new()
                } else {
                    format!("; one by one elsewhere: {}", p.note)
                }
            );
        } else {
            println!(
                "  {} / {} routed one by one, not as a pair: {}",
                p.positive, p.negative, p.note
            );
        }
    }
    for l in &result.lengths {
        let short = l.target - l.after;
        println!(
            "  length of {}: {:.2} mm, was {:.2} ({}){}",
            l.net,
            l.after,
            l.before,
            l.why,
            if short > 1e-3 {
                format!("; {:.2} mm short of {:.2}: no room for more meanders", short, l.target)
            } else {
                String::new()
            }
        );
    }
    if result.doubled > 0 || !result.single.is_empty() {
        println!(
            "  second vias beside the vias of wide tracks: {} added{}",
            result.doubled,
            if result.single.is_empty() {
                String::new()
            } else {
                format!(
                    "; no room at {} ({})",
                    result.single.len(),
                    result
                        .single
                        .iter()
                        .take(6)
                        .map(|(n, p)| format!("{} at {:.2}, {:.2}", n, p.0, p.1))
                        .collect::<Vec<String>>()
                        .join("; ")
                )
            }
        );
    }
    // pours the new copper may have cut (an estimate on the grid; KiCad's fill decides)
    let cuts = if changed {
        pour::cuts(&board, &rules, &result, opt.grid, None)
    } else {
        Vec::new()
    };
    for c in &cuts {
        println!(
            "  WARNING the {} pour looks cut by the new tracks: {} of {} pads off the main piece ({}{}). \
             Check with --drc; stitching vias or --include-poured can join them",
            c.net,
            c.cut_off.len(),
            c.pads,
            c.cut_off.iter().take(8).cloned().collect::<Vec<String>>().join(", "),
            if c.cut_off.len() > 8 { ", ..." } else { "" }
        );
    }
    let mut status = if left.is_empty() { 0 } else { 1 };
    let mut drc_counts: Option<(usize, usize)> = None;
    if check {
        match drc(&output, &input) {
            Ok(d) => {
                let poured = match d.refilled {
                    Some(text) if !keep_fills => std::fs::write(&output, text).is_ok(),
                    _ => false,
                };
                println!(
                    "KiCad DRC: {} errors, {} unconnected{}",
                    d.errors,
                    d.unconnected,
                    if poured { " (zones poured again and saved)" } else { "" }
                );
                if !poured && !stale.is_empty() {
                    println!("  zone fills were removed from the output: pour them again in KiCad (B)");
                }
                drc_counts = Some((d.errors, d.unconnected));
                if d.errors > 0 || d.unconnected > 0 {
                    status = 1;
                }
            }
            Err(e) => {
                eprintln!("autorouter: DRC not run: {}", e);
                status = 1;
            }
        }
    } else if !stale.is_empty() {
        println!(
            "  {} zone fill(s) removed, as the new copper made them stale: pour them again in KiCad (B), or use --drc",
            stale.len()
        );
    } else if changed && keep_fills && !board.fills.is_empty() {
        println!("  the zones' old fills were kept: they overlap the new copper until poured again in KiCad (B)");
    }
    if let Some(path) = report {
        let q = json::quote;
        let list = |v: Vec<String>| format!("[{}]", v.join(", "));
        let track_mm: f64 = result
            .tracks
            .iter()
            .map(|t| ((t.a.0 - t.b.0).powi(2) + (t.a.1 - t.b.1).powi(2)).sqrt())
            .sum();
        let text = format!(
            "{{\n  \"version\": {},\n  \"board\": {},\n  \"output\": {},\n  \"seconds\": {:.3},\n  \"passes\": {},\n  \
             \"nets\": {},\n  \"joined\": {},\n  \"tracks_added\": {},\n  \"track_mm\": {:.1},\n  \"vias_added\": {},\n  \"blind_vias\": {},\n  \"buried_vias\": {},\n  \
             \"second_vias\": {},\n  \"vias_left_single\": {},\n  \"items_removed\": {},\n  \"unrouted\": {},\n  \
             \"narrowed\": {},\n  \"pairs\": {},\n  \"lengths\": {},\n  \"pour_cuts\": {},\n  \"warnings\": {},\n  \"drc\": {}\n}}\n",
            q(env!("CARGO_PKG_VERSION")),
            q(&input.display().to_string()),
            q(&output.display().to_string()),
            secs,
            result.passes,
            result.nets.len(),
            result.nets.len() - left.len(),
            result.tracks.len(),
            track_mm,
            result.vias.len(),
            blind,
            buried,
            result.doubled,
            result.single.len(),
            removed,
            list(left
                .iter()
                .map(|n| format!(
                    "{{\"net\": {}, \"pieces\": {}, \"left\": {}}}",
                    q(&n.net),
                    n.pieces,
                    list(n.unrouted.iter().map(|u| q(u)).collect())
                ))
                .collect()),
            list(result
                .nets
                .iter()
                .flat_map(|n| n.necked.iter().map(move |(pad, w)| format!(
                    "{{\"net\": {}, \"pad\": {}, \"width\": {}}}",
                    q(&n.net),
                    q(pad),
                    w
                )))
                .collect()),
            list(result
                .pairs
                .iter()
                .map(|p| format!(
                    "{{\"positive\": {}, \"negative\": {}, \"as_pair\": {}, \"stretches\": {}, \"side_by_side\": {}, \"note\": {}, \"width\": {}, \"gap\": {}, \
                     \"coupled_mm\": {:.2}, \"length_positive_mm\": {:.2}, \"length_negative_mm\": {:.2}}}",
                    q(&p.positive),
                    q(&p.negative),
                    p.as_pair,
                    p.legs,
                    p.coupled_legs,
                    q(&p.note),
                    p.width,
                    p.gap,
                    p.coupled,
                    p.length_positive,
                    p.length_negative
                ))
                .collect()),
            list(result
                .lengths
                .iter()
                .map(|l| format!(
                    "{{\"net\": {}, \"why\": {}, \"before_mm\": {:.3}, \"target_mm\": {:.3}, \"after_mm\": {:.3}}}",
                    q(&l.net),
                    q(&l.why),
                    l.before,
                    l.target,
                    l.after
                ))
                .collect()),
            list(cuts
                .iter()
                .map(|c| format!(
                    "{{\"net\": {}, \"pads\": {}, \"cut_off\": {}}}",
                    q(&c.net),
                    c.pads,
                    list(c.cut_off.iter().map(|u| q(u)).collect())
                ))
                .collect()),
            list(result.warnings.iter().map(|w| q(w)).collect()),
            match drc_counts {
                Some((e, u)) => format!("{{\"errors\": {}, \"unconnected\": {}}}", e, u),
                None => "null".to_string(),
            }
        );
        if let Err(e) = std::fs::write(&path, text) {
            eprintln!("autorouter: cannot write {}: {}", path.display(), e);
            status = 2;
        }
    }
    exit(status);
}
