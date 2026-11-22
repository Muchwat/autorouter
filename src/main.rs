use autorouter::board::Board;
use autorouter::router::{self, Options};
use autorouter::rules::Rules;
use autorouter::write;
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
      --margin MM        extra gap on every clearance for the grid's corners (default 0.05)
      --width MM         one track width for every net (default: from the rules)
      --via D/DRILL      one via size for every net, e.g. 0.6/0.3 (default: from the rules)
      --no-vias          route without vias
      --via-cost N       what a via costs, in grid steps (default 12)
      --passes N         most negotiation passes (default 40)
      --drc              check the result with KiCad's DRC (kicad-cli), custom rules included
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

/// KiCad's DRC on `pcb`, with the rules from beside `rules_from`. (errors, unconnected, report)
fn drc(pcb: &Path, rules_from: &Path) -> Result<(usize, usize, String), String> {
    let cli = kicad_cli().ok_or("kicad-cli not found (set KICAD_CLI to its path)")?;
    // in a folder of its own, named like the original so its .kicad_pro and .kicad_dru apply
    let dir = std::env::temp_dir().join(format!("autorouter-drc-{}", std::process::id()));
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let stem = rules_from.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
    let board = dir.join(format!("{}.kicad_pcb", stem));
    std::fs::copy(pcb, &board).map_err(|e| e.to_string())?;
    for ext in ["kicad_pro", "kicad_dru"] {
        let src = rules_from.with_extension(ext);
        if src.exists() {
            std::fs::copy(&src, dir.join(format!("{}.{}", stem, ext))).map_err(|e| e.to_string())?;
        }
    }
    let report = dir.join("drc.rpt");
    let out = Command::new(&cli)
        .args([
            "pcb",
            "drc",
            "--severity-error",
            "--units",
            "mm",
            "--refill-zones",
            "-o",
        ])
        .arg(&report)
        .arg(&board)
        .output()
        .map_err(|e| format!("could not run {}: {}", cli.display(), e))?;
    let text = std::fs::read_to_string(&report)
        .map_err(|_| format!("kicad-cli wrote no report: {}", String::from_utf8_lossy(&out.stderr)))?;
    let _ = std::fs::remove_dir_all(&dir);
    let count = |what: &str| -> usize {
        text.lines()
            .find(|l| l.starts_with("** Found") && l.contains(what))
            .and_then(|l| l.split_whitespace().nth(2))
            .and_then(|n| n.parse().ok())
            .unwrap_or(0)
    };
    Ok((count("DRC violations"), count("unconnected"), text))
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
    let (mut input, mut output, mut rip, mut check, mut quiet) =
        (None::<PathBuf>, None::<PathBuf>, false, false, false);
    let mut layer_names: Option<Vec<String>> = None;
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
            "--grid" => opt.grid = mm(&value(&mut i, &a), &a),
            "--margin" => {
                opt.margin = value(&mut i, &a)
                    .parse()
                    .unwrap_or_else(|_| die("--margin needs a number"))
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
            "--via-cost" => opt.via_cost = mm(&value(&mut i, &a), &a),
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
    let text = write::with_routes(&board, &result, &[]);
    std::fs::write(&output, text).unwrap_or_else(|e| die(&format!("cannot write {}: {}", output.display(), e)));

    let left: Vec<&router::NetReport> = result.nets.iter().filter(|n| !n.unrouted.is_empty()).collect();
    if !quiet {
        for (net, why) in &result.skipped {
            eprintln!("  left alone: {} ({})", net, why);
        }
    }
    println!(
        "{}: {} of {} nets joined in {:.1} s ({} passes); {} tracks, {} vias added{}",
        output.display(),
        result.nets.len() - left.len(),
        result.nets.len(),
        secs,
        result.passes,
        result.tracks.len(),
        result.vias.len(),
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
    let mut status = if left.is_empty() { 0 } else { 1 };
    if check {
        match drc(&output, &input) {
            Ok((errors, unconnected, _)) => {
                println!("KiCad DRC: {} errors, {} unconnected", errors, unconnected);
                if errors > 0 || unconnected > 0 {
                    status = 1;
                }
            }
            Err(e) => {
                eprintln!("autorouter: DRC not run: {}", e);
                status = 1;
            }
        }
    }
    exit(status);
}
