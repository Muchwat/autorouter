//! autorouter-bench: route a set of boards from nothing and score the result.
//!
//! Each board has its tracks ripped up and routed again by the `autorouter`
//! beside this program, the result is checked by KiCad's DRC, and one line per
//! board says how it went. Saved results can be compared with an earlier run,
//! so a change to the router that loses nets or adds DRC errors is caught.

use autorouter::json::{self, Json};
use std::path::{Path, PathBuf};
use std::process::{exit, Command};

const USAGE: &str = "\
autorouter-bench: route boards from nothing and score the result with KiCad's DRC

usage: autorouter-bench [options] [BOARD.kicad_pcb ...]

      --list FILE      boards to run, one per line: a path, then any autorouter options
                       for that board (lines starting with # are skipped)
      --opts \"...\"     autorouter options for the boards named on the command line
      --repeat         also route each board twice more and check both come out the same
      --save FILE      write the results as JSON
      --baseline FILE  compare with saved results; exit 1 if a board joins fewer nets,
                       or has more DRC errors or unconnected items, than it did
      --work DIR       where to put the routed boards (default: a folder in the temp dir)
  -h, --help           this text

Every board is run with --rip --drc; the tracks it has now are not kept (locked ones are).";

struct Row {
    board: String,
    nets: usize,
    joined: usize,
    vias: usize,
    track_mm: f64,
    seconds: f64,
    errors: Option<usize>,
    unconnected: Option<usize>,
    same: Option<bool>,
    failed: Option<String>,
}

fn die(msg: &str) -> ! {
    eprintln!("autorouter-bench: {}", msg);
    exit(2)
}

fn num(j: &Json, key: &str) -> f64 {
    j.get(key).and_then(Json::as_f64).unwrap_or(0.0)
}

/// The board's text without the lines that differ from run to run (ids).
fn stable(path: &Path) -> Option<String> {
    let text = std::fs::read_to_string(path).ok()?;
    Some(
        text.lines()
            .filter(|l| !l.trim_start().starts_with("(uuid") && !l.trim_start().starts_with("(tstamp"))
            .collect::<Vec<&str>>()
            .join("\n"),
    )
}

fn run(router: &Path, board: &Path, opts: &[String], work: &Path, repeat: bool) -> Row {
    let stem = board
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("board")
        .to_string();
    let dir = work.join(&stem);
    let mut row = Row {
        board: stem.clone(),
        nets: 0,
        joined: 0,
        vias: 0,
        track_mm: 0.0,
        seconds: 0.0,
        errors: None,
        unconnected: None,
        same: None,
        failed: None,
    };
    if let Err(e) = std::fs::create_dir_all(&dir) {
        row.failed = Some(format!("cannot make {}: {}", dir.display(), e));
        return row;
    }
    let report = dir.join("report.json");
    let _ = std::fs::remove_file(&report);
    let go = |out: &Path, extra: &[&str]| {
        Command::new(router)
            .arg(board)
            .arg("-o")
            .arg(out)
            .args(["--rip", "-q"])
            .args(extra)
            .args(opts)
            .output()
    };
    let out = dir.join(format!("{}-routed.kicad_pcb", stem));
    match go(&out, &["--drc", "--report", &report.display().to_string()]) {
        Err(e) => {
            row.failed = Some(format!("could not run {}: {}", router.display(), e));
            return row;
        }
        Ok(o) if o.status.code() == Some(2) => {
            row.failed = Some(String::from_utf8_lossy(&o.stderr).trim().to_string());
            return row;
        }
        Ok(_) => {}
    }
    let j = match std::fs::read_to_string(&report)
        .map_err(|e| e.to_string())
        .and_then(|t| json::parse(&t))
    {
        Ok(j) => j,
        Err(e) => {
            row.failed = Some(format!("no report: {}", e));
            return row;
        }
    };
    row.nets = num(&j, "nets") as usize;
    row.joined = num(&j, "joined") as usize;
    row.vias = num(&j, "vias_added") as usize;
    row.track_mm = num(&j, "track_mm");
    row.seconds = num(&j, "seconds");
    if let Some(d) = j.get("drc").filter(|d| d.as_obj().is_some()) {
        row.errors = Some(num(d, "errors") as usize);
        row.unconnected = Some(num(d, "unconnected") as usize);
    }
    if repeat {
        let (a, b) = (dir.join("again-1.kicad_pcb"), dir.join("again-2.kicad_pcb"));
        let ok = go(&a, &[]).is_ok() && go(&b, &[]).is_ok();
        row.same = Some(ok && stable(&a).is_some() && stable(&a) == stable(&b));
    }
    row
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || args.iter().any(|a| a == "-h" || a == "--help") {
        println!("{}", USAGE);
        exit(if args.is_empty() { 2 } else { 0 });
    }
    let mut jobs: Vec<(PathBuf, Vec<String>)> = Vec::new();
    let (mut save, mut baseline, mut work, mut repeat) = (None::<PathBuf>, None::<PathBuf>, None::<PathBuf>, false);
    let mut opts: Vec<String> = Vec::new();
    let mut named: Vec<PathBuf> = Vec::new();
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
            "--list" => {
                let path = PathBuf::from(value(&mut i, &a));
                let text = std::fs::read_to_string(&path)
                    .unwrap_or_else(|e| die(&format!("cannot read {}: {}", path.display(), e)));
                let base = path.parent().map(|p| p.to_path_buf()).unwrap_or_default();
                for line in text
                    .lines()
                    .map(str::trim)
                    .filter(|l| !l.is_empty() && !l.starts_with('#'))
                {
                    // the board's path ends at .kicad_pcb, spaces and all; options follow
                    let cut = line
                        .find(".kicad_pcb")
                        .map(|k| k + ".kicad_pcb".len())
                        .unwrap_or(line.len());
                    let board = PathBuf::from(&line[..cut]);
                    let board = if board.is_absolute() { board } else { base.join(board) };
                    jobs.push((board, line[cut..].split_whitespace().map(|s| s.to_string()).collect()));
                }
            }
            "--opts" => opts = value(&mut i, &a).split_whitespace().map(|s| s.to_string()).collect(),
            "--repeat" => repeat = true,
            "--save" => save = Some(PathBuf::from(value(&mut i, &a))),
            "--baseline" => baseline = Some(PathBuf::from(value(&mut i, &a))),
            "--work" => work = Some(PathBuf::from(value(&mut i, &a))),
            s if s.starts_with('-') => die(&format!("unknown option {} (see --help)", s)),
            _ => named.push(PathBuf::from(&a)),
        }
        i += 1;
    }
    jobs.extend(named.into_iter().map(|b| (b, opts.clone())));
    if jobs.is_empty() {
        die("no boards given (see --help)");
    }
    let exe = std::env::current_exe().unwrap_or_else(|e| die(&format!("cannot find myself: {}", e)));
    let router = exe.with_file_name(format!("autorouter{}", std::env::consts::EXE_SUFFIX));
    if !router.exists() {
        die(&format!(
            "{} is not there: build it first (cargo build --release)",
            router.display()
        ));
    }
    let work = work.unwrap_or_else(|| std::env::temp_dir().join("autorouter-bench"));

    println!(
        "{:<24} {:>5} {:>7} {:>6} {:>5} {:>9} {:>7} {:>7} {:>8}{}",
        "board",
        "nets",
        "joined",
        "%",
        "vias",
        "track mm",
        "s",
        "errors",
        "unconn.",
        if repeat { "  repeat" } else { "" }
    );
    let mut rows: Vec<Row> = Vec::new();
    for (board, o) in &jobs {
        let r = run(&router, board, o, &work, repeat);
        match &r.failed {
            Some(why) => println!("{:<24} FAILED: {}", r.board, why),
            None => println!(
                "{:<24} {:>5} {:>7} {:>6.1} {:>5} {:>9.0} {:>7.1} {:>7} {:>8}{}",
                r.board,
                r.nets,
                r.joined,
                if r.nets > 0 {
                    100.0 * r.joined as f64 / r.nets as f64
                } else {
                    100.0
                },
                r.vias,
                r.track_mm,
                r.seconds,
                r.errors.map(|v| v.to_string()).unwrap_or("-".into()),
                r.unconnected.map(|v| v.to_string()).unwrap_or("-".into()),
                match r.same {
                    Some(true) => "    same",
                    Some(false) => "  DIFFER",
                    None => "",
                }
            ),
        }
        rows.push(r);
    }
    let ok: Vec<&Row> = rows.iter().filter(|r| r.failed.is_none()).collect();
    let (nets, joined): (usize, usize) = (ok.iter().map(|r| r.nets).sum(), ok.iter().map(|r| r.joined).sum());
    println!(
        "{} board(s): {} of {} nets joined ({:.1}%), {} DRC errors, {} unconnected, {:.1} s of routing",
        ok.len(),
        joined,
        nets,
        if nets > 0 {
            100.0 * joined as f64 / nets as f64
        } else {
            100.0
        },
        ok.iter().filter_map(|r| r.errors).sum::<usize>(),
        ok.iter().filter_map(|r| r.unconnected).sum::<usize>(),
        ok.iter().map(|r| r.seconds).sum::<f64>()
    );

    let mut status = 0;
    if rows.iter().any(|r| r.failed.is_some() || r.same == Some(false)) {
        status = 1;
    }
    if let Some(path) = &baseline {
        let text =
            std::fs::read_to_string(path).unwrap_or_else(|e| die(&format!("cannot read {}: {}", path.display(), e)));
        let base = json::parse(&text).unwrap_or_else(|e| die(&format!("{}: {}", path.display(), e)));
        for b in base.get("boards").map(Json::as_arr).unwrap_or(&[]) {
            let name = b.get("board").and_then(Json::as_str).unwrap_or("");
            let now = match rows.iter().find(|r| r.board == name && r.failed.is_none()) {
                Some(r) => r,
                None => continue,
            };
            let worse = |what: &str, was: f64, is: f64, more_is_worse: bool| {
                let bad = if more_is_worse { is > was } else { is < was };
                if bad {
                    println!("  WORSE {}: {} was {}, is {}", name, what, was, is);
                }
                bad
            };
            let mut bad = worse("nets joined", num(b, "joined"), now.joined as f64, false);
            if let (Some(e), Some(u)) = (now.errors, now.unconnected) {
                bad |= worse("DRC errors", num(b, "errors"), e as f64, true);
                bad |= worse("unconnected", num(b, "unconnected"), u as f64, true);
            }
            if bad {
                status = 1;
            }
        }
        if status == 0 {
            println!("no board is worse than in {}", path.display());
        }
    }
    if let Some(path) = &save {
        let body: Vec<String> = ok
            .iter()
            .map(|r| {
                format!(
                    "    {{\"board\": {}, \"nets\": {}, \"joined\": {}, \"vias\": {}, \"track_mm\": {:.1}, \"seconds\": {:.2}, \"errors\": {}, \"unconnected\": {}}}",
                    json::quote(&r.board),
                    r.nets,
                    r.joined,
                    r.vias,
                    r.track_mm,
                    r.seconds,
                    r.errors.unwrap_or(0),
                    r.unconnected.unwrap_or(0)
                )
            })
            .collect();
        let text = format!(
            "{{\n  \"version\": {},\n  \"boards\": [\n{}\n  ]\n}}\n",
            json::quote(env!("CARGO_PKG_VERSION")),
            body.join(",\n")
        );
        std::fs::write(path, text).unwrap_or_else(|e| die(&format!("cannot write {}: {}", path.display(), e)));
    }
    exit(status);
}
