# Benchmark

`autorouter-bench` rips up the tracks of each board, routes it again, checks the result with KiCad's DRC and prints one line per board. It needs `kicad-cli` (see `--drc` in the main README).

```bash
cargo build --release
```

```bash
target/release/autorouter-bench --list bench/boards.txt --repeat --save bench/results.json
```

A list file names one board per line, with any `autorouter` options for that board after the path. `boards.example.txt` shows the form. The boards themselves are not in this repository; point the list at boards of your own.

`--repeat` routes each board twice more and checks that both runs give the same tracks. `--baseline FILE` compares with saved results and exits 1 if any board joins fewer nets, or has more DRC errors, unconnected items or wide-track vias left without a second via (the `lone` column), than it did. Run it before and after a change to the router:

```bash
target/release/autorouter-bench --list bench/boards.txt --baseline bench/results.json
```

The boards' own `.kicad_pro` and `.kicad_dru` files apply, so the score is against each board's real rules. The board files are only read; routed copies go to a folder in the temp directory (`--work DIR` to choose).
