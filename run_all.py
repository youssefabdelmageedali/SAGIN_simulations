"""Run every experiment with fixed seeds; write PNGs, results/run_log.txt and results/metrics.json.

    python run_all.py                 # everything
    python run_all.py exp3            # one experiment (fig names such as fig4 also work)
    python run_all.py --quick --out /tmp/x   # small Monte Carlo sizes, for tests
"""
import argparse
import importlib
import json
import os
import sys
import time
from dataclasses import asdict

import numpy as np

from sagin.world import PLACEHOLDERS, SAGINWorld

EXPERIMENTS = {
    "exp1": ("experiments.exp1_graph", "fig2"),
    "exp2": ("experiments.exp2_coverage", "fig3"),
    "exp3": ("experiments.exp3_latency", "fig4"),
    "exp4": ("experiments.exp5a_compute", "fig5a"),
    "exp5": ("experiments.exp5b_energy", "fig5b"),
}
SEEDS = [1, 2, 3, 4, 5]


def _plain(x):
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return _plain(x.tolist())
    if isinstance(x, (np.bool_, bool)):
        return bool(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.floating, float)):
        x = float(x)
        return None if np.isnan(x) else x
    return x


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*", help="exp1..exp5 or fig2..fig5b (default: all)")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default="results")
    args = ap.parse_args(argv)
    by_fig = {fig: k for k, (_, fig) in EXPERIMENTS.items()}
    names = [by_fig.get(n, n) for n in args.names] or list(EXPERIMENTS)
    unknown = [n for n in names if n not in EXPERIMENTS]
    if unknown:
        ap.error(f"unknown experiment(s): {unknown}")
    os.makedirs(args.out, exist_ok=True)

    world = SAGINWorld()
    mpath = os.path.join(args.out, "metrics.json")
    metrics = {}
    if len(names) < len(EXPERIMENTS) and os.path.exists(mpath):
        with open(mpath) as f:
            metrics = json.load(f)
    metrics.update({"params": asdict(world.p), "placeholders": PLACEHOLDERS, "seeds": SEEDS,
                    "quick": args.quick})

    lines = []
    failed = []
    for name in names:
        module, fig = EXPERIMENTS[name]
        section = []
        t0 = time.time()
        res = importlib.import_module(module).run(world, SEEDS, args.out, quick=args.quick, log=section.append)
        dt = time.time() - t0
        metrics[name] = res
        bad = [k for k, v in res.get("checks", {}).items() if not v]
        failed += [f"{name}:{k}" for k in bad]
        lines += [f"===== {name} ({fig}) {dt:.0f}s", *section, ""]
        print(f"{name} ({fig}): {'OK' if not bad else 'CHECK FAILED ' + ', '.join(bad)} ({dt:.0f}s)", flush=True)

    with open(mpath, "w") as f:
        json.dump(_plain(metrics), f, indent=2, sort_keys=True)
    log_path = os.path.join(args.out, "run_log.txt")
    with open(log_path, "w" if len(names) == len(EXPERIMENTS) else "a") as f:
        f.write("\n".join(lines) + "\n")
    print(f"log: {log_path}\nmetrics: {mpath}")
    if failed:
        print("SANITY CHECK FAILURES: " + ", ".join(failed))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
