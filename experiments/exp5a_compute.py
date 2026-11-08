"""Exp 4 (Fig. 5a): task completion probability vs. task complexity, solo vs. graph-bounded split."""
import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread

TASKS_GFLOP = np.arange(0, 2001, 50)


def run(world, seeds, outdir, quick=False, log=print):
    p = world.p
    n_real = 5 if quick else 40
    side = p.swarm_window_km * 1e3
    coop, solo, degs, helpers = [], [], [], []
    for seed in seeds:
        rng = make_rng(seed, 4)
        caps = []
        for _ in range(n_real):
            xy = world.thomas_pcp(p.swarm_density_km2, side, rng)
            if len(xy) < 2:
                continue
            s = world.mesh_sinr(xy, rng, side_m=side)
            rates = world.helper_rates(s)
            caps.append(world.coop_capacity_gflop(rates))
            degs.append(world.sg_graph(s).sum(1).mean())
            helpers.append((rates > 0).sum(1).mean())
        caps = np.concatenate(caps)
        coop.append((caps[:, None] >= TASKS_GFLOP[None]).mean(0) * 100)
        solo.append(world.solo_completes(TASKS_GFLOP).astype(float) * 100)
    coop, solo = np.array(coop), np.array(solo)

    wall = p.drone_gflops * p.task_deadline_s
    checks = {
        "solo_exact_step": bool(np.all(solo[:, TASKS_GFLOP <= wall] == 100) and np.all(solo[:, TASKS_GFLOP > wall] == 0)),
        "coop_ge_solo": bool(np.all(coop >= solo)),
    }
    at = {g: coop[:, TASKS_GFLOP == g][:, 0] for g in (500, 1000, 1500, 2000)}
    log(f"solo: 100% up to {wall:g} GFLOP, 0% above (step at capacity x deadline)")
    log("coop completion: " + ", ".join(f"{g} GFLOP {v.mean():.1f}%" for g, v in at.items()))
    log(f"mean SG out-degree {np.mean(degs):.2f}, mean helpers used {np.mean(helpers):.2f}")
    for k, v in checks.items():
        log(f"check {k}: {v}")

    fig, ax = plt.subplots(figsize=FIGSIZE)
    band(ax, TASKS_GFLOP, coop, COLORS["drone"], f"graph-bounded split (≤{p.max_helpers} helpers)", marker="o",
         markevery=4)
    band(ax, TASKS_GFLOP, solo, COLORS["ref"], "solo drone", linestyle="--")
    ax.set_xlabel("task size (GFLOP), deadline 1 s")
    ax.set_ylabel("task completion probability (%)")
    ax.set_ylim(-2, 102)
    ax.legend(loc="lower left")
    save(fig, f"{outdir}/fig5a_completion_vs_complexity.png")

    return {
        "tasks_gflop": TASKS_GFLOP.tolist(),
        "coop_pct_curve_mean": coop.mean(0).tolist(),
        "solo_wall_gflop": wall,
        **{f"coop_pct_at_{g}gflop": spread(v) for g, v in at.items()},
        "mean_sg_out_degree": float(np.mean(degs)),
        "mean_helpers_used": float(np.mean(helpers)),
        "checks": checks,
    }
