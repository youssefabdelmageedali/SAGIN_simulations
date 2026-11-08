"""Exp 2 (Fig. 3): coverage probability vs. distance from the hub, per tier and hybrid."""
from dataclasses import replace

import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread
from sagin.world import SAGINWorld

DIST_KM = np.round(np.arange(0, 10.01, 0.5), 2)
CURVES = ["tn", "haps_ideal", "haps_err", "sat", "hybrid_ideal", "hybrid_err", "no_haps"]
SWEEP_TH_DB = [-5.0, 0.0, 5.0]
DRONE_GAINS_DBI = [0.0, 3.0, 10.0]
LABELS = {"tn": "TN alone", "haps_ideal": "HAPS alone, ideal", "haps_err": "HAPS alone, σ_b=3°",
          "sat": "satellite alone", "hybrid_ideal": "hybrid, ideal HAPS", "hybrid_err": "hybrid, σ_b=3°",
          "no_haps": "TN + satellite (no HAPS)"}


def coverage(world, seed, dist_km, n_real):
    """Coverage per curve at each distance for one seed; asserts hybrid >= each tier pathwise."""
    rng = make_rng(seed, 2)
    th = world.access_th
    sb = world.p.sigma_b_deg
    xy = np.column_stack([dist_km * 1e3, np.zeros(len(dist_km))])
    hits = {k: np.zeros(len(dist_km)) for k in CURVES}
    for _ in range(n_real):
        dep = world.deploy(rng)
        tn, _ = world.tn_sinr(dep, xy, rng)
        hseed = int(rng.integers(2 ** 63))  # same draws for both pointing cases
        h0, _ = world.haps_sinr(dep, xy, 0.0, np.random.default_rng(hseed))
        h3, _ = world.haps_sinr(dep, xy, sb, np.random.default_rng(hseed))
        sat, _ = world.sat_sinr(dep, xy, rng)
        cov = {
            "tn": tn > th,
            "haps_ideal": h0 > th,
            "haps_err": h3 > th,
            "sat": sat > th,
            "hybrid_ideal": np.maximum.reduce([tn, h0, sat]) > th,
            "hybrid_err": np.maximum.reduce([tn, h3, sat]) > th,
            "no_haps": np.maximum(tn, sat) > th,
        }
        for k in CURVES:
            hits[k] += cov[k]
    return {k: v / n_real for k, v in hits.items()}


def study(world, seeds, n_real, outdir, fig_main, fig_sweep, xlabel, log):
    """Coverage vs. distance and the access-threshold sweep for the world's access terminal."""
    res = [coverage(world, s, DIST_KM, n_real) for s in seeds]
    cov = {k: np.array([r[k] for r in res]) for k in CURVES}
    m = {k: v.mean(0) for k, v in cov.items()}
    singles = ["tn", "haps_ideal", "sat"]
    checks = {
        "tn_non_increasing": bool(np.all(np.diff(m["tn"]) <= 0.01)),
        "hybrid_ge_every_tier": bool(all(np.all(cov["hybrid_ideal"] >= cov[k] - 1e-12) for k in singles)
                                     and all(np.all(cov["hybrid_err"] >= cov[k] - 1e-12)
                                             for k in ["tn", "haps_err", "sat"])),
        "haps_err_le_ideal": bool(np.all(cov["haps_err"] <= cov["haps_ideal"] + 1e-12)),
    }
    below = np.nonzero(m["tn"] < 0.5)[0]
    tn_half_km = float(DIST_KM[below[0]]) if len(below) else None
    zero = np.nonzero(m["tn"] < 0.01)[0]
    tn_zero_km = float(DIST_KM[zero[0]]) if len(zero) else None
    for k in CURVES:
        log(f"{k:13s}: {m[k][0]:.3f} at 0 km -> {m[k][-1]:.3f} at 10 km (mean over distance {m[k].mean():.3f})")
    log(f"TN coverage first below 0.5 at {tn_half_km} km, below 0.01 at {tn_zero_km} km")
    log(f"HAPS tier gain at 10 km: hybrid(ideal) {m['hybrid_ideal'][-1]:.3f} vs TN+sat only {m['no_haps'][-1]:.3f}")
    for k, v in checks.items():
        log(f"check {k}: {v}")
        assert k != "hybrid_ge_every_tier" or v, "hybrid must dominate every tier"

    fig, ax = plt.subplots(figsize=FIGSIZE)
    band(ax, DIST_KM, cov["hybrid_ideal"], COLORS["hybrid"], "hybrid, ideal HAPS", marker="o")
    band(ax, DIST_KM, cov["hybrid_err"], COLORS["hybrid"], rf"hybrid, HAPS $\sigma_b$={world.p.sigma_b_deg:g}°",
         linestyle="--")
    band(ax, DIST_KM, cov["no_haps"], COLORS["hybrid"], "TN + satellite (no HAPS)", linestyle=":")
    band(ax, DIST_KM, cov["haps_ideal"], COLORS["haps"], "HAPS alone, ideal", marker="d")
    band(ax, DIST_KM, cov["haps_err"], COLORS["haps"], rf"HAPS alone, $\sigma_b$={world.p.sigma_b_deg:g}°",
         linestyle="--")
    band(ax, DIST_KM, cov["sat"], COLORS["sat"], "satellite alone", marker="x")
    band(ax, DIST_KM, cov["tn"], COLORS["tn"], "TN alone", marker="s")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(f"coverage Pr[SINR > {world.p.access_th_db:g} dB]")
    ax.set_ylim(-0.02, 1.02)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False)
    save(fig, f"{outdir}/{fig_main}")

    # access-threshold sensitivity (same seeds and geometry; only the threshold changes)
    sweep = {}
    for th_db in SWEEP_TH_DB:
        if th_db == world.p.access_th_db:
            c = cov
        else:
            w = SAGINWorld(replace(world.p, access_th_db=th_db))
            r = [coverage(w, s, DIST_KM, n_real) for s in seeds]
            c = {k: np.array([x[k] for x in r]) for k in CURVES}
        sweep[th_db] = c
    log("access-threshold sweep, coverage at 0 km / 5 km / 10 km (mean over seeds):")
    i5 = int(np.argmin(np.abs(DIST_KM - 5)))
    for k in CURVES:
        log(f"  {k:13s} " + " | ".join(
            f"{th:+g} dB: {sweep[th][k][:, 0].mean():.3f}/{sweep[th][k][:, i5].mean():.3f}/{sweep[th][k][:, -1].mean():.3f}"
            for th in SWEEP_TH_DB))
    fig, ax = plt.subplots(figsize=FIGSIZE)
    style = {"tn": (COLORS["tn"], "-", "s"), "haps_ideal": (COLORS["haps"], "-", "d"),
             "haps_err": (COLORS["haps"], "--", "d"), "sat": (COLORS["sat"], "-", "x"),
             "hybrid_ideal": (COLORS["hybrid"], "-", "o"), "hybrid_err": (COLORS["hybrid"], "--", "o"),
             "no_haps": (COLORS["hybrid"], ":", "o")}
    for k in CURVES:
        col, ls, mk = style[k]
        band(ax, SWEEP_TH_DB, np.array([sweep[th][k].mean(1) for th in SWEEP_TH_DB]).T, col, LABELS[k],
             linestyle=ls, marker=mk)
    ax.set_xlabel("access threshold (dB)")
    ax.set_ylabel("coverage, mean over 0–10 km")
    ax.set_xticks(SWEEP_TH_DB)
    ax.set_ylim(-0.02, 1.02)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False)
    save(fig, f"{outdir}/{fig_sweep}")

    out = {"distances_km": DIST_KM.tolist(), "tn_below_half_km": tn_half_km, "tn_below_1pct_km": tn_zero_km,
           "checks": checks}
    out["threshold_sweep"] = {f"{th:+g}dB": {k: {"at_0km": spread(c[k][:, 0]), "at_5km": spread(c[k][:, i5]),
                                                  "at_10km": spread(c[k][:, -1]),
                                                  "mean_over_distance": spread(c[k].mean(1))} for k in CURVES}
                              for th, c in sweep.items()}
    for k in CURVES:
        out[f"{k}_curve_mean"] = m[k].tolist()
        out[f"{k}_at_0km"] = spread(cov[k][:, 0])
        out[f"{k}_at_10km"] = spread(cov[k][:, -1])
        out[f"{k}_mean_over_distance"] = spread(cov[k].mean(1))
    return out


def run(world, seeds, outdir, quick=False, log=print):
    n_real = 300 if quick else 2000
    out = study(world, seeds, n_real, outdir, "fig3_coverage_vs_distance.png", "fig3b_coverage_threshold_sweep.png",
                "drone distance from hub (km)", log)

    # drone terminal antenna-gain sensitivity (Tx power unchanged; same seeds and geometry)
    log(f"drone antenna-gain sweep (Tx {world.p.p_drone_access_dbm:g} dBm), coverage at 10 km:")
    sweep = {}
    for g in DRONE_GAINS_DBI:
        w = world if g == world.p.g_drone_dbi else SAGINWorld(replace(world.p, g_drone_dbi=g))
        r = [coverage(w, s, DIST_KM, n_real) for s in seeds]
        c = {k: np.array([x[k] for x in r]) for k in CURVES}
        sweep[f"{g:g}dBi"] = {k: {"at_10km": spread(c[k][:, -1]), "mean_over_distance": spread(c[k].mean(1))}
                              for k in CURVES}
        log(f"  {g:4g} dBi: " + ", ".join(f"{k} {c[k][:, -1].mean():.3f}"
                                         for k in ["sat", "no_haps", "hybrid_ideal", "haps_ideal", "tn"]))
    out["drone_gain_sweep"] = sweep
    return out
