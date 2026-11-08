"""Exp 2 (Fig. 3): coverage probability vs. distance from the hub, per tier and hybrid."""
import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread

DIST_KM = np.round(np.arange(0, 10.01, 0.5), 2)
CURVES = ["tn", "haps_ideal", "haps_err", "sat", "hybrid_ideal", "hybrid_err", "no_haps"]


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


def run(world, seeds, outdir, quick=False, log=print):
    n_real = 300 if quick else 2000
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
    ax.set_xlabel("drone distance from hub (km)")
    ax.set_ylabel(f"coverage Pr[SINR > {world.p.access_th_db:g} dB]")
    ax.set_ylim(-0.02, 1.02)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False)
    save(fig, f"{outdir}/fig3_coverage_vs_distance.png")

    out = {"distances_km": DIST_KM.tolist(), "tn_below_half_km": tn_half_km, "tn_below_1pct_km": tn_zero_km,
           "checks": checks}
    for k in CURVES:
        out[f"{k}_curve_mean"] = m[k].tolist()
        out[f"{k}_at_0km"] = spread(cov[k][:, 0])
        out[f"{k}_at_10km"] = spread(cov[k][:, -1])
        out[f"{k}_mean_over_distance"] = spread(cov[k].mean(1))
    return out
