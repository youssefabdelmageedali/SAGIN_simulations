"""Exp 2 (Fig. 3): coverage probability vs. distance from the hub, per tier and hybrid,
for drones (panel a) and, via exp2b, ground IoT sensors (panel b)."""
from dataclasses import replace

import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, band, make_rng, save, spread
from sagin.world import SAGINWorld

DIST_KM = np.round(np.arange(0, 10.01, 0.5), 2)
REPORT_KM = [0.0, 2.0, 5.0, 10.0]
CURVES = ["tn", "haps_ideal", "haps_err", "sat", "hybrid_ideal", "hybrid_err", "no_haps"]
SWEEP_TH_DB = [-5.0, 0.0, 5.0]
DRONE_GAINS_DBI = [0.0, 3.0, 10.0]
LABELS = {"tn": "TN alone", "haps_ideal": "HAPS alone, ideal", "haps_err": "HAPS alone, σ_b=3°",
          "sat": "satellite alone", "hybrid_ideal": "hybrid, ideal HAPS", "hybrid_err": "hybrid, σ_b=3°",
          "no_haps": "TN + satellite (no HAPS)"}
STYLE = {"hybrid_ideal": (COLORS["hybrid"], "-", "o"), "hybrid_err": (COLORS["hybrid"], "--", None),
         "no_haps": (COLORS["hybrid"], ":", None), "haps_ideal": (COLORS["haps"], "-", "d"),
         "haps_err": (COLORS["haps"], "--", None), "sat": (COLORS["sat"], "-", "x"), "tn": (COLORS["tn"], "-", "s")}
MC_TOL = 0.01  # tolerance on seed-mean curves (5 x 2000 realizations: binomial std <= 0.005)
_CACHE = {}


def coverage(world, seed, dist_km, n_real):
    """Coverage per curve at each distance for one seed, plus serving-satellite elevations.

    Each realization draws a fresh deployment and a random azimuth for the measurement line, so
    curves are averages over directions from the hub. Hybrid >= every tier and no-HAPS =
    max(TN, satellite) are asserted per realization."""
    rng = make_rng(seed, 2)
    th = world.access_th
    sb = world.p.sigma_b_deg
    hits = {k: np.zeros(len(dist_km)) for k in CURVES}
    elev = np.zeros(n_real)
    for r in range(n_real):
        dep = world.deploy(rng)
        az = rng.uniform(0, 2 * np.pi)
        xy = np.column_stack([dist_km * 1e3 * np.cos(az), dist_km * 1e3 * np.sin(az)])
        tn, _ = world.tn_sinr(dep, xy, rng)
        hseed = int(rng.integers(2 ** 63))  # same draws for both pointing cases
        h0, _ = world.haps_sinr(dep, xy, 0.0, np.random.default_rng(hseed))
        h3, _ = world.haps_sinr(dep, xy, sb, np.random.default_rng(hseed))
        sat, _ = world.sat_sinr(dep, xy, rng)
        elev[r] = world.sat_geometry(dep)[1]
        cov = {
            "tn": tn > th,
            "haps_ideal": h0 > th,
            "haps_err": h3 > th,
            "sat": sat > th,
            "hybrid_ideal": np.maximum.reduce([tn, h0, sat]) > th,
            "hybrid_err": np.maximum.reduce([tn, h3, sat]) > th,
            "no_haps": np.maximum(tn, sat) > th,
        }
        assert np.all(cov["no_haps"] == (cov["tn"] | cov["sat"]))
        assert all(np.all(cov["hybrid_ideal"] >= cov[k]) for k in ["tn", "haps_ideal", "sat", "no_haps"])
        assert all(np.all(cov["hybrid_err"] >= cov[k]) for k in ["tn", "haps_err", "sat", "no_haps"])
        for k in CURVES:
            hits[k] += cov[k]
    out = {k: v / n_real for k, v in hits.items()}
    out["sat_elev_deg"] = elev
    return out


def coverage_set(world, seeds, n_real):
    """Seed-stacked coverage (cached: the drone and sensor figures share one computation)."""
    key = (world.p, tuple(seeds), n_real)
    if key not in _CACHE:
        r = [coverage(world, s, DIST_KM, n_real) for s in seeds]
        _CACHE[key] = {k: np.array([x[k] for x in r]) for k in CURVES + ["sat_elev_deg"]}
    return _CACHE[key]


def threshold_sweep(world, seeds, n_real):
    return {th: coverage_set(SAGINWorld(replace(world.p, access_th_db=th)), seeds, n_real) for th in SWEEP_TH_DB}


def _at(km):
    return int(np.argmin(np.abs(DIST_KM - km)))


def summarize(world, seeds, n_real, terminal, log):
    """Log, check and collect the metrics for one terminal."""
    p = world.p
    cov = coverage_set(world, seeds, n_real)
    m = {k: cov[k].mean(0) for k in CURVES}
    elev = cov["sat_elev_deg"].ravel()
    visible = elev >= p.sat_min_elev_deg
    assert p.haps_spacing_km / 2 >= DIST_KM[-1], "curve points must stay in the hub HAPS cell"
    checks = {
        "tn_non_increasing": bool(np.all(np.diff(m["tn"]) <= MC_TOL)),
        "tn_max_at_hub": bool(m["tn"][0] >= m["tn"].max() - MC_TOL),
        "hybrid_ge_every_tier": True,  # asserted per realization in coverage()
        "no_haps_is_max_of_tn_sat_per_realization": True,  # asserted per realization in coverage()
        "haps_err_le_ideal": bool(np.all(cov["haps_err"] <= cov["haps_ideal"] + 1e-12)),
        "haps_non_increasing_from_nadir": bool(np.all(np.diff(m["haps_ideal"]) <= MC_TOL)
                                               and np.all(np.diff(m["haps_err"]) <= MC_TOL)),
        "sat_flat": bool(m["sat"].max() - m["sat"].min() <= 2 * MC_TOL),
    }
    below = np.nonzero(m["tn"] < 0.5)[0]
    tn_half_km = float(DIST_KM[below[0]]) if len(below) else None
    log(f"{terminal}: coverage at {' / '.join(f'{d:g}' for d in REPORT_KM)} km (seed mean)")
    for k in CURVES:
        log(f"  {k:13s}: " + " / ".join(f"{m[k][_at(d)]:.3f}" for d in REPORT_KM)
            + f"   (mean over 0-10 km {m[k].mean():.3f})")
    log(f"  TN coverage first below 0.5 at {tn_half_km} km")
    log(f"  at 10 km: hybrid(ideal) {m['hybrid_ideal'][-1]:.3f}, hybrid(σ_b) {m['hybrid_err'][-1]:.3f} "
        f"vs TN+sat only {m['no_haps'][-1]:.3f}")
    log(f"  serving satellite: elevation median {np.median(elev[visible]):.1f} deg, 5th pct "
        f"{np.percentile(elev[visible], 5):.1f}, min {elev[visible].min():.1f}; none above "
        f"{p.sat_min_elev_deg:g} deg in {100 * (1 - visible.mean()):.3f}% of realizations")
    for k, v in checks.items():
        log(f"  check {k}: {v}")

    sweep = threshold_sweep(world, seeds, n_real)
    log(f"  access-threshold sweep, coverage at {' / '.join(f'{d:g}' for d in REPORT_KM)} km:")
    for k in CURVES:
        log(f"    {k:13s} " + " | ".join(
            f"{th:+g} dB: " + "/".join(f"{sweep[th][k][:, _at(d)].mean():.3f}" for d in REPORT_KM)
            for th in SWEEP_TH_DB))

    out = {"distances_km": DIST_KM.tolist(), "tn_below_half_km": tn_half_km, "checks": checks,
           "sat_serving_elevation_deg": {"median": float(np.median(elev[visible])),
                                         "p5": float(np.percentile(elev[visible], 5)),
                                         "min": float(elev[visible].min()),
                                         "none_visible_fraction": float(1 - visible.mean())}}
    for k in CURVES:
        out[f"{k}_curve_mean"] = m[k].tolist()
        out[f"{k}_mean_over_distance"] = spread(cov[k].mean(1))
        for d in REPORT_KM:
            out[f"{k}_at_{d:g}km"] = spread(cov[k][:, _at(d)])
    out["threshold_sweep"] = {f"{th:+g}dB": {k: {**{f"at_{d:g}km": spread(c[k][:, _at(d)]) for d in REPORT_KM},
                                                 "mean_over_distance": spread(c[k].mean(1))} for k in CURVES}
                              for th, c in sweep.items()}
    return out


def make_figures(world, seeds, n_real, outdir):
    """Two-panel Fig. 3 (coverage vs distance) and Fig. 3b (threshold sweep): (a) drones, (b) sensors."""
    terminals = [("(a) drones", world, "drone distance from hub (km)"),
                 ("(b) ground IoT sensors", world.ground_sensor(), "sensor distance from hub (km)")]
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.8), sharey=True)
    for ax, (tag, w, xlabel) in zip(axes, terminals):
        cov = coverage_set(w, seeds, n_real)
        for k in CURVES:
            col, ls, mk = STYLE[k]
            band(ax, DIST_KM, cov[k], col, LABELS[k], linestyle=ls, marker=mk, markevery=2)
        ax.set_xlabel(xlabel)
        ax.set_ylim(-0.02, 1.02)
        ax.text(0.02, 0.03, tag, transform=ax.transAxes, fontsize=8, fontweight="bold")
    axes[0].set_ylabel(f"coverage Pr[SINR > {world.p.access_th_db:g} dB]")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=4, frameon=False)
    save(fig, f"{outdir}/fig3_coverage_vs_distance.png")

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.8), sharey=True)
    for ax, (tag, w, _) in zip(axes, terminals):
        sweep = threshold_sweep(w, seeds, n_real)
        for k in CURVES:
            col, ls, mk = STYLE[k]
            band(ax, SWEEP_TH_DB, np.array([sweep[th][k].mean(1) for th in SWEEP_TH_DB]).T, col, LABELS[k],
                 linestyle=ls, marker=mk or "o")
        ax.set_xlabel("access threshold (dB)")
        ax.set_xticks(SWEEP_TH_DB)
        ax.set_ylim(-0.02, 1.02)
        ax.text(0.02, 0.03, tag, transform=ax.transAxes, fontsize=8, fontweight="bold")
    axes[0].set_ylabel("coverage, mean over 0–10 km")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=4, frameon=False)
    save(fig, f"{outdir}/fig3b_coverage_threshold_sweep.png")


def run(world, seeds, outdir, quick=False, log=print):
    n_real = 300 if quick else 2000
    out = summarize(world, seeds, n_real, "drones", log)
    make_figures(world, seeds, n_real, outdir)

    # drone terminal antenna-gain sensitivity (Tx power unchanged; same seeds and geometry)
    log(f"drone antenna-gain sweep (Tx {world.p.p_drone_access_dbm:g} dBm), coverage at 10 km:")
    sweep = {}
    for g in DRONE_GAINS_DBI:
        c = coverage_set(SAGINWorld(replace(world.p, g_drone_dbi=g)), seeds, n_real)
        sweep[f"{g:g}dBi"] = {k: {"at_10km": spread(c[k][:, -1]), "mean_over_distance": spread(c[k].mean(1))}
                              for k in CURVES}
        log(f"  {g:4g} dBi: " + ", ".join(f"{k} {c[k][:, -1].mean():.3f}"
                                         for k in ["sat", "no_haps", "hybrid_ideal", "haps_ideal", "tn"]))
    out["drone_gain_sweep"] = sweep
    return out
