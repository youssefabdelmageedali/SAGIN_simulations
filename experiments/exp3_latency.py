"""Exp 3 (Fig. 4): end-to-end uplink latency and outage vs. number of active drones."""
from dataclasses import replace

import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread
from sagin.world import SAGINWorld

N_GRID = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
QUICK_N_GRID = [1, 10, 30, 50]
POLICIES = ["all_tn", "all_haps", "all_sat", "balanced"]
LABELS = {"all_tn": "all on TN", "all_haps": "all on HAPS", "all_sat": "all on satellite",
          "balanced": "feasibility-aware balanced"}
STYLE = {"all_tn": dict(color=COLORS["tn"], marker="s", linestyle="--"),
         "all_haps": dict(color=COLORS["haps"], marker="d", linestyle="-."),
         "all_sat": dict(color=COLORS["sat"], marker="x", linestyle=":"),
         "balanced": dict(color=COLORS["hybrid"], marker="o")}


def assign(policy, sinr, th, tn_cap):
    """Tier per drone (TN=0, HAPS=1, SAT=2, -1 = no feasible tier). sinr: (R, N, 3)."""
    r, n, _ = sinr.shape
    fixed = {"all_tn": 0, "all_haps": 1, "all_sat": 2}
    if policy in fixed:  # fixed policies contend on their tier whether or not they are decodable
        return np.full((r, n), fixed[policy])
    feas = sinr > th
    order = np.argsort(-np.where(feas[..., 0], sinr[..., 0], -np.inf), axis=1, kind="stable")
    rank = np.argsort(order, axis=1, kind="stable")
    on_tn = feas[..., 0] & (rank < tn_cap)
    tier = np.full((r, n), -1)
    tier[on_tn] = 0
    rest = ~on_tn
    tier[rest & feas[..., 1]] = 1
    tier[rest & ~feas[..., 1] & feas[..., 2]] = 2
    tier[rest & ~feas[..., 1] & ~feas[..., 2] & feas[..., 0]] = 0  # TN is its only option
    return tier


def simulate(world, seed, n_grid, n_real, policies=POLICIES):
    """Mean latency (s, over non-outage drones) and outage fraction per policy and N, one seed."""
    p = world.p
    rng = make_rng(seed, 3)
    th = world.access_th
    bw = world.tier_bandwidths()
    lat = {k: np.full(len(n_grid), np.nan) for k in policies}
    out = {k: np.zeros(len(n_grid)) for k in policies}
    for ni, n in enumerate(n_grid):
        sinr = np.zeros((n_real, n, 3))
        dist = np.zeros((n_real, n, 3))
        for r in range(n_real):
            xy = world.swarm_in_mission(n, rng)
            dep = world.deploy(rng)
            sinr[r, :, 0], dist[r, :, 0] = world.tn_sinr(dep, xy, rng)
            sinr[r, :, 1], dist[r, :, 1] = world.haps_sinr(dep, xy, 0.0, rng)
            sinr[r, :, 2], dist[r, :, 2] = world.sat_sinr(dep, xy, rng)
        for pol in policies:
            tier = assign(pol, sinr, th, p.n_ra_tn + p.tn_cap_margin)
            t = np.maximum(tier, 0)
            s = np.take_along_axis(sinr, t[..., None], 2)[..., 0]
            d = np.take_along_axis(dist, t[..., None], 2)[..., 0]
            acc = world.random_access(tier, s > th, rng)
            latency = world.latency_s(acc, p.packet_mbit, world.shannon_rate(bw[t], s), d)
            ok = (tier >= 0) & np.isfinite(latency) & (latency <= p.latency_limit_s)
            out[pol][ni] = 1 - ok.mean()
            if ok.any():
                lat[pol][ni] = latency[ok].mean()
    return lat, out


def _collect(world, seeds, n_grid, n_real):
    lat = {k: [] for k in POLICIES}
    out = {k: [] for k in POLICIES}
    for s in seeds:
        l1, o1 = simulate(world, s, n_grid, n_real)
        for k in POLICIES:
            lat[k].append(l1[k] * 1e3)
            out[k].append(o1[k])
    return {k: np.array(v) for k, v in lat.items()}, {k: np.array(v) for k, v in out.items()}


def _plot(n_grid, lat, out, path):
    fig, ax = plt.subplots(figsize=FIGSIZE)
    for k in POLICIES:
        band(ax, n_grid, lat[k], STYLE[k]["color"], f"{LABELS[k]} (outage {100 * out[k].mean():.1f}%)",
             **{a: b for a, b in STYLE[k].items() if a != "color"})
    ax.set_xlabel("active drones requesting uplink")
    ax.set_ylabel("mean end-to-end latency (ms)")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False)
    save(fig, path)


def _metrics(n_grid, lat, out):
    res = {}
    for k in POLICIES:
        res[k] = {
            "latency_ms_curve_mean": np.nanmean(lat[k], 0).tolist(),
            "outage_curve_mean": out[k].mean(0).tolist(),
            f"latency_ms_at_N{n_grid[0]}": spread(lat[k][:, 0]),
            f"latency_ms_at_N{n_grid[-1]}": spread(lat[k][:, -1]),
            "outage_mean_over_N": spread(out[k].mean(1)),
            f"outage_at_N{n_grid[-1]}": spread(out[k][:, -1]),
        }
    return res


def run(world, seeds, outdir, quick=False, log=print):
    n_grid, n_real = (QUICK_N_GRID, 10) if quick else (N_GRID, 100)
    lat, out = _collect(world, seeds, n_grid, n_real)
    tn_curve = np.nanmean(lat["all_tn"], 0)
    checks = {"all_tn_latency_non_decreasing": bool(np.all(np.diff(tn_curve) >= -0.05 * tn_curve[:-1]))}
    for k in POLICIES:
        c = np.nanmean(lat[k], 0)
        log(f"{k:9s}: {c[0]:.1f} -> {c[-1]:.1f} ms (N={n_grid[0]}..{n_grid[-1]}), "
            f"outage mean {100 * out[k].mean():.1f}%, at N={n_grid[-1]} {100 * out[k][:, -1].mean():.1f}%")
    _plot(n_grid, lat, out, f"{outdir}/fig4_latency_vs_density.png")

    # same seeds and geometry, every tier at 5 MHz: separates contention from bandwidth effects
    eq = SAGINWorld(replace(world.p, b_tn_hz=5e6, b_haps_hz=5e6, b_sat_hz=5e6))
    lat_eq, out_eq = _collect(eq, seeds, n_grid, n_real)
    log("equal 5 MHz bandwidth on every tier:")
    for k in POLICIES:
        c = np.nanmean(lat_eq[k], 0)
        log(f"  {k:9s}: {c[0]:.1f} -> {c[-1]:.1f} ms, outage mean {100 * out_eq[k].mean():.1f}%")
    _plot(n_grid, lat_eq, out_eq, f"{outdir}/fig4b_latency_equal_bw.png")
    for k, v in checks.items():
        log(f"check {k}: {v}")
    return {"n_grid": n_grid, "nominal_bw": _metrics(n_grid, lat, out),
            "equal_5mhz_bw": _metrics(n_grid, lat_eq, out_eq), "checks": checks}
