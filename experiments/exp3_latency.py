"""Exp 3 (Fig. 4): end-to-end uplink latency and outage vs. number of active drones,
with a massive-IoT random-access background load."""
from dataclasses import replace

import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread
from sagin.world import SAGINWorld

N_GRID = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
QUICK_N_GRID = [1, 10, 30, 50]
POLICIES = ["all_tn", "all_haps", "all_sat", "tn_first", "balanced"]
CAUSES = ["no_coverage", "ra_failure", "over_limit"]
TIERS = ["tn", "haps", "sat"]
SWEEP_TH_DB = [-5.0, 0.0, 5.0]
IOT_LOADS = {"none": 0.0, "low": 0.1, "medium": 0.5, "high": 1.0}  # IoT preambles / resource / opportunity
HAPS_RA_REDUCED = 16  # congested-HAPS case: fewer HAPS random-access resources
LABELS = {"all_tn": "all on TN", "all_haps": "all on HAPS", "all_sat": "all on satellite",
          "tn_first": "terrestrial-first (baseline)", "balanced": "load-aware balanced"}
SHORT = {"all_tn": "TN", "all_haps": "HAPS", "all_sat": "Sat", "tn_first": "TN-1st", "balanced": "Bal."}
STYLE = {"all_tn": dict(color=COLORS["tn"], marker="s", linestyle="--"),
         "all_haps": dict(color=COLORS["haps"], marker="d", linestyle="-."),
         "all_sat": dict(color=COLORS["sat"], marker="x", linestyle=":"),
         "tn_first": dict(color=COLORS["hybrid"], marker="s", linestyle="--"),
         "balanced": dict(color=COLORS["hybrid"], marker="o", linestyle="-")}


def assign(world, policy, sinr, dist, th):
    """Tier per drone (TN=0, HAPS=1, SAT=2, -1 = no feasible tier). sinr, dist: (R, N, 3)."""
    p = world.p
    r, n, _ = sinr.shape
    fixed = {"all_tn": 0, "all_haps": 1, "all_sat": 2}
    if policy in fixed:  # fixed policies contend on their tier whether or not they are decodable
        return np.full((r, n), fixed[policy])
    feas = sinr > th
    tier = np.full((r, n), -1)
    if policy == "tn_first":  # keep up to n_ra_tn + margin drones on TN, then HAPS, then satellite
        order = np.argsort(-np.where(feas[..., 0], sinr[..., 0], -np.inf), axis=1, kind="stable")
        rank = np.argsort(order, axis=1, kind="stable")
        on_tn = feas[..., 0] & (rank < p.n_ra_tn + p.tn_cap_margin)
        tier[on_tn] = 0
        rest = ~on_tn
        tier[rest & feas[..., 1]] = 1
        tier[rest & ~feas[..., 1] & feas[..., 2]] = 2
        tier[rest & ~feas[..., 1] & ~feas[..., 2] & feas[..., 0]] = 0  # TN is its only option
        return tier
    # load-aware: drones in index order, each to the feasible tier with the lowest expected latency
    # given the drones already assigned there and the IoT load; an expected RA failure costs the limit
    static = world.latency_s(0.0, p.packet_mbit, world.shannon_rate(world.tier_bandwidths(), sinr), dist)
    n_ra = world.ra_resources()
    counts = np.zeros((r, 3))
    rows = np.arange(r)
    for i in range(n):
        e_acc, p_fail = world.expected_access(counts + 1, n_ra)
        cost = np.where(feas[:, i, :], (1 - p_fail) * (e_acc + static[:, i, :]) + p_fail * p.latency_limit_s,
                        np.inf)
        best = cost.argmin(1)
        ok = np.isfinite(cost[rows, best])
        tier[ok, i] = best[ok]
        counts[rows[ok], best[ok]] += 1
    return tier


def simulate(world, seed, n_grid, n_real, policies=POLICIES):
    """One seed. Per policy, arrays over n_grid:
    lat: mean latency (ms) over non-outage drones; out: outage fraction;
    no_coverage / ra_failure / over_limit: outage causes as fractions of all drones (mutually
    exclusive, checked in that order: no feasible tier or assigned-tier SINR below the access
    threshold; covered but random access gave up; accessed but latency over the limit);
    ra_fail (4, N): RA failures among covered drones per tier (TN, HAPS, SAT, all);
    acc_ms (4, N): mean access delay of successful accesses per tier; share (3, N): tier shares."""
    p = world.p
    rng = make_rng(seed, 3)
    th = world.access_th
    bw = world.tier_bandwidths()
    nn = len(n_grid)
    res = {k: {"lat": np.full(nn, np.nan), "out": np.zeros(nn), **{c: np.zeros(nn) for c in CAUSES},
               "ra_fail": np.full((4, nn), np.nan), "acc_ms": np.full((4, nn), np.nan), "share": np.zeros((3, nn))}
           for k in policies}
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
            tier = assign(world, pol, sinr, dist, th)
            t = np.maximum(tier, 0)
            s = np.take_along_axis(sinr, t[..., None], 2)[..., 0]
            d = np.take_along_axis(dist, t[..., None], 2)[..., 0]
            acc = world.random_access(tier, s > th, rng)
            latency = world.latency_s(acc, p.packet_mbit, world.shannon_rate(bw[t], s), d)
            no_cov = (tier < 0) | ~(s > th)
            ra_fail = ~no_cov & ~np.isfinite(acc)
            over = ~no_cov & ~ra_fail & (latency > p.latency_limit_s)
            ok = ~(no_cov | ra_fail | over)
            rp = res[pol]
            for c, m in zip(CAUSES, (no_cov, ra_fail, over)):
                rp[c][ni] = m.mean()
            rp["out"][ni] = 1 - ok.mean()
            if ok.any():
                rp["lat"][ni] = latency[ok].mean() * 1e3
            for k in range(4):
                sel = ~no_cov & (tier == k if k < 3 else True)
                if sel.any():
                    rp["ra_fail"][k, ni] = ra_fail[sel].mean()
                    good = sel & np.isfinite(acc)
                    if good.any():
                        rp["acc_ms"][k, ni] = acc[good].mean() * 1e3
                if k < 3:
                    rp["share"][k, ni] = (tier == k).mean()
    return res


def _collect(world, seeds, n_grid, n_real):
    runs = [simulate(world, s, n_grid, n_real) for s in seeds]
    return {k: {f: np.array([r[k][f] for r in runs]) for f in runs[0][k]} for k in POLICIES}


def _nanmean0(a):
    a = np.asarray(a, dtype=float)
    cnt = np.isfinite(a).sum(0)
    return np.where(cnt > 0, np.where(np.isfinite(a), a, 0.0).sum(0) / np.maximum(cnt, 1), np.nan)


def _spread_finite(v):
    v = np.asarray(v, dtype=float)
    return spread(v[np.isfinite(v)]) if np.isfinite(v).any() else None


def _plot(n_grid, res, path):
    fig, ax = plt.subplots(figsize=FIGSIZE)
    for k in POLICIES:
        band(ax, n_grid, res[k]["lat"], STYLE[k]["color"], f"{LABELS[k]} (outage {100 * res[k]['out'].mean():.1f}%)",
             **{a: b for a, b in STYLE[k].items() if a != "color"})
    ax.set_xlabel("active drones requesting uplink")
    ax.set_ylabel("mean end-to-end latency (ms)")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False)
    save(fig, path)


def _plot_causes(sweep, path):
    """Stacked outage-by-cause bars: one group per access threshold, one bar per policy."""
    fig, ax = plt.subplots(figsize=FIGSIZE)
    colors = {"no_coverage": "#bdbdbd", "ra_failure": "#d62728", "over_limit": "#8c564b"}
    names = {"no_coverage": "no coverage", "ra_failure": "random-access failure",
             "over_limit": "latency over limit"}
    w = 0.8 / len(POLICIES)
    ticks, ticklabels = [], []
    top = 0.0
    for gi, th in enumerate(sweep):
        for pi, k in enumerate(POLICIES):
            x = gi + (pi - (len(POLICIES) - 1) / 2) * w
            bottom = 0.0
            for c in CAUSES:
                v = 100 * sweep[th][k][c].mean()
                ax.bar(x, v, w * 0.9, bottom=bottom, color=colors[c], edgecolor=STYLE[k]["color"], linewidth=0.8,
                       label=names[c] if gi == 0 and pi == 0 else None)
                bottom += v
            top = max(top, bottom)
            ticks.append(x)
            ticklabels.append(SHORT[k])
    ax.set_xticks(ticks)
    ax.set_xticklabels(ticklabels, fontsize=5, rotation=90)
    for gi, th in enumerate(sweep):
        ax.annotate(f"threshold {th:+g} dB", (gi, -0.3), xycoords=("data", "axes fraction"), ha="center",
                    fontsize=6.5)
    ax.set_ylabel("outage, mean over N (%)")
    ax.set_ylim(0, max(5.0, 1.2 * top))
    ax.grid(axis="x", visible=False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.36), ncol=3, frameon=False)
    save(fig, path)


def _plot_iot(iot, field, scale, ylabel, path):
    """Per-tier RA metric at the largest N vs IoT load (fixed-tier policies) plus the balanced policy."""
    loads = list(IOT_LOADS.values())
    fig, ax = plt.subplots(figsize=FIGSIZE)
    for k, ti, lab in [("all_tn", 0, "TN (all on TN)"), ("all_haps", 1, "HAPS (all on HAPS)"),
                       ("all_sat", 2, "satellite (all on satellite)"), ("balanced", 3, "load-aware balanced")]:
        ys = np.array([iot[name][k][field][:, ti, -1] for name in IOT_LOADS]).T * scale
        band(ax, loads, ys, STYLE[k]["color"], lab, marker=STYLE[k]["marker"], linestyle=STYLE[k]["linestyle"])
    ax.set_xlabel("IoT RA load (preambles / resource / opportunity)")
    ax.set_ylabel(ylabel)
    ax.set_xticks(loads)
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    save(fig, path)


def _metrics(n_grid, res):
    big = n_grid[-1]
    out = {}
    for k in POLICIES:
        r = res[k]
        out[k] = {
            "latency_ms_curve_mean": _nanmean0(r["lat"]).tolist(),
            "outage_curve_mean": r["out"].mean(0).tolist(),
            f"latency_ms_at_N{n_grid[0]}": _spread_finite(r["lat"][:, 0]),
            f"latency_ms_at_N{big}": _spread_finite(r["lat"][:, -1]),
            "outage_mean_over_N": spread(r["out"].mean(1)),
            f"outage_at_N{big}": spread(r["out"][:, -1]),
            "outage_causes_mean_over_N": {c: spread(r[c].mean(1)) for c in CAUSES},
            f"outage_causes_at_N{big}": {c: spread(r[c][:, -1]) for c in CAUSES},
            f"per_tier_at_N{big}": {
                name: {"ra_failure_of_covered": _spread_finite(r["ra_fail"][:, ti, -1]),
                       "access_delay_ms": _spread_finite(r["acc_ms"][:, ti, -1]),
                       **({"share": spread(r["share"][:, ti, -1])} if ti < 3 else {})}
                for ti, name in enumerate(TIERS + ["all"])},
        }
    return out


def _log_block(log, n_grid, res, indent=""):
    for k in POLICIES:
        r = res[k]
        c = _nanmean0(r["lat"])
        split = ", ".join(f"{cn} {100 * r[cn].mean():.1f}%" for cn in CAUSES)
        log(f"{indent}{k:9s}: {c[0]:.1f} -> {c[-1]:.1f} ms (N={n_grid[0]}..{n_grid[-1]}), "
            f"outage mean {100 * r['out'].mean():.1f}% ({split}), at N={n_grid[-1]} {100 * r['out'][:, -1].mean():.1f}%")


def _log_tiers(log, n_grid, res, indent="  "):
    for k in POLICIES:
        r = res[k]
        parts = []
        for ti, name in enumerate(TIERS):
            share = r["share"][:, ti, -1].mean()
            if share == 0:
                continue
            rf = _nanmean0(r["ra_fail"][:, ti, -1:])[0]
            ad = _nanmean0(r["acc_ms"][:, ti, -1:])[0]
            parts.append(f"{name} share {100 * share:.0f}% RA-fail {100 * rf:.2f}% access {ad:.1f} ms")
        log(f"{indent}{k:9s} at N={n_grid[-1]}: " + "; ".join(parts))


def run(world, seeds, outdir, quick=False, log=print):
    n_grid, n_real = (QUICK_N_GRID, 10) if quick else (N_GRID, 100)
    res = _collect(world, seeds, n_grid, n_real)
    tn_curve = _nanmean0(res["all_tn"]["lat"])
    checks = {"all_tn_latency_non_decreasing": bool(np.all(np.diff(tn_curve) >= -0.05 * tn_curve[:-1]))}
    log("drone-only (no IoT load), nominal bandwidths and threshold:")
    _log_block(log, n_grid, res, indent="  ")
    _log_tiers(log, n_grid, res)
    _plot(n_grid, res, f"{outdir}/fig4_latency_vs_density.png")

    # same seeds and geometry, every tier at 5 MHz: separates contention from bandwidth effects
    eq = _collect(SAGINWorld(replace(world.p, b_tn_hz=5e6, b_haps_hz=5e6, b_sat_hz=5e6)), seeds, n_grid, n_real)
    log("equal 5 MHz bandwidth on every tier:")
    _log_block(log, n_grid, eq, indent="  ")
    _plot(n_grid, eq, f"{outdir}/fig4b_latency_equal_bw.png")

    # access-threshold sensitivity (nominal bandwidths, no IoT load)
    sweep = {th: res if th == world.p.access_th_db else
             _collect(SAGINWorld(replace(world.p, access_th_db=th)), seeds, n_grid, n_real) for th in SWEEP_TH_DB}
    for th in SWEEP_TH_DB:
        log(f"access threshold {th:+g} dB:")
        _log_block(log, n_grid, sweep[th], indent="  ")
    _plot_causes(sweep, f"{outdir}/fig4c_outage_causes.png")

    # massive-IoT background load on every tier's RA resources
    iot = {name: res if lam == world.p.iot_ra_load else
           _collect(SAGINWorld(replace(world.p, iot_ra_load=lam)), seeds, n_grid, n_real)
           for name, lam in IOT_LOADS.items()}
    for name, lam in IOT_LOADS.items():
        log(f"IoT load {name} ({lam:g} preambles/resource/opportunity):")
        _log_block(log, n_grid, iot[name], indent="  ")
        _log_tiers(log, n_grid, iot[name], indent="    ")
    _plot_iot(iot, "acc_ms", 1.0, f"mean access delay at N={n_grid[-1]} (ms)", f"{outdir}/fig4d_access_delay_vs_iot.png")
    _plot_iot(iot, "ra_fail", 100.0, f"RA failure at N={n_grid[-1]} (% of covered)",
              f"{outdir}/fig4e_ra_failure_vs_iot.png")

    # congested HAPS: same runs with the HAPS random-access capacity reduced
    iot16 = {name: _collect(SAGINWorld(replace(world.p, iot_ra_load=lam, n_ra_haps=HAPS_RA_REDUCED)), seeds,
                            n_grid, n_real) for name, lam in IOT_LOADS.items()}
    for name, lam in IOT_LOADS.items():
        log(f"HAPS RA resources {HAPS_RA_REDUCED}, IoT load {name} ({lam:g}):")
        _log_block(log, n_grid, iot16[name], indent="  ")
        _log_tiers(log, n_grid, iot16[name], indent="    ")

    for k, v in checks.items():
        log(f"check {k}: {v}")
    return {"n_grid": n_grid, "nominal_bw": _metrics(n_grid, res), "equal_5mhz_bw": _metrics(n_grid, eq),
            "threshold_sweep": {f"{th:+g}dB": _metrics(n_grid, v) for th, v in sweep.items()},
            "iot_load_sweep": {f"{name}_{lam:g}": _metrics(n_grid, iot[name]) for name, lam in IOT_LOADS.items()},
            f"haps_ra{HAPS_RA_REDUCED}_iot_load_sweep": {f"{name}_{lam:g}": _metrics(n_grid, iot16[name])
                                                         for name, lam in IOT_LOADS.items()},
            "checks": checks}
