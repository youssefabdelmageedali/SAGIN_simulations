"""Exp 5 (Fig. 5b): remaining battery vs. mission time for raw offload vs. edge-AI,
plus raw-streaming feasibility (required vs. achievable rate) and the resulting backlog."""
import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread

T_MIN = np.linspace(0, 80, 801)
TIERS = ["tn", "haps", "sat"]
# strategy -> (payload field, tier, on-board compute)
STRATEGIES = {
    "raw_sat": ("raw_mbit", "sat", False),
    "raw_haps": ("raw_mbit", "haps", False),
    "edge_sat": ("report_mbit", "sat", True),
    "edge_haps": ("report_mbit", "haps", True),
}
LABELS = {"raw_sat": "raw → satellite", "raw_haps": "raw → HAPS",
          "edge_sat": "edge-AI → satellite", "edge_haps": "edge-AI → HAPS"}
STYLE = {"raw_sat": dict(color=COLORS["sat"], linestyle="--"),
         "raw_haps": dict(color=COLORS["haps"], linestyle="--"),
         "edge_sat": dict(color=COLORS["drone"], linestyle=":"),
         "edge_haps": dict(color=COLORS["drone"], linestyle="-")}
TIER_LABEL = {"tn": "TN", "haps": "HAPS", "sat": "satellite"}


def run(world, seeds, outdir, quick=False, log=print):
    p = world.p
    n_drones = 50 if quick else 500
    bw = {"tn": p.b_tn_hz, "haps": p.b_haps_hz, "sat": p.b_sat_hz}
    t_s = T_MIN * 60
    req_raw = world.required_rate_bps(p.raw_mbit)
    req_rep = world.required_rate_bps(p.report_mbit)
    curves = {k: [] for k in STRATEGIES}
    life = {k: [] for k in STRATEGIES}
    rate_med = {k: [] for k in TIERS}
    rate_p95 = {k: [] for k in TIERS}
    feas_raw = {k: [] for k in TIERS}
    feas_rep = {k: [] for k in TIERS}
    delivered = {k: [] for k in TIERS}
    backlog = {k: [] for k in TIERS}
    for seed in seeds:
        rng = make_rng(seed, 5)
        sinr = {k: np.zeros(n_drones) for k in TIERS}
        for i in range(n_drones):
            xy = world.swarm_in_mission(1, rng)
            dep = world.deploy(rng)
            sinr["haps"][i] = world.haps_sinr(dep, xy, 0.0, rng)[0][0]
            sinr["sat"][i] = world.sat_sinr(dep, xy, rng)[0][0]
            sinr["tn"][i] = world.tn_sinr(dep, xy, rng)[0][0]
        rate = {k: world.shannon_rate(bw[k], sinr[k]) for k in TIERS}
        for k in TIERS:
            rate_med[k].append(np.median(rate[k]))
            rate_p95[k].append(np.percentile(rate[k], 95))
            feas_raw[k].append(np.mean(rate[k] >= req_raw))
            feas_rep[k].append(np.mean(rate[k] >= req_rep))
            delivered[k].append(np.median(np.minimum(rate[k] / req_raw, 1.0)))
            backlog[k].append(np.median(world.backlog_mbit(t_s, p.raw_mbit, rate[k]), axis=0) / 1e3)  # Gbit
        for k, (payload, tier, compute) in STRATEGIES.items():
            e = world.cycle_energy_j(getattr(p, payload), rate[tier], compute)
            curves[k].append(np.median(world.battery_percent(t_s, e), axis=0))
            life[k].append(np.median(world.endurance_s(e)) / 60)

    ceiling_min = world.flight_ceiling_s() / 60
    ceiling = np.clip(100 * (1 - T_MIN / ceiling_min), 0, 100)
    checks = {"below_flight_ceiling": bool(all(np.all(np.array(c) <= ceiling + 1e-9) for c in curves.values()))}
    log(f"raw streaming needs {req_raw / 1e6:.1f} Mbit/s ({p.raw_mbit:g} Mbit every {p.capture_interval_s:g} s); "
        f"edge-AI report needs {req_rep / 1e3:.1f} kbit/s")
    for k in TIERS:
        log(f"  {TIER_LABEL[k]:9s} achievable rate median {np.mean(rate_med[k]) / 1e6:.2f} Mbit/s "
            f"(95th pct {np.mean(rate_p95[k]) / 1e6:.2f}); raw feasible for {100 * np.mean(feas_raw[k]):.1f}% "
            f"of drones; median fraction of raw delivered {100 * np.mean(delivered[k]):.1f}%; "
            f"median backlog after {T_MIN[-1]:g} min {np.mean([b[-1] for b in backlog[k]]):.1f} Gbit; "
            f"report feasible for {100 * np.mean(feas_rep[k]):.1f}%")
    for k in STRATEGIES:
        log(f"{LABELS[k]:20s} median endurance {np.mean(life[k]):.1f} min "
            f"(seed range {min(life[k]):.1f}..{max(life[k]):.1f})")
    log(f"flight-only ceiling {ceiling_min:.1f} min")
    for k, v in checks.items():
        log(f"check {k}: {v}")
    assert checks["below_flight_ceiling"], "a battery curve exceeds the flight-only ceiling"

    fig, ax = plt.subplots(figsize=FIGSIZE)
    for k in ["edge_haps", "edge_sat", "raw_haps", "raw_sat"]:
        band(ax, T_MIN, curves[k], STYLE[k]["color"], LABELS[k], linestyle=STYLE[k]["linestyle"])
    ax.plot(T_MIN, ceiling, color=COLORS["ref"], linewidth=0.8, label=f"flight-only ceiling ({ceiling_min:.0f} min)")
    ax.set_xlabel("mission time (min)")
    ax.set_ylabel("remaining battery (%)")
    ax.set_ylim(-2, 102)
    ax.legend(loc="lower left")
    save(fig, f"{outdir}/fig5b_battery_vs_time.png")

    fig, ax = plt.subplots(figsize=FIGSIZE)
    for k, ls in zip(TIERS, ["-", "--", ":"]):
        band(ax, T_MIN, backlog[k], COLORS[k], f"raw → {TIER_LABEL[k]} "
             f"({100 * np.mean(delivered[k]):.0f}% of raw delivered)", linestyle=ls)
    ax.set_xlabel("mission time (min)")
    ax.set_ylabel("unsent raw backlog (Gbit)")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    save(fig, f"{outdir}/fig5c_raw_backlog.png")

    return {
        **{f"{k}_endurance_min": spread(v) for k, v in life.items()},
        "flight_ceiling_min": ceiling_min,
        "raw_required_rate_mbps": req_raw / 1e6,
        "report_required_rate_kbps": req_rep / 1e3,
        "achievable_rate_median_mbps": {k: spread(np.array(v) / 1e6) for k, v in rate_med.items()},
        "achievable_rate_p95_mbps": {k: spread(np.array(v) / 1e6) for k, v in rate_p95.items()},
        "raw_feasible_fraction": {k: spread(v) for k, v in feas_raw.items()},
        "raw_delivered_fraction_median": {k: spread(v) for k, v in delivered.items()},
        "raw_backlog_gbit_at_end": {k: spread([b[-1] for b in v]) for k, v in backlog.items()},
        "report_feasible_fraction": {k: spread(v) for k, v in feas_rep.items()},
        "checks": checks,
    }
