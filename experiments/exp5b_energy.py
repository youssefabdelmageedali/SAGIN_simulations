"""Exp 5 (Fig. 5b): remaining battery vs. mission time for raw offload vs. edge-AI."""
import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread

T_MIN = np.linspace(0, 80, 801)
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


def run(world, seeds, outdir, quick=False, log=print):
    p = world.p
    n_drones = 50 if quick else 500
    bw = {"haps": p.b_haps_hz, "sat": p.b_sat_hz}
    t_s = T_MIN * 60
    curves = {k: [] for k in STRATEGIES}
    life = {k: [] for k in STRATEGIES}
    raw_airtime = {"haps": [], "sat": []}
    for seed in seeds:
        rng = make_rng(seed, 5)
        sinr = {"haps": np.zeros(n_drones), "sat": np.zeros(n_drones)}
        for i in range(n_drones):
            xy = world.swarm_in_mission(1, rng)
            dep = world.deploy(rng)
            sinr["haps"][i] = world.haps_sinr(dep, xy, 0.0, rng)[0][0]
            sinr["sat"][i] = world.sat_sinr(dep, xy, rng)[0][0]
        rate = {k: world.shannon_rate(bw[k], sinr[k]) for k in bw}
        for k in raw_airtime:
            raw_airtime[k].append(np.median(p.raw_mbit * 1e6 / rate[k]))
        for k, (payload, tier, compute) in STRATEGIES.items():
            e = world.cycle_energy_j(getattr(p, payload), rate[tier], compute)
            curves[k].append(np.median(world.battery_percent(t_s, e), axis=0))
            life[k].append(np.median(world.endurance_s(e)) / 60)

    ceiling_min = world.flight_ceiling_s() / 60
    ceiling = np.clip(100 * (1 - T_MIN / ceiling_min), 0, 100)
    checks = {"below_flight_ceiling": bool(all(np.all(np.array(c) <= ceiling + 1e-9) for c in curves.values()))}
    for k in STRATEGIES:
        log(f"{LABELS[k]:20s} median endurance {np.mean(life[k]):.1f} min "
            f"(seed range {min(life[k]):.1f}..{max(life[k]):.1f})")
    log(f"flight-only ceiling {ceiling_min:.1f} min")
    log(f"median airtime to upload one raw capture: HAPS {np.mean(raw_airtime['haps']):.1f} s, "
        f"satellite {np.mean(raw_airtime['sat']):.1f} s (capture interval {p.capture_interval_s:g} s)")
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
    ax.legend(loc="upper right")
    save(fig, f"{outdir}/fig5b_battery_vs_time.png")

    return {
        **{f"{k}_endurance_min": spread(v) for k, v in life.items()},
        "flight_ceiling_min": ceiling_min,
        "raw_upload_airtime_s": {k: spread(v) for k, v in raw_airtime.items()},
        "checks": checks,
    }
