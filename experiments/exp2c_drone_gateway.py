"""Exp 2c (Fig. 3c): drones as aerial IoT gateways for ground sensors.

Two-hop path: ground sensor -> nearest drone of a Thomas-cluster swarm (TR 36.777 A2G channel,
co-channel sensors of other drones) -> drone backhaul to TN, HAPS or satellite with the drone radio.
Compared with the direct sensor links of exp2b, at three drone densities."""
import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, band, make_rng, save, spread
from experiments.exp2_coverage import DIST_KM

REPORT_KM = [0.0, 2.0, 5.0, 10.0]
CURVES = ["direct_hybrid", "direct_no_haps", "via_drone", "via_drone_no_haps", "combined", "combined_no_haps"]
LABELS = {"direct_hybrid": "direct, hybrid (TN+HAPS+sat)", "direct_no_haps": "direct, TN + sat (no HAPS)",
          "via_drone": "via drone only", "via_drone_no_haps": "via drone, TN/sat backhaul",
          "combined": "direct + via drone (hybrid)", "combined_no_haps": "direct + via drone, no HAPS"}
STYLE = {"direct_hybrid": (COLORS["hybrid"], "-", "o"), "direct_no_haps": (COLORS["hybrid"], ":", None),
         "via_drone": (COLORS["drone"], "-", "^"), "via_drone_no_haps": (COLORS["drone"], ":", None),
         "combined": (COLORS["ref"], "-", "s"), "combined_no_haps": (COLORS["ref"], "--", None)}
DENSITY_NAMES = ["low", "nominal", "high"]


def _best_latency(world, feasible, sinr, dist, bandwidth):
    """Lowest end-to-end latency over feasible tiers (one RA opportunity, no contention); inf if none."""
    p = world.p
    lat = world.latency_s(p.t_ra_s, p.packet_mbit, world.shannon_rate(bandwidth, sinr), dist)
    return np.where(feasible, lat, np.inf).min(-1)


def coverage(world, seed, density_km2, n_real, dist_km=DIST_KM):
    """Coverage per curve at each distance for one seed and drone density, plus latency samples."""
    p = world.p
    rng = make_rng(seed, 6, int(round(density_km2 * 100)))
    sensor = world.ground_sensor()
    th = world.access_th
    bw = world.tier_bandwidths()
    hits = {k: np.zeros(len(dist_km)) for k in CURVES}
    lat_via, lat_direct = [[] for _ in dist_km], [[] for _ in dist_km]
    for _ in range(n_real):
        dep = world.deploy(rng)
        az = rng.uniform(0, 2 * np.pi)
        xy = np.column_stack([dist_km * 1e3 * np.cos(az), dist_km * 1e3 * np.sin(az)])
        # direct sensor links (exp2b terminal, ideal HAPS pointing)
        d_sinr = np.zeros((len(xy), 3))
        d_dist = np.zeros((len(xy), 3))
        d_sinr[:, 0], d_dist[:, 0] = sensor.tn_sinr(dep, xy, rng)
        d_sinr[:, 1], d_dist[:, 1] = sensor.haps_sinr(dep, xy, 0.0, rng)
        d_sinr[:, 2], d_dist[:, 2] = sensor.sat_sinr(dep, xy, rng)
        d_ok = d_sinr > th
        # two-hop path via the nearest drone
        drones = world.swarm_disk(density_km2, p.gw_swarm_radius_km * 1e3, rng)
        if len(drones):
            g_sinr, srv, g_dist = world.gateway_sinr(xy, drones, rng)
            dxy = drones[srv]
            b_sinr = np.zeros((len(xy), 3))
            b_dist = np.zeros((len(xy), 3))
            b_sinr[:, 0], b_dist[:, 0] = world.tn_sinr(dep, dxy, rng)
            b_sinr[:, 1], b_dist[:, 1] = world.haps_sinr(dep, dxy, 0.0, rng)
            b_sinr[:, 2], b_dist[:, 2] = world.sat_sinr(dep, dxy, rng)
            hop1 = g_sinr > th
            b_ok = b_sinr > th
        else:
            hop1 = np.zeros(len(xy), dtype=bool)
            b_ok = np.zeros((len(xy), 3), dtype=bool)
        cov = {
            "direct_hybrid": d_ok.any(1),
            "direct_no_haps": d_ok[:, 0] | d_ok[:, 2],
            "via_drone": hop1 & b_ok.any(1),
            "via_drone_no_haps": hop1 & (b_ok[:, 0] | b_ok[:, 2]),
        }
        cov["combined"] = cov["direct_hybrid"] | cov["via_drone"]
        cov["combined_no_haps"] = cov["direct_no_haps"] | cov["via_drone_no_haps"]
        assert np.all(cov["combined"] >= cov["direct_hybrid"]) and np.all(cov["combined"] >= cov["via_drone"])
        assert np.all(cov["combined_no_haps"] >= cov["direct_no_haps"])
        assert np.all(cov["combined_no_haps"] >= cov["via_drone_no_haps"])
        for k in CURVES:
            hits[k] += cov[k]
        # latency: direct = best feasible tier; via drone = sensor->drone hop + best feasible backhaul tier
        ld = _best_latency(sensor, d_ok, d_sinr, d_dist, bw)
        if len(drones):
            hop1_lat = world.latency_s(p.t_ra_s, p.packet_mbit, world.shannon_rate(p.b_a2g_hz, g_sinr), g_dist)
            lv = np.where(hop1, hop1_lat + _best_latency(world, b_ok, b_sinr, b_dist, bw), np.inf)
        else:
            lv = np.full(len(xy), np.inf)
        for i in range(len(xy)):
            if np.isfinite(lv[i]):
                lat_via[i].append(lv[i])
            if np.isfinite(ld[i]):
                lat_direct[i].append(ld[i])
    out = {k: v / n_real for k, v in hits.items()}
    out["lat_via"], out["lat_direct"] = lat_via, lat_direct
    return out


def _at(km):
    return int(np.argmin(np.abs(DIST_KM - km)))


def run(world, seeds, outdir, quick=False, log=print):
    p = world.p
    n_real = 60 if quick else 500
    res = {}
    for name, lam in zip(DENSITY_NAMES, p.gw_densities_km2):
        runs = [coverage(world, s, lam, n_real) for s in seeds]
        res[name] = {k: np.array([r[k] for r in runs]) for k in CURVES}
        res[name]["lat_via"] = [np.concatenate([r["lat_via"][i] for r in runs]) for i in range(len(DIST_KM))]
        res[name]["lat_direct"] = [np.concatenate([r["lat_direct"][i] for r in runs]) for i in range(len(DIST_KM))]

    via_by_density = np.array([res[n]["via_drone"].mean() for n in DENSITY_NAMES])
    checks = {
        "combined_ge_components": True,  # asserted per realization in coverage()
        "via_drone_non_decreasing_in_density": bool(np.all(np.diff(via_by_density) >= -0.01)),
    }
    out = {"distances_km": DIST_KM.tolist(), "densities_km2": list(p.gw_densities_km2), "checks": checks}
    log(f"sensor -> nearest drone (TR 36.777 UMa-AV in the core, RMa-AV outside; {p.b_a2g_hz / 1e6:g} MHz; "
        f"co-channel activity {world.p_act:.3f}); backhaul with the drone radio (30 dBm, 10 dBi)")
    for name, lam in zip(DENSITY_NAMES, p.gw_densities_km2):
        r = res[name]
        log(f"drone density {name} ({lam:g}/km^2): coverage at {' / '.join(f'{d:g}' for d in REPORT_KM)} km")
        o = {}
        for k in CURVES:
            m = r[k].mean(0)
            log(f"  {k:18s}: " + " / ".join(f"{m[_at(d)]:.3f}" for d in REPORT_KM) + f"   (mean {m.mean():.3f})")
            o[f"{k}_curve_mean"] = m.tolist()
            o[f"{k}_mean_over_distance"] = spread(r[k].mean(1))
            for d in REPORT_KM:
                o[f"{k}_at_{d:g}km"] = spread(r[k][:, _at(d)])
        gain = (r["combined"] - r["direct_hybrid"]).mean(0)
        repl = (r["combined_no_haps"] - r["direct_hybrid"]).mean(0)
        log(f"  drones on top of hybrid (combined - direct hybrid): "
            + " / ".join(f"{gain[_at(d)]:+.3f}" for d in REPORT_KM))
        log(f"  no-HAPS + drones vs hybrid (combined_no_haps - direct hybrid): "
            + " / ".join(f"{repl[_at(d)]:+.3f}" for d in REPORT_KM))
        lv = np.concatenate(r["lat_via"]) * 1e3
        ld = np.concatenate(r["lat_direct"]) * 1e3
        lv_d = {f"at_{d:g}km": float(np.median(r["lat_via"][_at(d)]) * 1e3) if len(r["lat_via"][_at(d)]) else None
                for d in REPORT_KM}
        log(f"  latency median (covered samples): via drone {np.median(lv):.1f} ms (p95 {np.percentile(lv, 95):.1f}), "
            f"direct best tier {np.median(ld):.1f} ms (p95 {np.percentile(ld, 95):.1f})" if len(lv) and len(ld) else
            "  latency: no covered samples")
        o["drone_gain_over_hybrid"] = {f"at_{d:g}km": float(gain[_at(d)]) for d in REPORT_KM}
        o["no_haps_plus_drones_minus_hybrid"] = {f"at_{d:g}km": float(repl[_at(d)]) for d in REPORT_KM}
        o["latency_ms"] = {"via_drone_median": float(np.median(lv)) if len(lv) else None,
                           "via_drone_p95": float(np.percentile(lv, 95)) if len(lv) else None,
                           "direct_median": float(np.median(ld)) if len(ld) else None,
                           "direct_p95": float(np.percentile(ld, 95)) if len(ld) else None,
                           "via_drone_median_by_distance": lv_d}
        out[name] = o
    for k, v in checks.items():
        log(f"check {k}: {v}")

    fig, axes = plt.subplots(1, 3, figsize=(9.0, 2.8), sharey=True)
    for ax, name, lam in zip(axes, DENSITY_NAMES, p.gw_densities_km2):
        for k in CURVES:
            col, ls, mk = STYLE[k]
            band(ax, DIST_KM, res[name][k], col, LABELS[k], linestyle=ls, marker=mk, markevery=2)
        ax.set_xlabel("sensor distance from hub (km)")
        ax.set_ylim(-0.02, 1.02)
        ax.text(0.02, 0.03, f"({'abc'[DENSITY_NAMES.index(name)]}) {lam:g} drones/km²", transform=ax.transAxes,
                fontsize=8, fontweight="bold")
    axes[0].set_ylabel(f"coverage Pr[SINR > {p.access_th_db:g} dB]")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=3, frameon=False)
    save(fig, f"{outdir}/fig3c_drone_gateway.png")
    return out
