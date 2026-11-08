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
# orthogonal sub-channel reuse on the sensor -> drone band (sensitivity, v1.2): (K, colouring)
REUSE_CASES = [(1, False), (4, False), (8, False), (16, False), (4, True), (8, True)]


def _best_latency(world, feasible, sinr, dist, bandwidth):
    """Lowest end-to-end latency over feasible tiers (one RA opportunity, no contention); inf if none."""
    p = world.p
    lat = world.latency_s(p.t_ra_s, p.packet_mbit, world.shannon_rate(bandwidth, sinr), dist)
    return np.where(feasible, lat, np.inf).min(-1)


def coverage(world, seed, density_km2, n_real, dist_km=DIST_KM, n_subch=1, colouring=False):
    """Coverage per curve at each distance for one seed and drone density, plus latency and
    sensor -> drone SINR samples. n_subch > 1 splits the sensor -> drone band into orthogonal
    sub-channels (random choice per drone, or nearest-neighbour colouring)."""
    p = world.p
    rng = make_rng(seed, 6, int(round(density_km2 * 100)))
    sensor = world.ground_sensor()
    th = world.access_th
    bw = world.tier_bandwidths()
    hits = {k: np.zeros(len(dist_km)) for k in CURVES}
    lat_via, lat_direct = [[] for _ in dist_km], [[] for _ in dist_km]
    hop1_db, conflicts, n_drones = [], 0, 0
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
            ch = None
            if n_subch > 1:
                ch, c = world.drone_channels(drones, n_subch, colouring, rng)
                conflicts += c
                n_drones += len(drones)
            g_sinr, srv, g_dist = world.gateway_sinr(xy, drones, rng, channels=ch, n_subch=n_subch)
            hop1_db.append(10 * np.log10(np.maximum(g_sinr, 1e-30)))
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
            hop1_lat = world.latency_s(p.t_ra_s, p.packet_mbit, world.shannon_rate(p.b_a2g_hz / n_subch, g_sinr),
                                       g_dist)
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
    out["hop1_sinr_db"] = np.concatenate(hop1_db) if hop1_db else np.zeros(0)
    out["colour_conflict_fraction"] = conflicts / n_drones if n_drones else 0.0
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
        res[name]["hop1_sinr_db"] = np.concatenate([r["hop1_sinr_db"] for r in runs])

    # density trend of via-drone coverage: binomial standard error of each pooled mean, no fixed floor
    n_samp = len(seeds) * n_real * len(DIST_KM)
    via_by_density = np.array([res[n]["via_drone"].mean() for n in DENSITY_NAMES])
    se = np.sqrt(np.maximum(via_by_density * (1 - via_by_density), 1.0 / n_samp) / n_samp)
    steps = np.diff(via_by_density)
    step_se = np.sqrt(se[1:] ** 2 + se[:-1] ** 2)
    trend = ["decreasing" if d < -3 * e else "increasing" if d > 3 * e else "flat (within 3 SE)"
             for d, e in zip(steps, step_se)]
    checks = {
        "combined_ge_components": True,  # asserted per realization in coverage()
        "via_drone_non_decreasing_in_density": bool(np.all(steps >= -3 * step_se)),
    }
    log("via-drone coverage vs density (mean over 0-10 km, pooled over seeds): " + ", ".join(
        f"{lam:g}/km^2 {v:.4f} (SE {e:.4f})" for lam, v, e in zip(p.gw_densities_km2, via_by_density, se))
        + "; steps: " + ", ".join(f"{d:+.4f} ({t})" for d, t in zip(steps, trend)))
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
        log(f"  median sensor -> drone SINR {np.median(r['hop1_sinr_db']):.1f} dB")
        o["hop1_sinr_db_median"] = float(np.median(r["hop1_sinr_db"]))
        o["drone_gain_over_hybrid"] = {f"at_{d:g}km": float(gain[_at(d)]) for d in REPORT_KM}
        o["no_haps_plus_drones_minus_hybrid"] = {f"at_{d:g}km": float(repl[_at(d)]) for d in REPORT_KM}
        o["latency_ms"] = {"via_drone_median": float(np.median(lv)) if len(lv) else None,
                           "via_drone_p95": float(np.percentile(lv, 95)) if len(lv) else None,
                           "direct_median": float(np.median(ld)) if len(ld) else None,
                           "direct_p95": float(np.percentile(ld, 95)) if len(ld) else None,
                           "via_drone_median_by_distance": lv_d}
        out[name] = o
    out["via_drone_density_trend"] = {"via_mean": via_by_density.tolist(), "se": se.tolist(),
                                      "steps": steps.tolist(), "trend": trend}
    out["reuse_sensitivity"] = reuse_sensitivity(world, seeds, 30 if quick else 150, outdir, log, res)
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


def reuse_sensitivity(world, seeds, n_real, outdir, log, nominal):
    """Sensor -> drone orthogonal reuse sweep (v1.2). K = 1 is taken from the main (v1.1) run."""
    p = world.p
    out = {}
    via_mean = {}
    log(f"orthogonal reuse of the {p.b_a2g_hz / 1e6:g} MHz sensor -> drone band (K sub-channels of "
        f"{p.b_a2g_hz / 1e3:g}/K kHz; {n_real} realizations per seed, K = 1 from the main run):")
    for k, colouring in REUSE_CASES:
        tag = f"K{k}_{'colouring' if colouring else 'random'}"
        out[tag] = {}
        for name, lam in zip(DENSITY_NAMES, p.gw_densities_km2):
            if k == 1:
                r = nominal[name]
                conflict = 0.0
            else:
                runs = [coverage(world, s, lam, n_real, n_subch=k, colouring=colouring) for s in seeds]
                r = {c: np.array([x[c] for x in runs]) for c in CURVES}
                r["lat_via"] = [np.concatenate([x["lat_via"][i] for x in runs]) for i in range(len(DIST_KM))]
                r["hop1_sinr_db"] = np.concatenate([x["hop1_sinr_db"] for x in runs])
                conflict = float(np.mean([x["colour_conflict_fraction"] for x in runs]))
            via = r["via_drone"].mean(0)
            gain = (r["combined"] - r["direct_hybrid"]).mean(0)
            repl = (r["combined_no_haps"] - r["direct_hybrid"]).mean(0)
            lv = np.concatenate(r["lat_via"]) * 1e3
            med = float(np.median(r["hop1_sinr_db"]))
            via_mean[(tag, name)] = r["via_drone"].mean(1)
            out[tag][name] = {
                **{f"via_drone_at_{d:g}km": spread(r["via_drone"][:, _at(d)]) for d in REPORT_KM},
                "via_drone_mean_over_distance": spread(r["via_drone"].mean(1)),
                "hop1_sinr_db_median": med,
                "drone_gain_over_hybrid": {f"at_{d:g}km": float(gain[_at(d)]) for d in REPORT_KM},
                "no_haps_plus_drones_minus_hybrid": {f"at_{d:g}km": float(repl[_at(d)]) for d in REPORT_KM},
                "via_latency_ms_median": float(np.median(lv)) if len(lv) else None,
                "colour_conflict_fraction": conflict,
            }
            log(f"  {tag:14s} {name:7s} ({lam:g}/km^2): via drone " + " / ".join(f"{via[_at(d)]:.3f}" for d in REPORT_KM)
                + f" at {' / '.join(f'{d:g}' for d in REPORT_KM)} km; hop-1 SINR median {med:.1f} dB; "
                f"on top of hybrid " + " / ".join(f"{gain[_at(d)]:+.3f}" for d in REPORT_KM)
                + "; no-HAPS+drones - hybrid " + " / ".join(f"{repl[_at(d)]:+.3f}" for d in REPORT_KM)
                + (f"; via latency median {np.median(lv):.1f} ms" if len(lv) else "; via latency: no covered samples")
                + (f"; colouring conflicts {100 * conflict:.2f}% of drones" if colouring else ""))

    fig, ax = plt.subplots(figsize=(3.5, 2.6))
    ks = [k for k, c in REUSE_CASES if not c]
    for name, col in zip(DENSITY_NAMES, [COLORS["drone"], COLORS["hybrid"], COLORS["ref"]]):
        lam = p.gw_densities_km2[DENSITY_NAMES.index(name)]
        ys = np.array([via_mean[(f"K{k}_random", name)] for k in ks]).T
        band(ax, ks, ys, col, f"{lam:g} drones/km², random reuse", marker="o")
        kc = [k for k, c in REUSE_CASES if c]
        yc = np.array([via_mean[(f"K{k}_colouring", name)] for k in kc]).T
        ax.errorbar(kc, yc.mean(0), yerr=[yc.mean(0) - yc.min(0), yc.max(0) - yc.mean(0)], color=col,
                    marker="s", linestyle="none", capsize=2, label=f"{lam:g} drones/km², NN colouring")
    ax.set_xscale("log", base=2)
    ax.set_xticks(ks)
    ax.set_xticklabels([str(k) for k in ks])
    ax.set_xlabel("sensor → drone sub-channels K (1 MHz / K each)")
    ax.set_ylabel("via-drone coverage, mean 0–10 km")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=1, frameon=False, fontsize=6)
    save(fig, f"{outdir}/fig3d_drone_gateway_reuse.png")
    return out
