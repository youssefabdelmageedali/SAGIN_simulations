"""Exp 1 (Fig. 2): mean node degree vs. drone density, SG-derived vs. naive distance rule."""
import numpy as np
import matplotlib.pyplot as plt

from experiments.common import COLORS, FIGSIZE, band, make_rng, save, spread

DENSITIES = [1, 2, 4, 6, 8, 10, 15, 20, 30, 50, 80, 120, 180, 250]  # drones / km^2
QUICK_DENSITIES = [1, 10, 50, 250]


def window_m(density_km2, n_target):
    """Torus side: about n_target drones per realization, between 3 and 10 km."""
    return float(np.clip(np.sqrt(n_target / density_km2) * 1e3, 3e3, 10e3))


def run(world, seeds, outdir, quick=False, log=print):
    dens = QUICK_DENSITIES if quick else DENSITIES
    n_target, reps = (300, 1) if quick else (2000, 3)
    sg = np.zeros((len(seeds), len(dens)))
    naive = np.zeros_like(sg)
    r0s = []
    for si, seed in enumerate(seeds):
        rng = make_rng(seed, 1)
        r0 = None
        for di, lam in enumerate(dens):
            side = window_m(lam, n_target)
            deg_sg, dists, edge_len = [], [], []
            for _ in range(reps):
                xy = world.thomas_pcp(lam, side, rng)
                if len(xy) < 2:
                    continue
                adj = world.sg_graph(world.mesh_sinr(xy, rng, side_m=side))
                d = world.pairwise_dist(xy, side)
                deg_sg.append(adj.sum(1).mean())
                dists.append(d)
                edge_len.append(d[adj])
            if r0 is None:  # lowest density: naive range = median SG edge length
                r0 = float(np.median(np.concatenate(edge_len)))
                r0s.append(r0)
            deg_nv = [((d < r0).sum(1) - 1).mean() for d in dists]
            sg[si, di] = np.mean(deg_sg)
            naive[si, di] = np.mean(deg_nv)

    mean_sg = sg.mean(0)
    ipk = int(mean_sg.argmax())
    seed_peaks = sg.max(1)
    seed_peak_dens = [dens[i] for i in sg.argmax(1)]
    drop = 1 - mean_sg[-1] / mean_sg[ipk]
    checks = {
        "sg_peaks_before_max_density": bool(ipk < len(dens) - 1),
        "naive_monotone_non_decreasing": bool(np.all(np.diff(naive.mean(0)) >= -1e-9)),
    }
    log(f"SG-derived degree peaks at density {dens[ipk]}/km^2, value {mean_sg[ipk]:.3f} "
        f"(seed peaks {seed_peaks.min():.3f}..{seed_peaks.max():.3f} at {seed_peak_dens})")
    log(f"SG-derived degree at {dens[-1]}/km^2: {mean_sg[-1]:.3f} ({100 * drop:.1f}% below peak)")
    log(f"Naive rule r0 = {np.mean(r0s):.1f} m (seed range {min(r0s):.1f}..{max(r0s):.1f}); "
        f"naive degree {naive.mean(0)[0]:.3f} -> {naive.mean(0)[-1]:.3f}")
    for k, v in checks.items():
        log(f"check {k}: {v}")

    fig, ax = plt.subplots(figsize=FIGSIZE)
    h1 = band(ax, dens, sg, COLORS["drone"], r"SG-derived (SINR > $\gamma_{th}$)", marker="o")
    ax.set_xscale("log")
    ax.set_xlabel(r"drone density (drones/km$^2$)")
    ax.set_ylabel("mean node degree, SG-derived")
    ax.set_ylim(bottom=0)
    ax2 = ax.twinx()
    ax2.grid(False)
    h2 = band(ax2, dens, naive, COLORS["ref"], f"distance rule (r0 = {np.mean(r0s):.0f} m)",
              linestyle="--", marker="s")
    ax2.set_ylabel("mean node degree, distance rule")
    ax2.set_yscale("log")
    ax.legend(handles=[h1, h2], loc="upper left")
    save(fig, f"{outdir}/fig2_degree_vs_density.png")

    return {
        "densities_per_km2": dens,
        "sg_degree_mean": mean_sg.tolist(),
        "naive_degree_mean": naive.mean(0).tolist(),
        "sg_peak_density_per_km2": dens[ipk],
        "sg_peak_degree": spread(sg[:, ipk]),
        "sg_degree_at_max_density": spread(sg[:, -1]),
        "sg_drop_from_peak_pct": float(100 * drop),
        "naive_r0_m": spread(r0s),
        "naive_degree_at_max_density": spread(naive[:, -1]),
        "checks": checks,
    }
