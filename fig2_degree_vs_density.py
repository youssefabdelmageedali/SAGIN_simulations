import os
"""
fig2_degree_vs_density.py
=========================
Figure 2 (graph properties, "the spine"): mean node degree vs drone density,
SG-derived graph vs a naive distance-based rule.

This is the figure that PLOTS the paper's central claim: the communication
graph is derived from stochastic geometry, not assumed from distance. The
SG-derived curve rises then rolls over as swarm interference destroys edges;
the distance rule keeps climbing without bound.

REVISION: drones are now placed with drop_drone_clusters() -- a real
Thomas Poisson Cluster Process (mission-oriented sub-groups) -- instead of
uniform placement (drop_drones). This matches the architecture text's
long-standing claim that "the aerial swarm is modeled as a PCP," which the
simulation had not actually implemented until now. Density is controlled
via the number of clusters; realized drone count is a Poisson random
variable (reported, not fixed) -- standard practice for cluster-process
density sweeps.

Uses ONLY sagin_world.SAGINWorld -- no physics reimplemented here.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sagin_world import SAGINWorld, Params

# ---- experiment configuration ---------------------------------------------
N_CLUSTERS_VALUES = [1, 2, 3, 5, 8, 12, 18, 25, 40, 60, 90, 130]
AREA_SIDE_KM = 2.0          # swarm cluster region
GAMMA_TH_DB = -5.0          # consistent with the paper's default threshold
P_ACTIVE = 0.5              # fraction of drones transmitting per snapshot
D_CONN_M = 400.0            # distance-rule connection radius (metres)
N_TRIALS = 500              # Monte Carlo trials per density point
SEED = 42

w = SAGINWorld(seed=SEED)

sg_mean, sg_sem = [], []
dist_mean, dist_sem = [], []
avg_n_drones = []

for n_clusters in N_CLUSTERS_VALUES:
    sg_degs, dist_degs, n_drones_this = [], [], []
    trial = 0
    while trial < N_TRIALS:
        result = w.drop_drone_clusters(n_clusters=n_clusters, area_side_km=AREA_SIDE_KM)
        pos = result["positions"]
        if len(pos) < 2:
            continue  # degenerate draw (Poisson gave near-zero drones); redraw
        g = w.build_graph(pos, gamma_th_db=GAMMA_TH_DB, p_active=P_ACTIVE)
        d = w.build_graph_distance_rule(pos, d_conn_m=D_CONN_M,
                                         p_active=P_ACTIVE,
                                         active_mask=g["active"])
        sg_degs.append(g["mean_degree"])
        dist_degs.append(d["mean_degree"])
        n_drones_this.append(len(pos))
        trial += 1

    sg_mean.append(np.mean(sg_degs))
    sg_sem.append(np.std(sg_degs) / np.sqrt(N_TRIALS))
    dist_mean.append(np.mean(dist_degs))
    dist_sem.append(np.std(dist_degs) / np.sqrt(N_TRIALS))
    avg_n_drones.append(np.mean(n_drones_this))

sg_mean = np.array(sg_mean); sg_sem = np.array(sg_sem)
dist_mean = np.array(dist_mean); dist_sem = np.array(dist_sem)
avg_n_drones = np.array(avg_n_drones)

# Density for the x-axis (realized average drones per km^2)
density = avg_n_drones / (AREA_SIDE_KM ** 2)

# ---- sanity check: report the peak (for the manuscript TODO) -------------
peak_idx = np.argmax(sg_mean)
print(f"SG-derived degree peaks at avg N={avg_n_drones[peak_idx]:.0f} "
      f"(density={density[peak_idx]:.1f}/km^2), value={sg_mean[peak_idx]:.3f}")
print(f"SG-derived degree at highest density: {sg_mean[-1]:.3f} "
      f"({100*(1-sg_mean[-1]/sg_mean[peak_idx]):.0f}% below peak)")
sg_declining = np.all(np.diff(sg_mean[peak_idx:]) <= 1e-6)
print(f"SG-derived degree monotone non-increasing after peak: {sg_declining}")

# ---- plot ------------------------------------------------------------------
fig, ax1 = plt.subplots(figsize=(6.4, 4.4))
ax2 = ax1.twinx()

l1 = ax1.errorbar(density, sg_mean, yerr=sg_sem, color="#2a78d6", linewidth=2,
                   marker="o", markersize=4, capsize=2,
                   label="SG-derived (SINR > $\\gamma_{th}$)")
l2 = ax2.errorbar(density, dist_mean, yerr=dist_sem, color="#898781", linewidth=2,
                   marker="s", markersize=4, capsize=2, linestyle="--",
                   label="Naive distance rule")

ax1.axvline(density[peak_idx], color="#c9c7bd", linewidth=1.0, linestyle=":", zorder=0)
ax1.annotate(f"rollover (peak at {density[peak_idx]:.0f}/km$^2$)",
             xy=(density[peak_idx], sg_mean[peak_idx]),
             xytext=(density[peak_idx] + 60, sg_mean[peak_idx] + 0.6),
             fontsize=9, color="#52514e",
             arrowprops=dict(arrowstyle="->", color="#52514e", lw=0.8))

ax1.set_xlabel("drone density (drones / km$^2$, PCP mission clusters)")
ax1.set_ylabel("mean node degree — SG-derived", color="#2a78d6")
ax2.set_ylabel("mean node degree — distance rule", color="#898781")
ax1.tick_params(axis="y", labelcolor="#2a78d6")
ax2.tick_params(axis="y", labelcolor="#898781")
ax1.set_ylim(0, max(sg_mean) * 1.6)
ax2.set_ylim(0, max(dist_mean) * 1.1)

lines = [l1, l2]
ax1.legend(lines, [l.get_label() for l in lines], frameon=False, loc="upper center")
ax1.spines["top"].set_visible(False)
ax2.spines["top"].set_visible(False)
ax1.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(os.path.join(os.environ.get("SAGIN_OUT","results"),"fig2_degree_vs_density.png"), dpi=200)
print("Saved fig2_degree_vs_density.png")
