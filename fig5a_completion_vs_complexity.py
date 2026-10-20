import os
"""
fig5a_completion_vs_complexity.py
==================================
Figure 5(a) (public safety): task completion success vs. task complexity.

Demonstrates that cooperation -- bounded by the ACTUAL SG-derived graph
(a task can only be split to neighbours the owner has a real edge to, per
SS IV.B "graph-bounded cooperation") -- clears deadlines a lone drone cannot.

REVISION: the swarm is now placed with drop_drone_clusters() -- a real PCP
(mission-oriented sub-groups) -- instead of uniform placement, matching
the architecture text's long-standing claim. Swarm size is now a Poisson
random variable around the original ~30 drones (reported per trial, not
fixed).

Uses ONLY sagin_world.SAGINWorld -- no physics reimplemented here.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sagin_world import SAGINWorld, Params

# ---- experiment configuration ---------------------------------------------
G_VALUES = [100, 200, 300, 400, 470, 500, 600, 700, 800, 1000, 1500, 2000]
N_CLUSTERS = 4          # ~4 x 8 = 32 drones on average, close to the
                         # original fixed N_DRONES=30
AREA_SIDE_KM = 2.0
GAMMA_TH_DB = -5.0
P_ACTIVE = 0.5
N_TRIALS = 2000
SEED = 11

w = SAGINWorld(seed=SEED)

solo_rate, coop_rate = [], []
avg_n_drones = []

for g in G_VALUES:
    solo_results, coop_results, n_drones_this = [], [], []
    trial = 0
    while trial < N_TRIALS:
        result = w.drop_drone_clusters(n_clusters=N_CLUSTERS, area_side_km=AREA_SIDE_KM)
        pos = result["positions"]
        if len(pos) < 2:
            continue  # degenerate draw; redraw
        s, c = w.task_split_outcome(pos, task_gflops=g, gamma_th_db=GAMMA_TH_DB,
                                     p_active=P_ACTIVE)
        if s is not None:
            solo_results.append(s)
            coop_results.append(c)
            n_drones_this.append(len(pos))
        trial += 1
    solo_rate.append(np.mean(solo_results) * 100)
    coop_rate.append(np.mean(coop_results) * 100)
    avg_n_drones.append(np.mean(n_drones_this))

solo_rate = np.array(solo_rate)
coop_rate = np.array(coop_rate)
avg_n_drones = np.array(avg_n_drones)

# ---- sanity checks ----------------------------------------------------
print(f"Average swarm size across trials: {avg_n_drones.mean():.1f} drones "
      f"(Poisson-distributed via PCP, target ~{N_CLUSTERS*Params.pcp_mean_drones_per_cluster:.0f})")
print(f"Solo completion: {solo_rate[0]:.1f}% at G={G_VALUES[0]} -> "
      f"{solo_rate[-1]:.1f}% at G={G_VALUES[-1]}")
print(f"Coop completion: {coop_rate[0]:.1f}% at G={G_VALUES[0]} -> "
      f"{coop_rate[-1]:.1f}% at G={G_VALUES[-1]}")
solo_monotone = np.all(np.diff(solo_rate) <= 1e-6)
print(f"Solo monotone non-increasing in complexity: {solo_monotone}")

# ---- plot ------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.2, 4.3))

ax.plot(G_VALUES, coop_rate, color="#1baf7a", linewidth=2, marker="o",
        markersize=4, label=f"Graph-bounded split (up to {Params.max_helpers} neighbours)")
ax.plot(G_VALUES, solo_rate, color="#898781", linewidth=2, linestyle="--",
        marker="s", markersize=4, label="Solo (lone drone)")

ax.set_xlabel("task complexity (GFLOPS)")
ax.set_ylabel("task completion success (%)")
ax.set_ylim(-2, 102)
ax.legend(frameon=False, loc="center left")
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(os.path.join(os.environ.get("SAGIN_OUT","results"),"fig5a_completion_vs_complexity.png"), dpi=200)
print("Saved fig5a_completion_vs_complexity.png")
