import os
"""
fig3_coverage_vs_distance.py
============================
Figure 3 (agriculture): coverage probability vs distance from the hub,
full per-tier curve family.

REVISION: now shows each tier ALONE (TN, HAPS ideal, HAPS with pointing
error, satellite) alongside the two hybrid curves, so the reader sees WHY
each individual tier fails and what the hybrid inherits from each:
  - TN alone: cliffs at ~2-3 km (path-loss + threshold)
  - HAPS alone (ideal): high (~90%) and nearly flat -- but not perfect
    (aerial shadowing dips below threshold ~9% of the time)
  - HAPS alone (sigma_b=3deg): drops to ~63% -- pointing error costs a
    standalone HAPS ~28 percentage points, far more visible than on the
    hybrid curve where TN/satellite partially mask it. THE thesis link.
  - Satellite alone: ~47%, a genuine last resort, not a reliable primary
  - Hybrid: inherits the best of all three at every distance

Uses ONLY sagin_world.SAGINWorld -- no physics reimplemented here.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sagin_world import SAGINWorld, Params

# ---- experiment configuration ---------------------------------------------
DISTANCES_KM = np.linspace(0.0, 10.0, 21)
GAMMA_TH_DB = 5.0            # access threshold (see earlier tuning notes:
                              # chosen so satellite is a genuine last resort)
SIGMA_B_ERROR_DEG = 3.0      # [PLACEHOLDER] representative pointing-error std
N_TRIALS = 8000
SEED = 7

w = SAGINWorld(seed=SEED)

tn_cov, haps_ideal_cov, haps_err_cov, sat_cov = [], [], [], []
hybrid_ideal_cov, hybrid_err_cov = [], []

for d in DISTANCES_KM:
    tn = w.sinr_to_tn_db(d, n_trials=N_TRIALS)
    h0 = w.sinr_to_haps_db(d, sigma_b_deg=0.0, n_trials=N_TRIALS)
    h_err = w.sinr_to_haps_db(d, sigma_b_deg=SIGMA_B_ERROR_DEG, n_trials=N_TRIALS)
    st = w.sinr_to_sat_db(d, n_trials=N_TRIALS)

    tn_ok = tn > GAMMA_TH_DB
    h0_ok = h0 > GAMMA_TH_DB
    herr_ok = h_err > GAMMA_TH_DB
    st_ok = st > GAMMA_TH_DB

    tn_cov.append(tn_ok.mean())
    haps_ideal_cov.append(h0_ok.mean())
    haps_err_cov.append(herr_ok.mean())
    sat_cov.append(st_ok.mean())
    hybrid_ideal_cov.append((tn_ok | h0_ok | st_ok).mean())
    hybrid_err_cov.append((tn_ok | herr_ok | st_ok).mean())

tn_cov = np.array(tn_cov)
haps_ideal_cov = np.array(haps_ideal_cov)
haps_err_cov = np.array(haps_err_cov)
sat_cov = np.array(sat_cov)
hybrid_ideal_cov = np.array(hybrid_ideal_cov)
hybrid_err_cov = np.array(hybrid_err_cov)

# ---- sanity checks ----------------------------------------------------
print(f"TN-only monotone non-increasing: {np.all(np.diff(tn_cov) <= 1e-9)}")
print(f"TN-only: {tn_cov[0]:.3f} at 0km -> {tn_cov[-1]:.3f} at 10km")
print(f"HAPS alone (ideal): ~{haps_ideal_cov.mean():.3f} (near-flat)")
print(f"HAPS alone (sigma_b={SIGMA_B_ERROR_DEG}deg): ~{haps_err_cov.mean():.3f} "
      f"-- pointing error costs a standalone HAPS "
      f"{100*(haps_ideal_cov.mean()-haps_err_cov.mean()):.0f}pp")
print(f"Satellite alone: ~{sat_cov.mean():.3f} (genuine last resort)")
print(f"Hybrid (ideal): {hybrid_ideal_cov[0]:.3f} at 0km -> {hybrid_ideal_cov[-1]:.3f} at 10km")
print(f"Hybrid (err):   {hybrid_err_cov[0]:.3f} at 0km -> {hybrid_err_cov[-1]:.3f} at 10km")
# hybrid must dominate every individual tier at every distance
dom = np.all(hybrid_ideal_cov >= np.maximum.reduce([tn_cov, haps_ideal_cov, sat_cov]) - 1e-9)
print(f"Hybrid(ideal) >= max(individual tiers) at every distance: {dom}")

# ---- plot ------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.6, 4.5))

# hybrids (thick, the headline)
ax.plot(DISTANCES_KM, hybrid_ideal_cov, color="#2a78d6", linewidth=2.4,
        marker="o", markersize=4, label="Hybrid (best of 3), ideal HAPS", zorder=5)
ax.plot(DISTANCES_KM, hybrid_err_cov, color="#eda100", linewidth=2.4,
        marker="^", markersize=4,
        label=f"Hybrid, HAPS $\\sigma_b$={SIGMA_B_ERROR_DEG}$^\\circ$", zorder=5)

# individual tiers (thinner, the explanation)
ax.plot(DISTANCES_KM, haps_ideal_cov, color="#1baf7a", linewidth=1.4,
        linestyle="-.", marker="D", markersize=3, label="HAPS alone, ideal")
ax.plot(DISTANCES_KM, haps_err_cov, color="#1baf7a", linewidth=1.4,
        linestyle=":", marker="v", markersize=3,
        label=f"HAPS alone, $\\sigma_b$={SIGMA_B_ERROR_DEG}$^\\circ$")
ax.plot(DISTANCES_KM, sat_cov, color="#b07cc6", linewidth=1.4,
        linestyle="-.", marker="x", markersize=4, label="Satellite alone")
ax.plot(DISTANCES_KM, tn_cov, color="#898781", linewidth=1.4,
        linestyle="--", marker="s", markersize=3, label="TN alone")

ax.set_xlabel("distance from hub (km)")
ax.set_ylabel("coverage probability")
ax.set_ylim(-0.02, 1.05)
ax.legend(frameon=False, loc="center right", fontsize=8)
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(os.path.join(os.environ.get("SAGIN_OUT","results"),"fig3_coverage_vs_distance.png"), dpi=200)
print("Saved fig3_coverage_vs_distance.png")
