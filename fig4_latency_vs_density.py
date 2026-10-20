import os
"""
fig4_latency_vs_density.py
===========================
Figure 4 (infrastructure inspection): end-to-end latency vs active drone
density, full per-policy curve family.

REVISION 2: now shows every single-tier policy alongside the balanced one,
with each policy's OUTAGE rate (drones stranded with no usable link) in
the legend. This exposes the real trade-off honestly:
  - all-TN:   lowest outage of the single tiers (~2%) but latency EXPLODES
              under crowding (12 NPRACH resources)
  - all-HAPS: fastest single tier (64 resources, 5 MHz) -- but strands
              ~10% of drones in outage (HAPS coverage is ~90%, not 100%)
  - all-Sat:  flat but slower (low SINR -> low rate) and strands ~53%
  - feasibility-aware balancing: the ONLY policy that is simultaneously
    fast AND near-zero outage. No single tier achieves both; the policy
    earns its place on the outage axis, not just the latency axis.

Per-tier access modeled as slotted random access (collision/backoff);
assignment via feasible_load_balance() checks REAL per-drone SINR.
Uses ONLY sagin_world.SAGINWorld -- no physics reimplemented here.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sagin_world import SAGINWorld, Params

# ---- experiment configuration ---------------------------------------------
N_VALUES = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
DIST_MIN_KM, DIST_MAX_KM = 0.3, 1.5
GAMMA_TH = 5.0
N_TRIALS = 1500
SEED = 20

w = SAGINWorld(seed=SEED)
TIER_RES = {"tn": Params.n_ra_resources_tn,
            "haps": Params.n_ra_resources_haps,
            "sat": Params.n_ra_resources_sat}


def one_trial(n, rng):
    distances = rng.uniform(DIST_MIN_KM, DIST_MAX_KM, size=n)
    tn_sinr = np.array([w.sinr_to_tn_db(d, rng=rng, n_trials=1)[0] for d in distances])
    haps_sinr = np.array([w.sinr_to_haps_db(d, sigma_b_deg=0.0, rng=rng, n_trials=1)[0] for d in distances])
    sat_sinr = np.array([w.sinr_to_sat_db(d, rng=rng, n_trials=1)[0] for d in distances])
    sinr_by_tier = {"tn": tn_sinr, "haps": haps_sinr, "sat": sat_sinr}

    out = {}
    # ---- single-tier policies: everyone on tier X ----
    for tier in ("tn", "haps", "sat"):
        lats = [w.random_access_latency_ms(max(n - 1, 0), TIER_RES[tier],
                                            distances[i], tier, rng=rng, n_trials=1)[0]
                for i in range(n)]
        out[tier] = (float(np.mean(lats)),
                     float((sinr_by_tier[tier] <= GAMMA_TH).mean()))  # (lat, outage)

    # ---- feasibility-aware balanced policy ----
    result = w.feasible_load_balance(tn_sinr, haps_sinr, sat_sinr,
                                      gamma_tn_db=GAMMA_TH, gamma_haps_db=GAMMA_TH,
                                      gamma_sat_db=GAMMA_TH)
    loads = result["loads"]
    assignments = result["assignments"]
    tier_names = {0: "tn", 1: "haps", 2: "sat"}
    lats = []
    for i in range(n):
        t = assignments[i]
        if t < 0:
            continue
        name = tier_names[t]
        lats.append(w.random_access_latency_ms(max(loads[name] - 1, 0), TIER_RES[name],
                                                distances[i], name, rng=rng, n_trials=1)[0])
    out["balanced"] = (float(np.mean(lats)) if lats else np.nan,
                       result["outage_count"] / n)
    return out


rng = np.random.default_rng(SEED)
policies = ["tn", "haps", "sat", "balanced"]
lat_mean = {p: [] for p in policies}
lat_sem = {p: [] for p in policies}
outage_mean = {p: [] for p in policies}

for n in N_VALUES:
    acc = {p: ([], []) for p in policies}
    for _ in range(N_TRIALS):
        r = one_trial(n, rng)
        for p in policies:
            acc[p][0].append(r[p][0])
            acc[p][1].append(r[p][1])
    for p in policies:
        lat_mean[p].append(np.nanmean(acc[p][0]))
        lat_sem[p].append(np.nanstd(acc[p][0]) / np.sqrt(N_TRIALS))
        outage_mean[p].append(np.mean(acc[p][1]))

# ---- sanity checks ----------------------------------------------------
tn_lat = np.array(lat_mean["tn"])
print(f"all-TN latency monotone non-decreasing: {np.all(np.diff(tn_lat) >= -1e-6)}")
print(f"all-TN:   {tn_lat[0]:.0f} -> {tn_lat[-1]:.0f} ms  (outage {100*outage_mean['tn'][-1]:.1f}%)")
print(f"all-HAPS: {lat_mean['haps'][0]:.0f} -> {lat_mean['haps'][-1]:.0f} ms  (outage {100*outage_mean['haps'][-1]:.1f}%)")
print(f"all-Sat:  {lat_mean['sat'][0]:.0f} -> {lat_mean['sat'][-1]:.0f} ms  (outage {100*outage_mean['sat'][-1]:.1f}%)")
print(f"balanced: {lat_mean['balanced'][0]:.0f} -> {lat_mean['balanced'][-1]:.0f} ms  (outage {100*outage_mean['balanced'][-1]:.1f}%)")
print("Trade-off check: no single tier is best on BOTH latency and outage; "
      "balanced should be near-best on both.")

# ---- plot ------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.6, 4.5))

styles = {
    "tn":       dict(color="#898781", linestyle="--", marker="s", lw=1.6,
                     label=f"All on TN (outage {100*outage_mean['tn'][-1]:.0f}%)"),
    "haps":     dict(color="#1baf7a", linestyle="-.", marker="D", lw=1.6,
                     label=f"All on HAPS (outage {100*outage_mean['haps'][-1]:.0f}%)"),
    "sat":      dict(color="#b07cc6", linestyle=":", marker="x", lw=1.6,
                     label=f"All on Satellite (outage {100*outage_mean['sat'][-1]:.0f}%)"),
    "balanced": dict(color="#2a78d6", linestyle="-", marker="o", lw=2.4,
                     label=f"Feasibility-aware balanced (outage {100*outage_mean['balanced'][-1]:.1f}%)"),
}
for p in policies:
    ax.errorbar(N_VALUES, lat_mean[p], yerr=lat_sem[p], markersize=4,
                capsize=2, **styles[p], zorder=5 if p == "balanced" else 3)

ax.set_xlabel("active drones")
ax.set_ylabel("end-to-end latency (ms)")
ax.legend(frameon=False, loc="upper left", fontsize=8)
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(os.path.join(os.environ.get("SAGIN_OUT","results"),"fig4_latency_vs_density.png"), dpi=200)
print("Saved fig4_latency_vs_density.png")
