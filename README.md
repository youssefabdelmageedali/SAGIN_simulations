# SAGIN stochastic-geometry simulation

Simulation for "A Unified SAGIN Architecture for Autonomous Drone Swarms and Massive IoT".
Central claim under test: the communication graph is **derived from SINR**
(edge i→j iff SINR_ij > γ_th), not assumed from distance.

## Run

```bash
pip install -r requirements.txt
python run_all.py              # all experiments, 5 seeds, ~minutes on a laptop
python run_all.py exp3         # one experiment (fig names work too: python run_all.py fig4)
python -m pytest -q tests      # sanity tests
```

Outputs in `results/`: one PNG per figure (300 dpi, no titles), `run_log.txt` (every
number and sanity check as printed), `metrics.json` (every headline number with
mean/min/max over seeds, the full `Params`, and the placeholder list). Same seeds →
bit-identical `metrics.json` (tested).

## Layout

| Path | Role |
|---|---|
| `sagin/world.py` | the only physics engine: `Params`, `SAGINWorld` (geometry, path loss, fading, SINR, random access, latency, compute, energy) |
| `experiments/exp1_graph.py` | Exp 1 → `fig2_degree_vs_density.png` |
| `experiments/exp2_coverage.py` | Exp 2 → `fig3_coverage_vs_distance.png` (includes no-HAPS ablation) |
| `experiments/exp3_latency.py` | Exp 3 → `fig4_latency_vs_density.png`, `fig4b_latency_equal_bw.png` |
| `experiments/exp5a_compute.py` | Exp 4 → `fig5a_completion_vs_complexity.png` |
| `experiments/exp5b_energy.py` | Exp 5 → `fig5b_battery_vs_time.png` |
| `experiments/common.py` | plotting style, colours, seeding (no physics) |
| `run_all.py` | runs everything, writes log + metrics |
| `tests/test_world.py` | the 8 sanity tests |

## Model

- **Geometry.** 10 × 10 km region, hub at the origin. Drones at 100 m, HAPS at 20 km, LEO at 550 km.
  TN base stations: PPP inside a 1.5 km urban core around the hub. Drone swarm: Thomas cluster
  process. HAPS: BPP over the region. Satellites: spherical BPP (n_sat uniform on the orbital
  shell, serving = max elevation, ≥ 10°).
- **SINR.** SINR = P G |h|² PL⁻¹ / (I + N0), N0 = −174 dBm/Hz + 10log10(B) + 7 dB.
  TN: 128.1 + 37.6 log10(d_km); mesh: 40 + 25 log10(d_m); HAPS/sat: free space at 20 GHz, 3-D distance.
  Nakagami-m (m = 2) on TN and mesh; 4 dB log-normal shadowing on HAPS/sat.
- **Interference.** Mesh: every drone transmits at once on one of 8 sub-channels; all others on the
  same sub-channel interfere. TN uplink: one co-channel IoT user per other cell. HAPS: the co-channel
  user of each other HAPS, through the serving beam's off-axis gain. Satellite: 6 co-channel users of
  other beams at the sidelobe level.
- **HAPS beam.** Gaussian main lobe, 3 dB width 5°, sidelobe floor −30 dB; 1-D Gaussian pointing error σ_b.
- **Random access.** Slotted, N_RA resources per tier, success iff alone and decodable (SINR > access
  threshold); binary exponential back-off capped at 64 opportunities, 10 attempts max, all drones
  start at opportunity 0.
- **Latency** = access + packet / (B log2(1+SINR)) + 3-D distance / c. Outage = no feasible tier,
  RA failure, or latency > 1 s.
- **Compute.** Solo completes iff size ≤ 470 GFLOPS × 1 s. Cooperative: owner + up to 5 SG-graph
  out-neighbours (best SINR); each helper first receives 0.2 Mbit at its mesh Shannon rate, then
  computes until the deadline.
- **Energy** per 10 s cycle = 120 W × 10 s + 8 W × payload / rate (+ 10 W × 10 s with on-board compute).
- **Decision policy.** The balanced latency policy is a non-learned feasibility-aware rule. No
  Graph-MARL policy is implemented or claimed.

## Parameters

| Quantity | Value | Source |
|---|---|---|
| Carrier TN / HAPS / sat | 2 / 20 / 20 GHz | assumption |
| Drone Tx power access / mesh | 20 / 10 dBm | Lou, Belmekki, Alouini, arXiv:2310.09659 |
| Antenna gain drone / TN / HAPS / sat | 3 / 15 / 25 / 48 dBi | Lou et al. (drone 3 dBi); others tuned |
| Nakagami m | 2 | Lou et al. |
| Bandwidth TN / HAPS / sat / mesh | 1 / 5 / 1.5 / 20 MHz | HAPS ≈ HIBS 5 MHz (Alexandre 2021); sat ≈ IoT-NTN Cat M1 1.4 MHz (Jamshed 2025) |
| RA resources TN / HAPS / sat | 12 / 64 / 64 | TN = NB-IoT NPRACH (Lin 2016); 64 = **PLACEHOLDER** |
| RA opportunity length | 6.4 ms | NPRACH format 1 |
| Graph threshold γ_th | −5 dB | **PLACEHOLDER** |
| Access threshold | +5 dB | **PLACEHOLDER** |
| Drone compute | 470 GFLOPS | Jetson-Nano class (Hakani & Rawat 2024) |
| Task deadline | 1 s | **PLACEHOLDER** |
| Battery | 148 Wh | ≈ DJI Matrice 4D 149.9 Wh |
| Flight / compute / Tx power | 120 / 10 / 8 W | flight, compute: literature-based; Tx **PLACEHOLDER** |
| Payload raw / packet / report | 800 / 0.5 / 0.02 Mbit | **PLACEHOLDERS** |
| Capture interval | 10 s | **PLACEHOLDER** |
| Shadowing std (HAPS/sat) | 4 dB | **PLACEHOLDER** |
| HAPS beamwidth / sidelobe floor / σ_b | 5° / −30 dB / 3° | **PLACEHOLDERS** (σ_b = 3° from the spec) |
| BS height / IoT interferer power | 25 m / 20 dBm | **PLACEHOLDERS** |
| TN core radius / BS density / interferer radius | 1.5 km / 3 per km² / 300 m | **PLACEHOLDERS** |
| HAPS count / satellites / min elevation / sat co-channel users | 3 / 720 / 10° / 6 | **PLACEHOLDERS** |
| PCP mean cluster size / spread | 10 / 100 m | **PLACEHOLDERS** |
| Mission disk radius / spread (exp3, exp5) | 2 km / 200 m | **PLACEHOLDERS** |
| Exp4 swarm density / torus window | 10 per km² / 3 km | **PLACEHOLDERS** |
| Back-off cap / max RA attempts / latency limit / TN cap margin | 2⁶ / 10 / 1 s / +1 | **PLACEHOLDERS** |
| Helpers / chunk overhead | 5 / 0.2 Mbit | spec; **PLACEHOLDERS** |

The machine-readable list is `PLACEHOLDERS` in `sagin/world.py`, copied into `metrics.json`.

## Experiment-specific choices

- Exp 1 runs on a torus window (no edge effects) of side clip(√(2000/λ) km, 3, 10), 3 realizations
  per density per seed. r0 = median SG edge length at the lowest density (per seed).
- Exp 3 and Exp 5 use ideal HAPS pointing (σ_b = 0); Exp 2 shows the cost of σ_b = 3°.
- Exp 3 equal-bandwidth variant sets all tiers to 5 MHz (noise scales with B too).
- Every stochastic experiment: seeds 1–5, curves show the seed mean with a min–max band.

## Results

See `results/run_log.txt` and `results/metrics.json`. The summary of the latest run is in
the section below.

### Latest run (seeds 1–5, seed mean)

| Metric | Value |
|---|---|
| Exp 1 SG degree peak | 8.132 at 4 drones/km²; 6.048 at 250/km² (25.6% below peak) |
| Exp 1 distance rule | r0 = 150.1 m; degree 4.293 → 22.329 |
| Exp 2 TN alone | 0.362 at hub, peaks 0.43 at 1 km, < 0.01 from 2.5 km |
| Exp 2 HAPS alone ideal / σ_b = 3° | 0.316 / 0.158 (mean over distance) |
| Exp 2 satellite alone | 0.158 |
| Exp 2 hybrid at 10 km ideal / σ_b = 3° / no HAPS | 0.408 / 0.282 / 0.158 |
| Exp 3 latency N=1→50, outage (mean over N) | TN 152.7→210.7 ms, 72.6%; HAPS 44.0→52.9 ms, 68.7%; sat 135.8→146.3 ms, 83.8%; balanced 107.8→104.3 ms, 41.7% |
| Exp 4 coop completion at 1000 / 2000 GFLOP | 97.9% / 89.3% (solo: step at 470 GFLOP) |
| Exp 5 median endurance | raw→sat 18.2 min, raw→HAPS 43.7 min, edge-AI 68.3 min (ceiling 74.0 min) |

### Known sanity-check failures (not fixed, reported as found)

- `tn_non_increasing` (test 1): TN coverage rises from 0.362 at the hub to ~0.43 at 1 km before
  collapsing, because base stations exist only inside the 1.5 km core and edge cells have fewer
  co-channel interferers. A modelling consequence of the TN-deployment placeholder, not a code bug.
- `naive_monotone_non_decreasing` (exp1 only): one dip 4.682 → 4.574 between 4 and 6 drones/km²,
  Monte Carlo noise at low density (60-realization re-check: 4.53 ± 0.04 → 4.72 ± 0.04).
