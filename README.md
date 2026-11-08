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
| `experiments/exp2_coverage.py` | Exp 2 → `fig3_coverage_vs_distance.png` (incl. no-HAPS ablation), `fig3b_coverage_threshold_sweep.png` |
| `experiments/exp3_latency.py` | Exp 3 → `fig4_latency_vs_density.png`, `fig4b_latency_equal_bw.png`, `fig4c_outage_causes.png`, `fig4d_access_delay_vs_iot.png`, `fig4e_ra_failure_vs_iot.png` |
| `experiments/exp5a_compute.py` | Exp 4 → `fig5a_completion_vs_complexity.png` |
| `experiments/exp5b_energy.py` | Exp 5 → `fig5b_battery_vs_time.png`, `fig5c_raw_backlog.png`, `fig5d_bits_per_joule.png` |
| `experiments/common.py` | plotting style, colours, seeding (no physics) |
| `run_all.py` | runs everything, writes log + metrics |
| `tests/test_world.py` | the 8 spec sanity tests + 3GPP calibration + IoT-load test |

## Model

- **Geometry.** 10 × 10 km region, hub at the origin. Drones at 100 m, HAPS at 20 km, LEO at 600 km.
  TN base stations: homogeneous PPP over the whole region (0.2 /km²). Drone swarm: Thomas cluster
  process. HAPS: BPP over the region. Satellites: spherical BPP (n_sat uniform on the orbital
  shell, serving = max elevation, ≥ 10°).
- **SINR.** SINR = P G |h|² PL⁻¹ / (I + N). TN, mesh, HAPS receivers: N = −174 dBm/Hz + 10log10(B) + 7 dB.
  Satellite receiver: N = k·T·B with T = G_rx / (G/T) (G/T = 1.1 dB/K, G_rx = 30 dBi → T = 776 K).
  TN: 128.1 + 37.6 log10(d_km); mesh: 40 + 25 log10(d_m); HAPS: free space at 20 GHz; satellite: free
  space at 2 GHz plus 5.3 dB fixed losses; 3-D distance. Nakagami-m (m = 2) on TN and mesh; 4 dB
  log-normal shadowing on HAPS/sat.
- **Interference.** Mesh: every drone transmits at once on one of 8 sub-channels; all others on the
  same sub-channel interfere. TN uplink: one co-channel IoT user per other cell. HAPS: the co-channel
  user of each other HAPS, through the serving beam's off-axis gain. Satellite: 6 co-channel users of
  other beams at the sidelobe level.
- **HAPS beam.** Gaussian main lobe, 3 dB width 5°, sidelobe floor −30 dB; 1-D Gaussian pointing error σ_b.
- **Random access.** Slotted, N_RA resources per tier, success iff decodable (SINR > access threshold)
  and alone on the resource: no other drone and no IoT preamble. Massive-IoT background: active ground
  IoT devices (a thinned PPP) pick resources uniformly, so each resource carries an independent
  Poisson(`iot_ra_load`) number of IoT preambles per opportunity, on every tier. Binary exponential
  back-off capped at 64 opportunities, 10 attempts max, all drones start at opportunity 0.
- **Latency** = access + packet / (B log2(1+SINR)) + 3-D distance / c. Outage = no feasible tier,
  RA failure, or latency > 1 s.
- **Compute.** Solo completes iff size ≤ 470 GFLOPS × 1 s. Cooperative: owner + up to 5 SG-graph
  out-neighbours (best SINR); each helper first receives 0.2 Mbit at its mesh Shannon rate, then
  computes until the deadline.
- **Energy** per 10 s cycle = 120 W × 10 s + 8 W × airtime (+ 10 W × 10 s with on-board compute), where
  airtime = min(payload / rate, 10 s). If the required rate payload / 10 s exceeds the achievable rate the
  radio transmits the whole cycle and the excess joins a backlog queue (fluid model, `backlog_mbit`).
- **Decision policies** (non-learned rules; no Graph-MARL policy is implemented or claimed).
  *Load-aware balanced*: drones in index order, each to the feasible tier with the lowest expected
  latency = (1 − P_fail)(E[access | success] + transmission + propagation) + P_fail × 1 s, where the
  access terms come from `expected_access`: per-attempt success exp(−(n−1)/N_RA − iot_ra_load) with n
  the drones already on that tier plus this one, and the same back-off and attempt limit. This is
  conservative (it assumes every assigned drone keeps contending). *Terrestrial-first* (baseline,
  the previous "balanced" rule): up to N_RA,TN + 1 best-TN drones on TN, the rest to HAPS, then
  satellite, only to a feasible tier.

## Parameters

| Quantity | Value | Source |
|---|---|---|
| Carrier TN / HAPS / sat | 2 / 20 / 2 GHz | TN, HAPS: assumption; sat: S-band, 3GPP TR 38.821 Table 6.1.1.1-1, TR 36.763 Table 6.1-1 |
| Drone Tx power access / mesh | 30 / 10 dBm | access: Lou, Belmekki, Alouini, arXiv:2310.09659, Table I (UAV); mesh unchanged |
| Antenna gain drone / TN / HAPS | 10 / 15 / 30 dBi | drone, HAPS: Lou et al. Table I; TN tuned |
| Satellite (LEO-600, Set-1, S-band) altitude / Rx max gain / G/T | 600 km / 30 dBi / 1.1 dB/K | 3GPP TR 38.821 v16.2.0 Table 6.1.1.1-1 |
| Satellite losses | 5.3 dB = scintillation 2.2 + atmospheric 0.1 + polarization 3 | 3GPP TR 36.763 v17.0.0 Table 6.2-1 (LEO 600 km) |
| Satellite min elevation | 10° | 3GPP TR 36.763 Table 6.1-1 |
| Nakagami m | 2 | Lou et al. |
| Bandwidth TN / HAPS / sat / mesh | 1 / 5 / 1.08 / 20 MHz | HAPS ≈ HIBS 5 MHz (Alexandre 2021); sat: eMTC UL up to 1080 kHz, 3GPP TR 36.763 Table 6.1-1 |
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
| TN BS density / interferer radius | 0.2 per km² (equivalent cell radius 1.26 km) / 300 m | **PLACEHOLDERS** |
| HAPS count / satellites / sat co-channel users | 3 / 720 / 6 | **PLACEHOLDERS** |
| IoT RA load (preambles / resource / opportunity) | 0 (drone-only); swept 0.1 / 0.5 / 1.0 in exp3 | **PLACEHOLDERS** |
| PCP mean cluster size / spread | 10 / 100 m | **PLACEHOLDERS** |
| Mission disk radius / spread (exp3, exp5) | 2 km / 200 m | **PLACEHOLDERS** |
| Exp4 swarm density / torus window | 10 per km² / 3 km | **PLACEHOLDERS** |
| Back-off cap / max RA attempts / latency limit / TN cap margin | 2⁶ / 10 / 1 s / +1 | **PLACEHOLDERS** |
| Helpers / chunk overhead | 5 / 0.2 Mbit | spec; **PLACEHOLDERS** |

The machine-readable list is `PLACEHOLDERS` in `sagin/world.py`, copied into `metrics.json`.

## Changes in revision 2 (and why)

1. **UAV link-budget values.** Drone access Tx power 20 → 30 dBm and drone antenna gain 3 → 10 dBi
   (the paper's drones are UAVs; these are the UAV values in Lou et al. Table I). HAPS gain 25 → 30 dBi
   and satellite gain 48 → 50 dBi from the same table. Nothing else was changed to hit targets.
   Side effect: the drone gain is used at both ends of mesh links, so mesh links gain 14 dB too
   (exp1 and exp4 change); mesh Tx power stays 10 dBm.
2. **TN deployment.** The old 1.5 km "urban core" made TN coverage *rise* away from the hub (edge
   cells had fewer co-channel interferers) and failed the monotonicity test. Base stations are now a
   homogeneous PPP over the whole 10 × 10 km region at 0.2 /km², i.e. an equivalent cell radius
   1/√(πλ) = 1.26 km, inside the requested 1–1.5 km. Coverage is flat (≈ 0.89) up to ~3.5 km and falls
   only towards and beyond the region edge (5 km); it is **not** measurably highest at the hub.
   The co-channel interferer placement (300 m around each BS) was left unchanged.
3. **Raw-payload feasibility.** Raw streaming needs 800 Mbit / 10 s = 80 Mbit/s. The old energy model
   charged P_tx × payload / rate even when that exceeded the 10 s cycle (104 s on HAPS, 461 s on
   satellite), which is physically impossible. Now airtime is capped at the cycle and the shortfall is
   modelled as a growing backlog queue; exp5 reports required vs achievable rate per tier, the fraction
   of drones for which raw streaming is feasible, the fraction of raw data delivered and the backlog
   (`fig5c_raw_backlog.png`).
4. **Access-threshold sweep** (−5, 0, +5 dB) for exp2 (`fig3b_coverage_threshold_sweep.png`) and exp3
   (`fig4c_outage_causes.png`), same seeds and geometry, only the threshold changes.
5. **Outage causes** in exp3, mutually exclusive and checked in order: no coverage (no feasible tier,
   or assigned-tier SINR below the access threshold), random-access failure (covered but gave up after
   10 attempts), latency over the 1 s limit.

## Changes in revision 3 (and why)

1. **Satellite link budget from 3GPP NTN (LEO-600, S-band, Set-1).** All values were read from the
   documents themselves (3GPP FTP archive: `38821-g20.zip` = TR 38.821 v16.2.0, `36763-h00.zip` =
   TR 36.763 v17.0.0):
   - altitude 600 km, carrier 2 GHz, satellite Rx max gain 30 dBi, G/T 1.1 dB/K — TR 38.821
     Table 6.1.1.1-1 (Set-1, LEO-600, S-band UL);
   - uplink bandwidth 1.08 MHz — TR 36.763 Table 6.1-1, eMTC "up to 1080 kHz" (UL). The 30 MHz NR
     maximum per beam (TR 38.821 Table 6.1.1.1-5) is a beam capacity, not a device channel, and is not used;
   - losses 2.2 (scintillation) + 0.1 (atmospheric) + 3 (polarization) dB — TR 36.763 Table 6.2-1;
   - minimum elevation 10° — TR 36.763 Table 6.1-1.
   Noise is k·T·B with T = G_rx/(G/T) = 776 K. The UAV terminal keeps 30 dBm / 10 dBi (Lou et al.
   Table I). The formula is checked against TR 36.763 §6.2.2.1.1 case 3 (LEO-600 Set-1, 23 dBm, 0 dBi,
   FSPL 159.7 dB, all listed losses + 3 dB shadow margin + 3 dB beam-edge loss): −8.64 dB vs −8.6 dB
   in the TR at 1080 kHz, 16.0 dB at 3.75 kHz (test `test_sat_cn_matches_3gpp_calibration`).
   Not taken from 3GPP, kept as PLACEHOLDERS: the 3 dB **shadow margin** of TR 36.763 is not added
   (the 4 dB log-normal shadowing is drawn instead); the 3 dB **beam-edge loss** is not applied (the
   swarm spans a few km inside a 50 km beam, so it sits near beam centre); constellation size 720,
   −30 dB sidelobe floor and 6 co-channel users are unchanged placeholders.
2. **Massive-IoT background load** (`iot_ra_load`, IoT preambles per RA resource per opportunity, all
   tiers). Exp 3 sweeps none / low 0.1 / medium 0.5 / high 1.0 and reports RA failure and access
   delay per tier (`fig4d`, `fig4e`). Load values are placeholders, chosen to span light load up to the
   slotted-ALOHA throughput peak (1 preamble per resource).
3. **Load-aware balanced policy** replaces the terrestrial-first rule, which is kept as the named
   baseline `tn_first` (see Model).
4. **Useful data per joule** in exp5 for raw and edge-AI on TN, HAPS and satellite: delivered bits per
   cycle (min(payload, rate × 10 s), the rest is backlog) / cycle energy. Raw and report bits are not the
   same information, so `captures_per_kj` (fraction of one capture's information delivered per cycle,
   per kJ) is reported alongside.

Graph, compute and energy models are otherwise unchanged.

## Experiment-specific choices

- Exp 1 runs on a torus window (no edge effects) of side clip(√(2000/λ) km, 3, 10), 3 realizations
  per density per seed. r0 = median SG edge length at the lowest density (per seed).
- Exp 3 and Exp 5 use ideal HAPS pointing (σ_b = 0); Exp 2 shows the cost of σ_b = 3°.
- Exp 3 equal-bandwidth variant sets all tiers to 5 MHz (noise scales with B too).
- Every stochastic experiment: seeds 1–5, curves show the seed mean with a min–max band.

## Results

See `results/run_log.txt` and `results/metrics.json`. The summary of the latest run is in
the section below.

### Latest run (revision 3; seeds 1–5, seed mean)

| Metric | Value |
|---|---|
| Exp 1 SG degree peak | 8.762 at 4 drones/km²; 6.061 at 250/km² (30.8% below peak) |
| Exp 1 distance rule | r0 = 179.0 m; degree 5.481 → 31.229 |
| Exp 2 TN alone | 0.889 at hub, ≈ 0.89 to 3.5 km, below 0.5 from 7 km, 0.019 at 10 km |
| Exp 2 HAPS alone ideal / σ_b = 3° | 0.954 / 0.880 (mean over distance) |
| Exp 2 satellite alone (3GPP LEO-600 S-band) | 0.996 |
| Exp 2 hybrid at 10 km ideal / σ_b = 3° / no HAPS | 1.000 / 1.000 / 0.996 |
| Satellite median uplink rate (exp5) | 5.85 Mbit/s over 1.08 MHz (TN 6.53, HAPS 33.29) |
| Exp 3 latency N=1→50, outage (drone-only) | TN 92.8→146.5 ms, 11.2%; HAPS 22.7→30.3 ms, 8.8%; sat 101.2→108.1 ms, 0.4%; terrestrial-first 86.8→43.3 ms, 0.0%; load-aware 27.8→34.7 ms, 0.0% |
| Exp 3 high IoT load (1.0) outage / RA failure | TN 16.3% / 2.9%; HAPS 11.4% / 1.3%; sat 3.0% / 1.4%; terrestrial-first 2.7% / 1.5%; load-aware 2.2% / 1.3% |
| Exp 4 coop completion at 1000 / 2000 GFLOP | 98.2% / 91.0% (solo: step at 470 GFLOP) |
| Exp 5 raw streaming | needs 80 Mbit/s; feasible for 0.0% of drones on every tier |
| Exp 5 median endurance | raw 69.4 min, edge-AI 68.3 min (ceiling 74.0 min) |
| Exp 5 delivered per joule | raw: TN 51036.6, HAPS 260114.3, sat 45695.4 bit/J; edge-AI 15.4 bit/J; captures/kJ raw 0.064 / 0.325 / 0.057, edge-AI 0.769 |

### Known sanity-check failure (not fixed, reported as found)

- `naive_monotone_non_decreasing` (exp1 only): one dip 6.062 → 5.879 between 4 and 6 drones/km²,
  Monte Carlo noise at low density (only 15 realizations per density). Same seeds and geometry as the
  previous runs; a 60-realization re-check showed the step increasing.
