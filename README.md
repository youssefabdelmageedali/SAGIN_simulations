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
| `experiments/exp2_coverage.py` | Exp 2 → two-panel `fig3_coverage_vs_distance.png` ((a) drones, (b) sensors, incl. no-HAPS ablation) and two-panel `fig3b_coverage_threshold_sweep.png` |
| `experiments/exp2b_iot_coverage.py` | Exp 2b → panel (b) of `fig3_coverage_vs_distance.png` and `fig3b_coverage_threshold_sweep.png` (ground IoT sensor) |
| `experiments/exp3_latency.py` | Exp 3 → `fig4_latency_vs_density.png`, `fig4b_latency_equal_bw.png`, `fig4c_outage_causes.png`, `fig4d_access_delay_vs_iot.png`, `fig4e_ra_failure_vs_iot.png` |
| `experiments/exp5a_compute.py` | Exp 4 → `fig5a_completion_vs_complexity.png` |
| `experiments/exp5b_energy.py` | Exp 5 → `fig5b_battery_vs_time.png`, `fig5c_raw_backlog.png`, `fig5d_bits_per_joule.png` |
| `experiments/common.py` | plotting style, colours, seeding (no physics) |
| `run_all.py` | runs everything, writes log + metrics |
| `tests/test_world.py` | the 8 spec sanity tests + 3GPP calibration, IoT-load, TN-max-at-hub, region-size and HAPS-nadir tests |

## Model

- **Geometry.** 40 × 40 km region, hub at the origin. Drones at 100 m, HAPS at 20 km, LEO at 600 km.
  TN base stations: one BS at the hub plus an inhomogeneous PPP (4 /km² inside a 2 km urban core,
  0.05 /km² outside it). Drone swarm: Thomas cluster process. HAPS: fixed layout, one above the hub and
  the others on a 20 km ring, each serving a 10 km cell. Satellites: spherical BPP (n_sat uniform on the
  orbital shell, serving = max elevation, ≥ 10°). Coverage curves are averaged over a random azimuth
  per realization.
- **SINR.** SINR = P G |h|² PL⁻¹ / (I + N). TN, mesh, HAPS receivers: N = −174 dBm/Hz + 10log10(B) + 7 dB.
  Satellite receiver: N = k·T·B with T = G_rx / (G/T) (G/T = 1.1 dB/K, G_rx = 30 dBi → T = 776 K).
  TN: 128.1 + 37.6 log10(d_km); mesh: 40 + 25 log10(d_m); HAPS: free space at 20 GHz; satellite: free
  space at 2 GHz plus 5.3 dB fixed losses; 3-D distance. Nakagami-m (m = 2) on TN and mesh; 4 dB
  log-normal shadowing on HAPS/sat.
- **Interference.** Mesh: every drone transmits at once on one of 8 sub-channels; all others on the
  same sub-channel interfere. TN uplink: one co-channel IoT user per other cell, at full 20 dBm
  (no uplink power control). HAPS: the co-channel user of each other HAPS, uniform inside that HAPS's
  own 10 km cell, through the serving beam's off-axis gain. Satellite: 6 co-channel users of
  other beams at the sidelobe level.
- **Ground IoT sensor (exp2b).** `SAGINWorld.ground_sensor()` is the same world with the access-link
  terminal replaced by a sensor at 0 m, 23 dBm, 0 dBi; co-channel HAPS/satellite users are then sensors
  too. Path loss, fading, interference, deployments and thresholds are exactly those of exp2.
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
| Region side | 40 km | chosen so no curve to 10 km sees the edge (tested against 60 km) |
| TN: hub BS / core radius / core density / rural density / interferer radius | 1 BS at (0, 0) / 2 km / 4 per km² / 0.05 per km² / 300 m | **PLACEHOLDERS** (see revision 5) |
| HAPS layout: ring radius / cell radius | 20 km / 10 km (one HAPS above the hub, the other two at 0° and 180°) | **PLACEHOLDERS** |
| HAPS count / satellites / sat co-channel users | 3 / 720 / 6 | **PLACEHOLDERS** |
| Ground IoT sensor Tx power / antenna gain | 23 dBm / 0 dBi | UE power class 3 and omnidirectional 0 dBi: 3GPP TR 36.763 v17.0.0 Table 6.1-1; same as the handheld in TR 38.821 Table 6.1.1.1-3 |
| Ground IoT sensor height | 0 m | given in the task; **PLACEHOLDER** in the sense that no clutter, building-penetration or body loss is modelled |
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

## Changes in revision 4 (additions only; no existing model or parameter changed)

1. **Exp 2b, ground IoT coverage.** Same coverage experiment, seeds and geometry as exp2, with a ground
   sensor as the terminal (23 dBm and 0 dBi from TR 36.763 Table 6.1-1, verified in the document; 0 m
   height as specified). Same access-threshold sweep (−5, 0, +5 dB).
2. **Drone antenna-gain sensitivity for exp2**: 0, 3, 10 dBi at 30 dBm (`drone_gain_sweep` in
   `metrics.json`). 10 dBi is the nominal case.
All metrics already present in the previous run are bit-identical in this run.

## Changes in revision 5: audit of every Exp 2 / Exp 2b curve

**Changes (only these; justified one by one).**

1. **One TN base station at the hub (0, 0).** Without it, the hub was an arbitrary point of a PPP, so
   coverage there was no better than anywhere else in the region.
2. **Inhomogeneous TN PPP**: urban core of radius 2 km at 4 BS/km² (inter-site distance ≈ 0.5 km,
   typical of dense urban macro) and 0.05 BS/km² outside it (inter-site distance ≈ 4.5 km, typical of
   rural macro). Both values are PLACEHOLDERS: they represent a city hub with rural surroundings, not a
   specific operator deployment.
3. **Region 10 × 10 km → 40 × 40 km.** With the 10 km region every curve beyond 5 km was measured
   outside the deployment (TN fell to 0.019 at 10 km only because no BS existed there). A test now
   checks that every curve, for both terminals, changes by no more than Monte Carlo noise when the
   region grows to 60 km.
4. **HAPS layout: deterministic, independent of the region.** One HAPS above the hub, the others on
   a 20 km ring; each serves a 10 km cell (PLACEHOLDERS). All curve points (≤ 10 km) lie in the hub
   HAPS's cell, so distance from the hub equals distance from the nearest nadir.
5. **HAPS co-channel users placed in their own HAPS's cell** (uniform within 10 km of its nadir)
   instead of uniformly over the whole region. **This was the cause of the rising HAPS curves.**
   Diagnostic on the previous code (3000 realizations): with those users removed, drone HAPS
   coverage was 1.000 at every distance and sensor HAPS coverage fell 0.806 → 0.772 from 0 to 10 km.
   With them, coverage rose (sensor 0.704 → 0.767, drone 0.908 → 1.000). Region-wide users sat inside
   the serving beam's main lobe for a terminal near the hub; a terminal beyond 5 km was outside the
   region, and so were the users, off-axis. It was an artefact of interferer placement.
6. **Random measurement azimuth per realization.** The fixed HAPS layout is not rotation-symmetric;
   averaging over directions makes "distance from the hub" well defined. Satellite curves are unaffected.
7. **Checks**: hybrid ≥ every tier and no-HAPS = TN ∨ satellite are asserted per realization (they
   always were, by construction; now explicit). New per-curve checks: TN maximal at the hub, HAPS
   non-increasing from the nadir, satellite flat. New tests: TN maximal at the hub (both terminals),
   region-size independence 40 → 60 km (every curve, both terminals), HAPS non-increasing from the nearest nadir.

Not changed: path loss, fading, powers, thresholds, satellite model, all other parameters. Exp 1 and
Exp 4 are bit-identical. Exp 3 and Exp 5 change because they use the same TN and HAPS deployment.

**Per-curve audit** (seed means at 0 / 2 / 5 / 10 km; old = revision 4, new = this run).

| Terminal | Curve | Old | New | Expected from physics | Agrees? |
|---|---|---|---|---|---|
| drone | TN alone | 0.889 / 0.891 / 0.810 / 0.019 | 0.994 / 0.812 / 0.656 / 0.850 | max at the hub (BS 75 m below); interference-limited plateau in the core; in this model a dip just outside the core, where a core-edge BS ≈ 1 km away serves the drone and collects the full-power interference of ~50 core IoT users; recovery in the rural area to a level set by rural BS spacing | max at hub: yes. Monotone: **no** — dip to 0.269 at 3 km, see below |
| drone | HAPS alone, σ_b = 0 | 0.910 / 0.913 / 0.954 / 1.000 | 1.000 / 1.000 / 1.000 / 0.998 | flat near 1: slant range 20 → 22.4 km (+1 dB), large SNR margin | yes |
| drone | HAPS alone, σ_b = 3° | 0.818 / 0.820 / 0.879 / 0.945 | 0.957 / 0.955 / 0.954 / 0.945 | slightly below ideal (mean pointing loss 4.3 dB), slowly decreasing | yes |
| drone | satellite alone | 0.996 / 0.996 / 0.996 / 0.996 | 0.995 / 0.994 / 0.995 / 0.995 | flat: 10 km on the ground is < 1° of elevation at 600 km | yes |
| drone | hybrid, ideal | 1.000 / 1.000 / 1.000 / 1.000 | 1.000 / 1.000 / 1.000 / 1.000 | ≥ every tier per realization | yes |
| drone | hybrid, σ_b = 3° | 1.000 / 1.000 / 1.000 / 1.000 | 1.000 / 1.000 / 1.000 / 1.000 | ≥ every tier per realization | yes |
| drone | TN + satellite | 0.999 / 1.000 / 0.999 / 0.996 | 1.000 / 0.999 / 0.998 / 0.999 | max(TN, sat) per realization; ≈ satellite | yes |
| sensor | TN alone | 0.389 / 0.412 / 0.287 / 0.000 | 0.995 / 0.321 / 0.192 / 0.283 | max at the hub (BS 25 m away); weak 23 dBm / 0 dBi link is interference-limited in the core; same dip outside the core (0.046 at 2.5 km); rural level limited by the ~2 km noise-limited range vs ~2.2 km mean nearest-BS distance | max at hub: yes. Monotone: **no**, same cause |
| sensor | HAPS alone, σ_b = 0 | 0.712 / 0.711 / 0.749 / 0.762 | 0.809 / 0.808 / 0.791 / 0.732 | decreasing: longer slant range, and near the cell edge the neighbouring cell's co-channel user comes into the serving main lobe | yes (was rising: artefact, fixed) |
| sensor | HAPS alone, σ_b = 3° | 0.451 / 0.451 / 0.480 / 0.480 | 0.526 / 0.521 / 0.506 / 0.455 | same shape, lower | yes |
| sensor | satellite alone | 0.180 / 0.174 / 0.177 / 0.174 | 0.176 / 0.175 / 0.171 / 0.172 | flat; median SNR ≈ 17 dB below the drone's, so only shadowing peaks pass +5 dB | yes |
| sensor | hybrid, ideal | 0.859 / 0.863 / 0.855 / 0.806 | 0.999 / 0.894 / 0.861 / 0.842 | ≥ every tier; TN at the hub, HAPS elsewhere | yes |
| sensor | hybrid, σ_b = 3° | 0.730 / 0.734 / 0.695 / 0.572 | 0.997 / 0.727 / 0.668 / 0.677 | ≥ every tier; follows the TN dip where HAPS is weak | yes |
| sensor | TN + satellite | 0.500 / 0.514 / 0.413 / 0.174 | 0.996 / 0.438 / 0.331 / 0.406 | max(TN, sat) per realization; TN shape lifted by the satellite | yes |

**TN first below 50%**: drones 7.0 km → 2.5 km; sensors 0.0 km → 0.5 km.

**Serving satellite**: median elevation 53.7°, 5th percentile 31.3°, minimum 11.9°; no realization
without a satellite above 10° (0.000%). The serving satellite is chosen by maximum elevation and
rejected below 10°, as intended.

**Open issue: the TN dip outside the urban core** (accepted, not fixed). Diagnostic (1500
realizations) at 0 / 2 / 3 / 5 / 10 km for drones: all interferers 0.997 / 0.798 / 0.277 / 0.645 /
0.843; no interferers 1.000 / 1.000 / 1.000 / 0.993 / 0.965; rural interferers only 1.000 / 0.999 /
0.992 / 0.923 / 0.857; core interferers only 0.997 / 0.799 / 0.282 / 0.686 / 0.949. The dip is
caused entirely by the core cells' co-channel IoT users, which transmit at the full 20 dBm
placeholder although they are within 300 m of their own BS. Real uplinks use open-loop power control
(P = min(P_max, P0 + α·PL)), which would remove most of this interference. Adding it would be a new
model with new placeholders and would change every TN number in Exp 2, 3 and 5, so it is left as a
decision. Because of it, the original spec test `test_tn_coverage_non_increasing` fails (both terminals).

## Experiment-specific choices

- Exp 1 runs on a torus window (no edge effects) of side clip(√(2000/λ) km, 3, 10), 3 realizations
  per density per seed. r0 = median SG edge length at the lowest density (per seed).
- Exp 3 and Exp 5 use ideal HAPS pointing (σ_b = 0); Exp 2 shows the cost of σ_b = 3°.
- Exp 3 equal-bandwidth variant sets all tiers to 5 MHz (noise scales with B too).
- Every stochastic experiment: seeds 1–5, curves show the seed mean with a min–max band.

## Results

See `results/run_log.txt` and `results/metrics.json`. The summary of the latest run is in
the section below.

### Latest run (revision 5; seeds 1–5, seed mean)

| Metric | Value |
|---|---|
| Exp 1 SG degree peak | 8.762 at 4 drones/km²; 6.061 at 250/km² (30.8% below peak) |
| Exp 1 distance rule | r0 = 179.0 m; degree 5.481 → 31.229 |
| Exp 2 drones at 0 / 2 / 5 / 10 km | TN 0.994 / 0.812 / 0.656 / 0.850; HAPS 1.000 / 1.000 / 1.000 / 0.998 (σ_b=3°: 0.957 / 0.955 / 0.954 / 0.945); sat 0.995 / 0.994 / 0.995 / 0.995; hybrid 1.000 everywhere; TN + sat 1.000 / 0.999 / 0.998 / 0.999 |
| Exp 2 drone gain 0 / 3 / 10 dBi at 10 km | satellite 0.741 / 0.905 / 0.995; TN + satellite 0.877 / 0.965 / 0.999; hybrid 0.998 / 1.000 / 1.000 |
| Exp 2b sensors at 0 / 2 / 5 / 10 km | TN 0.995 / 0.321 / 0.192 / 0.283; HAPS 0.809 / 0.808 / 0.791 / 0.732 (σ_b=3°: 0.526 / 0.521 / 0.506 / 0.455); sat 0.176 / 0.175 / 0.171 / 0.172; hybrid 0.999 / 0.894 / 0.861 / 0.842 (σ_b=3°: 0.997 / 0.727 / 0.668 / 0.677); TN + sat 0.996 / 0.438 / 0.331 / 0.406 |
| Exp 2b sensors at 10 km: hybrid / no HAPS at −5 / 0 / +5 dB | 1.000 / 0.971, 0.993 / 0.774, 0.842 / 0.406 |
| Median uplink rate (exp5) | TN 6.06, HAPS 39.73, satellite 5.87 Mbit/s |
| Exp 3 latency N=1→50, outage (drone-only) | TN 94.3→152.0 ms, 13.3%; HAPS 19.5→26.5 ms, 0.0%; sat 102.6→107.8 ms, 0.5%; terrestrial-first 84.5→35.5 ms, 0.0%; load-aware 19.5→26.6 ms, 0.0% |
| Exp 3 high IoT load (1.0) outage / RA failure | TN 19.1% / 3.0%; HAPS 2.2% / 1.3%; sat 3.2% / 1.4%; terrestrial-first 2.8% / 1.6%; load-aware 2.0% / 1.2% |
| Exp 4 coop completion at 1000 / 2000 GFLOP | 98.2% / 91.0% (solo: step at 470 GFLOP) |
| Exp 5 raw streaming | needs 80 Mbit/s; feasible for 0.0% of drones on every tier; ≤ 49.7% of raw delivered (HAPS) |
| Exp 5 median endurance | raw 69.4 min, edge-AI 68.3 min (ceiling 74.0 min) |
| Exp 5 delivered per joule | raw: TN 47316.7, HAPS 310401.6, sat 45838.8 bit/J; edge-AI 15.4 bit/J; captures/kJ raw 0.059 / 0.388 / 0.057, edge-AI 0.769 |

### Known sanity-check failures (not fixed, reported as found)

- `tn_non_increasing` (exp2 and exp2b) and the spec test `test_tn_coverage_non_increasing`: TN dips
  outside the urban core (drones 0.269 at 3 km, sensors 0.046 at 2.5 km) and recovers in the rural
  area. Caused by full-power co-channel IoT interferers in the core cells (see revision 5, open issue).
- `naive_monotone_non_decreasing` (exp1 only): one dip 6.062 → 5.879 between 4 and 6 drones/km²,
  Monte Carlo noise at low density (only 15 realizations per density). Same seeds and geometry as the
  previous runs; a 60-realization re-check showed the step increasing.

## Claims (revision 5)

| Claim | Verdict | Numbers |
|---|---|---|
| (a) SG graph saturates while distance rule grows | holds | SG degree peaks 8.762 at 4/km², falls to 6.061 at 250/km² (−30.8%); distance rule 5.481 → 31.229 |
| (b) HAPS is needed for coverage (drones) | does not hold for the 10 dBi UAV | TN + satellite 0.998–1.000 at every distance vs hybrid 1.000 (10 km: 0.999 vs 1.000). For a 0 dBi drone TN + sat is 0.877 at 10 km vs hybrid 0.998 (3 dBi: 0.965 vs 1.000) |
| (c) HAPS is needed for latency/throughput | holds at nominal bandwidths; a bandwidth effect | all-HAPS 19.5→26.5 ms vs satellite 102.6→107.8 ms, TN 94.3→152.0 ms; HAPS median 39.73 Mbit/s vs TN 6.06, sat 5.87; at equal 5 MHz satellite gives 36.6→43.0 ms |
| (d) coordination prevents congestion collapse | does not hold | no collapse drone-only (RA failure 0.0% for all policies); at high IoT load, congestion outage (RA failure + over limit) load-aware 2.0% vs all-HAPS 2.2% |
| (e) cooperation beats solo | holds | 91.0% vs 0% completion at 2000 GFLOP |
| (f) edge-AI is needed for data delivery | holds (if the 0.02 Mbit report carries the needed information) | raw feasible for 0.0% of drones on every tier; ≤ 49.7% of raw delivered (HAPS); report feasible 99.9–100% |
| (g) edge-AI saves energy | does not hold | endurance edge-AI 68.3 min vs raw 69.4 min |
| (h) HAPS is needed for coverage of ground IoT sensors | holds away from the hub; margin shrinks as the threshold is relaxed | at +5 dB: hybrid vs TN + sat 0.999 vs 0.996 at the hub, 0.894 vs 0.438 at 2 km, 0.861 vs 0.331 at 5 km, 0.842 vs 0.406 at 10 km; at 10 km, 0 dB: 0.993 vs 0.774; −5 dB: 1.000 vs 0.971; with σ_b = 3°: 0.677 at 10 km (+5 dB) |
