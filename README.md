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
  same sub-channel interfere. TN uplink: one co-channel IoT user per other cell, placed within
  300 m of its BS and power-controlled towards it (see TN power control). HAPS: the co-channel user of each other HAPS, uniform inside that HAPS's
  own 10 km cell, through the serving beam's off-axis gain. Satellite: 6 co-channel users of
  other beams at the sidelobe level.
- **Co-channel activity.** Every co-channel interferer (TN IoT user, other-HAPS user, other-beam
  satellite user) is active independently with p_act = 1 − exp(−load), drawn per realization;
  nominal load 0.5 ("medium") → p_act = 0.393.
- **TN power control.** Open-loop fractional uplink power control P = min(P_max, P0 + α·PL) with
  P0 = −90 dBm and α = 1, PL = coupling loss to the own serving BS (propagation loss minus both
  antenna gains, as estimated from RSRP), so P0 is the target received power. Applied to the
  co-channel IoT interferers (P_max 20 dBm) and to the ground sensor's TN uplink (P_max 23 dBm).
  Drones transmit a fixed 30 dBm (UAV, Lou et al.).
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
| BS height / IoT interferer max power | 25 m / 20 dBm | **PLACEHOLDERS** |
| Co-channel load / activity p_act | 0.5 / 0.393 (= 1 − e^−0.5) | **PLACEHOLDER** (see revision 7) |
| TN uplink power control P0 / α | −90 dBm / 1.0 | 3GPP TR 38.802 v14.2.0 Table 9.1.2-1 (mMTC grant-free UL calibration baseline, "Open loop power control: Alpha=1, P0= -90 dBm", 1 RB per user) |
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

**Open issue: the TN dip outside the urban core** (addressed in revision 6 with power control). Diagnostic (1500
realizations) at 0 / 2 / 3 / 5 / 10 km for drones: all interferers 0.997 / 0.798 / 0.277 / 0.645 /
0.843; no interferers 1.000 / 1.000 / 1.000 / 0.993 / 0.965; rural interferers only 1.000 / 0.999 /
0.992 / 0.923 / 0.857; core interferers only 0.997 / 0.799 / 0.282 / 0.686 / 0.949. The dip is
caused entirely by the core cells' co-channel IoT users, which transmit at the full 20 dBm
placeholder although they are within 300 m of their own BS. Real uplinks use open-loop power control
(P = min(P_max, P0 + α·PL)), which would remove most of this interference. Adding it would be a new
model with new placeholders and would change every TN number in Exp 2, 3 and 5, so it is left as a
decision. Because of it, the original spec test `test_tn_coverage_non_increasing` fails (both terminals).

## Changes in revision 6: TN uplink power control

**Change (only this).** Open-loop fractional power control P = min(P_max, P0 + α·PL) for the
co-channel IoT interferers on TN and for the ground sensor's TN uplink. Drones keep a fixed 30 dBm:
no verified aerial-UE power-control source was used, so the UAV value from Lou et al. stays.

- **Source.** P0 = −90 dBm, α = 1 from 3GPP TR 38.802 v14.2.0 Table 9.1.2-1 (baseline for
  grant-free UL multiple access applied to mMTC: 1 RB per user, 20-byte packets), read from
  `38802-e20.zip` (3GPP FTP archive). This is the 3GPP setting closest to IoT devices. Other values in
  3GPP documents: TR 36.814 v9.2.0 Table A.2.2-1 (LTE-A calibration, full buffer) P0 = −106 dBm, α = 1.0;
  TR 38.802 Table A.2.x company assumptions range from P0 = −80 dBm/α = 0.8 to P0 = −100 dBm/α = 1.
- **Deviation from TS 36.213.** The spec formula adds 10·log10(M) for an M-RB allocation. Following
  the requested formula and the 1-RB assumption of TR 38.802 Table 9.1.2-1, it is omitted (including
  it for a 1 MHz ≈ 5.6-RB allocation would raise every power-controlled transmitter by 7.4 dB).
- **Effect on the deep dip.** Gone: drone TN at 3 km 0.269 → 0.903; sensor TN at 2.5 km 0.046 → 0.462.

**Fig. 3 curves, 0 / 1 / 2 / 2.5 / 3 / 5 / 10 km (old = revision 5, new = this run).** HAPS and
satellite curves are bit-identical (power control is TN only).

| Terminal | Curve | Old | New |
|---|---|---|---|
| drone | TN alone | 0.994 / 0.875 / 0.812 / 0.465 / 0.269 / 0.656 / 0.850 | 1.000 / 0.994 / 0.988 / 0.956 / 0.903 / 0.856 / 0.967 |
| drone | HAPS alone, σ_b = 0 | 1.000 / 1.000 / 1.000 / 1.000 / 1.000 / 1.000 / 0.998 | unchanged |
| drone | HAPS alone, σ_b = 3° | 0.957 / 0.957 / 0.955 / 0.955 / 0.956 / 0.954 / 0.945 | unchanged |
| drone | satellite alone | 0.995 / 0.995 / 0.994 / 0.996 / 0.997 / 0.995 / 0.995 | unchanged |
| drone | hybrid, ideal / σ_b = 3° | 1.000 everywhere | 1.000 everywhere |
| drone | TN + satellite | 1.000 / 0.999 / 0.999 / 0.998 / 0.998 / 0.998 / 0.999 | 1.000 everywhere |
| sensor | TN alone | 0.995 / 0.379 / 0.321 / 0.046 / 0.081 / 0.192 / 0.283 | 0.162 / 0.239 / 0.419 / 0.462 / 0.394 / 0.435 / 0.445 |
| sensor | HAPS alone, σ_b = 0 | 0.809 / 0.806 / 0.808 / 0.801 / 0.804 / 0.791 / 0.732 | unchanged |
| sensor | HAPS alone, σ_b = 3° | 0.526 / 0.524 / 0.521 / 0.520 / 0.515 / 0.506 / 0.455 | unchanged |
| sensor | satellite alone | 0.176 / 0.176 / 0.175 / 0.180 / 0.171 / 0.171 / 0.172 | unchanged |
| sensor | hybrid, ideal | 0.999 / 0.901 / 0.894 / 0.845 / 0.852 / 0.861 / 0.842 | 0.868 / 0.878 / 0.908 / 0.914 / 0.904 / 0.903 / 0.878 |
| sensor | hybrid, σ_b = 3° | 0.997 / 0.757 / 0.727 / 0.626 / 0.636 / 0.668 / 0.677 | 0.669 / 0.703 / 0.770 / 0.789 / 0.760 / 0.768 / 0.750 |
| sensor | TN + satellite | 0.996 / 0.489 / 0.438 / 0.217 / 0.239 / 0.331 / 0.406 | 0.309 / 0.369 / 0.519 / 0.558 / 0.500 / 0.531 / 0.541 |

TN first below 50%: drones 2.5 km → never (minimum 0.830 at 4.5 km); sensors 0.5 km → 0 km.

**New TN shape and the sanity check (highest at the hub, no rise beyond a mild transition).**
Diagnostic (1500 realizations, scratch re-implementation of the TN SINR; model unchanged), TN at
0 / 2 / 3 / 4 / 5 / 10 km:

| Terminal | Variant | Coverage |
|---|---|---|
| drone | model | 1.000 / 0.991 / 0.894 / 0.797 / 0.846 / 0.969 |
| drone | no co-channel interferers | 1.000 / 1.000 / 1.000 / 0.999 / 0.994 / 0.976 |
| drone | rural interferers only | 1.000 / 1.000 / 1.000 / 0.997 / 0.990 / 0.970 |
| drone | interferers power-controlled to their nearest BS | 1.000 / 1.000 / 0.989 / 0.897 / 0.892 / 0.971 |
| sensor | model | 0.151 / 0.417 / 0.389 / 0.375 / 0.405 / 0.413 |
| sensor | no co-channel interferers | 0.994 / 0.995 / 0.957 / 0.544 / 0.428 / 0.436 |
| sensor | rural interferers only | 0.994 / 0.987 / 0.953 / 0.529 / 0.415 / 0.414 |
| sensor | interferers power-controlled to their nearest BS | 0.347 / 0.597 / 0.501 / 0.383 / 0.408 / 0.413 |

- **Drones: highest at the hub (passes); not monotone (fails).** Coverage stays ≥ 0.988 inside the
  core, dips to 0.830 at 4.5 km and recovers to 0.967. Without core interferers the curve is
  monotone, so the dip is inter-cell interference from the ~50 core cells: their IoT users are received
  at −90 dBm at their own BS but are only a few hundred metres from the core-edge BSs, which serve
  drones beyond the core at 2+ km range. In the rural area the serving BSs are far from the core and
  noise-limited, hence the recovery. This is a physical urban/rural transition of a reuse-1 network with
  a fixed-power terminal, but larger than "mild" (−0.17), and part of it is an artefact (next point).
- **Artefact still present: interferer association.** Each co-channel IoT user is placed within 300 m
  of "its" BS, but the core inter-site distance is ≈ 0.5 km, so 37.5% of core IoT interferers are
  nearer another BS than their own and are power-controlled towards the farther one. Associating them
  with their nearest BS lifts the drone dip from 0.797 to 0.897 at 4 km and the sensor hub value from
  0.151 to 0.347. Not changed here ("change nothing else"); it is the next fix to make.
- **Sensors: not highest at the hub (fails).** With α = 1 every power-controlled user, the sensor
  included, is received at exactly −90 dBm, so the sensor loses its near-BS advantage and its SINR is
  set only by the interferers around its serving BS. The hub BS sits in the middle of the densest
  co-channel neighbourhood and has the worst SIR; core-edge BSs have half the neighbours (peak 0.462 at
  2.5 km). Beyond ~3.5 km the sensor hits its 23 dBm cap and coverage is noise-limited at ≈ 0.44 (rural
  BS spacing). The rise from the hub to the core edge is physical for full-compensation (α = 1) power
  control in a dense reuse-1 core (it persists, 0.347 → 0.641, with consistent association), and the
  association artefact makes it worse.

## Changes in revision 7: co-channel activity factor, simulated access estimate

1. **Co-channel activity (PLACEHOLDER).** IoT traffic is sparse, so a co-channel cell does not carry
   an IoT uplink on the drone's resource at all times. Each co-channel interferer (TN, and for
   consistency HAPS and satellite) is active independently with p_act = 1 − exp(−load): the probability
   that a Poisson(load) number of IoT transmissions occupies the resource, the same Poisson model as the
   RA background. Load levels are those of exp3 (none 0 / low 0.1 / medium 0.5 / high 1.0 → p_act 0 /
   0.095 / 0.393 / 0.632). Fig. 3 uses the medium level (p_act = 0.393). The full-load case (p_act = 1,
   the previous assumption) is reported alongside. In the exp3 IoT sweep the co-channel activity follows
   the sweep level; the exp3 drone-only RA case keeps the nominal activity.
2. **Load-aware policy with the actual expected access delay.** `balanced` now takes E[access delay |
   success] and P[failure] from Monte Carlo runs of `random_access` itself with the number of drones
   already assigned to the tier plus this one (400 trials per tier and size, fixed seed, cached per RA
   configuration). The previous analytic estimate, which assumes every assigned drone keeps contending
   at every attempt, is kept as `balanced_conservative`. With 13 drones on TN the actual expected delay
   is 16.2 ms vs 65.7 ms for the conservative estimate; 50 drones on a 64-resource tier: 13.4 vs 37.3 ms.

**Fig. 3 curves at 0 / 1 / 2 / 3 / 5 / 10 km** (old = revision 6, p_act = 1; new nominal p_act = 0.393;
new full load p_act = 1).

| Terminal | Curve | Old (rev. 6) | New, p_act = 0.393 | New, p_act = 1 |
|---|---|---|---|---|
| drone | TN alone | 1.000 / 0.994 / 0.988 / 0.903 / 0.856 / 0.967 | 1.000 / 0.997 / 0.995 / 0.961 / 0.929 / 0.970 | 1.000 / 0.993 / 0.988 / 0.899 / 0.858 / 0.965 |
| drone | HAPS alone, σ_b = 0 | 1.000 / 1.000 / 1.000 / 1.000 / 1.000 / 0.998 | 1.000 / 1.000 / 1.000 / 1.000 / 1.000 / 0.998 | 1.000 / 1.000 / 1.000 / 1.000 / 1.000 / 0.997 |
| drone | HAPS alone, σ_b = 3° | 0.957 / 0.957 / 0.955 / 0.956 / 0.954 / 0.945 | 0.962 / 0.960 / 0.961 / 0.961 / 0.960 / 0.953 | 0.955 / 0.955 / 0.956 / 0.955 / 0.954 / 0.944 |
| drone | satellite alone | 0.995 / 0.995 / 0.994 / 0.997 / 0.995 / 0.995 | 0.998 / 0.997 / 0.998 / 0.998 / 0.998 / 0.998 | 0.996 / 0.996 / 0.995 / 0.996 / 0.996 / 0.996 |
| drone | hybrid, ideal / σ_b = 3° | 1.000 everywhere | 1.000 everywhere | 1.000 everywhere |
| drone | TN + satellite | 1.000 everywhere | 1.000 everywhere | 1.000 / 1.000 / 1.000 / 1.000 / 0.999 / 1.000 |
| sensor | TN alone | 0.162 / 0.239 / 0.419 / 0.394 / 0.435 / 0.445 | 0.525 / 0.594 / 0.722 / 0.655 / 0.440 / 0.443 | 0.156 / 0.231 / 0.417 / 0.391 / 0.427 / 0.433 |
| sensor | HAPS alone, σ_b = 0 | 0.809 / 0.806 / 0.808 / 0.804 / 0.791 / 0.732 | 0.812 / 0.815 / 0.805 / 0.797 / 0.787 / 0.733 | 0.810 / 0.812 / 0.803 / 0.796 / 0.785 / 0.729 |
| sensor | HAPS alone, σ_b = 3° | 0.526 / 0.524 / 0.521 / 0.515 / 0.506 / 0.455 | 0.531 / 0.527 / 0.520 / 0.518 / 0.506 / 0.447 | 0.528 / 0.525 / 0.518 / 0.516 / 0.504 / 0.444 |
| sensor | satellite alone | 0.176 / 0.176 / 0.175 / 0.171 / 0.171 / 0.172 | 0.176 / 0.177 / 0.175 / 0.176 / 0.176 / 0.172 | 0.174 / 0.175 / 0.173 / 0.175 / 0.174 / 0.170 |
| sensor | hybrid, ideal | 0.868 / 0.878 / 0.908 / 0.904 / 0.903 / 0.878 | 0.925 / 0.938 / 0.956 / 0.942 / 0.907 / 0.877 | 0.867 / 0.881 / 0.904 / 0.899 / 0.904 / 0.873 |
| sensor | hybrid, σ_b = 3° | 0.669 / 0.703 / 0.770 / 0.760 / 0.768 / 0.750 | 0.812 / 0.844 / 0.889 / 0.863 / 0.778 / 0.744 | 0.667 / 0.700 / 0.766 / 0.756 / 0.772 / 0.738 |
| sensor | TN + satellite | 0.309 / 0.369 / 0.519 / 0.500 / 0.531 / 0.541 | 0.606 / 0.662 / 0.772 / 0.711 / 0.539 / 0.538 | 0.301 / 0.363 / 0.516 / 0.493 / 0.527 / 0.529 |

The new p_act = 1 column reproduces revision 6 within Monte Carlo noise (the activity draws change
the random stream). TN first below 50%: drones never; sensors 3.5 km (rev. 6: 0 km).

**Check: TN highest at the hub at nominal p_act.**
- Drones: **passes** (1.000 at the hub). The core-edge dip shrinks to 0.929 at 5 km; recovery to 0.970
  at 10 km (+0.041) remains, the urban/rural transition described in revision 6.
- Sensors: **fails** (0.525 at the hub, peak 0.722 at 2 km). Cause, diagnosed before changing anything
  else (1500 realizations, scratch re-implementation; TN at 0 / 1 / 2 / 3 / 5 / 10 km): model 0.522 /
  0.598 / 0.701 / 0.664 / 0.421 / 0.427; without co-channel interferers 0.994 / 0.991 / 0.995 / 0.967 /
  0.435 / 0.431; interferers power-controlled to their nearest BS 0.701 / 0.741 / 0.821 / 0.761 / 0.422 /
  0.426. Co-channel BSs within 1 km of the serving BS: 12.5 at the hub, 11.6 at 1 km, 6.5 at 2 km, 4.9 at
  3 km, 1.4 at 5 km. With α = 1 the sensor is received at −90 dBm wherever it is, so its SINR depends only
  on how many active co-channel neighbours its serving BS has; the hub BS has twice as many as a
  core-edge BS. This is physical for full-compensation power control in a dense reuse-1 core. The
  interferer-association artefact (37.5% of core interferers nearer another BS than their own) makes it
  worse but is not the cause: with consistent association the rise remains (0.701 → 0.821). Nothing
  else was changed.

## Experiment-specific choices

- Exp 1 runs on a torus window (no edge effects) of side clip(√(2000/λ) km, 3, 10), 3 realizations
  per density per seed. r0 = median SG edge length at the lowest density (per seed).
- Exp 3 and Exp 5 use ideal HAPS pointing (σ_b = 0); Exp 2 shows the cost of σ_b = 3°.
- Exp 3 equal-bandwidth variant sets all tiers to 5 MHz (noise scales with B too).
- Every stochastic experiment: seeds 1–5, curves show the seed mean with a min–max band.

## Results

See `results/run_log.txt` and `results/metrics.json`. The summary of the latest run is in
the section below.

### Latest run (revision 7; seeds 1–5, seed mean)

| Metric | Value |
|---|---|
| Exp 1 SG degree peak | 8.762 at 4 drones/km²; 6.061 at 250/km² (30.8% below peak) |
| Exp 1 distance rule | r0 = 179.0 m; degree 5.481 → 31.229 |
| Exp 2 drones at 0 / 2 / 5 / 10 km | TN 1.000 / 0.995 / 0.929 / 0.970; HAPS 1.000 / 1.000 / 1.000 / 0.998 (σ_b=3°: 0.962 / 0.961 / 0.960 / 0.953); sat 0.998 / 0.998 / 0.998 / 0.998; hybrid 1.000 everywhere; TN + sat 1.000 everywhere |
| Exp 2 drone gain 0 / 3 / 10 dBi at 10 km | satellite 0.748 / 0.913 / 0.998; TN + satellite 0.932 / 0.987 / 1.000; hybrid 1.000 / 1.000 / 1.000 |
| Exp 2b sensors at 0 / 2 / 5 / 10 km | TN 0.525 / 0.722 / 0.440 / 0.443; HAPS 0.812 / 0.805 / 0.787 / 0.733 (σ_b=3°: 0.531 / 0.520 / 0.506 / 0.447); sat 0.176 / 0.175 / 0.176 / 0.172; hybrid 0.925 / 0.956 / 0.907 / 0.877 (σ_b=3°: 0.812 / 0.889 / 0.778 / 0.744); TN + sat 0.606 / 0.772 / 0.539 / 0.538 |
| Exp 2b sensors at 10 km: hybrid / no HAPS at −5 / 0 / +5 dB | 1.000 / 0.987, 0.995 / 0.857, 0.877 / 0.538 |
| Median uplink rate (exp5) | TN 15.31, HAPS 41.09, satellite 6.20 Mbit/s |
| Exp 3 latency N=1→50, outage (drone-only RA) | TN 42.5→96.0 ms, 0.3%; HAPS 19.1→25.8 ms, 0.0%; sat 92.6→101.0 ms, 0.2%; terrestrial-first 42.4→28.6 ms; load-aware (conservative) 19.1→26.3 ms; load-aware 19.1→25.8 ms; all 0.0% outage |
| Exp 3 high IoT load (1.0, p_act 0.632): outage (RA failure) | TN 5.8% (3.3%); HAPS 2.2% (1.4%); sat 2.8% (1.3%); terrestrial-first 2.6% (1.6%); load-aware (conservative) 2.1% (1.3%); load-aware 2.2% (1.3%) |
| Exp 3, HAPS RA 16, drone-only: latency N=1→50 | HAPS 18.7→56.4 ms; terrestrial-first 33.9→43.0 ms; load-aware (conservative) 18.7→52.4 ms; load-aware 18.7→43.1 ms; all 0.0% outage except sat 0.2% |
| Exp 3, HAPS RA 16, high IoT load: outage (RA failure + over limit), latency at N=50 | TN 5.9% (5.4%), 263.5 ms; HAPS 4.2% (4.2%), 204.6 ms; sat 3.0% (2.6%), 179.0 ms; terrestrial-first 3.3% (3.3%), 162.2 ms; load-aware (conservative) 2.3% (2.4%), 138.5 ms; load-aware 2.8% (2.8%), 139.1 ms |
| Exp 4 coop completion at 1000 / 2000 GFLOP | 98.2% / 91.0% (solo: step at 470 GFLOP) |
| Exp 5 raw streaming | needs 80 Mbit/s; feasible for 0.0% of drones on every tier; ≤ 51.4% of raw delivered (HAPS) |
| Exp 5 median endurance | raw 69.4 min, edge-AI 68.3 min (ceiling 74.0 min) |

### Known sanity-check failures (not fixed, reported as found)

- `tn_non_increasing` and `tn_max_at_hub` (exp2b), test `test_tn_coverage_maximal_at_hub` (sensor
  case): sensor TN is 0.525 at the hub and peaks at 0.722 at 2 km. Cause in revision 7 (α = 1
  equalisation + co-channel neighbourhood density; the association artefact makes it worse).
  Drone TN passes both checks.
- `naive_monotone_non_decreasing` (exp1 only): one dip 6.062 → 5.879 between 4 and 6 drones/km²,
  Monte Carlo noise at low density (only 15 realizations per density). Same seeds and geometry as the
  previous runs; a 60-realization re-check showed the step increasing.

## Claims (revision 7)

| Claim | Verdict | Numbers |
|---|---|---|
| (a) SG graph saturates while distance rule grows | holds | SG degree peaks 8.762 at 4/km², falls to 6.061 at 250/km² (−30.8%); distance rule 5.481 → 31.229 |
| (b) HAPS is needed for coverage (drones) | does not hold | TN + satellite 1.000 at every distance and threshold (hybrid 1.000); with p_act = 1 also ≥ 0.999. Even a 0 dBi drone gets 0.932 at 10 km without HAPS vs 1.000 with it (3 dBi: 0.987 vs 1.000) |
| (c) HAPS is needed for latency/throughput | holds at nominal bandwidths; a bandwidth effect | all-HAPS 19.1→25.8 ms vs TN 42.5→96.0 ms and satellite 92.6→101.0 ms; HAPS median 41.09 Mbit/s vs TN 15.31, sat 6.20 |
| (d) coordination prevents congestion collapse | does not hold as stated; coordination helps latency when HAPS is congested | no collapse in any case (worst: all-TN 9.8% outage at N=50, HAPS RA 16 + high IoT load). HAPS RA 16, drone-only: load-aware 43.1 ms at N=50 vs all-HAPS 56.4 ms, terrestrial-first 43.0 ms (tie). HAPS RA 16 + high IoT load: load-aware 2.8% outage, 139.1 ms (conservative variant 2.3%, 138.5 ms) vs best single tier (satellite) 3.0%, 179.0 ms and terrestrial-first 3.3%, 162.2 ms |
| (e) cooperation beats solo | holds | 91.0% vs 0% completion at 2000 GFLOP |
| (f) edge-AI is needed for data delivery | holds (if the 0.02 Mbit report carries the needed information) | raw feasible for 0.0% of drones on every tier; ≤ 51.4% of raw delivered (HAPS); report feasible 100% |
| (g) edge-AI saves energy | does not hold | endurance edge-AI 68.3 min vs raw 69.4 min |
| (h) HAPS is needed for coverage of ground IoT sensors | holds at +5 and 0 dB; small margin at −5 dB | +5 dB, hybrid vs TN + sat: 0.925 vs 0.606 at the hub, 0.956 vs 0.772 at 2 km, 0.907 vs 0.539 at 5 km, 0.877 vs 0.538 at 10 km (σ_b = 3°: 0.744 at 10 km). 10 km at 0 dB: 0.995 vs 0.857; −5 dB: 1.000 vs 0.987 |
