# SAGIN stochastic-geometry simulation

Simulation for "A Unified SAGIN Architecture for Autonomous Drone Swarms and Massive IoT".
Central claim under test: the communication graph is **derived from SINR**
(edge i→j iff SINR_ij > γ_th), not assumed from distance.

## FROZEN MODEL (v1.2)

This is the model version the paper cites. v1.0 (commit `59d1aad`) froze Exp 1–5 and 2b; v1.1 adds
Exp 2c (drones as aerial IoT gateways) with new parameters only — every v1.0 result is bit-identical.
v1.2 adds the Exp 2c sensor → drone reuse sensitivity (no new `Params`; every v1.1 number is
bit-identical, only the strict density-trend check changes). Every `Params` field, its value and its
source. "Task specification" = given in the original simulation brief without a further reference;
"assumption, no citation" = a modelling value with no citable source; **PLACEHOLDER** = not
literature-matched, listed in `PLACEHOLDERS` (`sagin/world.py`) and in `metrics.json`. 3GPP values were
read from the documents (3GPP FTP archive: TR 38.821 v16.2.0, TR 36.763 v17.0.0, TR 38.802 v14.2.0).
Seeds 1–5; same seeds reproduce `results/metrics.json` bit for bit.

| Parameter | Value | Source / status |
|---|---|---|
| `region_km` | 40 | modelling choice (edge-independence tested vs 60 km) |
| `h_drone_m` | 100 | task specification (swarm below 1 km); assumption |
| `h_bs_m` | 25 | **PLACEHOLDER** — typical macro mast height, not tied to a reference |
| `h_iot_m` | 1.5 | assumption, no citation (height of co-channel IoT interferers) |
| `h_haps_m` | 20000 | task specification (stratospheric HAPS) |
| `h_sat_m` | 600000 | 3GPP TR 38.821 v16.2.0 Table 6.1.1.1-1 (LEO-600) |
| `f_tn_hz` | 2e+09 | assumption, no citation |
| `f_haps_hz` | 2e+10 | assumption, no citation |
| `f_sat_hz` | 2e+09 | 3GPP TR 38.821 Table 6.1.1.1-1, TR 36.763 v17.0.0 Table 6.1-1 (S-band) |
| `p_drone_access_dbm` | 30 | Lou, Belmekki, Alouini, arXiv:2310.09659, Table I (UAV) |
| `p_drone_mesh_dbm` | 10 | Lou et al. (task specification) |
| `p_iot_dbm` | 20 | **PLACEHOLDER** — co-channel IoT uplink interferer maximum power |
| `ul_p0_dbm` | -90 | 3GPP TR 38.802 v14.2.0 Table 9.1.2-1 (mMTC grant-free UL baseline) |
| `ul_alpha` | 1 | 3GPP TR 38.802 v14.2.0 Table 9.1.2-1 |
| `terminal_tn_power_control` | False | model switch: drones fixed power (UAV), sensors power-controlled |
| `g_drone_dbi` | 10 | Lou, Belmekki, Alouini, arXiv:2310.09659, Table I (UAV) |
| `g_tn_dbi` | 15 | assumption, no citation (task specification: 'tuned') |
| `g_haps_dbi` | 30 | Lou, Belmekki, Alouini, arXiv:2310.09659, Table I |
| `g_sat_dbi` | 30 | 3GPP TR 38.821 Table 6.1.1.1-1 (Set-1 LEO-600 S-band Rx max gain) |
| `sat_gt_dbk` | 1.1 | 3GPP TR 38.821 Table 6.1.1.1-1 (Set-1 LEO-600 S-band G/T) |
| `sat_losses_db` | 5.3 | 3GPP TR 36.763 Table 6.2-1 (scintillation 2.2 + atmospheric 0.1 + polarization 3) |
| `g_iot_dbi` | 0 | 3GPP TR 36.763 Table 6.1-1 (omnidirectional 0 dBi) |
| `nakagami_m` | 2 | Lou et al. (task specification) |
| `shadow_std_db` | 4 | **PLACEHOLDER** — shadowed-Rician approximation for HAPS/sat |
| `noise_psd_dbm_hz` | -174 | thermal noise at 290 K |
| `noise_figure_db` | 7 | 3GPP TR 38.821 Table 6.1.1.1-3 (handheld NF 7 dB); TR 36.763 Table 6.1-1 |
| `b_tn_hz` | 1e+06 | assumption, no citation (task specification) |
| `b_haps_hz` | 5e+06 | HIBS 5 MHz (Alexandre 2021, task specification) |
| `b_sat_hz` | 1.08e+06 | 3GPP TR 36.763 Table 6.1-1 (eMTC UL up to 1080 kHz) |
| `b_mesh_hz` | 2e+07 | assumption, no citation (task specification) |
| `n_subch_mesh` | 8 | assumption, no citation (task specification) |
| `haps_beamwidth_deg` | 5 | **PLACEHOLDER** — HAPS spot-beam 3 dB width |
| `sidelobe_db` | -30 | **PLACEHOLDER** — sidelobe floor for HAPS and satellite beams |
| `sigma_b_deg` | 3 | task specification (pointing-error case) |
| `tn_core_radius_km` | 2 | **PLACEHOLDER** — TN urban-core radius around the hub |
| `tn_core_density_km2` | 4 | **PLACEHOLDER** — TN urban-core BS density (inter-site distance ~0.5 km) |
| `tn_rural_density_km2` | 0.05 | **PLACEHOLDER** — TN rural BS density outside the core (inter-site distance ~4.5 km) |
| `tn_interferer_radius_m` | 300 | **PLACEHOLDER** — co-channel uplink user placement around other BSs |
| `n_haps` | 3 | **PLACEHOLDER** — number of HAPS (one above the hub, the rest on a ring) |
| `haps_spacing_km` | 20 | **PLACEHOLDER** — HAPS ring radius around the hub HAPS |
| `haps_cell_radius_km` | 10 | **PLACEHOLDER** — HAPS service-cell radius (co-channel users placed inside their own cell) |
| `n_sat` | 720 | **PLACEHOLDER** — LEO constellation size for the spherical BPP |
| `sat_min_elev_deg` | 10 | 3GPP TR 36.763 Table 6.1-1 |
| `n_sat_cochannel` | 6 | **PLACEHOLDER** — co-channel users in other satellite beams |
| `pcp_mean_offspring` | 10 | **PLACEHOLDER** — Thomas-process mean cluster size |
| `pcp_sigma_m` | 100 | **PLACEHOLDER** — Thomas-process cluster spread |
| `mission_radius_km` | 2 | **PLACEHOLDER** — mission area for exp3/exp5 swarms |
| `mission_sigma_m` | 200 | **PLACEHOLDER** — cluster spread for exp3/exp5 swarms |
| `swarm_density_km2` | 10 | **PLACEHOLDER** — exp4 swarm density |
| `swarm_window_km` | 3 | **PLACEHOLDER** — exp4 torus window side |
| `gamma_th_db` | -5 | **PLACEHOLDER** — graph SINR threshold |
| `access_th_db` | 5 | **PLACEHOLDER** — access SINR threshold |
| `n_ra_tn` | 12 | 3GPP NB-IoT NPRACH (Lin 2016, task specification) |
| `n_ra_haps` | 64 | **PLACEHOLDER** — HAPS random-access resources |
| `n_ra_sat` | 64 | **PLACEHOLDER** — satellite random-access resources |
| `t_ra_s` | 0.0064 | NPRACH format 1 (task specification) |
| `backoff_cap_exp` | 6 | **PLACEHOLDER** — binary exponential back-off cap |
| `ra_max_attempts` | 10 | **PLACEHOLDER** — RA attempts before giving up |
| `latency_limit_s` | 1 | **PLACEHOLDER** — latency outage limit |
| `tn_cap_margin` | 1 | **PLACEHOLDER** — terrestrial-first baseline TN load margin |
| `iot_ra_load` | 0 | **PLACEHOLDER** — IoT preambles per RA resource per opportunity (swept in exp3) |
| `cochannel_load` | 0.5 | **PLACEHOLDER** — co-channel IoT load per resource; activity p_act = 1 - exp(-load) (medium level 0.5 nominal) |
| `drone_gflops` | 470 | Jetson-Nano class (Hakani & Rawat 2024, task specification) |
| `task_deadline_s` | 1 | **PLACEHOLDER** — task deadline |
| `max_helpers` | 5 | **PLACEHOLDER** — helpers per task |
| `chunk_overhead_mbit` | 0.2 | **PLACEHOLDER** — per-helper transfer overhead |
| `battery_wh` | 148 | ≈ DJI Matrice 4D 149.9 Wh (task specification) |
| `p_flight_w` | 120 | literature-based (task specification) |
| `p_compute_w` | 10 | literature-based (task specification) |
| `p_tx_w` | 8 | **PLACEHOLDER** — radio transmit power draw |
| `raw_mbit` | 800 | **PLACEHOLDER** — raw sensor payload per capture |
| `packet_mbit` | 0.5 | **PLACEHOLDER** — uplink packet size (exp3) |
| `report_mbit` | 0.02 | **PLACEHOLDER** — edge-AI report size |
| `capture_interval_s` | 10 | **PLACEHOLDER** — capture cycle length |
| `h_sensor_m` | 0 | **PLACEHOLDER** — ground sensor height (given); no clutter or building-penetration loss is modelled |
| `p_sensor_dbm` | 23 | 3GPP TR 36.763 Table 6.1-1 (UE power class 3); TR 38.821 Table 6.1.1.1-3 |
| `g_sensor_dbi` | 0 | 3GPP TR 36.763 Table 6.1-1; TR 38.821 Table 6.1.1.1-3 |
| `f_a2g_hz` | 2e+09 | assumption, no citation (sensor → drone link in the TN band) |
| `b_a2g_hz` | 1e+06 | task specification (1 MHz sensor → drone channel) |
| `a2g_shadow_los_db` | 4 | **PLACEHOLDER** — A2G LoS shadowing std (TR 36.777 Table B-3 refers to TR 38.901; not verified here) |
| `a2g_shadow_nlos_db` | 6 | **PLACEHOLDER** — A2G NLoS shadowing std (TR 36.777 Table B-3 refers to TR 38.901; not verified here) |
| `gw_swarm_radius_km` | 12 | **PLACEHOLDER** — extent of the gateway swarm (covers the 0–10 km curves plus 2 km) |
| `gw_densities_km2` | (2.0, 10.0, 50.0) | **PLACEHOLDER** — gateway swarm densities low / nominal / high |

Fixed model structure (not parameters): TN 3GPP-like macro path loss 128.1 + 37.6 log10(d_km); mesh
40 + 25 log10(d_m); free space for HAPS (20 GHz) and satellite (2 GHz); Nakagami-m on TN/mesh,
log-normal shadowing on HAPS/satellite; satellite noise k·T·B from G/T; one TN BS at the hub;
deterministic HAPS layout; co-channel TN IoT interferers associated with their nearest BS and
power-controlled towards it; co-channel activity p_act = 1 − exp(−cochannel_load); slotted random
access with binary exponential back-off; non-learned decision rules only (no Graph-MARL).
Exp 2c sensor → drone link: 3GPP TR 36.777 v15.0.0 Annex B air-to-ground model (Table B-1 LoS probability,
Table B-2 path loss), read from the document's equation objects; UMa-AV for sensors inside the urban core,
RMa-AV outside; drone at 100 m is the aerial node; Nakagami-m fading; nearest-drone association.

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
| `experiments/exp2c_drone_gateway.py` | Exp 2c → `fig3c_drone_gateway.png` (drones as aerial IoT gateways, three densities), `fig3d_drone_gateway_reuse.png` (sub-channel reuse sensitivity) |
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
- **TN interferer association.** Each co-channel IoT interferer (one dropped within 300 m of each BS)
  is associated with its nearest BS (minimum path loss, the terminal's criterion) and power-controlled
  towards it; interferers associated with the terminal's serving BS are not co-scheduled.
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

## Changes in revision 8 (v1.0): interferer association

**Change (only this).** Every co-channel TN IoT interferer is associated with its nearest BS (by path
loss, as for drones and sensors) and power-controlled towards it; previously it was tied to the BS it
was dropped around, although 37.5% of core interferers were nearer another BS. Interferers associated
with the serving BS are not co-scheduled. Exp 1 and Exp 4 are bit-identical; HAPS and satellite curves
are bit-identical.

**Fig. 3 curves at 0 / 1 / 2 / 3 / 5 / 10 km** (old = revision 7).

| Terminal | Curve | Old, p_act 0.393 | New, p_act 0.393 | Old, p_act 1 | New, p_act 1 |
|---|---|---|---|---|---|
| drone | TN alone | 1.000 / 0.997 / 0.995 / 0.961 / 0.929 / 0.970 | 1.000 / 1.000 / 1.000 / 0.996 / 0.950 / 0.971 | 1.000 / 0.993 / 0.988 / 0.899 / 0.858 / 0.965 | 1.000 / 1.000 / 1.000 / 0.990 / 0.890 / 0.967 |
| drone | HAPS alone, σ_b = 0 | 1.000 / 1.000 / 1.000 / 1.000 / 1.000 / 0.998 | unchanged | 1.000 / 1.000 / 1.000 / 1.000 / 1.000 / 0.997 | unchanged |
| drone | HAPS alone, σ_b = 3° | 0.962 / 0.960 / 0.961 / 0.961 / 0.960 / 0.953 | unchanged | 0.955 / 0.955 / 0.956 / 0.955 / 0.954 / 0.944 | unchanged |
| drone | satellite alone | 0.998 / 0.997 / 0.998 / 0.998 / 0.998 / 0.998 | unchanged | 0.996 / 0.996 / 0.995 / 0.996 / 0.996 / 0.996 | unchanged |
| drone | hybrid, ideal / σ_b = 3° | 1.000 everywhere | 1.000 everywhere | 1.000 everywhere | 1.000 everywhere |
| drone | TN + satellite | 1.000 everywhere | 1.000 everywhere | 1.000 / 1.000 / 1.000 / 1.000 / 0.999 / 1.000 | 1.000 / 1.000 / 1.000 / 1.000 / 0.999 / 1.000 |
| sensor | TN alone | 0.525 / 0.594 / 0.722 / 0.655 / 0.440 / 0.443 | 0.697 / 0.735 / 0.819 / 0.734 / 0.442 / 0.443 | 0.156 / 0.231 / 0.417 / 0.391 / 0.427 / 0.433 | 0.346 / 0.416 / 0.590 / 0.500 / 0.432 / 0.436 |
| sensor | HAPS alone, σ_b = 0 | 0.812 / 0.815 / 0.805 / 0.797 / 0.787 / 0.733 | unchanged | 0.810 / 0.812 / 0.803 / 0.796 / 0.785 / 0.729 | unchanged |
| sensor | HAPS alone, σ_b = 3° | 0.531 / 0.527 / 0.520 / 0.518 / 0.506 / 0.447 | unchanged | 0.528 / 0.525 / 0.518 / 0.516 / 0.504 / 0.444 | unchanged |
| sensor | satellite alone | 0.176 / 0.177 / 0.175 / 0.176 / 0.176 / 0.172 | unchanged | 0.174 / 0.175 / 0.173 / 0.175 / 0.174 / 0.170 | unchanged |
| sensor | hybrid, ideal | 0.925 / 0.938 / 0.956 / 0.942 / 0.907 / 0.877 | 0.950 / 0.958 / 0.971 / 0.956 / 0.908 / 0.877 | 0.867 / 0.881 / 0.904 / 0.899 / 0.904 / 0.873 | 0.894 / 0.910 / 0.931 / 0.917 / 0.905 / 0.874 |
| sensor | hybrid, σ_b = 3° | 0.812 / 0.844 / 0.889 / 0.863 / 0.778 / 0.744 | 0.879 / 0.896 / 0.931 / 0.896 / 0.778 / 0.744 | 0.667 / 0.700 / 0.766 / 0.756 / 0.772 / 0.738 | 0.743 / 0.771 / 0.838 / 0.802 / 0.773 / 0.739 |
| sensor | TN + satellite | 0.606 / 0.662 / 0.772 / 0.711 / 0.539 / 0.538 | 0.746 / 0.781 / 0.853 / 0.777 / 0.540 / 0.538 | 0.301 / 0.363 / 0.516 / 0.493 / 0.527 / 0.529 | 0.459 / 0.518 / 0.661 / 0.583 / 0.531 / 0.531 |

TN first below 50%: drones never (minimum 0.950 at 5 km); sensors 3.5 km → 4.0 km.

**What remains of the sensor rise, and why it is physical.** Sensor TN still rises from 0.697 at the
hub to 0.834 at 2.5 km (core edge) and then falls to ≈ 0.44 in the rural area. With α = 1 every
power-controlled user, the sensor included, is received at −90 dBm, so in the reuse-1 core the sensor's
SINR is set only by its serving BS's active co-channel neighbours: the hub BS is surrounded on all sides
(12.5 BSs within 1 km), a core-edge BS sees interference from one side only (≈ 5–6), hence the rise; beyond
the core the sensor reaches its 23 dBm cap and becomes noise-limited by the sparse rural BS spacing.

**Exp 3 numbers that change** (TN only; old → new). Nominal: all-TN 42.5→96.0 ms, 0.3% → 38.4→93.2 ms,
0.0%; terrestrial-first 42.4→28.6 → 38.4→28.5 ms; load-aware 19.1→25.8 ms unchanged; conservative
19.1→26.3 → 19.1→26.1 ms. Equal 5 MHz: all-TN 14.0→69.2 ms, 0.2% → 13.2→68.2 ms, 0.0%. High IoT load
(N = 50 latency, outage): all-TN 263.7 ms, 5.8% → 261.8 ms, 5.4%; load-aware 105.3 ms, 2.2% → 101.7 ms,
2.1%; conservative 107.1 → 105.6 ms, 2.1%. HAPS RA 16 + high IoT load: all-TN 263.5 ms, 5.9% → 259.2 ms,
5.4%; all-HAPS 204.6 ms, 4.2% → 209.5 ms, 4.3%; satellite 179.0 ms, 3.0% → 181.1 ms, 2.9%; load-aware
139.1 → 137.4 ms, 2.8%; conservative 138.5 → 138.1 ms, 2.3%; terrestrial-first 162.2 ms, 3.3% →
161.6 ms, 3.4% (HAPS and satellite rows change only through Monte Carlo draws shared with TN).
Exp 5 TN median rate 15.31 → 16.20 Mbit/s.

## Changes in v1.1: Exp 2c, drones as aerial IoT gateways (addition only)

**Model.** Two-hop path: ground sensor (23 dBm, 0 dBi, TR 36.763 Table 6.1-1) → nearest drone of a
Thomas-cluster swarm over a 12 km disk around the hub (drones at 100 m) → backhaul from that drone to TN,
HAPS (ideal pointing) or satellite with the existing drone radio (30 dBm, 10 dBi, same models as Exp 2).
End-to-end success = sensor → drone SINR > 5 dB and best backhaul SINR > 5 dB.

- **Sensor → drone channel**: 3GPP TR 36.777 v15.0.0 Annex B, values verified in the document:
  Table B-1 LoS probability P = 1 for d2D ≤ d1, else d1/d2D + exp(−d2D/p1)(1 − d1/d2D); UMa-AV
  (22.5 < h ≤ 100 m) d1 = max(460 log10 h − 700, 18), p1 = 4300 log10 h − 3800; RMa-AV (40 < h ≤ 300 m)
  d1 = max(1350.8 log10 h − 1602, 18), p1 = max(15021 log10 h − 16053, 1000). Table B-2 path loss: UMa-AV
  LoS 28.0 + 22 log10 d3D + 20 log10 fc; UMa-AV NLoS −17.5 + (46 − 7 log10 h) log10 d3D + 20 log10(40π fc/3);
  RMa-AV LoS max(23.9 − 1.8 log10 h, 20) log10 d3D + 20 log10(40π fc/3); RMa-AV NLoS max(PL_LoS,
  −12 + (35 − 5.3 log10 h) log10 d3D + 20 log10(40π fc/3)). UMa-AV is used for sensors inside the 2 km urban
  core, RMa-AV outside. **Deviations**: TR 36.777 derives these for a ground BS at 25 m (UMa) / 35 m (RMa)
  and an aerial UE; here the ground end is a sensor at 0 m. UMa-AV path loss is specified for d2D ≤ 4 km
  and RMa-AV for ≤ 10 km; longer interferer links use the same formulas. Shadowing std 4 dB LoS / 6 dB
  NLoS is a PLACEHOLDER (TR 36.777 Table B-3 points to TR 38.901, which was not read). Nakagami-m (m = 2)
  fading, 1 MHz channel at 2 GHz, 7 dB noise figure at the drone, no power control on this hop.
- **Interference**: one co-channel sensor per other drone, placed uniformly inside that drone's
  nearest-drone cell, active with the same p_act (0.393), all on the same 1 MHz resource.
- **Latency of the via-drone path**: one RA opportunity (6.4 ms, no contention) + 0.5 Mbit at the
  sensor → drone Shannon rate + propagation, plus the same for the lowest-latency feasible backhaul tier.
  The direct path uses the lowest-latency feasible direct tier.
- **Densities** low / nominal / high = 2 / 10 / 50 drones/km² (PLACEHOLDER; 10 = the Exp 4 swarm).

**Results** (seed means, 5 × 500 realizations, coverage at 0 / 2 / 5 / 10 km):

| Curve | 2 drones/km² | 10 drones/km² | 50 drones/km² |
|---|---|---|---|
| direct, hybrid (TN+HAPS+sat) | 0.942 / 0.970 / 0.898 / 0.880 | 0.948 / 0.966 / 0.893 / 0.885 | 0.944 / 0.968 / 0.912 / 0.875 |
| direct, TN + sat (no HAPS) | 0.746 / 0.847 / 0.527 / 0.534 | 0.750 / 0.856 / 0.547 / 0.540 | 0.752 / 0.838 / 0.557 / 0.545 |
| via drone only | 0.010 / 0.006 / 0.006 / 0.007 | 0.006 / 0.005 / 0.003 / 0.003 | 0.000 / 0.000 / 0.000 / 0.000 |
| via drone, TN/sat backhaul | 0.010 / 0.006 / 0.006 / 0.007 | 0.006 / 0.005 / 0.003 / 0.003 | 0.000 / 0.000 / 0.000 / 0.000 |
| direct + via drone (hybrid) | 0.943 / 0.970 / 0.898 / 0.880 | 0.949 / 0.966 / 0.893 / 0.885 | 0.944 / 0.968 / 0.912 / 0.875 |
| direct + via drone, no HAPS | 0.749 / 0.848 / 0.529 / 0.537 | 0.753 / 0.857 / 0.548 / 0.542 | 0.752 / 0.838 / 0.557 / 0.545 |
| drones on top of hybrid | +0.001 / +0.000 / +0.000 / +0.000 | +0.000 / +0.000 / +0.000 / +0.000 | +0.000 / +0.000 / +0.000 / +0.000 |
| no-HAPS + drones − hybrid | −0.193 / −0.122 / −0.369 / −0.343 | −0.195 / −0.110 / −0.345 / −0.343 | −0.192 / −0.130 / −0.355 / −0.330 |
| median latency, via drone / direct (covered samples) | 223.9 / 39.4 ms | 229.8 / 39.3 ms | 248.9 / 39.5 ms |

(The direct curves differ slightly from Exp 2b because Exp 2c draws its own realizations.) In the figure
the combined curves lie on top of the direct ones, because drones add nothing.

**Why the via-drone path fails (diagnostic, 150 realizations per density).** The sensor → drone hop is
not range- or noise-limited: median SNR without interference 40–57 dB, P[SNR > 5 dB] ≥ 0.98; median
hop distance 860–880 m / 320–370 m / 130–140 m. With interference the median SINR is −18 to −23 dB,
−14 to −16 dB and −13 to −14 dB: about 360 / 1800 / 8900 co-channel sensors are active on the same
1 MHz, and a drone at 100 m sees most of them with high LoS probability (the same aerial-interference
effect TR 36.777 reports for aerial UEs). The backhaul succeeds for 100% of serving drones. A higher
density brings the drone closer but adds interferers in proportion, so via-drone coverage stays near
zero and falls slightly with density (0.006 → 0.005 → 0.000 averaged over distance); the
`via_drone_non_decreasing_in_density` check passes only within its 0.01 tolerance. The latency is
dominated by sending 0.5 Mbit over 1 MHz at an SINR just above the threshold (≈ 220 ms).

## Changes in v1.2: Exp 2c sensor → drone reuse sensitivity (addition only)

**Model.** The 1 MHz sensor → drone band is split into K orthogonal sub-channels of 1 MHz / K
(K = 1, 4, 8, 16), so noise power and Shannon rate scale with 1/K. Each drone uses one sub-channel:
either chosen uniformly at random, or by nearest-neighbour colouring (K = 4, 8: greedy colouring of the
symmetric (K − 1)-nearest-neighbour graph, drones in index order, so adjacent drones never share a
sub-channel; residual conflicts 2.67 / 2.49 / 1.94% of drones for K = 4 and 0.70 / 0.57 / 0.30% for K = 8
at 2 / 10 / 50 drones/km²). Only sensors served by drones on the same sub-channel interfere.
Everything else is as in v1.1; K = 1 is the v1.1 run itself.

- **Basis.** 3GPP TR 36.763 v17.0.0 Table 6.1-1 (read in this project): the NB-IoT uplink uses
  "up to 180 kHz with all permissible smaller resource allocations 12*15 kHz, 6*15 kHz, 3*15 kHz,
  1*15 kHz, 1*3.75 kHz", i.e. an uplink carrier split into orthogonal sub-allocations. **PLACEHOLDER**:
  the sub-channel sizes 1 MHz / K (250 / 125 / 62.5 kHz) are not NB-IoT allocation sizes; the random
  choice and the nearest-neighbour colouring rule are modelling choices with no 3GPP source read here.
- **Sample size.** K > 1 cases use 150 realizations per seed (750 per density) instead of 500.

**Results** (via-drone coverage at 0 / 2 / 5 / 10 km; median sensor → drone SINR; via-drone latency
median; drones on top of the hybrid and no-HAPS + drones − hybrid, both at 0 / 2 / 5 / 10 km):

| Reuse | Density | Via drone | SINR | Latency | On top of hybrid | No-HAPS + drones − hybrid |
|---|---|---|---|---|---|---|
| K = 1 | 2 | 0.010 / 0.006 / 0.006 / 0.007 | −20.1 dB | 223.9 ms | +0.001 / +0.000 / +0.000 / +0.000 | −0.193 / −0.122 / −0.369 / −0.343 |
| K = 1 | 10 | 0.006 / 0.005 / 0.003 / 0.003 | −14.7 dB | 229.8 ms | +0.000 / +0.000 / +0.000 / +0.000 | −0.195 / −0.110 / −0.345 / −0.343 |
| K = 1 | 50 | 0.000 / 0.000 / 0.000 / 0.000 | −13.3 dB | 248.9 ms | +0.000 / +0.000 / +0.000 / +0.000 | −0.192 / −0.130 / −0.355 / −0.330 |
| K = 4 random | 2 | 0.085 / 0.047 / 0.045 / 0.068 | −10.7 dB | 681.5 ms | +0.001 / +0.001 / +0.004 / +0.009 | −0.200 / −0.101 / −0.355 / −0.299 |
| K = 4 random | 10 | 0.052 / 0.072 / 0.047 / 0.061 | −7.7 dB | 768.2 ms | +0.001 / +0.000 / +0.007 / +0.004 | −0.189 / −0.112 / −0.340 / −0.331 |
| K = 4 random | 50 | 0.027 / 0.027 / 0.027 / 0.021 | −7.0 dB | 846.1 ms | +0.001 / +0.000 / +0.003 / +0.000 | −0.191 / −0.113 / −0.360 / −0.323 |
| K = 8 random | 2 | 0.164 / 0.120 / 0.128 / 0.208 | −5.5 dB | 1256.6 ms | +0.007 / +0.003 / +0.011 / +0.031 | −0.185 / −0.087 / −0.317 / −0.251 |
| K = 8 random | 10 | 0.128 / 0.159 / 0.131 / 0.159 | −4.0 dB | 1399.7 ms | +0.008 / +0.003 / +0.013 / +0.027 | −0.169 / −0.097 / −0.300 / −0.275 |
| K = 8 random | 50 | 0.091 / 0.088 / 0.075 / 0.080 | −3.8 dB | 1559.0 ms | +0.003 / +0.000 / +0.005 / +0.003 | −0.175 / −0.103 / −0.336 / −0.301 |
| K = 16 random | 2 | 0.289 / 0.247 / 0.285 / 0.384 | −0.5 dB | 2336.2 ms | +0.011 / +0.007 / +0.025 / +0.057 | −0.155 / −0.068 / −0.245 / −0.156 |
| K = 16 random | 10 | 0.271 / 0.267 / 0.243 / 0.317 | −0.2 dB | 2533.4 ms | +0.020 / +0.005 / +0.023 / +0.044 | −0.133 / −0.083 / −0.247 / −0.195 |
| K = 16 random | 50 | 0.245 / 0.216 / 0.216 / 0.200 | −0.4 dB | 2912.3 ms | +0.009 / +0.005 / +0.017 / +0.015 | −0.141 / −0.083 / −0.283 / −0.249 |
| K = 4 colouring | 2 | 0.083 / 0.040 / 0.073 / 0.064 | −10.2 dB | 684.4 ms | +0.004 / +0.003 / +0.008 / +0.004 | −0.173 / −0.105 / −0.359 / −0.316 |
| K = 4 colouring | 10 | 0.073 / 0.080 / 0.068 / 0.079 | −7.2 dB | 757.5 ms | +0.005 / +0.005 / +0.003 / +0.009 | −0.169 / −0.101 / −0.317 / −0.319 |
| K = 4 colouring | 50 | 0.025 / 0.027 / 0.020 / 0.023 | −7.0 dB | 833.4 ms | +0.000 / +0.003 / +0.004 / +0.004 | −0.181 / −0.136 / −0.355 / −0.336 |
| K = 8 colouring | 2 | 0.169 / 0.147 / 0.164 / 0.204 | −4.2 dB | 1258.8 ms | +0.009 / +0.008 / +0.013 / +0.029 | −0.149 / −0.092 / −0.315 / −0.240 |
| K = 8 colouring | 10 | 0.172 / 0.191 / 0.139 / 0.188 | −3.1 dB | 1386.8 ms | +0.008 / +0.008 / +0.007 / +0.021 | −0.148 / −0.088 / −0.279 / −0.265 |
| K = 8 colouring | 50 | 0.076 / 0.088 / 0.079 / 0.076 | −3.8 dB | 1554.4 ms | +0.004 / +0.007 / +0.005 / +0.015 | −0.171 / −0.124 / −0.324 / −0.312 |

(Densities in drones/km². Direct-path latency is ≈ 39 ms median in every case.)

- Orthogonal reuse raises the via-drone hop from SINR ≈ −13 to −20 dB (K = 1) to ≈ −0.2 to −0.5 dB (K = 16)
  and via-drone coverage from ≤ 0.010 to 0.20–0.38, but the median SINR stays below the +5 dB threshold
  in every case and the narrower sub-channel stretches the 0.5 Mbit hop to 1.3–2.9 s (K = 8–16).
- Nearest-neighbour colouring vs random reuse at the same K: −0.015 to +0.044 per point; averaged over
  0–10 km +0.000 to +0.024 (largest at K = 8, 10 drones/km²), i.e. a small gain, comparable to the
  Monte Carlo uncertainty of 750 realizations, and none at 50 drones/km².
- Higher density still lowers via-drone coverage at every K (K = 16: 0.29 → 0.27 → 0.22 at 0 km).
- Drones add at most +0.057 on top of the hybrid (K = 16, 2 drones/km², 10 km); no-HAPS + drones stays
  0.068–0.369 below the hybrid in every case.

**Density-trend check fixed.** `via_drone_non_decreasing_in_density` previously passed with a 0.01
tolerance that hid a decrease. It now allows each density step to fall by at most 3 binomial standard
errors of the pooled means (52,500 samples per density) and logs the trend: via-drone coverage (K = 1,
mean over 0–10 km) 0.0059 / 0.0046 / 0.0005 at 2 / 10 / 50 drones/km², steps −0.0013 (flat within 3 SE)
and −0.0041 (decreasing, SE 0.0003) → the check now **fails**. The pytest version uses the same rule
(3 SE, no fixed floor) on 1,800 samples per density; at that size it cannot resolve steps this small and
passes, so the full-run check is the one that reports the decrease.

## Experiment-specific choices

- Exp 1 runs on a torus window (no edge effects) of side clip(√(2000/λ) km, 3, 10), 3 realizations
  per density per seed. r0 = median SG edge length at the lowest density (per seed).
- Exp 3 and Exp 5 use ideal HAPS pointing (σ_b = 0); Exp 2 shows the cost of σ_b = 3°.
- Exp 3 equal-bandwidth variant sets all tiers to 5 MHz (noise scales with B too).
- Every stochastic experiment: seeds 1–5, curves show the seed mean with a min–max band.

## Results

See `results/run_log.txt` and `results/metrics.json`. The summary of the latest run is in
the section below.

### Latest run (v1.2; seeds 1–5, seed mean)

| Metric | Value |
|---|---|
| Exp 1 SG degree peak | 8.762 at 4 drones/km²; 6.061 at 250/km² (30.8% below peak) |
| Exp 1 distance rule | r0 = 179.0 m; degree 5.481 → 31.229 |
| Exp 2 drones at 0 / 2 / 5 / 10 km | TN 1.000 / 1.000 / 0.950 / 0.971; HAPS 1.000 / 1.000 / 1.000 / 0.998 (σ_b=3°: 0.962 / 0.961 / 0.960 / 0.953); sat 0.998 / 0.998 / 0.998 / 0.998; hybrid 1.000 everywhere; TN + sat 1.000 everywhere |
| Exp 2 drone gain 0 / 3 / 10 dBi at 10 km | satellite 0.748 / 0.913 / 0.998; TN + satellite 0.932 / 0.987 / 1.000; hybrid 1.000 / 1.000 / 1.000 |
| Exp 2b sensors at 0 / 2 / 5 / 10 km | TN 0.697 / 0.819 / 0.442 / 0.443; HAPS 0.812 / 0.805 / 0.787 / 0.733 (σ_b=3°: 0.531 / 0.520 / 0.506 / 0.447); sat 0.176 / 0.175 / 0.176 / 0.172; hybrid 0.950 / 0.971 / 0.908 / 0.877 (σ_b=3°: 0.879 / 0.931 / 0.778 / 0.744); TN + sat 0.746 / 0.853 / 0.540 / 0.538 |
| Exp 2b sensors at 10 km: hybrid / no HAPS at −5 / 0 / +5 dB | 1.000 / 0.987, 0.996 / 0.857, 0.877 / 0.538 |
| Median uplink rate (exp5) | TN 16.20, HAPS 41.09, satellite 6.20 Mbit/s |
| Exp 3 latency N=1→50, outage (drone-only RA) | TN 38.4→93.2 ms, 0.0%; HAPS 19.1→25.9 ms, 0.0%; sat 92.6→101.5 ms, 0.3%; terrestrial-first 38.4→28.5 ms; load-aware (conservative) 19.1→26.1 ms; load-aware 19.1→25.8 ms; 0.0% outage |
| Exp 3 high IoT load (1.0, p_act 0.632): outage, latency at N=50 | TN 5.4%, 261.8 ms; HAPS 2.2%, 108.0 ms; sat 3.0%, 182.0 ms; terrestrial-first 2.6%, 105.8 ms; load-aware (conservative) 2.1%, 105.6 ms; load-aware 2.1%, 101.7 ms |
| Exp 3, HAPS RA 16, drone-only: latency at N=50 | HAPS 56.4 ms; terrestrial-first 43.0 ms; load-aware (conservative) 52.4 ms; load-aware 43.1 ms |
| Exp 3, HAPS RA 16, high IoT load: outage (RA failure + over limit), latency at N=50 | TN 5.4% (5.4%), 259.2 ms; HAPS 4.3% (4.3%), 209.5 ms; sat 2.9% (2.5%), 181.1 ms; terrestrial-first 3.4% (3.4%), 161.6 ms; load-aware (conservative) 2.3% (2.3%), 138.1 ms; load-aware 2.8% (2.8%), 137.4 ms |
| Exp 2c via drone (sensor → drone → TN/HAPS/sat), mean over 0–10 km at 2 / 10 / 50 drones/km² | 0.006 / 0.005 / 0.000; drones add ≤ +0.001 on top of the hybrid; no-HAPS + drones is 0.11–0.37 below the hybrid |
| Exp 2c reuse sensitivity: via drone, mean over 0–10 km, K = 1 / 4 / 8 / 16 random at 10 drones/km² | ≈ 0.005 / 0.06 / 0.14 / 0.27; via latency median 230 / 768 / 1400 / 2533 ms |
| Exp 4 coop completion at 1000 / 2000 GFLOP | 98.2% / 91.0% (solo: step at 470 GFLOP) |
| Exp 5 raw streaming | needs 80 Mbit/s; feasible for 0.0% of drones on every tier; ≤ 51.4% of raw delivered (HAPS) |
| Exp 5 median endurance | raw 69.4 min, edge-AI 68.3 min (ceiling 74.0 min) |

### Known sanity-check failures (not fixed, reported as found)

- `tn_non_increasing` and `tn_max_at_hub` (exp2b), test `test_tn_coverage_maximal_at_hub` (sensor
  case): sensor TN 0.697 at the hub, peak 0.834 at 2.5 km. Physical with α = 1 power control in a
  reuse-1 core (revision 8). Drone TN passes both checks.
- `via_drone_non_decreasing_in_density` (exp2c): via-drone coverage falls with density (0.0059 →
  0.0046 → 0.0005, last step −0.0041 vs SE 0.0003). Physical: more drones bring the serving drone
  closer but add co-channel interferers in proportion (see v1.1 / v1.2 sections).
- `naive_monotone_non_decreasing` (exp1 only): one dip 6.062 → 5.879 between 4 and 6 drones/km²,
  Monte Carlo noise at low density (only 15 realizations per density). A 60-realization re-check
  showed the step increasing.

## Claims (v1.2)

| Claim | Verdict | Numbers |
|---|---|---|
| (a) SG graph saturates while distance rule grows | holds | SG degree peaks 8.762 at 4/km², falls to 6.061 at 250/km² (−30.8%); distance rule 5.481 → 31.229 |
| (b) HAPS is needed for coverage (drones) | does not hold | TN + satellite 1.000 at every distance and threshold (hybrid 1.000); ≥ 0.999 at p_act = 1. A 0 dBi drone gets 0.932 at 10 km without HAPS vs 1.000 with it (3 dBi: 0.987 vs 1.000) |
| (c) HAPS is needed for latency/throughput | holds at nominal bandwidths; a bandwidth effect | all-HAPS 19.1→25.9 ms vs TN 38.4→93.2 ms and satellite 92.6→101.5 ms; HAPS median 41.09 Mbit/s vs TN 16.20, sat 6.20 |
| (d) coordination prevents congestion collapse | does not hold as stated; coordination reduces latency when HAPS is congested | no collapse in any case (worst: all-TN 9.4% outage at N=50, HAPS RA 16 + high IoT load). HAPS RA 16, drone-only: load-aware 43.1 ms at N=50 vs all-HAPS 56.4 ms (terrestrial-first 43.0 ms). HAPS RA 16 + high IoT load: load-aware 2.8% outage, 137.4 ms (conservative 2.3%, 138.1 ms) vs best single tier (satellite) 2.9%, 181.1 ms; terrestrial-first 3.4%, 161.6 ms |
| (e) cooperation beats solo | holds | 91.0% vs 0% completion at 2000 GFLOP |
| (f) edge-AI is needed for data delivery | holds (if the 0.02 Mbit report carries the needed information) | raw feasible for 0.0% of drones on every tier; ≤ 51.4% of raw delivered (HAPS); report feasible 100% |
| (g) edge-AI saves energy | does not hold | endurance edge-AI 68.3 min vs raw 69.4 min |
| (h) HAPS is needed for coverage of ground IoT sensors | holds at +5 and 0 dB; small margin at −5 dB | +5 dB, hybrid vs TN + sat: 0.950 vs 0.746 at the hub, 0.971 vs 0.853 at 2 km, 0.908 vs 0.540 at 5 km, 0.877 vs 0.538 at 10 km (σ_b = 3°: 0.744 at 10 km). 10 km at 0 dB: 0.996 vs 0.857; −5 dB: 1.000 vs 0.987 |
| (i) drones as aerial IoT gateways extend sensor coverage or replace the HAPS | does not hold; orthogonal reuse helps the drone hop but not enough | K = 1 (shared 1 MHz): via-drone ≤ 0.010, +0.000–0.001 on top of the hybrid. With K = 16 orthogonal sub-channels via-drone coverage rises to 0.20–0.38 (median hop SINR −0.2 to −0.5 dB) but adds at most +0.057 on top of the hybrid, and no-HAPS + drones stays 0.068–0.369 below the hybrid at every K and density (K = 16, 10 drones/km², 10 km: −0.195). Nearest-neighbour colouring adds +0.000 to +0.024 over random reuse (mean over 0–10 km). Via-drone latency 224–249 ms (K = 1) to 2.3–2.9 s (K = 16) vs ≈ 39 ms direct. More drones lower via-drone coverage at every K |
