# SAGIN simulation project

Stochastic-geometry (SG) simulation for "A Unified SAGIN Architecture for Autonomous
Drone Swarms and Massive IoT". Central claim: the communication graph is DERIVED from
SINR (edge iff SINR_ij > gamma_th), not assumed from distance.

## Layout
- `sagin_world.py`  : the ONLY physics engine (class SAGINWorld, dataclass Params).
- `fig2..fig5b_*.py`: experiment scripts. They call SAGINWorld and must NOT re-implement physics.
- `run_all.py`      : runs everything, writes PNGs + `results/run_log.txt`.

## Rules
- Run with `python run_all.py`. Output goes to `results/`.
- Never change a Params value without saying so; many are literature-matched
  (Lou et al. arXiv:2310.09659; NPRACH 12 resources / 6.4 ms; Jetson Nano 470 GFLOPS;
  HAPS 5 MHz (HIBS); satellite 1.5 MHz (IoT-NTN Cat M1); battery 148 Wh).
- Placeholders (say so in reports): payload sizes (raw 800 Mbit, report 0.02, packet 0.5),
  HAPS/Sat random-access resources (64), HAPS beam width, shadow std.
- Every figure script prints sanity checks; if one prints False/mismatch, stop and report.
- Figures have NO baked-in titles (the paper adds captions).
- Report numbers exactly as the logs print them. Do not round up or invent results.

## Expected reference results (seed-fixed; small drift is fine, big drift is a bug)
- Fig2: SG degree peak ~6.0 at ~10 drones/km2, then ~4.7; distance rule keeps climbing.
- Fig3: TN dies ~3 km; sat alone ~47%; HAPS alone ~90% ideal / ~64% at sigma_b=3 deg;
  hybrid 93% / 78% at 10 km.
- Fig4: all-TN 106->552 ms (2% outage); all-HAPS 38->45 (8.3%); all-Sat 212->225 (52.9%);
  balanced 102->123 ms (0.1%).
- Fig5a: solo wall at 470 GFLOP; coop 100% -> 73.4% at 2000 GFLOP.
- Fig5b: raw->Sat dies ~43 min; raw->HAPS ~63; edge-AI ~73 (flight-only ceiling 74 min).
