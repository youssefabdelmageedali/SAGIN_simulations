# SAGIN simulation project

Stochastic-geometry (SG) simulation for "A Unified SAGIN Architecture for Autonomous
Drone Swarms and Massive IoT". Central claim: the communication graph is DERIVED from
SINR (edge iff SINR_ij > gamma_th), not assumed from distance.

## Layout
- `sagin/world.py`  : the ONLY physics engine (class SAGINWorld, dataclass Params, PLACEHOLDERS).
- `experiments/exp*.py`: experiment scripts. They call SAGINWorld and must NOT re-implement physics.
- `run_all.py`      : runs everything, writes PNGs + `results/run_log.txt` + `results/metrics.json`.
- `tests/test_world.py`: pytest sanity tests (`python -m pytest -q tests`).

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

## Reference results
The numbers previously listed here came from the old implementation (removed) and do not
apply to the from-scratch rebuild. Current results: `results/run_log.txt`, `results/metrics.json`.
