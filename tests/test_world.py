"""Sanity tests for the SAGIN physics engine and experiments (section 7 of the spec)."""
import filecmp
from dataclasses import replace

import numpy as np

import run_all
from experiments import exp2_coverage, exp3_latency
from experiments.common import make_rng
from sagin.world import SAGINWorld

WORLD = SAGINWorld()
DIST = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 10.0])


def test_tn_coverage_non_increasing():
    cov = exp2_coverage.coverage(WORLD, 11, DIST, 1500)["tn"]
    assert np.all(np.diff(cov) <= 0.03), cov  # Monte Carlo tolerance


def test_hybrid_dominates_every_tier():
    cov = exp2_coverage.coverage(WORLD, 12, DIST, 300)
    for k in ["tn", "haps_ideal", "sat"]:
        assert np.all(cov["hybrid_ideal"] >= cov[k])
    for k in ["tn", "haps_err", "sat"]:
        assert np.all(cov["hybrid_err"] >= cov[k])


def test_sinr_drops_when_interferers_added():
    rng = make_rng(13, 0)
    n, extra = 20, 10
    xy = rng.uniform(0, 1000, (n + extra, 2))
    fading = WORLD.nakagami(rng, (n + extra, n + extra))
    ch = np.zeros(n + extra, dtype=int)  # everyone on one sub-channel
    small = WORLD.mesh_sinr(xy[:n], rng, fading=fading[:n, :n], channels=ch[:n])
    big = WORLD.mesh_sinr(xy, rng, fading=fading, channels=ch)[:n, :n]
    off = ~np.eye(n, dtype=bool)
    assert np.all(big[off] < small[off])
    # the generic link formula as well
    assert WORLD.sinr(1.0, 2.0, 0.1) < WORLD.sinr(1.0, 1.0, 0.1)


def test_fixed_policy_latency_non_decreasing():
    n_grid = [1, 10, 25, 50]
    for pol in ["all_tn", "all_haps"]:
        lat = np.mean([exp3_latency.simulate(WORLD, s, n_grid, 30, policies=[pol])[pol]["lat"] for s in (1, 2)], 0)
        assert np.all(np.diff(lat) >= -0.05 * lat[:-1]), (pol, lat)


def test_battery_below_flight_ceiling():
    t = np.linspace(0, 100 * 60, 1001)
    ceiling = np.clip(100 * (1 - t / WORLD.flight_ceiling_s()), 0, 100)
    rates = np.array([1e3, 1e6, 1e8, np.inf])
    for payload in (0.02, 800):
        for compute in (False, True):
            curves = WORLD.battery_percent(t, WORLD.cycle_energy_j(payload, rates, compute))
            assert np.all(curves <= ceiling + 1e-9)


def test_solo_completion_step():
    cap = WORLD.p.drone_gflops * WORLD.p.task_deadline_s
    below = np.linspace(0, cap, 50)
    above = np.linspace(cap + 1e-6, 5 * cap, 50)
    assert np.all(WORLD.solo_completes(below))
    assert not np.any(WORLD.solo_completes(above))


def test_pointing_error_never_helps():
    cov = exp2_coverage.coverage(WORLD, 14, DIST, 500)
    assert np.all(cov["haps_err"] <= cov["haps_ideal"])


def test_same_seed_reproduces_metrics(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    run_all.main(["--quick", "--out", str(a)])  # exit code reflects sanity checks, not reproducibility
    run_all.main(["--quick", "--out", str(b)])
    assert filecmp.cmp(a / "metrics.json", b / "metrics.json", shallow=False)


def test_sat_cn_matches_3gpp_calibration():
    # TR 36.763 sec. 6.2.2.1.1, case 3 (LEO-600, Set-1, PC3 23 dBm, 0 dBi, FSPL 159.7 dB,
    # losses 2.2 + 0.1 + 3 + 3 shadow margin + 3 beam edge): UL C/N -8.6 dB at 1080 kHz, 16.0 dB at 3.75 kHz
    assert abs(WORLD.sat_link_cn_db(23, 0, 159.7, 11.3, 1.08e6) - (-8.6)) < 0.1
    assert abs(WORLD.sat_link_cn_db(23, 0, 159.7, 11.3, 3.75e3) - 16.0) < 0.1


def test_iot_load_raises_access_delay():
    rng = make_rng(15, 0)
    tier = np.zeros((300, 20), dtype=int)
    det = np.ones_like(tier, dtype=bool)
    quiet = WORLD.random_access(tier, det, rng)
    busy = SAGINWorld(replace(WORLD.p, iot_ra_load=1.0)).random_access(tier, det, rng)
    assert np.nanmean(np.where(np.isfinite(busy), busy, np.nan)) > np.nanmean(np.where(np.isfinite(quiet), quiet, np.nan))
