"""Exp 2b (Fig. 3, panel b): ground IoT sensor coverage vs. distance from the hub, per tier and hybrid."""
from experiments.exp2_coverage import make_figures, summarize


def run(world, seeds, outdir, quick=False, log=print):
    p = world.p
    n_real = 300 if quick else 2000
    log(f"ground IoT sensor terminal: height {p.h_sensor_m:g} m, Tx {p.p_sensor_dbm:g} dBm, "
        f"antenna gain {p.g_sensor_dbi:g} dBi (all other parameters as exp2)")
    out = summarize(world.ground_sensor(), seeds, n_real, "ground IoT sensors", log)
    make_figures(world, seeds, n_real, outdir)  # cached: same two-panel figures as exp2
    return out
