"""Exp 2b (Fig. 3c): ground IoT sensor coverage vs. distance from the hub, per tier and hybrid."""
from experiments.exp2_coverage import study


def run(world, seeds, outdir, quick=False, log=print):
    p = world.p
    log(f"ground IoT sensor terminal: height {p.h_sensor_m:g} m, Tx {p.p_sensor_dbm:g} dBm, "
        f"antenna gain {p.g_sensor_dbi:g} dBi (all other parameters as exp2)")
    return study(world.ground_sensor(), seeds, 300 if quick else 2000, outdir, "fig3c_iot_coverage_vs_distance.png",
                 "fig3d_iot_threshold_sweep.png", "sensor distance from hub (km)", log)
