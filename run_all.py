"""Run every SAGIN simulation and save figures + a log to ./results.
Usage:  python run_all.py            (all figures)
        python run_all.py fig3 fig4  (only some)
"""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results"); os.makedirs(OUT, exist_ok=True)
SCRIPTS = {
 "fig2": "fig2_degree_vs_density.py",
 "fig3": "fig3_coverage_vs_distance.py",
 "fig4": "fig4_latency_vs_density.py",
 "fig5a": "fig5a_completion_vs_complexity.py",
 "fig5b": "fig5b_battery_vs_time.py",
}
want = sys.argv[1:] or list(SCRIPTS)
env = dict(os.environ, SAGIN_OUT=OUT, MPLBACKEND="Agg")
log = open(os.path.join(OUT, "run_log.txt"), "w")
ok = True
for k in want:
    t = time.time()
    r = subprocess.run([sys.executable, SCRIPTS[k]], cwd=HERE, env=env,
                       capture_output=True, text=True)
    log.write(f"===== {k} ({time.time()-t:.0f}s) exit={r.returncode}\n{r.stdout}{r.stderr}\n")
    print(f"{k}: {'OK' if r.returncode==0 else 'FAILED'}  ({time.time()-t:.0f}s)")
    ok &= r.returncode == 0
log.close()
print("log:", os.path.join(OUT, "run_log.txt"))
sys.exit(0 if ok else 1)
