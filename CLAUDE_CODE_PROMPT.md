# Paste this into Claude Code (run it from inside this folder)

Read CLAUDE.md first. Then:

1. `pip install -r requirements.txt`
2. Run `python run_all.py` and wait for it to finish.
3. Open `results/run_log.txt`. Compare every number against the "Expected reference
   results" in CLAUDE.md and give me a table: metric | expected | got | match?
4. View each PNG in `results/` and tell me if anything looks wrong (legend, axes,
   curves in the wrong order, empty plot).
5. If a script fails or a number is far off, find the cause in the code, fix it in the
   smallest possible way, re-run only that figure (`python run_all.py fig4`), and tell me
   exactly what you changed. Do not silently change Params values.
6. Finish with a short summary: what ran, what matched, what changed, what is still a placeholder.

Optional extras (only if I ask): (a) run each figure with 5 different seeds and report
the spread; (b) a no-HAPS ablation of Fig. 3 (TN + satellite only vs TN + HAPS + satellite)
to demonstrate the HAPS tier is necessary; (c) a sensitivity sweep of the satellite and
HAPS bandwidths.
