"""Shared plotting style, seeding and summary helpers (no physics here)."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLORS = {
    "tn": "#7f7f7f",
    "haps": "#2ca02c",
    "sat": "#9467bd",
    "hybrid": "#1f77b4",
    "drone": "#ff7f0e",
    "ref": "#000000",
}
FIGSIZE = (3.5, 2.6)

plt.rcParams.update({
    "font.size": 8,
    "axes.labelsize": 8,
    "legend.fontsize": 6.5,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "lines.linewidth": 1.3,
    "lines.markersize": 3,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "savefig.dpi": 300,
})


def make_rng(seed, exp_id, *extra):
    return np.random.default_rng([int(seed), int(exp_id), *map(int, extra)])


def band(ax, x, ys, color, label, **kw):
    """Plot the mean over seeds (rows of ys) with a shaded min-max band."""
    ys = np.asarray(ys)
    ax.fill_between(x, ys.min(0), ys.max(0), color=color, alpha=0.2, linewidth=0)
    return ax.plot(x, ys.mean(0), color=color, label=label, **kw)[0]


def save(fig, path):
    """Save after checking the figure rules: no titles, every legend entry named."""
    has_legend = False
    for ax in fig.axes:
        assert not ax.get_title(), f"{path}: figures must not carry a title"
        leg = ax.get_legend()
        if leg is not None:
            has_legend = True
            for t in leg.get_texts():
                assert t.get_text() and not t.get_text().startswith("_"), f"{path}: unnamed legend entry"
    assert has_legend or fig.legends, f"{path}: missing legend"
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def spread(values):
    """mean / min / max over seeds."""
    v = np.asarray(values, dtype=float)
    return {"mean": float(v.mean()), "min": float(v.min()), "max": float(v.max())}
