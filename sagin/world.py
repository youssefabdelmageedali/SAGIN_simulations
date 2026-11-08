"""The single physics engine of the SAGIN simulation.

Every propagation, fading, interference, random-access, compute and energy
formula lives here. Experiment scripts only draw geometry and call these
methods; they never re-implement physics.

Conventions: positions in metres, hub at the origin (0, 0), powers in watts
unless a name ends in _dbm / _db / _dbi. SINR values are linear.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

C_LIGHT = 299_792_458.0  # m/s
R_EARTH = 6_371e3  # m


@dataclass(frozen=True)
class Params:
    # --- geometry -----------------------------------------------------------
    region_km: float = 10.0  # square region side, hub at the centre
    h_drone_m: float = 100.0
    h_bs_m: float = 25.0
    h_iot_m: float = 1.5
    h_haps_m: float = 20e3
    h_sat_m: float = 550e3
    # --- carriers -----------------------------------------------------------
    f_tn_hz: float = 2e9
    f_haps_hz: float = 20e9
    f_sat_hz: float = 20e9
    # --- transmit powers ----------------------------------------------------
    p_drone_access_dbm: float = 20.0
    p_drone_mesh_dbm: float = 10.0
    p_iot_dbm: float = 20.0
    # --- antenna gains ------------------------------------------------------
    g_drone_dbi: float = 3.0
    g_tn_dbi: float = 15.0
    g_haps_dbi: float = 25.0
    g_sat_dbi: float = 48.0
    g_iot_dbi: float = 0.0
    # --- fading / noise -----------------------------------------------------
    nakagami_m: float = 2.0
    shadow_std_db: float = 4.0
    noise_psd_dbm_hz: float = -174.0
    noise_figure_db: float = 7.0
    # --- bandwidths ---------------------------------------------------------
    b_tn_hz: float = 1e6
    b_haps_hz: float = 5e6
    b_sat_hz: float = 1.5e6
    b_mesh_hz: float = 20e6
    n_subch_mesh: int = 8
    # --- HAPS beam ----------------------------------------------------------
    haps_beamwidth_deg: float = 5.0  # 3 dB beamwidth of the Gaussian main lobe
    sidelobe_db: float = -30.0  # sidelobe floor relative to boresight (HAPS and sat)
    sigma_b_deg: float = 3.0  # pointing-error std used in the "with error" cases
    # --- deployments --------------------------------------------------------
    tn_core_radius_km: float = 1.5  # TN base stations: PPP inside this disk around the hub
    tn_density_km2: float = 3.0
    tn_interferer_radius_m: float = 300.0  # co-channel IoT uplink user around each other BS
    n_haps: int = 3
    n_sat: int = 720
    sat_min_elev_deg: float = 10.0
    n_sat_cochannel: int = 6
    # --- drone swarm (Thomas cluster process) -------------------------------
    pcp_mean_offspring: float = 10.0
    pcp_sigma_m: float = 100.0
    mission_radius_km: float = 2.0  # swarm parents for exp3/exp5: uniform in this disk
    mission_sigma_m: float = 200.0
    swarm_density_km2: float = 10.0  # exp4 swarm
    swarm_window_km: float = 3.0
    # --- thresholds ---------------------------------------------------------
    gamma_th_db: float = -5.0
    access_th_db: float = 5.0
    # --- random access / latency --------------------------------------------
    n_ra_tn: int = 12
    n_ra_haps: int = 64
    n_ra_sat: int = 64
    t_ra_s: float = 6.4e-3
    backoff_cap_exp: int = 6  # back-off window capped at 2**6 opportunities
    ra_max_attempts: int = 10
    latency_limit_s: float = 1.0
    tn_cap_margin: int = 1  # balanced policy keeps at most n_ra_tn + margin drones on TN
    # --- compute ------------------------------------------------------------
    drone_gflops: float = 470.0
    task_deadline_s: float = 1.0
    max_helpers: int = 5
    chunk_overhead_mbit: float = 0.2
    # --- energy -------------------------------------------------------------
    battery_wh: float = 148.0
    p_flight_w: float = 120.0
    p_compute_w: float = 10.0
    p_tx_w: float = 8.0
    raw_mbit: float = 800.0
    packet_mbit: float = 0.5
    report_mbit: float = 0.02
    capture_interval_s: float = 10.0


# Params fields that are not literature-matched, with a one-line reason.
PLACEHOLDERS = {
    "h_bs_m": "typical macro mast height, not tied to a reference",
    "p_iot_dbm": "co-channel IoT uplink interferer power",
    "shadow_std_db": "shadowed-Rician approximation for HAPS/sat",
    "haps_beamwidth_deg": "HAPS spot-beam 3 dB width",
    "sidelobe_db": "sidelobe floor for HAPS and satellite beams",
    "tn_core_radius_km": "TN deployed only in an urban core around the hub",
    "tn_density_km2": "TN base-station density",
    "tn_interferer_radius_m": "co-channel uplink user placement around other BSs",
    "n_haps": "number of HAPS over the region",
    "n_sat": "LEO constellation size for the spherical BPP",
    "sat_min_elev_deg": "minimum service elevation",
    "n_sat_cochannel": "co-channel users in other satellite beams",
    "pcp_mean_offspring": "Thomas-process mean cluster size",
    "pcp_sigma_m": "Thomas-process cluster spread",
    "mission_radius_km": "mission area for exp3/exp5 swarms",
    "mission_sigma_m": "cluster spread for exp3/exp5 swarms",
    "swarm_density_km2": "exp4 swarm density",
    "swarm_window_km": "exp4 torus window side",
    "gamma_th_db": "graph SINR threshold",
    "access_th_db": "access SINR threshold",
    "n_ra_haps": "HAPS random-access resources",
    "n_ra_sat": "satellite random-access resources",
    "backoff_cap_exp": "binary exponential back-off cap",
    "ra_max_attempts": "RA attempts before giving up",
    "latency_limit_s": "latency outage limit",
    "tn_cap_margin": "balanced-policy TN load margin",
    "task_deadline_s": "task deadline",
    "max_helpers": "helpers per task",
    "chunk_overhead_mbit": "per-helper transfer overhead",
    "p_tx_w": "radio transmit power draw",
    "raw_mbit": "raw sensor payload per capture",
    "packet_mbit": "uplink packet size (exp3)",
    "report_mbit": "edge-AI report size",
    "capture_interval_s": "capture cycle length",
}


def db2lin(x):
    return 10.0 ** (np.asarray(x, dtype=float) / 10.0)


def lin2db(x):
    return 10.0 * np.log10(x)


def dbm2w(x):
    return 10.0 ** ((np.asarray(x, dtype=float) - 30.0) / 10.0)


@dataclass
class Deployment:
    """One realization of the infrastructure point processes."""
    bs_xy: np.ndarray  # (K, 2) TN base stations (PPP in the urban core)
    iot_xy: np.ndarray  # (K, 2) one co-channel uplink IoT user per BS
    haps_xyz: np.ndarray  # (H, 3) HAPS (BPP over the region)
    haps_intf_xy: np.ndarray  # (H, 2) one co-channel user per HAPS
    sat_cos_psi: float  # geocentric angle cosine of the highest-elevation satellite


class SAGINWorld:
    def __init__(self, params: Params | None = None):
        self.p = params or Params()

    # ------------------------------------------------------------------ basics
    def noise_w(self, bandwidth_hz):
        p = self.p
        return dbm2w(p.noise_psd_dbm_hz + 10 * np.log10(bandwidth_hz) + p.noise_figure_db)

    @staticmethod
    def sinr(signal_w, interference_w, noise_w):
        return signal_w / (interference_w + noise_w)

    @property
    def gamma_th(self):
        return float(db2lin(self.p.gamma_th_db))

    @property
    def access_th(self):
        return float(db2lin(self.p.access_th_db))

    def tier_bandwidths(self):
        """Bandwidth per tier, indexed TN=0, HAPS=1, SAT=2."""
        p = self.p
        return np.array([p.b_tn_hz, p.b_haps_hz, p.b_sat_hz])

    @staticmethod
    def shannon_rate(bandwidth_hz, sinr):
        return bandwidth_hz * np.log2(1.0 + sinr)

    # ------------------------------------------------------------- path loss
    @staticmethod
    def pl_tn_db(d_m):
        """3GPP-like macro log-distance path loss (2 GHz)."""
        return 128.1 + 37.6 * np.log10(np.maximum(d_m, 10.0) / 1e3)

    @staticmethod
    def pl_mesh_db(d_m):
        """Drone-to-drone mesh path loss."""
        return 40.0 + 25.0 * np.log10(np.maximum(d_m, 1.0))

    @staticmethod
    def fspl_db(d_m, f_hz):
        """Free-space path loss with 3-D distance."""
        return 20 * np.log10(d_m) + 20 * np.log10(f_hz) - 147.55

    # ---------------------------------------------------------------- fading
    def nakagami(self, rng, shape):
        """Nakagami-m power gain (unit mean)."""
        m = self.p.nakagami_m
        return rng.gamma(m, 1.0 / m, size=shape)

    def shadowing(self, rng, shape):
        """Log-normal shadowing power gain."""
        return db2lin(self.p.shadow_std_db * rng.standard_normal(shape))

    def haps_gain_db(self, theta_deg):
        """Gaussian main lobe G0*exp(-4 ln2 (theta/theta_3dB)^2), floored at the sidelobe level."""
        p = self.p
        main = p.g_haps_dbi - 40 * np.log10(2) * (np.asarray(theta_deg) / p.haps_beamwidth_deg) ** 2
        return np.maximum(main, p.g_haps_dbi + p.sidelobe_db)

    # -------------------------------------------------------------- geometry
    @staticmethod
    def _uniform_disk(n, radius_m, rng):
        r = radius_m * np.sqrt(rng.uniform(size=n))
        a = rng.uniform(0, 2 * np.pi, size=n)
        return np.column_stack([r * np.cos(a), r * np.sin(a)])

    def thomas_pcp(self, density_km2, side_m, rng):
        """Thomas cluster process (PPP parents, Gaussian offspring) on a side_m torus."""
        p = self.p
        area_km2 = (side_m / 1e3) ** 2
        n_par = rng.poisson(density_km2 / p.pcp_mean_offspring * area_km2)
        parents = rng.uniform(0, side_m, (n_par, 2))
        counts = rng.poisson(p.pcp_mean_offspring, n_par)
        centres = np.repeat(parents, counts, axis=0)
        pts = centres + p.pcp_sigma_m * rng.standard_normal(centres.shape)
        return np.mod(pts, side_m)

    def swarm_in_mission(self, n, rng):
        """n drones of a Thomas-like swarm (cluster parents uniform in the mission disk)."""
        p = self.p
        n_par = int(np.ceil(n / p.pcp_mean_offspring))
        parents = self._uniform_disk(n_par, p.mission_radius_km * 1e3, rng)
        owner = rng.integers(0, n_par, n)
        return parents[owner] + p.mission_sigma_m * rng.standard_normal((n, 2))

    @staticmethod
    def pairwise_dist(xy, side_m=None):
        """Pairwise horizontal distances; wrap-around when side_m (torus) is given."""
        d = np.abs(xy[:, None, :] - xy[None, :, :])
        if side_m is not None:
            d = np.minimum(d, side_m - d)
        return np.hypot(d[..., 0], d[..., 1])

    def deploy(self, rng):
        """Draw one realization of TN (PPP), HAPS (BPP) and satellites (spherical BPP)."""
        p = self.p
        r_core = p.tn_core_radius_km * 1e3
        n_bs = rng.poisson(p.tn_density_km2 * np.pi * p.tn_core_radius_km ** 2)
        bs = self._uniform_disk(n_bs, r_core, rng)
        iot = bs + self._uniform_disk(n_bs, p.tn_interferer_radius_m, rng)
        half = p.region_km * 1e3 / 2
        haps = np.column_stack([rng.uniform(-half, half, (p.n_haps, 2)), np.full(p.n_haps, p.h_haps_m)])
        haps_intf = rng.uniform(-half, half, (p.n_haps, 2))
        # n_sat points uniform on the orbital sphere: cos(psi) ~ U(-1, 1) each; keep the max
        cos_psi = 2 * rng.uniform() ** (1.0 / p.n_sat) - 1
        return Deployment(bs, iot, haps, haps_intf, float(cos_psi))

    # ------------------------------------------------------------- mesh graph
    def mesh_sinr(self, xy, rng, side_m=None, fading=None, channels=None):
        """SINR matrix S[i, j] for drone i transmitting to drone j over the mesh.

        All drones transmit simultaneously, each on one of n_subch_mesh
        sub-channels; receiver j sees every other drone on transmitter i's
        sub-channel as interference.
        """
        p = self.p
        n = len(xy)
        d = self.pairwise_dist(xy, side_m)
        if fading is None:
            fading = self.nakagami(rng, (n, n))
        if channels is None:
            channels = rng.integers(0, p.n_subch_mesh, n)
        rx = dbm2w(p.p_drone_mesh_dbm + 2 * p.g_drone_dbi - self.pl_mesh_db(d)) * fading
        np.fill_diagonal(rx, 0.0)
        onehot = (channels[None, :] == np.arange(p.n_subch_mesh)[:, None]).astype(float)
        per_ch = onehot @ rx  # (C, n): total power on sub-channel c at receiver j
        interf = np.maximum(per_ch[channels, :] - rx, 0.0)
        s = self.sinr(rx, interf, self.noise_w(p.b_mesh_hz))
        np.fill_diagonal(s, 0.0)
        return s

    def sg_graph(self, sinr):
        """SG-derived graph: directed edge i->j iff SINR_ij > gamma_th."""
        return sinr > self.gamma_th

    # ------------------------------------------------------------ access tiers
    def tn_sinr(self, dep, xy, rng):
        """Uplink SINR and distance from drones at xy to their serving TN BS."""
        p = self.p
        n = len(xy)
        if len(dep.bs_xy) == 0:
            return np.zeros(n), np.full(n, np.inf)
        d = np.sqrt(((xy[:, None, :] - dep.bs_xy[None]) ** 2).sum(-1) + (p.h_drone_m - p.h_bs_m) ** 2)
        srv = d.argmin(1)
        d_srv = d[np.arange(n), srv]
        sig = dbm2w(p.p_drone_access_dbm + p.g_drone_dbi + p.g_tn_dbi - self.pl_tn_db(d_srv)) * self.nakagami(rng, n)
        # one co-channel IoT uplink user per other cell, received at the serving BS
        d_ib = np.sqrt(((dep.iot_xy[:, None, :] - dep.bs_xy[None]) ** 2).sum(-1) + (p.h_bs_m - p.h_iot_m) ** 2)
        pw = dbm2w(p.p_iot_dbm + p.g_iot_dbi + p.g_tn_dbi - self.pl_tn_db(d_ib))  # (interferer, BS)
        pw_srv = pw[:, srv].T.copy()  # (drone, interferer)
        pw_srv[np.arange(n), srv] = 0.0  # the serving cell schedules the drone, not its own IoT user
        interf = (pw_srv * self.nakagami(rng, pw_srv.shape)).sum(1)
        return self.sinr(sig, interf, self.noise_w(p.b_tn_hz)), d_srv

    def haps_sinr(self, dep, xy, sigma_b_deg, rng):
        """Uplink SINR and distance to the nearest HAPS, with Gaussian beam-pointing error."""
        p = self.p
        n = len(xy)
        pos = np.column_stack([xy, np.full(n, p.h_drone_m)])
        vec = pos[:, None, :] - dep.haps_xyz[None]
        d = np.linalg.norm(vec, axis=-1)
        srv = d.argmin(1)
        d_srv = d[np.arange(n), srv]
        shadow = self.shadowing(rng, n)
        theta = sigma_b_deg * rng.standard_normal(n)
        sig = dbm2w(p.p_drone_access_dbm + p.g_drone_dbi + self.haps_gain_db(np.abs(theta))
                    - self.fspl_db(d_srv, p.f_haps_hz)) * shadow
        # co-channel users of the other HAPS, seen through the serving beam's off-axis gain
        h_srv = dep.haps_xyz[srv]
        boresight = (pos - h_srv) / d_srv[:, None]
        ipos = np.column_stack([dep.haps_intf_xy, np.full(len(dep.haps_intf_xy), p.h_drone_m)])
        u = ipos[None, :, :] - h_srv[:, None, :]
        du = np.linalg.norm(u, axis=-1)
        cos_ang = np.clip((u * boresight[:, None, :]).sum(-1) / du, -1.0, 1.0)
        ang = np.degrees(np.arccos(cos_ang))
        pw = dbm2w(p.p_drone_access_dbm + p.g_drone_dbi + self.haps_gain_db(ang)
                   - self.fspl_db(du, p.f_haps_hz)) * self.shadowing(rng, du.shape)
        pw[np.arange(n), srv] = 0.0  # on the serving HAPS this resource belongs to the drone
        return self.sinr(sig, pw.sum(1), self.noise_w(p.b_haps_hz)), d_srv

    def sat_geometry(self, dep):
        """Slant range and elevation (deg) of the serving (max-elevation) satellite."""
        rs = R_EARTH + self.p.h_sat_m
        d = np.sqrt(R_EARTH ** 2 + rs ** 2 - 2 * R_EARTH * rs * dep.sat_cos_psi)
        sin_el = (rs * dep.sat_cos_psi - R_EARTH) / d
        return d, float(np.degrees(np.arcsin(np.clip(sin_el, -1, 1))))

    def sat_sinr(self, dep, xy, rng):
        """Uplink SINR and slant range to the serving LEO satellite.

        Drones are at most a few km apart, so they share the serving satellite
        and slant range (the offset is negligible against 550 km altitude).
        """
        p = self.p
        n = len(xy)
        d, el = self.sat_geometry(dep)
        shadow = self.shadowing(rng, n)
        intf_shadow = self.shadowing(rng, (n, p.n_sat_cochannel))
        if el < p.sat_min_elev_deg:
            return np.zeros(n), np.full(n, np.inf)
        base = p.p_drone_access_dbm + p.g_drone_dbi - self.fspl_db(d, p.f_sat_hz)
        sig = dbm2w(base + p.g_sat_dbi) * shadow
        interf = (dbm2w(base + p.g_sat_dbi + p.sidelobe_db) * intf_shadow).sum(1)
        return self.sinr(sig, interf, self.noise_w(p.b_sat_hz)), np.full(n, d)

    # --------------------------------------------------------- random access
    def random_access(self, tier, detectable, rng):
        """Slotted multi-resource random access with binary exponential back-off.

        tier: (T, N) int tier per drone (TN=0, HAPS=1, SAT=2, -1 = not contending).
        detectable: (T, N) bool, preamble decodable (SINR above the access threshold).
        All contenders start at opportunity 0. Per opportunity a contender picks one
        of its tier's resources uniformly and succeeds iff alone on it and detectable;
        after the k-th failure it waits U{0, ..., 2**min(k, cap) - 1} extra opportunities.
        Returns access delay in seconds, inf when not contending or after max attempts.
        """
        p = self.p
        n_ra = np.array([p.n_ra_tn, p.n_ra_haps, p.n_ra_sat])
        max_ra = int(n_ra.max())
        t, n = tier.shape
        pending = tier >= 0
        next_slot = np.zeros((t, n), dtype=np.int64)
        attempts = np.zeros((t, n), dtype=np.int64)
        delay = np.full((t, n), np.inf)
        slot = 0
        while pending.any():
            ti, di = np.nonzero(pending & (next_slot == slot))
            if len(ti):
                tt = tier[ti, di]
                res = rng.integers(0, n_ra[tt])
                key = (ti * 3 + tt) * max_ra + res
                alone = np.bincount(key, minlength=t * 3 * max_ra)[key] == 1
                ok = alone & detectable[ti, di]
                delay[ti[ok], di[ok]] = (slot + 1) * p.t_ra_s
                pending[ti[ok], di[ok]] = False
                fi, fd = ti[~ok], di[~ok]
                attempts[fi, fd] += 1
                k = attempts[fi, fd]
                pending[fi[k >= p.ra_max_attempts], fd[k >= p.ra_max_attempts]] = False
                next_slot[fi, fd] = slot + 1 + rng.integers(0, 2 ** np.minimum(k, p.backoff_cap_exp))
            slot += 1
        return delay

    def latency_s(self, access_s, payload_mbit, rate_bps, dist_m):
        """End-to-end latency = access + transmission + propagation."""
        with np.errstate(divide="ignore"):
            tx = payload_mbit * 1e6 / rate_bps
        return access_s + tx + dist_m / C_LIGHT

    # ---------------------------------------------------------------- compute
    def solo_completes(self, task_gflop):
        p = self.p
        return np.asarray(task_gflop) <= p.drone_gflops * p.task_deadline_s

    def helper_rates(self, sinr):
        """Mesh rates (bps) from each owner to its best max_helpers SG-graph neighbours (0 = none)."""
        p = self.p
        s = np.where(self.sg_graph(sinr), sinr, 0.0)
        top = -np.sort(-s, axis=1)[:, :p.max_helpers]
        if top.shape[1] < p.max_helpers:
            top = np.pad(top, ((0, 0), (0, p.max_helpers - top.shape[1])))
        return self.shannon_rate(p.b_mesh_hz, top)

    def coop_capacity_gflop(self, helper_rates_bps):
        """Largest task finished by the deadline when split over the owner and its helpers.

        Each helper first receives a chunk_overhead_mbit transfer over its mesh link,
        then computes until the deadline; the owner computes the whole time.
        """
        p = self.p
        c, dl = p.drone_gflops, p.task_deadline_s
        with np.errstate(divide="ignore"):
            t_tx = p.chunk_overhead_mbit * 1e6 / helper_rates_bps
        return c * dl + (c * np.clip(dl - t_tx, 0.0, None)).sum(-1)

    # ----------------------------------------------------------------- energy
    @property
    def battery_j(self):
        return self.p.battery_wh * 3600.0

    def flight_ceiling_s(self):
        return self.battery_j / self.p.p_flight_w

    def cycle_energy_j(self, payload_mbit, rate_bps, onboard_compute):
        """Energy per capture cycle: flight + radio (P_tx x airtime) + optional compute."""
        p = self.p
        with np.errstate(divide="ignore"):
            e = p.p_flight_w * p.capture_interval_s + p.p_tx_w * payload_mbit * 1e6 / rate_bps
        if onboard_compute:
            e = e + p.p_compute_w * p.capture_interval_s
        return e

    def battery_percent(self, t_s, cycle_energy_j):
        """Remaining battery (%) at times t_s for drones with the given per-cycle energy."""
        p_avg = np.asarray(cycle_energy_j, dtype=float) / self.p.capture_interval_s
        return np.clip(100.0 * (1.0 - np.outer(p_avg, t_s) / self.battery_j), 0.0, 100.0)

    def endurance_s(self, cycle_energy_j):
        return self.battery_j * self.p.capture_interval_s / np.asarray(cycle_energy_j, dtype=float)
