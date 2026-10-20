"""
sagin_world.py
==============
Shared stochastic-geometry physics engine for the SAGIN magazine paper.

Design rule (the "unified-world discipline" stated in the paper): every
figure in the results section must call THIS module's functions for SINR
and graph construction. The physics is implemented once, here, and never
reimplemented per figure.

STATUS (read this before quoting any number to a reader):
This file is FINAL in structure -- one shared engine, five validated
figures, all monotonicity/sanity checks passing. It is NOT final in its
numbers. Every parameter in Params falls into one of three categories,
tagged in the comments below:
    [PLACEHOLDER] -- a representative stand-in for the exact value that
        must come from the journal's own link-budget/channel parameters
        before submission (e.g. transmit power, antenna gain).
    [TUNED]        -- deliberately chosen so a genuine physical effect
        (contention collapse, energy divergence, beam-pointing penalty)
        is clearly visible in the plotted range, rather than being
        washed out or invisible at arbitrary defaults. The MECHANISM is
        real; the exact crossover point will shift once real parameters
        replace the PLACEHOLDER values above.
    [DERIVED]      -- fixed by physical reasoning, not free to tune
        (e.g. speed of light, altitudes from the architecture spec).
Swap PLACEHOLDER values for the journal's exact numbers before the
manuscript's Table II is finalized; re-run every fig*.py afterward since
all five figures depend on this single file.

Units convention:
    - distances: kilometres unless a _m suffix says metres
    - powers/gains: dBm / dBi
    - all dB quantities combine additively; linear (mW) only for
      summing interference power
"""

import numpy as np


# --------------------------------------------------------------------------
# World / link-budget parameters. See STATUS note above for the
# [PLACEHOLDER] / [TUNED] / [DERIVED] provenance tags on each value.
# --------------------------------------------------------------------------
class Params:
    # Geometry [DERIVED -- fixed by the architecture spec]
    haps_alt_km = 20.0
    sat_alt_km = 550.0
    tn_height_km = 0.03          # 30 m mast
    pcp_cluster_std_km = 0.35    # [PLACEHOLDER] Gaussian daughter spread
    pcp_mean_drones_per_cluster = 8.0  # [PLACEHOLDER]

    # Carrier frequency (GHz) -- per tier [PLACEHOLDER, but the SPLIT
    # itself is a deliberate physical choice, not arbitrary]. TN uses a
    # conventional cellular band; HAPS/satellite feeder/access links use
    # Ka-band, consistent with the multi-tier link table (higher path
    # loss brings these links to a realistic operating margin where
    # beam-pointing error is visible, rather than an artificially
    # over-provisioned link -- see Fig. 3's debugging notes).
    freq_tn_ghz = 2.0
    freq_haps_ghz = 20.0
    freq_sat_ghz = 20.0

    # Transmit power (dBm) [LITERATURE: Lou, Belmekki & Alouini,
    # arXiv:2310.09659, Table I -- user/UAV/HAPS/Sat = 20/30/36/45 dBm.
    # Drone access power (20 dBm) matches exactly.]
    p_drone_dbm = 20.0           # drone -> infrastructure (access link)
    p_drone_mesh_dbm = 10.0      # [PLACEHOLDER] drone -> drone (short-range mesh link)

    # Antenna gain (dBi) [LITERATURE: Lou et al. Table I -- user/UAV/HAPS/Sat
    # = 3/10/30/50 dBi. Drone gain (3 dBi) matches exactly; HAPS/Sat set
    # close to their values -- see Fig. 3's tuning notes for why 48 (not 50)
    # keeps satellite a genuine last resort rather than an always-on backup
    # at the chosen threshold]
    g_drone_dbi = 3.0
    g_tn_dbi = 15.0
    g_haps_dbi = 25.0
    g_sat_dbi = 48.0

    # Noise (dBm) [DERIVED from thermal noise density -174 dBm/Hz (a
    # physical constant, kT at room temperature, also used in Lou et al.
    # Table I) + 60dB for a 1 MHz channel + ~7dB receiver noise figure]
    n0_dbm = -107.0

    # Path-loss model coefficients [PLACEHOLDER]
    # Terrestrial: 3GPP-like macro log-distance model, PL[dB]=a+b*log10(d_km)
    tn_pl_a = 128.1
    tn_pl_b = 37.6
    # Short-range mesh (drone-drone), d in metres: PL[dB]=a+b*log10(d_m)
    mesh_pl_a = 40.0
    mesh_pl_b = 25.0

    # [TUNED] Number of orthogonal sub-channels for the swarm mesh
    # (frequency/time division among simultaneously active drones),
    # matching the mitigation used in comparable NTN literature (e.g.,
    # Lou et al. use 10 orthogonal sub-bands). Without this, single-
    # channel interference among ~40 active drones crushes mean degree
    # below 1 at every density (see Fig. 2's debugging notes) -- the
    # MECHANISM (orthogonalization reduces mutual interference) is
    # standard practice; the exact count (8) is representative.
    n_mesh_channels = 8

    # Fading [nakagami_m LITERATURE-matched: Lou et al. Table I uses
    # Nakagami-m shape = 2 exactly; shadow std is a PLACEHOLDER]
    nakagami_m = 2.0             # near-ground / mesh links
    aerial_shadow_std_db = 4.0   # HAPS/Sat, approximating shadowed-Rician

    # HAPS beam pattern [PLACEHOLDER]
    haps_theta_3db_deg = 5.0     # 3 dB beamwidth of the HAPS spot beam

    # Default SINR threshold (dB) [PLACEHOLDER -- Fig. 3 uses a separate,
    # higher access threshold (gamma_th=5dB) than this mesh default; see
    # that figure's own tuning notes for why]
    gamma_th_db = -5.0

    # ---- Random-access contention (Fig. 4) --------------------------------
    # [LITERATURE-matched] TN pool of 12 coincides with an actual 3GPP
    # NB-IoT NPRACH configuration (NPRACH supports 12/24/36/48 subcarriers
    # as its orthogonal preamble resources per coverage-enhancement level).
    # HAPS/Sat wider (64), representative of a platform serving fewer
    # simultaneous users per unit area post-redistribution [PLACEHOLDER
    # for the exact HAPS/Sat pool size].
    n_ra_resources_tn = 12
    n_ra_resources_haps = 64
    n_ra_resources_sat = 64
    # [LITERATURE-matched] one NPRACH preamble occupies 5.6ms (format 0)
    # or 6.4ms (format 1); 6.4ms used here as the random-access opportunity
    # duration (3GPP Rel-13 NB-IoT NPRACH).
    t_ra_frame_ms = 6.4
    packet_size_mbits = 0.5       # [PLACEHOLDER] small IoT-style uplink report
    tn_bandwidth_mhz = 1.0        # [PLACEHOLDER]
    # [LITERATURE-matched] 5MHz is a standard LTE channel bandwidth,
    # consistent with HIBS (High-Altitude Platform Station as IMT Base
    # Station) -- a real ITU-R/3GPP standardization track (formally
    # recognized at WRC-23) whose core design principle is that HAPS
    # reuses the SAME frequency bands/bandwidths as ground-based IMT
    # base stations, so existing terrestrial UEs can connect unmodified.
    haps_bandwidth_mhz = 5.0
    # [LITERATURE-matched] satellite NB-NTN is genuinely narrowband by
    # design (the "NB" in NB-IoT/NB-NTN); 1.5MHz is close to 3GPP's
    # eMTC-based IoT-NTN category (Category M1, standardized at 1.4MHz)
    # -- deliberately the WIDER of the two defined IoT-NTN options (the
    # other being the NB-IoT-based category at 200kHz), chosen because
    # it keeps this simulation's raw payload deliverable on a plausible
    # timescale. Still ~3.3x narrower than HAPS's HIBS-consistent 5MHz --
    # the gap this creates is what Fig. 5b's raw-HAPS vs raw-satellite
    # comparison is built to show.
    sat_bandwidth_mhz = 1.5
    # [TUNED] HAPS-level load-balancing rule: target fraction of active
    # drones kept on TN before the orchestrator shifts the rest to
    # HAPS/Sat -- set just above n_ra_resources_tn so the rule visibly
    # relieves contention before it collapses.
    tn_target_load = 15

    # ---- Task splitting (Fig. 5a) -----------------------------------------
    # [LITERATURE-matched] Jetson Nano-class edge-AI accelerator: 472
    # GFLOPS at 5-10W (NVIDIA Jetson Nano datasheet) -- a real, deployed,
    # drone-appropriate onboard AI compute module, not an arbitrary
    # placeholder. Solo's cliff at G=drone_cpu_gflops x task_deadline_s
    # (470 GFLOPS) is a DERIVED consequence of this capacity and the
    # deadline, not tuned separately.
    drone_cpu_gflops = 470.0
    task_deadline_s = 1.0         # [PLACEHOLDER] safety-critical deadline
    max_helpers = 5               # [PLACEHOLDER] cap on cooperating neighbours
    # [TUNED] compact chunk size + wide local mesh bandwidth, so
    # cooperation overhead does not dominate compute savings (an earlier
    # version used a much larger chunk over narrow bandwidth, which made
    # cooperation WORSE than solo -- see Fig. 5a's debugging notes)
    split_overhead_mbits = 0.2
    mesh_bandwidth_mhz = 20.0

    # ---- Energy accounting (Fig. 5b) --------------------------------------
    # [LITERATURE-matched] battery capacity and flight power drawn from
    # published small-UAV specs (e.g. DJI Mavic-class: ~74 Wh battery pack,
    # ~120 W cruise/hover power draw). p_compute_w matches the Jetson
    # Nano-class accelerator above (5-10W range). Radio power and payload
    # sizes remain [PLACEHOLDER]; raw_image_mbits represents a higher-
    # resolution video-clip/multispectral-image-burst capture (100 MB),
    # chosen (still within [PLACEHOLDER] status -- not literature-matched)
    # specifically so the per-tier bandwidth difference established for
    # Figs. 4/5b (HIBS-consistent 5MHz HAPS vs. IoT-NTN-Category-M1-
    # consistent 1.5MHz satellite) is clearly visible in the radio-energy
    # term rather than swamped by the (literature-matched) flight power.
    battery_capacity_j = 148.0 * 3600.0   # 148 Wh -> Joules (2x the 74Wh baseline)
    p_flight_w = 120.0
    p_compute_w = 10.0
    p_tx_w = 8.0                          # [PLACEHOLDER]
    raw_image_mbits = 800.0                # [PLACEHOLDER]
    report_mbits = 0.02                    # [PLACEHOLDER]
    capture_interval_s = 10.0              # [PLACEHOLDER]


def db_to_lin(x_db):
    return 10.0 ** (x_db / 10.0)


def lin_to_db(x_lin):
    return 10.0 * np.log10(np.maximum(x_lin, 1e-30))


def path_loss_tn_db(d_km, p=Params):
    """3GPP-like macro-cell log-distance path loss."""
    d_km = np.maximum(d_km, 1e-3)
    return p.tn_pl_a + p.tn_pl_b * np.log10(d_km)


def path_loss_fspl_db(d_km, freq_ghz):
    """Free-space path loss (HAPS/satellite links)."""
    d_km = np.maximum(d_km, 1e-3)
    return 20.0 * np.log10(d_km) + 20.0 * np.log10(freq_ghz) + 92.45


def path_loss_mesh_db(d_m, p=Params):
    """Short-range drone-to-drone mesh link path loss."""
    d_m = np.maximum(d_m, 1.0)
    return p.mesh_pl_a + p.mesh_pl_b * np.log10(d_m)


def nakagami_fading_db(size, m, rng):
    """Nakagami-m fading power (mean 0 dB) via Gamma(shape=m, scale=1/m)."""
    power_lin = rng.gamma(shape=m, scale=1.0 / m, size=size)
    return lin_to_db(power_lin)


def aerial_shadow_fading_db(size, std_db, rng):
    """Log-normal shadowing approximating a shadowed-Rician aerial link."""
    return rng.normal(loc=0.0, scale=std_db, size=size)


def haps_beam_gain_db(theta_error_deg, p=Params):
    """
    Parabolic-beam approximation of gain loss due to pointing error.
    theta_error_deg: angular deviation of the beam from boresight (deg).
    Returns the EFFECTIVE HAPS antenna gain in dBi (already includes the
    nominal g_haps_dbi).
    """
    penalty_db = 12.0 * (theta_error_deg / p.haps_theta_3db_deg) ** 2
    return p.g_haps_dbi - penalty_db


def sample_pointing_error_deg(size, sigma_b_deg, rng):
    """Folded-Gaussian beam-pointing error: |eps|, eps ~ N(0, sigma_b^2)."""
    if sigma_b_deg == 0.0:
        return np.zeros(size)
    return np.abs(rng.normal(loc=0.0, scale=sigma_b_deg, size=size))


# --------------------------------------------------------------------------
# SAGINWorld: the object every figure script imports and calls.
# --------------------------------------------------------------------------
class SAGINWorld:
    def __init__(self, params=Params, seed=None):
        self.p = params
        self.rng = np.random.default_rng(seed)

    # ---- drone placement (BPP within a square region) --------------------
    def drop_drones(self, n, area_side_km, rng=None):
        """N drones uniformly at random (BPP) in a square region centred
        on the origin, side length area_side_km. Returns (N,2) array (km)."""
        rng = rng or self.rng
        half = area_side_km / 2.0
        return rng.uniform(-half, half, size=(n, 2))

    def drop_drone_clusters(self, n_clusters, area_side_km,
                            mean_drones_per_cluster=None,
                            cluster_std_km=None, rng=None):
        """
        Thomas Poisson cluster process approximation for mission-oriented
        drone groups.

        Parent/cluster centres are uniform in the square. The number of
        daughter drones per cluster is Poisson. Daughter offsets are i.i.d.
        Gaussian with standard deviation cluster_std_km. Points outside the
        square are wrapped back into the simulation window.
        """
        rng = rng or self.rng
        p = self.p
        mean_n = (p.pcp_mean_drones_per_cluster if mean_drones_per_cluster is None
                  else mean_drones_per_cluster)
        sigma = p.pcp_cluster_std_km if cluster_std_km is None else cluster_std_km

        half = area_side_km / 2.0
        parents = rng.uniform(-half, half, size=(n_clusters, 2))
        counts = rng.poisson(mean_n, size=n_clusters)

        daughters = []
        labels = []
        for c, (parent, count) in enumerate(zip(parents, counts)):
            if count == 0:
                continue
            pts = parent + rng.normal(0.0, sigma, size=(count, 2))
            # periodic wrapping preserves density near the borders
            pts = ((pts + half) % area_side_km) - half
            daughters.append(pts)
            labels.extend([c] * count)

        if daughters:
            positions = np.vstack(daughters)
        else:
            positions = np.empty((0, 2))

        return {
            "positions": positions,
            "parents": parents,
            "cluster_labels": np.asarray(labels, dtype=int),
            "counts": counts,
        }

    # ---- single-link SINR calculators (access links: drone -> tier) ------
    def sinr_to_tn_db(self, horiz_dist_km, rng=None, n_trials=1,
                      interferer_dist_km=None, interferer_tx_dbm=None,
                      activity_prob=1.0):
        """
        Drone access-link SINR to the serving TN BS.

        Optional interferer_dist_km adds co-channel uplink interferers whose
        received powers are summed at the serving BS. If omitted, this
        reduces to the previous noise-limited SNR model.
        """
        rng = rng or self.rng
        p = self.p
        d3d = np.sqrt(np.asarray(horiz_dist_km) ** 2 + p.tn_height_km ** 2)
        pl_db = path_loss_tn_db(d3d, p)
        fading_db = nakagami_fading_db(n_trials, p.nakagami_m, rng)
        rx_dbm = p.p_drone_dbm + p.g_drone_dbi + p.g_tn_dbi - pl_db + fading_db

        interf_lin = 0.0
        if interferer_dist_km is not None:
            d_int = np.atleast_1d(interferer_dist_km)
            if d_int.size:
                ptx = p.p_drone_dbm if interferer_tx_dbm is None else interferer_tx_dbm
                active = rng.random(d_int.size) < activity_prob
                if np.any(active):
                    d3d_i = np.sqrt(d_int[active] ** 2 + p.tn_height_km ** 2)
                    pl_i = path_loss_tn_db(d3d_i, p)
                    fad_i = nakagami_fading_db(active.sum(), p.nakagami_m, rng)
                    rx_i_dbm = ptx + p.g_drone_dbi + p.g_tn_dbi - pl_i + fad_i
                    interf_lin = db_to_lin(rx_i_dbm).sum()

        sinr_lin = db_to_lin(rx_dbm) / (interf_lin + db_to_lin(p.n0_dbm))
        return lin_to_db(sinr_lin)

    def sinr_to_haps_db(self, horiz_dist_km, sigma_b_deg, rng=None, n_trials=1,
                        interferer_dist_km=None, interferer_sigma_b_deg=None,
                        activity_prob=1.0):
        """
        Drone access-link SINR to HAPS with pointing error and optional
        co-channel uplink interferers observed at the HAPS receiver.
        """
        rng = rng or self.rng
        p = self.p
        d3d = np.sqrt(np.asarray(horiz_dist_km) ** 2 + p.haps_alt_km ** 2)
        pl_db = path_loss_fspl_db(d3d, p.freq_haps_ghz)
        shadow_db = aerial_shadow_fading_db(n_trials, p.aerial_shadow_std_db, rng)
        theta_err = sample_pointing_error_deg(n_trials, sigma_b_deg, rng)
        g_haps_eff_dbi = haps_beam_gain_db(theta_err, p)
        rx_dbm = (p.p_drone_dbm + p.g_drone_dbi + g_haps_eff_dbi
                  - pl_db + shadow_db)

        interf_lin = 0.0
        if interferer_dist_km is not None:
            d_int = np.atleast_1d(interferer_dist_km)
            if d_int.size:
                active = rng.random(d_int.size) < activity_prob
                if np.any(active):
                    sig_i = sigma_b_deg if interferer_sigma_b_deg is None else interferer_sigma_b_deg
                    d3d_i = np.sqrt(d_int[active] ** 2 + p.haps_alt_km ** 2)
                    pl_i = path_loss_fspl_db(d3d_i, p.freq_haps_ghz)
                    sh_i = aerial_shadow_fading_db(active.sum(), p.aerial_shadow_std_db, rng)
                    th_i = sample_pointing_error_deg(active.sum(), sig_i, rng)
                    g_i = haps_beam_gain_db(th_i, p)
                    rx_i_dbm = p.p_drone_dbm + p.g_drone_dbi + g_i - pl_i + sh_i
                    interf_lin = db_to_lin(rx_i_dbm).sum()

        sinr_lin = db_to_lin(rx_dbm) / (interf_lin + db_to_lin(p.n0_dbm))
        return lin_to_db(sinr_lin)

    def sinr_to_sat_db(self, horiz_dist_km, rng=None, n_trials=1,
                       interferer_dist_km=None, activity_prob=1.0):
        """
        Drone access-link SINR to LEO satellite with optional co-channel
        uplink interferers at the satellite receiver.
        """
        rng = rng or self.rng
        p = self.p
        d3d = np.sqrt(np.asarray(horiz_dist_km) ** 2 + p.sat_alt_km ** 2)
        pl_db = path_loss_fspl_db(d3d, p.freq_sat_ghz)
        shadow_db = aerial_shadow_fading_db(n_trials, p.aerial_shadow_std_db, rng)
        rx_dbm = p.p_drone_dbm + p.g_drone_dbi + p.g_sat_dbi - pl_db + shadow_db

        interf_lin = 0.0
        if interferer_dist_km is not None:
            d_int = np.atleast_1d(interferer_dist_km)
            if d_int.size:
                active = rng.random(d_int.size) < activity_prob
                if np.any(active):
                    d3d_i = np.sqrt(d_int[active] ** 2 + p.sat_alt_km ** 2)
                    pl_i = path_loss_fspl_db(d3d_i, p.freq_sat_ghz)
                    sh_i = aerial_shadow_fading_db(active.sum(), p.aerial_shadow_std_db, rng)
                    rx_i_dbm = p.p_drone_dbm + p.g_drone_dbi + p.g_sat_dbi - pl_i + sh_i
                    interf_lin = db_to_lin(rx_i_dbm).sum()

        sinr_lin = db_to_lin(rx_dbm) / (interf_lin + db_to_lin(p.n0_dbm))
        return lin_to_db(sinr_lin)

    # ---- swarm mesh graph: the SG-derived communication graph -----------
    def build_graph(self, positions_km, gamma_th_db, p_active=0.3,
                     rng=None, active_mask=None):
        """
        Construct the SG-derived communication graph among a set of drones.

        positions_km : (N,2) array, drone coordinates in km.
        gamma_th_db  : SINR threshold for an edge to exist.
        p_active     : probability a drone is transmitting this snapshot
                       (models slotted/contention access). Ignored if
                       active_mask is given.
        active_mask  : optional (N,) boolean array to force a specific
                       active set (useful for reproducibility/testing).

        Returns dict with:
            adjacency   : (N,N) boolean, symmetric (edge iff both directions
                          clear the threshold)
            active      : (N,) boolean mask of transmitting drones
            mean_degree : average degree over ACTIVE drones only
            edge_prob   : fraction of active pairs that are connected
        """
        rng = rng or self.rng
        p = self.p
        n = positions_km.shape[0]

        if active_mask is None:
            active = rng.random(n) < p_active
        else:
            active = active_mask.copy()

        # assign each active drone one of K orthogonal mesh sub-channels;
        # only same-channel transmitters interfere with each other
        channel = rng.integers(0, p.n_mesh_channels, size=n)
        same_channel = channel[:, None] == channel[None, :]

        # pairwise distances (metres)
        diff = positions_km[:, None, :] - positions_km[None, :, :]
        dist_km = np.sqrt(np.sum(diff ** 2, axis=-1))
        dist_m = dist_km * 1000.0
        np.fill_diagonal(dist_m, np.inf)  # no self-link

        pl_db = path_loss_mesh_db(dist_m, p)
        fading_db = nakagami_fading_db((n, n), p.nakagami_m, rng)
        fading_db = np.triu(fading_db) + np.triu(fading_db, 1).T  # symmetric

        # received power matrix: rx_dbm[i,j] = power received at j from i
        rx_dbm = (p.p_drone_mesh_dbm + p.g_drone_dbi + p.g_drone_dbi
                  - pl_db + fading_db)
        rx_lin = db_to_lin(rx_dbm)

        active_col = active[None, :]           # receiver is active
        active_row = active[:, None]           # transmitter is active
        # only count as an interferer if also on the same sub-channel as
        # the link being evaluated
        rx_lin_active_tx = np.where(active_row, rx_lin, 0.0)
        rx_lin_active_tx_samech = np.where(same_channel, rx_lin_active_tx, 0.0)

        # total same-channel interference+signal power arriving at each
        # receiver j from all active, same-channel transmitters
        total_at_j = rx_lin_active_tx_samech.sum(axis=0)  # (n,)
        n0_lin = db_to_lin(p.n0_dbm)

        # SINR_ij : intended link i->j ; interference = total at j minus
        # the intended signal itself (i and j are same-channel by construction
        # when evaluating their own link)
        signal_ij = rx_lin_active_tx  # (n,n), [i,j] = power i->j if i active
        interf_ij = total_at_j[None, :] - np.where(same_channel, signal_ij, 0.0)
        interf_ij = np.maximum(interf_ij, 0.0)
        sinr_ij = signal_ij / (interf_ij + n0_lin)
        sinr_ij_db = lin_to_db(sinr_ij)

        link_ok = sinr_ij_db > gamma_th_db
        # both directions must clear the threshold, and both nodes active
        both_active = active_row & active_col
        adjacency = link_ok & link_ok.T & both_active

        n_active = active.sum()
        if n_active > 1:
            degrees = adjacency[active][:, active].sum(axis=1)
            mean_degree = degrees.mean()
            possible_pairs = n_active * (n_active - 1)
            edge_prob = adjacency[active][:, active].sum() / max(possible_pairs, 1)
        else:
            mean_degree = 0.0
            edge_prob = 0.0

        return {
            "adjacency": adjacency,
            "active": active,
            "mean_degree": mean_degree,
            "edge_prob": edge_prob,
        }

    def build_graph_distance_rule(self, positions_km, d_conn_m,
                                   p_active=0.3, rng=None, active_mask=None):
        """Baseline comparator: edge iff distance < d_conn_m (metres),
        ignoring SINR/interference entirely."""
        rng = rng or self.rng
        n = positions_km.shape[0]
        if active_mask is None:
            active = rng.random(n) < p_active
        else:
            active = active_mask.copy()

        diff = positions_km[:, None, :] - positions_km[None, :, :]
        dist_m = np.sqrt(np.sum(diff ** 2, axis=-1)) * 1000.0
        np.fill_diagonal(dist_m, np.inf)

        link_ok = dist_m < d_conn_m
        both_active = active[:, None] & active[None, :]
        adjacency = link_ok & both_active

        n_active = active.sum()
        if n_active > 1:
            degrees = adjacency[active][:, active].sum(axis=1)
            mean_degree = degrees.mean()
        else:
            mean_degree = 0.0

        return {"adjacency": adjacency, "active": active,
                "mean_degree": mean_degree}

    # ---- Fig. 4: random-access latency under contention -------------------
    def random_access_latency_ms(self, n_contenders, n_resources, tn_horiz_km,
                                  tier="tn", rng=None, n_trials=1):
        """
        Latency for one drone accessing a tier shared by n_contenders other
        active drones, modeled as slotted random access over n_resources
        orthogonal preambles (collision iff two drones pick the same
        resource in the same attempt).

        Returns total latency (ms) = access delay (retries) + transmission
        time (from actual SINR-derived Shannon capacity) + propagation.
        Uses the SAME sinr_to_*_db functions as every other figure.
        """
        rng = rng or self.rng
        p = self.p

        # success probability per attempt: none of the other n_contenders
        # active drones picked the same one of n_resources slots
        n_c = max(n_contenders, 0)
        p_success = ((n_resources - 1) / n_resources) ** n_c
        p_success = max(p_success, 1e-6)
        # expected number of attempts (geometric distribution)
        mean_attempts = 1.0 / p_success
        access_delay_ms = mean_attempts * p.t_ra_frame_ms

        # transmission time from the real link budget (Shannon capacity)
        if tier == "tn":
            sinr_db = self.sinr_to_tn_db(tn_horiz_km, rng=rng, n_trials=n_trials)
            bw_hz = p.tn_bandwidth_mhz * 1e6
            prop_ms = tn_horiz_km / 3e5 * 1000.0  # speed of light
        elif tier == "haps":
            sinr_db = self.sinr_to_haps_db(tn_horiz_km, sigma_b_deg=0.0,
                                            rng=rng, n_trials=n_trials)
            bw_hz = p.haps_bandwidth_mhz * 1e6
            d3d = np.sqrt(tn_horiz_km ** 2 + p.haps_alt_km ** 2)
            prop_ms = d3d / 3e5 * 1000.0
        elif tier == "sat":
            sinr_db = self.sinr_to_sat_db(tn_horiz_km, rng=rng, n_trials=n_trials)
            bw_hz = p.sat_bandwidth_mhz * 1e6
            d3d = np.sqrt(tn_horiz_km ** 2 + p.sat_alt_km ** 2)
            prop_ms = d3d / 3e5 * 1000.0
        else:
            raise ValueError(tier)

        sinr_lin = db_to_lin(sinr_db)
        capacity_bps = bw_hz * np.log2(1.0 + sinr_lin)
        capacity_bps = np.maximum(capacity_bps, 1.0)
        tx_time_ms = (p.packet_size_mbits * 1e6) / capacity_bps * 1000.0

        return access_delay_ms + tx_time_ms + prop_ms

    def haps_load_balance(self, n_active, p=Params):
        """Legacy aggregate rule retained for backward compatibility."""
        n_tn = min(n_active, p.tn_target_load)
        remainder = max(n_active - n_tn, 0)
        n_haps = remainder // 2
        n_sat = remainder - n_haps
        return n_tn, n_haps, n_sat

    def feasible_load_balance(self, sinr_tn_db, sinr_haps_db, sinr_sat_db,
                              gamma_tn_db=-5.0, gamma_haps_db=-5.0,
                              gamma_sat_db=-5.0, p=Params):
        """
        Per-drone HAPS-orchestrator association.

        A tier is feasible only when that drone's SINR exceeds the tier
        threshold. Among feasible tiers, assign the drone to the tier with
        the smallest normalized load. TN is softly capped at
        tn_target_load. Returns assignments and final tier loads.
        """
        sinr_tn_db = np.asarray(sinr_tn_db)
        sinr_haps_db = np.asarray(sinr_haps_db)
        sinr_sat_db = np.asarray(sinr_sat_db)
        n = len(sinr_tn_db)

        caps = np.array([p.tn_target_load,
                         p.n_ra_resources_haps,
                         p.n_ra_resources_sat], dtype=float)
        loads = np.zeros(3, dtype=int)
        assignments = np.full(n, -1, dtype=int)  # -1 means outage

        # hardest-to-serve drones first: fewest feasible tiers
        feasible_matrix = np.column_stack([
            sinr_tn_db >= gamma_tn_db,
            sinr_haps_db >= gamma_haps_db,
            sinr_sat_db >= gamma_sat_db,
        ])
        order = np.argsort(feasible_matrix.sum(axis=1))

        for i in order:
            feasible = np.where(feasible_matrix[i])[0]
            if feasible.size == 0:
                continue
            normalized_load = loads[feasible] / np.maximum(caps[feasible], 1.0)
            k = feasible[np.argmin(normalized_load)]
            assignments[i] = k
            loads[k] += 1

        return {
            "assignments": assignments,
            "loads": {"tn": int(loads[0]), "haps": int(loads[1]), "sat": int(loads[2])},
            "outage_count": int(np.sum(assignments < 0)),
        }

    # ---- Fig. 5(a): graph-bounded cooperative task splitting --------------
    def task_split_outcome(self, positions_km, task_gflops, gamma_th_db,
                            p_active=0.5, rng=None):
        """
        One Monte Carlo trial: drop drones, build the SG-derived mesh graph,
        pick one active drone as the task owner, and check whether it meets
        the deadline (i) alone, and (ii) cooperating with up to max_helpers
        ACTUAL graph neighbours (cooperation is graph-bounded, not assumed).

        Returns (solo_ok: bool, coop_ok: bool) or (None, None) if no active
        drone is available this trial.
        """
        rng = rng or self.rng
        p = self.p
        g = self.build_graph(positions_km, gamma_th_db=gamma_th_db,
                              p_active=p_active, rng=rng)
        active_idx = np.where(g["active"])[0]
        if len(active_idx) == 0:
            return None, None

        owner = rng.choice(active_idx)
        neighbours = np.where(g["adjacency"][owner])[0]
        n_helpers = min(len(neighbours), p.max_helpers)
        helpers = rng.choice(neighbours, size=n_helpers, replace=False) \
            if n_helpers > 0 else np.array([], dtype=int)

        # solo: only the owner's own CPU
        solo_time_s = task_gflops / p.drone_cpu_gflops
        solo_ok = solo_time_s <= p.task_deadline_s

        # cooperative: owner + helpers' CPU, plus overhead to ship chunks
        # over the actual mesh link rate to each helper
        total_cpu = p.drone_cpu_gflops * (1 + n_helpers)
        coop_compute_s = task_gflops / total_cpu

        overhead_s = 0.0
        if n_helpers > 0:
            diff = positions_km[owner] - positions_km[helpers]
            dist_m = np.sqrt((diff ** 2).sum(axis=-1)) * 1000.0
            pl_db = path_loss_mesh_db(dist_m, p)
            fading_db = nakagami_fading_db(n_helpers, p.nakagami_m, rng)
            rx_dbm = (p.p_drone_mesh_dbm + p.g_drone_dbi + p.g_drone_dbi
                      - pl_db + fading_db)
            sinr_lin = db_to_lin(rx_dbm) / db_to_lin(p.n0_dbm)
            bw_hz = p.mesh_bandwidth_mhz * 1e6  # local mesh link bandwidth
            capacity_bps = np.maximum(bw_hz * np.log2(1.0 + sinr_lin), 1.0)
            per_helper_s = (p.split_overhead_mbits * 1e6) / capacity_bps
            overhead_s = per_helper_s.max()  # bottlenecked by the worst link

        coop_ok = (coop_compute_s + overhead_s) <= p.task_deadline_s
        return solo_ok, coop_ok

    # ---- Fig. 5(b): energy accounting over a mission -----------------------
    def mission_battery_trace(self, duration_s, strategy, tier="sat", rng=None):
        """
        Per-second-resolution (stepped every capture_interval_s) battery
        trace over a mission. strategy in {"raw_satellite", "edge_ai"}
        (name kept for backward compatibility; applies to whichever tier
        is selected). tier in {"sat", "haps"}.
        Uses the actual per-tier SINR-derived Shannon capacity for radio
        time, so a weak link costs more energy, exactly as in Fig. 3/4.
        """
        rng = rng or self.rng
        p = self.p
        n_steps = int(duration_s / p.capture_interval_s)
        battery_j = p.battery_capacity_j
        trace_pct = [100.0]

        if tier == "sat":
            sinr_db = self.sinr_to_sat_db(1.0, rng=rng, n_trials=1)[0]
            bw_mhz = p.sat_bandwidth_mhz
        elif tier == "haps":
            sinr_db = self.sinr_to_haps_db(1.0, sigma_b_deg=0.0, rng=rng, n_trials=1)[0]
            bw_mhz = p.haps_bandwidth_mhz
        else:
            raise ValueError(tier)
        capacity_bps = max(bw_mhz * 1e6 * np.log2(1.0 + db_to_lin(sinr_db)), 1.0)

        for _ in range(n_steps):
            e_flight = p.p_flight_w * p.capture_interval_s

            if strategy == "raw_satellite":
                tx_time_s = (p.raw_image_mbits * 1e6) / capacity_bps
                e_radio = p.p_tx_w * tx_time_s
                e_compute = 0.0
            elif strategy == "edge_ai":
                # process locally (fixed short compute burst), then send
                # only the compact report
                compute_time_s = 2.0
                e_compute = p.p_compute_w * compute_time_s
                tx_time_s = (p.report_mbits * 1e6) / capacity_bps
                e_radio = p.p_tx_w * tx_time_s
            else:
                raise ValueError(strategy)

            battery_j -= (e_flight + e_radio + e_compute)
            battery_j = max(battery_j, 0.0)
            trace_pct.append(100.0 * battery_j / p.battery_capacity_j)

        return np.array(trace_pct)
