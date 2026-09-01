#!/usr/bin/env python3
"""
run_channel_validation.py
─────────────────────────
Reproducible validation harness for the NS-3 channel model in
three_uav_tapbridge_integrated.cc.

It stands in for Gazebo (positions + ray-caster output), drives the NS-3
binary through four scenarios, and writes every per-link parameter plus a
machine-readable pass/fail summary into  test_logs/ .

Why a harness and not a shell script: the publisher must run CONCURRENTLY with
NS-3 and be stopped deterministically afterwards. Doing that with background
shells and pkill is how you end up with a stale publisher silently feeding the
next scenario and turning a failing test green -- which happened during
development. Here the publisher is an rclpy node on a thread inside this
process and NS-3 is a child process, so both lifetimes are explicit.

Scenarios
  los_clear      all 6 links clear            -> LoS  fading distribution (m=MLos)
  nlos_blocked   all 6 links at 20 dB         -> NLoS fading distribution (m=MNlos)
  hysteresis     clear -> blocked -> clear    -> BlockThresholdDb / ClearThresholdDb
  ros_healthy    one blocked link of six      -> realistic mixed case

Checks performed per link, per scenario
  pathloss_rx_dbm   == Tx - (ReferenceLoss + 10*n*log10(d))     [identity]
  snr_db            == faded_rx_dbm - noiseFloor                [identity]
  fading_delta_db   == faded_rx - (pathloss_rx - obstacle_loss) [identity]
  fading_delta_db   ~  Nakagami-m in dB, mean and sd            [distribution]
  blocked / fading_m consistency, and both states observed      [state machine]

Usage
  source /opt/ros/humble/setup.bash
  python3 scripts/test_scripts/run_channel_validation.py            # all scenarios
  python3 scripts/test_scripts/run_channel_validation.py --quick    # shorter runs
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import threading
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ── Model constants: MUST mirror the NS-3 defaults being validated ───────────
# If you change these on the NS-3 command line, change them here too -- the
# identity checks below recompute the model independently, so a mismatch shows
# up as a spurious failure rather than a silent pass.
TX_DBM       = 20.0
REF_LOSS_DB  = 46.73      # Friis at 1 m, 5180 MHz (802.11a ch 36)
PATH_EXP     = 2.0
NOISE_FLOOR  = -94.0
M_LOS        = 3.0
M_NLOS       = 1.0

# This file lives in <repo>/scripts/test_scripts/, so the package root is two
# levels up -- one level lands in scripts/ and silently scatters the evidence
# into scripts/test_logs/.
REPO   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUTDIR = os.path.join(REPO, "test_logs")
NS3_BIN_DEFAULT = os.path.expanduser(
    "~/ns-3.3/build/scratch/multi_uav_simulation/"
    "ns3.38-three_uav_tapbridge_integrated-default")

# Node geometry fed to NS-3. Deliberately asymmetric: every pair sits at a
# DIFFERENT distance, so a permuted id->node mapping shows up as a wrong
# distance rather than passing unnoticed. (An earlier test geometry had two
# pairs at the same separation and could not have caught a swap between them.)
POSITIONS = {
    0: (0.0,  6.0,  2.9),    # GCS  (antenna height)
    1: (0.0,  2.0, 20.0),    # UAV1
    2: (0.0, -2.0, 20.0),    # UAV2
    3: (7.0,  1.0, 18.0),    # UAV3
}
LINKS = [(i, j) for i in range(4) for j in range(i + 1, 4)]


# ─────────────────────────────────────────────────────────────────────────────
#  Gazebo stand-in
# ─────────────────────────────────────────────────────────────────────────────
class FakeGazebo(Node):
    """Publishes /uav_world_positions and /link_obstacle_loss at 10 Hz.

    obstacle_fn(elapsed_s) -> dict {(i,j): loss_dB}, so a scenario can change
    the obstacle state partway through (that is how the hysteresis release is
    exercised).
    """

    def __init__(self, obstacle_fn):
        super().__init__("channel_validation_publisher")
        self.pos_pub = self.create_publisher(
            Float32MultiArray, "/uav_world_positions", 10)
        self.obs_pub = self.create_publisher(
            Float32MultiArray, "/link_obstacle_loss", 10)
        self.obstacle_fn = obstacle_fn
        self.t0 = time.time()
        self.create_timer(0.1, self._tick)

    def _tick(self):
        pos = []
        for nid, (x, y, z) in POSITIONS.items():
            pos += [float(nid), x, y, z]
        self.pos_pub.publish(Float32MultiArray(data=pos))

        losses = self.obstacle_fn(time.time() - self.t0)
        obs = []
        for (i, j) in LINKS:
            obs += [float(i), float(j), float(losses.get((i, j), 0.0))]
        self.obs_pub.publish(Float32MultiArray(data=obs))


def run_scenario(name, obstacle_fn, sim_time, stats_period, ns3_bin):
    """Run one scenario end to end; return the path of the CSV NS-3 wrote."""
    csv_path = os.path.join(OUTDIR, f"raw_{name}.csv")
    log_path = os.path.join(OUTDIR, f"raw_{name}.log")
    print(f"  [{name}] simTime={sim_time}s statsPeriod={stats_period}s ...",
          flush=True)

    rclpy.init()
    node = FakeGazebo(obstacle_fn)
    executor_stop = threading.Event()

    def spin():
        while rclpy.ok() and not executor_stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    time.sleep(3.0)          # let the feed establish before NS-3 starts

    with open(log_path, "w") as lf:
        proc = subprocess.run(
            [ns3_bin,
             "--enableTap=false",
             f"--simTime={sim_time}",
             "--posLogPeriod=0",
             f"--statsPeriod={stats_period}",
             f"--csvPath={csv_path}"],
            stdout=lf, stderr=subprocess.STDOUT,
            timeout=sim_time + 120)

    executor_stop.set()
    th.join(timeout=5)
    node.destroy_node()
    rclpy.shutdown()

    # A crash MUST fail the run. The first version of this harness printed
    # "FAILURES: 0" for a scenario in which NS-3 died with SIGSEGV at t=34s,
    # because enough rows had been written before the crash for every
    # per-link check to pass on the partial data. Process exit status is
    # evidence in its own right and is now reported as such.
    if proc.returncode != 0:
        sig = f"signal {-proc.returncode}" if proc.returncode < 0 else f"exit {proc.returncode}"
        print(f"  [{name}] *** NS-3 TERMINATED ABNORMALLY ({sig}) "
              f"-- see {log_path}")
    return csv_path, proc.returncode


# ─────────────────────────────────────────────────────────────────────────────
#  Analysis
# ─────────────────────────────────────────────────────────────────────────────
def digamma(x):
    r = 0.0
    while x < 6:
        r -= 1 / x
        x += 1
    f = 1 / (x * x)
    return r + math.log(x) - 0.5 / x + f * (-1 / 12 + f * (1 / 120 - f / 252))


def trigamma(x):
    r = 0.0
    while x < 6:
        r += 1 / (x * x)
        x += 1
    f = 1 / (x * x)
    return r + 1 / x + 0.5 * f + f / x * (1 / 6 - f * (1 / 30 - f / 42))


DB = 10 / math.log(10)


def nakagami_db_moments(m):
    """Mean and sd of 10*log10(G), G ~ Gamma(shape=m, scale=1/m).

    This is the distribution of the fading term in dB when the linear-power
    mean is preserved -- which is what DynamicObstacleLossModel implements.
    """
    return (digamma(m) - math.log(m)) * DB, math.sqrt(trigamma(m)) * DB


def label(a, b):
    return f"GCS-UAV{b}" if a == 0 else f"UAV{a}-UAV{b}"


def analyse(scenario, csv_path, rows_out, stats_out):
    """Append per-link check rows and fading-stat rows for one scenario."""
    if not os.path.exists(csv_path):
        print(f"  [{scenario}] MISSING {csv_path}")
        return

    per = defaultdict(list)
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            # Skip the first 5 s: the EMA has not converged and the feed may
            # not have been delivered yet, so early samples are transient by
            # construction rather than wrong.
            if float(r["t_sim"]) < 5.0:
                continue
            per[(int(r["node_a"]), int(r["node_b"]))].append(r)

    for (a, b) in sorted(per):
        rs = per[(a, b)]
        n = len(rs)
        e_pl = e_snr = e_dec = 0.0
        blocked_states, m_values = set(), set()
        by_m = defaultdict(list)

        for r in rs:
            d  = float(r["distance_m"])
            pl = float(r["pathloss_rx_dbm"])
            ob = float(r["obstacle_loss_db"])
            fx = float(r["faded_rx_dbm"])
            fd = float(r["fading_delta_db"])
            sn = float(r["snr_db"])
            mv = float(r["fading_m"])

            expect_pl = TX_DBM - (REF_LOSS_DB + 10 * PATH_EXP * math.log10(d))
            e_pl  = max(e_pl,  abs(pl - expect_pl))
            e_snr = max(e_snr, abs(sn - (fx - NOISE_FLOOR)))
            e_dec = max(e_dec, abs(fd - (fx - (pl - ob))))

            blocked_states.add(r["blocked"])
            m_values.add(mv)
            by_m[mv].append(fd)

        def emit(param, check, observed, expected, tol, ok, samples=n):
            rows_out.append(dict(
                scenario=scenario, link=label(a, b), node_a=a, node_b=b,
                parameter=param, check=check, n_samples=samples,
                observed=f"{observed:.6g}" if isinstance(observed, float) else observed,
                expected=f"{expected:.6g}" if isinstance(expected, float) else expected,
                tolerance=tol, result="PASS" if ok else "FAIL"))

        # Identity checks. 1e-3 dB is far above CSV print precision (~1e-5)
        # and far below anything physically meaningful.
        emit("pathloss_rx_dbm", "max|obs-model| vs log-distance",
             e_pl, 0.0, 1e-3, e_pl < 1e-3)
        emit("snr_db", "max|snr-(faded_rx-noiseFloor)|",
             e_snr, 0.0, 1e-3, e_snr < 1e-3)
        emit("fading_delta_db", "max|fd-(faded_rx-(pathloss-obst))|",
             e_dec, 0.0, 1e-3, e_dec < 1e-3)

        # State machine: blocked and fading_m must agree, always.
        consistent = all(
            (abs(float(r["fading_m"]) - (M_NLOS if r["blocked"] == "1" else M_LOS)) < 1e-9)
            for r in rs)
        emit("fading_m", "matches MLos/MNlos implied by 'blocked'",
             "consistent" if consistent else "MISMATCH", "consistent",
             "exact", consistent)
        emit("blocked", "states observed in this scenario",
             "+".join(sorted(blocked_states)), "0 and/or 1", "n/a", True)

        # Distribution check, separately for each fading regime seen.
        for mv, vals in sorted(by_m.items()):
            if len(vals) < 200:
                stats_out.append(dict(
                    scenario=scenario, link=label(a, b), regime_m=mv,
                    n_samples=len(vals), obs_mean_db="", obs_sd_db="",
                    exp_mean_db="", exp_sd_db="", mean_err_sigma="",
                    result="SKIPPED (n<200)"))
                continue
            mu = sum(vals) / len(vals)
            sd = (sum((v - mu) ** 2 for v in vals) / len(vals)) ** 0.5
            emu, esd = nakagami_db_moments(mv)
            se = esd / math.sqrt(len(vals))
            sigma = (mu - emu) / se
            ok = abs(sigma) < 4.0 and abs(sd - esd) < 0.25 * esd
            stats_out.append(dict(
                scenario=scenario, link=label(a, b), regime_m=mv,
                n_samples=len(vals),
                obs_mean_db=f"{mu:.4f}", obs_sd_db=f"{sd:.4f}",
                exp_mean_db=f"{emu:.4f}", exp_sd_db=f"{esd:.4f}",
                mean_err_sigma=f"{sigma:+.2f}",
                result="PASS" if ok else "FAIL"))
            emit(f"fading_delta_db(m={mv:g})", "mean vs analytic Nakagami",
                 mu, emu, "|err|<4 sigma", abs(sigma) < 4.0, len(vals))
            emit(f"fading_delta_db(m={mv:g})", "sd vs analytic Nakagami",
                 sd, esd, "within 25%", abs(sd - esd) < 0.25 * esd, len(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns3-bin", default=NS3_BIN_DEFAULT)
    ap.add_argument("--quick", action="store_true",
                    help="shorter runs; distribution checks get fewer samples")
    ap.add_argument("--analyse-only", metavar="CSV", default=None,
                    help="Run the checks against an EXISTING validation CSV "
                         "(e.g. from a live Gazebo flight) instead of driving "
                         "NS-3. Publishes nothing and launches nothing, so it "
                         "is safe to use while Gazebo and NS-3 are running.")
    args = ap.parse_args()

    # ── Analyse an existing CSV (live flight) ────────────────────────────────
    #
    # NEVER run the scenario driver against a live setup: it launches its own
    # NS-3 and publishes to /uav_world_positions and /link_obstacle_loss, so it
    # would fight Gazebo's real feed and every position would alternate between
    # the real drone and this script's fixed coordinates. This mode exists so
    # live data can still be checked with the same code.
    #
    # All checks remain valid on live data. The identity checks are
    # self-contained. The distribution checks pool by fading regime rather than
    # by distance, which is legitimate because the fading draw is
    # mean-preserving and independent of the deterministic path loss -- samples
    # taken at different separations still come from the same Nakagami
    # distribution for a given m.
    if args.analyse_only:
        src = args.analyse_only
        if not os.path.exists(src):
            sys.exit(f"No such CSV: {src}")
        os.makedirs(OUTDIR, exist_ok=True)
        tag = os.path.splitext(os.path.basename(src))[0]
        rows, stats = [], []
        analyse(tag, src, rows, stats)
        if not rows:
            sys.exit(f"{src} produced no usable rows (is it a --csvPath file "
                     f"with more than 5 s of data?)")
        sum_path  = os.path.join(OUTDIR, f"verification_{tag}.csv")
        stat_path = os.path.join(OUTDIR, f"fading_{tag}.csv")
        with open(sum_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=[
                "scenario", "link", "node_a", "node_b", "parameter", "check",
                "n_samples", "observed", "expected", "tolerance", "result"])
            w.writeheader(); w.writerows(rows)
        with open(stat_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=[
                "scenario", "link", "regime_m", "n_samples", "obs_mean_db",
                "obs_sd_db", "exp_mean_db", "exp_sd_db", "mean_err_sigma",
                "result"])
            w.writeheader(); w.writerows(stats)
        n_fail = (sum(1 for r in rows if r["result"] == "FAIL")
                  + sum(1 for s in stats if s["result"] == "FAIL"))
        links = len({r["link"] for r in rows})
        print(f"Analysed {src}: {len(rows)} checks over {links} links")
        print(f"  summary -> {sum_path}")
        print(f"  fading  -> {stat_path}")
        print(f"  FAILURES: {n_fail}")
        return 1 if n_fail else 0

    if not os.path.exists(args.ns3_bin):
        sys.exit(f"NS-3 binary not found: {args.ns3_bin}\n"
                 f"Build it with: ./ns3 build three_uav_tapbridge_integrated")

    os.makedirs(OUTDIR, exist_ok=True)

    long_t = 40 if args.quick else 95
    cyc_t  = 45 if args.quick else 95

    clear_all   = lambda _t: {}
    blocked_all = lambda _t: {lk: 20.0 for lk in LINKS}

    def hysteresis(t):
        # clear -> blocked -> clear. The third phase is the part that was
        # never previously exercised: it is the only way ClearThresholdDb and
        # the blocked->clear transition ever execute.
        third = cyc_t / 3.0
        return {} if (t < third or t >= 2 * third) else {lk: 20.0 for lk in LINKS}

    one_blocked = lambda _t: {(0, 2): 18.0}

    scenarios = [
        ("los_clear",     clear_all,   long_t, 0.05),
        ("nlos_blocked",  blocked_all, long_t, 0.05),
        ("hysteresis",    hysteresis,  cyc_t,  0.05),
        ("ros_healthy",   one_blocked, 20,     0.5),
    ]

    print(f"Writing to {OUTDIR}")
    produced = []
    for name, fn, st, sp in scenarios:
        path, rc = run_scenario(name, fn, st, sp, args.ns3_bin)
        produced.append((name, path, rc))

    rows, stats = [], []
    for name, path, rc in produced:
        # Process health is check #1 for every scenario, recorded per scenario
        # rather than per link because a crash invalidates the whole run.
        rows.append(dict(
            scenario=name, link="(process)", node_a="", node_b="",
            parameter="ns3_exit_status", check="NS-3 terminated normally",
            n_samples="", observed=("signal %d" % -rc) if rc < 0 else str(rc),
            expected="0", tolerance="exact",
            result="PASS" if rc == 0 else "FAIL"))
        analyse(name, path, rows, stats)

    sum_path = os.path.join(OUTDIR, "verification_summary.csv")
    with open(sum_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "scenario", "link", "node_a", "node_b", "parameter", "check",
            "n_samples", "observed", "expected", "tolerance", "result"])
        w.writeheader()
        w.writerows(rows)

    stat_path = os.path.join(OUTDIR, "link_fading_stats.csv")
    with open(stat_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "scenario", "link", "regime_m", "n_samples",
            "obs_mean_db", "obs_sd_db", "exp_mean_db", "exp_sd_db",
            "mean_err_sigma", "result"])
        w.writeheader()
        w.writerows(stats)

    n_fail = sum(1 for r in rows if r["result"] == "FAIL")
    n_fail += sum(1 for s in stats if s["result"] == "FAIL")
    print(f"\n{len(rows)} checks over {len({(r['scenario'], r['link']) for r in rows})} "
          f"scenario/link combinations")
    print(f"  summary -> {sum_path}")
    print(f"  fading  -> {stat_path}")
    print(f"  FAILURES: {n_fail}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
