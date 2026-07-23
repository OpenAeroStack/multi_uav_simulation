# test_logs — channel model validation evidence

Regenerate everything here with:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/test_scripts/run_channel_validation.py          # ~6 min
python3 scripts/test_scripts/run_channel_validation.py --quick  # ~3 min, fewer samples
```

The harness stands in for Gazebo (positions + ray-caster output) and drives
`three_uav_tapbridge_integrated` through four scenarios. Nothing here is
hand-edited; delete the directory and re-run to reproduce.

## Files

| File | Contents |
|---|---|
| `verification_summary.csv` | One row per check: scenario, link, parameter, observed, expected, tolerance, PASS/FAIL |
| `link_fading_stats.csv` | Fading distribution per link per regime: measured mean/sd vs analytic Nakagami, error in σ |
| `raw_<scenario>.csv` | The NS-3 per-link validation CSV for that scenario (every parameter, every sample) |
| `raw_<scenario>.log` | NS-3 stdout, including the startup banner and the integration check |

## Scenarios

| Scenario | Setup | Exercises |
|---|---|---|
| `los_clear` | all 6 links clear | LoS fading distribution, m = `MLos` = 3 |
| `nlos_blocked` | all 6 links at 20 dB | NLoS fading distribution, m = `MNlos` = 1 |
| `hysteresis` | clear → blocked → clear | `BlockThresholdDb` **and** `ClearThresholdDb`; the release path |
| `ros_healthy` | one blocked link of six | realistic mixed case, matches a live run |

## Checks per link

- `pathloss_rx_dbm` — recomputed independently as `Tx − (46.73 + 20·log₁₀ d)`
- `snr_db` — identity `= faded_rx_dbm − noiseFloor`
- `fading_delta_db` — identity `= faded_rx − (pathloss_rx − obstacle_loss)`
- `fading_delta_db` — distribution vs analytic Nakagami-m in dB
  (mean `(ψ(m) − ln m)·10/ln10`, sd `√ψ′(m)·10/ln10`), tolerance 4σ on the
  mean and 25 % on the sd
- `fading_m` — must equal `MLos`/`MNlos` implied by `blocked`, every sample
- `blocked` — both states observed
- `ns3_exit_status` — per scenario, not per link: a crash fails the run
  regardless of what the partial data says

The model constants at the top of `run_channel_validation.py` mirror the NS-3
defaults. Override either and the identity checks will report a mismatch — they
recompute the model independently rather than reading it back.

## Known gaps (not covered by anything here)

- **No real Gazebo.** The harness emits the format the ray-caster is believed
  to emit; both were written by the same hand, so a shared misunderstanding
  would not be caught.
- **No TapBridge, no traffic.** Every PHY counter reads zero, so the PHY
  delivery counters and the `MonitorSnifferRx` per-packet SNR log
  (`--snrLogFile`) have never produced a row.
- **Single geometry.** Positions are fixed per run; mobility-driven distance
  variation is not swept.

## Analysing a live Gazebo flight

**Never run the scenario driver while Gazebo is up.** It launches its own NS-3
and publishes to `/uav_world_positions` and `/link_obstacle_loss`, so it would
fight the real feed and every position would alternate between the real drone
and the harness's fixed coordinates at 10 Hz.

For a live run, let NS-3 write the CSV itself and analyse it afterwards:

```bash
# 1. live flight (Gazebo + world_pos_publisher + SITL already running)
ns3.38-three_uav_tapbridge_integrated-default \
    --csvPath=test_logs/live_flight.csv --snrLogFile=test_logs/live_snr.csv

# 2. same checks, no publishing, no NS-3 launched -- safe alongside a live run
python3 scripts/test_scripts/run_channel_validation.py --analyse-only test_logs/live_flight.csv
```

Outputs `verification_live_flight.csv` and `fading_live_flight.csv`.

All checks stay valid on live data: the identity checks are self-contained, and
the distribution checks pool by fading regime rather than by distance --
legitimate because the fading draw is mean-preserving and independent of the
deterministic path loss, so samples at different separations still come from
the same Nakagami distribution for a given m.

## Recording a live flight (`record_live_links.py`)

A PASSIVE recorder. It subscribes to the four link topics and writes CSV; it
publishes nothing and launches nothing, so it is safe to start and stop at any
point during a flight.

```bash
source /opt/ros/humble/setup.bash
python3 scripts/test_scripts/record_live_links.py --tag mission1          # until Ctrl-C
python3 scripts/test_scripts/record_live_links.py --tag mission1 --duration 300 --rate 5
```

Writes `live_<tag>_<UTC>_samples.csv`, `_events.csv` and `_summary.csv`.

| Output | Contents |
|---|---|
| `_samples.csv` | one row per link per tick: distance, RAW obstacle loss, RSSI, SNR, plus the age of each input feed |
| `_events.csv` | one row per link state change (BLOCKED / CLEARED / LOSS_CHANGE), detected at the full 10 Hz feed rate |
| `_summary.csv` | per link: sample count, % blocked, distance and SNR min/mean/max, peak obstacle loss |

It complements NS-3's `--csvPath` rather than replacing it:

- `--csvPath` logs the **EMA-smoothed** obstacle loss and the internal
  `blocked` / `fading_m` / `known` state.
- the recorder logs the **raw** ray-caster output off the topic, so comparing
  the two shows the smoothing itself and lets you sanity-check `EmaAlpha`.
- events are caught at 10 Hz, so a transition lasting a single ray-cast tick
  is recorded even at a 2 Hz sample rate.
- every row carries `pos_age_s` / `obs_age_s` / `ns3_age_s`, so a dead
  publisher shows up as a climbing age instead of silently repeated values.

Run both together for a complete picture.
