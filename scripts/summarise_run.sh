#!/bin/bash
# Summarise one run: edge inference, ns-3 delivery to the GCS, board thermals.
# Reads only logs, so it is safe to run at any time on any archived run.
#
#   ./scripts/summarise_run.sh                    # the live logs in /tmp
#   ./scripts/summarise_run.sh ~/results/2026...  # an archived run

set -uo pipefail

DIR="${1:-/tmp}"
BOARDS="${BOARDS:-1 2}"

[[ -d "$DIR" ]] || { echo "ERROR: no such directory: $DIR" >&2; exit 1; }

printf '%s\n' "═══════════════════════════════════════════════════════════════"
printf ' Run summary: %s\n' "$DIR"
printf '%s\n' "═══════════════════════════════════════════════════════════════"

for i in $BOARDS; do
    det="$DIR/detector_uav$i.log"
    gcs="$DIR/gcs_receiver_uav$i.log"
    thr="$DIR/thermal_uav$i.log"

    printf '\n── UAV%s ──────────────────────────────────────────────────────\n' "$i"

    # ── Edge inference (on the Pi) ──────────────────────────────────────────
    # Frame #1 is excluded everywhere: it is the OpenVINO compile, not steady
    # state, and at 10-13 s it would dominate every average.
    if [[ -s "$det" ]]; then
        awk '
            match($0, /\[Detector\] #[0-9]+/) {
                n++
                if (match($0, /\[INFO\] \[[0-9.]+\]/)) {
                    t = substr($0, RSTART+8, RLENGTH-9) + 0
                    if (!t0) t0 = t
                    t1 = t
                }
                if (match($0, /\| [0-9]+ detections/)) {
                    d = substr($0, RSTART+2, RLENGTH-13) + 0
                    if (d > 0) hits++
                }
                if (n > 1 && match($0, /inference=[0-9]+/)) {
                    v = substr($0, RSTART+10, RLENGTH-10) + 0
                    sum += v; c++
                    if (v > mx) mx = v
                    if (mn == 0 || v < mn) mn = v
                }
            }
            END {
                dur = t1 - t0
                printf "  edge    : %d frames in %.1f s = %.2f fps processed\n",
                       n, dur, (dur > 0 ? n/dur : 0)
                if (c) printf "  inference: mean %.0f ms | min %d | max %d | ceiling %.2f fps\n",
                       sum/c, mn, mx, 1000/(sum/c)
                printf "  detected: %d frames contained a person (%.1f %%)\n",
                       hits, (n ? 100*hits/n : 0)
            }' "$det"
    else
        printf '  edge    : no log (%s)\n' "$det"
    fi

    # ── Delivery to the GCS (across ns-3) ───────────────────────────────────
    # Every detection here crossed the simulated radio; the detector log is the
    # send side, so the two counts together give the delivery ratio.
    if [[ -s "$gcs" ]]; then
        sent=$(grep -c '\[Detector\] #' "$det" 2>/dev/null || echo 0)
        awk -v sent="$sent" '
            match($0, /size=[0-9]+B/) {
                b = substr($0, RSTART+5, RLENGTH-6) + 0
                n++; sum += b
                if (b > mx) mx = b
                if (mn == 0 || b < mn) mn = b
            }
            END {
                printf "  gcs     : %d detection messages received over ns-3\n", n
                if (n) printf "  payload : mean %.0f B | min %d | max %d\n", sum/n, mn, mx
                if (sent > 0)
                    printf "  delivery: %d/%d = %.1f %% reached the GCS\n",
                           n, sent, 100*n/sent
            }' "$gcs"
    else
        printf '  gcs     : no log (%s)\n' "$gcs"
    fi

    # ── Board thermals ──────────────────────────────────────────────────────
    # throttled=0x0 means the board never throttled. Any other value means a
    # timing comparison against a run that stayed cool is not valid.
    if [[ -s "$thr" ]]; then
        awk '
            match($0, /temp=[0-9.]+/) {
                t = substr($0, RSTART+5, RLENGTH-5) + 0
                n++; sum += t
                if (t > mx) mx = t
            }
            # A clock below its maximum IS throttling, measured directly —
            # stronger evidence than temperature, which only implies it.
            match($0, /frequency\([0-9]+\)=[0-9]+/) {
                split(substr($0, RSTART, RLENGTH), a, "=")
                f = a[2] / 1000000
                fn++; fsum += f
                if (f > fmx) fmx = f
                if (fmn == 0 || f < fmn) fmn = f
            }
            match($0, /volt=[0-9.]+V/) {
                v = substr($0, RSTART+5, RLENGTH-6) + 0
                vn++; vsum += v
            }
            match($0, /load=[0-9.]+/) {
                l = substr($0, RSTART+5, RLENGTH-5) + 0
                ln++; lsum += l
                if (l > lmx) lmx = l
            }
            match($0, /throttled=0x[0-9a-fA-F]+/) {
                s = substr($0, RSTART+10, RLENGTH-10)
                if (s != "0x0") flag = s
            }
            END {
                printf "  thermal : %d samples | mean %.1f C | peak %.1f C | throttled %s\n",
                       n, (n ? sum/n : 0), mx, (flag ? flag : "0x0 (never)")
                # 25 MHz threshold: measure_clock reports exact Hz, so a
                # steady clock still varies by a few hundred Hz run to run.
                if (fn) printf "  clock   : mean %.0f MHz | min %.0f | max %.0f%s\n",
                       fsum/fn, fmn, fmx,
                       (fmx - fmn > 25 ? "   <- clock was reduced" : "")
                if (vn) printf "  core    : mean %.3f V\n", vsum/vn
                if (ln) printf "  load    : mean %.2f | peak %.2f (of 4 cores)\n",
                       lsum/ln, lmx
            }' "$thr"
    else
        printf '  thermal : no log (%s)\n' "$thr"
    fi
done

printf '\n'
