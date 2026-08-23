#!/usr/bin/env bash
# Black-box recorder for the Pi's power and thermal state.
#
# The Pi has been dying abruptly - the journal simply stops mid-line with no
# shutdown sequence, which means it lost power rather than shut down. That
# leaves nothing to diagnose from. This samples the PMIC input rail, core
# current, temperature and throttle flags every few seconds so the LAST line
# before a cut shows what the board was doing as it died.
#
# Deliberately tiny: one line per sample, journald already rotates it, and it
# costs a vcgencmd call rather than anything that could itself load the board.
set -u
INTERVAL="${OMX_HEALTH_INTERVAL:-5}"

while true; do
    temp=$(vcgencmd measure_temp 2>/dev/null | tr -d "temp='C")
    thr=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
    ext5v=$(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null | awk -F= '{print $2}')
    core_a=$(vcgencmd pmic_read_adc VDD_CORE_A 2>/dev/null | awk -F= '{print $2}')
    load=$(cut -d' ' -f1-3 /proc/loadavg)
    printf 'temp=%sC throttled=%s ext5v=%s core=%s load=%s\n' \
           "${temp:-?}" "${thr:-?}" "${ext5v:-?}" "${core_a:-?}" "$load"
    sleep "$INTERVAL"
done
