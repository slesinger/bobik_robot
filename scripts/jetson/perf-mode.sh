#!/bin/bash
# Jetson TK1 performance mode (runtime only, reverts at reboot).
#   perf-mode.sh on   : CPU governor performance, cpuquiet off (all 4 cores online), GPU clock locked at 852 MHz
#   perf-mode.sh off  : CPU governor interactive, cpuquiet on, GPU clock back to dynamic scaling
# Measured 2026-11-23: without the lock the GPU idled at 72 MHz, which made GPU benchmarks about 12x slower
# until the clock ramped up. Performance mode costs more power; the amount was not measured.
GBUS=/sys/kernel/debug/clock/override.gbus
case "$1" in
  on)
    echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
    echo 0 | sudo tee /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable
    for i in 1 2 3; do echo 1 | sudo tee /sys/devices/system/cpu/cpu$i/online; done
    echo 852000000 | sudo tee $GBUS/rate
    echo 1 | sudo tee $GBUS/state ;;
  off)
    echo 0 | sudo tee $GBUS/state
    echo 1 | sudo tee /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable
    echo interactive | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ;;
  *) echo "usage: $0 on|off"; exit 1 ;;
esac
grep . /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable
echo online: $(cat /sys/devices/system/cpu/online)
echo gpu gbus rate: $(sudo cat /sys/kernel/debug/clock/gbus/rate)
