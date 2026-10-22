#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P TODAY_ROS_LOG_DIRS
ls -d /home/smart/.ros/log/2026-10-22* 2>/dev/null

P EPOCH_MAP
for e in 1792646528 1792646636 1792646731 1792647315 1792648152 1792648710 1792648747 1792649093 1792649245; do
  echo "$e -> $(date -d @$e '+%F %T')"
done

P LAUNCH_1426
cat "/home/smart/.ros/log/2026-10-22-14-26-46-766636-RK3576-21356/launch.log" 2>&1 | cut -c1-260

P LAUNCH_1325
cat "/home/smart/.ros/log/2026-10-22-13-25-30-522002-RK3576-54992/launch.log" 2>&1 | cut -c1-260

P CHASSIS_LOGS
for f in /home/smart/.ros/log/2026-10-22*/launch.log; do
  echo "=== $f"
  grep -i "chassis\|damiao\|wheeltec" "$f" 2>/dev/null | cut -c1-240 | head -30
done

P PY3_LOGS
for f in /home/smart/.ros/log/python3_*179264*.log /home/smart/.ros/log/python3_*179265*.log; do
  echo "=== $f"
  tail -40 "$f" 2>/dev/null | cut -c1-240
done
