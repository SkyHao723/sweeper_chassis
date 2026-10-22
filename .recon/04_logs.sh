#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P TB01_WS_LOG_DIRS
ls -la --time-style=long-iso /home/smart/tb01_ws/log/

P TB01_LOG_CONTENT
for f in /home/smart/tb01_ws/log/latest*/launch.log; do
  echo "=== $f"
  tail -n 60 "$f"
done 2>/dev/null
echo "--- find any launch.log ---"
find /home/smart/tb01_ws/log -name 'launch.log' -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null

P TB01_LOG_GREP
grep -rn "版本\|固件\|使能\|串口" /home/smart/tb01_ws/log/ 2>/dev/null | grep -v '\.meta' | tail -40 | cut -c1-200

P NOHUP_OUT
ls -l /home/smart/nohup.out
tail -50 /home/smart/nohup.out

P BASH_HISTORY_CHASSIS
grep -nE 'tb01|car_web|ros_drive|wheeltec|chassis|ros2 launch|ros2 run|python3 ' /home/smart/.bash_history | tail -80 | cut -c1-190

P BASH_HISTORY_TAIL
tail -60 /home/smart/.bash_history | cut -c1-190

P CAR_WEB_README_TOC
grep -n '^#' /home/smart/car_web/README.md | head -60

P CAR_WEB_README_HEAD
head -60 /home/smart/car_web/README.md

P CAR_WEB_VERSION_HINT
grep -n "DIAG_VERSION\|FUNC_FRAME\|版本\|固件 v" /home/smart/car_web/README.md | head -40 | cut -c1-200

P DESKTOP
ls -la --time-style=long-iso /home/smart/Desktop/

P BIN_DIR
ls -la --time-style=long-iso /home/smart/bin/

P AGENTS_DIR
ls -la --time-style=long-iso /home/smart/.agents/
