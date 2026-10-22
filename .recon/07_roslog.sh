#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P ROS_LOG_DIRS
ls -la --time-style=long-iso /home/smart/.ros/log/ | tail -25

P ROS_LOG_LATEST
ls -la --time-style=long-iso /home/smart/.ros/log/latest/ 2>&1

P ROS_LOG_LATEST_CONTENT
for f in /home/smart/.ros/log/latest/*; do
  echo "=== $f"
  head -c 4000 "$f"
  echo
done

P ROS_LOG_CHASSIS_GREP
grep -rl "tb01\|scrubber_damiao\|chassis" /home/smart/.ros/log/ 2>/dev/null | tail -10

P ROS_LOG_GREP_MSG
grep -rh "打开串口\|串口已打开\|诊断帧版本\|上位机使能\|驱动器失能\|固件" /home/smart/.ros/log/ 2>/dev/null | tail -30 | cut -c1-220

P SELFTEST_VERSION
grep -n "version\|DIAG_VERSION\|f\[30\]" /home/smart/tb01_ws/src/tb01_chassis/tb01_chassis/selftest.py | head -20

P CHECK_SERIAL
sed -n '1,120p' /home/smart/tb01_ws/src/tb01_chassis/tb01_chassis/check_serial.py

P MID360_BRINGUP_CMDVEL
grep -n "cmd_vel\|wheeltec\|chassis\|odom_combined" /home/smart/LY_Nav/src/navigation/mid360_nav_bridge/launch/mid360_nav_bringup.launch.py | head -30
