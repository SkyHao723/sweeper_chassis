#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P CHASSIS_WEB_PRESENT
grep -rln "chassis_web\|chassis_drive_web" /home/smart --include=*.sh --include=*.py --include=*.service 2>/dev/null --exclude-dir=.git --exclude-dir=build --exclude-dir=install | head
find /home/smart -maxdepth 3 -name 'chassis_web*' -o -maxdepth 3 -name 'chassis_drive_web*' 2>/dev/null | head

P DESKTOP_TT
cat /home/smart/Desktop/tt 2>&1

P ROSBAGS
ls -la --time-style=long-iso /home/smart/rosbags/ 2>&1 | head

P BASHRC_TAIL
sed -n '115,150p' /home/smart/.bashrc

P LY_GNSS_NAV
head -30 /home/smart/LY_GNSS/start_nav.sh
echo "--- README ---"
cat /home/smart/LY_GNSS/README.md

P PIP_SERIAL
python3 -c "import serial, sys; print('pyserial', serial.__version__)" 2>&1
python3 --version

P ROS2_BIN
which ros2
ls -l /opt/ros/humble/setup.bash
