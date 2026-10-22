#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P UDEV_RULES
ls -l /etc/udev/rules.d/ 2>&1
echo "--- grep wheeltec ---"
grep -rn "wheeltec\|1a86\|ttyUSB" /etc/udev/rules.d/ 2>/dev/null

P DEV_SYMLINK
ls -l /dev/wheeltec_controller /dev/wheeltec_lidar 2>&1

P VENDOR_NODE_BIN
ls -l /home/smart/wheeltec_ws/install/turn_on_wheeltec_robot/lib/turn_on_wheeltec_robot/ 2>&1

P WHEELTEC_PARAM
cat /home/smart/wheeltec_ws/src/turn_on_wheeltec_robot/config/wheeltec_param.yaml 2>&1

P VENDOR_CPP_HEAD
grep -n "usart_port_name\|baud\|0x7B\|0x7b\|BUFFERSIZE\|serial\|read\|cmd_vel\|odom" /home/smart/wheeltec_ws/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp | head -60

P VENDOR_CPP_SIZE
wc -l /home/smart/wheeltec_ws/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp

P ROBOT_SCRIPTS
for f in /home/smart/robot_scripts/*.sh; do echo "=== $f"; cat "$f"; done

P MULTI_MODE2_SRC
ls -la /home/smart/wheeltec_multi_mode2/src/
find /home/smart/wheeltec_multi_mode2/src -maxdepth 2 -printf '%p\n' | head -30
echo "--- run_leader ---"
cat /home/smart/wheeltec_multi_mode2/run_leader.sh

P CMDVEL_GREP_HOME
grep -rln "cmd_vel" /home/smart --include=*.py --include=*.sh --include=*.cpp --include=*.launch.py --include=*.yaml 2>/dev/null --exclude-dir=build --exclude-dir=install --exclude-dir=log --exclude-dir=.git --exclude-dir=third_party --exclude-dir=vendor --exclude-dir=node_modules | grep -v '^/home/smart/wheeltec_ws/src/navigation2' | head -40

P SERIAL_GREP_HOME
grep -rln "ttyUSB\|wheeltec_controller" /home/smart --include=*.py --include=*.sh --include=*.cpp --include=*.launch.py --include=*.yaml --include=*.service 2>/dev/null --exclude-dir=build --exclude-dir=install --exclude-dir=log --exclude-dir=.git --exclude-dir=third_party --exclude-dir=vendor --exclude-dir=node_modules | head -40

P LY_NAV_LAYOUT
ls -la /home/smart/LY_Nav/src/
ls -la /home/smart/LY_Nav/scripts/ 2>/dev/null

P INSPECT_LAYOUT
ls -la /home/smart/LY_INSPECT/src/
ls -la /home/smart/LY_INSPECT/scripts/

P SYSTEMD_FILE_LY
cat /etc/systemd/system/ly-gnss-web.service /etc/systemd/system/ly-gnss-driver.service 2>&1
