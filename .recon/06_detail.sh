#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P TB01_INSTALL_VS_SRC
ls -la --time-style=long-iso /home/smart/tb01_ws/install/tb01_chassis/lib/tb01_chassis/ 2>&1
echo "--- share config ---"
ls -la --time-style=long-iso /home/smart/tb01_ws/install/tb01_chassis/share/tb01_chassis/config/ 2>&1
echo "--- python pkg ---"
ls -la --time-style=long-iso /home/smart/tb01_ws/build/tb01_chassis/build/lib/tb01_chassis/ 2>&1
echo "--- md5 src vs install/build ---"
md5sum /home/smart/tb01_ws/src/tb01_chassis/tb01_chassis/node.py /home/smart/tb01_ws/build/tb01_chassis/build/lib/tb01_chassis/node.py 2>&1
md5sum /home/smart/tb01_ws/src/tb01_chassis/tb01_chassis/protocol.py /home/smart/tb01_ws/build/tb01_chassis/build/lib/tb01_chassis/protocol.py 2>&1
echo "--- last build log ---"
ls -la --time-style=long-iso /home/smart/tb01_ws/log/build_2026-10-22_14-26-30/tb01_chassis/ 2>&1
tail -20 /home/smart/tb01_ws/log/build_2026-10-22_14-26-30/tb01_chassis/streams.log 2>&1

P LY_INSPECT_STM32
cat /home/smart/LY_INSPECT/src/hardware/hardware/scripts/install_stm32_sensor_udev_rules.sh 2>&1
echo "--- hub node port ---"
grep -n "port\|ttyUSB\|baud" /home/smart/LY_INSPECT/src/hardware/hardware/hardware/environmental/stm32_hub_node.py | head -20
echo "--- hardware launch ---"
find /home/smart/LY_INSPECT/src/hardware -name '*.py' -path '*launch*' -printf '%p\n' | head
grep -rn "ttyUSB\|stm32" /home/smart/LY_INSPECT/src/hardware/hardware/config/*.yaml 2>/dev/null | head -20

P LY_INSPECT_BRINGUP
grep -n "ttyUSB\|wheeltec\|chassis\|base\|serial" /home/smart/LY_INSPECT/src/control_center/system_manager/system_manager/system/robot_bringup.py | head -40

P LY_GUIDE_PORT
grep -rn "port\|app.run\|uvicorn" /home/smart/LY_Guide/backend/*.py /home/smart/LY_Guide/backend/routes/*.py 2>/dev/null | head -10
cat /home/smart/LY_Guide/start.sh

P LY_NAV_MAP2D_WEB
cat /home/smart/LY_Nav/scripts/run_map2d_web.sh | head -40

P LY_NAV_CHASSIS_REF
grep -rln "turn_on_wheeltec_robot\|wheeltec_robot_node\|cmd_vel" /home/smart/LY_Nav/src /home/smart/LY_Nav/scripts 2>/dev/null | head -20

P HMI_TOPIC
grep -rn "cmd_vel" /home/smart/inspection_hmi_rk3576/inspection_hmi_qt/src 2>/dev/null | head -10
grep -rn "cmd_vel" /home/smart/inspection_hmi_rk3576/inspection_hmi_qt/config/*.json 2>/dev/null | head

P PORT_QUICK
ss -lntp 2>/dev/null | grep -E ':(8000|8080|8081|8899|5000|5001|3000)' || echo "no web port listening"

P ROS_ENV_TEST
ls -la /home/smart/tb01_ws/install/setup.bash /home/smart/tb01_ws/install/local_setup.bash 2>&1
