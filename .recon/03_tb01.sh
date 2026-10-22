#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P TB01_WS_TREE
find /home/smart/tb01_ws -type f -not -path '*/build/*' -not -path '*/install/*' -not -path '*/log/*' -not -name '*.pyc' -printf '%TY-%Tm-%Td %TH:%TM  %10s  %p\n' | sort -k4

P TB01_PKG_LS
ls -la --time-style=long-iso /home/smart/tb01_ws/src/tb01_chassis/
echo "--- pkg dir ---"
ls -la --time-style=long-iso /home/smart/tb01_ws/src/tb01_chassis/tb01_chassis/

P PACKAGE_XML
cat /home/smart/tb01_ws/src/tb01_chassis/package.xml 2>&1

P SETUP_PY
cat /home/smart/tb01_ws/src/tb01_chassis/setup.py 2>&1

P SETUP_CFG
cat /home/smart/tb01_ws/src/tb01_chassis/setup.cfg 2>&1

P RESOURCE
ls -la /home/smart/tb01_ws/src/tb01_chassis/resource/ 2>&1
cat /home/smart/tb01_ws/src/tb01_chassis/resource/* 2>&1

P DIAG_GREP
grep -rn "DIAG_VERSION\|FUNC_FRAME_MIN_VERSION\|PROTO_VERSION\|VERSION" /home/smart/tb01_ws/src/tb01_chassis/ 2>/dev/null | grep -v Binary

P TB01_WEB_LS
find /home/smart/tb01_web -type f -printf '%TY-%Tm-%Td %TH:%TM  %10s  %p\n' | sort

P CAR_WEB_LS
find /home/smart/car_web -type f -not -path '*__pycache__*' -printf '%TY-%Tm-%Td %TH:%TM  %10s  %p\n' | sort

P WHEELTEC_SRC_LIST
ls -la --time-style=long-iso /home/smart/wheeltec_ws/src/ | head -50

P TURN_ON_LIST
find /home/smart/wheeltec_ws/src/turn_on_wheeltec_robot -maxdepth 2 -printf '%TY-%Tm-%Td %TH:%TM  %p\n' 2>/dev/null | sort | head -60

P WHEELTEC_GIT
for d in /home/smart/wheeltec_ws/src/*/; do
  if [ -d "$d/.git" ]; then
    echo "--- $d"
    git -C "$d" log --oneline -2 2>&1
    git -C "$d" log -1 --format='%ci' 2>&1
    git -C "$d" remote -v 2>&1 | head -1
    git -C "$d" status --short 2>&1 | head -5
  fi
done
