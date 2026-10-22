#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
P() { echo; echo "##################### $* #####################"; }

P MULTI_MODE2
sed -n '1,40p' /home/smart/wheeltec_multi_mode2/run_follower.sh
echo "--- grep cmd_vel ---"
grep -rn "cmd_vel" /home/smart/wheeltec_multi_mode2/src /home/smart/wheeltec_multi_mode2/config 2>/dev/null | head -10
echo "--- formation.yaml ---"
cat /home/smart/wheeltec_multi_mode2/config/formation.yaml 2>&1

P TB01WEB_REINIT_UI
grep -n "reinit\|版本\|firmware" /home/smart/tb01_web/static/drive.js | head -20
echo "--- html ---"
grep -n "reinit\|固件\|版本" /home/smart/tb01_web/static/drive.html | head -20

P GIT_BASELINE
for d in /home/smart/LY_Nav /home/smart/LY_INSPECT; do
  echo "=== $d"
  git -C "$d" status --short 2>&1 | head -10
  git -C "$d" log -1 --format='%h %ci %s' 2>&1
done

P RECENT_MODIFIED FILES_15MIN
find /home/smart -maxdepth 2 -newermt '2026-10-22 15:30' -not -path '*/.ros/*' -not -path '*/.cache/*' -not -path '*/.local/*' -not -path '*/.trae-cn-server/*' -not -path '*/.vscode-server/*' -not -path '*/.config/*' -not -path '*/.npm/*' -not -path '*/.git/*' -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null | head -40

P TMP_SCRIPTS
ls -la /tmp/*.sh 2>/dev/null

P SERIAL_STATE_NOW
ls -l /dev/wheeltec_controller
stat -c '%n %y' /dev/ttyUSB0
echo "--- any process holding ttyUSB0 ---"
fuser -v /dev/ttyUSB0 2>&1 || echo "(nobody holds it)"

P PGPRE
pgrep -af "tb01|chassis|car_web|ros_drive|wheeltec"
echo "(exit $?)"
