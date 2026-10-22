#!/bin/bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

info() {
  d="$1"
  if [ ! -d "$d" ]; then echo; echo "########## MISSING: $d ##########"; return; fi
  echo
  echo "##################### $d #####################"
  stat -c '  BIRTH=%w  MODIFY=%y  ACCESS=%x' "$d"
  echo "  size=$(du -sh "$d" 2>/dev/null | cut -f1)  files=$(find "$d" -type f 2>/dev/null | wc -l)"
  echo "  OLDEST3 (mtime):"
  find "$d" -type f -printf '    %TY-%Tm-%Td %TH:%TM  %p\n' 2>/dev/null | sort | head -3
  echo "  NEWEST3 (mtime):"
  find "$d" -type f -printf '    %TY-%Tm-%Td %TH:%TM  %p\n' 2>/dev/null | sort | tail -3
  echo "  TOPLEVEL:"
  ls -la --time-style=long-iso "$d" 2>/dev/null | sed 1d | head -30 | cut -c1-140
  if [ -d "$d/.git" ]; then
    echo "  +++ GIT +++"
    echo "    branch: $(git -C "$d" rev-parse --abbrev-ref HEAD 2>&1)"
    echo "    last3: $(git -C "$d" log --oneline -3 2>&1 | tr '\n' '|')"
    echo "    lastdate: $(git -C "$d" log -1 --format='%ci' 2>&1)"
    echo "    firstdate: $(git -C "$d" log --reverse --format='%ci' 2>&1 | head -1)"
    echo "    commits: $(git -C "$d" rev-list --count HEAD 2>&1)"
    echo "    remote: $(git -C "$d" remote -v 2>&1 | tr '\n' '|')"
    echo "    status:"
    git -C "$d" status --short 2>&1 | head -25
  else
    echo "  (not a git repo)"
  fi
}

info /home/smart/wheeltec_ws
info /home/smart/tb01_ws
info /home/smart/tb01_web
info /home/smart/car_web
info /home/smart/wheeltec_multi_mode2
info /home/smart/robot_scripts
info /home/smart/nav2_ws
info /home/smart/LY_Nav
info /home/smart/LY_GNSS
info /home/smart/LY_Guide
info /home/smart/LY_INSPECT
info /home/smart/catkin_point_lio_unilidar
info /home/smart/inspection_hmi_rk3576
info /home/smart/yolo_test
info /home/smart/.git
