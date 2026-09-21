#!/bin/bash
# 底盘数据网页的启动包装 —— systemd 用
#
# 为什么要包一层: ROS 环境(source setup.bash)必须在启动前弄好, 而 systemd
# 的 ExecStart 里塞不下这些; 另外 ROS_DOMAIN_ID 必须显式设 —— .bashrc 里虽然
# 有, 但它对非交互 shell 直接 return(踩过, 见 docs/ros-integration.md 第 5 节)。
#
# ★ 不要加 set -u: ROS 的 setup.bash 内部引用未定义变量, 会被打断。
set -o pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/smart_ws/install/local_setup.bash" 2>/dev/null

exec python3 "$HOME/chassis_tools/chassis_web.py" "$@"
