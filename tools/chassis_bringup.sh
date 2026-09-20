#!/bin/bash
# ============================================================================
# 底盘 ROS 栈的启动 / 停止 / 状态检查  (在 RK3588 上跑)
#
#   ./chassis_bringup.sh start     启动(后台), 并等到 /odom 真的有数据
#   ./chassis_bringup.sh stop      停掉全部相关进程
#   ./chassis_bringup.sh restart   重启
#   ./chassis_bringup.sh status    看节点 / /odom 频率 / 串口占用
#
# 为什么不用手动敲 ros2 launch:
#   - 这套栈有几个"看起来像挂了其实没挂 / 看着没挂其实挂了"的坑, 见下面注释
#   - 启动后要等一会儿才能真正出数据, 脚本帮你等到确认
# ============================================================================

# ★ 不要加 set -u (nounset)。
#   ROS 的 setup.bash / local_setup.bash 内部会引用未定义的变量, 开着 set -u
#   会在 source 的那一行直接把脚本打断、静默退出 —— 而且如果把 stderr 重定向
#   掉了, 现象就是"脚本什么都不输出、退出码 1", 很难查。踩过。
set -o pipefail

LOG=/tmp/chassis_bringup.log
LAUNCH="ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py"
PORT=/dev/wheeltec_controller

# 自己的绝对路径 —— restart 分支要递归调用自己, 光用 "$0" 在
# 工作目录不对时会报"未找到命令"(踩过)
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"

# 环境: .bashrc 里已经配好, 但脚本是非交互 shell, 这里显式再 source 一次
if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/humble/setup.bash
    source "$HOME/smart_ws/install/local_setup.bash"
fi

stop_all() {
    pkill -9 -f turn_on_wheeltec_robot  2>/dev/null
    pkill -9 -f wheeltec_robot_node     2>/dev/null
    pkill -9 -f ekf_node                2>/dev/null
    pkill -9 -f static_transform_publisher 2>/dev/null
    pkill -9 -f robot_state_publisher   2>/dev/null
    pkill -9 -f joint_state_publisher   2>/dev/null
    sleep 3
}

# /odom 有没有真的在出数据。**直接订阅, 不走 ros2 daemon** ——
# daemon 的缓存过期时会误报"话题没有发布"(报 Could not determine the type),
# 排查时被这个坑过一次。
odom_alive() {
    timeout 10 python3 - <<'PYEOF' 2>/dev/null
import sys, time, rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('odom_alive_probe')
n = [0]
node.create_subscription(Odometry, '/odom', lambda m: n.__setitem__(0, n[0] + 1), 10)
t0 = time.time()
while time.time() - t0 < 4:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node(); rclpy.shutdown()
sys.exit(0 if n[0] > 20 else 1)
PYEOF
}

case "${1:-status}" in
start)
    echo "== 清理旧进程 =="
    stop_all
    echo "== 启动 (日志: $LOG) =="
    nohup $LAUNCH > "$LOG" 2>&1 &
    echo "== 等 /odom 出数据 (最多 40 秒) =="
    for i in $(seq 1 8); do
        sleep 5
        if odom_alive; then
            echo "OK  /odom 已在 20Hz 发布 (等了约 $((i*5)) 秒)"
            echo
            echo "节点:"
            ros2 node list 2>/dev/null | sed 's/^/  /'
            exit 0
        fi
        echo "  ... 已等 $((i*5)) 秒"
    done
    echo "!! 40 秒还没等到 /odom。排查:"
    echo "   tail -20 $LOG"
    echo "   fuser -v $PORT"
    echo "   (node list 是空的但进程在? 试 ros2 daemon stop; ros2 daemon start)"
    exit 1
    ;;
stop)
    stop_all
    echo "已停止"
    ;;
restart)
    "$SELF" stop
    "$SELF" start
    ;;
status)
    echo "== 节点 =="
    ros2 node list 2>/dev/null | sed 's/^/  /' || echo "  (空)"
    echo
    echo "== 串口 =="
    fuser -v $PORT 2>&1 | tail -2 || echo "  (没人占用)"
    echo
    echo "== /odom =="
    if odom_alive; then echo "  有数据 (20Hz)"; else echo "  !! 没有数据"; fi
    ;;
*)
    sed -n '2,12p' "$0"
    exit 1
    ;;
esac
