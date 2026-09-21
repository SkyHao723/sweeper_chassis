#!/bin/bash
# ============================================================================
# 底盘看门狗 —— /odom 停发就自动重启整个底盘栈
#
# 为什么需要它: 这台车有两种会让底盘"看起来上电了其实没数据"的故障,
# respawn 只能处理其中一种:
#
#   1. 节点崩溃    (CH340 掉线 -> SerialException -> exit -6)
#      -> launch 里的 respawn=True 能处理
#
#   2. **节点卡死** (进程活着、串口开着、但不发数据、也不在 ROS 图里)
#      -> respawn **完全处理不了** —— 它只对"进程退出"生效, 卡死的进程不退出。
#         机理多半是 CH340 重新枚举后, 节点手里的 fd 指向了已经不存在的设备,
#         它读不到数据但也不报错, 就那么一直挂着。
#      -> 只能从外面把它连根拔起再重启。
#
# 内核日志里能看到硬件侧的根因 (usb disconnect + re-enumerate):
#   ch341-uart ttyUSB10: usb_serial_generic_read_bulk_callback - urb stopped: -32
#   usb 3-1.2: USB disconnect, device number 3
#   ... 1.2: new full-speed USB device number 4
# 所以**硬件上也要查**: USB 线/接头是不是松了(车开动时的震动会让它反复掉线)。
#
# 安装成 systemd 服务:
#   sudo cp chassis_watchdog.sh /usr/local/bin/
#   sudo cp chassis-watchdog.service /etc/systemd/system/
#   sudo systemctl daemon-reload && sudo systemctl enable --now chassis-watchdog
# ============================================================================

CHECK_EVERY=10          # 每多少秒查一次
LOG=/tmp/chassis_watchdog.log
LAUNCH="ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py"

# ★ 不要加 set -u —— ROS 的 setup.bash 内部引用未定义变量, 会被打断
set -o pipefail

source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/smart_ws/install/local_setup.bash" 2>/dev/null

log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }

odom_alive() {
    # 直接订阅, 不走 ros2 daemon —— daemon 缓存过期会误报"话题没发布"
    timeout 10 python3 - <<'PYEOF' 2>/dev/null
import sys, time, rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('wd_probe')
n = [0]
node.create_subscription(Odometry, '/odom', lambda m: n.__setitem__(0, n[0] + 1), 10)
t0 = time.time()
while time.time() - t0 < 4:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node(); rclpy.shutdown()
sys.exit(0 if n[0] > 20 else 1)
PYEOF
}

kill_all() {
    local left
    for _ in 1 2 3; do
        left=$(pgrep -f 'turn_on_wheeltec_robot|wheeltec_robot_node|ekf_node|static_transform_publisher|robot_state_publisher|joint_state_publisher' | tr '\n' ' ')
        [ -z "$left" ] && break
        kill -9 $left 2>/dev/null
        sleep 2
    done
}

log "看门狗启动 (每 ${CHECK_EVERY}s 检查一次 /odom)"

fails=0
while true; do
    if odom_alive; then
        if [ "$fails" -gt 0 ]; then log "/odom 恢复正常 (之前连续失败 $fails 次)"; fi
        fails=0
    else
        fails=$((fails + 1))
        # 连续两次都失败才动手 —— 避免恰好撞上节点正在 respawn 的那两秒
        if [ "$fails" -ge 2 ]; then
            log "!! /odom 连续 $fails 次没数据 -> 重启底盘栈"
            kill_all
            nohup $LAUNCH > /tmp/chassis_launch.log 2>&1 &
            log "已重新启动, 日志 /tmp/chassis_launch.log"
            fails=0
            sleep 20        # 给它时间起来
        fi
    fi
    sleep $CHECK_EVERY
done
