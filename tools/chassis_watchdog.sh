#!/bin/bash
# ============================================================================
# 底盘看门狗 —— /odom 真的停发才重启整个底盘栈
#
# ★ 教训: 第一版写得太激进, 反而把系统搞不稳定了。
#   它每 10 秒新建一个 ROS 节点、只转 4 秒、要求收到 >20 条才算活。
#   问题是**新建节点的 DDS 发现本身可能就要好几秒**, 于是栈健康的时候也会
#   判失败; 连续两次失败(20 秒)就杀掉重启。实测日志里能看到它在一个健康
#   的栈上反复重启 —— 而**每次重启 /odom 位姿归零, 正在进行的任何测量都会
#   被截断**, 表现是"车明明动了但里程计只涨了一点点", 且完全不可复现。
#   查了很久才发现是自己加的看门狗在捣乱。
#
#   现在的策略:
#     1. **进程没了 -> 立即重启**(这个判据不依赖 DDS, 最可靠)
#     2. 数据探测要**连续失败 3 次(30 秒)才动手** —— 给 DDS 发现和节点 respawn
#        留足时间
#     3. 探测本身也放宽: 转 6 秒、阈值降到 >10 条
#
# 为什么需要它: 这台车有两种"看起来上电了其实没数据"的故障, respawn 只能
# 处理其中一种 ——
#   1. 节点崩溃(CH340 掉线 -> SerialException -> exit -6): respawn 能处理
#   2. **节点卡死**(进程活着、串口开着、但不发数据): respawn 处理不了,
#      因为它不退出。只能从外面连根拔起。
#
# 硬件侧的根因在内核日志里能看到(CH340 会 USB 层掉线再枚举):
#   ch341-uart ttyUSB10: urb stopped: -32
#   usb 3-1.2: USB disconnect -> new full-speed USB device
# 所以 USB 线/接头也要查。
# ============================================================================

CHECK_EVERY=10              # 每多少秒查一次
FAILS_TO_RESTART=3          # 连续失败多少次才重启(=30 秒)
LOG=/tmp/chassis_watchdog.log
LAUNCH="ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py"

# ★ 不要加 set -u —— ROS 的 setup.bash 内部引用未定义变量, 会被打断
set -o pipefail

if [ -z "${ROS_DOMAIN_ID:-}" ]; then export ROS_DOMAIN_ID=5; fi
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/smart_ws/install/local_setup.bash" 2>/dev/null

log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }

# 数据探测: 放宽到 6 秒 / >10 条。宁可漏报也不能误报 —— 误报的代价是把一个
# 正在正常工作的栈杀掉, 而漏报的代价只是晚 30 秒重启。
odom_alive() {
    timeout 15 python3 - <<'PYEOF' 2>/dev/null
import sys, time, rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('wd_probe')
n = [0]
node.create_subscription(Odometry, '/odom', lambda m: n.__setitem__(0, n[0] + 1), 10)
t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node(); rclpy.shutdown()
sys.exit(0 if n[0] > 10 else 1)
PYEOF
}

# 进程还在不在 —— 不依赖 DDS, 最可靠的判据。
# ★ 模式开头那个 / 不能省。不加的话 pgrep -f 会匹配到**任何**命令行里出现这串
#   字的进程 —— 实测它匹配到了我自己那条 `pgrep -af wheeltec_robot_node` 的
#   ssh 命令行。节点真死了却因为这种巧合被判定为"还活着", 看门狗就白装了。
#   实际的命令行是 .../lib/turn_on_wheeltec_robot/wheeltec_robot_node, 带斜杠能对上。
node_running() {
    pgrep -f '/wheeltec_robot_node' > /dev/null 2>&1
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

restart_stack() {
    log "!! $1 -> 重启底盘栈"
    kill_all
    nohup $LAUNCH > /tmp/chassis_launch.log 2>&1 &
    log "已重新启动, 日志 /tmp/chassis_launch.log"
    sleep 20            # 给它时间起来
}

log "看门狗启动 (每 ${CHECK_EVERY}s 查一次; 数据探测要连续失败 ${FAILS_TO_RESTART} 次才重启)"

fails=0
last_beat=$(date +%s)
while true; do
    if ! node_running; then
        # 进程都没了, 没什么好犹豫的
        restart_stack "底盘节点进程不在了"
        fails=0
    elif odom_alive; then
        if [ "$fails" -gt 0 ]; then
            log "/odom 正常 (之前连续失败 $fails 次, 没到 $FAILS_TO_RESTART 次所以没动手)"
        fi
        fails=0
    else
        fails=$((fails + 1))
        log "/odom 探测失败第 $fails 次 (进程还在, 再观察)"
        if [ "$fails" -ge "$FAILS_TO_RESTART" ]; then
            restart_stack "/odom 连续 $fails 次没数据(节点卡死?)"
            fails=0
        fi
    fi

    # 心跳: 健康时不记日志, 结果"日志里半天没动静"分不清是健康还是看门狗自己
    # 死了。每 5 分钟记一条, 超过 5 分钟没动静就说明看门狗本身出问题了。
    now=$(date +%s)
    if [ $((now - last_beat)) -ge 300 ]; then
        log "心跳: 正常 (节点进程在, /odom 在线)"
        last_beat=$now
    fi

    sleep $CHECK_EVERY
done
