#!/bin/bash
# 排查"发 0 速没停下来": 只读, 不发任何指令, 车不会动。
# 要点是**同时**记录 /cmd_vel 和 /odom —— 才能区分
#   (A) 命令侧: /cmd_vel 上还有别人在发非零  -> 命令根本没变成 0
#   (B) 执行侧: /cmd_vel 确实是 0 但车还在走 -> 驱动器刹车弱/溜坡/滑行
#
# ★ 不要加 set -u! ROS 的 setup.bash 引用了未绑定的 AMENT_TRACE_SETUP_FILES,
#   在 set -u 下 source 会把脚本直接打死 —— 症状是"零输出 + 退出码 1"。
#   本仓库 chassis_bringup.sh / chassis_watchdog.sh / chassis_web.sh 里都有同样的警告。
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/smart_ws/install/setup.bash 2>/dev/null

echo "===== 0. 时间 / 谁在跑 ====="
date '+%H:%M:%S'
uptime
ps -eo pid,etimes,cmd | grep -E 'wheeltec_robot_node|ekf_node|nav2|teleop|chassis_web|rviz' | grep -v grep | head -20

echo
echo "===== 1. /cmd_vel 上有几个发布者、分别是谁 ====="
echo "    ★★ 一个订阅者 + N 个发布者 = 谁最后发谁算数。发了 0 却不停车,"
echo "       第一嫌疑就是这里还有别人在发非零(2026-10 实测就是 controller_server)。"
timeout 20 ros2 topic info /cmd_vel 2>&1 | head -4
timeout 20 ros2 topic info -v /cmd_vel 2>&1 | grep -E 'Node name|Endpoint type' | paste - - 2>/dev/null | sed 's/Node name: //; s/Endpoint type: //' | sort | uniq -c

echo
echo "===== 2. 安全/回充标志现在的值 (非 0 会让所有速度帧被丢掉) ====="
for t in /chassis_security /robot_recharge_flag /robot_charging_flag; do
  echo "-- $t"
  timeout 6 ros2 topic info "$t" 2>&1 | head -3
  timeout 5 ros2 topic echo "$t" --once 2>&1 | head -3
done

echo
echo "===== 3. 同时记录 /cmd_vel 和 /odom 共 15 秒 ====="
echo "     ★ 这段时间里请**发一次 0 速**(或让车保持你要复现的状态)"
echo "     cmd_vx / cmd_wz 是主机下发的; odom_vx / odom_wz 是底盘实测的"
python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
n = Node('zero_stop_probe')
state = {'cv': None, 'cw': None, 'ct': 0.0, 'ov': None, 'ow': None, 'ot': 0.0}

def cb_cmd(m):
    state['cv'] = m.linear.x; state['cw'] = m.angular.z; state['ct'] = time.time()
def cb_odom(m):
    state['ov'] = m.twist.twist.linear.x
    state['ow'] = m.twist.twist.angular.z
    state['ot'] = time.time()

n.create_subscription(Twist, '/cmd_vel', cb_cmd, 20)
n.create_subscription(Odometry, '/odom', cb_odom, 20)

t0 = time.time()
print("  t     cmd_vx   cmd_wz  |  odom_vx  odom_wz   cmd_age odom_age")
while time.time() - t0 < 15.0:
    rclpy.spin_once(n, timeout_sec=0.05)
    now = time.time()
    el = now - t0
    if int(el * 5) != int((el - 0.05) * 5):      # 每 0.2s 一行
        cv = "%7.3f" % state['cv'] if state['cv'] is not None else "   None"
        cw = "%7.3f" % state['cw'] if state['cw'] is not None else "   None"
        ov = "%8.3f" % state['ov'] if state['ov'] is not None else "    None"
        ow = "%8.3f" % state['ow'] if state['ow'] is not None else "    None"
        ca = "%7.2f" % (now - state['ct']) if state['ct'] else "   None"
        oa = "%7.2f" % (now - state['ot']) if state['ot'] else "   None"
        print("%6.2f %s %s  | %s %s   %s %s" % (el, cv, cw, ov, ow, ca, oa))
n.destroy_node()
rclpy.shutdown()
PY
echo
echo "===== 4. 结论怎么读 ====="
echo "  第 1 节里除了 stop/teleop 还有 controller_server/behavior_server 在发"
echo "     -> (A) 命令侧: 你的 0 被 Nav2 20Hz 的非零命令盖掉了。**这是最常见的原因**。"
echo "        证据链: 只要还有非零速度帧进来, STM32 就一直在跟它跑;"
echo "        反过来说, 如果 0 真的生效了、之后又没有任何速度帧, STM32 自己的"
echo "        800ms 运动看门狗也会停车 —— 所以'完全不停'只能是别人在压着发。"
echo "  cmd_vx 变成 0 而 odom_vx 长时间不为 0  -> (B) 执行侧: 刹车弱/滑行/溜坡"
echo "  cmd_vx = None                        -> 根本没人发 /cmd_vel"
echo
echo "  修法(按推荐顺序):"
echo "    1) 让'停'先去停源头: 取消 Nav2 goal / 把 controller_server 失活,"
echo "       再持续发 0(≥20Hz 发 0.5~1 秒), 最后才杀进程。"
echo "    2) 装/写一个速度仲裁(twist_mux 之类): 急停/遥操高优先级, Nav2 低优先级。"
echo "       只发一两帧 0 是没用的 —— 会被 20Hz 的非零命令在 50ms 内覆盖。"
