#!/bin/bash
# 测 cmd_vel_guard: 车现在是静止的, 所以这个测试**不会让车动**
# (它只发 0)。要验证的是: 节点起得来、服务在、burst 真的按 20Hz 发 0、
# 以及"停没停"的判定能从 /odom 读出来。
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/smart_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=5

echo "===== 0. 语法 ====="
python3 -m py_compile /tmp/cvg.py && echo "py_compile OK" || exit 1

echo
echo "===== 1. 起服务(后台, 脱离本会话) ====="
# ★ 必须用**脚本路径**匹配, 不能用节点名/源文件名: 进程命令行是 "python3 /tmp/cvg.py",
#   拿 'cmd_vel_guard.py' 去 pkill -f 是匹配不到的 -> 旧进程不死, 两个进程同时提供
#   /chassis_stop, 你会拿到**旧代码**的回答却以为在测新代码(这次就栽在这)。
pkill -f '/tmp/cvg\.py' 2>/dev/null
sleep 2
left=$(ps -eo pid,cmd | grep '[c]vg\.py' | wc -l)
echo "残留旧进程: $left (必须是 0)"
if [ "$left" != "0" ]; then ps -eo pid,cmd | grep '[c]vg\.py'; exit 1; fi
setsid nohup python3 /tmp/cvg.py --hold 1.0 --max 1.0 > /tmp/cvg.log 2>&1 &
sleep 4
echo "-- 日志 --"; cat /tmp/cvg.log
echo "-- 进程 --"; ps -eo pid,cmd | grep '[c]vg\.py' | head -2
echo "-- 服务在不在 --"
timeout 12 ros2 service list 2>&1 | grep -i chassis_stop

echo
echo "===== 2. 一边订 /cmd_vel 一边调服务 ====="
python3 - <<'PY'
import rclpy, time, threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

rclpy.init()
n = Node('guard_test')
got = []
t0 = time.time()
n.create_subscription(Twist, '/cmd_vel',
                      lambda m: got.append((round(time.time()-t0, 3), m.linear.x, m.angular.z)), 50)
cli = n.create_client(Trigger, '/chassis_stop')
ex = MultiThreadedExecutor(); ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()

if not cli.wait_for_service(timeout_sec=10.0):
    print("★ 服务 /chassis_stop 不存在 —— 节点没起来?"); raise SystemExit(1)

print("调用 /chassis_stop ...")
t1 = time.time()
fut = cli.call_async(Trigger.Request())
while not fut.done() and time.time() - t1 < 20:
    time.sleep(0.05)
print("success =", fut.result().success)
print("message =", fut.result().message)
time.sleep(1.0)
ex.shutdown()

nz = [g for g in got if abs(g[1]) > 1e-9 or abs(g[2]) > 1e-9]
print("期间收到 %d 帧 /cmd_vel, 其中非零 %d 帧" % (len(got), len(nz)))
if got:
    print("时间跨度 %.2fs -> 平均约 %.1f Hz" % (got[-1][0]-got[0][0],
          len(got)/max(0.001, got[-1][0]-got[0][0])))
print("前 3 帧:", got[:3])
rclpy.shutdown()
PY

echo
echo "===== 3. 收尾: 停掉测试节点 ====="
pkill -f '/tmp/cvg\.py' 2>/dev/null
sleep 1
ps -eo pid,cmd | grep '[c]vg\.py' | head -2
echo "(空 = 已停)"
