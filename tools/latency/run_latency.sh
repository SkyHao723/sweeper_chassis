#!/bin/bash
# 测「/cmd_vel 发布 → 轮子实际转动」链路延迟 —— 在主机 RK3576 上一键跑
#
# 用法（主机上）:
#   bash /home/smart/latency/run_latency.sh            # 默认 20 个 trial
#   TRIALS=30 SPEED=0.20 bash .../run_latency.sh       # 可调
#   CHECK=1 bash .../run_latency.sh                    # 只做方向自检（车只动 5 cm）
#
# 前提:
#   - 车要落地、前后留 1.5 m 净空（会真的动，幅度很小）
#   - /tmp/probe_latency.py 已上传
# 产出:
#   /tmp/latency.json      发布时刻 + 全部 /odom 样本
#   /tmp/strace_node.log   节点每次串口 write 的墙钟时刻（含 11 B 命令帧）
#   /tmp/probe.log         运行日志
set -u

TRIALS="${TRIALS:-20}"
SPEED="${SPEED:-0.20}"
CHECK="${CHECK:-0}"

export ROS_DOMAIN_ID=7
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/smart/wheeltec_ws/install/setup.bash

NODE=/home/smart/wheeltec_ws/install/turn_on_wheeltec_robot/lib/turn_on_wheeltec_robot/wheeltec_robot_node
PORT=/dev/wheeltec_controller
[ -e "$PORT" ] || PORT=/dev/ttyUSB0

echo "=== 环境检查 ==="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  NODE=$NODE"
echo "串口: $PORT"
if [ ! -e "$PORT" ]; then
  echo "!! 串口不存在 (lsusb 如下) —— USB 线/供电问题"
  lsusb
  exit 1
fi

echo "=== 停 car_web 让出串口 ==="
pkill -f car_web.py 2>/dev/null
sleep 1
echo "占用检查: $(fuser -v "$PORT" 2>&1 | head -3 | tr '\n' ' ')"

echo "=== 起厂商节点 (strace 抓串口 write 时刻) ==="
rm -f /tmp/strace_node.log /tmp/node.log /tmp/latency.json
setsid nohup strace -f -tt -T -e trace=write,writev -o /tmp/strace_node.log \
  "$NODE" --ros-args -p usart_port_name:="$PORT" -p serial_baud_rate:=115200 \
  > /tmp/node.log 2>&1 < /dev/null &
sleep 6

if ! pgrep -f wheeltec_robot_node >/dev/null; then
  echo "!! 节点没起来, /tmp/node.log 尾部:"
  tail -40 /tmp/node.log
  exit 1
fi
echo "节点 PID: $(pgrep -f wheeltec_robot_node | tr '\n' ' ')"
tail -6 /tmp/node.log

echo "=== 跑 probe ==="
PROBE_ARGS="--trials $TRIALS --speed $SPEED --out /tmp/latency.json"
if [ "$CHECK" = "1" ]; then
  PROBE_ARGS="$PROBE_ARGS --check-only"
fi
python3 /tmp/probe_latency.py $PROBE_ARGS 2>&1 | tee /tmp/probe.log

echo "=== 收尾: 停节点 ==="
pkill -TERM -f wheeltec_robot_node 2>/dev/null
sleep 1
pkill -KILL -f wheeltec_robot_node 2>/dev/null
sleep 0.5

echo "=== 产出 ==="
ls -l /tmp/latency.json /tmp/strace_node.log /tmp/node.log 2>&1
echo "strace 里 write 行数: $(grep -c 'write' /tmp/strace_node.log 2>/dev/null || echo 0)"
echo "=== 完成 ==="
