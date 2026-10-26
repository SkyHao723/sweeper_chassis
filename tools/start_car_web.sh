#!/bin/bash
# 在 RK3576 主机上启动网页遥控服务 (car_web)
# 用法: bash /tmp/start_car_web.sh
set -u

echo "== 1. 停掉旧的 car_web =="
pkill -f 'car_web.py' 2>/dev/null
sleep 1
pgrep -af 'car_web.py' && echo "  还有残留进程!" || echo "  已清空"

echo "== 2. 检查串口 =="
if [ ! -e /dev/wheeltec_controller ]; then
    echo "  !! 找不到 /dev/wheeltec_controller"
    ls -l /dev/ttyUSB* 2>&1
    lsusb | grep -i 1a86 || echo "  !! lsusb 里也没有 1a86:7523 (USB 线/供电问题)"
    exit 1
fi
ls -l /dev/wheeltec_controller

echo "== 3. 启动 =="
cd /home/smart/car_web || { echo "  !! /home/smart/car_web 不存在"; exit 1; }
setsid nohup python3 -u car_web.py \
    --serial /dev/wheeltec_controller --baud 115200 \
    --port 8000 --host 0.0.0.0 --tx-hz 30 \
    >> /tmp/car_web.log 2>&1 < /dev/null &
sleep 3

echo "== 4. 结果 =="
PID=$(pgrep -f 'car_web.py' | head -1)
if [ -z "$PID" ]; then
    echo "  !! 没起来, 日志尾部:"
    tail -30 /tmp/car_web.log
    exit 1
fi
echo "  PID=$PID"
ss -ltn 2>/dev/null | grep -q ':8000' && echo "  端口 8000 已监听" || echo "  !! 端口 8000 未监听"
echo "  --- 日志尾部 ---"
tail -15 /tmp/car_web.log
