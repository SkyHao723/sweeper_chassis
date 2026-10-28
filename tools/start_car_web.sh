#!/bin/bash
# 在 RK3576 主机上启动网页遥控服务 (car_web)
#
# 用法（脚本已装在永久位置，重启不丢）:
#     bash ~/car_web/start.sh
#
# 注意: 不要放在 /tmp 里用 —— /tmp 在主机重启后会被清空。
set -u

echo "== 1. 停掉旧的 car_web =="
pkill -f 'car_web.py' 2>/dev/null
sleep 1
if pgrep -f 'car_web.py' >/dev/null; then
    echo "  还有残留进程:"
    pgrep -af 'car_web.py'
else
    echo "  已清空"
fi

echo "== 2. 检查串口 =="
# 优先用 udev 软链; 软链没生成(udev 规则没生效/刚插上)时退回 /dev/ttyUSB0
PORT=/dev/wheeltec_controller
if [ ! -e "$PORT" ]; then
    if [ -e /dev/ttyUSB0 ]; then
        echo "  ! 没有 $PORT, 但 /dev/ttyUSB0 在, 直接用它"
        PORT=/dev/ttyUSB0
    else
        echo "  !! 找不到 /dev/wheeltec_controller, 也没有 /dev/ttyUSB0"
        ls -l /dev/ttyUSB* 2>&1
        echo "  --- lsusb ---"
        lsusb
        if lsusb | grep -qi '1a86:7523'; then
            echo "  !! CH340 在 lsusb 里但没有 ttyUSB 节点 -> 多半是驱动没加载, 试: sudo modprobe ch341"
        else
            echo "  !! lsusb 里没有 1a86:7523 (CH340) -> 物理层问题:"
            echo "     USB 线没插 / 松了 / 是充电线 / STM32 没上电"
        fi
        exit 1
    fi
fi
ls -l "$PORT"

echo "== 3. 启动 =="
cd /home/smart/car_web || { echo "  !! /home/smart/car_web 不存在"; exit 1; }
setsid nohup python3 -u car_web.py \
    --serial "$PORT" --baud 115200 \
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
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "  打开: http://${IP:-<主机IP>}:8000"
echo "  --- 日志尾部 ---"
tail -15 /tmp/car_web.log
