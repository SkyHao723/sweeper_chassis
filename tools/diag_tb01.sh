#!/bin/bash
echo "=== 1. 现在在跑吗 ==="
pgrep -af 'tb01_chassis|scrubber_damiao' | head -3 || echo "  ★ 没在跑"
echo
echo "=== 2. 串口状态 ==="
ls -l /dev/wheeltec_controller /dev/ttyUSB0 2>/dev/null || echo "  ★ 没有串口设备"
fuser -v /dev/ttyUSB0 2>&1 | head -3
echo
echo "=== 3. 源码位置 ==="
ls -d /home/smart/tb01_ws/src/*/ 2>/dev/null | head -6
echo "--- tb01_chassis 包内容 ---"
find /home/smart/tb01_ws/src -maxdepth 3 -path '*tb01_chassis*' \( -name '*.py' -o -name '*.cpp' -o -name '*.yaml' \) 2>/dev/null | head -20
echo
echo "=== 4. 配置 (端口/波特率/协议) ==="
find /home/smart/tb01_ws -name 'tb01_chassis.yaml' 2>/dev/null | head -2 | while read f; do
  echo "--- $f ---"
  head -40 "$f"
done
echo
echo "=== 5. 最近日志里的错误 ==="
LOGDIR=$(ls -dt /home/smart/.ros/log/*/ 2>/dev/null | head -1)
if [ -n "$LOGDIR" ]; then
  echo "  最新日志目录: $LOGDIR"
  grep -rihE 'error|warn|fail|Traceback|Exception|exit' "$LOGDIR" 2>/dev/null | tail -12
else
  echo "  没有 .ros/log"
fi
echo
echo "=== 6. tb01_web 还在跑吗 ==="
pgrep -af ros_drive_server | head -2 || echo "  没在跑"
curl -s -o /dev/null -w '  8080 HTTP %{http_code}\n' --max-time 2 http://127.0.0.1:8080/ 2>/dev/null || echo "  8080 不通"
