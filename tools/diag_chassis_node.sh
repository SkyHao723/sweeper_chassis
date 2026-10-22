#!/bin/bash
# 诊断 tb01_chassis: 它到底有没有从串口读到/发出数据
echo "=== 进程 ==="
PID=21462
if [ -d /proc/$PID ]; then
  echo "  PID $PID 活着"
  tr '\0' ' ' < /proc/$PID/cmdline; echo
  echo "--- 运行时长 ---"
  awk '{printf "  %.1f 分钟\n", (systime()-$22)/60}' /proc/$PID/stat 2>/dev/null || echo "  (读不到)"
else
  echo "  ★ PID $PID 已经不存在了"
  echo "--- 找现在占着串口的 ---"
  fuser -v /dev/ttyUSB0 2>&1 | head -3
  echo "--- 所有 chassis 相关进程 ---"
  pgrep -af 'tb01|chassis' | head -6
  exit 0
fi

echo
echo "=== 它读/写了多少字节 (/proc/PID/io) ==="
cat /proc/$PID/io 2>/dev/null

echo
echo "=== 它打开的串口 fd ==="
ls -l /proc/$PID/fd 2>/dev/null | grep -E 'ttyUSB|wheeltec' | head -3

echo
echo "=== 节点日志 (最后 20 行) ==="
LOGDIR=$(ls -dt /home/smart/.ros/log/*/ 2>/dev/null | head -1)
echo "  日志目录: $LOGDIR"
if [ -n "$LOGDIR" ]; then
  ls -t "$LOGDIR" 2>/dev/null | head -6
  echo "--- 找 error/warn ---"
  grep -rihE 'error|warn|fail|timeout|exception|Traceback' "$LOGDIR" 2>/dev/null | tail -10
fi

echo
echo "=== 串口是否还有人读? 用 strace 抓 2 秒 read 情况 ==="
if command -v strace >/dev/null 2>&1; then
  timeout 3 strace -f -p $PID -e trace=read,write -c 2>&1 | tail -12
else
  echo "  (没装 strace)"
fi
