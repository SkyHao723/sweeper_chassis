#!/bin/bash
# 现场快诊: "RK3588 控制不了车" 的时候, 一条命令定位到底是哪一层断的。
#
#   bash tools/field_diag.sh            # 只读检查, 不动任何进程
#   bash tools/field_diag.sh --read-serial   # 额外读 STM32 诊断帧(会先停掉 ROS 底盘栈)
#
# 为什么需要它: "控制不了"有至少四层可能, 一层层猜非常费时间。
#   A. 串口/USB 层: CH340 掉了、设备号变了、端口被别人占着
#   B. STM32 层:    根本没在跑(烧录后 BOOT0 忘了拨回来 / 没复位), 或没在发
#   C. 命令链:      RK3588 有没有真的把 /cmd_vel 发出去, 有没有多个发布者在打架
#   D. 执行层:      ★ 最容易被忽略的一层 —— 驱动器没上电/CAN 不通,
#                   固件会**只发刹车**(就绪联锁), 车当然不动, 而遥测/IMU 一切正常。
#                   这时看 /odom、/imu 全是好的, 会让人以为是"软件问题"。
set -u
PORT=/dev/wheeltec_controller
DO_SERIAL=0
[ "${1:-}" = "--read-serial" ] && DO_SERIAL=1

echo "############ A. 串口 / USB 层 ############"
ls -l "$PORT" 2>&1
ls -l /dev/ttyUSB* 2>&1
echo "-- CH340 有没有掉过线(重新枚举) --"
dmesg 2>/dev/null | grep -iE 'ch34|usb.*disconnect' | tail -8
echo "-- 谁占着串口 --"
fuser -v "$PORT" 2>&1 | head -6

echo
echo "############ B. STM32 在不在发 ############"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/smart_ws/install/setup.bash 2>/dev/null
# 遥测帧里带的 IMU 加速度: 有重力(≈9.8) 就说明 STM32 在发、厂商节点在解。
echo "-- /imu/data_raw 加速度 (某个轴应约 ±9.8) --"
timeout 8 ros2 topic echo /imu/data_raw --field linear_acceleration 2>&1 | head -4
echo "-- 电池电压 (STM32 报的, 0.0 = STM32 的电池采样是 0V) --"
timeout 5 ros2 topic echo /PowerVoltage --once 2>&1 | head -2

echo
echo "############ C. 命令链 ############"
echo "-- /cmd_vel 上有几个发布者、是谁 --"
timeout 10 ros2 topic info -v /cmd_vel 2>&1 | grep -E 'Publisher count|Subscription count|Node name|Endpoint type'
echo "-- 相关进程 --"
ps -eo pid,etimes,tty,cmd | grep -E 'teleop|wheeltec_robot_node|topic (pub|echo)' | grep -v grep

if [ "$DO_SERIAL" = "1" ]; then
  echo
  echo "############ D. 执行层: 读 STM32 诊断帧 ############"
  echo "(会先停掉 ROS 底盘栈让出串口, 读完自动重启)"
  touch /tmp/chassis_watchdog.pause
  for pat in 'turn_on_wheeltec_robot.launch.py' 'wheeltec_robot_node' 'ekf_node' \
             'robot_state_publisher' 'joint_state_publisher'; do
    for pid in $(ps -eo pid,cmd | grep -F "$pat" | grep -v grep | awk '{print $1}'); do
      kill "$pid" 2>/dev/null
    done
  done
  sleep 4
  for i in 1 2 3 4 5; do
    fuser -s "$PORT" 2>/dev/null || break
    sleep 2
  done
  python3 /tmp/decode_diag.py -p "$PORT" --seconds 4 2>&1 | tail -8
  echo
  echo "★ 看这一段里的 flags:"
  echo "    '驱动器未就绪(等回码, 车被刹住)' -> 驱动器没上电 / CAN 不通。"
  echo "       固件这时**只发刹车**, 所以 RK3588 发什么都不会动。这是硬件问题。"
  echo "    can_err 很快到 255 -> STM32 发的帧没人应答(总线上没有活着的驱动器)。"
  echo "    '起步助推中' -> 说明烧的是 2026-09 之后的新固件(带 bit6)。"
  echo
  rm -f /tmp/chassis_watchdog.pause
  cd /home/smart
  setsid nohup bash -lc 'source /opt/ros/humble/setup.bash; source ~/smart_ws/install/setup.bash; export ROS_DOMAIN_ID=5; ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py' > /tmp/chassis_relaunch.log 2>&1 &
  sleep 8
  echo "-- 重启后节点 --"
  ps -eo pid,cmd | grep -F 'wheeltec_robot_node' | grep -v grep
fi
