#!/bin/bash
# "芯片 ID" = USB 的 VID:PID。这个脚本一次看清:
#   - 车上插了几个 USB 串口芯片、分别是什么
#   - 底盘那个口能不能被唯一标识(CH340 没有序列号, 只能按物理口)
#   - 有几个 udev 规则在抢同一个软链接
#   - 最近有没有掉线/重新枚举
# 只读, 不改任何东西。
echo "===== 1. 现在插着的 USB 串口芯片 (VID:PID) ====="
lsusb 2>/dev/null
echo
echo "-- 只看串口类(1a86=沁恒/WCH, 10c4=Silicon Labs, 0403=FTDI, 067b=Prolific) --"
lsusb 2>/dev/null | grep -iE '1a86|10c4|0403|067b|ch34|cp210|ft232|pl2303'

echo
echo "===== 2. 有几个 CH340(1a86:7523)? 有几个串口设备? ====="
echo -n "CH340 个数: "; lsusb -d 1a86:7523 2>/dev/null | wc -l
echo "-- /dev/ttyUSB* /dev/ttyACM* --"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1

echo
echo "===== 3. 底盘那个口是什么, 能不能唯一标识 ====="
ls -l /dev/wheeltec_controller 2>&1
D=$(readlink -f /dev/wheeltec_controller 2>/dev/null)
echo "解析到: $D"
if [ -n "$D" ] && [ -e "$D" ]; then
  echo "-- 它的 USB 属性 (关键: ID_SERIAL 有没有唯一性, KERNELS 是哪个物理口) --"
  udevadm info -q property -n "$D" 2>/dev/null | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_SERIAL_SHORT|ID_PATH|DEVPATH|ID_USB_INTERFACE_NUM)='
  echo "-- 从父设备取(VID:PID 真正所在的那一层) --"
  udevadm info -q property -a -n "$D" 2>/dev/null | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_PATH' | head -8
  echo "-- 物理口路径(KERNELS 用这个) --"
  udevadm info -a -n "$D" 2>/dev/null | grep -m2 'KERNELS==' 
fi

echo
echo "===== 4. 装了哪些 udev 规则 ====="
grep -rn 'wheeltec\|1a86\|ttyUSB' /etc/udev/rules.d/ 2>/dev/null | head -20

echo
echo "-- ★ 有几条规则在抢同一个软链接(多于 1 条就是隐患: 谁最后命中谁赢) --"
for L in wheeltec_controller wheeltec_lidar wheeltec_IMU; do
  n=$(grep -l "SYMLINK+=\"$L\"" /etc/udev/rules.d/*.rules 2>/dev/null | wc -l)
  printf "  %-22s %s 条\n" "$L" "$n"
done

echo
echo "===== 5. 这台内核到底能不能驱动 CH343(1a86:55d4)? ====="
echo "★ 换 CH343 之前必须先过这一关: 内核不认 -> 插上根本不出 tty, 直接不能用。"
if modinfo ch341 2>/dev/null | grep -qi '55d4'; then
  echo "  ch341 认 CH343 -> 插上应该能出 /dev/ttyCH343USB* 或 ttyUSB*"
else
  echo "  ★ ch341.ko 的 alias 里**没有 55d4** -> 内核自带驱动不认 CH343"
fi
echo -n "  是否装了 WCH 的 out-of-tree 驱动: "
if find /lib/modules -iname '*ch343*' 2>/dev/null | grep -q .; then
  find /lib/modules -iname '*ch343*' 2>/dev/null | head -3
else
  echo "没找到(没有 ch343.ko, 也没有 dkms)"
fi
echo "  当前存在的 tty: $(ls /dev/ttyUSB* /dev/ttyCH343USB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"


echo
echo "===== 6. 最近有没有掉线/重新枚举 ====="
dmesg 2>/dev/null | grep -iE 'ch341|usb .*disconnect|ttyUSB' | tail -12
