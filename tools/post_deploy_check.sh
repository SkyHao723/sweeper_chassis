#!/bin/bash
# 部署后总检查: 看门狗 / 栈唯一性 / odom / 串口
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/smart_ws/install/local_setup.bash" 2>/dev/null

echo "=== 看门狗 ==="
echo "systemd : $(systemctl is-active chassis-watchdog)"
echo "KillMode: $(systemctl show chassis-watchdog -p KillMode --value)"
echo "日志末尾:"
tail -4 /tmp/chassis_watchdog.log | sed 's/^/    /'

echo
echo "=== 栈的唯一性 (重复启动会抢串口) ==="
echo "ros2 launch 进程数 : $(pgrep -c -f 'ros2 launch turn_on_wheeltec_robot')"
echo "底盘节点进程数     : $(pgrep -c -f '/wheeltec_robot_node')"
echo "ekf_node 进程数    : $(pgrep -c -f 'ekf_node')"
echo "串口占用:"
if command -v fuser > /dev/null 2>&1; then
    fuser -v /dev/wheeltec_controller 2>&1 | sed 's/^/    /' || echo "    (fuser 没输出)"
fi

echo
echo "=== /odom 实测 ==="
timeout 12 python3 - <<'PYEOF' 2>&1 | tail -4
import rclpy, time
from nav_msgs.msg import Odometry
rclpy.init()
n = rclpy.create_node('post_deploy_chk')
c = [0]; v = [0.0]
def cb(m):
    c[0] += 1
    v[0] = m.twist.twist.linear.x
n.create_subscription(Odometry, '/odom', cb, 10)
t = time.time()
while time.time() - t < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
print('  /odom 5 秒内 %d 条 (约 %.1f Hz), 当前 vx=%.4f' % (c[0], c[0] / 5.0, v[0]))
n.destroy_node(); rclpy.shutdown()
PYEOF
