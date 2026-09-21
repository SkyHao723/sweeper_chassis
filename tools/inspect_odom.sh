#!/bin/bash
# 查 /odom 到底是谁发的、字段内容对不对
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/smart_ws/install/local_setup.bash" 2>/dev/null

echo "=== /odom 发布者 ==="
ros2 topic info /odom --verbose 2>/dev/null | grep -E 'Type|Publisher count|Subscription count|node namespace|node name|topic type'

echo
echo "=== /odom 频率 ==="
timeout 6 ros2 topic hz /odom 2>/dev/null | head -3

echo
echo "=== /odom 完整内容 (一帧) ==="
timeout 10 ros2 topic echo /odom --once 2>/dev/null

echo
echo "=== 有没有别的节点也在发 odom (看 nav_msgs) ==="
ros2 node list 2>/dev/null
