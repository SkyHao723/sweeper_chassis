#!/bin/bash
# 启动底盘节点 (心跳版), 供 192.168.5.7 使用
source /opt/ros/humble/setup.bash
source /home/smart/wheeltec/install/setup.bash
export ROS_DOMAIN_ID=15
exec ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
