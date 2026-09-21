#!/bin/bash
# 原地转性能对 wz 扫描 —— 找出低速环的坏区间, 给外环限幅定范围
# 用法: bash ~/chassis_tools/turn_sweep.sh
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/smart_ws/install/local_setup.bash" 2>/dev/null

# wz -> 单轮 RPM: rpm = wz * 0.930/2 / 0.645 * 60 = wz * 43.26
for wz in 0.30 0.50 0.70 1.00; do
    rpm=$(python3 -c "print('%+.1f' % ($wz * 43.26))")
    echo
    echo "##########################  wz=$wz  (单轮目标约 $rpm RPM)  ##########################"
    python3 "$HOME/chassis_tools/turn_truth.py" "$wz" 4 2>&1 \
        | grep -viE 'warning|deprecat' \
        | sed -n '/^====/,/^怎么读/p'
    sleep 3
done
echo
echo "扫描结束。"
