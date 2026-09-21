#!/bin/bash
# 校验 ekf.yaml 改完还能被正常解析, 关键参数没被动过
python3 - <<'PYEOF'
import yaml
p = '/home/smart/smart_ws/src/turn_on_wheeltec_robot/config/ekf.yaml'
d = yaml.safe_load(open(p, encoding='utf-8'))
r = d['ekf_filter_node']['ros__parameters']
print('YAML 解析 OK')
print('  frequency                      =', r['frequency'])
print('  two_d_mode                     =', r['two_d_mode'])
print('  world_frame                    =', r['world_frame'])
print('  base_link_frame                =', r['base_link_frame'])
print('  imu0_twist_rejection_threshold =', r['imu0_twist_rejection_threshold'])
print('  odom0_config                   =', r['odom0_config'])
print('  imu0_config                    =', r['imu0_config'])
PYEOF

echo "--- 注释里那两处更正是否写进去了 ---"
grep -c "协方差不是固件设的" /home/smart/smart_ws/src/turn_on_wheeltec_robot/config/ekf.yaml
echo "--- 备份 ---"
ls -la /home/smart/smart_ws/src/turn_on_wheeltec_robot/config/ekf.yaml.orig 2>/dev/null | awk '{print $NF, $5"字节"}'
