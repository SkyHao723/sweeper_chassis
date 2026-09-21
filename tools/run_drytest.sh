#!/bin/bash
# 起一个 dry-run 实例并跑安全测试
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/smart_ws/install/local_setup.bash" 2>/dev/null
cd "$HOME/chassis_tools" || exit 1

# 用 PID 收尾, 别用 pkill -f —— 模式串会出现在本脚本自己的命令行里,
# 实测会把执行这条命令的 shell 一起杀掉 (栽过好几次了)。
OLD=$(pgrep -f 'chassis_web.py' | tr '\n' ' ')
for p in $OLD; do
    if tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q -- '--dry-run'; then
        kill -9 "$p" 2>/dev/null && echo "  收掉旧的 dry-run 实例 pid=$p"
    fi
done

# ★ 必须**等到端口真的空出来**再起新的。以前只 sleep 1 就走了, 结果新实例
#   "Address already in use" 当场崩掉, 而测试连上了那个没死透的旧实例 ——
#   报了一堆假 FAIL, 查了半天才发现测试连的根本不是新代码。
for i in $(seq 1 20); do
    ss -ltn 2>/dev/null | grep -q ':8090' || break
    sleep 0.5
done
if ss -ltn 2>/dev/null | grep -q ':8090'; then
    echo "!! 8090 还是被占着, 放弃 (占用者:)"
    ss -ltnp 2>/dev/null | grep ':8090'
    exit 1
fi
echo "  8090 已空出来"

nohup env PYTHONUNBUFFERED=1 python3 chassis_web.py -p 8090 --dry-run \
    --vx-max 0.35 --wz-max 1.2 > /tmp/dryrun.log 2>&1 &
DRY=$!
sleep 4
echo "dry-run 实例 pid=$DRY"
tail -8 /tmp/dryrun.log
if ! kill -0 "$DRY" 2>/dev/null; then
    echo "!! dry-run 实例已经死了, 上面就是原因"
    exit 1
fi
echo
# ★ 用脚本自己所在的目录, 别写死 /tmp —— 这台机器的 /tmp 会被清掉, 实测因此
#   白跑一轮("can't open file /tmp/test_webctl.py")。两个脚本现在都在
#   ~/chassis_tools/ 下, 就按相对自己的位置找。
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 -u "$HERE/test_webctl.py"
rc=$?
echo
echo "测试退出码: $rc"
kill "$DRY" 2>/dev/null
echo "(已关掉 dry-run 实例)"
exit $rc
