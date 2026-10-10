#!/usr/bin/env python3
"""可靠停车: 把"停"做对, 而不是往 /cmd_vel 发一帧 0。

## 为什么需要它

`ros2 topic pub --once /cmd_vel ... "{x: 0}"` 只发**一帧** 0。而 /cmd_vel 是
**一个订阅者 + N 个发布者、没有仲裁**:

    controller_server  --20Hz 非零--┐
    你的 1 帧 0        -------------┴--► wheeltec_robot --> STM32

ROS 2 里谁最后到谁算数, 所以那 1 帧在 ~50ms 内就被盖掉了 —— 车不会停。
2026-10-10 实测 /cmd_vel 上有 5 个发布者(controller_server / behavior_server ×3
/ stop_all_services), 而且没装 twist_mux。

★ 一个反推: 只要 0 之后**再没有任何速度帧**, STM32 自己的 800ms 运动看门狗也会
  停车。所以"完全不停"必然意味着还有别人在持续发非零。

## 它做什么

收到 `/chassis_stop`(std_srvs/Trigger) 后, 按**正确顺序**来:

  1. **先让源头闭嘴** —— 取消所有 /navigate_to_pose goal。这一步才是关键:
     源头不停, 发多少帧 0 都是白搭。
  2. 再以 20Hz 持续发 0 共 `hold_s`(默认 1.0s) —— 让停车**立刻**生效,
     而不是等 STM32 那 800ms 看门狗。
  3. **量一下到底停没停**: 看 /odom 的车体速度。这一条最重要 ——
     不用猜, 直接把"停没停"变成可验证的结论。

  若量到还在动, 就**不再跟它抢**了(和 Nav2 对着 20Hz 发只会让车一抖一抖),
  直接返回失败 + 实测速度, 并提示改用 reset_ros.sh / 失活 Nav2。

## 用法

    python3 tools/cmd_vel_guard.py &          # 或 systemd
    ros2 service call /chassis_stop std_srvs/srv/Trigger

## 和"正确架构"的关系

这只是**不需要改同事 Nav2 配置**的兜底。长期正确做法是加优先级仲裁:
Nav2 发 /cmd_vel_nav, 急停/遥操走高优先级, mux 输出 /cmd_vel
(即 twist_mux; 本机没装)。见 docs/nav2-reference.md §5.1。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal

# 要取消的 action。★ 只取消 /navigate_to_pose 不够: 恢复行为(spin/backup)是
# behavior_server 自己在发的, BT 被取消之后它可能还在把速度发出去。
CANCEL_ACTIONS = ("/navigate_to_pose", "/navigate_through_poses",
                  "/follow_path", "/spin", "/backup")

CTRL_HZ = 20.0
STOPPED_VX = 0.02        # m/s   小于它就算停了
STOPPED_WZ = 0.05        # rad/s


class CmdVelGuard(Node):
    def __init__(self, hold_s=1.0, max_s=1.0, odom_topic="/odom",
                 cmd_topic="/cmd_vel", actions=CANCEL_ACTIONS):
        super().__init__("cmd_vel_guard")
        self.hold_s = float(hold_s)
        self.max_s = float(max_s)

        # ★★★ 回调组是这里最容易踩的坑, 实测踩过:
        #   rclpy 里**不指定回调组 = 用节点的默认回调组, 而默认是
        #   MutuallyExclusive** —— 同一个回调组里的回调**永远不会并发**,
        #   哪怕你用的是 MultiThreadedExecutor。
        #   后果: /chassis_stop 的回调一阻塞(要发 1 秒的 0), /odom 回调就
        #   完全收不到东西("本次期间收到 0 帧"), 而且 cancel_goal 的响应也
        #   永远处理不了 -> 取消目标报 none。停车判定因此全是瞎的。
        #   把它们放进 ReentrantCallbackGroup, 多线程才有意义。
        self._cbg = ReentrantCallbackGroup()

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 20,
                                 callback_group=self._cbg)
        self.srv = self.create_service(Trigger, "/chassis_stop", self._on_stop,
                                       callback_group=self._cbg)

        self._vx = None
        self._wz = None
        self._odom_t = 0.0
        self._odom_n = 0                     # 收了多少帧 odom(排查用)

        # ★ 不用 rclpy 的 ActionClient 来取消: Humble 的 ActionClient **没有**
        #   公开的 cancel-all 方法(实测 dir(ActionClient) 里只有私有的
        #   _cancel_goal / _cancel_goal_async), 而它只能取消"自己发出的"目标。
        #   我们不是发目标的人, 所以直接调 action 底层的 cancel_goal 服务 ——
        #   这是 ROS 2 官方给"非发起方要求取消"留的通道。
        #   goal_info 全 0 = 取消**所有**目标。
        self._cancel_cli = {}
        for a in actions:
            name = a + "/_action/cancel_goal"
            try:
                self._cancel_cli[a] = self.create_client(
                    CancelGoal, name, callback_group=self._cbg)
            except Exception as e:
                self.get_logger().warn("建不了 %s 的取消客户端: %s" % (name, e))

        self.get_logger().info(
            "chassis_stop 就绪: hold=%.1fs max=%.1fs, 取消 %d 个 action: %s"
            % (self.hold_s, self.max_s, len(self._cancel_cli),
               " ".join(self._cancel_cli.keys())))

    # ------------------------------------------------------------------ odom
    def _cb_odom(self, msg):
        self._vx = msg.twist.twist.linear.x
        self._wz = msg.twist.twist.angular.z
        self._odom_t = time.time()
        self._odom_n += 1

    def _measured(self):
        """返回 (vx, wz)。拿不到新鲜 odom 就返回 None。"""
        if self._vx is None or (time.time() - self._odom_t) > 1.0:
            return None
        return (self._vx, self._wz)

    def _is_stopped(self):
        m = self._measured()
        if m is None:
            return None                      # 没有观测 -> 不敢下结论
        return abs(m[0]) < STOPPED_VX and abs(m[1]) < STOPPED_WZ

    # ------------------------------------------------------------- 关键第 1 步
    def _cancel_goals(self, timeout=1.5):
        """取消所有导航目标 —— 让 controller_server 停止发非零。

        goal_info 全 0 的语义(action_msgs/srv/CancelGoal 官方定义):
          goal_id 全 0 且 stamp 为 0  ->  取消**所有**目标
        返回 "ok" / "none"(没有服务端, 说明 Nav2 没跑, 那也没人在发) /
        以及其它错误串。"""
        done = []
        for a, cli in self._cancel_cli.items():
            if not cli.service_is_ready():
                if not cli.wait_for_service(timeout_sec=0.2):
                    continue                 # 这个 action 没起, 跳过
            req = CancelGoal.Request()        # 默认就是全 0 = 取消全部
            req.goal_info.goal_id.uuid = [0] * 16
            req.goal_info.stamp.sec = 0
            req.goal_info.stamp.nanosec = 0
            fut = cli.call_async(req)
            t0 = time.time()
            while not fut.done() and (time.time() - t0) < timeout:
                time.sleep(0.02)
            if fut.done() and fut.result() is not None:
                done.append("%s(rc=%d)" % (a, fut.result().return_code))
        if not done:
            return "none"
        return "ok:" + ",".join(done)


    # ------------------------------------------------------------- 关键第 2 步
    def _burst(self, dur_s):
        """按 CTRL_HZ 持续发 0。返回实际发了多少帧。

        ★ 这里**不能**调 rclpy.spin_once: 我们正跑在回调里, 重入 executor 要么
          死锁要么把 /odom 卡住, 后面"到底停没停"就量不准了。所以用
          MultiThreadedExecutor(见 main), 让 /odom 在别的线程上照常更新。

        ★ 也不能只发一帧 —— 一帧 0 会被别人的 20Hz 非零在 50ms 内盖掉。"""
        period = 1.0 / CTRL_HZ
        n = 0
        t0 = time.time()
        while (time.time() - t0) < dur_s:
            self.pub.publish(Twist())
            n += 1
            time.sleep(period)
        return n

    def _on_stop(self, _req, res):
        n0 = self._odom_n
        before = self._measured()
        cancel = self._cancel_goals()

        frames = self._burst(self.hold_s)
        time.sleep(0.15)                     # 等 /odom 追上刚才的命令
        stopped = self._is_stopped()

        # 还在动就再给一次机会(但不无限跟 Nav2 抢)
        if stopped is False and self.max_s > self.hold_s:
            frames += self._burst(self.max_s - self.hold_s)
            time.sleep(0.15)
            stopped = self._is_stopped()

        after = self._measured()
        got_odom = self._odom_n - n0

        def fmt(m):
            return "None" if m is None else "vx=%.3f wz=%.3f" % (m[0], m[1])

        res.success = bool(stopped)
        if stopped is None:
            res.message = ("发了 %d 帧 0, 但**拿不到 /odom**(本次期间收到 %d 帧, "
                           "累计 %d 帧), 无法确认是否停住。取消目标=%s"
                           % (frames, got_odom, self._odom_n, cancel))
        elif stopped:
            res.message = ("已停车 (前 %s -> 后 %s, 取消目标=%s, %d 帧 0, "
                           "期间 /odom %d 帧)"
                           % (fmt(before), fmt(after), cancel, frames, got_odom))
        else:
            res.message = (
                "★ 没能停住 (前 %s -> 后 %s)。取消目标=%s, 期间 /odom %d 帧。"
                "有别的节点在持续发非零, 本节点**已停止跟它抢**"
                "(继续对着发只会让车一抖一抖)。请改用 "
                "scripts/reset_ros.sh, 或把 Nav2 失活; 根治办法是加优先级仲裁"
                "(见 docs/nav2-reference.md §5.1)"
                % (fmt(before), fmt(after), cancel, got_odom))
        self.get_logger().warn(res.message)
        return res


def main(argv=None):
    ap = argparse.ArgumentParser(description="可靠停车服务 /chassis_stop")
    ap.add_argument("--hold", type=float, default=1.0, help="发 0 的时长(秒), 默认 1.0")
    ap.add_argument("--max", type=float, default=1.0, help="最多跟它抢多久(秒), 默认 1.0")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    args = ap.parse_args(argv)

    rclpy.init()
    node = CmdVelGuard(args.hold, args.max, args.odom, args.cmd_topic)
    # ★ 必须多线程: 停车流程整个跑在服务回调里(要阻塞上百毫秒),
    #   单线程 executor 会把 /odom 回调一起卡住 -> "到底停没停"量出来是假的。
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前把最后一条命令坐实成停车 —— 死手兜底
        try:
            for _ in range(5):
                node.pub.publish(Twist())
                time.sleep(0.05)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
