# 关节状态监控示例：本示例演示如何使用SDK订阅遥测关节位置。
# 用 set_feedback_callback() 订阅遥测，每 500 ms 用 last_feedback() 打印实时快照，
# 直到可选时长结束。
# [seconds] 为遥测时长，0（或负数）表示一直运行到进程被杀 —— 适合在机器人旁边工作时
# 挂一个长期监控。
#
# 样本停止到达（如网络中断）时，缓存的最后一帧会带着不断增长的 age 与 STALE 标记继续
# 打印，并触发一次 [WARN] 警告。
#
# 用法：get_joint.py [seconds] [<ip>:<port>] [namespace]
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：get_joint.py <seconds> <ip>:<port> ""
# 用法与缺省值也可以直接问示例：get_joint.py --help
#
# 句柄不显式关闭，进程退出时随进程回收；需要提前释放时可正常 close()/with（v1.0.1 起，
# 此前版本的例外说明见 README）。

import sys
import threading
import time

import example_common as common
import shidou

PRINT_INTERVAL_S = 0.5
# 遥测是连续流；沉默超过该阈值即触发 stale 回调。
STALE_THRESHOLD_MS = 1000


def print_snapshot(feedback, seq, age_ms, fsm, stale):
    print("fsm={} seq={} age={:.1f} ms{} motors={}".format(
        fsm, seq, age_ms, " [STALE]" if stale else "", len(feedback.motor_ids)))
    for index, motor_id in enumerate(feedback.motor_ids):
        position = feedback.position[index] if index < len(feedback.position) else 0.0
        velocity = feedback.velocity[index] if index < len(feedback.velocity) else 0.0
        effort = feedback.effort[index] if index < len(feedback.effort) else 0.0
        temperature = feedback.temperature[index] if index < len(feedback.temperature) else 0.0
        fault = feedback.fault_code[index] if index < len(feedback.fault_code) else 0
        online = feedback.online[index] if index < len(feedback.online) else 0
        print("  motor {:2d}  pos={:8.4f}  vel={:7.4f}  eff={:6.3f}  temp={:5.1f}  "
              "fault=0x{:04x}  online={}".format(
                  motor_id, position, velocity, effort, temperature, fault, online))


def monitor_joints(robot, cfg, verdicts, seconds):
    """订阅与打印：每 500 ms 打一帧快照，直到 seconds 用完（<= 0 表示一直运行到进程被杀）。"""
    print("monitoring {} for {:.0f} s (0 = until killed)...".format(cfg.robot_address, seconds))

    # 精确接收计数：缓存只保留最新样本，轮询缓存会少算；回调每个样本触发一次。
    # 回调在 zenoh 会话线程上跑，用锁保护计数。
    received = [0]
    received_lock = threading.Lock()

    def on_feedback(_feedback):
        with received_lock:
            received[0] += 1

    robot.set_feedback_callback(on_feedback)
    robot.set_stale_callback(
        lambda ms: verdicts.warn("no telemetry for {:.0f} ms (link or robot node down?)".format(ms)),
        threshold_ms=STALE_THRESHOLD_MS)

    have_feedback = False
    age_min = 0.0
    age_max = 0.0
    start = time.monotonic()
    while seconds <= 0.0 or time.monotonic() - start < seconds:
        time.sleep(PRINT_INTERVAL_S)

        feedback = robot.last_feedback()
        if feedback is None:
            print("no feedback yet")
            continue
        age = robot.feedback_age_ms
        if not have_feedback:
            have_feedback = True
            age_min = age_max = age
        else:
            age_min = min(age_min, age)
            age_max = max(age_max, age)
        print_snapshot(feedback, robot.feedback_seq, age, robot.fsm_state,
                       age > STALE_THRESHOLD_MS)

    elapsed = time.monotonic() - start
    shidou.shutdown()
    samples = received[0]
    if samples == 0:
        verdicts.fail("no feedback received")
        return
    print("received {} samples in {:.1f} s (~{:.1f} Hz), age {:.1f}..{:.1f} ms".format(
        samples, elapsed, samples / elapsed if elapsed > 0.0 else 0.0, age_min, age_max))
    verdicts.pass_("get_joint completed")


def main():
    # 监控进程在 stdout 被重定向到文件时也必须即时打印（在解析参数之前设置）。
    sys.stdout.reconfigure(line_buffering=True)

    # 运行时长（秒）；<= 0 表示一直运行到进程被杀。缺省值写在参数声明里，与显式取值
    # 走同一条解析路径；非数字必须是非法调用——不能静默取 0，那等于把有界监控变成
    # 一直运行到进程被杀。解析结果由这个闭包接走（与 C++ 侧解析闭包写
    # `double seconds` 同形），body 再从闭包取。
    seconds = 0.0

    def parse_seconds(text):
        nonlocal seconds
        seconds = common.parse_double("seconds", "a number", text)

    seconds_arg = common.PositionalArg(
        "seconds", "10",
        "telemetry duration in seconds; 0 or negative runs until the process is killed",
        parse_seconds)

    def body(robot, cfg, verdicts):
        monitor_joints(robot, cfg, verdicts, seconds)

    return common.run([seconds_arg], body)


if __name__ == "__main__":
    sys.exit(main())
