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
#
# 真机上不要用 with、也不要调 close()：销毁带流量的句柄有已知缺陷，进程退出即可
# （原因与例外见 README）。

import argparse
import sys
import threading
import time

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


def main():
    parser = argparse.ArgumentParser(description="订阅关节遥测并周期打印快照。")
    parser.add_argument("seconds", nargs="?", type=float, default=10.0,
                        help="遥测时长（秒）；0 或负数表示一直运行到进程被杀")
    parser.add_argument("address", nargs="?", default="192.168.168.168:7447",
                        help="机器人侧 zenoh 桥的 <ip>:<port>（缺省 192.168.168.168:7447）")
    parser.add_argument("namespace", nargs="?", default="robot168",
                        help="keyexpr 前缀，须与桥配置一致；桥未启用前缀时传空串")
    args = parser.parse_args()

    # 监控进程在 stdout 被重定向到文件时也必须即时打印。
    sys.stdout.reconfigure(line_buffering=True)

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=args.address, ns=args.namespace))
    if not robot.ready:
        print("[FAIL] {}".format(robot.last_error()))
        return 1
    print("monitoring {} for {:.0f} s (0 = until killed)...".format(args.address, args.seconds))

    # 精确接收计数：缓存只保留最新样本，轮询缓存会少算；回调每个样本触发一次。
    # 回调在 zenoh 会话线程上跑，用锁保护计数。
    received = [0]
    received_lock = threading.Lock()

    def on_feedback(_feedback):
        with received_lock:
            received[0] += 1

    robot.set_feedback_callback(on_feedback)
    robot.set_stale_callback(
        lambda ms: print("[WARN] no telemetry for {:.0f} ms (link or robot node down?)".format(ms)),
        threshold_ms=STALE_THRESHOLD_MS)

    have_feedback = False
    age_min = 0.0
    age_max = 0.0
    start = time.monotonic()
    while args.seconds <= 0.0 or time.monotonic() - start < args.seconds:
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
        print("[FAIL] no feedback received")
        return 1
    print("received {} samples in {:.1f} s (~{:.1f} Hz), age {:.1f}..{:.1f} ms".format(
        samples, elapsed, samples / elapsed if elapsed > 0.0 else 0.0, age_min, age_max))
    print("[PASS] get_joint completed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
