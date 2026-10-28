# 只读示例：每 500 ms 用 last_feedback() 打印一帧最新关节反馈，共 10 帧、5 秒。
# 逐电机打印位置、速度、力矩、使能、在线、故障码与温度，用来在机器人旁确认
# 遥测链路与电机在线情况。不下发任何目标。
#
# 流程：构造 Robot（订阅在后台线程收包）→ 循环取缓存快照 → 打印 seq/age 与
# 每个电机的全部反馈字段。
#
# 用法：get_joint.py [<ip>:<port>] [namespace]

import sys
import time

import shidou

PRINT_PERIOD_S = 0.5     # 打印间隔 s
FRAMES = 10              # 打印帧数；10 × 0.5 s = 5 秒

# 表头与数据行用同一组列宽，保证两处对齐。
HEADER_FORMAT = "{:>6}  {:>8}  {:>10}  {:>8}  {:>7}  {:>7}  {:>6}  {:>5}"
ROW_FORMAT = "{:6d}  {:8.4f}  {:10.4f}  {:8.4f}  {:7.1f}  {:7d}  {:6d}  {:5d}"


def at(values, index, default):
    """反馈的各数组按 motor_ids 索引对齐；个别数组缺项时返回缺省值，避免越界。"""
    return values[index] if index < len(values) else default


def main():
    address = sys.argv[1] if len(sys.argv) > 1 else "192.168.168.168:7447"
    namespace = sys.argv[2] if len(sys.argv) > 2 else "robot168"

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=address, namespace=namespace))
    if not robot.ready:
        print(robot.last_error())
        return 1

    print(HEADER_FORMAT.format("motor", "pos(rad)", "vel(rad/s)", "eff(Nm)",
                               "temp(C)", "enabled", "online", "fault"))

    # 10 帧 × 500 ms = 5 秒。
    for _ in range(FRAMES):
        time.sleep(PRINT_PERIOD_S)

        # last_feedback() 读的是缓存里的最新一帧；还没有样本时返回 None。
        feedback = robot.last_feedback()
        if feedback is None:
            print("no feedback yet")
            continue
        # age 是这帧的年龄；持续变大说明遥测断流。
        print("fsm={} seq={} age={:.1f} ms motors={}".format(
            robot.fsm_state, robot.feedback_seq, robot.feedback_age_ms,
            len(feedback.motor_ids)))
        # enabled/online 是 0/1：enabled = 该电机已使能，online = 未禁用且状态新鲜；
        # fault 0 = 无故障。
        for index, motor_id in enumerate(feedback.motor_ids):
            print(ROW_FORMAT.format(
                motor_id,
                at(feedback.position, index, 0.0),
                at(feedback.velocity, index, 0.0),
                at(feedback.effort, index, 0.0),
                at(feedback.temperature, index, 0.0),
                at(feedback.enabled, index, 0),
                at(feedback.online, index, 0),
                at(feedback.fault_code, index, 0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
