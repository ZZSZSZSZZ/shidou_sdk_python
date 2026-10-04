# 顶杆示例：本示例演示如何使用SDK在使能模式下填写顶杆目标位置参数使顶杆前往目标点并读回反馈。
# 流程：get_state() 查询状态 → 不在 ENABLED 则先 enable()（全程保持在使能模式，不下发模式切换）
# → 从遥测反馈读顶杆当前位置 → 打印动作计划并等回车确认 → 填目标位置下发一帧 send_body()
# （只填顶杆字段；目标由命令行给出，缺省 0，即零点）→ 轮询反馈直到到位或超时 → 结束
# （单程，不回起点）。
#
# send_body() 的顶杆字段：pushrod_id 为顶杆电机号（0 = 本帧无顶杆目标），position 单位为
# mm，是绝对目标位置参数（填多少就走到哪）；velocity / acceleration 为设备侧轮廓速度与
# 加速度（mm/s、mm/s²）；轮组字段留空（空数组 = 本帧不动轮组）。顶杆在 ENABLED 与
# POSITION 下均生效、进入位置闭环后自保持，所以示例每次动作只填参数下发一帧，不逐周期重发。
# 顶杆反馈不在独立话题：它随 joint_states 一起到达，按 motor_id 与轮组区分（顶杆 mm）。
#
# 起点取反馈里的当前位置，所以**没有反馈样本时示例不动电机**并直接退出——不盲发目标。
# 目标位置的物理含义（哪一端是零点、位置增大朝哪走）由机器人端定义，示例只负责填参数。
# send_* 的失败不写错误文本（见 README），失败时只报结果。
#
# 用法：pushrod_control.py [target_mm] [<ip>:<port>] [namespace]
# target_mm 为目标位置参数（mm，绝对值，填多少就走到哪），缺省 0（零点）。
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：pushrod_control.py <target_mm> <ip>:<port> ""
#
# 句柄不显式关闭，进程退出时随进程回收；需要提前释放时可正常 close()/with（v1.0.1 起，
# 此前版本的例外说明见 README）。

import argparse
import sys
import time

import example_common as common
import shidou

FEEDBACK_PERIOD_S = 0.1        # 到位判据的反馈轮询周期
PUSHROD_ID = 17                # 顶杆电机号（示例值，按机器人实际配置修改）
DEFAULT_TARGET_MM = 0.0        # 目标位置参数缺省值（命令行不填时取 0，即零点）
PROFILE_VELOCITY = 20.0        # 轮廓速度 mm/s（示例值）
PROFILE_ACCELERATION = 50.0    # 轮廓加速度 mm/s²（示例值）
POSITION_TOLERANCE_MM = 1.0    # 到位判据：位置差不超过该值即算到位
MOVE_TIMEOUT_MARGIN_S = 10.0   # 到位等待超时余量（行程 / 轮廓速度 + 余量）
FEEDBACK_WAIT_S = 5.0          # 等首帧反馈的上限（订阅建立到首帧到达的空窗）
FSM_ENABLED = "ENABLED"


def read_position(robot, motor_id):
    """从遥测反馈里按电机号取位置；没有样本或样本中缺该电机时返回 (False, 0.0)。"""
    feedback = robot.last_feedback()
    if feedback is None:
        return False, 0.0
    for index, motor in enumerate(feedback.motor_ids):
        if motor == motor_id and index < len(feedback.position):
            return True, feedback.position[index]
    return False, 0.0


def wait_first_position(robot, motor_id):
    """等首帧反馈，返回 (是否拿到位置, 位置, 是否见过样本)。

    订阅建立到首帧 joint_states 到达之间有短暂空窗，立刻读会误判成"无反馈"。
    saw_sample 区分"一帧反馈都没有"（链路或节点没通）与"有反馈但没有该电机"（电机号配错）。
    """
    start = time.monotonic()
    saw_sample = False
    while True:
        found, value = read_position(robot, motor_id)
        if found:
            return True, value, saw_sample
        saw_sample = robot.feedback_age_ms >= 0.0  # 负值表示还没收到过任何样本
        if time.monotonic() - start >= FEEDBACK_WAIT_S:
            return False, 0.0, saw_sample
        time.sleep(FEEDBACK_PERIOD_S)


def send_pushrod(robot, position):
    """下发一帧顶杆目标：只填顶杆字段，轮组字段留空（本帧不动轮组）。"""
    return robot.send_body(pushrod_id=PUSHROD_ID, position=position,
                           velocity=PROFILE_VELOCITY, acceleration=PROFILE_ACCELERATION)


def wait_in_position(robot, start_mm, goal_mm):
    """轮询反馈直到位置进入容差或超时；返回 (是否到位, 实测位置)。

    超时按本段行程（start → goal）与轮廓速度估算，避免长行程被固定超时误判为失败。
    """
    timeout = abs(goal_mm - start_mm) / PROFILE_VELOCITY + MOVE_TIMEOUT_MARGIN_S
    begin = time.monotonic()
    polls = 0
    measured = 0.0
    while True:
        found, now_mm = read_position(robot, PUSHROD_ID)
        if found:
            measured = now_mm
            if abs(now_mm - goal_mm) <= POSITION_TOLERANCE_MM:
                return True, measured
            polls += 1
            if polls % 10 == 0:  # 约每 1 s 打一行进度
                print("  position={:.2f} mm (goal {:.2f} mm)".format(now_mm, goal_mm))
        if time.monotonic() - begin >= timeout:
            return False, measured
        time.sleep(FEEDBACK_PERIOD_S)


def main():
    parser = argparse.ArgumentParser(description="使能模式下把顶杆送到目标位置并按反馈判定到位。")
    parser.add_argument("target_mm", nargs="?", default=None,
                        help="目标位置参数（mm，绝对值），缺省 0（零点）")
    parser.add_argument("address", nargs="?", default="192.168.168.168:7447",
                        help="机器人侧 zenoh 桥的 <ip>:<port>（缺省 192.168.168.168:7447）")
    parser.add_argument("namespace", nargs="?", default="robot168",
                        help="keyexpr 前缀，须与桥配置一致；桥未启用前缀时传空串")
    args = parser.parse_args()

    # 目标位置参数（mm，绝对值）；非法参数直接退出，避免把输入错误当成目标下发给电机。
    target_mm = DEFAULT_TARGET_MM
    if args.target_mm is not None:
        try:
            target_mm = float(args.target_mm)
        except ValueError:
            print("[FAIL] bad target_mm: {} (expected a number in mm)".format(args.target_mm))
            return 1

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=args.address, ns=args.namespace))
    if not robot.ready:
        print("[FAIL] {}".format(robot.last_error()))
        return 1

    try:
        state = robot.get_state()
    except shidou.ShidouError as error:
        print("[FAIL] get_state: {}".format(error))
        shidou.shutdown()
        return 1
    print("fsm_state={}".format(state.fsm_state))

    # 顶杆在 ENABLED 下即可接收目标位置参数，示例全程保持在使能模式，不下发模式切换；
    # 处于其他状态（POSITION / STOP）时先切回 ENABLED。
    if state.fsm_state != FSM_ENABLED:
        try:
            robot.enable()
        except shidou.ShidouError as error:
            print("[FAIL] enable: {}".format(error))
            shidou.shutdown()
            return 1
        print("fsm_state={}".format(robot.fsm_state))

    # 起点只能来自反馈：拿不到就什么都不发（盲发目标可能让顶杆从任意位置起跳）。
    found, start_mm, saw_sample = wait_first_position(robot, PUSHROD_ID)
    if not found:
        print("[FAIL] no position for pushrod motor {} in {:.0f} s ({}): "
              "nothing was commanded".format(
                  PUSHROD_ID, FEEDBACK_WAIT_S,
                  "feedback has no such motor" if saw_sample else "no feedback sample"))
        shidou.shutdown()
        return 1

    print("plan: pushrod {} {:.2f} mm -> {:.2f} mm (profile {:.0f} mm/s, {:.0f} mm/s^2)".format(
        PUSHROD_ID, start_mm, target_mm, PROFILE_VELOCITY, PROFILE_ACCELERATION))
    if not common.wait_enter("press Enter to start, Ctrl+C to exit"):
        print("aborted, nothing was commanded")
        shidou.shutdown()
        return 0

    if not send_pushrod(robot, target_mm):
        print("[FAIL] send_body failed")
        shidou.shutdown()
        return 1
    reached, measured = wait_in_position(robot, start_mm, target_mm)
    if not reached:
        print("[FAIL] pushrod did not reach {:.2f} mm (measured {:.2f} mm)".format(
            target_mm, measured))
        shidou.shutdown()
        return 1
    print("[PASS] pushrod {:.2f} mm -> {:.2f} mm (measured {:.2f} mm)".format(
        start_mm, target_mm, measured))

    shidou.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
