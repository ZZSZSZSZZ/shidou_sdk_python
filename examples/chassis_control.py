# 底盘示例：本示例演示如何使用SDK驱动底盘轮组（send_body 的轮组字段）并读回反馈。
# 流程：get_state() 查询状态 → 不在 ENABLED 则先 enable() → set_mode(POSITION) →
# 打印动作计划并等回车确认 → 以 100 Hz 持续下发轮速（正转 → 停 → 反转，每段先斜坡到
# 目标速度再保持）→ 每段结束按容差判定轮组反馈 → 停发一段验证死区 → 切回 ENABLED。
#
# send_body() 的轮组字段：wheel_ids 为轮毂电机号、velocities 为 rad/s，两者按索引对齐；
# max_currents 为电流上限（A，可短可空，缺省项用设备默认值）；顶杆字段留空
# （pushrod_id = 0 = 本帧无顶杆目标）。
#
# 轮组必须**持续下发**：机器人端按消息到达时刻计时，500 ms 无新消息就把轮速归零
# （死区），所以段内按固定周期连发（恒定值持续重发不会触发死区），而不是发一帧就等。
# 轮组只在 POSITION 态生效，其余状态下发会被强制归零。轮组反馈随 joint_states 到达，
# 按 motor_id 与顶杆区分（轮组 rad 与 rad/s）。收尾故意停发一段：超过死区窗口后轮速
# 应自行归零，不靠显式零速指令；只有仍未归零时才补发零速兜底。
# send_* 的失败不写错误文本（见 README），失败时只报结果。
#
# 用法：chassis_control.py [<ip>:<port>] [namespace]
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：chassis_control.py <ip>:<port> ""
#
# 本示例会让机器人真的移动：确认处放弃即什么都不下发，运行前先清场。
# 真机上不要用 with、也不要调 close()：销毁带流量的句柄有已知缺陷，进程退出即可
# （原因与例外见 README）。

import argparse
import sys
import time

import example_common as common
import shidou

PERIOD_S = 0.01             # 100 Hz 下发
WHEEL_IDS = [18, 19]        # 轮毂电机号
MAX_CURRENT = 5.0           # 每轮电流上限 A（示例值）
SPEED = 1.0                 # 段内轮速幅值 rad/s（示例值）
RAMP_SECONDS = 1.0          # 每段从上一速度斜坡到目标速度的时长
STOP_SECONDS = 1.0          # 停止段的保持时长
WHEEL_TOLERANCE = 0.3       # 轮速判定容差 rad/s（示例值）
DEADMAN_SECONDS = 1.5       # 死区用例的停发时长（> 机器人端 500 ms 窗口）
FSM_ENABLED = "ENABLED"

# 动作段：(标签, 目标速度 rad/s, 保持时长 s)，每段先斜坡到目标速度再保持。
SEGMENTS = [
    ("forward", SPEED, 1.5),
    ("stop", 0.0, STOP_SECONDS),
    ("reverse", -SPEED, 1.5),
]


def send_wheels(robot, speed):
    """下发一帧轮速：各轮同速、电流上限取示例值；顶杆字段留空 = 本帧不动顶杆。"""
    return robot.send_body(wheel_ids=WHEEL_IDS,
                           velocities=[speed] * len(WHEEL_IDS),
                           max_currents=[MAX_CURRENT] * len(WHEEL_IDS))


def publish_loop(robot, speed, seconds):
    """以固定周期持续下发同一速度，持续 seconds 秒；返回 False 表示下发失败。"""
    start = time.monotonic()
    while True:
        if time.monotonic() - start >= seconds:
            return True
        if not send_wheels(robot, speed):
            return False
        time.sleep(PERIOD_S)


def print_wheel_feedback(robot, expected_vel):
    """打印反馈里的各轮（位置 rad、速度 rad/s）并与目标速度比容差；返回超差轮数。

    反馈样本里没有轮组时（假节点只发关节电机）只报未判定，不算失败。
    """
    feedback = robot.last_feedback()
    if feedback is None:
        print("  wheels not judged: no feedback sample")
        return 0
    judged = 0
    off = 0
    for motor_id in WHEEL_IDS:
        for index, motor in enumerate(feedback.motor_ids):
            if motor != motor_id:
                continue
            position = feedback.position[index] if index < len(feedback.position) else 0.0
            velocity = feedback.velocity[index] if index < len(feedback.velocity) else 0.0
            ok = abs(velocity - expected_vel) <= WHEEL_TOLERANCE
            judged += 1
            if not ok:
                off += 1
            print("  wheel {}: pos={:.3f} rad vel={:.3f} rad/s{}".format(
                motor_id, position, velocity, "" if ok else " [off target]"))
            break
    if judged == 0:
        print("  wheels not judged: none of {} in feedback".format(len(WHEEL_IDS)))
    elif off == 0:
        print("  wheels ok: {}/{} within {:.3f} rad/s of {:.3f}".format(
            judged, len(WHEEL_IDS), WHEEL_TOLERANCE, expected_vel))
    else:
        print("  wheels off target: {} of {} judged".format(off, judged))
    return off


def main():
    parser = argparse.ArgumentParser(description="驱动底盘轮组走一段正转/停/反转并判定反馈。")
    parser.add_argument("address", nargs="?", default="192.168.168.168:7447",
                        help="机器人侧 zenoh 桥的 <ip>:<port>（缺省 192.168.168.168:7447）")
    parser.add_argument("namespace", nargs="?", default="robot168",
                        help="keyexpr 前缀，须与桥配置一致；桥未启用前缀时传空串")
    args = parser.parse_args()

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

    # 模式命令只能从 ENABLED 发起（STOP 态只接受 enable，不能直接切模式）。
    if state.fsm_state != FSM_ENABLED:
        try:
            robot.enable()
        except shidou.ShidouError as error:
            print("[FAIL] enable: {}".format(error))
            shidou.shutdown()
            return 1
        print("fsm_state={}".format(robot.fsm_state))

    # 轮组只在 POSITION 态生效：切模式并确认后再发轮速。
    try:
        robot.set_mode(shidou.ControlMode.POSITION)
    except shidou.ShidouError as error:
        print("[FAIL] set_mode(POSITION): {}".format(error))
        shidou.shutdown()
        return 1
    print("fsm_state={}".format(robot.fsm_state))

    planned = sum(RAMP_SECONDS + hold for _, _, hold in SEGMENTS)
    print("plan: wheels {}/{}, {} segments ({:.0f} s): +{:.1f} -> 0 -> {:.1f} rad/s, then "
          "{:.1f} s without publishing (deadman)".format(
              WHEEL_IDS[0], WHEEL_IDS[1], len(SEGMENTS), planned, SPEED, -SPEED,
              DEADMAN_SECONDS))
    print("the robot WILL move, keep the area clear")
    if not common.wait_enter("press Enter to start, Ctrl+C to exit"):
        print("aborted, nothing was commanded")
        shidou.shutdown()
        return 0

    wheels_ok = True  # 任一段或死区用例的轮速判定失败即置 False
    previous = 0.0
    for label, speed, hold in SEGMENTS:
        print("segment {}: {:.2f} -> {:.2f} rad/s (ramp {:.0f} s, hold {:.1f} s)".format(
            label, previous, speed, RAMP_SECONDS, hold))
        # 斜坡段按实际经过时间插值，睡眠抖动不累积。
        begin = time.monotonic()
        while True:
            elapsed = time.monotonic() - begin
            if elapsed >= RAMP_SECONDS:
                break
            if not send_wheels(robot, previous + (speed - previous) * (elapsed / RAMP_SECONDS)):
                print("[FAIL] send_body failed")
                shidou.shutdown()
                return 1
            time.sleep(PERIOD_S)
        if not publish_loop(robot, speed, hold):
            print("[FAIL] send_body failed")
            shidou.shutdown()
            return 1
        if print_wheel_feedback(robot, speed) > 0:
            wheels_ok = False
        previous = speed

    # 死区用例：停发 DEADMAN_SECONDS（超过机器人端 500 ms 窗口），期间一帧不发，轮速应
    # 自行归零；只有仍偏离 0 时才补发零速兜底（否则连发零速会掩盖死区是否真的生效）。
    print("deadman: {:.1f} s without publishing, wheels should return to 0 rad/s".format(
        DEADMAN_SECONDS))
    time.sleep(DEADMAN_SECONDS)
    deadman_off = print_wheel_feedback(robot, 0.0)
    if deadman_off > 0:
        wheels_ok = False
        if not publish_loop(robot, 0.0, STOP_SECONDS):
            print("[FAIL] send_body failed")
            shidou.shutdown()
            return 1

    # 切回 ENABLED，机器人回到可再次接收模式命令的状态。
    try:
        robot.enable()
    except shidou.ShidouError as error:
        shidou.shutdown()
        print("[FAIL] enable: {}".format(error))
        return 1
    shidou.shutdown()
    print("[PASS] Enable -> fsm_state={}".format(robot.fsm_state))
    if not wheels_ok:
        print("[FAIL] wheel velocities off target (see above)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
