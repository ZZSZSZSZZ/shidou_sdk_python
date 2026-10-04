# MIT 目标流示例：本示例演示如何使用SDK按臂布局对 MIT 关节做位置扫动。
# 流程：get_state() 查询状态 → 不在 ENABLED 则先 enable() → set_mode(POSITION)
# → 以 100 Hz 下发 send_mit() → 扫动结束切回 ENABLED。
# 扫动电机取每臂末端，电机号由 arm_info 的自由度得出（左臂 = dof，右臂 = dof + 7）；
# 单臂扫动一个电机，双臂依次扫动（先左后右）。
# 每个电机从当前位置出发，直线插值走三段：当前位置 → 0.3 (1 秒) → -0.3 (1 秒) → 0 (1 秒)。
# send_mit() 字段：positions 为目标角，velocities 为速度前馈（取段内斜率），kps/kds 为
# 关节刚度/阻尼（示例值），torques 留空即前馈力矩 0。
# 注意：模式命令只能从 ENABLED 发起（STOP 状态只接受 enable），切 POSITION 前先确认
# 当前状态；扫动结束切回 ENABLED。
#
# 用法：mit_control.py [<ip>:<port>] [namespace]
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：mit_control.py <ip>:<port> ""
#
# 本示例会下发模式切换与运动目标：没有确认提示，运行即动作，先确保机器人周围安全。
# 句柄不显式关闭，进程退出时随进程回收；需要提前释放时可正常 close()/with（v1.0.1 起，
# 此前版本的例外说明见 README）。

import argparse
import sys
import time

import shidou

PERIOD_S = 0.01          # 100 Hz 下发
SEGMENT_SECONDS = 1.0    # 每段 1 秒
WAYPOINT_HIGH = 0.3      # 第一段终点 rad
WAYPOINT_LOW = -0.3      # 第二段终点 rad
WAYPOINT_HOME = 0.0      # 回零目标 rad
GAIN_KP = 40.0           # 关节刚度（示例值，可按需调整）
GAIN_KD = 2.0            # 关节阻尼（示例值，可按需调整）
FSM_ENABLED = "ENABLED"


def parse_dof(text):
    """解析自由度段："5" → 左右同为 5；"5x7" → 左 5、右 7；非法返回 None。"""
    left_text, separator, right_text = text.partition("x")
    if not separator:
        right_text = left_text
    if not left_text.isdigit() or not right_text.isdigit():
        return None
    return int(left_text), int(right_text)


def parse_arm_layout(arm_info):
    """解析 arm_info（"型号_布局_自由度"，布局段为 left / right / dual）。

    返回 (has_left, has_right, left_dof, right_dof)；没有可识别的布局段或自由度
    段非法时返回 None。非布局段（如型号段）跳过，不解析其后的段。
    """
    segments = arm_info.split("_")
    for index in range(len(segments) - 1):
        layout = segments[index]
        if layout not in ("left", "right", "dual"):
            continue
        dof = parse_dof(segments[index + 1])
        if dof is None:
            return None
        left_dof, right_dof = dof
        has_left = layout in ("left", "dual")
        has_right = layout in ("right", "dual")
        return (has_left, has_right,
                left_dof if has_left else 0, right_dof if has_right else 0)
    return None


def current_position(state, motor_id):
    """从 get_state 快照中找电机当前位置；快照没有该电机时回退 0 并告警。"""
    for index, motor in enumerate(state.motor_ids):
        if motor == motor_id and index < len(state.positions):
            return state.positions[index]
    print("[WARN] motor {} not in get_state snapshot, sweep from 0".format(motor_id))
    return 0.0


def sweep_motor(robot, motor_id, start):
    """单电机三段直线插值扫动：start → 0.3 (1 s) → -0.3 (1 s) → 0 (1 s)。

    段内以实际经过时间为插值变量，睡眠抖动不累积；velocity 取段内斜率做速度前馈，
    kp/kd 固定为示例值。每段结束补发终点帧（速度前馈清零），保证段间衔接与最终
    停位精确。
    """
    waypoints = [start, WAYPOINT_HIGH, WAYPOINT_LOW, WAYPOINT_HOME]
    print("motor {}: {:.3f} -> {:.3f} -> {:.3f} -> {:.3f} ({:.0f} s per segment)".format(
        motor_id, waypoints[0], waypoints[1], waypoints[2], waypoints[3], SEGMENT_SECONDS))
    for segment in range(len(waypoints) - 1):
        begin = waypoints[segment]
        end = waypoints[segment + 1]
        slope = (end - begin) / SEGMENT_SECONDS  # 段内速度前馈 rad/s
        segment_start = time.monotonic()
        while True:
            elapsed = time.monotonic() - segment_start
            if elapsed >= SEGMENT_SECONDS:
                break
            # torques 不填 = 前馈力矩 0。
            if not robot.send_mit(motor_ids=[motor_id],
                                  positions=[begin + (end - begin) * (elapsed / SEGMENT_SECONDS)],
                                  velocities=[slope],
                                  kps=[GAIN_KP], kds=[GAIN_KD]):
                # send_* 的失败不写错误文本（见 README），这里只报结果。
                print("[FAIL] publish motor {} failed".format(motor_id))
                return False
            time.sleep(PERIOD_S)
        if not robot.send_mit(motor_ids=[motor_id], positions=[end], velocities=[0.0],
                              kps=[GAIN_KP], kds=[GAIN_KD]):
            print("[FAIL] publish motor {} failed".format(motor_id))
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description="对每臂末端电机做 MIT 位置扫动。")
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

    # 先查状态：arm_info 决定扫动电机，fsm_state 决定是否需要先 Enable。
    try:
        state = robot.get_state()
    except shidou.ShidouError as error:
        print("[FAIL] get_state: {}".format(error))
        shidou.shutdown()
        return 1
    print("arm_info={}".format(state.arm_info))
    print("fsm_state={}".format(state.fsm_state))

    # 模式命令只能从 ENABLED 发起（STOP 态只接受 enable，不能直接切模式），
    # 当前不在 ENABLED 时先使能握手。
    if state.fsm_state != FSM_ENABLED:
        try:
            robot.enable()
        except shidou.ShidouError as error:
            print("[FAIL] enable: {}".format(error))
            shidou.shutdown()
            return 1
        print("fsm_state={}".format(robot.fsm_state))

    layout = parse_arm_layout(state.arm_info)
    if layout is None:
        print("[FAIL] arm_info={}: cannot parse layout/dof".format(state.arm_info))
        shidou.shutdown()
        return 1
    has_left, has_right, left_dof, right_dof = layout
    layout_name = "dual" if has_left and has_right else ("left" if has_left else "right")
    print("arm layout: {} (left dof={}, right dof={})".format(layout_name, left_dof, right_dof))

    # 末端电机号由自由度得出：左 = dof、右 = dof + 7；双臂依次（先左后右）。
    motors = []
    if has_left:
        motors.append(left_dof)
    if has_right:
        motors.append(right_dof + 7)

    # MIT 目标由 POSITION 状态消费：先切入 POSITION 模式，确认后开始扫动。
    try:
        robot.set_mode(shidou.ControlMode.POSITION)
    except shidou.ShidouError as error:
        print("[FAIL] set_mode(POSITION): {}".format(error))
        shidou.shutdown()
        return 1
    print("fsm_state={}, sweeping {} motor(s)".format(robot.fsm_state, len(motors)))

    for motor_id in motors:
        if not sweep_motor(robot, motor_id, current_position(state, motor_id)):
            shidou.shutdown()
            return 1

    # 扫动结束切回 ENABLED，机器人回到可再次接收模式命令的状态。
    try:
        robot.enable()
    except shidou.ShidouError as error:
        shidou.shutdown()
        print("[FAIL] enable: {}".format(error))
        return 1
    shidou.shutdown()
    print("[PASS] Enable -> fsm_state={}".format(robot.fsm_state))
    return 0


if __name__ == "__main__":
    sys.exit(main())
