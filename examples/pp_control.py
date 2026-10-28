# PP 关节示例：每条臂的末端电机从当前位置走一个来回（+0.3 rad 再回原位）。
# PP 的轮廓规划在驱动侧完成，所以每程只发一帧终点位置，不逐周期插值；
# 灵足与 EYOU 两族臂都支持 PP。
#
# 流程：查询状态与臂型 → 进入 PP → 逐电机走一个来回 → 切回 ENABLED。
#
# 用法：pp_control.py [<ip>:<port>] [namespace]

import sys
import time

import shidou

SEGMENT_WAIT_S = 1.0     # 每程发完后等驱动走完轮廓
AMPLITUDE = 0.3          # 单程摆幅 rad（示例值，可按需调整）


def parse_dof(text):
    """解析自由度段："5" → 左右同为 5；"5x7" → 左 5、右 7；非法返回 None。"""
    left_text, separator, right_text = text.partition("x")
    if not separator:
        right_text = left_text
    if not left_text.isdigit() or not right_text.isdigit():
        return None
    return int(left_text), int(right_text)


def parse_arm_layout(arm_info):
    """解析 arm_info 里的构型段：left_7 / right_7 / dual_5x7；旧版机器人还会带
    电机家族前缀（如 EYOU_left_7），非构型段跳过。返回 (has_left, has_right,
    left_dof, right_dof)，解析失败返回 None。"""
    segments = arm_info.split("_")
    for index in range(len(segments) - 1):
        side = segments[index]
        if side not in ("left", "right", "dual"):
            continue
        dof = parse_dof(segments[index + 1])
        if dof is None:
            return None
        left_dof, right_dof = dof
        has_left = side in ("left", "dual")
        has_right = side in ("right", "dual")
        return (has_left, has_right,
                left_dof if has_left else 0, right_dof if has_right else 0)
    return None


def current_position(state, motor_id):
    """从 get_state 快照中找电机当前位置；快照没有该电机时回退 0。"""
    for index, motor in enumerate(state.motor_ids):
        if motor == motor_id and index < len(state.positions):
            return state.positions[index]
    print("motor {} not in get_state snapshot, sweep from 0".format(motor_id))
    return 0.0


def sweep_motor(robot, motor_id, start):
    """单电机走一个来回：start → start + AMPLITUDE → start。每程只发一帧终点位置，
    由驱动侧规划整段轮廓；速度、力矩上限与加速度取机器人端配置。失败返回 1，
    错误消息已打印。"""
    peak = start + AMPLITUDE
    print("motor {}: {:.3f} -> {:.3f} -> {:.3f}".format(motor_id, start, peak, start))
    for position in (peak, start):
        # 只填电机号与位置；其余字段留空 = 取机器人端配置。
        if not robot.send_position(motor_ids=[motor_id], positions=[position]):
            print("send_position: {}".format(robot.last_error()))
            return 1
        # 等驱动把这一段走完，再发下一程。
        time.sleep(SEGMENT_WAIT_S)
    return 0


def sweep_joints(robot):
    state = robot.get_state()
    print("arm_info={}".format(state.arm_info))
    print("fsm_state={}".format(state.fsm_state))

    # arm_info 决定哪几条臂参与扫动，以及各自的末端电机号。
    layout = parse_arm_layout(state.arm_info)
    if layout is None:
        print("arm_info={}: cannot parse layout/dof".format(state.arm_info))
        return 1
    has_left, has_right, left_dof, right_dof = layout
    print("arm layout: {} (left dof={}, right dof={})".format(
        "dual" if has_left and has_right else ("left" if has_left else "right"),
        left_dof, right_dof))

    # 末端电机号由自由度得出：左 = dof、右 = dof + 7；双臂依次（先左后右）。
    motors = []
    if has_left:
        motors.append(left_dof)
    if has_right:
        motors.append(right_dof + 7)

    # enable 落点始终是 ENABLED；PP 由 set_mode 进入（先停驱动再重新使能）。
    robot.enable(shidou.ControlMode.PP)
    robot.set_mode(shidou.ControlMode.PP)
    # 状态由 set_mode 回执后刷新，确认已进入 PP。
    print("fsm_state={}".format(robot.fsm_state))

    for motor_id in motors:
        if sweep_motor(robot, motor_id, current_position(state, motor_id)) != 0:
            return 1

    # 回到 ENABLED；未停稳时 enable 自动重发。
    robot.enable(shidou.ControlMode.PP)
    # 打印最终状态，确认回到 ENABLED。
    print("fsm_state={}".format(robot.fsm_state))
    return 0


def main():
    address = sys.argv[1] if len(sys.argv) > 1 else "192.168.168.168:7447"
    namespace = sys.argv[2] if len(sys.argv) > 2 else "robot168"

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=address, namespace=namespace))
    if not robot.ready:
        print(robot.last_error())
        return 1
    return sweep_joints(robot)


if __name__ == "__main__":
    sys.exit(main())
