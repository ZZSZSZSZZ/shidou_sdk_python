# 只读示例：查询一次机器人状态并逐字段打印，用于确认地址、namespace 与机器人端
# 桥是否对得上（FSM 状态、臂型/机身构型、电机列表、最近一帧遥测的年龄）。
# 不下发任何目标。
#
# 流程：构造 Robot（自开会话）→ get_state 服务查询 → 逐字段打印。
#
# 用法：get_state.py [<ip>:<port>] [namespace]

import sys

import shidou


def main():
    # 可选参数：机器人地址与 namespace，省略时用出厂默认值。
    address = sys.argv[1] if len(sys.argv) > 1 else "192.168.168.168:7447"
    # namespace 必须与机器人侧桥配置完全一致；桥未启用 namespace 时传空串。
    namespace = sys.argv[2] if len(sys.argv) > 2 else "robot168"

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=address, namespace=namespace))
    if not robot.ready:
        print(robot.last_error())
        return 1

    # 服务查询：一次取回 FSM 状态、配置字段与最新遥测快照。
    state = robot.get_state()
    print("fsm_state      : {}".format(state.fsm_state))
    print("arm_info       : {}".format(state.arm_info))
    print("gripper_info   : {}".format(state.gripper_info))
    # 型号名：手写配置通道下为空串。
    print("model          : {}".format(state.model))
    # 机身构型：none / wheels / pushrod / wheels_pushrod 的组合。
    print("body_info      : {}".format(state.body_info))
    print("library_status : 0x{:04x}".format(state.library_status))
    print("control_freq_hz: {:.1f}".format(state.control_freq_hz))
    print("cycle avg/max  : {:.2f} / {:.2f} ms".format(state.control_cycle_avg_ms,
                                                       state.control_cycle_max_ms))
    print("cycle overruns : {}".format(state.cycle_overruns))
    # 电机列表与位置来自服务快照；顺序不保证与遥测一致，跨来源按 motor_ids 对齐。
    # mode/confidence 也按 motor_ids 对齐，机器人不上报时显示 "-"；只有
    # confirmed 表示设备确认处在所报模式，其余情况下设备可能仍停在旧模式。
    print("motors         : {}".format(len(state.motor_ids)))
    for index, motor_id in enumerate(state.motor_ids):
        position = state.positions[index] if index < len(state.positions) else 0.0
        mode = state.control_mode[index] if index < len(state.control_mode) else "-"
        confidence = (state.control_mode_confidence[index]
                      if index < len(state.control_mode_confidence) else "-")
        print("  motor {} pos={:.4f} mode={} confidence={}".format(
            motor_id, position, mode, confidence))
    # 遥测快照：尚未收到样本时 has_feedback 为 false、age 为负。
    if state.has_feedback:
        print("feedback       : seq={} age={:.1f} ms".format(state.feedback_seq,
                                                             state.feedback_age_ms))
    else:
        print("feedback       : none yet")
    return 0


if __name__ == "__main__":
    sys.exit(main())
