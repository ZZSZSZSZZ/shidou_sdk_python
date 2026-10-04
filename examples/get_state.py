# 状态查询示例：本示例演示如何使用SDK查询机器人状态。
# 用 get_state() 一次取回状态，逐字段打印。
#
# 用法：get_state.py [<ip>:<port>] [namespace]
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：get_state.py <ip>:<port> ""
#
# 句柄不显式关闭，进程退出时随进程回收；需要提前释放时可正常 close()/with（v1.0.1 起，
# 此前版本的例外说明见 README）。

import argparse
import sys

import shidou


def main():
    parser = argparse.ArgumentParser(description="查询一次机器人状态并逐字段打印。")
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

    # get_state 返回服务字段与遥测缓存的合并结果；反馈字段来自遥测快照。
    try:
        state = robot.get_state()
    except shidou.ShidouError as error:
        print("[FAIL] {}".format(error))
        return 1

    print("fsm_state      : {}".format(state.fsm_state))
    print("arm_info       : {}".format(state.arm_info))
    print("gripper_info   : {}".format(state.gripper_info))
    print("library_status : 0x{:04x}".format(state.library_status))
    print("control_freq_hz: {:.1f}".format(state.control_freq_hz))
    print("cycle avg/max  : {:.2f} / {:.2f} ms".format(state.control_cycle_avg_ms,
                                                       state.control_cycle_max_ms))
    print("cycle overruns : {}".format(state.cycle_overruns))
    print("motors         : {}".format(len(state.motor_ids)))
    for index, motor_id in enumerate(state.motor_ids):
        position = state.positions[index] if index < len(state.positions) else 0.0
        print("  motor {} pos={:.4f}".format(motor_id, position))
    # 遥测快照：尚未收到样本时 has_feedback 为 false、age 为负。
    if state.has_feedback:
        print("feedback       : seq={} age={:.1f} ms".format(state.feedback_seq,
                                                             state.feedback_age_ms))
    else:
        print("feedback       : none yet")

    shidou.shutdown()
    print("[PASS] get_state completed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
