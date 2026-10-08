# 状态查询示例：本示例演示如何使用SDK查询机器人状态。
# 用 get_state() 一次取回状态，逐字段打印。
#
# 用法：get_state.py [<ip>:<port>] [namespace]
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：get_state.py <ip>:<port> ""
# 用法与缺省值也可以直接问示例：get_state.py --help
#
# 句柄不显式关闭，进程退出时随进程回收；需要提前释放时可正常 close()/with（v1.0.1 起，
# 此前版本的例外说明见 README）。

import sys

import example_common as common
import shidou


def query_state(robot, cfg, verdicts):
    """取一次状态并逐字段打印。"""
    # get_state 返回服务字段与遥测缓存的合并结果；反馈字段来自遥测快照。
    try:
        state = robot.get_state()
    except shidou.ShidouError as error:
        verdicts.fail("get_state: {}".format(error))
        return

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
    verdicts.pass_("get_state completed")


def main():
    # 状态查询没有自己的参数：位置参数只有 [<ip>:<port>] 与 [namespace]。
    return common.run([], query_state)


if __name__ == "__main__":
    sys.exit(main())
