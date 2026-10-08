# FSM 切换示例：本示例演示如何使用SDK在 STOP 与 ENABLED 状态间切换。
# 按回车在 STOP 与 ENABLED 之间来回切换，Ctrl+C 退出。
# 退出时 FSM 保持当前状态不变。
#
# 初始状态用 get_state() 查询，不假设订阅缓存里已有 fsm_state 样本。
#
# 用法：set_fsm.py [<ip>:<port>] [namespace]
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：set_fsm.py <ip>:<port> ""
# 用法与缺省值也可以直接问示例：set_fsm.py --help
#
# 句柄不显式关闭，进程退出时随进程回收；需要提前释放时可正常 close()/with（v1.0.1 起，
# 此前版本的例外说明见 README）。

import sys

import example_common as common
import shidou


def toggle_fsm(robot, cfg, verdicts):
    """切换循环：回车切一次，stdin 关闭 / Ctrl+C / Ctrl+Z 退出（见 example_common.py）。

    某次切换失败只记 [FAIL] 并继续接受下一次按键；退出码由判定积累给出，
    操作者主动退出本身不算失败。
    """
    try:
        state = robot.get_state()
    except shidou.ShidouError as error:
        verdicts.fail("get_state: {}".format(error))
        return
    # 初始不是 STOP（ENABLED 或任一控制模式）时，第一次按键先 Stop。
    stopped = state.fsm_state == "STOP"
    print("current fsm_state={}".format(state.fsm_state))
    print("press Enter to toggle STOP/ENABLED, Ctrl+C to exit")

    toggles = 0
    while True:
        # 回车 = 切换一次；stdin 关闭 / Ctrl+C = 退出（见 example_common.py）。
        if not common.wait_enter():
            print("toggled {} times, final fsm_state={}".format(toggles, robot.fsm_state))
            shidou.shutdown()
            return
        action = "Enable" if stopped else "Stop"
        try:
            if stopped:
                robot.enable()
            else:
                robot.stop()
        except shidou.ShidouError as error:
            verdicts.fail("{}: {} (state unchanged)".format(action, error))
            continue
        stopped = robot.fsm_state == "STOP"
        toggles += 1
        verdicts.pass_("{} -> fsm_state={}".format(action, robot.fsm_state))


def main():
    # FSM 切换没有自己的参数：位置参数只有 [<ip>:<port>] 与 [namespace]。
    return common.run([], toggle_fsm)


if __name__ == "__main__":
    sys.exit(main())
