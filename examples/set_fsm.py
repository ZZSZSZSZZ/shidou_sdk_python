# FSM 切换示例：本示例演示如何使用SDK在 STOP 与 ENABLED 状态间切换。
# 按回车在 STOP 与 ENABLED 之间来回切换，Ctrl+C 退出。
# 退出时 FSM 保持当前状态不变。
#
# 初始状态用 get_state() 查询，不假设订阅缓存里已有 fsm_state 样本。
#
# 用法：set_fsm.py [<ip>:<port>] [namespace]
# namespace 须与机器人侧桥配置的 namespace 完全一致（缺省 robot168）；桥未启用
# namespace 时显式传空串：set_fsm.py <ip>:<port> ""
#
# 真机上不要用 with、也不要调 close()：销毁带流量的句柄有已知缺陷，进程退出即可
# （原因与例外见 README）。

import argparse
import sys

import example_common as common
import shidou


def main():
    parser = argparse.ArgumentParser(description="在 STOP 与 ENABLED 之间手动切换。")
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
        return 1
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
            return 0
        action = "Enable" if stopped else "Stop"
        try:
            if stopped:
                robot.enable()
            else:
                robot.stop()
        except shidou.ShidouError as error:
            print("[FAIL] {}: {} (state unchanged)".format(action, error))
            continue
        stopped = robot.fsm_state == "STOP"
        toggles += 1
        print("[PASS] {} -> fsm_state={}".format(action, robot.fsm_state))


if __name__ == "__main__":
    sys.exit(main())
