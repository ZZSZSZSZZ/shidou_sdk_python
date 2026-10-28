# 状态机最小切换：先进入 ENABLED（阻尼保持：电机通电、跟手有阻尼但不会自己动），
# 再 stop() 回失能。全程不下发运动目标。
#
# 流程：查询当前状态 → 不在 ENABLED 时 enable → 等 1 秒 → stop。
# 注意：stop() 后电机失能，悬空的手臂会失去支撑。
#
# 用法：set_fsm.py [<ip>:<port>] [namespace]

import sys
import time

import shidou


def main():
    address = sys.argv[1] if len(sys.argv) > 1 else "192.168.168.168:7447"
    namespace = sys.argv[2] if len(sys.argv) > 2 else "robot168"

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=address, namespace=namespace))
    if not robot.ready:
        print(robot.last_error())
        return 1

    # 先查状态，已经在 ENABLED 就不再重复使能。
    state = robot.get_state()
    print("fsm_state={}".format(state.fsm_state))

    # 先进入 ENABLED，再 stop() 回到失能，完成一次往返。
    if state.fsm_state != "ENABLED":
        robot.enable()
        print("fsm_state={}".format(robot.fsm_state))
        # 留 1 秒观察阻尼保持，再退回失能。
        time.sleep(1.0)

    # stop 立即失能；示例到此结束。
    robot.stop()
    print("fsm_state={}".format(robot.fsm_state))
    return 0


if __name__ == "__main__":
    sys.exit(main())
