# 底盘示例：轮组正转 2 秒、停 1.5 秒、反转 2 秒，100 Hz 持续下发，最后切回 ENABLED。
# 轮组只在模式态生效，且必须持续收到目标：机器人端 500 ms 收不到新帧就把轮速归零。
#
# 流程：使能 → 切 PP → 逐段持续下发 → 补零速 → 切回 ENABLED。
# 注意：示例会驱动底盘，运行前留出空间并确认急停可用。
#
# 用法：chassis_control.py [<ip>:<port>] [namespace]

import sys
import time

import shidou

PERIOD_S = 0.01        # 目标下发周期：10 ms = 100 Hz
WHEEL_IDS = [18, 19]   # 轮毂电机号（示例值，按实机核对）
MAX_CURRENT = 5.0      # 每轮电流上限 A（示例值）
SPEED = 0.5            # 轮速幅值 rad/s（示例值）

# 动作段：(标签, 目标速度 rad/s, 保持时长 s)。
SEGMENTS = [
    ("forward", SPEED, 2.0),
    ("stop", 0.0, 1.5),
    ("reverse", -SPEED, 2.0),
]


def send_wheels(robot, speed):
    """下发一帧轮速（rad/s）：各轮同速、电流上限取示例值；顶杆字段留空 = 本帧不动顶杆。"""
    return robot.send_body(wheel_ids=WHEEL_IDS,
                           velocities=[speed] * len(WHEEL_IDS),
                           max_currents=[MAX_CURRENT] * len(WHEEL_IDS))


def publish_loop(robot, speed, seconds):
    """以固定周期持续下发同一速度，持续 seconds 秒；任一次下发失败立即返回 False。"""
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        if not send_wheels(robot, speed):
            return False
        time.sleep(PERIOD_S)
    return True


def main():
    address = sys.argv[1] if len(sys.argv) > 1 else "192.168.168.168:7447"
    namespace = sys.argv[2] if len(sys.argv) > 2 else "robot168"

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=address, namespace=namespace))
    if not robot.ready:
        print(robot.last_error())
        return 1

    # 轮组只在模式态生效：先使能，再切 PP（STOP 下 set_mode 回 busy）。
    robot.enable()
    robot.set_mode(shidou.ControlMode.PP)
    print("fsm_state={}".format(robot.fsm_state))

    for label, speed, hold in SEGMENTS:
        print("segment {}: {:.2f} rad/s for {:.1f} s".format(label, speed, hold))
        if not publish_loop(robot, speed, hold):
            print("send_body: {}".format(robot.last_error()))
            return 1

    # 补发 0.5 秒零速，避免轮组带着旧目标进入 ENABLED。
    if not publish_loop(robot, 0.0, 0.5):
        print("send_body: {}".format(robot.last_error()))
        return 1

    # 回到 ENABLED。
    robot.enable()
    print("fsm_state={}".format(robot.fsm_state))
    return 0


if __name__ == "__main__":
    sys.exit(main())
