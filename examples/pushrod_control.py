# 顶杆示例：在 ENABLED 下把顶杆送到目标位置（mm），随后每 500 ms 打印一次位置
# 反馈，共 5 秒。一帧 send_body() 只填顶杆字段，机器人端按示例给出的轮廓速度与
# 加速度执行；示例不做到位判定。
#
# 流程：解析目标 → 构造 Robot → 必要时使能 → 下发一帧目标 → 打印 5 秒反馈。
#
# 用法：pushrod_control.py [target_mm] [<ip>:<port>] [namespace]

import sys
import time

import shidou

PUSHROD_ID = 17                # 顶杆电机号（示例值，按机器人实际配置修改）
PROFILE_VELOCITY = 20.0        # 轮廓速度 mm/s（示例值）
PROFILE_ACCELERATION = 50.0    # 轮廓加速度 mm/s²（示例值）
PRINT_PERIOD_S = 0.5           # 打印间隔 s
FRAMES = 10                    # 打印帧数；10 × 0.5 s = 5 秒


def read_position(robot, motor_id):
    """从遥测反馈里按电机号取位置；没有样本或样本中缺该电机时返回 None。"""
    feedback = robot.last_feedback()
    if feedback is None:
        return None
    for index, motor in enumerate(feedback.motor_ids):
        if motor == motor_id and index < len(feedback.position):
            return feedback.position[index]
    return None


def main():
    # 缺省 0 = 零点；非法目标不能静默当 0 下发。
    target_text = sys.argv[1] if len(sys.argv) > 1 else "0"
    try:
        target_mm = float(target_text)
    except ValueError:
        print("bad target_mm: {} (expected a number in mm)".format(target_text))
        return 1
    address = sys.argv[2] if len(sys.argv) > 2 else "192.168.168.168:7447"
    namespace = sys.argv[3] if len(sys.argv) > 3 else "robot168"

    shidou.init_logging("info")
    robot = shidou.Robot(shidou.Config(robot_address=address, namespace=namespace))
    if not robot.ready:
        print(robot.last_error())
        return 1

    state = robot.get_state()
    print("fsm_state={}".format(state.fsm_state))

    # 顶杆在 ENABLED 下即可接收目标位置参数，示例不下发模式切换。
    if state.fsm_state != "ENABLED":
        robot.enable()
        print("fsm_state={}".format(robot.fsm_state))

    # 只填顶杆字段；轮组字段留空 = 本帧不动轮组。
    if not robot.send_body(pushrod_id=PUSHROD_ID, position=target_mm,
                           velocity=PROFILE_VELOCITY,
                           acceleration=PROFILE_ACCELERATION):
        print("send_body: {}".format(robot.last_error()))
        return 1
    print("pushrod {} -> {:.2f} mm (profile {:.0f} mm/s, {:.0f} mm/s^2)".format(
        PUSHROD_ID, target_mm, PROFILE_VELOCITY, PROFILE_ACCELERATION))

    # 每 500 ms 取一次反馈；顶杆位置随 joint_states 到达，按 motor_id 取。
    for _ in range(FRAMES):
        time.sleep(PRINT_PERIOD_S)
        position = read_position(robot, PUSHROD_ID)
        if position is None:
            print("  no position for pushrod motor {} yet".format(PUSHROD_ID))
        else:
            print("  pushrod position={:.2f} mm".format(position))
    return 0


if __name__ == "__main__":
    sys.exit(main())
