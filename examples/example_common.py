"""示例共用小工具：等一次回车。

需要人确认后才动作的示例（set_fsm / pushrod_control / chassis_control）复用
本文件，使"回车 = 继续、stdin 关闭或 Ctrl+C = 放弃"只有一处定义；各示例与本文件
同目录，直接 `import example_common as common` 即可。
"""


def wait_enter(prompt=None):
    """等一次回车：回车返回 True；stdin 关闭 / Ctrl+C / Ctrl+Z 返回 False。

    prompt 为 None 时不打印提示（调用方已打印过行动计划或操作说明）。
    返回 False 表示放弃——调用方必须在放弃路径上什么都不下发。
    """
    if prompt:
        print(prompt)
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        return False
    return True
