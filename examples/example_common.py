"""示例共用 harness：operator 契约在本语言的实现。

示例对 operator 承诺的面在这里集中定义：`-h/--help` 的用法文本（示例自身的参数在
前，尾部两个参数在后，含缺省值）、地址与 namespace 的解析、生命周期骨架（日志初始化
→ Robot 构造 → ready 失败时一行 [FAIL] 并退出 1）、判定前缀词汇与退出码积累，以及等
待一次回车。

这些条款的取值不在这里手写：下方生成区是两语言共用的 operator 契约条款真源的渲染
结果（真源不随示例发布），手改会被视为"派生副本过期"。退出码口径与放弃语义是行为
而非取值，两语言各自实现、按同一份真源核对：0 = 全部判定通过（含操作者放弃）；
1 = 有判定失败或非法调用。

每个示例只声明一次自己的参数（见 PositionalArg）：解析与 --help 用法文本都由同一份
声明派生，不另写用法字符串。main 里只剩业务动作：run() 就绪后示例只负责下发与判定，
判定用 Verdicts 输出，退出码由判定积累，示例不自己拼退出码。

会话随句柄走：示例不显式 close()（进程退出即回收），收尾的 shidou.shutdown() 是保留
写法（已无操作）；它的位置仍由各示例自己掌握（有的早退路径不调，见各示例）。

需要人确认后才动作的示例（set_fsm / pushrod_control / chassis_control）复用本文件的
wait_enter，使"回车 = 继续、stdin 关闭或 Ctrl+C = 放弃"只有一处定义。各示例与本文件
同目录，直接 `import example_common as common` 即可。

用法文本用英文，与 C++ 侧逐字一致（两侧由同一份条款真源渲染）；各示例的运行时输出
同样用英文（同一套前缀与词汇），注释用中文。
"""

import argparse
import os
import sys

import shidou

# >>> generated example-contract clauses: do not edit by hand, rendered from the operator-contract clause truth
# 以下取值由两语言共用的 operator 契约条款真源渲染而来；手改无效，会被视为“派生副本过期”。
DEFAULT_ROBOT_ADDRESS = "192.168.168.168:7447"
DEFAULT_NAMESPACE = "robot168"

# 用法尾部的两个位置参数：(token, 说明行, 缺省值)，顺序即用法文本顺序。
USAGE_TAIL = (
    ("[<ip>:<port>]", "robot-side zenoh bridge address", "192.168.168.168:7447"),
    ("[namespace]", "keyexpr prefix matching the bridge; \"\" when the bridge has none", "robot168"),
)

# 判定前缀词汇：前缀含其后的空格，紧接判定消息。
VERDICT_PASS = "[PASS] "
VERDICT_FAIL = "[FAIL] "
VERDICT_WARN = "[WARN] "
VERDICT_INFO = "[INFO] "
# <<< generated example-contract clauses


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


class ArgError(Exception):
    """非法参数：消息就是 [FAIL] 那一行的原因。"""


def parse_double(name, expected, text):
    """数字位置参数共用的解析：非数字是非法调用，不做静默回退。

    name 出现在失败原因里（"bad <name>: <text> (expected <expected>)"），与 C++ 侧
    example::ParseDouble 同形；返回解析结果，由声明方的闭包自己接走（见 PositionalArg）。
    """
    try:
        return float(text)
    except ValueError:
        raise ArgError("bad {}: {} (expected {})".format(name, text, expected))


class PositionalArg:
    """一个可选的位置参数：示例只声明一次，解析与 --help 用法文本都从这份声明派生。

    metavar      用法文本里的占位名，例如 "target_mm"
    default_text 不填时使用的文本；与显式取值走同一条 parse 路径，缺省值只写在这里
    help         --help 说明行（英文，与示例的运行时输出一致）
    parse        解析并校验文本，非法时抛 ArgError；解析结果由声明方的闭包自己接走
                 （与 C++ 侧 parse 写捕获的输出参数同形），run() 只驱动它做校验
    """

    def __init__(self, metavar, default_text, help, parse):
        self.metavar = metavar
        self.default_text = default_text
        self.help = help
        self.parse = parse


class Verdicts:
    """判定输出与退出码积累：判定前缀取自生成区的词汇表。

    任一次 fail() 之后 failed 恒为真，run() 据此返回 1；示例不自己返回失败码。
    """

    def __init__(self):
        self._failed = False  # 出现过 [FAIL]：粘住，不因后续判定通过而清零

    @property
    def failed(self):
        return self._failed

    def _line(self, prefix, message):
        """前缀 + 一行消息，四个判定方法共用（与 C++ 侧那份定义同形）。"""
        print(prefix + message)

    def pass_(self, message):
        """对应 C++ 的 Pass（pass 是 Python 关键字）。"""
        self._line(VERDICT_PASS, message)

    def fail(self, message):
        self._failed = True
        self._line(VERDICT_FAIL, message)

    def warn(self, message):
        self._line(VERDICT_WARN, message)

    def info(self, message):
        self._line(VERDICT_INFO, message)


def print_help_line(token, help_text, default_text, width):
    """--help 说明行：占位名对齐后接说明与缺省值。"""
    print("{}  {} (default {})".format(token.ljust(width), help_text, default_text))


def print_usage(program, args):
    """用法文本：示例自身的参数按声明顺序在前，尾部两个参数按生成区的顺序在后。"""
    own = ["[{}]".format(arg.metavar) for arg in args]
    tail = [token for token, _help, _default in USAGE_TAIL]
    print("usage: {}{}".format(program, "".join(" " + token for token in own + tail)))
    print()
    width = max(len(token) for token in own + tail)
    for arg, token in zip(args, own):
        print_help_line(token, arg.help, arg.default_text, width)
    for token, help_text, default_text in USAGE_TAIL:
        print_help_line(token, help_text, default_text, width)


def program_name(argv0):
    """--help 的用法行只写脚本名：argv[0] 带目录与 .py，去掉后与 C++ 侧的用法文本一致
    （C++ 侧同样去掉目录与 .exe）。"""
    name = os.path.basename(argv0 or "")
    if name.endswith(".py"):
        name = name[:-3]
    return name


class _ContractParser(argparse.ArgumentParser):
    """把 argparse 的报错口径改成契约的：一行 [FAIL] 加退出 1，而不是 argparse 默认的 2。"""

    def error(self, message):
        print("[FAIL] {}".format(message))
        raise SystemExit(1)


def _build_parser(program, args):
    """位置参数的收集：示例自己的参数按声明顺序，随后是地址与 namespace。

    这里只按声明收集文本（含缺省文本），值与校验交给声明里的 parse，使显式取值与缺省值
    走同一条路径；示例自身参数的用法文本由 print_usage 从同一份声明渲染。
    """
    parser = _ContractParser(prog=program, add_help=False)
    for arg in args:
        parser.add_argument(arg.metavar, nargs="?", default=arg.default_text,
                            metavar=arg.metavar)
    parser.add_argument("address", nargs="?", default=DEFAULT_ROBOT_ADDRESS,
                        metavar="<ip>:<port>")
    parser.add_argument("namespace", nargs="?", default=DEFAULT_NAMESPACE)
    return parser


def run(args, body):
    """示例的契约外壳：--help → 参数解析 → 日志初始化 → Robot 构造 → ready 门禁，
    就绪后调用 body。body 只写业务动作；判定用 Verdicts，退出码由判定积累。
    shidou.shutdown()（保留写法，句柄自持会话后已无操作）的位置仍由各示例自己掌握。

    body 收到 harness 实际使用的配置（示例若要打印自己连的地址就取它）与契约对象；
    示例自己的位置参数由解析闭包自己接走（与 C++ 侧的 [&value] 捕获同形），三个参数
    与 C++ 侧 body(robot, cfg, verdicts) 逐项对齐。
    """
    program = program_name(sys.argv[0])
    argv = sys.argv[1:]
    # --help / -h 先于一切：不初始化日志、不构造 Robot。
    for token in argv:
        if token == "--help" or token == "-h":
            print_usage(program, args)
            return 0

    options, extra = _build_parser(program, args).parse_known_args(argv)
    if extra:
        # 多出的位置参数即非法调用，与 C++ harness 同一句话。
        print("[FAIL] unexpected argument: {} (see --help)".format(extra[0]))
        return 1
    for arg in args:
        # 解析与校验走声明里的 parse（缺省文本与显式取值同一条路径）；结果由声明方的
        # 闭包接走，这里只驱动校验。
        try:
            arg.parse(getattr(options, arg.metavar))
        except ArgError as error:
            print("[FAIL] {}".format(error))
            return 1

    verdicts = Verdicts()
    config = shidou.Config(robot_address=options.address, ns=options.namespace)
    shidou.init_logging("info")

    robot = shidou.Robot(config)
    if not robot.ready:
        verdicts.fail(robot.last_error())
        return 1
    body(robot, config, verdicts)
    return 1 if verdicts.failed else 0
