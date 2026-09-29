# ShiDou SDK Python 绑定

在 Linux 上用 Python 控制 ShiDou 机器人的客户端。底层与 C++ SDK 是同一份实现
（`libshidou_py.so`，ctypes 调用，不依赖 CPython 版本），能力一致：状态读取、
使能/停止、模式切换、五种目标流、轨迹上传、三类回调。控制语义全部在 C++ 侧，
绑定不新增行为。

本仓库是**预编译载荷**：`.so` 已经构建好，安装不需要编译器，也不需要联网。

## 目录结构

```
shidou/                    # Python 包；_lib/<platform>/ 下是预编译的 .so
  _lib/linux/              # x86_64：libshidou_py.so + libzenohc.so
  _lib/linux-arm64/        # aarch64：同上
examples/                  # 控制示例（get_state / get_joint / set_fsm /
                           # mit_control / pushrod_control / chassis_control）
tests/smoke.py             # 冒烟：不连机器人（peer 模式自建会话），19 项检查
pyproject.toml             # 打包元数据（pip 用）
```

## 安装

需要 Python ≥ 3.8。推荐就地安装本仓库（拉取新版本后无需重装）：

```bash
git clone https://github.com/ZZSZSZSZZ/shidou_sdk_python.git
cd shidou_sdk_python
pip3 install -e .
```

也可以装进 site-packages（每次更新后要重装），或者完全不装：

```bash
pip3 install .
# 或者：零依赖，不装任何东西，直接指定包路径
PYTHONPATH=/path/to/shidou_sdk_python python3 examples/get_state.py
```

`pip3 install` 走 PEP 517/660，构建隔离会从 PyPI 取一次 `setuptools`（离线时用上面的
`PYTHONPATH` 方式，或 `pip3 install --no-build-isolation -e .`）。

**pip 要够新**：Ubuntu 22.04 自带的 pip 22.0.2 与当前 setuptools 不兼容（实测），
pip 24.2 可用（实测）。两种失败方式都不指向真正原因：

- `pip3 install -e .` 报 `build backend is missing the 'build_editable' hook`（实际是
  旧 pip 判断后端能力的方式失效）；
- `pip3 install .` 更隐蔽：**成功退出**，但装出来的是没有元数据的 `UNKNOWN 0.0.0`，
  `import shidou` 依然失败。

所以先升级 pip，再安装：

```bash
pip3 install -U pip        # 或 python3 -m pip install --user -U pip
pip3 install -e .
```

运行环境与载荷一致：Linux x86_64 / aarch64，glibc ≥ 2.35（Ubuntu 22.04 构建）。
其他平台 import 时会给出明确的 `ImportError`。

## 最小示例

```python
import shidou

shidou.init_logging("info")
# ns 必须与机器人侧桥配置的 namespace 完全一致；桥未启用时显式传 ""
config = shidou.Config(robot_address="192.168.168.168:7447", ns="robot168")

robot = shidou.Robot(config)        # 真机上不要用 with、也不要 close()，见下
if not robot.ready:
    raise SystemExit("connect failed: " + robot.last_error())

robot.enable()                                  # ENABLED 握手（阻塞）
robot.set_mode(shidou.ControlMode.POSITION)     # 模式命令要求已 ENABLED
robot.send_position(motor_ids=[1, 2], positions=[0.1, 0.1])   # 流式目标

state = robot.get_state()                       # 阻塞，失败抛 ShidouError
print(state.fsm_state, state.motor_ids, state.positions)

shidou.shutdown()                               # 进程收尾一次
```

`Config` 的字段传 `None` 表示沿用 SDK 默认值（`robot_address`
`192.168.168.168:7447`、`mode` `"client"`、`log_level` `"info"`）；而 `ns` 的 `""`
是有意义的「无前缀」，两者不要混淆。

遥测与回调：

```python
feedback = robot.last_feedback()     # None = 还没有样本
print(robot.feedback_age_ms, robot.feedback_seq, robot.fsm_state)

def on_feedback(fb):                 # zenoh 会话线程，禁止阻塞
    print(fb.motor_ids, fb.position, fb.enabled)

robot.set_feedback_callback(on_feedback)
robot.set_fsm_callback(lambda state: print("fsm:", state))
robot.set_stale_callback(lambda age_ms: print("stale", age_ms), threshold_ms=1000)
robot.set_feedback_callback(None)    # 传 None 注销
```

| Python | C++（`shidou_sdk` 仓） | 说明 |
|---|---|---|
| `shidou.Config(...)` | `comm::ZenohConfig` | 会话与命名空间配置 |
| `shidou.Robot(config)`、`close()`、`with` | `robot::Robot`、`~Robot` | 句柄；关闭后不可再用 |
| `robot.ready`、`robot.last_error()` | `Ready()`、`LastError()` | 错误文本**粘性** |
| `enable` / `stop` / `set_mode` / `upload_trajectory` / `set_namespace` | 同名 | 阻塞；失败抛 `ShidouError` |
| `send_mit` / `send_csp` / `send_position` / `send_gripper` / `send_body` | 同名 `Send*` | 非阻塞、线程安全；返回 `True/False` |
| `get_state()` | `GetRobotState` | 阻塞；失败抛 `ShidouError` |
| `last_feedback()` / `feedback_age_ms` / `feedback_seq` / `fsm_state` | 同名 | 遥测快照 |
| `set_feedback_callback` / `set_fsm_callback` / `set_stale_callback` | `Set*Callback` | 传 `None` 注销 |
| `shidou.init_logging(level)` / `shidou.shutdown()` | `InitLogging` / `ZenohFactory::Shutdown` | 进程级，各一次 |
| `shidou.MITWaypoint` / `shidou.CSPWaypoint` | `msg::MITWaypoint` / `CSPWaypoint` | 轨迹路径点 |

## 运行示例

运行前机器人侧 zenoh 桥须已在监听，且 namespace 与桥配置一致。示例直接运行
（`pip3 install -e .` 之后），或加 `PYTHONPATH=/path/to/shidou_sdk_python` 前缀：

```bash
python3 examples/get_state.py 192.168.168.168:7447 robot168
```

共同约定：

- 参数顺序固定：示例自身的参数在前，`[<ip>:<port>]` 与 `[namespace]` 在后，两者均可省略。
- 缺省 `robot_address` 为 `192.168.168.168:7447`，缺省 namespace 为 `robot168`。namespace
  与桥不一致时示例等不到样本并报 `[FAIL]`；桥未启用 namespace 时传空串：
  `python3 examples/set_fsm.py <ip>:<port> ""`。
- `<ip>:<port>` 为机器人侧 zenoh 桥的监听地址与端口。
- 输出行前缀统一：`[PASS]` 为判定通过，`[FAIL]` 为判定失败，另有 `[WARN]` / `[INFO]` 提示；
  退出码 0 表示示例的所有判定均通过。
- 每个示例都带 `--help`，可以先看用法再决定是否运行。

| 示例 | 作用 | 用法 | 回车确认 | 预期输出 |
| --- | --- | --- | --- | --- |
| `get_state` | 一次取回机器人状态并逐字段打印 | `get_state [<ip>:<port>] [namespace]` | 否 | 状态字段与各电机位置，末行 `[PASS] get_state completed` |
| `get_joint` | 订阅关节遥测，每 500 ms 打印一帧快照 | `get_joint [seconds] [<ip>:<port>] [namespace]` | 否 | 周期快照与速率统计，末行 `[PASS] get_joint completed`；样本断流时快照带 STALE 并出现 `[WARN]` |
| `set_fsm` | 在 STOP 与 ENABLED 之间切换 | `set_fsm [<ip>:<port>] [namespace]` | 每次回车切换一次，Ctrl+C 退出 | 每次切换打印 `[PASS] <动作> -> fsm_state=<新状态>` |
| `mit_control` | 对每臂末端电机做 MIT 位置扫动：查询状态 → 必要时 Enable → `set_mode(POSITION)` → 以 100 Hz 下发 → 切回 ENABLED | `mit_control [<ip>:<port>] [namespace]` | 否 | 各段位置/速度打印，末行 `[PASS] Enable -> fsm_state=ENABLED` |
| `pushrod_control` | 使能模式下把顶杆送到目标位置（mm），并按反馈判定到位 | `pushrod_control [target_mm] [<ip>:<port>] [namespace]` | 是（先打印计划） | `[PASS] pushrod <起点> mm -> <目标> mm (measured <实测> mm)` |
| `chassis_control` | 驱动轮组正转 → 停 → 反转，每段按容差判定，末尾停发一段验证 500 ms 死区 | `chassis_control [<ip>:<port>] [namespace]` | 是（先打印计划） | 各段轮速打印，结束切回 ENABLED 打印 `[PASS] Enable -> fsm_state=ENABLED`；轮速未达容差时追加 `[FAIL] wheel velocities off target` |

补充说明：

- `get_joint` 的 `[seconds]` 为 0 或负数表示一直运行到进程被杀，适合在机器人旁长期挂监控。
- `pushrod_control` 的 `[target_mm]` 缺省 0（即零点）；顶杆反馈随 `joint_states` 到达，示例按
  `motor_id` 从中取顶杆位置，因此顶杆未上线时会报 `[FAIL] no position for pushrod motor ...`。
- `chassis_control` 在段内按固定周期持续下发轮速：机器人侧 500 ms 收不到新消息就把轮速归零，
  发一帧就等不会产生运动。
- 需要回车确认的两个示例在动作前打印动作计划；stdin 关闭或 Ctrl+C 视为放弃，不发送任何目标。
- `mit_control` / `pushrod_control` / `chassis_control` 会下发模式切换与运动目标，确认计划前请
  确保机器人周围安全。

## 契约与陷阱

| 现象 | 原因 | 处理 |
|---|---|---|
| `ImportError: libshidou_py.so not found` | 包不完整，或 `PYTHONPATH` 未指向仓库根 | 用 `pip3 install -e .` 安装，或把 `PYTHONPATH` 指到仓库根 |
| `ImportError: ... this machine reports ...` | 本机架构在 `shidou/_lib/` 里没有对应载荷 | 见提示里列出的已发布架构；`SHIDOU_LIB` 可指定别处的 `.so` |
| `ImportError: ... ABI version` | `.so` 与 `shidou` 包不配套 | 用同一个 tag 的整棵树（不要混用两个版本的目录） |
| `pip3 install` 报 `build backend is missing the 'build_editable' hook`，或装完 `import shidou` 仍失败（装的是 `UNKNOWN 0.0.0`） | pip 太旧（Ubuntu 22.04 自带 22.0.2）与当前 setuptools 不兼容 | `pip3 install -U pip` 后重装，或改用 `PYTHONPATH` |
| 连上了但静默无数据 | `ns` 与桥的 `namespace` 不一致 | 与机器人侧对账（板端 `bridge_config.json5`） |
| 第二个 `Robot` 总是 `ready == False` | 进程级单例：首个 `Robot` 已定下会话 | 多机器人用 `set_namespace` 切换，不要建多个配置不同的 `Robot` |
| 调用长时间不返回 | 阻塞方法要等满超时（enable/stop 5 s、mode 2 s、get_state 5 s、轨迹 10 s） | 传 `timeout_ms`；这些方法只能从非回调线程调用 |
| 回调里调用阻塞方法后卡住 | 回调跑在 zenoh 会话线程 / 遥测看门狗线程上 | 回调内只做拷贝与标记，动作交给主线程 |
| `ShidouError` 里是上一次的失败文本 | `last_error()` 与 C++ 一样是**粘性**的，成功不清空；`send_*` 失败（未就绪/发布失败）本身不写错误文本 | 判断成败请看返回值/异常，而不是错误文本是否为空 |
| `send_*` 报数组长度不匹配 | 语义要求数组与 `motor_ids`（或 `joint_id`）**逐位对齐**，封装层会拦下 | 可选数组要么省略，要么与主导数组等长 |
| `fb.enabled` / `fb.online` 是 `0/1` 整数列表 | 线上是每电机一字节，不是 `bool` 向量 | 按整数用，或自行 `bool()` |
| 回调里的异常没有冒出来 | ctypes 回调内异常只会打到 stderr | 回调体内自己 `try/except` |
| 刚注销/替换的回调又跑了一次 | 回调在锁外调用，注销不等已在跑的调用结束 | 回调里只做幂等的事；别在回调里持有会被注销推翻的状态 |
| 轨迹点传错了类型（MIT 的当 CSP 用） | 两种路径点形状相同，按位置传很容易错 | `upload_trajectory` 的路径点一律用关键字传（`mit_points=` / `csp_points=`） |
| 用 `last_feedback()` 的电机顺序去填轨迹点或目标 | 服务的电机顺序与 `joint_states` 的顺序**可以不同**（真机实测 `get_state` 给 `[14, 1, 2, …, 13]`，反馈给 `[1, 2, …, 14]`） | 两个来源各自内部按索引对齐，跨来源一律**按 `motor_ids` 对齐**；轨迹路点按规定用 `get_state` 的顺序 |

回调与线程模型（与 C++ 相同）：`feedback`、`fsm_state` 回调在 **zenoh 会话线程**上触发；
`stale` 回调在**遥测看门狗线程**上触发（每 50 ms 轮询一次）；ctypes 会在进入回调时自动
获取 GIL，所以回调里可以正常写 Python，只是不能阻塞。

`get_state()` 与 `last_feedback()` 返回的对象是从 C 侧借用视图**立即拷贝**出来的，拷贝与
取视图这两步由句柄内的一把锁串起来（两者各一把锁，互不等待）：多线程并发调用安全，长
`get_state` 不会拖住遥测读取。句柄的其余方法沿用 C++ 的线程契约。

**真机例外**：机器人上 `joint_states` 持续在发，而**销毁一个带流量的句柄有已知缺陷**——
`Robot` 的 `telemetry_` 先于两个订阅者析构（C++ 侧既有问题，`~Robot()` 只停看门狗，不与
zenoh 会话线程同步），崩溃点是 `close()` 而不是绑定。用 Python 上真机时**不要用 `with`、
也不要调 `close()`**（仓库里的示例都遵守这一条），让进程自己结束：`Robot` 没有
`__del__`，不调就不销毁句柄，窗口不存在（代价是 zenoh 会话不做优雅关闭，桥端按租约超时
回收，对控制无影响）。需要长驻进程或反复建句柄的用法，等该缺陷修复后再用。

## 验证

`tests/smoke.py` 覆盖句柄生命周期、遥测哨兵值、服务失败路径、数组长度校验、轨迹编组、
粘性错误、并发读 `get_state`/`last_feedback`、回调注册/注销、关闭后拒绝调用等 19 项检查。
它用 peer 模式自建会话，**不需要机器人**：

```bash
python3 tests/smoke.py
```

载荷本身在 CI 里按架构各验一遍：`readelf` 断言 `libshidou_py.so` 的 RUNPATH 为
`$ORIGIN`、`pip install -e .` 后从仓库外 import、跑冒烟、构建 wheel 并断言其中含两个
架构的 `.so`（每架构的 `.so` 只在对应架构的 runner 上做真实加载）。

## 许可

本仓库以 BSD-3-Clause 发布（见 LICENSE）。第三方依赖许可见 [licenses/](licenses/)：
zenoh-c / zenoh-cpp 为 Apache-2.0 或 EPL-2.0 双许可，spdlog 为 MIT——三者的代码都静态
进了 `libshidou_py.so`。

C++ SDK（协议与字段定义、`robot_ops` 板端工具）：
<https://github.com/ZZSZSZSZZ/shidou_sdk>。
