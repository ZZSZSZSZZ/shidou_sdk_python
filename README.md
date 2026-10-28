# ShiDou SDK Python 绑定

使用 Eclipse Zenoh 为底层通信协议、以 ctypes 调用 C++ 实现的 Python 机器人客户端。

## 目录结构

```
shidou/                    # Python 包；_lib/<platform>/ 下是随包的预编译库
  _lib/linux/              # x86_64（Ubuntu 22.04，glibc 2.35）：libshidou_py.so + libzenohc.so
  _lib/linux-arm64/        # aarch64（同上）：同上
  _lib/win/                # Windows x64：shidou_py.dll + zenohc.dll
  _lib/win-arm64/          # Windows ARM64：同上
examples/                  # 控制示例（get_state / get_joint / set_fsm / mit_control /
                           # pp_control / pushrod_control / chassis_control）
pyproject.toml             # 打包元数据（pip 用）
GETTING_STARTED.md         # 快速上手：按顺序跑通七个示例
```

## 安装

需要 Python ≥ 3.8。推荐就地安装（拉取新版本后无需重装）：

```bash
git clone https://github.com/ZZSZSZSZZ/shidou_sdk_python.git
cd shidou_sdk_python
pip3 install -e .
```

Windows 上是同一条路（x86_64 与 ARM64 都一样），只是 `python3` / `pip3` 换成
`python` / `pip`（下同）：用 `python -m pip install -e .`、
`python examples/get_state.py …`。`shidou/_lib/win/` 或 `shidou/_lib/win-arm64/`
会被自动选中，装载层把 DLL 搜索路径指向库所在目录（对应 Linux 的 `$ORIGIN`），
import 不需要设置任何环境变量。

**Windows ARM64 需要 CPython ≥ 3.11**：官方 Windows ARM64 安装包自 3.11 起提供，
更早的 CPython 没有官方 arm64 安装包，3.10 及更早的 ARM64 Windows 无法使用本载荷。
`pyproject.toml` 的 `requires-python = ">=3.8"` 保持不变（该下限对 Linux 与
Windows x64 成立），win-arm64 的 ≥ 3.11 是这个平台的准入条件。

也可以装进 site-packages（每次更新后要重装），或者完全不装：

```bash
pip3 install .
# 或者：零依赖，不装任何东西，直接指定包路径
PYTHONPATH=/path/to/shidou_sdk_python python3 examples/get_state.py
```

安装走 PEP 517，构建隔离会从 PyPI 取一次 `setuptools`（离线时用上面的 `PYTHONPATH`
方式，或 `pip3 install --no-build-isolation -e .`）。

**pip 要够新**：Ubuntu 22.04 自带的 pip 22.0.2 与当前 setuptools 不兼容（实测），
pip 24.2 可用（实测）。两种失败方式都不指向真正原因：可编辑安装报 `build backend is
missing the 'build_editable' hook`；非可编辑安装更隐蔽——**成功退出**，但装出来的是
没有元数据的 `UNKNOWN 0.0.0`，`import shidou` 依然失败。先 `pip3 install -U pip` 再安装。
Windows 上同理先 `python -m pip install -U pip`（新安装包自带的 pip 已够新，实测 Python
3.13 自带的 pip 25.0.1 直接可编辑安装通过）。

## 运行示例

安装后可直接运行示例（未安装则加 `PYTHONPATH=/path/to/shidou_sdk_python` 前缀）。
运行前机器人侧 zenoh 桥须已在监听，且 namespace 与桥配置一致。

```bash
python3 examples/get_state.py 192.168.168.168:7447 robot168
```

它只查询状态、不下发任何目标。第一次上手建议按这个顺序，前两个不会让机器人产生运动：

1. `get_state`：只查询状态，确认地址与 namespace；
2. `set_fsm`：看清 STOP 与 ENABLED 的区别，切换本身不产生运动；
3. `pp_control`：第一次运动，每条臂的末端电机走 `+0.3 rad` 再回到原位；
4. `mit_control`（仅灵足臂）/ `pushrod_control` / `chassis_control`：按机型选一个。

每一步的讲解、输出对照与常见问题见 [GETTING_STARTED.md](GETTING_STARTED.md)。

共同约定：

- 示例接受两个可选参数：`[<ip>:<port>]` 与 `[namespace]`；示例自身的参数（如
  `pushrod_control` 的 `[target_mm]`）在前。省略时使用缺省值。
- 出厂默认 `robot_address` 为 `192.168.168.168:7447`，出厂默认 namespace 为 `robot168`。namespace
  与桥不一致时示例收不到数据；桥未启用 namespace 时传空串：
  `python3 examples/set_fsm.py <ip>:<port> ""`。
- `<ip>:<port>` 为机器人侧 zenoh 桥的监听地址与端口。
- 示例不带 `--help`，只演示功能本身；参数顺序以上表与源码为准。
- 运动示例运行即下发目标，不做回车确认；运行前请确保机器人周围安全。

| 示例 | 作用 | 用法 |
| --- | --- | --- |
| `get_state` | 一次取回机器人状态并逐字段打印 | `get_state [<ip>:<port>] [namespace]` |
| `get_joint` | 每 500 ms 打印一帧关节反馈，共 10 帧、5 秒；逐电机打印位置、速度、力矩、使能、在线、报警码与温度 | `get_joint [<ip>:<port>] [namespace]` |
| `set_fsm` | 打印当前状态，演示 Enable 与 Stop | `set_fsm [<ip>:<port>] [namespace]` |
| `mit_control` | 每臂末端电机做 MIT 位置来回：当前位置 → `+0.3 rad` → 原位（各 1 秒，100 Hz） | `mit_control [<ip>:<port>] [namespace]` |
| `pp_control` | 每臂末端电机做 PP（轮廓位置）来回：当前位置 → `+0.3 rad` → 原位（每程只发一帧终点位置） | `pp_control [<ip>:<port>] [namespace]` |
| `pushrod_control` | 使能模式下把顶杆送到目标位置（mm），随后打印 5 秒位置反馈 | `pushrod_control [target_mm] [<ip>:<port>] [namespace]` |
| `chassis_control` | 轮组正转 2 秒 → 停 1.5 秒 → 反转 2 秒，结束后切回 ENABLED | `chassis_control [<ip>:<port>] [namespace]` |

补充说明：

- `pushrod_control` 的 `[target_mm]` 缺省 0（即零点）；目标只下发一帧，机器人端按
  20 mm/s、50 mm/s² 的轮廓执行。示例不对到位做判定。
- `chassis_control` 在段内按固定周期持续下发轮速：机器人侧 500 ms 收不到新消息就把轮速归零，
  发一帧就等不会产生运动。
- 关节目标只在对应的模式态（MIT / CSP / PP）生效，目标不会切换模式；模式切换用 `set_mode`
  （机器人端 `run_command` 服务；切换时机器人先停驱动、写电机模式、再重新使能，机械臂短暂
  失去力矩；STOP 下先 `enable()`）。`enable` 进入的 ENABLED 是阻尼保持，`enable(mode)` 也
  一样（机器人不读 `enable` 的模式字段）。能力按臂型裁剪：
  灵足支持 mit / csp / pp，EYOU 支持 csp / pp，不支持的模式会被拒绝（`unsupported_mode`）。
  `pp_control` 用的 PP 两族臂都支持（`mit_control` 只适用于灵足臂）。
- `mit_control` / `pp_control` / `pushrod_control` / `chassis_control` 会下发模式切换与运动目标，
  运行前请确保机器人周围安全。

## 在你的工程中使用

```python
import shidou

shidou.init_logging("info")
# namespace 必须与机器人侧桥配置完全一致；桥未启用时显式传 ""
config = shidou.Config(robot_address="192.168.168.168:7447", namespace="robot168")

robot = shidou.Robot(config)                    # 句柄生命周期见「注意事项」
robot.enable()                                  # 进入 ENABLED 阻尼保持（阻塞）
robot.set_mode(shidou.ControlMode.PP)           # 切 PP：停驱动→写电机模式→重新使能（阻塞）
robot.send_position(motor_ids=[1, 2], positions=[0.1, 0.1])   # 流式目标，PP 态才生效

state = robot.get_state()                       # 阻塞，失败抛 ShidouError
print(state.fsm_state, state.motor_ids, state.positions)

robot.close()                                   # 释放该句柄打开的会话（也可用 with）
```

接口与 C++ SDK 同名同义：`send_mit` / `send_csp` / `send_position` / `send_gripper` /
`send_body` 非阻塞、线程安全、返回 `True/False`；`enable` / `enable(mode)` / `stop` /
`set_mode` / `set_namespace` 阻塞、失败抛 `ShidouError`（`enable(mode)` 与 `enable`
是同一条使能边，机器人不读模式字段，落点仍是 ENABLED；进入模式态用 `set_mode`）；遥测是
`last_feedback()` 与 `feedback_age_ms` / `feedback_seq` / `fsm_state` 快照；
回调是 `set_feedback_callback` / `set_fsm_callback` / `set_stale_callback`（传 `None`
注销）。`Config` 的字段传 `None` 表示沿用 SDK 默认值。

回调在 zenoh 会话线程上触发，禁止阻塞（ctypes 进入回调时自动取 GIL，可以正常写 Python）：

```python
def on_feedback(fb):
    print(fb.motor_ids, fb.position, fb.enabled)

robot.set_feedback_callback(on_feedback)
robot.set_stale_callback(lambda age_ms: print("stale", age_ms), threshold_ms=1000)
```

## 注意事项

- 预编译库由 GitHub Actions 流水线构建：Linux 两条（Ubuntu 22.04，glibc ≥ 2.35）；
  Windows 两条，工具集基线不同——x64 在 windows-latest 用 VS2022（v143）构建，
  ARM64 在 windows-11-arm 原生构建，该镜像只有 VS2026 / MSVC 14.51（无 VS2022
  可用），基线无法与 x64 对齐，如实分列——`shidou/_lib/` 一架构一目录、不可混用；
  本机架构没有对应库、或包不完整时，import 会给出明确的 `ImportError`（`SHIDOU_LIB`
  可指定别处的库文件：Linux 的 `.so` 或 Windows 的 `shidou_py.dll`）
- **句柄生命周期**：每个 `Robot` 在创建时打开自己的会话（配置各自生效，同进程可以并存
  多个句柄），`close()`（含 `with` 退出）在销毁句柄时释放它，只影响自己那一份；不调
  `close()` 则进程退出时随之释放（仓库里的示例都不显式 `close()`）。`close()` 会同步
  关闭会话的传输：本地会话毫秒级，机器人地址失联时可能等数秒，因此不要从回调里调用。
  v1.0.1 起销毁带流量的句柄已安全；**v1.0.0 及更早的版本**仍有析构崩溃缺陷，请继续回避
  （不调 `close()`，让进程自己结束）
- `last_error()` 是**最近一次失败**的诊断记录（粘性，成功不清空）；异常文本是**本次
  调用**的消息，`send_*` 失败时也记录本次原因。创建失败也算一次失败：构造交回的句柄
  `ready` 为假，`last_error()` 从创建起就是失败原因；随后任何失败的调用都会把这条记录
  顶掉。判断本次成败请看返回值/异常，而不是错误文本是否为空
- 同进程建多个 `Robot` 不再是陷阱：每个句柄自带一个会话，配置各自生效（各连各的地址/
  namespace），`close()` 只释放自己那一份；`shidou.shutdown()` 保留兼容（句柄自持会话后
  已无操作，随时可调）。连不上不是异常：构造交回的句柄 `ready` 为假，`last_error()` 就是
  原因（会话打不开给会话自己的原因，通信对象建不起来给 `failed to create comm objects: …`），
  日志里另有同一原因的 error 行
- 阻塞方法要等满超时（enable/stop 5 s（`enable(mode)` 同）、set_mode 10 s、
  `get_state` 5 s），且只能从非回调线程调用；回调里只做拷贝与标记，动作交给主线程
- `send_*` 的数组要和 `motor_ids` 逐位对齐，可选数组要么省略、要么等长
- `get_state()` 与 `last_feedback()` 的电机顺序**可以不同**（真机实测一个给 `[14, 1, …,
  13]`、另一个给 `[1, 2, …, 14]`），跨来源一律按 `motor_ids` 对齐
- 版本对应 tag；Release 页提供该版本树的 tar.gz / zip 归档

## 许可

本仓库以 BSD-3-Clause 发布（见 LICENSE）。第三方依赖许可见 [licenses/](licenses/)：
zenoh-c / zenoh-cpp 为 Apache-2.0 或 EPL-2.0 双许可，spdlog 为 MIT——三者的代码都静态
进了绑定库（Linux 为 `libshidou_py.so`，Windows 为 `shidou_py.dll`）。线协议与字段定义
见 C++ SDK 的 `include/shidou/msg/`：<https://github.com/ZZSZSZSZZ/shidou_sdk>。
