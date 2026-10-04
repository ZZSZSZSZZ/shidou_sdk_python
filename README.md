# ShiDou SDK Python 绑定

使用 Eclipse Zenoh 为底层通信协议、以 ctypes 调用 C++ 实现的 Python 机器人客户端。
预编译载荷：安装不需要编译器，也不需要联网。

## 目录结构

```
shidou/                    # Python 包；_lib/<platform>/ 下是随包发布的 .so
  _lib/linux/              # x86_64（Ubuntu 22.04，glibc 2.35）
  _lib/linux-arm64/        # aarch64（同上）
examples/                  # 控制示例（get_state / get_joint / set_fsm /
                           # mit_control / pushrod_control / chassis_control）
pyproject.toml             # 打包元数据（pip 用）
```

## 安装

需要 Python ≥ 3.8。推荐就地安装（拉取新版本后无需重装）：

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

安装走 PEP 517，构建隔离会从 PyPI 取一次 `setuptools`（离线时用上面的 `PYTHONPATH`
方式，或 `pip3 install --no-build-isolation -e .`）。

**pip 要够新**：Ubuntu 22.04 自带的 pip 22.0.2 与当前 setuptools 不兼容（实测），
pip 24.2 可用（实测）。两种失败方式都不指向真正原因：可编辑安装报 `build backend is
missing the 'build_editable' hook`；非可编辑安装更隐蔽——**成功退出**，但装出来的是
没有元数据的 `UNKNOWN 0.0.0`，`import shidou` 依然失败。先 `pip3 install -U pip` 再安装。

## 运行示例

安装后可直接运行示例（未安装则加 `PYTHONPATH=/path/to/shidou_sdk_python` 前缀）。
运行前机器人侧 zenoh 桥须已在监听，且 namespace 与桥配置一致。

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

## 在你的工程中使用

```python
import shidou

shidou.init_logging("info")
# ns 必须与机器人侧桥配置的 namespace 完全一致；桥未启用时显式传 ""
config = shidou.Config(robot_address="192.168.168.168:7447", ns="robot168")

robot = shidou.Robot(config)                    # 句柄生命周期见「注意事项」
robot.enable()                                  # ENABLED 握手（阻塞）
robot.set_mode(shidou.ControlMode.POSITION)     # 模式命令要求已 ENABLED
robot.send_position(motor_ids=[1, 2], positions=[0.1, 0.1])

state = robot.get_state()                       # 阻塞，失败抛 ShidouError
print(state.fsm_state, state.motor_ids, state.positions)

shidou.shutdown()                               # 进程收尾一次
```

接口与 C++ SDK 同名同义：`send_mit` / `send_csp` / `send_position` / `send_gripper` /
`send_body` 非阻塞、线程安全、返回 `True/False`；`enable` / `stop` / `set_mode` /
`upload_trajectory` / `set_namespace` 阻塞、失败抛 `ShidouError`；遥测是
`last_feedback()` 与 `feedback_age_ms` / `feedback_seq` / `fsm_state` 快照；
回调是 `set_feedback_callback` / `set_fsm_callback` / `set_stale_callback`（传 `None`
注销）；轨迹路点用 `shidou.MITWaypoint` / `shidou.CSPWaypoint`。`Config` 的字段传
`None` 表示沿用 SDK 默认值。

回调在 zenoh 会话线程上触发，禁止阻塞（ctypes 进入回调时自动取 GIL，可以正常写 Python）：

```python
def on_feedback(fb):
    print(fb.motor_ids, fb.position, fb.enabled)

robot.set_feedback_callback(on_feedback)
robot.set_stale_callback(lambda age_ms: print("stale", age_ms), threshold_ms=1000)
```

## 注意事项

- 预编译载荷均为 Ubuntu 22.04 构建（glibc ≥ 2.35）：`shidou/_lib/` 下的 `linux` 与
  `linux-arm64` 一架构一目录、不可混用；本机架构没有对应载荷、或包不完整时，import
  会给出明确的 `ImportError`（`SHIDOU_LIB` 可指定别处的 `.so`）
- **句柄生命周期**：进程退出时句柄随之释放（仓库里的示例都不显式 `close()`，需要提前
  释放时可正常 `close()` 或用 `with`）。v1.0.1 起销毁带流量的句柄已安全；**v1.0.0 及
  更早的载荷**仍有析构崩溃缺陷，请继续回避（不调 `close()`，让进程自己结束）
- `last_error()` 是**粘性**的（成功不清空）；`send_*` 失败本身不写错误文本。判断成败
  请看返回值/异常，而不是错误文本是否为空
- `Robot` 是进程级单例：第二个配置不同的 `Robot` 会一直是 `ready == False`。多机器人
  用 `set_namespace` 切换，不要建多个 `Robot`
- 阻塞方法要等满超时（enable/stop 5 s、mode 2 s、`get_state` 5 s、轨迹 10 s），且只能
  从非回调线程调用；回调里只做拷贝与标记，动作交给主线程
- `send_*` 与轨迹路点的数组要和 `motor_ids`（或 `joint_id`）逐位对齐，可选数组要么
  省略、要么等长；两种路点形状相同，一律用关键字传（`mit_points=` / `csp_points=`）
- `get_state()` 与 `last_feedback()` 的电机顺序**可以不同**（真机实测一个给 `[14, 1, …,
  13]`、另一个给 `[1, 2, …, 14]`），跨来源一律按 `motor_ids` 对齐
- 版本对应 tag；Release 页提供该版本树的 tar.gz / zip 归档

## 许可

本仓库以 BSD-3-Clause 发布（见 LICENSE）。第三方依赖许可见 [licenses/](licenses/)：
zenoh-c / zenoh-cpp 为 Apache-2.0 或 EPL-2.0 双许可，spdlog 为 MIT——三者的代码都静态
进了 `libshidou_py.so`。协议与字段定义、`robot_ops` 板端工具见 C++ SDK：
<https://github.com/ZZSZSZSZZ/shidou_sdk>。
