# eir_loco_runtime

轻量的纯 C++ 机器人运行时，为 MuJoCo 仿真和硬件 SDK 提供统一的
`start / read / write / stop` 接口。核心不依赖 ROS 2、`ros2_control` 或 `hardware_interface`。
可选 ROS 2 层只发布状态和告警，不接收控制指令。

- **控制**：主电机的位置、速度、前馈力矩、Kp、Kd。
- **反馈**：关节、电机状态，以及 `State::sdk` 中的 IMU、末端、BMS、原始告警等。
- **保护**：启动阻尼、反馈/命令看门狗、故障锁定、停止后重启。
- **飞行记录**：可选纯 C++ 异步记录，支持人形/四足布局、事故前后历史、持续记录和触发豁免；[配置与使用](#飞行记录器) 见下文。
- **依赖**：使用工作空间 `sdk/` 中的头文件和预编译库，不编译 `internal/`。

EQR11 / EQR11W 已接入独立硬件 SDK，提供 12→16 / 16→16 电机槽映射和 0 编码器接口，完成软件与假总线验证。硬件族选择、配置和时效约束见 [EQR 接入](doc/EQR接入.md)；EQR 应用实机入口仍禁用。

仿真可通过 `finger_joint_names`、`finger_coupling_csv` / `finger_couplings` 设置手指映射和耦合，
通过 `finger_hold_position`（12 槽，0–100%）、`finger_hold_kp/kd`、`finger_torque_limits`（左右手各一项，Nm）
设置固定保持姿态；全部在启动阶段配置，不新增控制线程。此固定保持配置仅用于仿真，硬件配置会被拒绝。动态手指通过可选 `Config::hand_control` 与 `Command::hand_position` 接入原 SDK，说明见 [手指接口](doc/手指接口.md)。

## 快速开始：无界面仿真

以下命令均在**工作空间根目录**执行。需要 CMake ≥ 3.18、支持 C++20 的编译器、
Eigen3、GLFW 和 GTest；公共核心接口要求 C++17。解码工具使用 Python 3 标准库。

```bash
cmake -S . -B /tmp/eir-runtime-build \
  -DCMAKE_BUILD_TYPE=Release -DEIR_WITH_HARDWARE=OFF \
  -DEIR_BUILD_CTRL=OFF
cmake --build /tmp/eir-runtime-build --parallel 4
ctest --test-dir /tmp/eir-runtime-build --output-on-failure

/tmp/eir-runtime-build/src/eir_loco_runtime/eir_runtime_example \
  src/eir_loco_runtime/tests/data/one_joint.xml joint motor
```

示例运行约 1 秒，保持阻尼并打印反馈，不发送运动目标。最小模型未配置手指等部件，
SDK 提示缺少相关部件属于预期现象。

| CMake 选项 | 默认值 | 用途 |
| --- | --- | --- |
| `EIR_WITH_HARDWARE` | `OFF` | 编译硬件后端；仿真验证设为 `OFF` |
| `EIR_HARDWARE_FAMILY` | `aiq11` | 硬件库选择：`aiq11` 或 `eqr11`，一份构建只链接一种 |
| `EIR_EQR_SDK_ROOT` | `${EIR_SDK_ROOT}/variants/eqr11/${CMAKE_SYSTEM_PROCESSOR}/1.1.0` | EQR 版本化 SDK 目录，仅启用 EQR 硬件构建时使用 |
| `EIR_WITH_MUJOCO` | `ON` | 编译 MuJoCo 3.14.0 后端 |
| `EIR_RUNTIME_WITH_ROS2` | `OFF` | 编译只读 ROS 2 发布库 |
| `BUILD_TESTING` | `ON` | 编译测试；关闭后不需要 GTest |

## 接入控制程序

链接 `eir::loco_runtime`，从 [完整示例](examples/runtime_example.cpp) 开始。
[公共接口](include/eir_loco_runtime/runtime.hpp) 包含全部配置和数据类型。
按电机指定启动和 Damping 模式增益，见 [阻尼配置](doc/阻尼配置.md)。

```cpp
#include <eir_loco_runtime/runtime.hpp>
using namespace eir::loco;

Config config;
config.model_path = "/path/to/robot.xml";
config.joint_names = {"joint"};  // MJCF joint 名称
config.motor_names = {"motor"}; // MJCF actuator 名称，也是命令数组顺序
config.suspend = false;         // 默认 true；按模型选择是否悬吊
Runtime runtime(config);
```

| 调用 | 行为 |
| --- | --- |
| `start()` | 等待首帧反馈，成功后进入 `Damping` |
| `read(state)` | 复制最新快照；无反馈或过期时返回 `false`，不修改输出 |
| `setDamping(false)` | 控制器就绪后，显式进入 `Running` |
| `write(command)` | 下发五组电机指令；阻尼模式下只发送阻尼指令 |
| `setDamping(true)` | 恢复阻尼 |
| `releaseSuspension()` | 仿真取消悬吊，重复调用无副作用；硬件、未启动或急停状态返回 `false` |
| `status()` | 查询模式、反馈年龄、悬吊请求状态 `suspended` 和错误原因 |
| `estop()` / `stop()` | 锁定急停 / 停止线程、释放后端并等待记录器收尾 |
| `saveFlightRecord()` | 请求后台保存事故，返回值仅表示受理 |
| `flightStatistics()` | 查询记录器健康、缓存预算、丢失及保存结果 |
| `recordApplication(packet)` | 提交应用诊断或事件；使用定长 `flight::Packet` |

控制循环中先 `read()`，再计算 `Command(config.motor_names.size())` 的
`position / velocity / torque / kp / kd`，最后调用 `write()`；返回失败时退出循环并检查
`status().message`。`start()` 等生命周期调用由应用串行执行，监控线程可调用 `read/status`。
`releaseSuspension()` 与 SDK 读写保护串行，放在普通工作线程；成功表示 SDK 接受释放请求，下一物理步生效。
不提供运行中重新挂起；停止后重新启动按 `Config::suspend` 恢复，悬吊模型要求基座为 XYZ 三个 slide 加 ball 关节。

默认反馈、命令超时均为 **100 ms**，看门狗周期 **5 ms**；只有 `Running` 要求持续更新命令；每次后端写入（含初始阻尼）也受命令超时约束。
EQR SDK 另有独立的 **100 ms** 命令超时锁定，在 `Damping` 下同样生效；使用 EQR 硬件后端时也须由普通 IO 线程定期 `write()`，详见 [EQR 接入](doc/EQR接入.md)。
非法命令、反馈异常或超时会锁定 `Estopped`，排查后必须 `stop()` 再 `start()`，不会自动恢复。
构造时的配置错误抛出 `std::invalid_argument`；启动失败返回 `false`。
看门狗不等待普通 IO 锁；急停先锁定后端，线程回收留给 `stop()`。迟到的写入返回失败，不能刷新命令有效期。
注入后端的 `estop()` 必须可与 `write()` 并发、快速锁定输出且拒绝后续运动，不得等待写入锁或销毁资源。
`stop()` 仍需等待正在执行的后端调用返回，不能强行终止任意第三方阻塞代码。
首帧等待默认 2 秒，从 SDK 初始化返回后计时，不能中断 SDK 的阻塞初始化。

仿真沿用 MJCF 的 timestep、积分器和接触配置；编译和运行均使用 SDK 内 MuJoCo 3.14.0。
当前不提供步态控制、末端/头部控制或机器人专用限位策略，也不承诺硬实时。

## 飞行记录器

记录器随核心库编译，默认关闭，不依赖 ROS。由应用将自己的配置文件解析为
[flight::Config](include/eir_loco_runtime/flight_recorder.hpp)，runtime 本身不读取 YAML。

在实际机型的模型、关节和电机映射配置完成后，构造 runtime 前设置：

```cpp
config.robot_identity = "pro"; // 实际机型：pro / sport / eqr11 / eqr11w
config.flight_recorder.enabled = true;
config.flight_recorder.directory = "/var/tmp/eir-flight";
// 需要监测头部舵机，但掉线仅留事件、不保存事故：
config.flight_recorder.data |= flight::Head;
config.flight_recorder.trigger_exemptions.push_back(
    {flight::Event::HardwareOffline, flight::Component::HeadServo, {}});
Runtime runtime(config);
```

runtime 根据已有映射生成记录布局。EQR 可沿用 `hardware_robot` 身份；轮关节由
`continuous_joints` 标记。其他程序也可直接构造 `flight::Recorder(Config, Layout)`。
头部、手和力传感器默认不录制；公共默认触发豁免名单为空。未安装的硬件通过 `data`
关闭采集，按类别或具体电机名称设置 `trigger_exemptions`。这些配置不改变运动保护。

| 默认参数 | 值 |
| --- | --- |
| 命令/反馈、应用诊断、持续精简记录 | 最高 500 Hz、50 Hz、50 Hz |
| 故障窗口、重复触发合并期 | 前 10 秒/后 5 秒、首次触发后 10 秒 |
| 内存预算、目录总配额、磁盘空闲底线 | 256 MiB、2 GiB、1 GiB |
| 持续/事件文件、同步间隔 | 每类当前及上一份，每份 64 MiB；100 ms |

默认触发包括电机错误/离线、反馈/命令超时、非法命令/反馈、后端失败、应用故障、人工急停和手动保存。
普通告警、模式切换只记事件。实际触发来自已有检测路径；采集某个传感器字段不等于新增其掉线保护。
请求等离散记录可设置 `Packet::EventOnly`：即使 `event=None` 也写入事件文件，
不占诊断历史采样槽。带该标志的触发故障仍保存快照和事故，不能用它取消触发。

详细数据按来源频率保存在内存历史中，触发后后台保存快照、前历史及后窗口；磁盘持续文件只保存精简状态。
已入队事件先独立写入 `events.bin`，同一采样时间桶中的额外非触发包不再挤占历史；
完整分析需要同时查看事件与事故文件。EventOnly 请求保留在事件文件中，事故文件保留采样诊断和触发包。
采集使用定长复制和有界队列；一个同进程后台线程负责缓存和文件 IO。竞争、队列满、缓存覆盖或存储失败会计数，不能承诺所有数据都保存。

```cpp
const bool accepted = runtime.saveFlightRecord(); // true 不等于已经落盘，重复请求可能合并
const auto stats = runtime.flightStatistics();
// 检查 enabled、healthy、dropped、write_failures、quota_rejections。
// completed 是完整事故数；incomplete 是提前结束、丢失或保存失败的事故数。
```

`memory_bytes` 是记录器计算的缓存预算，非进程 RSS；`max_submit_ns` 只统计入队接口耗时，
不包含调用者打包耗时。`accepted` 含降采样检查通过的提交次数，不等于文件中的帧数。
PRO 32 轴标准仿真布局约占 253.3 MiB，接近 256 MiB 上限；新增字段、频率或窗口前重新核算。

关闭顺序：先抑制运动，再停止并等待应用全部记录生产线程/回调，最后调用 `runtime.stop()`。
普通退出可用 `estop(false)` 抑制输出而不触发新事故。`stop()` 排空已受理队列、处理手动保存并等待后台同步；
已有事故提前结束标记不完整，不额外等待完整后窗口。记录 API 不得与重启并发使用。
磁盘或第三方 SDK 阻塞会延长退出，当前没有固定退出截止时间。

每次运行生成 `eirflight_*` 目录；保存文件含校验和及版本，使用随库安装的工具查看：

```bash
eir-decode-flight-record /path/incident_时间_编号.bin --csv /tmp/事故.csv
# 源码使用：python3 src/eir_loco_runtime/tools/解码飞行记录.py FILE
```

输出 CSV 或逐行 JSON；截断或校验错误返回 2，仍可导出已验证的完整记录。
突然断电恢复不属于当前验收要求；仅已持久化数据可能保留，内存详细历史不会自动落盘。
公开版本的构建与验证范围见工作空间的 doc/公共/。内部测试记录不随本次公开分支交付。

## 可选 ROS 2 发布层

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B /tmp/eir-runtime-ros2-build \
  -DCMAKE_BUILD_TYPE=Release -DEIR_WITH_HARDWARE=OFF \
  -DEIR_RUNTIME_WITH_ROS2=ON -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/eir-runtime-ros2-build --parallel 4
ROS_LOCALHOST_ONLY=1 ctest --test-dir /tmp/eir-runtime-ros2-build --output-on-failure
```

链接 `eir::loco_runtime_ros2`。在应用的监控线程或定时器中使用已有 ROS 节点 `node`：

```cpp
#include <eir_loco_runtime_ros2/state_publisher.hpp>
eir::loco::ros2::PublisherOptions options;
// 仅发布告警：options.joint_states = false; options.imu = false;
eir::loco::ros2::StatePublisher publisher(node, runtime.config(), options);

// 每个发布周期执行，即使 read() 失败也发布诊断。
eir::loco::State snapshot;
runtime.read(snapshot);
publisher.publish(snapshot, runtime.status());
```

| 默认话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/msg/JointState` | 关节位置、速度、力矩 |
| `imu` | `sensor_msgs/msg/Imu` | 首个 IMU 的姿态、角速度、加速度 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | runtime 状态、电机状态和 SDK 告警 |

话题名可配置；反馈过期时停止发布关节和 IMU 数据，继续发布 STALE/ERROR 诊断。
发布层不创建控制线程，也没有控制订阅或服务。

## 安装与下游使用

```bash
cmake --install /tmp/eir-runtime-build --prefix /tmp/eir-runtime-install
cp -a sdk /tmp/eir-runtime-install/sdk
```

下游将安装目录加入 `CMAKE_PREFIX_PATH`，然后：

```cmake
find_package(eir_loco_runtime CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE eir::loco_runtime)
```

安装包需保留 `include/`、`lib/`、`sdk/` 的相对位置；SDK 不会随 runtime 自动安装。
记录器包含在 `libeir_loco_runtime.so` 中，无单独记录器 SDK；`bin/` 安装解码工具。
新增公共配置/后端接口后，所有下游 C++ 使用者应与 runtime 成套重编，避免布局或虚表不一致。

## 验证范围与已知问题

2026-10-13 最新记录器集成回归通过 8 项 CTest，包括记录器/解码、runtime、最小 MuJoCo、
AIQ 控制、注入硬件后端、ROS 诊断/手动保存及 PRO 仿真。公共记录器 16 项测试和 ASan/UBSan 通过；
退出交错场景重复 100 次通过。TSAN 因本机运行环境报错未完成，不能记为通过。

四机型的记录布局已通过软件测试，EQR 应用记录器未在本轮接入或验收。
原有核心读写、看门狗、急停/重启、可选 ROS 消息测试的版本与范围以工作空间证据为准；
不将旧数量当作当前完整用例数。详细结果见 [最新测试记录](../../doc/test-results/2026-10-13/飞行记录器退出修复/测试记录.md)。

硬件默认不编译，当前验证为代码、注入后端或假总线。AIQ11 PRO 已有独立硬件入口；
EQR 独立 SDK 软件验收见 [EQR 接入](doc/EQR接入.md)。真实总线故障、目标主控长时负载和慢盘退出仍待验收。
SDK 急停锁定修复未经实机验收，恢复须销毁并重建 SDK 实例；硬件映射及 YAML 需明确校验。
运动策略、地面动作质量和足底接触力的验证由对应应用文档说明，不能由核心读写回归推定。

本项目自有源码采用 [BSD-3-Clause](LICENSE)；第三方组件保留原许可证。

## 可选仿真深度

`Config::depth` 默认关闭。启用后须指定准确的相机 site、预处理版本与采样参数，配置错误在启动时拒绝。普通线程调用 `readDepth(DepthStack&)` 读取 8×18×32 快照；`stamp` 是 `steady_clock` 采集时间，调用方必须检查有效性、序号、像素及年龄，重复读取不会刷新时间。深度不进入关节反馈，也不通过实时线程发布 ROS。SDK 头文件/动态库需成套更新后重编；当前仅提供仿真来源。
