# EQR 接入

runtime 已提供 EQR11 四足和 EQR11W 轮足的硬件后端，使用独立交付的 `eir_base_sdk_eqr`，复用现有 `RobotState` / `RobotCmd`。当前完成的是软件和假总线验证，没有连接机器人。EQR 应用的 CLI/YAML 及未注入后端的硬件 Runner 仍拒绝启动。

## 构建与 SDK 选择

以下命令在提供 `sdk/` 和公共 CMake 的集成工作空间根目录执行，只编译后端并运行软件测试：

```bash
cmake -S . -B build/eqr-runtime-check \
  -DEIR_WITH_HARDWARE=ON -DEIR_HARDWARE_FAMILY=eqr11 \
  -DEIR_WITH_MUJOCO=OFF -DBUILD_TESTING=ON
cmake --build build/eqr-runtime-check -j4
ctest --test-dir build/eqr-runtime-check --output-on-failure
```

一份 runtime 构建只选择一种硬件 SDK。`EIR_HARDWARE_FAMILY` 仍默认 `aiq11`；EQR11 和 EQR11W 共用构建选项 `eqr11`，具体型号由运行时配置区分。仿真使用 `BackendType::Simulation`，无需打开硬件构建选项。

EQR SDK 默认目录为 `${EIR_SDK_ROOT}/variants/eqr11/${CMAKE_SYSTEM_PROCESSOR}/1.1.0/`，可通过 `EIR_EQR_SDK_ROOT` 指定配套交付包。当前交付 x86_64 包含：

- `include/eir_base_sdk_eqr/`：公共接口和 ABI 标记。
- `include/eir_lowlevel_sdk/`：配套共享数据结构。
- `lib/libeir_base_sdk_eqr.so.1`：独立 SONAME，实际文件为 `.so.1.1.0`。
- `share/eir_base_sdk_eqr/config/eqr11.yaml`、`eqr11w.yaml`：明确型号的 SDK 配置。
- `manifest.json`：源提交、架构、ABI、依赖和文件 SHA256。

构建阶段比较共享头文件指纹，真实后端在 `Init` 前比较已加载库的 ABI 标记并校验型号配置。当前设备库依赖 EtherLab 1.6 的 `libethercat.so.1`、yaml-cpp 0.7 和配套 C++ ABI，详情以清单为准。安装 runtime 后仍需随包提供匹配的 SDK；公开源码无需 `internal/`。

## 配置与槽位

| 字段 | EQR 要求 |
| --- | --- |
| `backend` | `BackendType::Hardware` |
| `hardware_family` | `HardwareFamily::Eqr` |
| `hardware_robot` | `eqr11` 或 `eqr11w`，与 SDK YAML 的 `RobotType` 一致 |
| `hardware_motor_slots` | 固定 16 |
| `encoder_count` / `encoder_names` | 0 / 空 |
| `joint_names` / `motor_names` | 同一组名称、顺序相同；覆盖全部活动关节 |
| `motor_slots` | 显式提供、无重复；每个名字必须匹配其 SDK 槽 |
| `control_mode` | 仅普通模式 1；不用于校准或置零 |
| `hand_control` / 手指映射 | 关闭 / 空 |

每条腿占 4 槽，顺序为 `fl`、`fr`、`hl`、`hr`。前三槽分别对应 `<腿>_leg_1`、`<腿>_leg_2`、`<腿>_leg_3`，第四槽为 `<腿>_wheel`。四足禁用 3/7/11/15 槽，轮足使用全部 16 槽。

四足配置示例只构造参数，不调用设备初始化：

```cpp
#include <eir_loco_runtime/runtime.hpp>
using namespace eir::loco;

Config config;
config.backend = BackendType::Hardware;
config.hardware_family = HardwareFamily::Eqr;
config.hardware_robot = "eqr11";
config.hardware_config = "/path/to/eqr-sdk/share/eir_base_sdk_eqr/config/eqr11.yaml";
config.hardware_motor_slots = 16;
config.encoder_count = 0;
config.joint_names = {
    "fl_leg_1", "fl_leg_2", "fl_leg_3", "fr_leg_1", "fr_leg_2", "fr_leg_3",
    "hl_leg_1", "hl_leg_2", "hl_leg_3", "hr_leg_1", "hr_leg_2", "hr_leg_3"
};
config.motor_names = config.joint_names;
config.motor_slots = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
```

四足 `Command` 长度为 12，轮足为 16，均采用 `motor_names` 顺序。runtime 将其映射为 SDK 的固定 16 槽；四足未映射轮槽保持 mode=0、所有数值为零。SDK 配置使用原电机名 `fl_1` 等，应用/runtime 使用上述关节名，通过槽位对应；仿真则仅使用 MJCF 名称顺序，`motor_slots` 必须为空。

## 时效与故障恢复

- SDK 发布命令要求单调时钟纳秒时间戳严格递增、年龄不超过 100 ms；runtime 在普通 IO 写入处生成该时间戳。SDK 检查有限值、维度、控制模式、禁用槽和电机协议范围，拒绝时锁定急停。
- runtime 校验 EQR 反馈精确 16 槽，以及 SDK 源时间戳递增、未超时；反馈年龄保留源时间，不因回调到达而刷新。上层控制器和推理仍须检查自身输入与输出时效。
- `start()` 后为 `Damping`，只在进入时发送一次阻尼。**EQR SDK 自身的 100 ms 命令 watchdog 在阻尼模式下也生效**，普通 IO 线程须定期调用 `write()`；该模式下 runtime 会输出阻尼。看门狗线程不代替应用执行普通 IO。
- 急停、无效反馈或超时持续锁定；故障排查后通过 `stop()` / `start()` 开启新生命周期，不能用 `setDamping(false)` 清除。SDK 不重放上一生命周期的命令。
- 生命周期调用由应用串行化，后端急停可独立并发；停止会等待在途普通调用/回调返回。当前不承诺硬实时。

## 软件验证边界

2026-10-10 接入验证包括 SDK 13 项安全用例、应用→runtime→SDK→假总线 4 项集成用例及 ASan/UBSan，另有 EQR 设备库链接软件测试、ABI 错配拒绝、EQR 悬吊/RL 回归和原功能兼容回归。SDK 核心测试使用传输替身，未运行真实 EtherCAT 初始化；公开 runtime 测试使用注入后端或故意非法的启动配置。

完整记录由集成工作空间 `doc/test-results/2026-10-10/EQR11硬件接口接入/验证记录.md` 维护，固件语义和设备验收事项见 `doc/EQR/EQR11实机接口移植.md`。实际 PDO/IMU/电机标定、固件采样冻结检测、在途报文、设备 watchdog、目标机实时性能均未验收；代码和软件测试通过不开放应用实机入口。
