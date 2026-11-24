# PID 调参助手 更新记录

## v1.4（2026-11-24）

v1.3 源码完整备份在 `Tools\PIDTuner_backup_v1.3_20261124`。

### 新增与完善功能
- **阶跃同起点残影对比（Ghost 对比底图）**（`scope.py`、`main.py`）：
  - 左侧“阶跃与游标”新增 **【📌 锁定对比底图】**：将当前阶跃响应定格为紫色虚线参考底图，后续任意时刻的新阶跃都会**自动对齐跳变起点与幅值叠加绘制**，并在面板同步对比新旧超调与调节时间。
- **硬件与信号体检工具箱 + FFT 频谱主频诊断**（`diagnostics.py`）：
  - 顶部新增 **【🩺 信号体检】**：一键测算各通道均值（可用于提取平衡车静止机械中值角 `BALANCE_TARGET_PITCH_DEG`）、标准差 `σ`（传感器底噪）、峰峰值、**FFT 主振荡频率**（自动区分低频控制振荡与高频微分/量化噪声）以及 USB-TTL 攒包抖动率，支持一键将实测帧率写入固定上报周期。
- **一键生成 C 语言参数代码**（`diagnostics.py`）：
  - 右侧新增 **【C 代码】** 按钮：内置 3 套直接适配你 STM32 工程的代码模板（`User/pid_debug.c` 与 `pid.c` 速度环宏、`User/balance.c` 直立环宏、四环结构体初始化语句），一键复制到剪贴板。
- **历史 CSV 离线导入与回放**（`storage.py`、`main.py`）：
  - 顶部新增 **【打开 CSV】**：可将录制或导出的 CSV（或通用多列 CSV）重新载入示波器，完整还原通道名、单位、参考线与历史参数快照，支持离线缩放、A/B 游标测量、阶跃计算与 FFT 体检。
- **滑块连续微调 + 5 档步长切换 + 终端自定义快捷指令**：
  - 右侧面板新增 **【滑块微调】** 开关与 `0.001 ~ 10` 五档滚轮步长选择；串口终端上方新增 5 个可右键编辑名称与内容的快捷指令按钮（自动持久化保存）。
- **后台异常看门狗 + 平衡车工程原生状态兼容**：
  - 自动监测“输出非零但传感器读数卡死不变”与“控制输出连续满幅饱和 >2.5 s”并弹出提示；
  - 针对 `User/pid_debug.c` 返回 `only SPD supported` 时自动切换至 `SPD` 速度环读回参数；
  - 原生识别 `Doc/main_balance_test.c` 的 `#BALANCE:CALIBRATING`、`READY`、`INIT_ERROR=1..8`（自动翻译 MPU6050 错误码）、`START_REJECTED`、`TILT_STOP`、`IMU_TIMEOUT` 并联动顶部横幅提示。

---

## v1.3（2026-11-24）

v1.2 源码备份在 `Tools\PIDTuner_backup_v1.2_20261124`，v1.1 备份在 `Tools\PIDTuner_backup_v1.1_20261124`。

### 新增功能
- **PID 自动整定（Auto-Tune）**（`autotune.py`、`autotune_ui.py`）：
  - 用户只需输入**目标值**（如控速 `60` / `100`）与**起始基准**，无需修改下位机固件协议，软件即可自动完成激励、采样、打分与寻优。
  - **双阶段混合算法**：第 1 阶段通过闭环比例探针阶跃自动辨识被控对象的稳态增益、主导时间常数与纯延迟（采用 Skogestad SIMC 内模整定）或二阶欠阻尼极点（极点配置整定）；第 2 阶段通过物理特征引导的 Hooke–Jeeves 模式搜索在真实阶跃波形上多轮迭代微调。
  - **三种控制器模式与三种风格**：支持 `PI`（控速推荐）、`PID`（位置/姿态）、`PD`（直立/快响），可选 `平稳无超调`、`均衡推荐`、`快速响应` 权重配置，并内置控制输出高频抖动惩罚与满幅饱和提示。
  - **平衡车局部微调与安全回退**：提供“从当前参数局部微调”策略，适配倒立摆等开环不稳定对象；途中支持随时按 `Esc` 急停、手动提前停止采用当前最优组，以及整定完成后**一键撤销回整定前参数**。
- **新版图文操作手册**：配套生成 `PID调参助手-v1.3-图文使用手册.docx` 与 `.pdf`，涵盖快速上手、串口双轴配置、参数保护、目标值发生器、自动整定实战、平衡车固件接入与故障排查。

---

## v1.2（2026-11-24）

v1.1 源码完整备份在 `Tools\PIDTuner_backup_v1.1_20261124`（不含 exe）。

### 上位机
- **倒立摆仿真对象**：仿真对象新增“倒立摆（平衡车直立）”。不加控制会倒下；D 项作用于角速度，相当于用陀螺仪；
  倾角偏离 TGT 超过 45° 判定倒地，仿真设备自动停机并上报 `#STOP:fall`，界面显示倒地横幅。
- **目标值发生器**（PID 面板下方）：单次阶跃、方波、正弦、三角波。以启动时 TGT 输入框的数值为基准，
  按幅值和周期由电脑下发 `TGT`，阶跃指标可重复对比。
  - 启动前检查波形范围是否在本组 TGT 允许范围内；不弹大幅变化确认、不进撤销历史、不画参数竖线。
  - 阶跃、方波只在跳变时下发；正弦、三角按“节拍 Hz”下发，上一条未应答时跳过本拍，不会堆积。
  - 手动停止时恢复基准 TGT；急停、断线、命令失败、设备停机时立即停止。
- **心跳改由串口线程发送**：每 0.2 s 一次 `@PING`，前提是界面在 1 s 内刷新过。拖动窗口等短暂卡顿不会误停机，
  界面真的卡死时心跳停止，下位机 0.5 s 后自行停机。心跳不再刷屏写入终端。
- **仿真器模拟心跳**：`@RUN` 回复 `#CAP:PING`，收到过 `@PING` 后 0.5 s 收不到就上报 `#STOP:heartbeat`。
  界面卡顿时仿真暂停推进，不会“快进”导致误停。
- **断线自动重连**（连接设置，可关闭）：意外断开后每 1~3 s 尝试重开同一串口，保留波形；重连成功只读回参数
  （`@GET`），**不会自动启动**。按钮显示“取消重连”。
- **阶跃指标**：IAE、超调、尾段误差对缺失数据（NaN）不再变成 nan。
- **代码结构**：大幅变化确认改为可注入回调（`DeviceSession(confirm=...)`），可脱离对话框测试；
  新增 `generator.py`；统一 LF 行尾并加入 `.gitattributes`。

### 下位机示例
- `pid_link.c` 不再使用 `printf`/`strtof`：数字用整数运算格式化（4 位小数、去尾零）和解析，
  **不再需要 `-u _printf_float`**，F103C8 省下十几 KB Flash。`PID_CheckFloatFormat()` 保留为空函数以兼容。
- 新增 `PID_PositionRate()`：D 项直接使用测得的角速度（MPU6050 陀螺仪），比对角度差分噪声小。
- 新增 `PID_Fault(reason)`：可在控制中断里调用，立即清零输出，由前台 `PID_WatchdogTick` 上报 `#STOP:<reason>`。
- 新增 `PID_ReportValues()`：上报 1~4 个任意通道。
- 新增 `balance_example.c/.h`：平衡车串级骨架（直立环 PD + 速度环 PI + 转向环），5 ms 定时器中断固定周期、
  死区补偿、输出限幅、倒地保护（|俯仰 − ANG.TGT| > 40°）。硬件通过 `Board_ReadImu / Board_ReadEncoders /
  Board_SetMotors` 三个钩子接入。

### 测试
- `test_tuner.py`：新增倒立摆、仿真心跳、发生器、线程心跳、倒地停机、自动重连、NaN 指标等用例。
- `tests/firmware_check.c`：新增数字格式化/解析、`PID_PositionRate`、`PID_Fault` 用例。
- `tests/balance_check.c`：平衡车骨架对简化倒立摆的闭环测试与倒地停机测试。

编译固件测试（任选 gcc 或 zig）：

```
gcc -I firmware_example tests/firmware_check.c firmware_example/pid_link.c -lm -o fw && ./fw
gcc -I firmware_example -I tests/hal_stub tests/hal_check.c firmware_example/hal_demo.c firmware_example/pid_link.c -lm -o hal && ./hal
gcc -I firmware_example tests/balance_check.c firmware_example/balance_example.c firmware_example/pid_link.c -lm -o bal && ./bal
```
