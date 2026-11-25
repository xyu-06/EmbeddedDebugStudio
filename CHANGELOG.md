# 嵌入式综合调试助手（EmbeddedDebugStudio）更新记录

## v1.8（2026-11-25）

### 新增第 5 工作台【🧩 接口构建】（`firmware_studio.py`，快捷键 `Ctrl+5`）与任意列顺序 `#ROLES:`
- **🧩 统一接口构建与说明中心（三大接口 × 三大主流平台）**：
  - 将原先分散在各调试页的代码复制收拢至 **【🧩 接口构建】** 统一配置与生成，并在 **📈 PID 调参 / 🧭 IMU 姿态 / 🎮 遥控与 HC-05** 顶栏提供一键直达快捷按钮（`🧩 接口代码与说明` / `🧩 IMU 接口与说明` / `🧩 遥控接口与说明`），点击即可带着当前页面的格式与波特率上下文跳入【接口构建】选平台、复制/导出源码并查阅对应接口说明。
  - **覆盖全软件 3 类串口接口**：① **PID 闭环调参接口**（1～8 通道、`#NAMES:`/`#ROLES:`、`@RUN/@STOP/@PING/@SET`、`500 ms` 看门狗）、② **IMU / MPU6050 姿态接口**（支持六轴物理量、原始计数 LSB、平衡车四列、欧拉角三列，含 `#BALANCE:` 校准与保护横幅）、③ **小车差速遥控接口 `J,x,y`**（精确对齐 `BlueSerial_MixWheels` 差速混控、`S` 急停、`LED_ON/OFF/BEEP/SERVO` 与超时失联保护）。
  - **覆盖 3 大主流目标库**：**STM32 HAL 库（CubeMX · `huart` 中断收发）**、**STM32 标准外设库 SPL（`stm32f10x.h` · `USARTx` 中断）** 与 **ESP32 / Arduino（`Serial` C++ · `.ino`）**；STM32 代码全部基于整数定点数串行化，**零 `-u _printf_float` 依赖**，已通过 `-std=c99 -Wall -Wextra -Werror` 严格编译验证。
  - **动态【📖 协议与使用配置说明】**：随左栏所选接口类型与目标库自动切换对应接线步骤、CubeMX/Keil SPL/Arduino 接入指引、协议报文表及故障排查清单，并附全软件三大接口速查总览；PyInstaller 打包同步内嵌 `firmware_example/` 模板目录，单文件版 EXE 可离线导出全部源码。
- **任意通道顺序自动绑定协议 `#ROLES:`**（`engine.py`、`main.py`、`autotune_ui.py`）：
  - 下位机只需上报 `#ROLES:output,error,actual,target`（或使用规范通道名），无论将 PWM、误差、实际速度、目标值放在第几列，上位机均自动识别并锁定真实反馈列、目标列与输出参考列，彻底消除顺序不一致导致的调参错误。

---

## v1.7.1（2026-11-25）

### Bug 修复：STM32 `pid_debug.c` 自动整定误选目标通道导致整定失灵
- **延后并在 `WAIT_START` 按通道数创建参考线**（`autotune_ui.py`、`main.py`）：
  - 修复下位机在收到 `@RUN` 前未上报数据（`w.count == 0`）时，自动整定误判 `w.count <= 2` 而提前插入 `"整定目标"` 参考线，进而在首帧到达（`#NAMES:target,actual,output,error`）后把 `actual_ch` 默认误选成 `CH1 (target)`、把阶跃超调/调节时间误算为 `0.0% / 0.00 s / J=0.000` 并挑出过小 `KP/KI` 的缺陷。
  - `refresh_channels()` 与 `AutoTuneController.detect_channels()` 优先按通道名（`target/目标`、`actual/实际`、`output/pwm`）与 `>=3` 通道结构精准锁定真实反馈通道（`CH2`）与输出参考通道（`CH3`），即便用户手动添加了参考线也保持正确匹配。
- **增加目标通道误选防呆保护**（`autotune_ui.py`）：若 `OBSERVE` 阶跃首帧至末帧全程贴合 `target`（`±2%` 内无真实响应过程），立即中止整定、将整定前原参数写回设备并明确提示修正反馈通道。

---

## v1.7（2026-11-25）

针对 v1.6 工程审计报告完成全部 **12 项协议一致性、控制安全性与多通道扩展加固**：

### 安全加固与协议一致性
- **自动整定全链路安全保护**（`autotune.py`、`autotune_ui.py`、`main.py`）：
  - **无赢家退出强制回滚下位机参数**：凡整定中途失败、采样超时、设备异常停机且尚无有效最优组、或中途断开串口，均自动把整定前的原 `KP/KI/KD/TGT` 写回下位机与界面，绝不将第 N 轮试探参数留在设备上；
  - **直立环（`ANG`）强约束与安全确认**：选中 `ANG` 组时强制锁定“从当前参数局部微调”策略（对话框与 `start()` 双重校验），局部模式拒绝全零 `KP/KI`，单轮变化比例上限收紧至 `±35%`（总体限制 `[0.4x, 2.5x]`），开始按钮须勾选“车轮已架空离地或已加装机械限位”后才解锁。
- **原子组参数写入命令 `@<GRP>.SET=kp,ki,kd,tgt`**（`firmware_example/pid_link.c`、`engine.py`、`device_session.py`、`autotune_ui.py`、`main.py`）：
  - 下位机 `@RUN` 新增上报 `#CAP:SET`，上位机自动识别并在“发送本组”和自动整定切参时优先使用单条 `@<GRP>.SET=kp,ki,kd,tgt` 一次生效（全 4 项校验通过才更新控制环），消除分条发送导致新旧参数短暂混用的风险；旧固件自动兼容回退分条下发。
- **断开连接 / 串口让渡前先冲洗急停报文**（`transport.py`、`main.py`、`remote_studio.py`）：
  - 新增 `SerialLink.flush_and_close()`，PID 页 `disconnect()` / `release_port()` 与遥控页 `disconnect_port()` 在关闭串口句柄前均先写入 `@STOP`（及整定回滚参数）或 `J,0,0` + `S` 并 `flush()` 确认发出。
- **遥控防卡死与摇杆 100 ms 心跳保活**（`remote_studio.py`、`workbench.py`）：
  - 捕获 `hideEvent`、`focusOutEvent`、`ActivationChange`/`WindowStateChange` 与全局 `ApplicationDeactivate`/`WindowDeactivate`，弹窗或切走窗口失焦时立即清零 `held_keys` 并急发 `J,0,0`；
  - 鼠标拖拽非零摇杆保持不动时，`tick()` 每 `100 ms` 自动重发当前控制指令充当心跳保活（配合车端 0.5 s 无指令自动停车）。
- **HC-05 波特率列表修复与写前对比校验**（`remote_studio.py`）：
  - 补全标准波特率 **`57600`**，将兼容手册原文 `5760` 明确标注为 **`5760（说明书原文，疑似笔误）`** 并在选中写入时弹窗二次确认；串口已连接执行 `AT+UART=` 写入前先下发 `AT+UART?` 读回模块当前配置对比确认，避免误改失联。
- **仿真器与固件心跳完全对齐**（`engine.py`）：移除仿真器延迟布防开关 `armed`，与 STM32 `pid_link.c` 保持一致：`@RUN` 后若未持续收到 `@PING` 心跳，满 `0.5 s` 即自动 `#STOP:heartbeat` 停机。

### 多通道扩展、倒立摆扰动测试与代码清理
- **全链路扩展至 8 通道实时示波器 + 4 参考线**（`pid_link.c/.h`、`engine.py`、`scope.py`、`storage.py`、`layout_ui.py`、`configuration.py`、`diagnostics.py`、`main.py`）：协议、示波器、换算标定与 CSV 导出导入均支持 1～8 通道（`<= 4` 通道导出仍保持 21 列向后兼容）。
- **倒立摆仿真物理增强与 `👋 推一下` 扰动按钮**（`engine.py`、`layout_ui.py`、`main.py`）：新增 `25 ms` 一阶执行器滞后与姿态测量噪声，新增 `@NUDGE` 指令与倒立摆仿真工具栏 `👋 推一下` 按钮以测试抗扰动恢复能力。
- **统一版本常量与死代码清理**（`version.py`、`main.py`）：新增 `version.py` 统一管理版本号，删除 `main.py` 中已废弃的 `#ERROR:float printf` 死代码分支。

---

## v1.6（2026-11-25）

联动整合 Android 端 [BlueSerialStudio](https://github.com/xyu-06/BlueSerialStudio) 协议生态，升级为四大工作台一体机并同步至云端仓库：

### 新增功能
- **第四工作台【🎮 遥控与 HC-05】（`remote_studio.py`，快捷键 `Ctrl+4`）**：
  - **小车差速遥控台（100% 兼容 `BlueSerialStudio` `J,{x},{y}` 协议）**：支持 **360° 单摇杆（鼠标拖拽回中）**、**左右双摇杆（左油门 Y / 右转向 X 独立归零）**、**四按键 / 键盘 `WASD` 长按渐进加速（18% 起步、1.6s 满速 100%、反向键互消）** 三种控制模式；实时计算并预览 `BlueSerial_MixWheels` 左右轮差速百分比，支持自定义数据包模板、`LED_ON / LED_OFF / BEEP / SERVO` 快捷控制、切走页面自动发 `J,0,0` 安全归零、全局 `Esc` 急停以及一键复制 STM32 差速混控 C 代码。
  - **HC-05 蓝牙模块 AT 配置工具箱（USB-TTL 串口直发 + 剪贴板复制双模式）**：完整移植 `AtCommandBuilder` 参数校验规则（名称 `AT+NAME=`、配对码 `AT+PSWD=`、主从角色 `AT+ROLE=`、串口参数 `AT+UART=` 含兼容说明书原文 `5760` 波特率及 `AT+ORGL` `38400,0,0` 必读提示），所有指令自动附加 `CRLF (\r\n)`，既可一键复制，也可直接通过当前打开的电脑 USB-TTL 端口发送并在底部回显区实时查看 `OK` / `+VERSION` 响应。

---

## v1.5（2026-11-25）

v1.4 源码完整备份在 `Tools\PIDTuner_backup_v1.4_20261124`。

### 多模块综合工作台架构（可扩展一体机升级）
- **上方总菜单 + 顶栏一键模块导航栏**（`layout_ui.py`、`main.py`、`workbench.py`）：
  - 新增顶部菜单栏 **【调试模块(M)】** / **【视图与扩展(V)】** 以及常驻导航栏（支持鼠标点击页签或快捷键 `Ctrl+1` / `Ctrl+2` / `Ctrl+3` 秒切工作台），页签实时显示各模块连接运行圆点 `●` 与全局端口占用提示；
  - **100% 完整保留原有【📈 PID 调参】工作台**（含底部可折叠串口终端、智能自动整定、Ghost 阶跃对比底图、FFT 信号体检与 C 代码生成等全部功能），并在任意页面下保留 `Esc` 最高优先级电机急停；
  - **跨模块同串口智能让渡**：当某串口（如 `COM5`）正被 PID 页占用而用户在串口页或 IMU 页点击连接同一端口时，自动弹窗询问并安全释放原页面串口句柄后再无缝接管，彻底告别切换软件反复开关端口的繁琐；
  - **统一插件扩展接口（`WorkbenchModule`）**：内置【视图与扩展 → 模块扩展接口说明…】代码模板，后续新增 **📷 摄像头图传（OV7670 / OpenMV）**、蓝牙遥控或 CAN 总线页面时仅需注册一行代码即可接入。
- **独立全功能【🔌 串口调试】页面**（`serial_studio.py`）：
  - 独立串口引擎，支持 `4800 ~ 2000000` 自定义波特率、5~8 数据位、5 种校验位、硬件/软件流控、**DTR / RTS 电平实时控制**与断线自动重连；
  - 接收区支持 **ASCII / HEX 十六进制** 一键切换、**UTF-8 / GBK（兼容中文老工程） / Latin-1** 编码切换、毫秒时间戳、收发方向标识、**空闲超时自动断帧（可设 ms）**、**关键字实时过滤日志**、暂停显示与一键导出 `.txt` 日志；
  - 发送区支持多行文本或 HEX 发送（`Ctrl+Enter` 快捷发送）、历史发送回溯、自定义行尾（无 / `CRLF` / `LF` / `CR`）、**自动追加校验（Sum8 累加和 / XOR 异或 / Modbus CRC16）**、**周期定时发送**、**二进制/文本文件分块发送**，以及右侧 **12 路可导入导出 JSON 的多条快捷指令面板**。
- **独立 3D 可视化【🧭 IMU 姿态】页面**（`imu_studio.py`）：
  - 零 OpenGL 依赖的纯矢量抗锯齿 **3D 双轮平衡车实时姿态模型**（支持鼠标左键拖拽旋转视角、双击复位、车轮随直立 PWM 转动、超阈值车身变红告警）+ **航空人工地平仪（带 `±30°` 红色倾角保护刻度线）** + 底部实时滚动俯仰/横滚/角速度曲线；
  - 原生兼容 4 种数据源格式：① 平衡车工程 `Doc/main_balance_test.c` 四列格式（`target,pitch,gyro,pwm` 及 `#BALANCE:` 校准/保护状态横幅）、② 欧拉角三列（`pitch,roll,yaw`）、③ 六轴物理量（`g` 与 `°/s`）、④ `MPU6050` 原始寄存器计数（`16384 LSB/g`、`131 LSB/°/s`），六轴模式下内置与 `User/balance.c` 同款的实时一阶互补滤波器（可调时间常数 `τ`）及姿态动态仿真器；
  - 支持 **50 帧静止采样一键标定平衡车机械中值角**，并可**一键同步写入【PID 调参】页的 `ANG.TGT` 输入框**及复制 `MPU6050` 串口上报 C 代码片段。

---

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
