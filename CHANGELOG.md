# PID 调参助手 更新记录

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
