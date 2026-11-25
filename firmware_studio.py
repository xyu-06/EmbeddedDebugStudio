"""Interactive MCU PID interface code generator & protocol workbench module."""
from pathlib import Path
import re
import sys

from PySide6 import QtCore as C, QtGui, QtWidgets as W

from engine import LOOPS, MAX_CHANNELS

CHANNEL_ROLES = (
    ("target", "目标值 (Target)"),
    ("actual", "实际反馈 (Actual)"),
    ("output", "控制输出 (PWM/Output)"),
    ("error", "误差 (Error)"),
    ("other", "观察量 (Other)"),
)

ROLE_KEYS = tuple(k for k, _ in CHANNEL_ROLES)

PLATFORMS = (
    "STM32 HAL（推荐 · 零 printf_float 依赖）",
    "通用裸机 C99（标准库 / 自定义串口）",
    "Arduino / ESP32（Serial C++）",
)

INTERFACE_PRESETS = (
    (
        "⚡ 电机速度环 (4通道 SPD)",
        {
            "primary_loop": "SPD",
            "control_ms": 10,
            "report_ms": 20,
            "pwm_limit": 1000.0,
            "defaults": {"KP": 5.0, "KI": 30.0, "KD": 0.05, "TGT": 100.0},
            "channels": [
                ("target_rpm", "target"),
                ("actual_rpm", "actual"),
                ("pwm", "output"),
                ("error", "error"),
            ],
        },
    ),
    (
        "⚖ 平衡车双环 (6通道 ANG+SPD)",
        {
            "primary_loop": "ANG",
            "control_ms": 5,
            "report_ms": 15,
            "pwm_limit": 1000.0,
            "defaults": {"KP": 32.0, "KI": 0.0, "KD": 1.15, "TGT": 0.0},
            "channels": [
                ("pitch_tgt", "target"),
                ("pitch", "actual"),
                ("pwm", "output"),
                ("gyro", "other"),
                ("speed_tgt", "other"),
                ("speed", "other"),
            ],
        },
    ),
    (
        "🔀 自定义乱序示例 (PWM在前)",
        {
            "primary_loop": "SPD",
            "control_ms": 10,
            "report_ms": 20,
            "pwm_limit": 1000.0,
            "defaults": {"KP": 4.0, "KI": 20.0, "KD": 0.0, "TGT": 80.0},
            "channels": [
                ("pwm_out", "output"),
                ("err", "error"),
                ("wheel_rpm", "actual"),
                ("set_rpm", "target"),
            ],
        },
    ),
    (
        "🎯 位置伺服 / 云台 (4通道 ANG)",
        {
            "primary_loop": "ANG",
            "control_ms": 10,
            "report_ms": 20,
            "pwm_limit": 1000.0,
            "defaults": {"KP": 8.0, "KI": 1.5, "KD": 0.45, "TGT": 45.0},
            "channels": [
                ("pos_target", "target"),
                ("pos_actual", "actual"),
                ("drive_pwm", "output"),
                ("pos_err", "error"),
            ],
        },
    ),
)

PROTOCOL_GUIDE_HTML = """
<html><head><meta charset="utf-8"><style>
body { font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif; font-size: 10pt; line-height: 1.55; }
h2 { color: #197a63; margin-top: 14px; margin-bottom: 6px; font-size: 12pt; }
h3 { color: #225fc1; margin-top: 10px; margin-bottom: 4px; font-size: 10.5pt; }
table { border-collapse: collapse; width: 100%; margin: 6px 0 12px 0; }
th, td { border: 1px solid #8aa0b2; padding: 5px 8px; text-align: left; }
th { background: rgba(34, 95, 193, 0.12); font-weight: 600; }
code { font-family: Consolas, "Cascadia Mono", monospace; background: rgba(120,140,160,0.16); padding: 1px 4px; border-radius: 3px; }
pre { font-family: Consolas, monospace; background: rgba(120,140,160,0.12); padding: 8px 10px; border-radius: 5px; white-space: pre-wrap; }
.tip { padding: 8px 10px; border-left: 4px solid #197a63; background: rgba(25,122,99,0.10); margin: 8px 0; }
</style></head><body>
<h2>一、3 步把任意单片机接入上位机（支持任意列顺序）</h2>
<p>不必背诵通道顺序，也不必自己手写浮点串口解析——在左侧选好平台、填入你的通道名称与角色，点击右上角 <b>📦 一键导出全部文件</b>，放入工程按下面 3 步调用即可：</p>
<ol>
  <li><b>加入核心文件</b>：STM32/裸机把导出的 <code>pid_link.h/.c</code> 和 <code>pid_port_hal.h/.c</code>（或 <code>pid_port_generic.c</code>）加入编译，<b>无需勾选 <code>-u _printf_float</code></b>；Arduino/ESP32 直接打开 <code>EmbeddedDebugStudio_PID.ino</code>。</li>
  <li><b>串口与执行器挂钩</b>：在串口接收回调调用 <code>PID_Port_RxCpltCallback()</code>，在主循环调用 <code>PID_Port_Poll()</code>，并实现电机输出函数 <code>PID_Port_ApplyOutput(float pwm)</code>。</li>
  <li><b>按你配置的顺序上报浮点数组</b>：使用 <code>PID_ReportValues(ch, count)</code> 发送最多 8 个 <code>float</code>。因为头文件自动带有 <code>#NAMES:</code> 与 <code>#ROLES:</code> 角色声明，上位机会自动识别哪一列是<b>实际反馈</b>、哪一列是<b>目标值</b>、哪一列是<b>PWM输出</b>。</li>
</ol>
<div class="tip"><b>为什么建议开启 <code>#ROLES:</code>？</b><br>以前固定约定 CH1=目标、CH2=实际、CH3=PWM；开启 <code>#ROLES:</code> 后，即便你把 PWM 放在第 1 列、速度放在第 3 列、目标放在第 4 列，软件也能 100% 精准绑定，不会再选错通道。</div>

<h2>二、下位机 → 上位机 报文协议表</h2>
<table>
  <tr><th>报文</th><th>示例</th><th>说明</th></tr>
  <tr><td><code>v1,v2,...\n</code></td><td><code>650.0,100.0,98.2,1.8\n</code></td><td>1~8 通道实时采样行（支持可选前缀 <code>tag:值</code>）</td></tr>
  <tr><td><code>#NAMES:...\n</code></td><td><code>#NAMES:pwm,err,rpm,set\n</code></td><td>设置界面 1~8 通道默认显示名称</td></tr>
  <tr><td><code>#ROLES:...\n</code></td><td><code>#ROLES:output,error,actual,target\n</code></td><td><b>v1.8 新增</b>：声明各列角色（<code>target/actual/output/error/other</code>），任意列顺序都能自动绑定</td></tr>
  <tr><td><code>#CAP:PING\n</code></td><td><code>#CAP:PING\n</code></td><td>在 <code>@RUN</code> 时声明已启用 0.5 s 心跳看门狗（断线自动停机）</td></tr>
  <tr><td><code>#CAP:SET\n</code></td><td><code>#CAP:SET\n</code></td><td>声明支持原子组写入 <code>@&lt;GRP&gt;.SET=kp,ki,kd,tgt</code></td></tr>
  <tr><td><code>#STOP:&lt;reason&gt;\n</code></td><td><code>#STOP:heartbeat\n</code> / <code>#STOP:fall\n</code></td><td>下位机主动停机通知（超时停机 / 平衡车倒地保护），上位机弹出警示横幅</td></tr>
  <tr><td><code>@ACK:...</code> / <code>@PARAM:...</code></td><td><code>@ACK:SPD.SET=5,30,0.05,100\n</code></td><td>命令确认（框变绿）与参数读回（<code>@SPD.GET</code> 应答）</td></tr>
</table>

<h2>三、上位机 → 下位机 控制指令表</h2>
<table>
  <tr><th>指令</th><th>触发时机</th><th>下位机要求</th></tr>
  <tr><td><code>@RUN\n</code></td><td>点击【启动】或【自动整定】</td><td>使能控制输出、清零积分、重置心跳计时，回复 <code>#CAP:PING</code> / <code>#CAP:SET</code> / <code>@ACK:RUN</code></td></tr>
  <tr><td><code>@STOP\n</code></td><td>按 <code>Esc</code>、点【急停】或断开串口</td><td><b>立即清零电机 PWM 输出</b>、置 <code>pid_running = 0</code>，回复 <code>@ACK:STOP</code></td></tr>
  <tr><td><code>@PING\n</code></td><td>运行期间上位机每 200 ms 自动发</td><td>喂狗续命；若超过 <b>500 ms</b> 未收到 <code>@PING</code> 或 <code>@RUN</code>，必须停机清 PWM 并上报 <code>#STOP:heartbeat</code></td></tr>
  <tr><td><code>@SPD.GET\n</code></td><td>点击【读取】或重连成功</td><td>回复 <code>@PARAM:SPD.KP=...,SPD.KI=...,SPD.KD=...,SPD.TGT=...</code>（支持 <code>ANG/SPD/YAW/AUX</code>）</td></tr>
  <tr><td><code>@SPD.SET=kp,ki,kd,tgt\n</code></td><td>点击【发送本组】或自动整定切参</td><td>原子更新四项参数（全校验通过才写入）并回复 <code>@ACK:SPD.SET=...</code></td></tr>
  <tr><td><code>@SPD.KP=5.0\n</code> 等</td><td>回车修改单项或旧固件回退</td><td>支持单项设置 <code>KP / KI / KD / TGT</code> 及清积分 <code>@SPD.RST</code></td></tr>
</table>

<h2>四、常见问题排查</h2>
<ul>
  <li><b>只看到收到字节但示波器没有曲线？</b> 检查是否先点了 <b>【启动】(@RUN)</b>，或你的数据末尾是否带了换行符 <code>\n</code>。</li>
  <li><b>刚点【启动】半秒就提示心跳停机？</b> 确认在 <code>@RUN</code> 后至少 500 ms 内不要阻塞串口接收循环（不要在主循环里写 <code>HAL_Delay(600)</code>）。</li>
  <li><b>自动整定提示“误选目标通道”？</b> 若未接编码器直接把目标值赋给了反馈变量，或者没加 <code>#ROLES:</code> 且把目标值放到了反馈列，请修正列角色后重试。</li>
</ul>
</body></html>
"""


def load_core_firmware_file(filename: str) -> str:
    """Load firmware_example/<filename> from source tree or PyInstaller bundle."""
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    for root in (base, Path(__file__).resolve().parent):
        candidate = root / "firmware_example" / filename
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8")
    raise FileNotFoundError(f"未找到固件模板文件：{filename}")


def sanitize_channel_name(raw: str, idx: int) -> str:
    """Normalize arbitrary user text into a safe FireWater/#NAMES token."""
    cleaned = re.sub(r"[,:\r\n\s]+", "_", raw.strip())
    return (cleaned[:24] or f"CH{idx + 1}")


def build_protocol_headers(channels, emit_roles: bool = True) -> tuple[str, str]:
    """Return ('#NAMES:...', '#ROLES:...') strings for the given [(name, role), ...]."""
    names = [sanitize_channel_name(name, i) for i, (name, _) in enumerate(channels)]
    roles = [role if role in ROLE_KEYS else "other" for _, role in channels]
    names_line = "#NAMES:" + ",".join(names)
    roles_line = ("#ROLES:" + ",".join(roles)) if emit_roles else ""
    return names_line, roles_line


def _c_float(v: float) -> str:
    s = f"{float(v):.6g}"
    return s + ("f" if ("." in s or "e" in s or "E" in s) else ".0f")


def _channel_expr(role: str, idx: int) -> str:
    if role == "target":
        return "pid_loops[PRIMARY_PID_GROUP].target"
    if role == "actual":
        return "actual_val"
    if role == "output":
        return "applied_pwm"
    if role == "error":
        return "(pid_loops[PRIMARY_PID_GROUP].target - actual_val)"
    return f"0.0f /* TODO: CH{idx + 1} 自定义观测量 */"


def generate_interface_bundle(config: dict) -> dict[str, str]:
    """Generate complete C99/Arduino interface source files for the configured channels."""
    raw_channels = list(config.get("channels") or [("target", "target"), ("actual", "actual"), ("pwm", "output"), ("error", "error")])
    channels = [(sanitize_channel_name(n, i), r if r in ROLE_KEYS else "other") for i, (n, r) in enumerate(raw_channels[:MAX_CHANNELS])]
    if not channels:
        channels = [("actual", "actual")]

    platform = int(config.get("platform", 0))
    uart_handle = re.sub(r"[^A-Za-z0-9_]", "", str(config.get("uart_handle", "huart2")).strip()) or "huart2"
    baudrate = int(config.get("baudrate", 115200))
    control_ms = max(1, int(config.get("control_ms", 10)))
    report_ms = max(1, int(config.get("report_ms", 20)))
    heartbeat_ms = max(200, int(config.get("heartbeat_ms", 500)))
    pwm_limit = float(config.get("pwm_limit", 1000.0))
    primary_loop = str(config.get("primary_loop", "SPD")).upper()
    if primary_loop not in LOOPS:
        primary_loop = "SPD"
    defaults = config.get("defaults") or {"KP": 5.0, "KI": 30.0, "KD": 0.05, "TGT": 100.0}
    kp = float(defaults.get("KP", 5.0))
    ki = float(defaults.get("KI", 30.0))
    kd = float(defaults.get("KD", 0.05))
    tgt = float(defaults.get("TGT", 100.0))
    emit_roles = bool(config.get("emit_roles", True))

    names_hdr, roles_hdr = build_protocol_headers(channels, emit_roles=emit_roles)
    n_ch = len(channels)
    c_header_lines = f"{names_hdr}\\n" + (f"{roles_hdr}\\n" if emit_roles else "")
    fill_lines_c = "\n".join(
        f"    ch[{i}] = {_channel_expr(role, i)}; /* {name} ({role}) */"
        for i, (name, role) in enumerate(channels)
    )
    fill_lines_ino = "\n".join(
        f"    ch[{i}] = {_channel_expr(role, i).replace('float ', '')}; // {name} ({role})"
        for i, (name, role) in enumerate(channels)
    )

    readme = f"""# EmbeddedDebugStudio 快速接入说明

- 生成配置：共 **{n_ch} 通道** · 主调环路 **`{primary_loop}`** · 控制周期 **`{control_ms} ms`** · 上报周期 **`{report_ms} ms`** · PWM 限幅 **`{pwm_limit:g}`** · 波特率 **`{baudrate}`**
- 协议声明：
  - `{names_hdr}`
  {f'- `{roles_hdr}`' if emit_roles else '- （未启用 `#ROLES:`，上位机将按通道名称语义自动匹配）'}

## 实车安全清单
1. 首次运行 **【⚡ 自动整定 PID…】** 前请先将车轮架空或加上机械限位。
2. 串口连上后在右侧选参数组 **`{primary_loop}`**，点 **【读取】** 确认默认参数回传（绿框），再点 **【启动】** 与 `Esc` 急停验证停机有效。
3. 拔掉串口线后下位机应在 `500 ms` 内自动触发 `#STOP:heartbeat` 并清零电机 PWM。
"""

    if platform == 0:
        pid_port_hal_h = f"""#ifndef PID_PORT_HAL_H_
#define PID_PORT_HAL_H_

#include "main.h"
#include "pid_link.h"

#define PRIMARY_PID_GROUP PID_{primary_loop}
#define PID_PORT_CHANNELS {n_ch}U
#define PID_PORT_CONTROL_MS {control_ms}U
#define PID_PORT_REPORT_MS {report_ms}U
#define PID_PORT_PWM_LIMIT {pwm_limit:.1f}f

/** @brief 初始化串口调参接口（零 printf_float 依赖，自动声明通道名称与角色）。 */
void PID_Port_Init(UART_HandleTypeDef *uart);

/** @brief 在主循环高频调用：解析命令、喂狗/停机保护、发送回复与波形。 */
void PID_Port_Poll(void);

/** @brief 在 HAL_UART_RxCpltCallback 中转发调用。 */
void PID_Port_RxCpltCallback(UART_HandleTypeDef *uart);

/** @brief 在 HAL_UART_ErrorCallback 中转发调用。 */
void PID_Port_ErrorCallback(UART_HandleTypeDef *uart);

/** @brief 用户硬件输出钩子：将计算好的控制量（已限幅，停机时为 0）写入电机驱动。 */
void PID_Port_ApplyOutput(float value);

/** @brief 使用本页配置的通道顺序直接上报 {n_ch} 路浮点数据。 */
void PID_Port_ReportSample(float actual_val, float applied_pwm);

#endif
"""

        pid_port_hal_c = f"""#include "pid_port_hal.h"
#include <string.h>

#define PID_PORT_RX_FIFO_SIZE 256U
#define PID_PORT_TX_FRAME_LEN 192U
#define PID_PORT_REPLY_SLOTS  8U

typedef struct {{
    uint16_t length;
    uint8_t data[PID_PORT_TX_FRAME_LEN];
}} PidTxFrame;

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static volatile uint8_t s_fifo[PID_PORT_RX_FIFO_SIZE];
static volatile uint8_t s_rd, s_wr, s_overflow, s_rearm;

static PidTxFrame s_replies[PID_PORT_REPLY_SLOTS];
static PidTxFrame s_telemetry;
static PidTxFrame s_in_flight;
static uint8_t s_reply_head, s_reply_count, s_tx_overflow;
static uint8_t s_prev_running;

void PID_ZeroOutputs(void)
{{
    PID_Port_ApplyOutput(0.0f);
}}

void PID_Write(const char *data, unsigned len)
{{
    if (!data || len == 0U || len > sizeof(s_in_flight.data)) {{
        return;
    }}
    PidTxFrame *frame = &s_telemetry;
    if (data[0] == '@' || data[0] == '#') {{
        if (s_reply_count >= PID_PORT_REPLY_SLOTS) {{
            s_tx_overflow = 1U;
            return;
        }}
        frame = &s_replies[(s_reply_head + s_reply_count) % PID_PORT_REPLY_SLOTS];
        s_reply_count++;
    }}
    memcpy(frame->data, data, len);
    frame->length = (uint16_t)len;
}}

static void SendChannelHeader(void)
{{
    static const char hdr[] = "{c_header_lines}";
    PID_Write(hdr, (unsigned)(sizeof(hdr) - 1U));
}}

static void ServiceTx(void)
{{
    if (!s_uart || s_uart->gState != HAL_UART_STATE_READY) {{
        return;
    }}
    if (s_tx_overflow && s_reply_count < PID_PORT_REPLY_SLOTS) {{
        static const char warn[] = "#WARN:reply queue overflow\\n";
        s_tx_overflow = 0U;
        PID_Write(warn, (unsigned)(sizeof(warn) - 1U));
    }}
    PidTxFrame *next = (s_reply_count != 0U) ? &s_replies[s_reply_head] : &s_telemetry;
    if (next->length == 0U) {{
        return;
    }}
    s_in_flight = *next;
    if (HAL_UART_Transmit_IT(s_uart, s_in_flight.data, s_in_flight.length) == HAL_OK) {{
        next->length = 0U;
        if (s_reply_count != 0U) {{
            s_reply_head = (uint8_t)((s_reply_head + 1U) % PID_PORT_REPLY_SLOTS);
            s_reply_count--;
        }}
    }}
}}

void PID_Port_Init(UART_HandleTypeDef *uart)
{{
    s_uart = uart;
    s_rd = s_wr = s_overflow = s_prev_running = 0U;
    s_reply_head = s_reply_count = s_tx_overflow = 0U;
    s_telemetry.length = 0U;

    pid_loops[PRIMARY_PID_GROUP].kp = {_c_float(kp)};
    pid_loops[PRIMARY_PID_GROUP].ki = {_c_float(ki)};
    pid_loops[PRIMARY_PID_GROUP].kd = {_c_float(kd)};
    pid_loops[PRIMARY_PID_GROUP].target = {_c_float(tgt)};

    PID_ZeroOutputs();
    PID_WatchdogTick(HAL_GetTick());
    PID_CheckFloatFormat();
    s_rearm = (uint8_t)(HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK);
    SendChannelHeader();
}}

void PID_Port_RxCpltCallback(UART_HandleTypeDef *uart)
{{
    if (uart != s_uart) {{
        return;
    }}
    uint8_t next = (uint8_t)(s_wr + 1U);
    if (next == s_rd) {{
        s_overflow = 1U;
    }} else {{
        s_fifo[s_wr] = s_rx_byte;
        s_wr = next;
    }}
    if (HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK) {{
        s_rearm = 1U;
    }}
}}

void PID_Port_ErrorCallback(UART_HandleTypeDef *uart)
{{
    if (uart == s_uart) {{
        s_overflow = 1U;
        s_rearm = 1U;
    }}
}}

void PID_Port_Poll(void)
{{
    PID_WatchdogTick(HAL_GetTick());
    if (s_rearm && s_uart) {{
        HAL_UART_AbortReceive(s_uart);
        s_rearm = (uint8_t)(HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK);
    }}
    for (unsigned drained = 0U; drained < PID_PORT_RX_FIFO_SIZE; ++drained) {{
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        uint8_t lost = s_overflow;
        if (lost) {{
            s_rd = s_wr;
            s_overflow = 0U;
        }}
        uint8_t has_byte = (uint8_t)(s_rd != s_wr);
        uint8_t b = 0U;
        if (has_byte) {{
            b = s_fifo[s_rd];
            s_rd = (uint8_t)(s_rd + 1U);
        }}
        __set_PRIMASK(mask);
        if (lost) {{
            PID_DiscardPartial();
        }}
        if (!has_byte) {{
            break;
        }}
        PID_Consume(b);
    }}
    if (pid_running && !s_prev_running) {{
        SendChannelHeader();
    }}
    s_prev_running = pid_running;
    ServiceTx();
}}

void PID_Port_ReportSample(float actual_val, float applied_pwm)
{{
    float ch[PID_PORT_CHANNELS];
{fill_lines_c}
    PID_ReportValues(ch, PID_PORT_CHANNELS);
}}
"""

        main_example = f"""/* 将本示例的关键段落合入你的 main.c（串口波特率建议设为 {baudrate}） */
#include "main.h"
#include "pid_port_hal.h"

extern UART_HandleTypeDef {uart_handle};

static float s_applied_pwm = 0.0f;

/* 1. 实现电机/执行器输出（停机或超时时会自动传 0） */
void PID_Port_ApplyOutput(float value)
{{
    s_applied_pwm = value;
    /* TODO: 替换为你的电机输出函数，例如：
     * Motor_SetLeftPWM((int16_t)value);
     */
}}

/* 2. 在 HAL 串口回调里转发单字节中断与错误恢复 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{{
    PID_Port_RxCpltCallback(huart);
}}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{{
    PID_Port_ErrorCallback(huart);
}}

/* 3. 在 main() 初始化与 while(1) 中调用 */
void App_PidTuningDemo(void)
{{
    PID_Port_Init(&{uart_handle});
    uint32_t last_ctrl_ms = HAL_GetTick();
    uint32_t last_rep_ms = last_ctrl_ms;

    while (1) {{
        PID_Port_Poll();

        uint32_t now = HAL_GetTick();
        if (now - last_ctrl_ms >= PID_PORT_CONTROL_MS) {{
            float dt = (float)(now - last_ctrl_ms) * 0.001f;
            last_ctrl_ms = now;

            float actual_val = 0.0f; /* TODO: 替换为真实传感器读数（如左轮 RPM / 俯仰角） */
            float out = PID_Position(&pid_loops[PRIMARY_PID_GROUP], actual_val, dt, PID_PORT_PWM_LIMIT);
            PID_Port_ApplyOutput(out);
        }}

        if (now - last_rep_ms >= PID_PORT_REPORT_MS) {{
            last_rep_ms = now;
            float actual_val = 0.0f; /* TODO: 与上方保持同一最新反馈值 */
            PID_Port_ReportSample(actual_val, s_applied_pwm);
        }}
    }}
}}
"""
        return {
            "main_integration_example.c": main_example,
            "pid_port_hal.h": pid_port_hal_h,
            "pid_port_hal.c": pid_port_hal_c,
            "pid_link.h": load_core_firmware_file("pid_link.h"),
            "pid_link.c": load_core_firmware_file("pid_link.c"),
            "README_接入说明.md": readme,
        }

    if platform == 1:
        generic_c = f"""/* 通用裸机 C99 接入层：只需实现下方 3 个硬件函数，即可配合 pid_link.c 使用 */
#include <stdint.h>
#include "pid_link.h"

#define PRIMARY_PID_GROUP PID_{primary_loop}
#define PID_PORT_CHANNELS {n_ch}U
#define PID_PORT_PWM_LIMIT {pwm_limit:.1f}f

/* ---------- TODO 1/2/3：替换为你的单片机底层驱动 ---------- */
extern void UART_SendBytes(const char *data, unsigned len);
extern uint32_t GetTickMs(void);
extern void Motor_ApplyPwm(float pwm);
/* ------------------------------------------------------------ */

static float s_applied_pwm = 0.0f;
static uint8_t s_prev_running = 0U;

void PID_Write(const char *data, unsigned len) {{ UART_SendBytes(data, len); }}
void PID_ZeroOutputs(void) {{ s_applied_pwm = 0.0f; Motor_ApplyPwm(0.0f); }}

static void SendHeader(void)
{{
    static const char hdr[] = "{c_header_lines}";
    PID_Write(hdr, (unsigned)(sizeof(hdr) - 1U));
}}

void PidPort_Init(void)
{{
    pid_loops[PRIMARY_PID_GROUP].kp = {_c_float(kp)};
    pid_loops[PRIMARY_PID_GROUP].ki = {_c_float(ki)};
    pid_loops[PRIMARY_PID_GROUP].kd = {_c_float(kd)};
    pid_loops[PRIMARY_PID_GROUP].target = {_c_float(tgt)};
    PID_ZeroOutputs();
    PID_WatchdogTick(GetTickMs());
    SendHeader();
}}

/* 在串口接收中断里把每个字节喂给 PID_Consume(byte)（或在主循环从环形缓冲取） */
void PidPort_OnControlStep(float actual_val, float dt_s)
{{
    PID_WatchdogTick(GetTickMs());
    if (pid_running && !s_prev_running) {{
        SendHeader();
    }}
    s_prev_running = pid_running;

    float out = PID_Position(&pid_loops[PRIMARY_PID_GROUP], actual_val, dt_s, PID_PORT_PWM_LIMIT);
    s_applied_pwm = out;
    Motor_ApplyPwm(out);
}}

void PidPort_Report(float actual_val)
{{
    float ch[PID_PORT_CHANNELS];
    float applied_pwm = s_applied_pwm;
{fill_lines_c}
    PID_ReportValues(ch, PID_PORT_CHANNELS);
}}
"""
        return {
            "pid_port_generic.c": generic_c,
            "pid_link.h": load_core_firmware_file("pid_link.h"),
            "pid_link.c": load_core_firmware_file("pid_link.c"),
            "README_接入说明.md": readme,
        }

    # Platform 2: Arduino / ESP32 sketch
    loop_idx = LOOPS.index(primary_loop)
    ino = f"""/* EmbeddedDebugStudio 兼容串口 PID 接口（Arduino / ESP32）
 * 支持：#NAMES / #ROLES 任意列顺序、#CAP:PING ({heartbeat_ms} ms 保护)、#CAP:SET 原子写入
 */
#include <Arduino.h>

struct PidLoop {{
  float kp, ki, kd, target;
  float integral, prev_actual, output;
  bool initialized;
}};

enum {{ PID_ANG = 0, PID_SPD = 1, PID_YAW = 2, PID_AUX = 3 }};
static const char *LOOP_NAMES[4] = {{"ANG", "SPD", "YAW", "AUX"}};
static const int PRIMARY_PID_GROUP = {loop_idx}; // {primary_loop}
static PidLoop pid_loops[4];
static bool pid_running = false;
static unsigned long last_ping_ms = 0;
static float applied_pwm = 0.0f;
static String rx_buf;

void sendHeader() {{
  Serial.print(F("{c_header_lines}"));
}}

void applyMotorPwm(float pwm) {{
  applied_pwm = constrain(pwm, -{pwm_limit:.1f}f, {pwm_limit:.1f}f);
  // TODO: 在这里把 applied_pwm 输出到你的电机/舵机引脚
}}

void stopMotor(const char *reason = nullptr) {{
  pid_running = false;
  applied_pwm = 0.0f;
  applyMotorPwm(0.0f);
  for (int i = 0; i < 4; ++i) {{
    pid_loops[i].integral = 0.0f;
    pid_loops[i].initialized = false;
    pid_loops[i].output = 0.0f;
  }}
  if (reason) {{
    Serial.print(F("#STOP:"));
    Serial.println(reason);
  }}
}}

float computePid(PidLoop &p, float actual, float dt) {{
  if (!pid_running || dt <= 0.0f) {{
    p.output = 0.0f;
    p.prev_actual = actual;
    return 0.0f;
  }}
  float err = p.target - actual;
  float deriv = p.initialized ? (actual - p.prev_actual) / dt : 0.0f;
  p.prev_actual = actual;
  p.initialized = true;
  float trial_i = p.integral + p.ki * err * dt;
  float raw = p.kp * err + trial_i - p.kd * deriv;
  float out = constrain(raw, -{pwm_limit:.1f}f, {pwm_limit:.1f}f);
  if (fabsf(raw) <= {pwm_limit:.1f}f || (raw - out) * p.ki * err < 0.0f) {{
    p.integral = trial_i;
  }}
  p.output = out;
  return out;
}}

void handleCommand(String line) {{
  line.trim();
  if (!line.startsWith("@")) return;
  String body = line.substring(1);
  if (body == "PING") {{ last_ping_ms = millis(); return; }}
  if (body == "STOP") {{ stopMotor(); Serial.println(F("@ACK:STOP")); return; }}
  if (body == "RUN") {{
    last_ping_ms = millis();
    if (!pid_running) {{
      for (auto &lp : pid_loops) {{ lp.integral = 0.0f; lp.initialized = false; }}
      pid_running = true;
    }}
    sendHeader();
    Serial.println(F("#CAP:PING"));
    Serial.println(F("#CAP:SET"));
    Serial.println(F("@ACK:RUN"));
    return;
  }}

  int grp = PRIMARY_PID_GROUP;
  int dot = body.indexOf('.');
  int eq = body.indexOf('=');
  if (dot > 0 && (eq < 0 || dot < eq)) {{
    String g = body.substring(0, dot);
    grp = -1;
    for (int i = 0; i < 4; ++i) if (g == LOOP_NAMES[i]) grp = i;
    if (grp < 0) {{ Serial.println(F("@ERR:unknown loop")); return; }}
    body = body.substring(dot + 1);
    eq = body.indexOf('=');
  }}
  PidLoop &lp = pid_loops[grp];
  if (body == "GET") {{
    Serial.printf("@PARAM:%s.KP=%.4f,%s.KI=%.4f,%s.KD=%.4f,%s.TGT=%.4f\\n",
                  LOOP_NAMES[grp], lp.kp, LOOP_NAMES[grp], lp.ki,
                  LOOP_NAMES[grp], lp.kd, LOOP_NAMES[grp], lp.target);
    return;
  }}
  if (body == "RST") {{
    lp.integral = 0.0f; lp.initialized = false;
    Serial.print(F("@ACK:")); Serial.println(line.substring(1));
    return;
  }}
  if (eq < 0) {{ Serial.println(F("@ERR:expected key=value")); return; }}
  String k = body.substring(0, eq);
  String v = body.substring(eq + 1);
  if (k == "SET") {{
    int c1 = v.indexOf(','), c2 = v.indexOf(',', c1 + 1), c3 = v.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) {{ Serial.println(F("@ERR:expected kp,ki,kd,tgt")); return; }}
    lp.kp = v.substring(0, c1).toFloat();
    lp.ki = v.substring(c1 + 1, c2).toFloat();
    lp.kd = v.substring(c2 + 1, c3).toFloat();
    lp.target = v.substring(c3 + 1).toFloat();
    Serial.print(F("@ACK:")); Serial.println(line.substring(1));
    return;
  }}
  float val = v.toFloat();
  if (k == "KP") lp.kp = val;
  else if (k == "KI") lp.ki = val;
  else if (k == "KD") lp.kd = val;
  else if (k == "TGT") lp.target = val;
  else {{ Serial.println(F("@ERR:unknown key")); return; }}
  Serial.print(F("@ACK:")); Serial.println(line.substring(1));
}}

void reportTelemetry(float actual_val) {{
  float ch[{n_ch}];
{fill_lines_ino}
  for (int i = 0; i < {n_ch}; ++i) {{
    if (i) Serial.print(',');
    Serial.print(ch[i], 4);
  }}
  Serial.println();
}}

void setup() {{
  Serial.begin({baudrate});
  pid_loops[PRIMARY_PID_GROUP] = {{{_c_float(kp)}, {_c_float(ki)}, {_c_float(kd)}, {_c_float(tgt)}, 0, 0, 0, false}};
  sendHeader();
}}

void loop() {{
  while (Serial.available()) {{
    char c = (char)Serial.read();
    if (c == '\\n') {{ handleCommand(rx_buf); rx_buf = ""; }}
    else if (c != '\\r' && rx_buf.length() < 120) rx_buf += c;
  }}
  unsigned long now = millis();
  if (pid_running && now - last_ping_ms >= {heartbeat_ms}UL) stopMotor("heartbeat");

  static unsigned long last_ctrl = 0, last_rep = 0;
  if (now - last_ctrl >= {control_ms}UL) {{
    float dt = (now - last_ctrl) * 0.001f;
    last_ctrl = now;
    float actual_val = 0.0f; // TODO: 读取传感器（如编码器 RPM / 角度）
    applyMotorPwm(computePid(pid_loops[PRIMARY_PID_GROUP], actual_val, dt));
  }}
  if (now - last_rep >= {report_ms}UL) {{
    last_rep = now;
    float actual_val = 0.0f; // TODO: 与上方实际值一致
    reportTelemetry(actual_val);
  }}
}}
"""
    return {
        "EmbeddedDebugStudio_PID.ino": ino,
        "README_接入说明.md": readme,
    }


class FirmwareStudioPage(W.QWidget):
    """5th Workbench module: one-click MCU PID interface generator & protocol manual."""

    module_id = "firmware_builder"
    module_title = "🧩 接口构建"

    def __init__(self, host=None):
        super().__init__()
        self.host = host
        self._bundle: dict[str, str] = {}
        self._suppress_regen = False
        self._channel_rows: list[tuple[W.QLabel, W.QLineEdit, W.QComboBox]] = []
        self._selected_row = 0
        self._build_ui()
        self.apply_preset(0)

    def _build_ui(self):
        root = W.QHBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(10)

        # ---------- Left scrollable builder config ----------
        left_scroll = W.QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(W.QFrame.Shape.NoFrame)
        left_scroll.setFixedWidth(400)
        left_box = W.QWidget()
        left_layout = W.QVBoxLayout(left_box)
        left_layout.setContentsMargins(2, 2, 6, 2)
        left_layout.setSpacing(8)
        left_scroll.setWidget(left_box)
        root.addWidget(left_scroll)

        # Quick presets card
        preset_card = W.QFrame()
        preset_card.setObjectName("card")
        pv = W.QVBoxLayout(preset_card)
        pv.setContentsMargins(10, 8, 10, 8)
        pv.setSpacing(6)
        pv.addWidget(W.QLabel("<b>⚡ 场景预设模板（一键填充）</b>"))
        pgrid = W.QGridLayout()
        pgrid.setSpacing(6)
        for idx, (label, _) in enumerate(INTERFACE_PRESETS):
            btn = W.QPushButton(label)
            btn.clicked.connect(lambda _=False, i=idx: self.apply_preset(i))
            pgrid.addWidget(btn, idx // 2, idx % 2)
        pv.addLayout(pgrid)
        left_layout.addWidget(preset_card)

        # Platform & loop card
        cfg_card = W.QFrame()
        cfg_card.setObjectName("card")
        cf = W.QFormLayout(cfg_card)
        cf.setContentsMargins(10, 8, 10, 8)
        cf.setSpacing(6)
        cf.addRow(W.QLabel("<b>⚙ 目标工程与默认参数</b>"))

        self.platform_box = W.QComboBox()
        self.platform_box.addItems(list(PLATFORMS))
        self.uart_edit = W.QLineEdit("huart2")
        self.baud_box = W.QComboBox()
        self.baud_box.setEditable(True)
        self.baud_box.addItems(["115200", "256000", "460800", "921600", "9600"])
        self.loop_box = W.QComboBox()
        self.loop_box.addItems(list(LOOPS))
        self.loop_box.setCurrentText("SPD")

        self.ctrl_ms = W.QSpinBox()
        self.ctrl_ms.setRange(1, 200)
        self.ctrl_ms.setValue(10)
        self.rep_ms = W.QSpinBox()
        self.rep_ms.setRange(1, 500)
        self.rep_ms.setValue(20)
        self.pwm_lim = W.QDoubleSpinBox()
        self.pwm_lim.setRange(1.0, 100000.0)
        self.pwm_lim.setDecimals(1)
        self.pwm_lim.setValue(1000.0)

        self.def_kp = W.QDoubleSpinBox()
        self.def_ki = W.QDoubleSpinBox()
        self.def_kd = W.QDoubleSpinBox()
        self.def_tgt = W.QDoubleSpinBox()
        for sp, val in ((self.def_kp, 5.0), (self.def_ki, 30.0), (self.def_kd, 0.05), (self.def_tgt, 100.0)):
            sp.setRange(-100000.0, 100000.0)
            sp.setDecimals(4)
            sp.setValue(val)

        self.emit_roles_chk = W.QCheckBox("上报 #ROLES: 角色声明（支持任意列顺序）")
        self.emit_roles_chk.setChecked(True)

        cf.addRow("目标平台", self.platform_box)
        cf.addRow("UART 句柄", self.uart_edit)
        cf.addRow("波特率", self.baud_box)
        cf.addRow("默认调参环路", self.loop_box)
        cf.addRow("控制 / 上报周期 ms", self._pair_row(self.ctrl_ms, self.rep_ms))
        cf.addRow("PWM 输出限幅", self.pwm_lim)
        cf.addRow("初始 KP / KI", self._pair_row(self.def_kp, self.def_ki))
        cf.addRow("初始 KD / TGT", self._pair_row(self.def_kd, self.def_tgt))
        cf.addRow(self.emit_roles_chk)
        left_layout.addWidget(cfg_card)

        # Channels card
        ch_card = W.QFrame()
        ch_card.setObjectName("card")
        cv = W.QVBoxLayout(ch_card)
        cv.setContentsMargins(10, 8, 10, 8)
        cv.setSpacing(6)
        cv.addWidget(W.QLabel("<b>📡 串口上报通道定义（1～8 通道，可任意调序）</b>"))

        btn_row = W.QHBoxLayout()
        self.add_ch_btn = W.QPushButton("+ 添加通道")
        self.del_ch_btn = W.QPushButton("- 删除通道")
        self.up_ch_btn = W.QPushButton("↑ 上移")
        self.dn_ch_btn = W.QPushButton("↓ 下移")
        for b in (self.add_ch_btn, self.del_ch_btn, self.up_ch_btn, self.dn_ch_btn):
            btn_row.addWidget(b)
        cv.addLayout(btn_row)

        self.ch_rows_box = W.QVBoxLayout()
        self.ch_rows_box.setSpacing(4)
        cv.addLayout(self.ch_rows_box)

        self.row_selector = W.QSpinBox()
        self.row_selector.setRange(1, 1)
        row_sel = W.QHBoxLayout()
        row_sel.addWidget(W.QLabel("当前操作行：CH"))
        row_sel.addWidget(self.row_selector)
        row_sel.addStretch(1)
        cv.addLayout(row_sel)

        hint = W.QLabel("提示：无论你怎么调换 CH1～CH8 的位置，只要角色选对并勾选 #ROLES:，上位机自动识别反馈与 PWM。")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#5b7083;font-size:9pt;")
        cv.addWidget(hint)
        left_layout.addWidget(ch_card)
        left_layout.addStretch(1)

        # ---------- Right output code + protocol docs ----------
        right_box = W.QWidget()
        right_layout = W.QVBoxLayout(right_box)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)
        root.addWidget(right_box, 1)

        # Top action bar
        top_bar = W.QFrame()
        top_bar.setObjectName("top")
        tb = W.QHBoxLayout(top_bar)
        tb.setContentsMargins(10, 6, 10, 6)
        self.proto_badge = W.QLabel()
        self.proto_badge.setStyleSheet("font-weight:600;color:#176a57;")
        tb.addWidget(self.proto_badge, 1)
        self.copy_file_btn = W.QPushButton("📋 复制当前文件")
        self.export_dir_btn = W.QPushButton("📦 一键导出全部文件…")
        self.export_dir_btn.setObjectName("accent")
        self.goto_pid_btn = W.QPushButton("📈 去 PID 页联调")
        self.goto_pid_btn.setObjectName("primary")
        tb.addWidget(self.copy_file_btn)
        tb.addWidget(self.export_dir_btn)
        tb.addWidget(self.goto_pid_btn)
        right_layout.addWidget(top_bar)

        self.tabs = W.QTabWidget()
        right_layout.addWidget(self.tabs, 1)

        # Tab 1: Code preview
        code_tab = W.QWidget()
        ct = W.QVBoxLayout(code_tab)
        ct.setContentsMargins(6, 6, 6, 6)
        file_bar = W.QHBoxLayout()
        file_bar.addWidget(W.QLabel("预览生成文件："))
        self.file_combo = W.QComboBox()
        file_bar.addWidget(self.file_combo, 1)
        ct.addLayout(file_bar)

        self.code_edit = W.QPlainTextEdit()
        self.code_edit.setReadOnly(True)
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        font.setPointSize(10)
        self.code_edit.setFont(font)
        self.code_edit.setLineWrapMode(W.QPlainTextEdit.LineWrapMode.NoWrap)
        ct.addWidget(self.code_edit, 1)
        self.tabs.addTab(code_tab, "💻 快速接入代码")

        # Tab 2: Built-in protocol guide
        doc_tab = W.QWidget()
        dt = W.QVBoxLayout(doc_tab)
        dt.setContentsMargins(6, 6, 6, 6)
        self.guide_view = W.QTextBrowser()
        self.guide_view.setHtml(PROTOCOL_GUIDE_HTML)
        dt.addWidget(self.guide_view, 1)
        self.tabs.addTab(doc_tab, "📖 协议与接口说明")

        # Connections
        for sig in (
            self.platform_box.currentIndexChanged,
            self.uart_edit.textChanged,
            self.baud_box.currentTextChanged,
            self.loop_box.currentIndexChanged,
            self.ctrl_ms.valueChanged,
            self.rep_ms.valueChanged,
            self.pwm_lim.valueChanged,
            self.def_kp.valueChanged,
            self.def_ki.valueChanged,
            self.def_kd.valueChanged,
            self.def_tgt.valueChanged,
            self.emit_roles_chk.toggled,
        ):
            sig.connect(self.regenerate)
        self.file_combo.currentIndexChanged.connect(self._show_selected_file)
        self.copy_file_btn.clicked.connect(self.copy_current_file)
        self.export_dir_btn.clicked.connect(self.choose_and_export)
        self.goto_pid_btn.clicked.connect(lambda: self.host.switch_module(0) if self.host else None)
        self.add_ch_btn.clicked.connect(self.add_channel)
        self.del_ch_btn.clicked.connect(self.remove_channel)
        self.up_ch_btn.clicked.connect(lambda: self.move_channel(-1))
        self.dn_ch_btn.clicked.connect(lambda: self.move_channel(1))
        self.row_selector.valueChanged.connect(lambda v: setattr(self, "_selected_row", v - 1))

    @staticmethod
    def _pair_row(w1: W.QWidget, w2: W.QWidget) -> W.QWidget:
        box = W.QWidget()
        row = W.QHBoxLayout(box)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.addWidget(w1, 1)
        row.addWidget(w2, 1)
        return box

    def _set_channels(self, channels):
        self._suppress_regen = True
        try:
            while self._channel_rows:
                row_w, _, _ = self._channel_rows.pop()
                self.ch_rows_box.removeWidget(row_w)
                row_w.setParent(None)
                row_w.deleteLater()

            for idx, (name, role) in enumerate(channels[:MAX_CHANNELS]):
                row_w = W.QWidget()
                row = W.QHBoxLayout(row_w)
                row.setContentsMargins(0, 0, 0, 0)
                row.setSpacing(5)
                lbl = W.QLabel(f"CH{idx + 1}")
                lbl.setFixedWidth(30)
                ed = W.QLineEdit(str(name))
                ed.setMaxLength(24)
                cb = W.QComboBox()
                for rk, rlabel in CHANNEL_ROLES:
                    cb.addItem(rlabel, rk)
                pos = cb.findData(role)
                cb.setCurrentIndex(pos if pos >= 0 else 4)
                ed.textChanged.connect(self.regenerate)
                cb.currentIndexChanged.connect(self.regenerate)
                row.addWidget(lbl)
                row.addWidget(ed, 1)
                row.addWidget(cb, 1)
                self.ch_rows_box.addWidget(row_w)
                self._channel_rows.append((row_w, ed, cb))

            self.row_selector.setRange(1, max(1, len(self._channel_rows)))
            self.row_selector.setValue(min(self.row_selector.value(), len(self._channel_rows)))
            self._selected_row = self.row_selector.value() - 1
            self.add_ch_btn.setEnabled(len(self._channel_rows) < MAX_CHANNELS)
            self.del_ch_btn.setEnabled(len(self._channel_rows) > 1)
        finally:
            self._suppress_regen = False
        self.regenerate()

    def channels_data(self) -> list[tuple[str, str]]:
        return [
            (ed.text(), str(cb.currentData() or "other"))
            for _, ed, cb in self._channel_rows
        ]

    def add_channel(self):
        ch = self.channels_data()
        if len(ch) >= MAX_CHANNELS:
            return
        ch.append((f"ch{len(ch) + 1}", "other"))
        self._set_channels(ch)
        self.row_selector.setValue(len(ch))

    def remove_channel(self):
        ch = self.channels_data()
        if len(ch) <= 1:
            return
        idx = max(0, min(len(ch) - 1, self.row_selector.value() - 1))
        ch.pop(idx)
        self._set_channels(ch)

    def move_channel(self, delta: int):
        ch = self.channels_data()
        idx = max(0, min(len(ch) - 1, self.row_selector.value() - 1))
        nxt = idx + delta
        if not (0 <= nxt < len(ch)):
            return
        ch[idx], ch[nxt] = ch[nxt], ch[idx]
        self._set_channels(ch)
        self.row_selector.setValue(nxt + 1)

    def apply_preset(self, index: int):
        _, preset = INTERFACE_PRESETS[max(0, min(len(INTERFACE_PRESETS) - 1, index))]
        self._suppress_regen = True
        try:
            self.loop_box.setCurrentText(preset["primary_loop"])
            self.ctrl_ms.setValue(preset["control_ms"])
            self.rep_ms.setValue(preset["report_ms"])
            self.pwm_lim.setValue(preset["pwm_limit"])
            d = preset["defaults"]
            self.def_kp.setValue(d["KP"])
            self.def_ki.setValue(d["KI"])
            self.def_kd.setValue(d["KD"])
            self.def_tgt.setValue(d["TGT"])
        finally:
            self._suppress_regen = False
        self._set_channels(preset["channels"])

    def config_dict(self) -> dict:
        try:
            baud = int(self.baud_box.currentText().strip())
        except ValueError:
            baud = 115200
        return {
            "platform": self.platform_box.currentIndex(),
            "uart_handle": self.uart_edit.text(),
            "baudrate": baud,
            "primary_loop": self.loop_box.currentText(),
            "control_ms": self.ctrl_ms.value(),
            "report_ms": self.rep_ms.value(),
            "pwm_limit": self.pwm_lim.value(),
            "defaults": {
                "KP": self.def_kp.value(),
                "KI": self.def_ki.value(),
                "KD": self.def_kd.value(),
                "TGT": self.def_tgt.value(),
            },
            "emit_roles": self.emit_roles_chk.isChecked(),
            "channels": self.channels_data(),
        }

    def current_bundle(self) -> dict[str, str]:
        return dict(self._bundle)

    def regenerate(self):
        if self._suppress_regen:
            return
        cfg = self.config_dict()
        names_hdr, roles_hdr = build_protocol_headers(cfg["channels"], emit_roles=cfg["emit_roles"])
        self.proto_badge.setText(f"{names_hdr}" + (f"   |   {roles_hdr}" if roles_hdr else ""))
        self._bundle = generate_interface_bundle(cfg)

        prev_file = self.file_combo.currentText()
        with C.QSignalBlocker(self.file_combo):
            self.file_combo.clear()
            self.file_combo.addItems(list(self._bundle.keys()))
            idx = self.file_combo.findText(prev_file)
            self.file_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self._show_selected_file()

    def _show_selected_file(self):
        name = self.file_combo.currentText()
        self.code_edit.setPlainText(self._bundle.get(name, ""))

    def copy_current_file(self):
        text = self.code_edit.toPlainText()
        if not text:
            return
        W.QApplication.clipboard().setText(text)
        fname = self.file_combo.currentText()
        if self.host is not None and hasattr(self.host, "feedback"):
            self.host.feedback(f"已复制接口文件 {fname} 到剪贴板")

    def export_bundle(self, target_dir) -> list[Path]:
        out_dir = Path(target_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        written = []
        for fname, content in self._bundle.items():
            dst = out_dir / fname
            dst.write_text(content, encoding="utf-8")
            written.append(dst)
        return written

    def choose_and_export(self):
        folder = W.QFileDialog.getExistingDirectory(
            self,
            "选择导出目录（将自动写入接口源码与说明）",
            "",
            W.QFileDialog.Option.DontUseNativeDialog,
        )
        if not folder:
            return
        try:
            written = self.export_bundle(folder)
        except OSError as exc:
            W.QMessageBox.critical(self, "导出失败", str(exc))
            return
        names = "\n".join(f"• {p.name}" for p in written)
        W.QMessageBox.information(
            self,
            "接口构建导出成功",
            f"已成功生成 {len(written)} 个文件至：\n{folder}\n\n{names}",
        )

    # ---------- WorkbenchModule hooks ----------
    def active_port(self):
        return None

    def release_port(self):
        return None

    def apply_theme(self, dark: bool):
        if dark:
            self.code_edit.setStyleSheet("background:#182028;color:#dbe7f3;border:1px solid #445566;")
            self.guide_view.setStyleSheet("background:#1e262f;color:#e4eaf1;border:1px solid #445566;")
        else:
            self.code_edit.setStyleSheet("")
            self.guide_view.setStyleSheet("")

    def save_state(self, settings: C.QSettings):
        import json
        settings.setValue("firmware_studio/config", json.dumps(self.config_dict(), ensure_ascii=False))

    def restore_state(self, settings: C.QSettings):
        import json
        raw = settings.value("firmware_studio/config", "")
        if not raw:
            return
        try:
            cfg = json.loads(raw)
        except (TypeError, ValueError):
            return
        self._suppress_regen = True
        try:
            if isinstance(cfg.get("platform"), int) and 0 <= cfg["platform"] < self.platform_box.count():
                self.platform_box.setCurrentIndex(cfg["platform"])
            if cfg.get("uart_handle"):
                self.uart_edit.setText(str(cfg["uart_handle"]))
            if cfg.get("baudrate"):
                self.baud_box.setCurrentText(str(cfg["baudrate"]))
            if cfg.get("primary_loop") in LOOPS:
                self.loop_box.setCurrentText(cfg["primary_loop"])
            for key, spin in (("control_ms", self.ctrl_ms), ("report_ms", self.rep_ms), ("pwm_limit", self.pwm_lim)):
                if isinstance(cfg.get(key), (int, float)):
                    spin.setValue(cfg[key])
            d = cfg.get("defaults") or {}
            for k, spin in (("KP", self.def_kp), ("KI", self.def_ki), ("KD", self.def_kd), ("TGT", self.def_tgt)):
                if isinstance(d.get(k), (int, float)):
                    spin.setValue(float(d[k]))
            if "emit_roles" in cfg:
                self.emit_roles_chk.setChecked(bool(cfg["emit_roles"]))
        finally:
            self._suppress_regen = False
        if isinstance(cfg.get("channels"), list) and cfg["channels"]:
            self._set_channels([(str(c[0]), str(c[1])) for c in cfg["channels"] if isinstance(c, (list, tuple)) and len(c) >= 2])
        else:
            self.regenerate()

    def shutdown(self):
        return None
