"""Unified MCU serial interface generator & protocol documentation workbench."""
from pathlib import Path
import json
import re
import sys

from PySide6 import QtCore as C, QtGui, QtWidgets as W

from engine import LOOPS, MAX_CHANNELS

INTERFACE_TYPES = (
    "📈 PID 闭环调参接口",
    "🧭 IMU / MPU6050 姿态接口",
    "🎮 小车差速遥控接口 (J,x,y)",
)

PLATFORMS = (
    "STM32 HAL 库（CubeMX · huart 中断收发）",
    "STM32 标准库 SPL（stm32f10x.h · USARTx）",
    "ESP32 / Arduino（Serial C++ · .ino）",
)

PLATFORM_UART_LABELS = ("UART 句柄", "USART 外设", "Serial 对象")
PLATFORM_DEFAULT_HANDLES = ("huart2", "USART2", "Serial")

CHANNEL_ROLES = (
    ("target", "目标值 (Target)"),
    ("actual", "实际反馈 (Actual)"),
    ("output", "控制输出 (PWM/Output)"),
    ("error", "误差 (Error)"),
    ("other", "观察量 (Other)"),
)
ROLE_KEYS = tuple(k for k, _ in CHANNEL_ROLES)

IMU_FORMATS = (
    "六轴物理量：ax,ay,az,gx,gy,gz (g, °/s)",
    "MPU6050 原始计数：ax,ay,az,gx,gy,gz (LSB)",
    "平衡车四列：target,pitch,gyro,pwm",
    "欧拉角三列：pitch,roll,yaw (°)",
)

IMU_FORMAT_NAMES = (
    ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"),
    ("ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw"),
    ("target", "pitch", "gyro", "pwm"),
    ("pitch", "roll", "yaw"),
)

INTERFACE_PRESETS = (
    (
        "⚡ 速度环 (4ch SPD)",
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
        "⚖ 平衡车 (6ch 双环)",
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
        "🔀 乱序示例 (PWM在前)",
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
        "🎯 位置伺服 (4ch ANG)",
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

IMU_PRESETS = (
    ("⚖ 平衡车六轴 (50Hz 推荐)", {"imu_format": 0, "imu_report_ms": 20, "imu_emit_banners": True, "baudrate": 115200}),
    ("📦 原始计数 LSB (50Hz)", {"imu_format": 1, "imu_report_ms": 20, "imu_emit_banners": True, "baudrate": 115200}),
    ("📐 平衡车四列 / 欧拉角", {"imu_format": 2, "imu_report_ms": 20, "imu_emit_banners": True, "baudrate": 115200}),
)

REMOTE_PRESETS = (
    ("🚗 标准差速小车 (115200)", {"remote_cmd_fmt": "J,{x},{y}", "remote_pwm_limit": 1000, "remote_failsafe_ms": 500, "remote_aux_cmds": True, "baudrate": 115200}),
    ("📡 HC-05 蓝牙小车 (9600)", {"remote_cmd_fmt": "J,{x},{y}", "remote_pwm_limit": 1000, "remote_failsafe_ms": 500, "remote_aux_cmds": True, "baudrate": 9600}),
    ("🛡 强保护竞技小车 (250ms)", {"remote_cmd_fmt": "J,{x},{y}", "remote_pwm_limit": 7200, "remote_failsafe_ms": 250, "remote_aux_cmds": True, "baudrate": 115200}),
)

_GUIDE_STYLE = """<style>
body { font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif; font-size: 10pt; line-height: 1.58; }
h2 { color: #197a63; margin-top: 14px; margin-bottom: 6px; font-size: 12pt; }
h3 { color: #225fc1; margin-top: 10px; margin-bottom: 4px; font-size: 10.5pt; }
table { border-collapse: collapse; width: 100%; margin: 6px 0 12px 0; }
th, td { border: 1px solid #8aa0b2; padding: 5px 8px; text-align: left; }
th { background: rgba(34, 95, 193, 0.12); font-weight: 600; }
code { font-family: Consolas, "Cascadia Mono", monospace; background: rgba(120,140,160,0.16); padding: 1px 4px; border-radius: 3px; }
.tip { padding: 8px 10px; border-left: 4px solid #197a63; background: rgba(25,122,99,0.10); margin: 8px 0; }
.warn { padding: 8px 10px; border-left: 4px solid #d47b12; background: rgba(212,123,18,0.10); margin: 8px 0; }
</style>"""

_OVERVIEW_MATRIX_HTML = """
<h2>五、全软件三大串口接口速查总览</h2>
<table>
  <tr><th>工作台页面</th><th>接口类型</th><th>主要数据流向</th><th>核心协议格式</th><th>支持目标平台</th></tr>
  <tr><td><b>📈 PID 调参</b> (<code>Ctrl+1</code>)</td><td>闭环参数与高刷示波</td><td>双向（上下行）</td><td>CSV 1~8 列 + <code>#NAMES:</code>/<code>#ROLES:</code> + <code>@RUN/@STOP/@PING/@&lt;GRP&gt;.SET</code></td><td>STM32 HAL / SPL / ESP32·Arduino</td></tr>
  <tr><td><b>🧭 IMU 姿态</b> (<code>Ctrl+3</code>)</td><td>3D 姿态与中值角标定</td><td>下位机 → 上位机</td><td>4 种格式（六轴 g·°/s、原始 LSB、四列平衡车、欧拉角）+ <code>#BALANCE:</code></td><td>STM32 HAL / SPL / ESP32·Arduino</td></tr>
  <tr><td><b>🎮 遥控与 HC-05</b> (<code>Ctrl+4</code>)</td><td>差速遥控与蓝牙配参</td><td>上位机 → 小车</td><td><code>J,{x},{y}\\n</code>、急停 <code>S\\n</code>、<code>LED_ON/OFF/BEEP/SERVO</code> + <code>BlueSerial_MixWheels</code></td><td>STM32 HAL / SPL / ESP32·Arduino</td></tr>
</table>
"""


def build_interface_guide_html(interface_type: int = 0, platform: int = 0) -> str:
    """Return dynamic usage, wiring, protocol, and troubleshooting HTML for any (interface, platform)."""
    iface = interface_type if interface_type in (0, 1, 2) else 0
    plat = platform if platform in (0, 1, 2) else 0

    if plat == 0:
        plat_title = "STM32 HAL 库（CubeMX · 中断收发）"
        plat_steps = """
        <ol>
          <li><b>CubeMX 配置</b>：开启目标串口（如 <code>USART2</code>），勾选全局中断 <code>USART2 global interrupt</code>；波特率与左侧保持一致。</li>
          <li><b>工程加入文件</b>：点击右上角 <b>📦 一键导出全部文件</b>，将生成的 <code>.c/.h</code> 加入工程（Keil/CMake/CubeIDE），<b>无需勾选 <code>-u _printf_float</code></b>（全部使用整数运算格式化，节省十余 KB Flash）。</li>
          <li><b>挂钩回调与主循环</b>：按 <code>main_*_example.c</code> 提示，在 <code>HAL_UART_RxCpltCallback</code> / <code>ErrorCallback</code> 转发回调，并在 <code>while (1)</code> 调用对应 <code>*_Poll()</code>。</li>
        </ol>"""
    elif plat == 1:
        plat_title = "STM32 标准库 SPL（stm32f10x.h · USARTx）"
        plat_steps = """
        <ol>
          <li><b>Keil / 标准库工程配置</b>：确保 <code>stm32f10x_conf.h</code> 已启用 <code>stm32f10x_usart.h</code> 与 <code>misc.h</code>（NVIC 中断），初始化波特率及 <code>USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE)</code>。</li>
          <li><b>挂接两处中断（关键）</b>：在 <code>SysTick_Handler</code> 调用生成的 <code>*_SPL_SysTick1ms()</code> 维护毫秒时基；在 <code>USARTx_IRQHandler</code> 调用 <code>*_SPL_IRQHandler()</code> 完成无阻塞中断收发。</li>
          <li><b>零浮点 printf 依赖</b>：标准库代码同样内置定点数串行化，不依赖 MicroLIB 或 <code>printf</code> 重定向 <code>fputc</code>，直接用串口发送寄存器发送字节。</li>
        </ol>"""
    else:
        plat_title = "ESP32 / Arduino（Serial C++ · .ino）"
        plat_steps = """
        <ol>
          <li><b>打开草图</b>：导出后用 Arduino IDE 或 PlatformIO 打开生成的 <code>EmbeddedDebugStudio_*.ino</code>。</li>
          <li><b>串口选择</b>：USB 转串口直连使用默认 <code>Serial</code>；若使用 ESP32 硬件第二串口或外接 HC-05 蓝牙模块，将文件顶部的宏定义改为 <code>Serial2</code> 并配置 <code>Serial2.begin(baud, SERIAL_8N1, RX, TX)</code> 即可。</li>
          <li><b>AVR Uno/Nano 兼容</b>：生成代码已全部采用标准 <code>Serial.print()</code>，避免使用仅 ESP32 支持的 <code>Serial.printf()</code>。</li>
        </ol>"""

    if iface == 0:
        iface_section = f"""
<h2>一、PID 闭环调参接口 · 当前平台：{plat_title}</h2>
{plat_steps}
<div class="tip"><b>任意列顺序自动绑定（<code>#ROLES:</code>）</b><br>以前需要死记 CH1=目标、CH2=实际、CH3=PWM；开启 <code>#ROLES:</code> 后，即便你把 PWM 放第 1 列、反馈放第 3 列、目标放第 4 列，软件也能 100% 精准识别，自动整定不再误选通道。</div>
<h2>二、PID 上/下行核心协议表</h2>
<table>
  <tr><th>方向 / 报文</th><th>示例</th><th>说明</th></tr>
  <tr><td>下位机 → 上位机</td><td><code>650.0,100.0,98.2,1.8\\n</code></td><td>1~8 通道实时采样行（必须以 <code>\\n</code> 结尾）</td></tr>
  <tr><td>下位机 → 上位机</td><td><code>#NAMES:target,actual,pwm,err\\n</code></td><td>设置界面 1~8 通道显示名称</td></tr>
  <tr><td>下位机 → 上位机</td><td><code>#ROLES:target,actual,output,error\\n</code></td><td>声明各列角色（<code>target/actual/output/error/other</code>），任意列顺序都能自动绑定</td></tr>
  <tr><td>下位机 → 上位机</td><td><code>#CAP:PING\\n</code> / <code>#CAP:SET\\n</code></td><td>声明支持 0.5 s 心跳保护与原子组写入 <code>@&lt;GRP&gt;.SET</code></td></tr>
  <tr><td>上位机 → 下位机</td><td><code>@RUN\\n</code> / <code>@STOP\\n</code> / <code>@PING\\n</code></td><td>启动、急停（清 PWM）与 200 ms 保活心跳；超 500 ms 未收到心跳必须停机上报 <code>#STOP:heartbeat</code></td></tr>
  <tr><td>上位机 → 下位机</td><td><code>@SPD.GET\\n</code> / <code>@SPD.SET=kp,ki,kd,tgt\\n</code></td><td>读取四项参数（应答 <code>@PARAM:...</code>）与原子写入（应答 <code>@ACK:...</code>）</td></tr>
</table>
<h2>三、联调与排查清单</h2>
<ul>
  <li><b>首次接电机必做</b>：车轮架空或加装机械限位 → 连串口点<b>【读取】</b>确认绿框 → 点<b>【启动】</b>并按 <code>Esc</code> 测试急停 → 拔掉串口线验证 <code>0.5 s</code> 自动停机。</li>
  <li><b>只见接收字节无曲线？</b> 检查主循环是否执行 <code>Poll()</code>、数据行尾是否带 <code>\\n</code>，以及是否已点<b>【启动】(@RUN)</b>。</li>
  <li><b>一启动半秒就提示心跳停机？</b> 请勿在 <code>while(1)</code> 中使用阻塞 <code>HAL_Delay(500)</code> 或长时间关中断导致 <code>@PING</code> 丢失。</li>
</ul>"""
    elif iface == 1:
        iface_section = f"""
<h2>一、IMU / MPU6050 姿态接口 · 当前平台：{plat_title}</h2>
{plat_steps}
<div class="tip"><b>上位机支持 4 种协议自动匹配</b><br>只要下位机发送对应的 <code>#NAMES:</code> 帧或保持列数一致，在【🧭 IMU 姿态】页选好同一数据格式即可驱动 3D 平衡车模型、航空人工地平仪及一键标定机械中值角。</div>
<h2>二、IMU 数据格式与协议说明</h2>
<table>
  <tr><th>格式模式</th><th>串口发送示例（推荐 50 Hz / 20 ms）</th><th>量纲与换算公式</th></tr>
  <tr><td><b>六轴物理量 (推荐)</b></td><td><code>0.021,-0.014,0.998,0.42,-1.15,0.08\\n</code></td><td>加速度单位 <code>g</code>（<code>raw / 16384.0</code> @±2g）；角速度 <code>°/s</code>（<code>raw / 131.0</code> @±250°/s 或 <code>raw / 16.4</code> @±2000°/s），上位机自动做互补滤波</td></tr>
  <tr><td><b>MPU6050 原始计数</b></td><td><code>340,-210,16350,12,-35,4\\n</code></td><td>直接发送传感器 6 个 <code>int16_t</code> 寄存器原始值（±2g, ±250°/s），省去下位机浮点除法</td></tr>
  <tr><td><b>平衡车四列</b></td><td><code>0.00,-1.85,4.20,120\\n</code></td><td>依次为 <code>target(°), pitch(°), gyro(°/s), pwm</code>，驱动 3D 车身倾角与车轮转动</td></tr>
  <tr><td><b>欧拉角三列</b></td><td><code>-1.85,0.40,12.30\\n</code></td><td>依次为 <code>pitch(°), roll(°), yaw(°)</code>（适合已有 DMP / 四元数融合输出的固件）</td></tr>
</table>
<h3>可选状态横幅（搭配 <code>#NAMES:</code>）</h3>
<ul>
  <li><code>#BALANCE:CALIBRATING\\n</code> / <code>#BALANCE:READY\\n</code>：上电静止零偏校准开始与完成通知（上位机弹出黄/绿横幅）。</li>
  <li><code>#BALANCE:INIT_ERROR=&lt;code&gt;\\n</code>：MPU6050 I2C 检测失败时上报错误码并定位接线/地址故障。</li>
  <li><code>#BALANCE:TILT_STOP\\n</code>：倾角超阈值保护触发告警。</li>
</ul>
<h2>三、联调与机械中值角标定步骤</h2>
<ol>
  <li>烧录生成的 IMU 接口代码后，切换到 <b>【🧭 IMU 姿态】</b>（<code>Ctrl+3</code>），选择同一串口、波特率与数据格式，点击<b>【连接 IMU 串口】</b>。</li>
  <li>用手前后俯仰模块，观察 3D 车身方向；若前后反了，勾选左侧<b>【反转俯仰角极性】</b>。</li>
  <li>将平衡车扶正处于静止平衡位置约 2 秒，点击顶部<b>【🎯 标定机械中值角】</b>采集 50 帧均值，完成后点击<b>【将中值角同步至 PID 页 ANG.TGT】</b>即可直接闭环。</li>
</ol>"""
    else:
        iface_section = f"""
<h2>一、小车差速遥控接口 (J,x,y) · 当前平台：{plat_title}</h2>
{plat_steps}
<div class="warn"><b>失联自动停车看门狗（Failsafe）已内置</b><br>上位机在非零摇杆保持期间每 <code>100 ms</code> 重发保活指令；生成的下位机接口默认开启超时保护（如 <code>500 ms</code> 未收到有效报文自动归零左右轮输出），防止蓝牙断线导致小车失控飞车！</div>
<h2>二、BlueSerialStudio 兼容遥控协议与差速混控</h2>
<table>
  <tr><th>指令格式</th><th>参数说明</th><th>下位机动作要求</th></tr>
  <tr><td><code>J,&lt;x&gt;,&lt;y&gt;\\n</code></td><td><code>x</code> 转向 (<code>-100..100</code>，右为正)<br><code>y</code> 前后速度 (<code>-100..100</code>，前为正)</td><td>调用内置 <code>MixWheels(x, y, &amp;left, &amp;right)</code> 转为左右轮限幅 PWM 并喂狗</td></tr>
  <tr><td><code>S\\n</code> / <code>J,0,0\\n</code></td><td>紧急停车 / 摇杆回中</td><td><b>立即将左右轮 PWM 清零</b>并停止计数</td></tr>
  <tr><td><code>LED_ON\\n</code> / <code>LED_OFF\\n</code></td><td>开关车灯（可选辅助指令）</td><td>点亮或熄灭车载 LED 指示灯</td></tr>
  <tr><td><code>BEEP\\n</code> / <code>SERVO,&lt;deg&gt;\\n</code></td><td>蜂鸣器 / 舵机角度 <code>0..180</code></td><td>执行鸣笛或更新云台/转向舵机脉宽</td></tr>
</table>
<h3>差速混控算法（与 PC/Android 端完全一致）</h3>
<ul>
  <li>当 <code>y == 0</code>（原地旋转）：左轮 = <code>+x</code>，右轮 = <code>-x</code>。</li>
  <li>前进或后退叠加转向：右转（<code>x &gt; 0</code>）时按比例衰减右轮 <code>right = y * (100 - x) / 100</code>；左转（<code>x &lt; 0</code>）时衰减左轮 <code>left = y * (100 + x) / 100</code>，再等比缩放到 <code>±PWM_LIMIT</code>。</li>
</ul>
<h2>三、蓝牙 HC-05 配网与联调步骤</h2>
<ol>
  <li>若使用 HC-05 蓝牙模块，先通过 USB-TTL 按住按键上电进入慢闪 AT 模式，在 <b>【🎮 遥控与 HC-05】</b>（<code>Ctrl+4</code>）选波特率 <code>38400</code>，使用右侧 <b>AT 配置工具箱</b> 将模块波特率改为与小车一致（常用 <code>9600</code> 或 <code>115200</code>）。</li>
  <li>将 HC-05 交叉接至单片机串口（模块 TX → 单片机 RX，模块 RX → 单片机 TX），烧录本页生成的代码。</li>
  <li>电脑连接蓝牙串口后点击<b>【打开串口】</b>，拖动 360° 单摇杆、双摇杆或按住 <code>W/A/S/D</code> 即可遥控；按 <code>Space</code> 或全局 <code>Esc</code> 立即停车。</li>
</ol>"""

    return f"<html><head><meta charset='utf-8'>{_GUIDE_STYLE}</head><body>{iface_section}{_OVERVIEW_MATRIX_HTML}</body></html>"


PROTOCOL_GUIDE_HTML = build_interface_guide_html(0, 0)


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
    return cleaned[:24] or f"CH{idx + 1}"


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


def _safe_uart_handle(raw: str, platform: int) -> str:
    default = PLATFORM_DEFAULT_HANDLES[platform if platform in (0, 1, 2) else 0]
    cleaned = re.sub(r"[^A-Za-z0-9_]", "", str(raw or "").strip())
    if not cleaned:
        return default
    if platform == 0 and cleaned.upper().startswith("USART"):
        return "huart" + cleaned[5:] if len(cleaned) > 5 else default
    if platform == 1 and cleaned.lower().startswith("huart"):
        return "USART" + cleaned[5:] if len(cleaned) > 5 else default
    if platform == 2 and (cleaned.lower().startswith("huart") or cleaned.upper().startswith("USART")):
        return default
    return cleaned


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


def _usart_irq_name(usart_name: str) -> str:
    up = usart_name.upper()
    if up in ("USART1", "USART2", "USART3", "UART4", "UART5", "USART6"):
        return f"{up}_IRQHandler"
    return "USART2_IRQHandler"


def _usart_rcc_setup(usart_name: str, baudrate: int) -> str:
    up = usart_name.upper()
    if up == "USART1":
        rcc = "RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);"
        pins = "/* PA9=TX (AF_PP), PA10=RX (IN_FLOATING) */"
        irqn = "USART1_IRQn"
    elif up == "USART3":
        rcc = "RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);\n    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);"
        pins = "/* PB10=TX (AF_PP), PB11=RX (IN_FLOATING) */"
        irqn = "USART3_IRQn"
    else:
        rcc = "RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);\n    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);"
        pins = "/* PA2=TX (AF_PP), PA3=RX (IN_FLOATING) */"
        irqn = "USART2_IRQn"
    return (
        f"    {rcc}\n"
        f"    {pins}\n"
        f"    USART_InitTypeDef u;\n"
        f"    USART_StructInit(&u);\n"
        f"    u.USART_BaudRate = {baudrate}U;\n"
        f"    USART_Init({up}, &u);\n"
        f"    USART_ITConfig({up}, USART_IT_RXNE, ENABLE);\n"
        f"    USART_Cmd({up}, ENABLE);\n"
        f"    NVIC_EnableIRQ({irqn});"
    )


_IMU_BASE_HELPERS_C = """static int ImuAppendChar(char *buf, unsigned cap, unsigned *pos, char ch)
{
    if (*pos + 1U >= cap) {
        return -1;
    }
    buf[*pos] = ch;
    (*pos)++;
    buf[*pos] = '\\0';
    return 0;
}

static int ImuAppendUInt(char *buf, unsigned cap, unsigned *pos, uint32_t v)
{
    char tmp[12];
    unsigned n = 0U;
    if (v == 0U) {
        return ImuAppendChar(buf, cap, pos, '0');
    }
    while (v > 0U && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n > 0U) {
        if (ImuAppendChar(buf, cap, pos, tmp[--n]) != 0) {
            return -1;
        }
    }
    return 0;
}"""

_IMU_STR_HELPER_C = """static int ImuAppendStr(char *buf, unsigned cap, unsigned *pos, const char *s)
{
    while (*s != '\\0') {
        if (ImuAppendChar(buf, cap, pos, *s++) != 0) {
            return -1;
        }
    }
    return 0;
}"""

_IMU_INT_HELPER_C = """static int ImuAppendInt(char *buf, unsigned cap, unsigned *pos, int32_t val)
{
    uint32_t mag;
    if (val < 0) {
        if (ImuAppendChar(buf, cap, pos, '-') != 0) {
            return -1;
        }
        mag = (uint32_t)(-(val + 1)) + 1U;
    } else {
        mag = (uint32_t)val;
    }
    return ImuAppendUInt(buf, cap, pos, mag);
}"""

_IMU_FIXED_HELPER_C = """static int ImuAppendFixed(char *buf, unsigned cap, unsigned *pos, float val, uint8_t decimals)
{
    uint32_t scale = 1U;
    for (uint8_t i = 0U; i < decimals; ++i) {
        scale *= 10U;
    }
    if (val != val) {
        return ImuAppendChar(buf, cap, pos, '0');
    }
    if (val < 0.0f) {
        if (ImuAppendChar(buf, cap, pos, '-') != 0) {
            return -1;
        }
        val = -val;
    }
    if (val > 2000000.0f) {
        val = 2000000.0f;
    }
    uint32_t whole = (uint32_t)val;
    float frac_f = (val - (float)whole) * (float)scale + 0.5f;
    uint32_t frac = (uint32_t)frac_f;
    if (frac >= scale) {
        whole += 1U;
        frac -= scale;
    }
    if (ImuAppendUInt(buf, cap, pos, whole) != 0) {
        return -1;
    }
    if (decimals == 0U) {
        return 0;
    }
    if (ImuAppendChar(buf, cap, pos, '.') != 0) {
        return -1;
    }
    uint32_t div = scale / 10U;
    while (div > 0U) {
        uint8_t d = (uint8_t)((frac / div) % 10U);
        if (ImuAppendChar(buf, cap, pos, (char)('0' + d)) != 0) {
            return -1;
        }
        div /= 10U;
    }
    return 0;
}"""


def _build_imu_helpers_c(imu_fmt: int, emit_banners: bool) -> str:
    parts = [_IMU_BASE_HELPERS_C]
    if emit_banners:
        parts.append(_IMU_STR_HELPER_C)
    parts.append(_IMU_INT_HELPER_C if imu_fmt == 1 else _IMU_FIXED_HELPER_C)
    return "\n\n".join(parts)


def _generate_pid_bundle(cfg: dict) -> dict[str, str]:
    raw_channels = list(cfg.get("channels") or [("target", "target"), ("actual", "actual"), ("pwm", "output"), ("error", "error")])
    channels = [(sanitize_channel_name(n, i), r if r in ROLE_KEYS else "other") for i, (n, r) in enumerate(raw_channels[:MAX_CHANNELS])]
    if not channels:
        channels = [("actual", "actual")]

    platform = int(cfg.get("platform", 0))
    if platform not in (0, 1, 2):
        platform = 0
    uart_handle = _safe_uart_handle(cfg.get("uart_handle", ""), platform)
    baudrate = int(cfg.get("baudrate", 115200))
    control_ms = max(1, int(cfg.get("control_ms", 10)))
    report_ms = max(1, int(cfg.get("report_ms", 20)))
    heartbeat_ms = max(200, int(cfg.get("heartbeat_ms", 500)))
    pwm_limit = float(cfg.get("pwm_limit", 1000.0))
    primary_loop = str(cfg.get("primary_loop", "SPD")).upper()
    if primary_loop not in LOOPS:
        primary_loop = "SPD"
    defaults = cfg.get("defaults") or {"KP": 5.0, "KI": 30.0, "KD": 0.05, "TGT": 100.0}
    kp = float(defaults.get("KP", 5.0))
    ki = float(defaults.get("KI", 30.0))
    kd = float(defaults.get("KD", 0.05))
    tgt = float(defaults.get("TGT", 100.0))
    emit_roles = bool(cfg.get("emit_roles", True))

    names_hdr, roles_hdr = build_protocol_headers(channels, emit_roles=emit_roles)
    n_ch = len(channels)
    c_header_lines = f"{names_hdr}\\n" + (f"{roles_hdr}\\n" if emit_roles else "")
    fill_lines_c = "\n".join(
        f"    ch[{i}] = {_channel_expr(role, i)}; /* {name} ({role}) */"
        for i, (name, role) in enumerate(channels)
    )
    fill_lines_ino = "\n".join(
        f"    ch[{i}] = {_channel_expr(role, i)}; // {name} ({role})"
        for i, (name, role) in enumerate(channels)
    )
    plat_name = PLATFORMS[platform]

    readme = f"""# EmbeddedDebugStudio · PID 闭环调参接口接入说明

- 目标平台：**{plat_name}**
- 接口句柄 / 外设：**`{uart_handle}`** · 波特率：**`{baudrate}`**
- 通道与参数：共 **{n_ch} 通道** · 主调环路 **`{primary_loop}`** · 控制 **`{control_ms} ms`** · 上报 **`{report_ms} ms`** · PWM 限幅 **`{pwm_limit:g}`**
- 协议声明：
  - `{names_hdr}`
  {f'- `{roles_hdr}`' if emit_roles else '- （未启用 `#ROLES:`，上位机将按通道名称语义自动匹配）'}

## 接入步骤
1. 将本目录生成的文件加入工程编译（STM32 均无需 `-u _printf_float`）。
2. 按示例文件中的注释挂接串口接收/发送回调与毫秒时基（HAL 用 `HAL_GetTick()`，SPL 在 `SysTick_Handler` 调 `PID_Port_SPL_SysTick1ms()`），并实现电机 PWM 输出函数 `ApplyOutput`。
3. 上位机切换到 **【📈 PID 调参】(`Ctrl+1`)**，选中串口与波特率连接后先点 **【读取】**，再点 **【启动】** 与 `Esc` 急停验证。

## 实车安全清单
- 首次运行 **【⚡ 自动整定 PID…】** 前请务必将车轮架空离地或加装机械限位。
- 拔掉串口线后下位机应在 `500 ms` 内自动触发 `#STOP:heartbeat` 并清零电机 PWM。
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
#define PID_PORT_PWM_LIMIT {_c_float(pwm_limit)}

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

        main_example = f"""/* STM32 HAL 接入示例（串口波特率设为 {baudrate}，句柄 {uart_handle}） */
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
        irq_name = _usart_irq_name(uart_handle)
        usart_init_lines = _usart_rcc_setup(uart_handle, baudrate)
        pid_port_spl_h = f"""#ifndef PID_PORT_SPL_H_
#define PID_PORT_SPL_H_

#include "stm32f10x.h"
#include "pid_link.h"

#define PRIMARY_PID_GROUP PID_{primary_loop}
#define PID_PORT_CHANNELS {n_ch}U
#define PID_PORT_CONTROL_MS {control_ms}U
#define PID_PORT_REPORT_MS {report_ms}U
#define PID_PORT_PWM_LIMIT {_c_float(pwm_limit)}

/** @brief 初始化 STM32 标准库 SPL 串口调参接口（零 printf_float 依赖）。 */
void PID_Port_SPL_Init(USART_TypeDef *usart);

/** @brief 在 SysTick_Handler() 每 1 ms 调用一次，提供看门狗时基。 */
void PID_Port_SPL_SysTick1ms(void);

/** @brief 返回当前系统毫秒计数。 */
uint32_t PID_Port_SPL_GetTick(void);

/** @brief 在 {irq_name}() 中直接调用，完成无阻塞 RXNE 接收与 TXE 发送。 */
void PID_Port_SPL_IRQHandler(void);

/** @brief 在主循环高频调用：解析命令、喂狗/停机保护、触发下一帧发送。 */
void PID_Port_SPL_Poll(void);

/** @brief 用户电机输出钩子（停机或超 500 ms 失联自动传 0）。 */
void PID_Port_SPL_ApplyOutput(float value);

/** @brief 按配置顺序上报 {n_ch} 路浮点波形数据。 */
void PID_Port_SPL_ReportSample(float actual_val, float applied_pwm);

#endif
"""

        pid_port_spl_c = f"""#include "pid_port_spl.h"
#include <string.h>

#define PID_PORT_RX_FIFO_SIZE 256U
#define PID_PORT_TX_FRAME_LEN 192U
#define PID_PORT_REPLY_SLOTS  8U

typedef struct {{
    uint16_t length;
    uint8_t data[PID_PORT_TX_FRAME_LEN];
}} PidSplFrame;

static USART_TypeDef *s_usart;
static volatile uint32_t s_tick_ms;
static volatile uint8_t s_fifo[PID_PORT_RX_FIFO_SIZE];
static volatile uint8_t s_rd, s_wr, s_overflow;

static PidSplFrame s_replies[PID_PORT_REPLY_SLOTS];
static PidSplFrame s_telemetry;
static PidSplFrame s_in_flight;
static volatile uint16_t s_tx_pos;
static volatile uint8_t s_tx_active;
static uint8_t s_reply_head, s_reply_count, s_tx_overflow;
static uint8_t s_prev_running;

void PID_Port_SPL_SysTick1ms(void)
{{
    s_tick_ms++;
}}

uint32_t PID_Port_SPL_GetTick(void)
{{
    return s_tick_ms;
}}

void PID_ZeroOutputs(void)
{{
    PID_Port_SPL_ApplyOutput(0.0f);
}}

void PID_Write(const char *data, unsigned len)
{{
    if (!data || len == 0U || len > sizeof(s_in_flight.data)) {{
        return;
    }}
    PidSplFrame *frame = &s_telemetry;
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
    if (!s_usart || s_tx_active) {{
        return;
    }}
    if (s_tx_overflow && s_reply_count < PID_PORT_REPLY_SLOTS) {{
        static const char warn[] = "#WARN:reply queue overflow\\n";
        s_tx_overflow = 0U;
        PID_Write(warn, (unsigned)(sizeof(warn) - 1U));
    }}
    PidSplFrame *next = (s_reply_count != 0U) ? &s_replies[s_reply_head] : &s_telemetry;
    if (next->length == 0U) {{
        return;
    }}
    s_in_flight = *next;
    next->length = 0U;
    if (s_reply_count != 0U) {{
        s_reply_head = (uint8_t)((s_reply_head + 1U) % PID_PORT_REPLY_SLOTS);
        s_reply_count--;
    }}
    s_tx_pos = 0U;
    s_tx_active = 1U;
    USART_ITConfig(s_usart, USART_IT_TXE, ENABLE);
}}

void PID_Port_SPL_Init(USART_TypeDef *usart)
{{
    s_usart = usart;
    s_rd = s_wr = s_overflow = s_prev_running = 0U;
    s_reply_head = s_reply_count = s_tx_overflow = 0U;
    s_tx_active = 0U;
    s_tx_pos = 0U;
    s_telemetry.length = 0U;

    pid_loops[PRIMARY_PID_GROUP].kp = {_c_float(kp)};
    pid_loops[PRIMARY_PID_GROUP].ki = {_c_float(ki)};
    pid_loops[PRIMARY_PID_GROUP].kd = {_c_float(kd)};
    pid_loops[PRIMARY_PID_GROUP].target = {_c_float(tgt)};

    PID_ZeroOutputs();
    PID_WatchdogTick(s_tick_ms);
    PID_CheckFloatFormat();
    SendChannelHeader();
    if (s_usart) {{
        USART_ITConfig(s_usart, USART_IT_RXNE, ENABLE);
    }}
}}

void PID_Port_SPL_IRQHandler(void)
{{
    if (!s_usart) {{
        return;
    }}
    if (USART_GetITStatus(s_usart, USART_IT_RXNE) != RESET) {{
        uint8_t b = (uint8_t)USART_ReceiveData(s_usart);
        uint8_t next = (uint8_t)(s_wr + 1U);
        if (next == s_rd) {{
            s_overflow = 1U;
        }} else {{
            s_fifo[s_wr] = b;
            s_wr = next;
        }}
    }}
    if (USART_GetITStatus(s_usart, USART_IT_TXE) != RESET) {{
        if (s_tx_active && s_tx_pos < s_in_flight.length) {{
            USART_SendData(s_usart, s_in_flight.data[s_tx_pos++]);
        }} else {{
            s_tx_active = 0U;
            USART_ITConfig(s_usart, USART_IT_TXE, DISABLE);
        }}
    }}
}}

void PID_Port_SPL_Poll(void)
{{
    PID_WatchdogTick(s_tick_ms);
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

void PID_Port_SPL_ReportSample(float actual_val, float applied_pwm)
{{
    float ch[PID_PORT_CHANNELS];
{fill_lines_c}
    PID_ReportValues(ch, PID_PORT_CHANNELS);
}}
"""

        main_spl = f"""/* STM32 标准库 SPL (stm32f10x.h) PID 闭环调参接入示例 · {uart_handle} @ {baudrate} */
#include "stm32f10x.h"
#include "pid_port_spl.h"

static float s_applied_pwm = 0.0f;

/* 1. 电机 PWM 输出钩子（停机与 0.5 s 心跳保护自动归零） */
void PID_Port_SPL_ApplyOutput(float value)
{{
    s_applied_pwm = value;
    /* TODO: 写入定时器 CCR，例如 Motor_SetPWM((int16_t)value); */
}}

/* 2. 在 stm32f10x_it.c 的 SysTick 与串口中断中转发以下两函数 */
void SysTick_Handler(void)
{{
    PID_Port_SPL_SysTick1ms();
}}

void {irq_name}(void)
{{
    PID_Port_SPL_IRQHandler();
}}

static void Board_UsartInit(void)
{{
{usart_init_lines}
}}

/* 3. 在 main() 中初始化并循环调用 */
void App_PidSplDemo(void)
{{
    SysTick_Config(SystemCoreClock / 1000U);
    Board_UsartInit();
    PID_Port_SPL_Init({uart_handle});

    uint32_t last_ctrl_ms = PID_Port_SPL_GetTick();
    uint32_t last_rep_ms = last_ctrl_ms;
    while (1) {{
        PID_Port_SPL_Poll();

        uint32_t now = PID_Port_SPL_GetTick();
        if (now - last_ctrl_ms >= PID_PORT_CONTROL_MS) {{
            float dt = (float)(now - last_ctrl_ms) * 0.001f;
            last_ctrl_ms = now;
            float actual_val = 0.0f; /* TODO: 读取编码器速度或角度传感器 */
            float out = PID_Position(&pid_loops[PRIMARY_PID_GROUP], actual_val, dt, PID_PORT_PWM_LIMIT);
            PID_Port_SPL_ApplyOutput(out);
        }}
        if (now - last_rep_ms >= PID_PORT_REPORT_MS) {{
            last_rep_ms = now;
            float actual_val = 0.0f; /* TODO: 保持与控制步骤相同的最新反馈值 */
            PID_Port_SPL_ReportSample(actual_val, s_applied_pwm);
        }}
    }}
}}
"""
        return {
            "main_integration_spl.c": main_spl,
            "pid_port_spl.h": pid_port_spl_h,
            "pid_port_spl.c": pid_port_spl_c,
            "pid_link.h": load_core_firmware_file("pid_link.h"),
            "pid_link.c": load_core_firmware_file("pid_link.c"),
            "README_接入说明.md": readme,
        }

    # Platform 2: ESP32 / Arduino sketch (portable Serial.print, no Serial.printf)
    loop_idx = LOOPS.index(primary_loop)
    ino = f"""/* EmbeddedDebugStudio 兼容串口 PID 接口（ESP32 / Arduino 通用）
 * 支持：#NAMES / #ROLES 任意列顺序、#CAP:PING ({heartbeat_ms} ms 保护)、#CAP:SET 原子写入
 * 如需改用 ESP32 Serial2 或软串口，修改下方 #define PID_SERIAL 即可。
 */
#include <Arduino.h>
#include <math.h>

#define PID_SERIAL {uart_handle}

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
  PID_SERIAL.print(F("{c_header_lines}"));
}}

void applyMotorPwm(float pwm) {{
  applied_pwm = constrain(pwm, -{_c_float(pwm_limit)}, {_c_float(pwm_limit)});
  // TODO: 在此处将 applied_pwm 输出至 PWM 引脚 / ledcWrite / analogWrite
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
    PID_SERIAL.print(F("#STOP:"));
    PID_SERIAL.println(reason);
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
  float out = constrain(raw, -{_c_float(pwm_limit)}, {_c_float(pwm_limit)});
  if (fabsf(raw) <= {_c_float(pwm_limit)} || (raw - out) * p.ki * err < 0.0f) {{
    p.integral = trial_i;
  }}
  p.output = out;
  return out;
}}

void printParamResponse(int grp) {{
  const char *g = LOOP_NAMES[grp];
  const PidLoop &lp = pid_loops[grp];
  PID_SERIAL.print(F("@PARAM:"));
  PID_SERIAL.print(g); PID_SERIAL.print(F(".KP="));  PID_SERIAL.print(lp.kp, 4);     PID_SERIAL.print(',');
  PID_SERIAL.print(g); PID_SERIAL.print(F(".KI="));  PID_SERIAL.print(lp.ki, 4);     PID_SERIAL.print(',');
  PID_SERIAL.print(g); PID_SERIAL.print(F(".KD="));  PID_SERIAL.print(lp.kd, 4);     PID_SERIAL.print(',');
  PID_SERIAL.print(g); PID_SERIAL.print(F(".TGT=")); PID_SERIAL.println(lp.target, 4);
}}

void handleCommand(String line) {{
  line.trim();
  if (!line.startsWith("@")) return;
  String body = line.substring(1);
  if (body == "PING") {{ last_ping_ms = millis(); return; }}
  if (body == "STOP") {{ stopMotor(); PID_SERIAL.println(F("@ACK:STOP")); return; }}
  if (body == "RUN") {{
    last_ping_ms = millis();
    if (!pid_running) {{
      for (int i = 0; i < 4; ++i) {{ pid_loops[i].integral = 0.0f; pid_loops[i].initialized = false; }}
      pid_running = true;
    }}
    sendHeader();
    PID_SERIAL.println(F("#CAP:PING"));
    PID_SERIAL.println(F("#CAP:SET"));
    PID_SERIAL.println(F("@ACK:RUN"));
    return;
  }}

  int grp = PRIMARY_PID_GROUP;
  int dot = body.indexOf('.');
  int eq = body.indexOf('=');
  if (dot > 0 && (eq < 0 || dot < eq)) {{
    String g = body.substring(0, dot);
    grp = -1;
    for (int i = 0; i < 4; ++i) if (g == LOOP_NAMES[i]) grp = i;
    if (grp < 0) {{ PID_SERIAL.println(F("@ERR:unknown loop")); return; }}
    body = body.substring(dot + 1);
    eq = body.indexOf('=');
  }}
  PidLoop &lp = pid_loops[grp];
  if (body == "GET") {{ printParamResponse(grp); return; }}
  if (body == "RST") {{
    lp.integral = 0.0f; lp.initialized = false;
    PID_SERIAL.print(F("@ACK:")); PID_SERIAL.println(line.substring(1));
    return;
  }}
  if (eq < 0) {{ PID_SERIAL.println(F("@ERR:expected key=value")); return; }}
  String k = body.substring(0, eq);
  String v = body.substring(eq + 1);
  if (k == "SET") {{
    int c1 = v.indexOf(','), c2 = v.indexOf(',', c1 + 1), c3 = v.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) {{ PID_SERIAL.println(F("@ERR:expected kp,ki,kd,tgt")); return; }}
    lp.kp = v.substring(0, c1).toFloat();
    lp.ki = v.substring(c1 + 1, c2).toFloat();
    lp.kd = v.substring(c2 + 1, c3).toFloat();
    lp.target = v.substring(c3 + 1).toFloat();
    PID_SERIAL.print(F("@ACK:")); PID_SERIAL.println(line.substring(1));
    return;
  }}
  float val = v.toFloat();
  if (k == "KP") lp.kp = val;
  else if (k == "KI") lp.ki = val;
  else if (k == "KD") lp.kd = val;
  else if (k == "TGT") lp.target = val;
  else {{ PID_SERIAL.println(F("@ERR:unknown key")); return; }}
  PID_SERIAL.print(F("@ACK:")); PID_SERIAL.println(line.substring(1));
}}

void reportTelemetry(float actual_val) {{
  float ch[{n_ch}];
{fill_lines_ino}
  for (int i = 0; i < {n_ch}; ++i) {{
    if (i) PID_SERIAL.print(',');
    PID_SERIAL.print(ch[i], 4);
  }}
  PID_SERIAL.println();
}}

void setup() {{
  PID_SERIAL.begin({baudrate});
  pid_loops[PRIMARY_PID_GROUP] = {{{_c_float(kp)}, {_c_float(ki)}, {_c_float(kd)}, {_c_float(tgt)}, 0, 0, 0, false}};
  sendHeader();
}}

void loop() {{
  while (PID_SERIAL.available()) {{
    char c = (char)PID_SERIAL.read();
    if (c == '\\n') {{ handleCommand(rx_buf); rx_buf = ""; }}
    else if (c != '\\r' && rx_buf.length() < 120) rx_buf += c;
  }}
  unsigned long now = millis();
  if (pid_running && now - last_ping_ms >= {heartbeat_ms}UL) stopMotor("heartbeat");

  static unsigned long last_ctrl = 0, last_rep = 0;
  if (now - last_ctrl >= {control_ms}UL) {{
    float dt = (now - last_ctrl) * 0.001f;
    last_ctrl = now;
    float actual_val = 0.0f; // TODO: 读取编码器 / 传感器反馈
    applyMotorPwm(computePid(pid_loops[PRIMARY_PID_GROUP], actual_val, dt));
  }}
  if (now - last_rep >= {report_ms}UL) {{
    last_rep = now;
    float actual_val = 0.0f; // TODO: 与上方反馈保持一致
    reportTelemetry(actual_val);
  }}
}}
"""
    return {
        "EmbeddedDebugStudio_PID.ino": ino,
        "README_接入说明.md": readme,
    }


def _generate_imu_bundle(cfg: dict) -> dict[str, str]:
    platform = int(cfg.get("platform", 0))
    if platform not in (0, 1, 2):
        platform = 0
    imu_fmt = int(cfg.get("imu_format", 0))
    if imu_fmt not in (0, 1, 2, 3):
        imu_fmt = 0
    uart_handle = _safe_uart_handle(cfg.get("uart_handle", ""), platform)
    baudrate = int(cfg.get("baudrate", 115200))
    report_ms = max(5, int(cfg.get("imu_report_ms", 20)))
    emit_banners = bool(cfg.get("imu_emit_banners", True))

    col_names = IMU_FORMAT_NAMES[imu_fmt]
    names_hdr = "#NAMES:" + ",".join(col_names)
    fmt_label = IMU_FORMATS[imu_fmt]
    plat_name = PLATFORMS[platform]

    if imu_fmt == 0:
        sig_args = "float ax_g, float ay_g, float az_g, float gx_dps, float gy_dps, float gz_dps"
        call_args = "ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps"
        demo_decl = "float ax_g = 0.0f, ay_g = 0.0f, az_g = 1.0f, gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;"
        demo_vals = "ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps"
        c_pack_body = """    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, ax_g, 4U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, ay_g, 4U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, az_g, 4U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, gx_dps, 2U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, gy_dps, 2U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, gz_dps, 2U) == 0);"""
    elif imu_fmt == 1:
        sig_args = "int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw"
        call_args = "ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw"
        demo_decl = "int16_t ax_raw = 0, ay_raw = 0, az_raw = 16384, gx_raw = 0, gy_raw = 0, gz_raw = 0;"
        demo_vals = "ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw"
        c_pack_body = """    ok &= (ImuAppendInt(buf, sizeof(buf), &pos, (int32_t)ax_raw) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendInt(buf, sizeof(buf), &pos, (int32_t)ay_raw) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendInt(buf, sizeof(buf), &pos, (int32_t)az_raw) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendInt(buf, sizeof(buf), &pos, (int32_t)gx_raw) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendInt(buf, sizeof(buf), &pos, (int32_t)gy_raw) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendInt(buf, sizeof(buf), &pos, (int32_t)gz_raw) == 0);"""
    elif imu_fmt == 2:
        sig_args = "float target_deg, float pitch_deg, float gyro_dps, float pwm_out"
        call_args = "target_deg, pitch_deg, gyro_dps, pwm_out"
        demo_decl = "float target_deg = 0.0f, pitch_deg = 0.0f, gyro_dps = 0.0f, pwm_out = 0.0f;"
        demo_vals = "target_deg, pitch_deg, gyro_dps, pwm_out"
        c_pack_body = """    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, target_deg, 2U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, pitch_deg, 2U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, gyro_dps, 2U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, pwm_out, 1U) == 0);"""
    else:
        sig_args = "float pitch_deg, float roll_deg, float yaw_deg"
        call_args = "pitch_deg, roll_deg, yaw_deg"
        demo_decl = "float pitch_deg = 0.0f, roll_deg = 0.0f, yaw_deg = 0.0f;"
        demo_vals = "pitch_deg, roll_deg, yaw_deg"
        c_pack_body = """    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, pitch_deg, 2U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, roll_deg, 2U) == 0);
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, ',') == 0);
    ok &= (ImuAppendFixed(buf, sizeof(buf), &pos, yaw_deg, 2U) == 0);"""

    readme = f"""# EmbeddedDebugStudio · IMU / MPU6050 姿态接口接入说明

- 目标平台：**{plat_name}**
- 接口句柄 / 外设：**`{uart_handle}`** · 波特率：**`{baudrate}`**
- 数据格式：**{fmt_label}**
- 上报周期：**`{report_ms} ms`**（建议 20 ms = 50 Hz） · 状态横幅：`{"开启 (#BALANCE:)" if emit_banners else "关闭"}`
- 协议声明行：`{names_hdr}`

## 接入步骤
1. 将生成的代码加入工程编译（独立整数定点数编码，零 `printf_float` 依赖）。
2. 初始化 MPU6050 后，每隔 `{report_ms} ms` 调用上报函数传入传感器值，在主循环调用 `Poll()` 完成串口非阻塞发送。
3. 打开上位机 **【🧭 IMU 姿态】(`Ctrl+3`)**，选择同波特率与数据格式即可查看 3D 平衡车模型、地平仪并标定机械中值角。
"""

    if platform == 0:
        imu_hal_h = f"""#ifndef IMU_PORT_HAL_H_
#define IMU_PORT_HAL_H_

#include <stdint.h>
#include "main.h"

#define IMU_PORT_FORMAT_ID  {imu_fmt}U
#define IMU_PORT_REPORT_MS  {report_ms}U
#define IMU_PORT_BANNERS_ON {1 if emit_banners else 0}U

void IMU_Port_Init(UART_HandleTypeDef *uart);
void IMU_Port_Poll(void);
void IMU_Port_SendHeader(void);
void IMU_Port_SendCalibrating(void);
void IMU_Port_SendReady(void);
void IMU_Port_SendInitError(uint8_t who_am_i);
void IMU_Port_SendTiltStop(void);
void IMU_Port_ReportSample({sig_args});

#endif
"""

        imu_hal_c = f"""#include "imu_port_hal.h"
#include <string.h>

#define IMU_TX_FRAME_LEN 128U

typedef struct {{
    uint16_t length;
    uint8_t data[IMU_TX_FRAME_LEN];
}} ImuTxFrame;

static UART_HandleTypeDef *s_uart;
static ImuTxFrame s_banner;
static ImuTxFrame s_sample;
static ImuTxFrame s_in_flight;

{_build_imu_helpers_c(imu_fmt, emit_banners)}

static void QueueTextFrame(ImuTxFrame *slot, const char *text, unsigned len)
{{
    if (!text || len == 0U || len > sizeof(slot->data)) {{
        return;
    }}
    memcpy(slot->data, text, len);
    slot->length = (uint16_t)len;
}}

static void SendStaticBanner(const char *msg)
{{
#if IMU_PORT_BANNERS_ON
    QueueTextFrame(&s_banner, msg, (unsigned)strlen(msg));
    IMU_Port_Poll();
#else
    (void)msg;
#endif
}}

void IMU_Port_SendHeader(void)
{{
    static const char hdr[] = "{names_hdr}\\n";
    QueueTextFrame(&s_banner, hdr, (unsigned)(sizeof(hdr) - 1U));
}}

void IMU_Port_SendCalibrating(void) {{ SendStaticBanner("#BALANCE:CALIBRATING\\n"); }}
void IMU_Port_SendReady(void) {{ IMU_Port_SendHeader(); SendStaticBanner("#BALANCE:READY\\n"); }}
void IMU_Port_SendTiltStop(void) {{ SendStaticBanner("#BALANCE:TILT_STOP\\n"); }}

void IMU_Port_SendInitError(uint8_t who_am_i)
{{
#if IMU_PORT_BANNERS_ON
    char buf[IMU_TX_FRAME_LEN];
    unsigned pos = 0U;
    if (ImuAppendStr(buf, sizeof(buf), &pos, "#BALANCE:INIT_ERROR=") == 0 &&
        ImuAppendUInt(buf, sizeof(buf), &pos, (uint32_t)who_am_i) == 0 &&
        ImuAppendChar(buf, sizeof(buf), &pos, '\\n') == 0) {{
        QueueTextFrame(&s_banner, buf, pos);
    }}
#else
    (void)who_am_i;
#endif
}}

void IMU_Port_Init(UART_HandleTypeDef *uart)
{{
    s_uart = uart;
    s_banner.length = 0U;
    s_sample.length = 0U;
    IMU_Port_SendHeader();
}}

void IMU_Port_Poll(void)
{{
    if (!s_uart || s_uart->gState != HAL_UART_STATE_READY) {{
        return;
    }}
    ImuTxFrame *next = (s_banner.length != 0U) ? &s_banner : &s_sample;
    if (next->length == 0U) {{
        return;
    }}
    s_in_flight = *next;
    if (HAL_UART_Transmit_IT(s_uart, s_in_flight.data, s_in_flight.length) == HAL_OK) {{
        next->length = 0U;
    }}
}}

void IMU_Port_ReportSample({sig_args})
{{
    char buf[IMU_TX_FRAME_LEN];
    unsigned pos = 0U;
    int ok = 1;
{c_pack_body}
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, '\\n') == 0);
    if (ok) {{
        QueueTextFrame(&s_sample, buf, pos);
    }}
}}
"""

        main_imu = f"""/* STM32 HAL MPU6050 / IMU 串口联调示例（{uart_handle} @ {baudrate}，每 {report_ms} ms 上报） */
#include "main.h"
#include "imu_port_hal.h"

extern UART_HandleTypeDef {uart_handle};

void App_ImuTelemetryDemo(void)
{{
    IMU_Port_Init(&{uart_handle});
    IMU_Port_SendCalibrating();
    /* TODO: 初始化 MPU6050 并静止采零偏；若 WHO_AM_I 异常可调用 IMU_Port_SendInitError(code); */
    IMU_Port_SendReady();

    uint32_t last_rep = HAL_GetTick();
    while (1) {{
        IMU_Port_Poll();
        uint32_t now = HAL_GetTick();
        if (now - last_rep >= IMU_PORT_REPORT_MS) {{
            last_rep = now;
            {demo_decl}
            /* TODO: 填入真实 MPU6050 / 互补滤波读取值 */
            IMU_Port_ReportSample({demo_vals});
            (void)call_guard;
        }}
    }}
}}
""".replace("            (void)call_guard;\n", "")
        return {
            "main_imu_example.c": main_imu,
            "imu_port_hal.h": imu_hal_h,
            "imu_port_hal.c": imu_hal_c,
            "README_接入说明.md": readme,
        }

    if platform == 1:
        irq_name = _usart_irq_name(uart_handle)
        usart_init_lines = _usart_rcc_setup(uart_handle, baudrate)
        imu_spl_h = f"""#ifndef IMU_PORT_SPL_H_
#define IMU_PORT_SPL_H_

#include <stdint.h>
#include "stm32f10x.h"

#define IMU_PORT_FORMAT_ID  {imu_fmt}U
#define IMU_PORT_REPORT_MS  {report_ms}U
#define IMU_PORT_BANNERS_ON {1 if emit_banners else 0}U

void IMU_Port_SPL_Init(USART_TypeDef *usart);
void IMU_Port_SPL_SysTick1ms(void);
uint32_t IMU_Port_SPL_GetTick(void);
void IMU_Port_SPL_IRQHandler(void);
void IMU_Port_SPL_Poll(void);
void IMU_Port_SPL_SendHeader(void);
void IMU_Port_SPL_SendCalibrating(void);
void IMU_Port_SPL_SendReady(void);
void IMU_Port_SPL_SendInitError(uint8_t who_am_i);
void IMU_Port_SPL_SendTiltStop(void);
void IMU_Port_SPL_ReportSample({sig_args});

#endif
"""

        imu_spl_c = f"""#include "imu_port_spl.h"
#include <string.h>

#define IMU_TX_FRAME_LEN 128U

typedef struct {{
    uint16_t length;
    uint8_t data[IMU_TX_FRAME_LEN];
}} ImuSplFrame;

static USART_TypeDef *s_usart;
static volatile uint32_t s_tick_ms;
static ImuSplFrame s_banner;
static ImuSplFrame s_sample;
static ImuSplFrame s_in_flight;
static volatile uint16_t s_tx_pos;
static volatile uint8_t s_tx_active;

{_build_imu_helpers_c(imu_fmt, emit_banners)}

static void QueueTextFrame(ImuSplFrame *slot, const char *text, unsigned len)
{{
    if (!text || len == 0U || len > sizeof(slot->data)) {{
        return;
    }}
    memcpy(slot->data, text, len);
    slot->length = (uint16_t)len;
}}

static void SendStaticBanner(const char *msg)
{{
#if IMU_PORT_BANNERS_ON
    QueueTextFrame(&s_banner, msg, (unsigned)strlen(msg));
    IMU_Port_SPL_Poll();
#else
    (void)msg;
#endif
}}

void IMU_Port_SPL_SysTick1ms(void) {{ s_tick_ms++; }}
uint32_t IMU_Port_SPL_GetTick(void) {{ return s_tick_ms; }}

void IMU_Port_SPL_SendHeader(void)
{{
    static const char hdr[] = "{names_hdr}\\n";
    QueueTextFrame(&s_banner, hdr, (unsigned)(sizeof(hdr) - 1U));
}}

void IMU_Port_SPL_SendCalibrating(void) {{ SendStaticBanner("#BALANCE:CALIBRATING\\n"); }}
void IMU_Port_SPL_SendReady(void) {{ IMU_Port_SPL_SendHeader(); SendStaticBanner("#BALANCE:READY\\n"); }}
void IMU_Port_SPL_SendTiltStop(void) {{ SendStaticBanner("#BALANCE:TILT_STOP\\n"); }}

void IMU_Port_SPL_SendInitError(uint8_t who_am_i)
{{
#if IMU_PORT_BANNERS_ON
    char buf[IMU_TX_FRAME_LEN];
    unsigned pos = 0U;
    if (ImuAppendStr(buf, sizeof(buf), &pos, "#BALANCE:INIT_ERROR=") == 0 &&
        ImuAppendUInt(buf, sizeof(buf), &pos, (uint32_t)who_am_i) == 0 &&
        ImuAppendChar(buf, sizeof(buf), &pos, '\\n') == 0) {{
        QueueTextFrame(&s_banner, buf, pos);
    }}
#else
    (void)who_am_i;
#endif
}}

void IMU_Port_SPL_Init(USART_TypeDef *usart)
{{
    s_usart = usart;
    s_banner.length = 0U;
    s_sample.length = 0U;
    s_tx_active = 0U;
    s_tx_pos = 0U;
    IMU_Port_SPL_SendHeader();
}}

void IMU_Port_SPL_IRQHandler(void)
{{
    if (!s_usart) {{
        return;
    }}
    if (USART_GetITStatus(s_usart, USART_IT_TXE) != RESET) {{
        if (s_tx_active && s_tx_pos < s_in_flight.length) {{
            USART_SendData(s_usart, s_in_flight.data[s_tx_pos++]);
        }} else {{
            s_tx_active = 0U;
            USART_ITConfig(s_usart, USART_IT_TXE, DISABLE);
        }}
    }}
}}

void IMU_Port_SPL_Poll(void)
{{
    if (!s_usart || s_tx_active) {{
        return;
    }}
    ImuSplFrame *next = (s_banner.length != 0U) ? &s_banner : &s_sample;
    if (next->length == 0U) {{
        return;
    }}
    s_in_flight = *next;
    next->length = 0U;
    s_tx_pos = 0U;
    s_tx_active = 1U;
    USART_ITConfig(s_usart, USART_IT_TXE, ENABLE);
}}

void IMU_Port_SPL_ReportSample({sig_args})
{{
    char buf[IMU_TX_FRAME_LEN];
    unsigned pos = 0U;
    int ok = 1;
{c_pack_body}
    ok &= (ImuAppendChar(buf, sizeof(buf), &pos, '\\n') == 0);
    if (ok) {{
        QueueTextFrame(&s_sample, buf, pos);
    }}
}}
"""

        main_imu_spl = f"""/* STM32 标准库 SPL MPU6050 / IMU 串口联调示例（{uart_handle} @ {baudrate}） */
#include "stm32f10x.h"
#include "imu_port_spl.h"

void SysTick_Handler(void) {{ IMU_Port_SPL_SysTick1ms(); }}
void {irq_name}(void) {{ IMU_Port_SPL_IRQHandler(); }}

static void Board_UsartInit(void)
{{
{usart_init_lines}
}}

void App_ImuSplDemo(void)
{{
    SysTick_Config(SystemCoreClock / 1000U);
    Board_UsartInit();
    IMU_Port_SPL_Init({uart_handle});
    IMU_Port_SPL_SendCalibrating();
    /* TODO: 初始化 MPU6050 并采零偏 */
    IMU_Port_SPL_SendReady();

    uint32_t last_rep = IMU_Port_SPL_GetTick();
    while (1) {{
        IMU_Port_SPL_Poll();
        uint32_t now = IMU_Port_SPL_GetTick();
        if (now - last_rep >= IMU_PORT_REPORT_MS) {{
            last_rep = now;
            {demo_decl}
            /* TODO: 填充你的 MPU6050 / 滤波姿态数据 */
            IMU_Port_SPL_ReportSample({demo_vals});
        }}
    }}
}}
"""
        return {
            "main_imu_spl.c": main_imu_spl,
            "imu_port_spl.h": imu_spl_h,
            "imu_port_spl.c": imu_spl_c,
            "README_接入说明.md": readme,
        }

    # Platform 2: ESP32 / Arduino MPU6050 Wire I2C + Serial
    if imu_fmt == 0:
        ino_report = """  IMU_SERIAL.print(ax_g, 4);   IMU_SERIAL.print(',');
  IMU_SERIAL.print(ay_g, 4);   IMU_SERIAL.print(',');
  IMU_SERIAL.print(az_g, 4);   IMU_SERIAL.print(',');
  IMU_SERIAL.print(gx_dps, 2); IMU_SERIAL.print(',');
  IMU_SERIAL.print(gy_dps, 2); IMU_SERIAL.print(',');
  IMU_SERIAL.println(gz_dps, 2);"""
    elif imu_fmt == 1:
        ino_report = """  IMU_SERIAL.print(ax); IMU_SERIAL.print(',');
  IMU_SERIAL.print(ay); IMU_SERIAL.print(',');
  IMU_SERIAL.print(az); IMU_SERIAL.print(',');
  IMU_SERIAL.print(gx); IMU_SERIAL.print(',');
  IMU_SERIAL.print(gy); IMU_SERIAL.print(',');
  IMU_SERIAL.println(gz);"""
    elif imu_fmt == 2:
        ino_report = """  // 平衡车四列：target, pitch, gyro, pwm
  IMU_SERIAL.print(0.00f, 2);     IMU_SERIAL.print(',');
  IMU_SERIAL.print(pitch_deg, 2); IMU_SERIAL.print(',');
  IMU_SERIAL.print(gy_dps, 2);    IMU_SERIAL.print(',');
  IMU_SERIAL.println(0);"""
    else:
        ino_report = """  // 欧拉角三列：pitch, roll, yaw
  IMU_SERIAL.print(pitch_deg, 2); IMU_SERIAL.print(',');
  IMU_SERIAL.print(roll_deg, 2);  IMU_SERIAL.print(',');
  IMU_SERIAL.println(yaw_deg, 2);"""

    banner_code = (
        f'  IMU_SERIAL.println(F("#BALANCE:CALIBRATING"));\n'
        f'  // 上电静止采 100 帧陀螺仪零偏\n'
        f'  calibrateGyroBias();\n'
        f'  IMU_SERIAL.println(F("{names_hdr}"));\n'
        f'  IMU_SERIAL.println(F("#BALANCE:READY"));'
    ) if emit_banners else f'  calibrateGyroBias();\n  IMU_SERIAL.println(F("{names_hdr}"));'

    ino = f"""/* EmbeddedDebugStudio · MPU6050 姿态直连草图（ESP32 / Arduino 通用）
 * 格式：{fmt_label}
 * 接线：VCC->3.3V/5V, GND->GND, SDA/SCL->MCU I2C（ESP32 默认 GPIO21/22，Arduino Uno 为 A4/A5）
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define IMU_SERIAL {uart_handle}
static const uint8_t MPU_ADDR = 0x68;
static float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
static float pitch_deg = 0.0f, roll_deg = 0.0f, yaw_deg = 0.0f;

bool mpuWrite(uint8_t reg, uint8_t val) {{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}}

void mpuRead6Axis(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 14);
  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read(); // 跳过温度寄存器
  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
}}

void calibrateGyroBias() {{
  long sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 100; ++i) {{
    int16_t ax, ay, az, gx, gy, gz;
    mpuRead6Axis(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    delay(5);
  }}
  gx_bias = (sx / 100.0f) / 131.0f;
  gy_bias = (sy / 100.0f) / 131.0f;
  gz_bias = (sz / 100.0f) / 131.0f;
}}

void setup() {{
  IMU_SERIAL.begin({baudrate});
  Wire.begin();
  if (!mpuWrite(0x6B, 0x00)) {{
    IMU_SERIAL.println(F("#BALANCE:INIT_ERROR=1"));
    while (1) delay(500);
  }}
  mpuWrite(0x1B, 0x00); // ±250 °/s (131 LSB/°/s)
  mpuWrite(0x1C, 0x00); // ±2 g (16384 LSB/g)
  delay(50);
{banner_code}
}}

void loop() {{
  static unsigned long last_rep = 0;
  unsigned long now = millis();
  if (now - last_rep < {report_ms}UL) return;
  float dt = (last_rep == 0) ? ({report_ms}UL * 0.001f) : ((now - last_rep) * 0.001f);
  last_rep = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpuRead6Axis(ax, ay, az, gx, gy, gz);
  float ax_g = ax / 16384.0f, ay_g = ay / 16384.0f, az_g = az / 16384.0f;
  float gx_dps = gx / 131.0f - gx_bias;
  float gy_dps = gy / 131.0f - gy_bias;
  float gz_dps = gz / 131.0f - gz_bias;
  float acc_pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;
  float acc_roll  = atan2f(ay_g, az_g) * 57.29578f;
  pitch_deg = 0.96f * (pitch_deg + gy_dps * dt) + 0.04f * acc_pitch;
  roll_deg  = 0.96f * (roll_deg  + gx_dps * dt) + 0.04f * acc_roll;
  yaw_deg  += gz_dps * dt;
  (void)pitch_deg; (void)roll_deg; (void)yaw_deg; (void)ax_g; (void)gy_dps;

{ino_report}
}}
"""
    return {
        "EmbeddedDebugStudio_IMU.ino": ino,
        "README_接入说明.md": readme,
    }


def _extract_drive_prefix(cmd_fmt: str) -> str:
    raw = str(cmd_fmt or "J,{x},{y}").strip()
    if "{x}" in raw:
        prefix = raw.split("{x}", 1)[0]
    else:
        prefix = "J,"
    return prefix or "J,"


def _c_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def _generate_remote_bundle(cfg: dict) -> dict[str, str]:
    platform = int(cfg.get("platform", 0))
    if platform not in (0, 1, 2):
        platform = 0
    uart_handle = _safe_uart_handle(cfg.get("uart_handle", ""), platform)
    baudrate = int(cfg.get("baudrate", 9600))
    cmd_fmt = str(cfg.get("remote_cmd_fmt", "J,{x},{y}")).strip() or "J,{x},{y}"
    drive_prefix = _extract_drive_prefix(cmd_fmt)
    pwm_limit = max(1, int(round(float(cfg.get("remote_pwm_limit", 1000)))))
    failsafe_ms = max(100, int(cfg.get("remote_failsafe_ms", 500)))
    aux_cmds = bool(cfg.get("remote_aux_cmds", True))
    plat_name = PLATFORMS[platform]
    prefix_c = _c_escape(drive_prefix)

    readme = f"""# EmbeddedDebugStudio · 小车差速遥控接口 (BlueSerialStudio) 接入说明

- 目标平台：**{plat_name}**
- 接口句柄 / 外设：**`{uart_handle}`** · 波特率：**`{baudrate}`**
- 指令模板：**`{cmd_fmt}`** · 左右轮 PWM 限幅：**`±{pwm_limit}`**
- 失联保护看门狗：**`{failsafe_ms} ms`**（超时未收到遥控报文自动停车清零 PWM） · 辅助指令（灯/蜂鸣/舵机）：`{"开启" if aux_cmds else "关闭"}`

## 协议与差速混控说明
- 上位机与 Android 端 `BlueSerialStudio` 均发送 `{drive_prefix}x,y\\n`（`x,y ∈ [-100, 100]`），急停发送 `S\\n` 或 `{drive_prefix}0,0\\n`。
- 原地旋转（`y == 0`）：左轮 `+x`，右轮 `-x`；行驶转向时按 `100 - |x|` 百分比缩减内侧轮并放大至 `±{pwm_limit}`。

## 联调步骤
1. 若接 HC-05 蓝牙模块，请先在 **【🎮 遥控与 HC-05】(`Ctrl+4`)** 用 USB-TTL（`38400` 波特率）把模块串口配成 `{baudrate}`。
2. 烧录本接口代码并交叉连接串口（TX↔RX），即可直接拖拽单摇杆、双摇杆或按 `W/A/S/D` 遥控；按 `Space` 或 `Esc` 急停。
"""

    c_parser_core = f"""static int ClampInt(int v, int lo, int hi)
{{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}}

static int ParseSignedInt(const char *s, const char **end_ptr, int *out)
{{
    int sign = 1;
    int val = 0;
    int digits = 0;
    while (*s == ' ') {{
        s++;
    }}
    if (*s == '-') {{
        sign = -1;
        s++;
    }} else if (*s == '+') {{
        s++;
    }}
    while (*s >= '0' && *s <= '9') {{
        val = val * 10 + (*s - '0');
        digits++;
        s++;
        if (val > 100000) {{
            val = 100000;
        }}
    }}
    if (digits == 0) {{
        return 0;
    }}
    *out = sign * val;
    if (end_ptr) {{
        *end_ptr = s;
    }}
    return 1;
}}"""

    aux_decl_h_hal = (
        "\n/** @brief 可选辅助指令回调：cmd 如 \"LED_ON\" / \"LED_OFF\" / \"BEEP\" / \"SERVO\"，arg 为附加数值（如舵机角度）。 */\n"
        "void Remote_Port_OnAuxCommand(const char *cmd, int arg);\n"
    ) if aux_cmds else ""

    aux_decl_h_spl = (
        "\n/** @brief 可选辅助指令回调：cmd 如 \"LED_ON\" / \"LED_OFF\" / \"BEEP\" / \"SERVO\"，arg 为附加数值。 */\n"
        "void Remote_Port_SPL_OnAuxCommand(const char *cmd, int arg);\n"
    ) if aux_cmds else ""

    if platform == 0:
        remote_hal_h = f"""#ifndef REMOTE_PORT_HAL_H_
#define REMOTE_PORT_HAL_H_

#include <stdint.h>
#include "main.h"

#define REMOTE_DRIVE_PREFIX  "{prefix_c}"
#define REMOTE_PWM_LIMIT     {pwm_limit}
#define REMOTE_FAILSAFE_MS   {failsafe_ms}U

void Remote_Port_Init(UART_HandleTypeDef *uart);
void Remote_Port_Poll(void);
void Remote_Port_RxCpltCallback(UART_HandleTypeDef *uart);
void Remote_Port_ErrorCallback(UART_HandleTypeDef *uart);

/** @brief 与 BlueSerialStudio 完全一致的差速混控函数：输入 x,y(-100..100)，输出左右轮 PWM。 */
void Remote_MixWheels(int x, int y, int16_t *left_out, int16_t *right_out);

/** @brief 用户硬件驱动钩子：把左右轮限幅后的 PWM 值写入电机驱动（急停/失联自动传 0, 0）。 */
void Remote_Port_ApplyWheels(int16_t left_pwm, int16_t right_pwm);{aux_decl_h_hal}
#endif
"""

        aux_dispatch_hal = """    } else if (strcmp(line, "LED_ON") == 0) {
        Remote_Port_OnAuxCommand("LED_ON", 1);
    } else if (strcmp(line, "LED_OFF") == 0) {
        Remote_Port_OnAuxCommand("LED_OFF", 0);
    } else if (strcmp(line, "BEEP") == 0) {
        Remote_Port_OnAuxCommand("BEEP", 1);
    } else if (strncmp(line, "SERVO,", 6) == 0) {
        int deg = 90;
        if (ParseSignedInt(line + 6, NULL, &deg)) {
            Remote_Port_OnAuxCommand("SERVO", ClampInt(deg, 0, 180));
        }""" if aux_cmds else ""

        remote_hal_c = f"""#include "remote_port_hal.h"
#include <string.h>

#define REMOTE_RX_FIFO_SIZE 256U
#define REMOTE_LINE_MAX     64U

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static volatile uint8_t s_fifo[REMOTE_RX_FIFO_SIZE];
static volatile uint8_t s_rd, s_wr, s_overflow, s_rearm;

static char s_line[REMOTE_LINE_MAX];
static uint8_t s_line_len;
static uint32_t s_last_drive_ms;
static uint8_t s_moving;

{c_parser_core}

void Remote_MixWheels(int x, int y, int16_t *left_out, int16_t *right_out)
{{
    x = ClampInt(x, -100, 100);
    y = ClampInt(y, -100, 100);
    int left_pct = 0;
    int right_pct = 0;
    if (y == 0) {{
        left_pct = x;
        right_pct = -x;
    }} else {{
        int mag = (x < 0) ? -x : x;
        int abs_y = (y < 0) ? -y : y;
        int inner = (abs_y * (100 - mag) + 50) / 100;
        if (x > 0) {{
            if (y > 0) {{
                left_pct = y;
                right_pct = inner;
            }} else {{
                left_pct = -inner;
                right_pct = y;
            }}
        }} else if (x < 0) {{
            if (y > 0) {{
                left_pct = inner;
                right_pct = y;
            }} else {{
                left_pct = y;
                right_pct = -inner;
            }}
        }} else {{
            left_pct = y;
            right_pct = y;
        }}
    }}
    *left_out = (int16_t)((left_pct * REMOTE_PWM_LIMIT) / 100);
    *right_out = (int16_t)((right_pct * REMOTE_PWM_LIMIT) / 100);
}}

static void HandleLine(const char *line)
{{
    static const char prefix[] = REMOTE_DRIVE_PREFIX;
    const size_t plen = sizeof(prefix) - 1U;
    if (strcmp(line, "S") == 0 || strcmp(line, "STOP") == 0) {{
        s_moving = 0U;
        Remote_Port_ApplyWheels(0, 0);
        return;
    }}
    if (strncmp(line, prefix, plen) == 0) {{
        int x = 0, y = 0;
        const char *p = NULL;
        if (ParseSignedInt(line + plen, &p, &x) && p && *p == ',' &&
            ParseSignedInt(p + 1, NULL, &y)) {{
            int16_t lw = 0, rw = 0;
            Remote_MixWheels(x, y, &lw, &rw);
            s_last_drive_ms = HAL_GetTick();
            s_moving = (uint8_t)(lw != 0 || rw != 0);
            Remote_Port_ApplyWheels(lw, rw);
        }}
{aux_dispatch_hal}
    }}
}}

void Remote_Port_Init(UART_HandleTypeDef *uart)
{{
    s_uart = uart;
    s_rd = s_wr = s_overflow = s_line_len = s_moving = 0U;
    s_last_drive_ms = HAL_GetTick();
    Remote_Port_ApplyWheels(0, 0);
    s_rearm = (uint8_t)(HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK);
}}

void Remote_Port_RxCpltCallback(UART_HandleTypeDef *uart)
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

void Remote_Port_ErrorCallback(UART_HandleTypeDef *uart)
{{
    if (uart == s_uart) {{
        s_overflow = 1U;
        s_rearm = 1U;
    }}
}}

void Remote_Port_Poll(void)
{{
    if (s_rearm && s_uart) {{
        HAL_UART_AbortReceive(s_uart);
        s_rearm = (uint8_t)(HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK);
    }}
    for (unsigned drained = 0U; drained < REMOTE_RX_FIFO_SIZE; ++drained) {{
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
            s_line_len = 0U;
        }}
        if (!has_byte) {{
            break;
        }}
        if (b == '\\n' || b == '\\r') {{
            if (s_line_len > 0U) {{
                s_line[s_line_len] = '\\0';
                HandleLine(s_line);
                s_line_len = 0U;
            }}
        }} else if (s_line_len + 1U < sizeof(s_line)) {{
            s_line[s_line_len++] = (char)b;
        }}
    }}
    if (s_moving && (HAL_GetTick() - s_last_drive_ms) >= REMOTE_FAILSAFE_MS) {{
        s_moving = 0U;
        Remote_Port_ApplyWheels(0, 0);
    }}
}}
"""

        aux_stub = (
            "\nvoid Remote_Port_OnAuxCommand(const char *cmd, int arg)\n"
            "{\n    (void)cmd;\n    (void)arg;\n"
            "    /* TODO: 根据 cmd 处理 LED_ON / LED_OFF / BEEP / SERVO */\n}\n"
        ) if aux_cmds else ""

        main_remote = f"""/* STM32 HAL 差速遥控与 HC-05 接入示例（{uart_handle} @ {baudrate}，内置 {failsafe_ms} ms 断线停车） */
#include "main.h"
#include "remote_port_hal.h"

extern UART_HandleTypeDef {uart_handle};

void Remote_Port_ApplyWheels(int16_t left_pwm, int16_t right_pwm)
{{
    (void)left_pwm;
    (void)right_pwm;
    /* TODO: 写入左右电机定时器 PWM，例如：
     * Motor_SetLeftPWM(left_pwm);
     * Motor_SetRightPWM(right_pwm);
     */
}}
{aux_stub}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{{
    Remote_Port_RxCpltCallback(huart);
}}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{{
    Remote_Port_ErrorCallback(huart);
}}

void App_RemoteDemo(void)
{{
    Remote_Port_Init(&{uart_handle});
    while (1) {{
        Remote_Port_Poll();
    }}
}}
"""
        return {
            "main_remote_example.c": main_remote,
            "remote_port_hal.h": remote_hal_h,
            "remote_port_hal.c": remote_hal_c,
            "README_接入说明.md": readme,
        }

    if platform == 1:
        irq_name = _usart_irq_name(uart_handle)
        usart_init_lines = _usart_rcc_setup(uart_handle, baudrate)
        remote_spl_h = f"""#ifndef REMOTE_PORT_SPL_H_
#define REMOTE_PORT_SPL_H_

#include <stdint.h>
#include "stm32f10x.h"

#define REMOTE_DRIVE_PREFIX  "{prefix_c}"
#define REMOTE_PWM_LIMIT     {pwm_limit}
#define REMOTE_FAILSAFE_MS   {failsafe_ms}U

void Remote_Port_SPL_Init(USART_TypeDef *usart);
void Remote_Port_SPL_SysTick1ms(void);
void Remote_Port_SPL_IRQHandler(void);
void Remote_Port_SPL_Poll(void);
void Remote_MixWheels(int x, int y, int16_t *left_out, int16_t *right_out);
void Remote_Port_SPL_ApplyWheels(int16_t left_pwm, int16_t right_pwm);{aux_decl_h_spl}
#endif
"""

        aux_dispatch_spl = """    } else if (strcmp(line, "LED_ON") == 0) {
        Remote_Port_SPL_OnAuxCommand("LED_ON", 1);
    } else if (strcmp(line, "LED_OFF") == 0) {
        Remote_Port_SPL_OnAuxCommand("LED_OFF", 0);
    } else if (strcmp(line, "BEEP") == 0) {
        Remote_Port_SPL_OnAuxCommand("BEEP", 1);
    } else if (strncmp(line, "SERVO,", 6) == 0) {
        int deg = 90;
        if (ParseSignedInt(line + 6, NULL, &deg)) {
            Remote_Port_SPL_OnAuxCommand("SERVO", ClampInt(deg, 0, 180));
        }""" if aux_cmds else ""

        remote_spl_c = f"""#include "remote_port_spl.h"
#include <string.h>

#define REMOTE_RX_FIFO_SIZE 256U
#define REMOTE_LINE_MAX     64U

static USART_TypeDef *s_usart;
static volatile uint32_t s_tick_ms;
static volatile uint8_t s_fifo[REMOTE_RX_FIFO_SIZE];
static volatile uint8_t s_rd, s_wr, s_overflow;

static char s_line[REMOTE_LINE_MAX];
static uint8_t s_line_len;
static uint32_t s_last_drive_ms;
static uint8_t s_moving;

{c_parser_core}

void Remote_Port_SPL_SysTick1ms(void)
{{
    s_tick_ms++;
}}

void Remote_MixWheels(int x, int y, int16_t *left_out, int16_t *right_out)
{{
    x = ClampInt(x, -100, 100);
    y = ClampInt(y, -100, 100);
    int left_pct = 0;
    int right_pct = 0;
    if (y == 0) {{
        left_pct = x;
        right_pct = -x;
    }} else {{
        int mag = (x < 0) ? -x : x;
        int abs_y = (y < 0) ? -y : y;
        int inner = (abs_y * (100 - mag) + 50) / 100;
        if (x > 0) {{
            if (y > 0) {{
                left_pct = y;
                right_pct = inner;
            }} else {{
                left_pct = -inner;
                right_pct = y;
            }}
        }} else if (x < 0) {{
            if (y > 0) {{
                left_pct = inner;
                right_pct = y;
            }} else {{
                left_pct = y;
                right_pct = -inner;
            }}
        }} else {{
            left_pct = y;
            right_pct = y;
        }}
    }}
    *left_out = (int16_t)((left_pct * REMOTE_PWM_LIMIT) / 100);
    *right_out = (int16_t)((right_pct * REMOTE_PWM_LIMIT) / 100);
}}

static void HandleLine(const char *line)
{{
    static const char prefix[] = REMOTE_DRIVE_PREFIX;
    const size_t plen = sizeof(prefix) - 1U;
    if (strcmp(line, "S") == 0 || strcmp(line, "STOP") == 0) {{
        s_moving = 0U;
        Remote_Port_SPL_ApplyWheels(0, 0);
        return;
    }}
    if (strncmp(line, prefix, plen) == 0) {{
        int x = 0, y = 0;
        const char *p = NULL;
        if (ParseSignedInt(line + plen, &p, &x) && p && *p == ',' &&
            ParseSignedInt(p + 1, NULL, &y)) {{
            int16_t lw = 0, rw = 0;
            Remote_MixWheels(x, y, &lw, &rw);
            s_last_drive_ms = s_tick_ms;
            s_moving = (uint8_t)(lw != 0 || rw != 0);
            Remote_Port_SPL_ApplyWheels(lw, rw);
        }}
{aux_dispatch_spl}
    }}
}}

void Remote_Port_SPL_Init(USART_TypeDef *usart)
{{
    s_usart = usart;
    s_rd = s_wr = s_overflow = s_line_len = s_moving = 0U;
    s_last_drive_ms = s_tick_ms;
    Remote_Port_SPL_ApplyWheels(0, 0);
    if (s_usart) {{
        USART_ITConfig(s_usart, USART_IT_RXNE, ENABLE);
    }}
}}

void Remote_Port_SPL_IRQHandler(void)
{{
    if (s_usart && USART_GetITStatus(s_usart, USART_IT_RXNE) != RESET) {{
        uint8_t b = (uint8_t)USART_ReceiveData(s_usart);
        uint8_t next = (uint8_t)(s_wr + 1U);
        if (next == s_rd) {{
            s_overflow = 1U;
        }} else {{
            s_fifo[s_wr] = b;
            s_wr = next;
        }}
    }}
}}

void Remote_Port_SPL_Poll(void)
{{
    for (unsigned drained = 0U; drained < REMOTE_RX_FIFO_SIZE; ++drained) {{
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
            s_line_len = 0U;
        }}
        if (!has_byte) {{
            break;
        }}
        if (b == '\\n' || b == '\\r') {{
            if (s_line_len > 0U) {{
                s_line[s_line_len] = '\\0';
                HandleLine(s_line);
                s_line_len = 0U;
            }}
        }} else if (s_line_len + 1U < sizeof(s_line)) {{
            s_line[s_line_len++] = (char)b;
        }}
    }}
    if (s_moving && (s_tick_ms - s_last_drive_ms) >= REMOTE_FAILSAFE_MS) {{
        s_moving = 0U;
        Remote_Port_SPL_ApplyWheels(0, 0);
    }}
}}
"""

        aux_stub_spl = (
            "\nvoid Remote_Port_SPL_OnAuxCommand(const char *cmd, int arg)\n"
            "{\n    (void)cmd;\n    (void)arg;\n"
            "    /* TODO: 响应 LED_ON / LED_OFF / BEEP / SERVO */\n}\n"
        ) if aux_cmds else ""

        main_remote_spl = f"""/* STM32 标准库 SPL 差速遥控与 HC-05 接入示例（{uart_handle} @ {baudrate}） */
#include "stm32f10x.h"
#include "remote_port_spl.h"

void SysTick_Handler(void) {{ Remote_Port_SPL_SysTick1ms(); }}
void {irq_name}(void) {{ Remote_Port_SPL_IRQHandler(); }}

void Remote_Port_SPL_ApplyWheels(int16_t left_pwm, int16_t right_pwm)
{{
    (void)left_pwm;
    (void)right_pwm;
    /* TODO: 设置左右轮电机 PWM */
}}
{aux_stub_spl}
static void Board_UsartInit(void)
{{
{usart_init_lines}
}}

void App_RemoteSplDemo(void)
{{
    SysTick_Config(SystemCoreClock / 1000U);
    Board_UsartInit();
    Remote_Port_SPL_Init({uart_handle});
    while (1) {{
        Remote_Port_SPL_Poll();
    }}
}}
"""
        return {
            "main_remote_spl.c": main_remote_spl,
            "remote_port_spl.h": remote_spl_h,
            "remote_port_spl.c": remote_spl_c,
            "README_接入说明.md": readme,
        }

    # Platform 2: ESP32 / Arduino sketch
    aux_ino = """  else if (line == "LED_ON")  { /* TODO: digitalWrite(LED_PIN, HIGH); */ }
  else if (line == "LED_OFF") { /* TODO: digitalWrite(LED_PIN, LOW); */ }
  else if (line == "BEEP")    { /* TODO: tone(...) */ }
  else if (line.startsWith("SERVO,")) {
    int deg = constrain(line.substring(6).toInt(), 0, 180);
    (void)deg; // TODO: 写入舵机角度
  }""" if aux_cmds else ""

    ino = f"""/* EmbeddedDebugStudio / BlueSerialStudio 小车差速遥控草图（ESP32 / Arduino 通用）
 * 指令：{cmd_fmt} · 急停：S / {drive_prefix}0,0 · 失联自动停车：{failsafe_ms} ms
 * 若连接 HC-05 模块，可将下方 #define REMOTE_SERIAL 改为 Serial2 (ESP32) 或 SoftwareSerial (Uno)
 */
#include <Arduino.h>

#define REMOTE_SERIAL      {uart_handle}
#define REMOTE_PWM_LIMIT   {pwm_limit}
#define REMOTE_FAILSAFE_MS {failsafe_ms}UL
static const char DRIVE_PREFIX[] = "{prefix_c}";

static String rx_buf;
static unsigned long last_drive_ms = 0;
static bool moving = false;

void applyWheels(int left_pwm, int right_pwm) {{
  (void)left_pwm;
  (void)right_pwm;
  // TODO: 将 [-{pwm_limit}, +{pwm_limit}] 的左右轮 PWM 写入你的电机驱动（如 L298N / TB6612）
}}

void mixWheels(int x, int y, int &left_out, int &right_out) {{
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  int lp = 0, rp = 0;
  if (y == 0) {{
    lp = x; rp = -x;
  }} else {{
    int mag = (x < 0) ? -x : x;
    int abs_y = (y < 0) ? -y : y;
    int inner = (abs_y * (100 - mag) + 50) / 100;
    if (x > 0) {{
      if (y > 0) {{ lp = y; rp = inner; }} else {{ lp = -inner; rp = y; }}
    }} else if (x < 0) {{
      if (y > 0) {{ lp = inner; rp = y; }} else {{ lp = y; rp = -inner; }}
    }} else {{
      lp = y; rp = y;
    }}
  }}
  left_out  = (lp * REMOTE_PWM_LIMIT) / 100;
  right_out = (rp * REMOTE_PWM_LIMIT) / 100;
}}

void handleLine(String line) {{
  line.trim();
  if (line == "S" || line == "STOP") {{
    moving = false;
    applyWheels(0, 0);
    return;
  }}
  int plen = sizeof(DRIVE_PREFIX) - 1;
  if (line.startsWith(DRIVE_PREFIX)) {{
    String rest = line.substring(plen);
    int comma = rest.indexOf(',');
    if (comma > 0) {{
      int x = rest.substring(0, comma).toInt();
      int y = rest.substring(comma + 1).toInt();
      int lw = 0, rw = 0;
      mixWheels(x, y, lw, rw);
      last_drive_ms = millis();
      moving = (lw != 0 || rw != 0);
      applyWheels(lw, rw);
    }}
  }}
{aux_ino}
}}

void setup() {{
  REMOTE_SERIAL.begin({baudrate});
  applyWheels(0, 0);
}}

void loop() {{
  while (REMOTE_SERIAL.available()) {{
    char c = (char)REMOTE_SERIAL.read();
    if (c == '\\n' || c == '\\r') {{
      if (rx_buf.length() > 0) {{ handleLine(rx_buf); rx_buf = ""; }}
    }} else if (rx_buf.length() < 80) {{
      rx_buf += c;
    }}
  }}
  if (moving && millis() - last_drive_ms >= REMOTE_FAILSAFE_MS) {{
    moving = false;
    applyWheels(0, 0);
  }}
}}
"""
    return {
        "EmbeddedDebugStudio_Remote.ino": ino,
        "README_接入说明.md": readme,
    }


def generate_interface_bundle(config: dict) -> dict[str, str]:
    """Generate complete C99/Arduino interface files for (interface_type, platform)."""
    iface = int(config.get("interface_type", 0))
    if iface == 1:
        return _generate_imu_bundle(config)
    if iface == 2:
        return _generate_remote_bundle(config)
    return _generate_pid_bundle(config)


class FirmwareStudioPage(W.QWidget):
    """5th Workbench module: unified MCU serial interface generator & per-interface guide."""

    module_id = "firmware_builder"
    module_title = "🧩 接口构建"

    def __init__(self, host=None):
        super().__init__()
        self.host = host
        self._bundle: dict[str, str] = {}
        self._suppress_regen = False
        self._channel_rows: list[tuple[W.QWidget, W.QLineEdit, W.QComboBox]] = []
        self._selected_row = 0
        self._iface_type = 0
        self._last_platform = 0
        self._build_ui()
        self.apply_preset(0)
        self.set_interface_type(0)

    def _build_ui(self):
        root = W.QHBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(6)

        self.splitter = W.QSplitter(C.Qt.Orientation.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setHandleWidth(6)
        root.addWidget(self.splitter, 1)

        # ---------- Left scrollable builder config ----------
        left_scroll = W.QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(W.QFrame.Shape.NoFrame)
        left_scroll.setMinimumWidth(320)
        left_scroll.setHorizontalScrollBarPolicy(C.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        left_box = W.QWidget()
        left_box.setFocusPolicy(C.Qt.FocusPolicy.ClickFocus)
        left_layout = W.QVBoxLayout(left_box)
        left_layout.setContentsMargins(2, 2, 6, 2)
        left_layout.setSpacing(8)
        left_scroll.setWidget(left_box)
        self.splitter.addWidget(left_scroll)

        # 0. Interface Type Switcher Card
        iface_card = W.QFrame()
        iface_card.setObjectName("card")
        iv = W.QVBoxLayout(iface_card)
        iv.setContentsMargins(10, 8, 10, 8)
        iv.setSpacing(6)
        iv.addWidget(W.QLabel("<b>🧩 接口类型（集中构建全软件三大串口接口）</b>"))
        iface_row = W.QHBoxLayout()
        iface_row.setSpacing(4)
        self.iface_group = W.QButtonGroup(self)
        self.iface_group.setExclusive(True)
        self.iface_buttons: list[W.QPushButton] = []
        short_labels = ("📈 PID 调参", "🧭 IMU 姿态", "🎮 差速遥控")
        for idx, title in enumerate(short_labels):
            btn = W.QPushButton(title)
            btn.setCheckable(True)
            btn.setMinimumWidth(0)
            btn.setToolTip(INTERFACE_TYPES[idx])
            btn.clicked.connect(lambda _=False, i=idx: self.set_interface_type(i))
            self.iface_group.addButton(btn, idx)
            iface_row.addWidget(btn, 1)
            self.iface_buttons.append(btn)
        iv.addLayout(iface_row)
        left_layout.addWidget(iface_card)

        # 1. Shared Platform & Serial Card
        plat_card = W.QFrame()
        plat_card.setObjectName("card")
        pf = W.QFormLayout(plat_card)
        pf.setFieldGrowthPolicy(W.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        pf.setContentsMargins(10, 8, 10, 8)
        pf.setSpacing(6)
        pf.addRow(W.QLabel("<b>⚙ 目标库与串口基础配置</b>"))

        self.platform_box = W.QComboBox()
        self.platform_box.addItems(list(PLATFORMS))
        self.platform_box.setMinimumWidth(0)
        self.platform_box.setMinimumContentsLength(10)
        self.platform_box.setSizePolicy(W.QSizePolicy.Policy.Expanding, W.QSizePolicy.Policy.Fixed)
        self.platform_box.setSizeAdjustPolicy(W.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.uart_edit = W.QLineEdit("huart2")
        self.uart_edit.setMinimumWidth(0)
        self.uart_label = W.QLabel("UART 句柄")
        self.baud_box = W.QComboBox()
        self.baud_box.setEditable(True)
        self.baud_box.setMinimumWidth(0)
        self.baud_box.setSizePolicy(W.QSizePolicy.Policy.Expanding, W.QSizePolicy.Policy.Fixed)
        self.baud_box.addItems(["115200", "256000", "460800", "921600", "38400", "9600"])

        pf.addRow("目标库 / 平台", self.platform_box)
        pf.addRow(self.uart_label, self.uart_edit)
        pf.addRow("波特率", self.baud_box)
        left_layout.addWidget(plat_card)

        # ---------- PID Cards (iface == 0) ----------
        self.pid_preset_card = W.QFrame()
        self.pid_preset_card.setObjectName("card")
        pv = W.QVBoxLayout(self.pid_preset_card)
        pv.setContentsMargins(10, 8, 10, 8)
        pv.setSpacing(6)
        pv.addWidget(W.QLabel("<b>⚡ PID 场景预设模板（一键填充）</b>"))
        pgrid = W.QGridLayout()
        pgrid.setSpacing(6)
        for idx, (label, _) in enumerate(INTERFACE_PRESETS):
            btn = W.QPushButton(label)
            btn.setMinimumWidth(0)
            btn.clicked.connect(lambda _=False, i=idx: self.apply_preset(i))
            pgrid.addWidget(btn, idx // 2, idx % 2)
        pv.addLayout(pgrid)
        left_layout.addWidget(self.pid_preset_card)

        self.pid_cfg_card = W.QFrame()
        self.pid_cfg_card.setObjectName("card")
        cf = W.QFormLayout(self.pid_cfg_card)
        cf.setFieldGrowthPolicy(W.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        cf.setContentsMargins(10, 8, 10, 8)
        cf.setSpacing(6)
        cf.addRow(W.QLabel("<b>🎛 PID 环路与默认初始参数</b>"))

        self.loop_box = W.QComboBox()
        self.loop_box.addItems(list(LOOPS))
        self.loop_box.setMinimumWidth(0)
        self.loop_box.setSizePolicy(W.QSizePolicy.Policy.Expanding, W.QSizePolicy.Policy.Fixed)
        self.loop_box.setCurrentText("SPD")
        self.ctrl_ms = W.QSpinBox()
        self.ctrl_ms.setRange(1, 200)
        self.ctrl_ms.setValue(10)
        self.ctrl_ms.setMinimumWidth(0)
        self.rep_ms = W.QSpinBox()
        self.rep_ms.setRange(1, 500)
        self.rep_ms.setValue(20)
        self.rep_ms.setMinimumWidth(0)
        self.pwm_lim = W.QDoubleSpinBox()
        self.pwm_lim.setRange(1.0, 100000.0)
        self.pwm_lim.setDecimals(1)
        self.pwm_lim.setValue(1000.0)
        self.pwm_lim.setMinimumWidth(0)

        self.def_kp = W.QDoubleSpinBox()
        self.def_ki = W.QDoubleSpinBox()
        self.def_kd = W.QDoubleSpinBox()
        self.def_tgt = W.QDoubleSpinBox()
        for sp, val in ((self.def_kp, 5.0), (self.def_ki, 30.0), (self.def_kd, 0.05), (self.def_tgt, 100.0)):
            sp.setRange(-100000.0, 100000.0)
            sp.setDecimals(4)
            sp.setValue(val)
            sp.setMinimumWidth(0)

        self.emit_roles_chk = W.QCheckBox("上报 #ROLES: 角色声明（支持任意列顺序）")
        self.emit_roles_chk.setChecked(True)

        cf.addRow("默认调参环路", self.loop_box)
        cf.addRow("控制 / 上报周期 ms", self._pair_row(self.ctrl_ms, self.rep_ms))
        cf.addRow("PWM 输出限幅", self.pwm_lim)
        cf.addRow("初始 KP / KI", self._pair_row(self.def_kp, self.def_ki))
        cf.addRow("初始 KD / TGT", self._pair_row(self.def_kd, self.def_tgt))
        cf.addRow(self.emit_roles_chk)
        left_layout.addWidget(self.pid_cfg_card)

        self.ch_card = W.QFrame()
        self.ch_card.setObjectName("card")
        cv = W.QVBoxLayout(self.ch_card)
        cv.setContentsMargins(10, 8, 10, 8)
        cv.setSpacing(6)
        cv.addWidget(W.QLabel("<b>📡 串口上报通道定义（1～8 通道，可任意调序）</b>"))

        btn_row = W.QHBoxLayout()
        self.add_ch_btn = W.QPushButton("+ 添加")
        self.del_ch_btn = W.QPushButton("- 删除")
        self.up_ch_btn = W.QPushButton("↑ 上移")
        self.dn_ch_btn = W.QPushButton("↓ 下移")
        for b in (self.add_ch_btn, self.del_ch_btn, self.up_ch_btn, self.dn_ch_btn):
            b.setMinimumWidth(0)
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

        hint = W.QLabel("提示：无论怎么调换 CH1～CH8 的位置，只要角色选对并勾选 #ROLES:，上位机自动识别反馈与 PWM。")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#5b7083;font-size:9pt;")
        cv.addWidget(hint)
        left_layout.addWidget(self.ch_card)

        # ---------- IMU Cards (iface == 1) ----------
        self.imu_preset_card = W.QFrame()
        self.imu_preset_card.setObjectName("card")
        ipv = W.QVBoxLayout(self.imu_preset_card)
        ipv.setContentsMargins(10, 8, 10, 8)
        ipv.setSpacing(6)
        ipv.addWidget(W.QLabel("<b>⚡ IMU / MPU6050 预设模板</b>"))
        for idx, (label, _) in enumerate(IMU_PRESETS):
            btn = W.QPushButton(label)
            btn.setMinimumWidth(0)
            btn.clicked.connect(lambda _=False, i=idx: self.apply_imu_preset(i))
            ipv.addWidget(btn)
        left_layout.addWidget(self.imu_preset_card)

        self.imu_cfg_card = W.QFrame()
        self.imu_cfg_card.setObjectName("card")
        icf = W.QFormLayout(self.imu_cfg_card)
        icf.setFieldGrowthPolicy(W.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        icf.setContentsMargins(10, 8, 10, 8)
        icf.setSpacing(6)
        icf.addRow(W.QLabel("<b>🧭 IMU 数据协议与上报参数</b>"))
        self.imu_format_box = W.QComboBox()
        self.imu_format_box.addItems(list(IMU_FORMATS))
        self.imu_format_box.setMinimumWidth(0)
        self.imu_format_box.setMinimumContentsLength(10)
        self.imu_format_box.setSizePolicy(W.QSizePolicy.Policy.Expanding, W.QSizePolicy.Policy.Fixed)
        self.imu_format_box.setSizeAdjustPolicy(W.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.imu_rep_ms = W.QSpinBox()
        self.imu_rep_ms.setRange(5, 500)
        self.imu_rep_ms.setValue(20)
        self.imu_rep_ms.setSuffix(" ms")
        self.imu_banners_chk = W.QCheckBox("上电发送 #BALANCE:CALIBRATING / READY 状态横幅")
        self.imu_banners_chk.setChecked(True)
        icf.addRow("输出格式", self.imu_format_box)
        icf.addRow("上报周期", self.imu_rep_ms)
        icf.addRow(self.imu_banners_chk)
        left_layout.addWidget(self.imu_cfg_card)

        # ---------- Remote Cards (iface == 2) ----------
        self.remote_preset_card = W.QFrame()
        self.remote_preset_card.setObjectName("card")
        rpv = W.QVBoxLayout(self.remote_preset_card)
        rpv.setContentsMargins(10, 8, 10, 8)
        rpv.setSpacing(6)
        rpv.addWidget(W.QLabel("<b>⚡ 差速小车遥控预设模板</b>"))
        for idx, (label, _) in enumerate(REMOTE_PRESETS):
            btn = W.QPushButton(label)
            btn.setMinimumWidth(0)
            btn.clicked.connect(lambda _=False, i=idx: self.apply_remote_preset(i))
            rpv.addWidget(btn)
        left_layout.addWidget(self.remote_preset_card)

        self.remote_cfg_card = W.QFrame()
        self.remote_cfg_card.setObjectName("card")
        rcf = W.QFormLayout(self.remote_cfg_card)
        rcf.setFieldGrowthPolicy(W.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        rcf.setContentsMargins(10, 8, 10, 8)
        rcf.setSpacing(6)
        rcf.addRow(W.QLabel("<b>🎮 混控参数与失联保护</b>"))
        self.remote_fmt_edit = W.QLineEdit("J,{x},{y}")
        self.remote_fmt_edit.setMinimumWidth(0)
        self.remote_pwm_lim = W.QSpinBox()
        self.remote_pwm_lim.setRange(50, 50000)
        self.remote_pwm_lim.setValue(1000)
        self.remote_failsafe_ms = W.QSpinBox()
        self.remote_failsafe_ms.setRange(100, 5000)
        self.remote_failsafe_ms.setValue(500)
        self.remote_failsafe_ms.setSuffix(" ms")
        self.remote_aux_chk = W.QCheckBox("生成 LED_ON / LED_OFF / BEEP / SERVO 辅助解析")
        self.remote_aux_chk.setChecked(True)
        rcf.addRow("数据包模板", self.remote_fmt_edit)
        rcf.addRow("PWM 满量程 ±", self.remote_pwm_lim)
        rcf.addRow("失联自动停车", self.remote_failsafe_ms)
        rcf.addRow(self.remote_aux_chk)
        left_layout.addWidget(self.remote_cfg_card)

        left_layout.addStretch(1)

        # ---------- Right output code + protocol docs ----------
        right_box = W.QWidget()
        right_box.setMinimumWidth(340)
        right_layout = W.QVBoxLayout(right_box)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)
        self.splitter.addWidget(right_box)
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([480, 760])

        # Top action bar
        top_bar = W.QFrame()
        top_bar.setObjectName("top")
        tb = W.QHBoxLayout(top_bar)
        tb.setContentsMargins(10, 6, 10, 6)
        self.proto_badge = W.QLabel()
        self.proto_badge.setMinimumWidth(0)
        self.proto_badge.setSizePolicy(W.QSizePolicy.Policy.Ignored, W.QSizePolicy.Policy.Preferred)
        self.proto_badge.setStyleSheet("font-weight:600;color:#176a57;")
        tb.addWidget(self.proto_badge, 1)
        self.copy_file_btn = W.QPushButton("📋 复制当前文件")
        self.export_dir_btn = W.QPushButton("📦 一键导出全部文件…")
        self.export_dir_btn.setObjectName("accent")
        self.goto_pid_btn = W.QPushButton("📈 去 PID 页联调")
        self.goto_pid_btn.setObjectName("primary")
        for b in (self.copy_file_btn, self.export_dir_btn, self.goto_pid_btn):
            b.setMinimumWidth(0)
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

        # Tab 2: Dynamic per-interface usage & protocol guide
        doc_tab = W.QWidget()
        dt = W.QVBoxLayout(doc_tab)
        dt.setContentsMargins(6, 6, 6, 6)
        self.guide_view = W.QTextBrowser()
        self.guide_view.setHtml(PROTOCOL_GUIDE_HTML)
        dt.addWidget(self.guide_view, 1)
        self.tabs.addTab(doc_tab, "📖 协议与使用配置说明")

        # Connections
        self.platform_box.currentIndexChanged.connect(self._on_platform_changed)
        for sig in (
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
            self.imu_format_box.currentIndexChanged,
            self.imu_rep_ms.valueChanged,
            self.imu_banners_chk.toggled,
            self.remote_fmt_edit.textChanged,
            self.remote_pwm_lim.valueChanged,
            self.remote_failsafe_ms.valueChanged,
            self.remote_aux_chk.toggled,
        ):
            sig.connect(self.regenerate)
        self.file_combo.currentIndexChanged.connect(self._show_selected_file)
        self.copy_file_btn.clicked.connect(self.copy_current_file)
        self.export_dir_btn.clicked.connect(self.choose_and_export)
        self.goto_pid_btn.clicked.connect(self._jump_to_target_workbench)
        self.add_ch_btn.clicked.connect(self.add_channel)
        self.del_ch_btn.clicked.connect(self.remove_channel)
        self.up_ch_btn.clicked.connect(lambda: self.move_channel(-1))
        self.dn_ch_btn.clicked.connect(lambda: self.move_channel(1))
        self.row_selector.valueChanged.connect(lambda v: setattr(self, "_selected_row", v - 1))
        for cls in (W.QAbstractSpinBox, W.QComboBox):
            for ctrl in left_box.findChildren(cls):
                ctrl.setFocusPolicy(C.Qt.FocusPolicy.StrongFocus)

    @staticmethod
    def _pair_row(w1: W.QWidget, w2: W.QWidget) -> W.QWidget:
        box = W.QWidget()
        row = W.QHBoxLayout(box)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.addWidget(w1, 1)
        row.addWidget(w2, 1)
        return box

    def interface_type(self) -> int:
        return self._iface_type

    def configure_from_workbench(
        self,
        iface_type: int,
        *,
        imu_format: int | None = None,
        remote_cmd_fmt: str | None = None,
        baudrate: str | None = None,
    ):
        """Sync relevant context from another workbench and switch to the matching builder view."""
        self._suppress_regen = True
        try:
            if baudrate and str(baudrate).strip():
                self.baud_box.setCurrentText(str(baudrate).strip())
            if imu_format is not None and 0 <= imu_format < self.imu_format_box.count():
                self.imu_format_box.setCurrentIndex(int(imu_format))
            if remote_cmd_fmt and str(remote_cmd_fmt).strip():
                self.remote_fmt_edit.setText(str(remote_cmd_fmt).strip())
        finally:
            self._suppress_regen = False
        self.set_interface_type(iface_type)

    def set_interface_type(self, index: int):
        idx = index if index in (0, 1, 2) else 0
        self._iface_type = idx
        for i, btn in enumerate(self.iface_buttons):
            with C.QSignalBlocker(btn):
                btn.setChecked(i == idx)
        for w in (self.pid_preset_card, self.pid_cfg_card, self.ch_card):
            w.setVisible(idx == 0)
        for w in (self.imu_preset_card, self.imu_cfg_card):
            w.setVisible(idx == 1)
        for w in (self.remote_preset_card, self.remote_cfg_card):
            w.setVisible(idx == 2)
        target_labels = ("📈 去 PID 页联调", "🧭 去 IMU 页联调", "🎮 去遥控页联调")
        self.goto_pid_btn.setText(target_labels[idx])
        self.regenerate()

    def _jump_to_target_workbench(self):
        if self.host is None or not hasattr(self.host, "switch_module"):
            return
        target_tab = {0: 0, 1: 2, 2: 3}.get(self._iface_type, 0)
        self.host.switch_module(target_tab)

    def _on_platform_changed(self, new_plat: int):
        plat = new_plat if new_plat in (0, 1, 2) else 0
        self.uart_label.setText(PLATFORM_UART_LABELS[plat])
        cur = self.uart_edit.text().strip()
        if not cur or cur in PLATFORM_DEFAULT_HANDLES:
            with C.QSignalBlocker(self.uart_edit):
                self.uart_edit.setText(PLATFORM_DEFAULT_HANDLES[plat])
        self._last_platform = plat
        self.regenerate()

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
                ed.setMinimumWidth(0)
                cb = W.QComboBox()
                cb.setFocusPolicy(C.Qt.FocusPolicy.StrongFocus)
                cb.setMinimumWidth(0)
                cb.setMinimumContentsLength(8)
                cb.setSizePolicy(W.QSizePolicy.Policy.Expanding, W.QSizePolicy.Policy.Fixed)
                cb.setSizeAdjustPolicy(W.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
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

    def apply_imu_preset(self, index: int):
        _, preset = IMU_PRESETS[max(0, min(len(IMU_PRESETS) - 1, index))]
        self._suppress_regen = True
        try:
            self.imu_format_box.setCurrentIndex(preset["imu_format"])
            self.imu_rep_ms.setValue(preset["imu_report_ms"])
            self.imu_banners_chk.setChecked(preset["imu_emit_banners"])
            self.baud_box.setCurrentText(str(preset["baudrate"]))
        finally:
            self._suppress_regen = False
        self.regenerate()

    def apply_remote_preset(self, index: int):
        _, preset = REMOTE_PRESETS[max(0, min(len(REMOTE_PRESETS) - 1, index))]
        self._suppress_regen = True
        try:
            self.remote_fmt_edit.setText(preset["remote_cmd_fmt"])
            self.remote_pwm_lim.setValue(preset["remote_pwm_limit"])
            self.remote_failsafe_ms.setValue(preset["remote_failsafe_ms"])
            self.remote_aux_chk.setChecked(preset["remote_aux_cmds"])
            self.baud_box.setCurrentText(str(preset["baudrate"]))
        finally:
            self._suppress_regen = False
        self.regenerate()

    def config_dict(self) -> dict:
        try:
            baud = int(self.baud_box.currentText().strip())
        except ValueError:
            baud = 115200
        return {
            "interface_type": self._iface_type,
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
            "imu_format": self.imu_format_box.currentIndex(),
            "imu_report_ms": self.imu_rep_ms.value(),
            "imu_emit_banners": self.imu_banners_chk.isChecked(),
            "remote_cmd_fmt": self.remote_fmt_edit.text(),
            "remote_pwm_limit": self.remote_pwm_lim.value(),
            "remote_failsafe_ms": self.remote_failsafe_ms.value(),
            "remote_aux_cmds": self.remote_aux_chk.isChecked(),
        }

    def current_bundle(self) -> dict[str, str]:
        return dict(self._bundle)

    def regenerate(self):
        if self._suppress_regen:
            return
        cfg = self.config_dict()
        iface = cfg["interface_type"]
        plat = cfg["platform"]
        if iface == 0:
            names_hdr, roles_hdr = build_protocol_headers(cfg["channels"], emit_roles=cfg["emit_roles"])
            badge_txt = f"{names_hdr}" + (f"   |   {roles_hdr}" if roles_hdr else "")
        elif iface == 1:
            names_hdr = "#NAMES:" + ",".join(IMU_FORMAT_NAMES[cfg["imu_format"]])
            badge_txt = f"IMU 格式: {names_hdr}  ·  {cfg['imu_report_ms']} ms"
        else:
            badge_txt = f"遥控协议: {cfg['remote_cmd_fmt']}  ·  ±{cfg['remote_pwm_limit']}  ·  失联 {cfg['remote_failsafe_ms']} ms"
        self.proto_badge.setText(badge_txt)
        self.proto_badge.setToolTip(badge_txt)
        self._bundle = generate_interface_bundle(cfg)
        self.guide_view.setHtml(build_interface_guide_html(iface, plat))

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
        settings.setValue("firmware_studio/config", json.dumps(self.config_dict(), ensure_ascii=False))
        settings.setValue("firmware_studio/splitter", self.splitter.saveState())

    def restore_state(self, settings: C.QSettings):
        split_state = settings.value("firmware_studio/splitter")
        if split_state is not None:
            self.splitter.restoreState(split_state)
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
                self.uart_label.setText(PLATFORM_UART_LABELS[cfg["platform"]])
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
            if isinstance(cfg.get("imu_format"), int) and 0 <= cfg["imu_format"] < self.imu_format_box.count():
                self.imu_format_box.setCurrentIndex(cfg["imu_format"])
            if isinstance(cfg.get("imu_report_ms"), int):
                self.imu_rep_ms.setValue(cfg["imu_report_ms"])
            if "imu_emit_banners" in cfg:
                self.imu_banners_chk.setChecked(bool(cfg["imu_emit_banners"]))
            if cfg.get("remote_cmd_fmt"):
                self.remote_fmt_edit.setText(str(cfg["remote_cmd_fmt"]))
            if isinstance(cfg.get("remote_pwm_limit"), (int, float)):
                self.remote_pwm_lim.setValue(int(cfg["remote_pwm_limit"]))
            if isinstance(cfg.get("remote_failsafe_ms"), int):
                self.remote_failsafe_ms.setValue(cfg["remote_failsafe_ms"])
            if "remote_aux_cmds" in cfg:
                self.remote_aux_chk.setChecked(bool(cfg["remote_aux_cmds"]))
            iface = cfg.get("interface_type", 0)
            if isinstance(iface, int) and iface in (0, 1, 2):
                self._iface_type = iface
        finally:
            self._suppress_regen = False
        if isinstance(cfg.get("channels"), list) and cfg["channels"]:
            self._set_channels([(str(c[0]), str(c[1])) for c in cfg["channels"] if isinstance(c, (list, tuple)) and len(c) >= 2])
        self.set_interface_type(self._iface_type)

    def shutdown(self):
        return None
