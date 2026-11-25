"""Build EmbeddedDebugStudio-v1.6-Manual.docx (and HTML for headless Edge PDF export) from docs/images."""
from pathlib import Path
import shutil
import sys

HERE = Path(__file__).resolve().parent
IMG_DIR = HERE / "images"
OUT_DIR = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else HERE
OUT_DIR.mkdir(parents=True, exist_ok=True)


def main():
    import docx
    from docx.shared import Inches, Pt, RGBColor
    from docx.enum.text import WD_ALIGN_PARAGRAPH
    from docx.enum.table import WD_TABLE_ALIGNMENT
    from docx.oxml import OxmlElement, parse_xml
    from docx.oxml.ns import nsdecls, qn

    def set_font(run_or_style, name="Microsoft YaHei UI"):
        rPr = run_or_style._element.get_or_add_rPr()
        rFonts = rPr.find(qn("w:rFonts"))
        if rFonts is None:
            rFonts = OxmlElement("w:rFonts")
            rPr.append(rFonts)
        for attr in ("w:ascii", "w:hAnsi", "w:eastAsia", "w:cs"):
            rFonts.set(qn(attr), name)

    doc = docx.Document()
    set_font(doc.styles["Normal"])
    doc.styles["Normal"].font.size = Pt(10.5)

    p = doc.add_paragraph()
    r = p.add_run("嵌入式综合调试助手 v1.6 · 图文使用手册（PID / 串口 / IMU / 蓝牙遥控）")
    r.bold = True
    r.font.size = Pt(19)
    r.font.color.rgb = RGBColor(0x18, 0x3B, 0x6B)
    set_font(r)

    sections = [
        ("1. 软件全景与四大模块导航（Ctrl+1~4）",
         "双击运行 PIDTuner.exe 即可免安装启动。顶栏支持一键切换【📈 PID 调参】、【🔌 串口调试】、【🧭 IMU 姿态】、【🎮 遥控与 HC-05】四大工作台，支持同串口跨页自动让渡与全局 Esc 急停。",
         "fig01_overview.png"),
        ("2. 串口连接、通道缩放与独立双 Y 轴",
         "支持将下位机整数放大数据按比例还原为工程物理量（如 RPM），并将转速与 PWM 分配至左右独立坐标轴同屏观察。",
         "fig02_serial_and_channels.png"),
        ("3. 在线调参、三色确认状态与参数保护",
         "绿框代表单片机已确认(@ACK)，琥珀色为未发送草稿，红框为超限或超时告警；支持上下限保护与一键撤销。",
         "fig03_pid_panel_states.png"),
        ("4. 智能 PID 自动整定（Auto-Tune）向导与前后对比",
         "无需改动下位机固件，设定目标值后自动执行闭环比例探针模型辨识（SIMC/极点配置）与 Hooke–Jeeves 多轮闭环寻优。",
         "fig04_autotune_dialog.png"),
        ("5. 自动整定全过程波形实录",
         "软件自动记录每轮评分 J 并在完成后写回最优 KP/KI/KD，不满意可一键撤销恢复原参数。",
         "fig05_autotune_compare.png"),
        ("6. 目标值信号发生器与平衡车倒立摆仿真",
         "内置单次阶跃/方波/正弦/三角波发生器，以及带 45° 倒地自动停机保护(#STOP:fall)的平衡车倒立摆物理模型。",
         "fig06_generator_and_pendulum.png"),
        ("7. 单次边沿触发捕获、双游标与暗色护眼主题",
         "支持上升沿/下降沿自动冻结抓拍突加负载瞬态，放置 A/B 双游标精确测量 Δt 与 ΔY，并支持夜间暗色主题。",
         "fig07_capture_and_cursors.png"),
        ("8. 同起点 Ghost 残影对比、滑块微调、FFT 体检与 C 代码生成",
         "一键锁定旧参数紫色虚线底图同起点叠加对比；一键体检零偏、底噪 σ 与 FFT 主频，并生成 STM32 C 语言宏定义代码。",
         "fig09_ghost_and_sliders.png"),
        ("9. 独立全功能【🔌 串口调试】工作台（Ctrl+2）",
         "支持 UTF-8/GBK/Latin-1、ASCII/HEX、DTR/RTS、空闲超时断帧、关键字实时过滤、Modbus CRC16 校验、定时发送与 12 路 JSON 快捷指令集。",
         "fig11_serial_studio.png"),
        ("10. 3D 可视化【🧭 IMU 姿态】与机械中值角标定（Ctrl+3）",
         "零 OpenGL 渲染可旋转 3D 平衡车与航空人工地平仪(±30°保护线)，内置六轴实时互补滤波与 50 帧机械中值角一键标定同步 PID。",
         "fig12_imu_studio.png"),
        ("11. 【🎮 遥控与 HC-05】工作台：联动 BlueSerialStudio（Ctrl+4）",
         "完整兼容 BlueSerialStudio 的 J,x,y 三模式差速遥控（360° 单摇杆/双摇杆/WASD 长按加速）与支持 USB-TTL 直发收看 OK 的 HC-05 AT 配置工具箱。",
         "fig13_remote_hc05.png"),
    ]

    html_parts = [
        "<!doctype html><html><head><meta charset='utf-8'>",
        "<style>body{font-family:'Microsoft YaHei UI',sans-serif;color:#223449;max-width:920px;margin:24px auto;padding:0 20px;line-height:1.6}"
        "h1{color:#183b6b;border-bottom:2px solid #225fc1;padding-bottom:8px}h2{color:#197a63;margin-top:26px}"
        "img{max-width:100%;border:1px solid #cfd8e2;border-radius:6px;margin:8px 0 18px 0}</style></head><body>",
        "<h1>嵌入式综合调试助手 v1.6 · 图文使用手册</h1>",
    ]

    for heading, body, img_name in sections:
        h = doc.add_heading(level=1)
        rh = h.add_run(heading)
        rh.font.size = Pt(14)
        rh.bold = True
        rh.font.color.rgb = RGBColor(0x1B, 0x4F, 0x9C)
        set_font(rh)

        pb = doc.add_paragraph()
        rb = pb.add_run(body)
        rb.font.size = Pt(10.5)
        set_font(rb)

        img_file = IMG_DIR / img_name
        if img_file.exists():
            pi = doc.add_paragraph()
            pi.alignment = WD_ALIGN_PARAGRAPH.CENTER
            pi.add_run().add_picture(str(img_file), width=Inches(6.2))
            html_parts.append(f"<h2>{heading}</h2><p>{body}</p><img src='{img_file.as_uri()}'>")

    html_parts.append("</body></html>")
    out_docx = OUT_DIR / "EmbeddedDebugStudio-v1.6-Manual.docx"
    out_html = OUT_DIR / "_manual_print.html"
    doc.save(str(out_docx))
    out_html.write_text("\n".join(html_parts), encoding="utf-8")
    print("Built DOCX:", out_docx)


if __name__ == "__main__":
    main()
