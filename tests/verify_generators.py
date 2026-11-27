"""Generate and C99-compile all 30 firmware interface configurations (HAL / SPL / Arduino) plus SPL UART1~5 coverage."""
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from firmware_studio import SPL_SUPPORTED_USARTS, generate_interface_bundle  # noqa: E402

HAL_MAIN_H = """#ifndef PID_VERIFY_HAL_H
#define PID_VERIFY_HAL_H
#include <stdint.h>
typedef struct { unsigned gState; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart2, huart3;
enum { HAL_OK = 0, HAL_BUSY = 1, HAL_UART_STATE_READY = 2, HAL_UART_STATE_BUSY_TX = 3 };
static inline uint32_t HAL_GetTick(void) { return 0U; }
static inline int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *b, uint16_t n) { (void)h; (void)b; (void)n; return HAL_OK; }
static inline int HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *b, uint16_t n) { (void)h; (void)b; (void)n; return HAL_OK; }
static inline int HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
"""

SPL_STM32_H = """#ifndef STM32F10X_H
#define STM32F10X_H
#include <stdint.h>
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;
typedef enum { RESET = 0, SET = !RESET } ITStatus;
typedef struct { uint32_t SR; uint32_t DR; } USART_TypeDef;
typedef struct { uint32_t CRL; } GPIO_TypeDef;
typedef struct { uint32_t USART_BaudRate; } USART_InitTypeDef;
typedef enum {
    USART1_IRQn = 37,
    USART2_IRQn = 38,
    USART3_IRQn = 39,
    UART4_IRQn  = 52,
    UART5_IRQn  = 53
} IRQn_Type;
extern USART_TypeDef usart1_inst, usart2_inst, usart3_inst, uart4_inst, uart5_inst;
extern GPIO_TypeDef gpioa_inst, gpiob_inst, gpioc_inst, gpiod_inst;
static uint32_t SystemCoreClock = 72000000U;
#define USART1 (&usart1_inst)
#define USART2 (&usart2_inst)
#define USART3 (&usart3_inst)
#define UART4  (&uart4_inst)
#define UART5  (&uart5_inst)
#define GPIOA  (&gpioa_inst)
#define GPIOB  (&gpiob_inst)
#define GPIOC  (&gpioc_inst)
#define GPIOD  (&gpiod_inst)
#define GPIO_Pin_2  ((uint16_t)0x0004)
#define GPIO_Pin_3  ((uint16_t)0x0008)
#define GPIO_Pin_9  ((uint16_t)0x0200)
#define GPIO_Pin_10 ((uint16_t)0x0400)
#define GPIO_Pin_11 ((uint16_t)0x0800)
#define GPIO_Pin_12 ((uint16_t)0x1000)
#define GPIO_Speed_50MHz 3
#define GPIO_Mode_AF_PP 0x18
#define GPIO_Mode_IN_FLOATING 0x04
typedef struct { uint16_t GPIO_Pin; uint8_t GPIO_Speed; uint8_t GPIO_Mode; } GPIO_InitTypeDef;
#define RCC_APB2Periph_USART1 1U
#define RCC_APB2Periph_GPIOA  2U
#define RCC_APB2Periph_GPIOB  4U
#define RCC_APB2Periph_GPIOC  8U
#define RCC_APB2Periph_GPIOD  16U
#define RCC_APB1Periph_USART2 1U
#define RCC_APB1Periph_USART3 2U
#define RCC_APB1Periph_UART4  4U
#define RCC_APB1Periph_UART5  8U
#define USART_IT_RXNE 1U
#define USART_IT_TXE  2U
static inline void RCC_APB2PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
static inline void RCC_APB1PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
static inline void GPIO_Init(GPIO_TypeDef *g, GPIO_InitTypeDef *i) { (void)g; (void)i; }
static inline void USART_StructInit(USART_InitTypeDef *u) { u->USART_BaudRate = 9600U; }
static inline void USART_Init(USART_TypeDef *u, USART_InitTypeDef *i) { (void)u; (void)i; }
static inline void USART_Cmd(USART_TypeDef *u, FunctionalState s) { (void)u; (void)s; }
static inline void NVIC_EnableIRQ(IRQn_Type irqn) { (void)irqn; }
static inline void SysTick_Config(uint32_t ticks) { (void)ticks; }
static inline ITStatus USART_GetITStatus(USART_TypeDef *u, uint16_t it) { (void)u; (void)it; return RESET; }
static inline uint16_t USART_ReceiveData(USART_TypeDef *u) { (void)u; return 0; }
static inline void USART_SendData(USART_TypeDef *u, uint16_t d) { (void)u; (void)d; }
static inline void USART_ITConfig(USART_TypeDef *u, uint16_t it, FunctionalState s) { (void)u; (void)it; (void)s; }
static inline void USART_ClearITPendingBit(USART_TypeDef *u, uint16_t it) { (void)u; (void)it; }
static inline uint32_t __get_PRIMASK(void) { (void)SystemCoreClock; return 0U; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
"""


def compile_c99_dir(src_dir: Path, inc_dir: Path) -> int:
    c_files = sorted(src_dir.glob("*.c"))
    for c_file in c_files:
        cmd = [
            sys.executable, "-m", "ziglang", "cc",
            "-std=c99", "-Wall", "-Wextra", "-Werror", "-c",
            f"-I{src_dir}", f"-I{inc_dir}",
            str(c_file), "-o", str(src_dir / f"{c_file.stem}.obj"),
        ]
        subprocess.run(cmd, cwd=str(src_dir), check=True)
    return len(c_files)


def main() -> int:
    configs: list[tuple[str, dict]] = []
    for plat in (0, 1, 2):
        configs.append((f"pid_p{plat}", {"interface_type": 0, "platform": plat}))
    for fmt in (0, 1, 2, 3):
        for inc_ts in (False, True):
            for plat in (0, 1, 2):
                configs.append((
                    f"imu_f{fmt}_ts{int(inc_ts)}_p{plat}",
                    {"interface_type": 1, "platform": plat, "imu_format": fmt, "imu_include_ts": inc_ts},
                ))
    for plat in (0, 1, 2):
        configs.append((f"remote_p{plat}", {"interface_type": 2, "platform": plat}))

    assert len(configs) == 30, f"Expected 30 configs, got {len(configs)}"

    # Unsupported SPL USARTs (e.g. USART6) must fail loudly instead of silently generating USART2 init
    for iface in (0, 1, 2):
        try:
            generate_interface_bundle({"interface_type": iface, "platform": 1, "uart_handle": "USART6"})
            raise AssertionError(f"Expected ValueError for unsupported SPL USART6 (iface={iface})")
        except ValueError:
            pass

    total_c_compiled = 0
    total_ino_checked = 0
    with TemporaryDirectory(prefix="eds_gen30_") as tmp:
        tmp_path = Path(tmp)
        hal_inc = tmp_path / "hal_inc"
        spl_inc = tmp_path / "spl_inc"
        hal_inc.mkdir()
        spl_inc.mkdir()
        (hal_inc / "main.h").write_text(HAL_MAIN_H, encoding="utf-8")
        (spl_inc / "stm32f10x.h").write_text(SPL_STM32_H, encoding="utf-8")

        for tag, cfg in configs:
            bundle = generate_interface_bundle(cfg)
            out_dir = tmp_path / tag
            out_dir.mkdir()
            for fname, content in bundle.items():
                (out_dir / fname).write_text(content, encoding="utf-8")
            plat = cfg["platform"]
            if plat == 0:
                total_c_compiled += compile_c99_dir(out_dir, hal_inc)
            elif plat == 1:
                total_c_compiled += compile_c99_dir(out_dir, spl_inc)
            else:
                ino_files = [k for k in bundle if k.endswith(".ino")]
                assert len(ino_files) == 1
                code = bundle[ino_files[0]]
                if cfg["interface_type"] == 0:
                    assert "#define PID_SENSOR_HOOK_READY 0" in code
                    assert "readSensorFeedback()" in code
                    assert "parseStrictFloat" in code
                elif cfg["interface_type"] == 1:
                    assert "#NAMES:" in code
                else:
                    assert "mixWheels" in code
                total_ino_checked += 1

        # Verify all 5 supported SPL UARTs (including UART4 & UART5 pins/RCC/IRQn) compile cleanly
        for uart in SPL_SUPPORTED_USARTS:
            for iface in (0, 1, 2):
                bundle = generate_interface_bundle({"interface_type": iface, "platform": 1, "uart_handle": uart})
                out_dir = tmp_path / f"spl_{uart}_i{iface}"
                out_dir.mkdir()
                for fname, content in bundle.items():
                    (out_dir / fname).write_text(content, encoding="utf-8")
                total_c_compiled += compile_c99_dir(out_dir, spl_inc)

    print(
        f"GENERATOR VERIFICATION OK: configs={len(configs)}, "
        f"c99_files={total_c_compiled}, ino_checked={total_ino_checked}, "
        f"spl_uarts={','.join(SPL_SUPPORTED_USARTS)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
