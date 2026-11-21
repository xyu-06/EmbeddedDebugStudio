/* Add this file and pid_link.c to an F1/F4 Cube HAL application.
 * The plant is numerical. Replace Board_ApplyOutput and feedback together
 * when integrating your actual motor driver and its board-specific limits. */
#include <math.h>
#include <string.h>
#include "hal_demo.h"
#include "pid_link.h"

static UART_HandleTypeDef *uart;
static uint8_t rx_byte;
static volatile uint8_t fifo[256], rd, wr, overflow, rearm;
static uint32_t control_ms, report_ms;
static float plant, applied;

/* Foreground-owned queues. HAL owns in_flight until its TX state is READY.
 * Replies retain order; when bandwidth is exhausted keep the newest sample. */
typedef struct {
    uint16_t length;
    uint8_t data[192];
} TxFrame;
static TxFrame replies[8], telemetry, in_flight;
static uint8_t read_reply, reply_count, tx_overflow;

static void Board_ApplyOutput(float value)
{
    applied = value;
}

void PID_ZeroOutputs(void)
{
    Board_ApplyOutput(0);
}

void PID_Write(const char *data, unsigned len)
{
    if (!len || len > sizeof in_flight.data) return;
    TxFrame *frame = &telemetry;
    if (data[0] == '@' || data[0] == '#') {
        if (reply_count == 8) { tx_overflow = 1; return; }
        frame = &replies[(read_reply + reply_count) % 8];
        ++reply_count;
    }
    memcpy(frame->data, data, len);
    frame->length = (uint16_t)len;
}

static void service_tx(void)
{
    if (uart->gState != HAL_UART_STATE_READY) return;
    if (tx_overflow && reply_count < 8) {
        static const char warning[] = "#WARN:reply queue overflow\n";
        tx_overflow = 0;
        PID_Write(warning, sizeof warning - 1);
    }
    TxFrame *next = reply_count ? &replies[read_reply] : &telemetry;
    if (!next->length) return;
    in_flight = *next;
    if (HAL_UART_Transmit_IT(uart, in_flight.data, in_flight.length) == HAL_OK) {
        next->length = 0;
        if (reply_count) { read_reply = (read_reply + 1) % 8; --reply_count; }
    }
}

void PID_DemoInit(UART_HandleTypeDef *handle)
{
    uart = handle;
    control_ms = report_ms = HAL_GetTick();
    pid_loops[PID_ANG].kp = 2;
    pid_loops[PID_ANG].ki = 1.2f;
    pid_loops[PID_ANG].kd = .15f;
    pid_loops[PID_ANG].target = 10;
    PID_ZeroOutputs();
    PID_WatchdogTick(control_ms);
    PID_CheckFloatFormat();
    rearm = HAL_UART_Receive_IT(uart, &rx_byte, 1) != HAL_OK;
    static const char names[] = "#NAMES:target,actual,output,error\n";
    PID_Write(names, sizeof names - 1);
}

/* Forward your existing HAL callbacks here. */
void PID_DemoRxDone(UART_HandleTypeDef *handle)
{
    if (handle != uart) return;
    uint8_t next = (uint8_t)(wr + 1);
    if (next == rd) overflow = 1;
    else { fifo[wr] = rx_byte; wr = next; }
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1) != HAL_OK) rearm = 1;
}

void PID_DemoRxError(UART_HandleTypeDef *handle)
{
    if (handle == uart) { overflow = 1; rearm = 1; }
}

void PID_DemoPoll(void)
{
    PID_WatchdogTick(HAL_GetTick());
    if (rearm) {
        HAL_UART_AbortReceive(uart);
        rearm = HAL_UART_Receive_IT(uart, &rx_byte, 1) != HAL_OK;
    }
    for (unsigned drained = 0; drained < sizeof fifo; ++drained) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        uint8_t lost = overflow;
        if (lost) { rd = wr; overflow = 0; }
        uint8_t available = rd != wr, b = 0;
        if (available) { b = fifo[rd]; rd = (uint8_t)(rd + 1); }
        __set_PRIMASK(mask);
        if (lost) PID_DiscardPartial();
        if (!available) break;
        PID_Consume(b);
    }
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - control_ms;
    if (elapsed >= 5) {
        float dt = (float)elapsed * .001f;
        control_ms = now;
        Board_ApplyOutput(PID_Position(&pid_loops[PID_ANG], plant, dt, 100));
        plant += (applied - plant) * fminf(dt / .55f, 1.0f);
    }
    if (now - report_ms >= 10) {
        report_ms = now;
        PID_Report(pid_running ? pid_loops[PID_ANG].target : 0, plant, applied);
    }
    service_tx();
}

