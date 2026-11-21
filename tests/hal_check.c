#include "hal_demo.h"
#include "pid_link.h"
#include <assert.h>
#include <string.h>

static uint32_t tick;
static uint8_t *rx, *transmitting;
static uint16_t tx_len;
static UART_HandleTypeDef port = { HAL_UART_STATE_READY };
uint32_t HAL_GetTick(void) { return tick; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *p,uint8_t *b,uint16_t n)
{ assert(p == &port && n == 1); rx = b; return HAL_OK; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *p) { (void)p; return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *p,uint8_t *b,uint16_t n)
{
    assert(p->gState == HAL_UART_STATE_READY);
    p->gState = HAL_UART_STATE_BUSY_TX; transmitting = b; tx_len = n;
    return HAL_OK;
}
static void receive(const char *s)
{
    while (*s) { *rx = (uint8_t)*s++; PID_DemoRxDone(&port); }
    PID_DemoPoll();
}
static void expect_frame(const char *text)
{
    assert(tx_len == strlen(text) && memcmp(transmitting,text,tx_len) == 0);
    port.gState = HAL_UART_STATE_READY;
    PID_DemoPoll();
}
int main(void)
{
    PID_DemoInit(&port);
    PID_DemoPoll();
    receive("@RUN\n");
    assert(pid_running);
    for (tick = 10; tick <= 400; tick += 10) PID_DemoPoll();
    /* The wire is stalled. The frame remains intact, and commands precede telemetry. */
    expect_frame("#NAMES:target,actual,output,error\n");
    expect_frame("#CAP:PING\n");
    expect_frame("@ACK:RUN\n");
    tick = 500; PID_DemoPoll();
    assert(!pid_running);
    port.gState = HAL_UART_STATE_READY;
    PID_DemoPoll();
    expect_frame("#STOP:heartbeat\n");
    return 0;
}
