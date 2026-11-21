#ifndef PID_HAL_DEMO_H
#define PID_HAL_DEMO_H
#include "main.h"
void PID_DemoInit(UART_HandleTypeDef *handle);
void PID_DemoPoll(void);
void PID_DemoRxDone(UART_HandleTypeDef *handle);
void PID_DemoRxError(UART_HandleTypeDef *handle);
#endif

