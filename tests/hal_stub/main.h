#ifndef PID_TEST_HAL_H
#define PID_TEST_HAL_H
#include <stdint.h>
typedef struct { unsigned gState; } UART_HandleTypeDef;
enum { HAL_OK, HAL_BUSY, HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_TX };
uint32_t HAL_GetTick(void);
int HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
