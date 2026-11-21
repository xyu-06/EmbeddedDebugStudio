#ifndef PID_LINK_H
#define PID_LINK_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd, target;
    float integral, prev_error, prev_actual, prev_rate, output;
    uint8_t initialized;
} PID_Loop;

enum { PID_ANG, PID_SPD, PID_YAW, PID_AUX, PID_GROUPS };

extern PID_Loop pid_loops[PID_GROUPS];
extern uint8_t pid_running;

/* Application hooks. All callbacks execute in foreground, including STOP. */
void PID_Write(const char *data, unsigned len);
void PID_ZeroOutputs(void);

/* Feed complete UART bytes from a foreground ring-buffer consumer. */
void PID_Consume(uint8_t byte);
void PID_DiscardPartial(void);
void PID_WatchdogTick(uint32_t now_ms);
void PID_CheckFloatFormat(void);
void PID_Report(float target, float actual, float output);
float PID_Position(PID_Loop *p, float actual, float dt, float limit);
float PID_Incremental(PID_Loop *p, float actual, float dt, float limit);
#endif

