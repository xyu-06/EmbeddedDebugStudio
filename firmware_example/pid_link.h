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
extern volatile uint8_t pid_running;

/* Application hooks. PID_Write executes in foreground only.
 * PID_ZeroOutputs may also be called from the control interrupt through
 * PID_Fault, so it must only write PWM compare registers / an enable pin. */
void PID_Write(const char *data, unsigned len);
void PID_ZeroOutputs(void);

/* Feed complete UART bytes from a foreground ring-buffer consumer. */
void PID_Consume(uint8_t byte);
void PID_DiscardPartial(void);
/* Foreground, every main-loop pass: heartbeat watchdog and deferred fault report. */
void PID_WatchdogTick(uint32_t now_ms);
/* Kept for compatibility. Numbers are formatted with integer printf only,
 * so newlib-nano does not need -u _printf_float (saves Flash on F103C8). */
void PID_CheckFloatFormat(void);

/* Telemetry: 1..4 channels per line, 4 decimals, trailing zeros trimmed. */
void PID_ReportValues(const float *values, unsigned count);
void PID_Report(float target, float actual, float output);

/* Stop from any context (e.g. fall detection in the control ISR). The motors
 * are zeroed immediately, "#STOP:<reason>" is sent by PID_WatchdogTick. */
void PID_Fault(const char *reason);

/* Positional PID, D on measurement (differenced), conditional integration. */
float PID_Position(PID_Loop *p, float actual, float dt, float limit);
/* Same, but the D term uses a measured rate, e.g. the MPU6050 gyro for the
 * upright loop. Much less noisy than differencing the angle. */
float PID_PositionRate(PID_Loop *p, float actual, float rate, float dt, float limit);
float PID_Incremental(PID_Loop *p, float actual, float dt, float limit);

/* Format v (6 decimals below 1, 4 at or above 1) into out (>= 16 bytes). */
int PID_FormatNumber(char *out, unsigned size, float v);
#endif
