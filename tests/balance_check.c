/* Closed-loop host test of balance_example.c against a crude inverted pendulum.
 * Compile with firmware_example/pid_link.c and firmware_example/balance_example.c. */
#include "balance_example.h"
#include "pid_link.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static char last[128];
static float pitch = 2, rate, applied;
static int32_t wheels;
void PID_Write(const char *d, unsigned n) { if (n < sizeof last) { memcpy(last, d, n); last[n] = 0; } }
void Balance_ZeroMotors(void);
void PID_ZeroOutputs(void) { Balance_ZeroMotors(); }
void Board_ReadImu(float *p, float *g, float *y) { *p = pitch; *g = rate; *y = 0; }
void Board_ReadEncoders(int32_t *l, int32_t *r) { *l = *r = wheels; }
void Board_SetMotors(int32_t l, int32_t r) { applied = (float)(l + r) * .5f; }

static void physics(void)
{
    /* deg/s^2: gravity tips it over; the motor term has the sign the PID
     * convention expects (output = KP*(target - pitch) - KD*rate). */
    float accel = 60.0f * sinf(pitch * 3.14159265f / 180) * 57.29578f + .4f * applied;
    rate += accel * BALANCE_DT_S;
    pitch += rate * BALANCE_DT_S;
    wheels = (int32_t)(applied / 200);
}

static void feed(const char *s) { while (*s) PID_Consume((uint8_t)*s++); PID_Consume('\n'); }

int main(void)
{
    uint32_t ms = 0;
    Balance_Init();
    assert(!strcmp(last, "#NAMES:pitch_tgt,pitch,pwm,speed\n"));
    pid_loops[PID_SPD].kp = pid_loops[PID_SPD].ki = 0;  /* upright loop only */
    PID_WatchdogTick(ms);
    feed("@RUN");
    for (int i = 0; i < 400; ++i) {           /* 2 s with heartbeats */
        Balance_ControlISR();
        physics();
        ms += 5;
        if (ms % 200 == 0) feed("@PING");
        Balance_Poll(ms);
    }
    assert(pid_running && fabsf(pitch) < 1.0f);
    pitch = 50;                                /* knocked over */
    Balance_ControlISR();
    assert(!pid_running && applied == 0);
    Balance_Poll(ms += 5);
    assert(!strcmp(last, "#STOP:fall\n"));
    return 0;
}
