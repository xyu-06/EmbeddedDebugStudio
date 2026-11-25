/* Host-side behavior test. Compile together with firmware_example/pid_link.c. */
#include "pid_link.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static char reply[256];
static unsigned stops;
void PID_Write(const char *data, unsigned len)
{
    assert(len < sizeof reply);
    memcpy(reply, data, len);
    reply[len] = 0;
}
void PID_ZeroOutputs(void) { ++stops; }
static void send_line(const char *s)
{
    while (*s) PID_Consume((uint8_t)*s++);
    PID_Consume('\n');
}
static const char *fmt(float v)
{
    static char out[16];
    PID_FormatNumber(out, sizeof out, v);
    return out;
}

int main(void)
{
    /* Integer-only number formatting (6 decimals below 1, 4 at or above 1). */
    assert(!strcmp(fmt(0), "0") && !strcmp(fmt(-0.5f), "-0.5") && !strcmp(fmt(1.2f), "1.2"));
    assert(!strcmp(fmt(123.4567f), "123.4567") && !strcmp(fmt(0.000025f), "0.000025"));
    assert(!strcmp(fmt(-0.0000004f), "0"));
    assert(!strcmp(fmt(0.9999999f), "1") && !strcmp(fmt(1000000), "1000000"));
    assert(!strcmp(fmt(NAN), "nan"));

    PID_CheckFloatFormat();
    PID_WatchdogTick(0xffffff00u);
    send_line("@RUN");
    assert(pid_running && !strcmp(reply,"@ACK:RUN\n"));
    PID_Loop *p = &pid_loops[PID_ANG];
    p->kd = 1;
    PID_Position(p, 0, .01f, 100);
    send_line("@ANG.TGT=10");
    assert(!strcmp(reply,"@ACK:ANG.TGT=10\n"));
    assert(PID_Position(p, 0, .01f, 100) == 0);
    assert(fabsf(PID_Position(p, .1f, .01f, 100) + 10) < .001f);
    /* Gyro rate drives the D term directly. */
    assert(fabsf(PID_PositionRate(p, 0, 5, .01f, 100) + 5) < .001f);
    send_line("@ANG.RST");
    assert(PID_Incremental(p, 0, .01f, 100) == 0);
    send_line("@ANG.TGT=20");
    assert(PID_Incremental(p, 0, .01f, 100) == 0);
    assert(fabsf(PID_Incremental(p, .1f, .01f, 100) + 10) < .001f);

    /* Hand-written parser: decimals, signs, exponents, rejects junk. */
    send_line("@SPD.KP=-12.3456");
    assert(fabsf(pid_loops[PID_SPD].kp + 12.3456f) < 1e-4f);
    send_line("@SPD.KI=2.5e-3");
    assert(fabsf(pid_loops[PID_SPD].ki - .0025f) < 1e-7f);
    send_line("@SPD.KD=.75");
    assert(fabsf(pid_loops[PID_SPD].kd - .75f) < 1e-7f);
    send_line("@SPD.KD=1.5x");
    assert(!strcmp(reply,"@ERR:value out of range\n") && fabsf(pid_loops[PID_SPD].kd - .75f) < 1e-7f);
    send_line("@SPD.KD=2e7");
    assert(!strcmp(reply,"@ERR:value out of range\n"));
    send_line("@SPD.GET");
    assert(!strcmp(reply,"@PARAM:SPD.KP=-12.3456,SPD.KI=0.0025,SPD.KD=0.75,SPD.TGT=0\n"));

    /* Atomic SET updates all four loop parameters at once; bad value leaves all untouched. */
    send_line("@SPD.SET=1.5,2.5,0.25,80");
    assert(!strcmp(reply,"@ACK:SPD.SET=1.5,2.5,0.25,80\n"));
    assert(fabsf(pid_loops[PID_SPD].kp - 1.5f) < 1e-6f && fabsf(pid_loops[PID_SPD].target - 80.0f) < 1e-6f);
    send_line("@SPD.SET=9,9,bad,9");
    assert(!strcmp(reply,"@ERR:value out of range\n"));
    assert(fabsf(pid_loops[PID_SPD].kp - 1.5f) < 1e-6f && fabsf(pid_loops[PID_SPD].kd - 0.25f) < 1e-6f);

    PID_WatchdogTick(0xffffffc8u);
    send_line("@PING");
    PID_WatchdogTick(0x1bbu);
    assert(pid_running);
    PID_WatchdogTick(0x1bcu);
    assert(!pid_running && stops == 1 && !strcmp(reply,"#STOP:heartbeat\n"));
    assert(p->integral == 0 && p->output == 0);
    send_line("@PING");
    assert(!pid_running);

    /* A fault from the control interrupt stops at once and reports in foreground. */
    send_line("@RUN");
    PID_Fault("fall");
    assert(!pid_running && stops == 2);
    PID_WatchdogTick(0x1c0u);
    assert(!strcmp(reply,"#STOP:fall\n") && stops == 3);

    send_line("@RUN");
    send_line("@STOP");
    assert(!pid_running && stops == 4 && !strcmp(reply,"@ACK:STOP\n"));
    PID_Report(1, .25f, 2);
    assert(!strcmp(reply,"1,0.25,2,0.75\n"));
    const float two[] = {-3, 4.5f};
    PID_ReportValues(two, 2);
    assert(!strcmp(reply,"-3,4.5\n"));
    const float six[] = {1, 2, 3, 4, 5.5f, -6.25f};
    PID_ReportValues(six, 6);
    assert(!strcmp(reply,"1,2,3,4,5.5,-6.25\n"));
    return 0;
}
