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

int main(void)
{
    PID_CheckFloatFormat();
    PID_WatchdogTick(0xffffff00u);
    send_line("@RUN");
    assert(pid_running && !strcmp(reply,"@ACK:RUN\n"));
    PID_Loop *p = &pid_loops[PID_ANG];
    p->kd = 1;
    PID_Position(p, 0, .01f, 100);
    send_line("@ANG.TGT=10");
    assert(PID_Position(p, 0, .01f, 100) == 0);
    assert(fabsf(PID_Position(p, .1f, .01f, 100) + 10) < .001f);
    send_line("@ANG.RST");
    assert(PID_Incremental(p, 0, .01f, 100) == 0);
    send_line("@ANG.TGT=20");
    assert(PID_Incremental(p, 0, .01f, 100) == 0);
    assert(fabsf(PID_Incremental(p, .1f, .01f, 100) + 10) < .001f);
    PID_WatchdogTick(0xffffffc8u);
    send_line("@PING");
    PID_WatchdogTick(0x1bbu);
    assert(pid_running);
    PID_WatchdogTick(0x1bcu);
    assert(!pid_running && stops == 1 && !strcmp(reply,"#STOP:heartbeat\n"));
    assert(p->integral == 0 && p->output == 0);
    send_line("@PING");
    assert(!pid_running);
    send_line("@RUN");
    send_line("@STOP");
    assert(!pid_running && stops == 2 && !strcmp(reply,"@ACK:STOP\n"));
    PID_Report(1, .25f, 2);
    assert(!strcmp(reply,"1,0.25,2,0.75\n"));
    return 0;
}
