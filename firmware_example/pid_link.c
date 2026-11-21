#include "pid_link.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

PID_Loop pid_loops[PID_GROUPS];
uint8_t pid_running;

static const char *const groups[] = {"ANG", "SPD", "YAW", "AUX"};
static char line[100];
static unsigned used;
static uint8_t skip_line;
static uint8_t float_format_ok;
static uint32_t now_ms, last_ping_ms;

static void write_text(const char *text)
{
    PID_Write(text, (unsigned)strlen(text));
}

static void reset_state(PID_Loop *p)
{
    p->integral = p->prev_error = p->prev_actual = p->prev_rate = p->output = 0;
    p->initialized = 0;
}

static void stop_all(void)
{
    pid_running = 0;
    PID_ZeroOutputs();
    for (int i = 0; i < PID_GROUPS; ++i) reset_state(&pid_loops[i]);
}

void PID_WatchdogTick(uint32_t tick)
{
    now_ms = tick;
    if (pid_running && (uint32_t)(tick - last_ping_ms) >= 500U) {
        stop_all();
        write_text("#STOP:heartbeat\n");
    }
}

void PID_CheckFloatFormat(void)
{
    char probe[16];
    volatile double check_value = 1.25;
    snprintf(probe, sizeof probe, "%.6g", check_value);
    float_format_ok = strcmp(probe, "1.25") == 0;
    if (!float_format_ok) write_text("#ERROR:float printf unavailable; link -u _printf_float\n");
}

static float clamp(float v, float limit)
{
    return fminf(limit, fmaxf(-limit, v));
}

float PID_Position(PID_Loop *p, float actual, float dt, float limit)
{
    if (!pid_running || dt <= 0 || limit <= 0) return 0;
    float e = p->target - actual;
    float rate = p->initialized ? (actual - p->prev_actual) / dt : 0;
    float trial = p->integral + p->ki * e * dt;
    float raw = p->kp * e + trial - p->kd * rate;
    p->output = clamp(raw, limit);
    if (raw == p->output || (raw - p->output) * p->ki * e < 0)
        p->integral = trial;
    p->prev_error = e;
    p->prev_actual = actual;
    p->initialized = 1;
    return p->output;
}

float PID_Incremental(PID_Loop *p, float actual, float dt, float limit)
{
    if (!pid_running || dt <= 0 || limit <= 0) return 0;
    float e = p->target - actual;
    float rate = p->initialized ? (actual - p->prev_actual) / dt : 0;
    float delta = p->kp * (e - p->prev_error) - p->kd * (rate - p->prev_rate);
    p->output = clamp(p->output + delta + p->ki * e * dt, limit);
    p->prev_rate = rate;
    p->prev_actual = actual;
    p->prev_error = e;
    p->initialized = 1;
    return p->output;
}

static void command(void)
{
    if (line[0] != '@') return;
    char reply[192];
    char *op = line + 1;
    if (strcmp(op, "STOP") == 0) {
        stop_all();
        write_text("@ACK:STOP\n");
        return;
    }
    if (strcmp(op, "RUN") == 0) {
        last_ping_ms = now_ms;
        pid_running = 1;
        write_text("#CAP:PING\n");
        write_text("@ACK:RUN\n");
        return;
    }
    if (strcmp(op, "PING") == 0) {
        if (pid_running) last_ping_ms = now_ms;
        return;
    }
    int g = PID_ANG;
    /* A decimal point in the value follows '=', not the loop prefix. */
    char *dot = strchr(op, '.');
    char *eq = strchr(op, '=');
    if (dot && (!eq || dot < eq)) {
        for (g = 0; g < PID_GROUPS; ++g)
            if ((size_t)(dot-op) == strlen(groups[g]) &&
                strncmp(op, groups[g], (size_t)(dot-op)) == 0) break;
        if (g == PID_GROUPS) { write_text("@ERR:unknown group\n"); return; }
        op = dot + 1;
    }
    PID_Loop *p = &pid_loops[g];
    if (strcmp(op, "GET") == 0) {
        if (!float_format_ok) { write_text("@ERR:link -u _printf_float\n"); return; }
        snprintf(reply, sizeof reply, "@PARAM:%s.KP=%.8g,%s.KI=%.8g,%s.KD=%.8g,%s.TGT=%.8g\n",
                 groups[g], (double)p->kp, groups[g], (double)p->ki,
                 groups[g], (double)p->kd, groups[g], (double)p->target);
        write_text(reply);
        return;
    }
    if (strcmp(op, "RST") == 0) {
        reset_state(p);
    } else {
        if (!eq || eq[1] == '\0') { write_text("@ERR:expected key=value\n"); return; }
        errno = 0;
        char *end;
        float v = strtof(eq+1, &end);
        if (*end || errno == ERANGE || !isfinite(v) || fabsf(v) > 1000000.0f) {
            write_text("@ERR:value out of range\n"); return;
        }
        size_t n = (size_t)(eq-op);
        if (n == 2 && strncmp(op,"KP",2)==0) p->kp=v;
        else if (n == 2 && strncmp(op,"KI",2)==0) p->ki=v;
        else if (n == 2 && strncmp(op,"KD",2)==0) p->kd=v;
        else if (n == 3 && strncmp(op,"TGT",3)==0) p->target=v;
        else { write_text("@ERR:unknown key\n"); return; }
    }
    snprintf(reply, sizeof reply, "@ACK:%s\n", line+1);
    write_text(reply);
}

void PID_DiscardPartial(void)
{
    used = 0;
    skip_line = 1;
}

void PID_Consume(uint8_t b)
{
    if (b == '\n') {
        if (!skip_line && used) { line[used] = 0; command(); }
        used = skip_line = 0;
    } else if (b != '\r' && !skip_line) {
        if (b < 32 || b > 126 || used >= sizeof line - 1) {
            PID_DiscardPartial();
            write_text("@ERR:invalid line\n");
        } else line[used++] = (char)b;
    }
}

void PID_Report(float target, float actual, float output)
{
    if (!float_format_ok) return;
    char buf[88];
    snprintf(buf, sizeof buf, "%.6g,%.6g,%.6g,%.6g\n", (double)target,
             (double)actual, (double)output, (double)(target-actual));
    write_text(buf);
}

