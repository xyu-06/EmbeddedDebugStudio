#include "pid_link.h"
#include <math.h>
#include <string.h>

/* No printf/scanf: numbers are parsed and formatted here with integer
 * arithmetic, so the example fits an STM32F103C8 (64 KB) with newlib-nano
 * and needs neither -u _printf_float nor strtof. */

PID_Loop pid_loops[PID_GROUPS];
volatile uint8_t pid_running;

static const char *const groups[] = {"ANG", "SPD", "YAW", "AUX"};
static char line[128];
static unsigned used;
static uint8_t skip_line;
static uint32_t now_ms, last_ping_ms;
static const char *volatile fault_reason;

static void write_text(const char *text)
{
    PID_Write(text, (unsigned)strlen(text));
}

static void append(char *buf, unsigned size, unsigned *pos, const char *s)
{
    while (*s && *pos + 1 < size) buf[(*pos)++] = *s++;
    buf[*pos] = 0;
}

int PID_FormatNumber(char *out, unsigned size, float v)
{
    char tmp[24];
    unsigned n = 0, pos = 0;
    if (size < 2) return 0;
    if (!isfinite(v)) { out[0] = 0; append(out, size, &pos, "nan"); return (int)pos; }
    uint8_t negative = v < 0;
    float a = negative ? -v : v;
    if (a > 4.0e9f) a = 4.0e9f;
    uint32_t ip = (uint32_t)a;
    /* 6 decimals below 1 (for small Ki/Kd), 4 decimals at or above 1. */
    unsigned digits = ip == 0 ? 6U : 4U;
    uint32_t scale = ip == 0 ? 1000000U : 10000U;
    uint32_t frac = (uint32_t)((a - (float)ip) * (float)scale + 0.5f);
    if (frac >= scale) { ++ip; frac -= scale; }
    if (frac) {
        while (frac % 10U == 0) { frac /= 10U; --digits; }
        for (unsigned i = 0; i < digits; ++i) { tmp[n++] = (char)('0' + frac % 10U); frac /= 10U; }
        tmp[n++] = '.';
    }
    do { tmp[n++] = (char)('0' + ip % 10U); ip /= 10U; } while (ip);
    if (negative && !(n == 1 && tmp[0] == '0')) tmp[n++] = '-';
    while (n && pos + 1 < size) out[pos++] = tmp[--n];
    out[pos] = 0;
    return (int)pos;
}

/* [+-]digits[.digits][(e|E)[+-]digits], whole string. Returns 1 on success. */
static int parse_number(const char *s, float *result)
{
    static const float pow10[] = {
        1e0f, 1e1f, 1e2f, 1e3f, 1e4f, 1e5f, 1e6f, 1e7f, 1e8f, 1e9f, 1e10f
    };
    uint8_t negative = 0, any = 0;
    uint32_t mantissa = 0;
    int exponent = 0, digits = 0;
    if (*s == '+' || *s == '-') negative = *s++ == '-';
    for (; *s >= '0' && *s <= '9'; ++s, any = 1) {
        if (digits < 9) { mantissa = mantissa * 10U + (uint32_t)(*s - '0'); if (mantissa) ++digits; }
        else ++exponent;
    }
    if (*s == '.') {
        for (++s; *s >= '0' && *s <= '9'; ++s, any = 1) {
            if (digits < 9) { mantissa = mantissa * 10U + (uint32_t)(*s - '0'); --exponent; if (mantissa) ++digits; }
        }
    }
    if (!any) return 0;
    if (*s == 'e' || *s == 'E') {
        int sign = 1, e = 0;
        ++s;
        if (*s == '+' || *s == '-') sign = *s++ == '-' ? -1 : 1;
        if (*s < '0' || *s > '9') return 0;
        for (; *s >= '0' && *s <= '9'; ++s) if (e < 100) e = e * 10 + (*s - '0');
        exponent += sign * e;
    }
    if (*s || exponent > 20) return 0;
    float v = (float)mantissa;
    while (exponent > 10) { v *= 1e10f; exponent -= 10; }
    if (exponent > 0) v *= pow10[exponent];
    while (exponent < -10 && v != 0.0f) { v /= 1e10f; exponent += 10; }
    if (exponent < 0 && v != 0.0f) v /= pow10[-exponent];
    *result = negative ? -v : v;
    return 1;
}

static void reset_state(PID_Loop *p)
{
    p->integral = p->prev_error = p->prev_actual = p->prev_rate = p->output = 0;
    p->initialized = 0;
}

/* pid_running is cleared first: a control interrupt that preempts the reset
 * sees it and returns without touching the loop state. */
static void stop_all(void)
{
    pid_running = 0;
    PID_ZeroOutputs();
    for (int i = 0; i < PID_GROUPS; ++i) reset_state(&pid_loops[i]);
}

void PID_Fault(const char *reason)
{
    pid_running = 0;
    PID_ZeroOutputs();
    fault_reason = reason;
}

void PID_WatchdogTick(uint32_t tick)
{
    now_ms = tick;
    const char *reason = fault_reason;
    if (reason) {
        char msg[48];
        unsigned pos = 0;
        fault_reason = 0;
        stop_all();
        msg[0] = 0;
        append(msg, sizeof msg, &pos, "#STOP:");
        append(msg, sizeof msg, &pos, reason);
        append(msg, sizeof msg, &pos, "\n");
        write_text(msg);
    }
    if (pid_running && (uint32_t)(tick - last_ping_ms) >= 500U) {
        stop_all();
        write_text("#STOP:heartbeat\n");
    }
}

void PID_CheckFloatFormat(void)
{
}

static float clamp(float v, float limit)
{
    return fminf(limit, fmaxf(-limit, v));
}

float PID_PositionRate(PID_Loop *p, float actual, float rate, float dt, float limit)
{
    if (!pid_running || dt <= 0 || limit <= 0) return 0;
    float e = p->target - actual;
    float trial = p->integral + p->ki * e * dt;
    float raw = p->kp * e + trial - p->kd * rate;
    p->output = clamp(raw, limit);
    if (raw == p->output || (raw - p->output) * p->ki * e < 0)
        p->integral = trial;
    p->prev_error = e;
    p->prev_actual = actual;
    p->prev_rate = rate;
    p->initialized = 1;
    return p->output;
}

float PID_Position(PID_Loop *p, float actual, float dt, float limit)
{
    if (!pid_running || dt <= 0 || limit <= 0) return 0;
    float rate = p->initialized ? (actual - p->prev_actual) / dt : 0;
    return PID_PositionRate(p, actual, rate, dt, limit);
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

static void param_reply(int g)
{
    static const char *const keys[] = {"KP", "KI", "KD", "TGT"};
    const PID_Loop *p = &pid_loops[g];
    const float values[] = {p->kp, p->ki, p->kd, p->target};
    char reply[160], num[16];
    unsigned pos = 0;
    reply[0] = 0;
    append(reply, sizeof reply, &pos, "@PARAM:");
    for (int i = 0; i < 4; ++i) {
        if (i) append(reply, sizeof reply, &pos, ",");
        append(reply, sizeof reply, &pos, groups[g]);
        append(reply, sizeof reply, &pos, ".");
        append(reply, sizeof reply, &pos, keys[i]);
        append(reply, sizeof reply, &pos, "=");
        PID_FormatNumber(num, sizeof num, values[i]);
        append(reply, sizeof reply, &pos, num);
    }
    append(reply, sizeof reply, &pos, "\n");
    write_text(reply);
}

static void command(void)
{
    if (line[0] != '@') return;
    char reply[sizeof line + 8];
    unsigned pos = 0;
    char *op = line + 1;
    if (strcmp(op, "STOP") == 0) {
        stop_all();
        write_text("@ACK:STOP\n");
        return;
    }
    if (strcmp(op, "RUN") == 0) {
        fault_reason = 0;
        last_ping_ms = now_ms;
        pid_running = 1;
        write_text("#CAP:PING\n");
        write_text("#CAP:SET\n");
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
        param_reply(g);
        return;
    }
    if (strcmp(op, "RST") == 0) {
        reset_state(p);
    } else if (strncmp(op, "SET=", 4) == 0) {
        char copy[120];
        float vals[4];
        size_t plen = strlen(op + 4);
        if (plen == 0 || plen >= sizeof copy) {
            write_text("@ERR:value out of range\n");
            return;
        }
        memcpy(copy, op + 4, plen + 1);
        char *cur = copy;
        for (int i = 0; i < 4; ++i) {
            char *comma = strchr(cur, ',');
            if ((i < 3 && !comma) || (i == 3 && comma)) {
                write_text("@ERR:expected 4 values\n");
                return;
            }
            if (comma) *comma = 0;
            if (!parse_number(cur, &vals[i]) || !isfinite(vals[i]) || fabsf(vals[i]) > 1000000.0f) {
                write_text("@ERR:value out of range\n");
                return;
            }
            if (comma) cur = comma + 1;
        }
        /* Atomic update: all 4 parameters validated before applying together.
         * Wrap with __disable_irq()/__enable_irq() if ISR reads pid_loops concurrently. */
        p->kp = vals[0];
        p->ki = vals[1];
        p->kd = vals[2];
        p->target = vals[3];
    } else {
        if (!eq || eq[1] == '\0') { write_text("@ERR:expected key=value\n"); return; }
        float v;
        if (!parse_number(eq+1, &v) || !isfinite(v) || fabsf(v) > 1000000.0f) {
            write_text("@ERR:value out of range\n"); return;
        }
        size_t n = (size_t)(eq-op);
        if (n == 2 && strncmp(op,"KP",2)==0) p->kp=v;
        else if (n == 2 && strncmp(op,"KI",2)==0) p->ki=v;
        else if (n == 2 && strncmp(op,"KD",2)==0) p->kd=v;
        else if (n == 3 && strncmp(op,"TGT",3)==0) p->target=v;
        else { write_text("@ERR:unknown key\n"); return; }
    }
    reply[0] = 0;
    append(reply, sizeof reply, &pos, "@ACK:");
    append(reply, sizeof reply, &pos, line+1);
    append(reply, sizeof reply, &pos, "\n");
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

void PID_ReportValues(const float *values, unsigned count)
{
    char buf[160], num[16];
    unsigned pos = 0;
    if (count < 1 || count > 8) return;
    buf[0] = 0;
    for (unsigned i = 0; i < count; ++i) {
        if (i) append(buf, sizeof buf, &pos, ",");
        PID_FormatNumber(num, sizeof num, values[i]);
        append(buf, sizeof buf, &pos, num);
    }
    append(buf, sizeof buf, &pos, "\n");
    write_text(buf);
}

void PID_Report(float target, float actual, float output)
{
    const float values[] = {target, actual, output, target - actual};
    PID_ReportValues(values, 4);
}
