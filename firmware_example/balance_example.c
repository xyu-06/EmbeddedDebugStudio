/* Balance car control skeleton for the PID tuner. Hardware-independent:
 * see balance_example.h for the hooks and the integration steps.
 * PID_ZeroOutputs is defined in your UART glue (hal_demo.c defines a
 * numeric one); for a real car make it call Balance_ZeroMotors(). */
#include <math.h>
#include "balance_example.h"
#include "pid_link.h"

static volatile float report[5];     /* written in the ISR, read in Poll */
static uint32_t report_ms;
static unsigned speed_tick;
static float speed_filtered, speed_out;

static float compensate(float pwm)
{
    if (pwm > 0.5f) pwm += BALANCE_DEADZONE;
    else if (pwm < -0.5f) pwm -= BALANCE_DEADZONE;
    return fminf(BALANCE_PWM_MAX, fmaxf(-BALANCE_PWM_MAX, pwm));
}

void Balance_ZeroMotors(void)
{
    Board_SetMotors(0, 0);
}

void Balance_Init(void)
{
    /* Starting points only (compare units per degree); tune them in the tool. */
    pid_loops[PID_ANG].kp = 300.0f;  pid_loops[PID_ANG].kd = 20.0f;  pid_loops[PID_ANG].target = 0.0f;
    pid_loops[PID_SPD].kp = 60.0f;   pid_loops[PID_SPD].ki = 0.3f;
    pid_loops[PID_YAW].kp = 20.0f;
    speed_tick = 0;
    speed_filtered = speed_out = 0;
    Balance_ZeroMotors();
    static const char names[] = "#NAMES:pitch_target,pitch,gyro_pitch,pwm,wheel_speed\n";
    static const char roles[] = "#ROLES:target,actual,other,output,other\n";
    PID_Write(names, sizeof names - 1);
    PID_Write(roles, sizeof roles - 1);
}

void Balance_ControlISR(void)
{
    float pitch, gyro, gyro_yaw;
    int32_t left, right;
    Board_ReadImu(&pitch, &gyro, &gyro_yaw);
    Board_ReadEncoders(&left, &right);
    float speed = (float)(left + right);
    speed_filtered += 0.3f * (speed - speed_filtered);
    report[0] = pid_loops[PID_ANG].target;
    report[1] = pitch;
    report[2] = gyro;
    report[4] = speed_filtered;
    if (!pid_running) {
        speed_tick = 0;
        speed_out = 0;
        report[3] = 0;
        return;
    }
    if (fabsf(pitch - pid_loops[PID_ANG].target) > BALANCE_FALL_DEG) {
        PID_Fault("fall");
        report[3] = 0;
        return;
    }
    if (++speed_tick >= BALANCE_SPEED_DIV) {
        speed_tick = 0;
        speed_out = PID_Position(&pid_loops[PID_SPD], speed_filtered,
                                 BALANCE_DT_S * BALANCE_SPEED_DIV, BALANCE_PWM_MAX * 0.5f);
    }
    float upright = PID_PositionRate(&pid_loops[PID_ANG], pitch, gyro, BALANCE_DT_S, BALANCE_PWM_MAX);
    float turn = PID_Position(&pid_loops[PID_YAW], gyro_yaw, BALANCE_DT_S, BALANCE_PWM_MAX * 0.3f);
    float pwm = BALANCE_MOTOR_SIGN * (upright + speed_out);
    float l = compensate(pwm + turn), r = compensate(pwm - turn);
    report[3] = pwm;
    if (!pid_running) { Balance_ZeroMotors(); return; }   /* stopped meanwhile */
    Board_SetMotors((int32_t)l, (int32_t)r);
}

void Balance_Poll(uint32_t now_ms)
{
    PID_WatchdogTick(now_ms);
    if ((uint32_t)(now_ms - report_ms) >= 10U) {
        report_ms = now_ms;
        const float values[5] = {report[0], report[1], report[2], report[3], report[4]};
        PID_ReportValues(values, 5);
    }
}
