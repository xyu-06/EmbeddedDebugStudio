#ifndef BALANCE_EXAMPLE_H
#define BALANCE_EXAMPLE_H
#include <stdint.h>

/* Two-wheel balance car on top of pid_link:
 *   ANG  upright loop, PD(+I) on pitch, D from the gyro (PID_PositionRate)
 *   SPD  speed loop, PI on the wheel speed sum, every BALANCE_SPEED_DIV ticks
 *   YAW  steering loop, PI on yaw rate (gyro z)
 *   pwm = ANG + SPD, left = pwm + YAW, right = pwm - YAW
 *
 * Integration:
 *   1. Implement the four Board_* hooks below for your MPU6050/encoders/TB6612.
 *   2. Call Balance_Init() once after HAL init and the UART link (hal_demo or your own).
 *   3. Call Balance_ControlISR() from a fixed 5 ms timer interrupt, e.g.
 *      HAL_TIM_PeriodElapsedCallback for TIM2/TIM3 at 200 Hz.
 *   4. Call Balance_Poll() from the main loop (telemetry and PID_WatchdogTick).
 * Signs depend on how the IMU and motors are mounted: adjust BALANCE_MOTOR_SIGN
 * and the encoder sign until a forward lean drives the wheels forward. */

#define BALANCE_DT_S        0.005f   /* control period, must match the timer */
#define BALANCE_SPEED_DIV   8        /* speed loop every 8 x 5 ms = 40 ms */
#define BALANCE_PWM_MAX     7000.0f  /* TB6612 compare value at 100 % */
#define BALANCE_DEADZONE    300.0f   /* compare value where the wheels start to turn */
#define BALANCE_FALL_DEG    40.0f    /* |pitch - ANG.TGT| beyond this stops the motors */
#define BALANCE_MOTOR_SIGN  1.0f

/* Board hooks, implemented by the application. Called from the control ISR. */
void Board_ReadImu(float *pitch_deg, float *gyro_pitch_dps, float *gyro_yaw_dps);
void Board_ReadEncoders(int32_t *left_counts, int32_t *right_counts);
void Board_SetMotors(int32_t left_compare, int32_t right_compare);   /* signed */

void Balance_Init(void);
void Balance_ControlISR(void);
void Balance_Poll(uint32_t now_ms);
/* Call this from your PID_ZeroOutputs (ISR-safe: only writes the motor outputs). */
void Balance_ZeroMotors(void);
#endif
