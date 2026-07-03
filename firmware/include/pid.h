#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Portable PID controller core.
 *
 * No hardware dependencies (no HAL, no Arduino headers) so this exact
 * source file is compiled both into the STM32 firmware and into the
 * native PC simulation harness under test/native/. The control law
 * under test on real hardware is therefore bit-for-bit what the
 * simulation validates.
 */

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;

    float integrator;
    float prev_measurement;
    int has_prev_measurement;

    float dt; /* seconds, fixed sample time of the control loop */
} PidController;

void pid_init(PidController *pid, float kp, float ki, float kd,
              float out_min, float out_max, float dt);

void pid_set_gains(PidController *pid, float kp, float ki, float kd);

/* Clears integrator/derivative history without touching gains. */
void pid_reset(PidController *pid);

/*
 * Computes one control step. Derivative acts on the measurement (not
 * the error) to avoid derivative kick when the setpoint changes
 * abruptly. The integrator uses clamped anti-windup: it only
 * accumulates while the unsaturated output would stay within range,
 * so a persistently saturated actuator does not wind the integral
 * term up unboundedly.
 */
float pid_step(PidController *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
