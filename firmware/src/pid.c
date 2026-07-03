#include "pid.h"

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(PidController *pid, float kp, float ki, float kd,
              float out_min, float out_max, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->dt = dt;
    pid_reset(pid);
}

void pid_set_gains(PidController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_reset(PidController *pid) {
    pid->integrator = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev_measurement = 0;
}

float pid_step(PidController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    float d_measurement = 0.0f;
    if (pid->has_prev_measurement) {
        d_measurement = (measurement - pid->prev_measurement) / pid->dt;
    }
    pid->prev_measurement = measurement;
    pid->has_prev_measurement = 1;

    float p_term = pid->kp * error;
    float d_term = -pid->kd * d_measurement;

    /* Tentative integrator update, applied only if it doesn't push the
       unsaturated output further past the rail (clamped anti-windup). */
    float tentative_integrator = pid->integrator + error * pid->dt;
    float tentative_output = p_term + pid->ki * tentative_integrator + d_term;

    if (tentative_output > pid->out_max) {
        if (error > 0.0f) {
            /* Would saturate further high; hold integrator. */
        } else {
            pid->integrator = tentative_integrator;
        }
    } else if (tentative_output < pid->out_min) {
        if (error < 0.0f) {
            /* Would saturate further low; hold integrator. */
        } else {
            pid->integrator = tentative_integrator;
        }
    } else {
        pid->integrator = tentative_integrator;
    }

    float output = p_term + pid->ki * pid->integrator + d_term;
    return clampf(output, pid->out_min, pid->out_max);
}
