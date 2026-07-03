#include "motor_model.h"
#include <math.h>

void motor_model_init_defaults(MotorModel *m) {
    /* Roughly a small brushed DC gearmotor with an integrated
       quadrature encoder, e.g. the kind used on hobby robotics
       platforms. Numbers are illustrative, not a datasheet. */
    m->R = 1.0;      /* ohm */
    m->L = 0.5e-3;   /* H */
    m->J = 0.01;     /* kg*m^2, load reflected through gearbox */
    m->b = 0.1;      /* N*m*s/rad */
    m->Kt = 0.05;    /* N*m/A */
    m->Ke = 0.05;    /* V*s/rad */
    m->t_load = 0.0;

    m->current = 0.0;
    m->speed = 0.0;
    m->position = 0.0;

    m->encoder_counts_per_rev = 3000;
}

typedef struct {
    double di;
    double dw;
    double dtheta;
} Derivative;

static Derivative motor_deriv(const MotorModel *m, double current, double speed,
                               double voltage) {
    Derivative d;
    d.di = (voltage - m->R * current - m->Ke * speed) / m->L;
    d.dw = (m->Kt * current - m->b * speed - m->t_load) / m->J;
    d.dtheta = speed;
    return d;
}

void motor_model_step(MotorModel *m, double voltage, double dt) {
    /* Electrical time constant (L/R) is typically much faster than
       the mechanical one, so subdivide into small RK4 substeps for
       numerical stability regardless of the outer control period. */
    const int substeps = 50;
    double h = dt / substeps;

    for (int s = 0; s < substeps; s++) {
        double i0 = m->current, w0 = m->speed, th0 = m->position;

        Derivative k1 = motor_deriv(m, i0, w0, voltage);
        Derivative k2 = motor_deriv(m, i0 + 0.5 * h * k1.di,
                                    w0 + 0.5 * h * k1.dw, voltage);
        Derivative k3 = motor_deriv(m, i0 + 0.5 * h * k2.di,
                                    w0 + 0.5 * h * k2.dw, voltage);
        Derivative k4 = motor_deriv(m, i0 + h * k3.di, w0 + h * k3.dw, voltage);

        m->current = i0 + (h / 6.0) * (k1.di + 2 * k2.di + 2 * k3.di + k4.di);
        m->speed = w0 + (h / 6.0) * (k1.dw + 2 * k2.dw + 2 * k3.dw + k4.dw);
        m->position = th0 + (h / 6.0) * (k1.dtheta + 2 * k2.dtheta +
                                          2 * k3.dtheta + k4.dtheta);
    }
}

double motor_model_position_counts(const MotorModel *m) {
    return m->position / (2.0 * M_PI) * m->encoder_counts_per_rev;
}

double motor_model_speed_counts_per_sec(const MotorModel *m) {
    return m->speed / (2.0 * M_PI) * m->encoder_counts_per_rev;
}
