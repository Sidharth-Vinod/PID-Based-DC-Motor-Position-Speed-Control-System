#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

/*
 * Small electromechanical DC motor plant model used ONLY by the
 * native PC simulation harness (test/native/). It stands in for the
 * physical motor + encoder so the exact same pid.c / protocol.c that
 * runs on the STM32 can be exercised in closed loop without hardware.
 *
 *   L di/dt    = V - R*i - Ke*w
 *   J dw/dt    = Kt*i - b*w - t_load
 *   dtheta/dt  = w
 */
typedef struct {
    /* Electrical */
    double R;   /* winding resistance, ohm */
    double L;   /* winding inductance, H */
    /* Mechanical */
    double J;   /* rotor + load inertia, kg*m^2 */
    double b;   /* viscous friction, N*m*s/rad */
    /* Coupling */
    double Kt;  /* torque constant, N*m/A */
    double Ke;  /* back-EMF constant, V*s/rad */

    double t_load; /* constant external load torque disturbance, N*m */

    /* State */
    double current;   /* A */
    double speed;      /* rad/s */
    double position;   /* rad, unwrapped */

    int encoder_counts_per_rev;
} MotorModel;

void motor_model_init_defaults(MotorModel *m);

/* Advances the plant by dt seconds given applied voltage (V), using
   fixed-substep RK4 integration for numerical stability at the
   control loop's sample rate. */
void motor_model_step(MotorModel *m, double voltage, double dt);

/* Convenience accessors in encoder units, matching what real
   hardware would report to the PID loop. */
double motor_model_position_counts(const MotorModel *m);
double motor_model_speed_counts_per_sec(const MotorModel *m);

#endif /* MOTOR_MODEL_H */
