#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* H-bridge motor driver (L298N / TB6612FNG style: PWM + 2 direction
   pins). Backed by TIM3 CH1 for PWM and two GPIO pins for direction. */

void motor_init(void);

/* percent in [-100, 100]; sign selects direction, magnitude is duty cycle. */
void motor_set_duty(float percent);

void motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
