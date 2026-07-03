#include "motor.h"
#include "stm32f1xx_hal.h"

/* PA6 = TIM3_CH1 (PWM magnitude), PB0 = IN1, PB1 = IN2 (direction). */
extern TIM_HandleTypeDef htim3;

#define MOTOR_IN1_PORT GPIOB
#define MOTOR_IN1_PIN GPIO_PIN_0
#define MOTOR_IN2_PORT GPIOB
#define MOTOR_IN2_PIN GPIO_PIN_1

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void motor_init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    motor_stop();
}

void motor_set_duty(float percent) {
    percent = clampf(percent, -100.0f, 100.0f);

    if (percent >= 0.0f) {
        HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
        percent = -percent;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t compare = (uint32_t)((percent / 100.0f) * (float)period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
}

void motor_stop(void) {
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}
