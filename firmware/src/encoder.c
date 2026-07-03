#include "encoder.h"
#include "stm32f1xx_hal.h"

/* PA0 = TIM2_CH1, PA1 = TIM2_CH2, hardware quadrature encoder mode. */
extern TIM_HandleTypeDef htim2;

static volatile int32_t total_counts = 0;
static uint16_t last_cnt = 0;
static float last_speed = 0.0f;

void encoder_init(void) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    encoder_reset();
}

void encoder_update(float dt) {
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    /* 16-bit unsigned subtraction wraps correctly even across the
       hardware counter's overflow, as long as it doesn't turn more
       than half a revolution's worth of counts between updates. */
    int16_t delta = (int16_t)(cnt - last_cnt);
    last_cnt = cnt;

    total_counts += delta;
    last_speed = (dt > 0.0f) ? ((float)delta / dt) : 0.0f;
}

void encoder_reset(void) {
    total_counts = 0;
    last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    last_speed = 0.0f;
}

int32_t encoder_get_position_counts(void) {
    return total_counts;
}

float encoder_get_speed_counts_per_sec(void) {
    return last_speed;
}
