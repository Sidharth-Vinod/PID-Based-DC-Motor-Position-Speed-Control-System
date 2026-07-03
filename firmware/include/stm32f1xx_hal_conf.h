#ifndef STM32F1xx_HAL_CONF_H
#define STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal HAL module set enabled for this project: GPIO, TIM (encoder
   + PWM + control tick), USART, RCC/CORTEX (required by all HAL
   apps). Trimmed from the CubeMX default template. */
#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED

#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U /* Blue Pill onboard crystal */
#endif
#define HSE_STARTUP_TIMEOUT 100U

#if !defined(HSI_VALUE)
#define HSI_VALUE 8000000U
#endif

#define VDD_VALUE 3300U
#define TICK_INT_PRIORITY 0U
#define USE_RTOS 0U
#define PREFETCH_ENABLE 1U

#define assert_param(expr) ((void)0U)

/* stm32f1xx_hal.h consults the *_MODULE_ENABLED defines above to pull
   in exactly the module headers this project needs. */

#ifdef __cplusplus
}
#endif

#endif /* STM32F1xx_HAL_CONF_H */
