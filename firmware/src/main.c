/*
 * STM32F103C8 ("Blue Pill") application entry point.
 *
 * Wires together the hardware-independent control core (pid.c,
 * protocol.c -- the same files the native simulation in
 * test/native/ links against) with the STM32 HAL peripheral drivers
 * (encoder.c, motor.c) and a USART link to the Python dashboard.
 *
 * Peripheral map:
 *   TIM2 CH1/CH2 (PA0/PA1) - hardware quadrature encoder input
 *   TIM3 CH1 (PA6)         - motor PWM magnitude
 *   PB0 / PB1               - motor direction (H-bridge IN1/IN2)
 *   TIM4                    - 100 Hz control-loop tick interrupt
 *   USART1 (PA9/PA10)      - 115200 8N1 link to host dashboard
 *   PC13                    - status LED (blinks while enabled)
 *
 * See docs/wiring.md for the full wiring diagram.
 */
#include "stm32f1xx_hal.h"
#include "pid.h"
#include "protocol.h"
#include "encoder.h"
#include "motor.h"

#define CONTROL_HZ 100
#define CONTROL_DT (1.0f / CONTROL_HZ)

TIM_HandleTypeDef htim2; /* encoder */
TIM_HandleTypeDef htim3; /* motor PWM */
TIM_HandleTypeDef htim4; /* control tick */
UART_HandleTypeDef huart1;

static PidController pid;
static volatile int mode = MODE_POSITION;
static volatile float setpoint = 0.0f;
static volatile int enabled = 0;
static volatile uint32_t tick_ms = 0;
static volatile uint8_t control_tick_ready = 0;

static uint8_t rx_byte;
static char rx_line[64];
static size_t rx_len = 0;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Encoder_Init(void);
static void MX_TIM3_PWM_Init(void);
static void MX_TIM4_ControlTick_Init(void);
static void MX_USART1_UART_Init(void);
static void apply_command(const char *line);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Encoder_Init();
    MX_TIM3_PWM_Init();
    MX_TIM4_ControlTick_Init();
    MX_USART1_UART_Init();

    encoder_init();
    motor_init();
    pid_init(&pid, 0.08f, 0.35f, 0.002f, -100.0f, 100.0f, CONTROL_DT);

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    HAL_TIM_Base_Start_IT(&htim4);

    while (1) {
        if (control_tick_ready) {
            control_tick_ready = 0;

            encoder_update(CONTROL_DT);
            float measurement = (mode == MODE_POSITION)
                ? (float)encoder_get_position_counts()
                : encoder_get_speed_counts_per_sec();

            float output = enabled ? pid_step(&pid, setpoint, measurement) : 0.0f;
            if (enabled) {
                motor_set_duty(output);
            } else {
                motor_stop();
            }

            Telemetry telem = {
                .t_ms = tick_ms,
                .mode = mode,
                .setpoint = setpoint,
                .position = (float)encoder_get_position_counts(),
                .speed = encoder_get_speed_counts_per_sec(),
                .error = setpoint - measurement,
                .output = output,
            };
            char out[96];
            int n = protocol_format_telemetry(out, sizeof(out), &telem);
            if (n > 0) {
                HAL_UART_Transmit(&huart1, (uint8_t *)out, (uint16_t)n, 20);
            }

            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}

/* Fires at CONTROL_HZ from TIM4's update interrupt. Kept minimal:
   just marks the tick and advances the clock, all real work happens
   in the main loop so a slow UART transmit never delays the timer
   ISR return. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        control_tick_ready = 1;
        tick_ms += (uint32_t)(CONTROL_DT * 1000.0f);
    }
}

/* One byte at a time from the host; assembles a line and applies it
   once '\n' is seen, then re-arms the interrupt for the next byte. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_byte == '\n') {
            rx_line[rx_len] = '\0';
            apply_command(rx_line);
            rx_len = 0;
        } else if (rx_len < sizeof(rx_line) - 1) {
            rx_line[rx_len++] = (char)rx_byte;
        } else {
            rx_len = 0; /* overflow: drop the malformed line */
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

static void apply_command(const char *line) {
    Command cmd = protocol_parse_command(line);
    switch (cmd.type) {
        case CMD_SET_GAINS:
            pid_set_gains(&pid, cmd.kp, cmd.ki, cmd.kd);
            break;
        case CMD_SET_SETPOINT:
            setpoint = cmd.setpoint;
            break;
        case CMD_SET_MODE:
            mode = cmd.mode;
            pid_reset(&pid);
            break;
        case CMD_ENABLE:
            enabled = cmd.enable;
            if (!enabled) pid_reset(&pid);
            break;
        case CMD_RESET:
            encoder_reset();
            pid_reset(&pid);
            break;
        default:
            break;
    }
}

/* 72 MHz SYSCLK from the 8 MHz HSE crystal fitted on Blue Pill boards
   (PLL x9), APB1 = 36 MHz, APB2 = 72 MHz -- the standard Blue Pill
   clock tree used by most CubeMX-generated projects for this board. */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* PC13: onboard status LED. */
    gpio.Pin = GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* PB0/PB1: H-bridge direction pins. */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void MX_TIM2_Encoder_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM_Encoder_InitTypeDef enc = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF; /* full 16-bit range */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    enc.EncoderMode = TIM_ENCODERMODE_TI12; /* 4x quadrature decoding */
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 6;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 6;
    HAL_TIM_Encoder_Init(&htim2, &enc);

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void MX_TIM3_PWM_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* APB1 timer clock = 72 MHz; prescaler+period chosen for a
       ~20 kHz PWM carrier, well above audible/mechanical resonance. */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 3599; /* 72MHz / 3600 = 20 kHz */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void MX_TIM4_ControlTick_Init(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* APB1 timer clock = 72 MHz -> prescale to 10 kHz, period 100 ->
       100 Hz control tick interrupt. */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 7199;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 99;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);

    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

static void MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_9; /* TX */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_10; /* RX */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}
