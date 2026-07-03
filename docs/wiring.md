# Wiring — STM32F103C8 ("Blue Pill")

Reference wiring for `firmware/`. Any TB6612FNG/L298N-style dual-direction
driver and any quadrature encoder motor will work as long as the pin
mapping below is preserved (or updated to match in `firmware/src/main.c`).

## Pin map

| Signal                | Blue Pill pin | Peripheral   | Notes                                   |
|------------------------|--------------|--------------|------------------------------------------|
| Encoder channel A      | PA0          | TIM2_CH1     | Hardware quadrature decode               |
| Encoder channel B      | PA1          | TIM2_CH2     | Hardware quadrature decode               |
| Motor PWM (magnitude)  | PA6          | TIM3_CH1     | ~20 kHz carrier                          |
| Motor direction IN1    | PB0          | GPIO output  | Drive high for forward                   |
| Motor direction IN2    | PB1          | GPIO output  | Drive high for reverse                   |
| UART TX (to host)      | PA9          | USART1_TX    | 115200 8N1                               |
| UART RX (from host)    | PA10         | USART1_RX    | 115200 8N1                               |
| Status LED             | PC13         | GPIO output  | Onboard LED, toggles every control tick   |

## Power

- Encoder: 3.3 V or 5 V per its datasheet (most Hall-effect quadrature
  encoders on hobby gearmotors are 5 V tolerant open-collector/push-pull —
  check before wiring directly to a 3.3 V-only MCU pin; add a level
  shifter or resistor divider if the encoder outputs 5 V logic).
- Motor driver logic (IN1/IN2/PWM): 3.3 V from the Blue Pill is fine for
  TB6612FNG and L298N logic inputs.
- Motor driver output stage (motor supply): separate supply matched to
  the motor's rated voltage, with a common ground back to the Blue Pill.
- USART: connect through a USB-TTL adapter (e.g. CP2102/FT232) for the
  Python dashboard's serial link — Blue Pill has no native USB-CDC.

## Programming

Flash over SWD with an ST-Link (or ST-Link V2 clone):

```
SWDIO -> PA13
SWCLK -> PA14
GND   -> GND
3V3   -> 3V3 (or power the board separately and just share GND)
```

```
cd firmware
pio run -t upload
```
