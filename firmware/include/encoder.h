#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Quadrature encoder reader backed by TIM2 hardware encoder mode
   (2x/4x hardware quadrature decoding, no CPU-side ISR needed for
   counting). The timer's 16-bit hardware counter is polled once per
   control tick and the signed delta is folded into a 32-bit running
   total, so wraparound of the hardware counter is transparent. */

void encoder_init(void);

/* Call once per control tick, before reading position/speed, with
   the tick period in seconds. */
void encoder_update(float dt);

void encoder_reset(void);

int32_t encoder_get_position_counts(void);
float encoder_get_speed_counts_per_sec(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
