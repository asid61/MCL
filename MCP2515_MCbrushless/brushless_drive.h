#ifndef BRUSHLESSDRIVE
#define BRUSHLESSDRIVE

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include "hardware/pwm.h"
#include "MCP2515.h"
#include "interrupts.h"

#define AH 15
#define AL 14
#define BH 17
#define BL 16
#define CH 19
#define CL 18

#define HALLA 8
#define HALLB 10
#define HALLC 12

#define BRUSHED_PWM_H_ON 4096
#define BRUSHED_PWM_H_OFF 0
#define BRUSHED_PWM_L_ON 0
#define BRUSHED_PWM_L_OFF 4096

#define BRUSHED_PWM_CLK_DIV 2 // divide clock down
#define BRUSHED_PWM_CLK_WRAP 4095  // PWM frequency = PWMCLK / (WRAP + 1)

void brushless_init();
void init_hall_gpio();
void brushless_slice_init(uint slice, uint chanHgpio, uint chanLgpio);
void brushless_set_throttle(int16_t throttle);
void brushless_set_encoder(int64_t val);
long brushless_get_encoder();
void brushless_calculate_encoder(int laststate, int newstate);
uint8_t brushless_get_hall_states();
void brushless_set_hbridge(uint8_t lowstate, uint8_t highstate, uint slice, uint tempthrottle);
uint8_t brushless_commutation_callback(bool force);

#endif
