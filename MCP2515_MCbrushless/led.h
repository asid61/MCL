#ifndef LED
#define LED

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/structs/systick.h"

#define LEDG_GPIO 24
#define LEDR_GPIO 25

#define LED_TICK_TIME 50 // 50ms tick

#define LED_STATE_NOSIGNAL 0
#define LED_STATE_DISABLED_COAST 1
#define LED_STATE_DISABLED_BRAKE 2
#define LED_STATE_ENABLED_COAST 3
#define LED_STATE_ENABLED_BRAKE 4
#define LED_STATE_LOWFTHROTTLE 5
#define LED_STATE_HIGHFTHROTTLE 6
#define LED_STATE_LOWRTHROTTLE 7
#define LED_STATE_HIGHRTHROTTLE 8
#define LED_STATE_FLIM 9
#define LED_STATE_RLIM 10
#define LED_STATE_HWFAULT 11

void LED_Init();
void LED_SETG(uint8_t state);
void LED_SETR(uint8_t state);
void LED_Set_Status(uint8_t status);
bool LED_Update(struct repeating_timer *rt);

#endif