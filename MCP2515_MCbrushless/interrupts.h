#ifndef INTERRUPTS
#define INTERRUPTS
#include <stdio.h>
#include "pico/stdlib.h"
#include "brushless_drive.h"

void gpio_interrupt_callback(uint gpio, uint32_t events);

#endif