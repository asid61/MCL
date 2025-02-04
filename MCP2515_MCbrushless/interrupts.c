#include "interrupts.h"


void gpio_interrupt_callback(uint gpio, uint32_t events) {
    if(gpio == HALLA || gpio == HALLB || gpio == HALLC) {
        brushless_commutation_callback(false);
    }
}

