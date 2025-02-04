#include "CANstack.h"

uint8_t enabled = 0;
uint32_t enablewatchdog = 0;


uint8_t encoderframetrigger = 0;
uint16_t encoderframeperiod = 10;
struct repeating_timer encoderframetimer;

void CANstack_init() {
    CANstack_begin_encoderframe(5);
    return;
}

void CANstack_begin_encoderframe(uint16_t period) {
    if (!add_repeating_timer_ms(-period, CANstack_encoderframe_callback, NULL, &encoderframetimer))
    {
        busy_wait_ms(1000);
        printf("Failed to add encoder frame timer\r\n");
    }
    enablewatchdog = time_us_32();
}

bool CANstack_encoderframe_callback(struct repeating_timer *rt) {
    encoderframetrigger = 1;
}

uint8_t CANstack_get_encoder_frame_trigger() {
    return encoderframetrigger;
}

void CANstack_set_encoder_frame_trigger(uint8_t newval) {
    encoderframetrigger = newval;
}

void CANstack_feed_enable_watchdog() {
    enablewatchdog = time_us_32();
}

void CANstack_check_enable_watchdog() {
    if(time_us_32() - enablewatchdog > ENABLE_WATCHDOG_TIMEOUT_US) {
        printf("CAN watchdog timeout!\r\n");
        enabled = 0; // disable
    }
}