#ifndef CANSTACK
#define CANSTACK

#include <stdio.h>
#include "pico/stdlib.h"
#include "CAN_util.h"
#include "brushless_drive.h"

#define ENABLE_WATCHDOG_TIMEOUT_US 200000 // set timeout before motor controller is disabled

void CANstack_init();
void CANstack_begin_encoderframe(uint16_t period);
bool CANstack_encoderframe_callback(struct repeating_timer *rt);
uint8_t CANstack_get_encoder_frame_trigger();
void CANstack_set_encoder_frame_trigger(uint8_t newval);
void CANstack_feed_enable_watchdog();
void CANstack_check_enable_watchdog();

#endif