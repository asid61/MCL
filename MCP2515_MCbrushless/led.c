#include "led.h"

/*
Target behavior of LEDs:

No CAN or PWM: Green off, red slow blink
CAN or PWM present, disabled: Green off, red on
CAN or PWM present at 0 throttle (enabled): Green slow blink, red on if brake mode is on, otherwise red off.
+ throttle: Green blinking fast w/ period like duty cycle
- throttle: Red blinking fast w/ period like duty cycle
forward limit triggered: green and then red in quick succession
reverse limit triggered: red and then green in quick succession
hardware fault: alternating slow green and red

100ms minimum period, 2s overall period -> array should be 20 bytes wide
*/

uint8_t LEDG_STATES[12][20] = { 
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // No CAN or PWM
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // Disabled, coast mode
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // Disabled, brake mode
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Enabled, coast mode
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Enabled, brake mode
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, // low + throttle
    {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0}, // high + throttle
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // low - throttle
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // high - throttle
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // flim
    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0}, // rlim
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // HW fault
};

uint8_t LEDR_STATES[12][20] = { 
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // No CAN or PWM
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Disabled, coast mode
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // Disabled, brake mode
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Enabled, coast mode
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // Enabled, brake mode
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // low + throttle
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // high + throttle
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, // low - throttle
    {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0}, // high - throttle
    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0}, // flim
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // rlim
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}  // HW fault
};

uint8_t ledstatus;
uint8_t ledindex;
uint32_t ledtime;
struct repeating_timer ledtimer;

void LED_Init() 
{
    gpio_init(LEDG_GPIO);                        
    gpio_set_dir(LEDG_GPIO, GPIO_OUT);
    gpio_put(LEDG_GPIO, 0);
    gpio_init(LEDR_GPIO);                        
    gpio_set_dir(LEDR_GPIO, GPIO_OUT);
    gpio_put(LEDR_GPIO, 0);
    ledstatus = 0;
    ledindex = 0;
    ledtime = time_us_32();

    if (!add_repeating_timer_ms(-LED_TICK_TIME, LED_Update, NULL, &ledtimer)) {
        busy_wait_ms(1000);
        printf("Failed to add LED timer\r\n");
    }
}

void LED_SETG(uint8_t state) {
    gpio_put(LEDG_GPIO, state);
}

void LED_SETR(uint8_t state) {
    gpio_put(LEDR_GPIO, state);
}

void LED_Set_Status(uint8_t status) {
    ledstatus = status;
    // ledindex = 0; // this can cause errors when status is rapidly toggled
}

bool LED_Update(struct repeating_timer *rt) {
    ledindex = (ledindex + 1) % 20; // increment the index and wrap to 0
    gpio_put(LEDG_GPIO, LEDG_STATES[ledstatus][ledindex]); // set green LED
    gpio_put(LEDR_GPIO, LEDR_STATES[ledstatus][ledindex]); // set red LED
    return true;
}

