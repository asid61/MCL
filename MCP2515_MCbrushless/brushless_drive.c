#include "brushless_drive.h"

uint PA_slice;
uint AH_chan;
uint AL_chan; // low channels are inverted -> 0 means on

uint PB_slice;
uint BH_chan;
uint BL_chan; 

uint PC_slice;
uint CH_chan;
uint CL_chan; 

uint HALLA_mask = 0x1 << HALLA;
uint HALLB_mask = 0x1 << HALLB;
uint HALLC_mask = 0x1 << HALLC;

long encodervalue;

int lastphase;
int16_t throttle;
uint8_t brakemode; // if 1, brake mode engaged


// hall phase interpretations from https://lucidar.me/en/actuators/commutation-for-bldc-motors/
// converts the ABC hall bits into commutation values for forward rotation and supplies which commutation phase
// passing in ADB to the first index will return the 6 commutation settings and the # of the phase
// Q1L, Q1H, Q2L, Q2H, Q3L, Q3H, phase number
uint8_t hall_phase_state_FWD[8][7] = {
    {0, 0, 0, 0, 0, 0, 255}, // commutation error
    {0, 1, 0, 0, 1, 0, 2}, // phase 2
    {1, 0, 0, 1, 0, 0, 4}, // phase 4
    {0, 0, 0, 1, 1, 0, 3}, // phase 3
    {0, 0, 1, 0, 0, 1, 6}, // phase 6
    {0, 1, 1, 0, 0, 0, 1}, // phase 1
    {1, 0, 0, 0, 0, 1, 5}, // phase 5
    {0, 0, 0, 0, 0, 0, 255}, // commutation error
};

// same as above, but H and L states are reversed
uint8_t hall_phase_state_REV[8][7] = {
    {0, 0, 0, 0, 0, 0, 255}, // commutation error
    {1, 0, 0, 0, 0, 1, 2}, // phase 2
    {0, 1, 1, 0, 0, 0, 4}, // phase 4
    {0, 0, 1, 0, 0, 1, 3}, // phase 3
    {0, 0, 0, 1, 1, 0, 6}, // phase 6
    {1, 0, 0, 1, 0, 0, 1}, // phase 1
    {0, 1, 0, 0, 1, 0, 5}, // phase 5
    {0, 0, 0, 0, 0, 0, 255}, // commutation error
};

void brushless_init() {
    encodervalue = 0;
    lastphase = 1;

    init_hall_gpio(HALLA);
    init_hall_gpio(HALLB);
    init_hall_gpio(HALLC);

    PA_slice = pwm_gpio_to_slice_num(AH);
    AH_chan = pwm_gpio_to_channel(AH);
    AL_chan = pwm_gpio_to_channel(AL);
    brushless_slice_init(PA_slice, AH, AL);

    PB_slice = pwm_gpio_to_slice_num(BH);
    BH_chan = pwm_gpio_to_channel(BH);
    BL_chan = pwm_gpio_to_channel(BL);
    brushless_slice_init(PB_slice, BH, BL);

    PC_slice = pwm_gpio_to_slice_num(CH);
    CH_chan = pwm_gpio_to_channel(CH);
    CL_chan = pwm_gpio_to_channel(CL);
    brushless_slice_init(PC_slice, CH, CL);

    brushless_set_throttle(0);
}

void init_hall_gpio(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_pulls(pin, true, false);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_interrupt_callback);
}

void brushless_slice_init(uint slice, uint chanHgpio, uint chanLgpio) {
    uint chanH = pwm_gpio_to_channel(chanHgpio);
    uint chanL = pwm_gpio_to_channel(chanLgpio);
    pwm_config brushless_pwm_config = pwm_get_default_config(); // wraps at 0xffff with system clock speed
    pwm_config_set_output_polarity(&brushless_pwm_config, false, false); // invert L channel ()
    pwm_config_set_clkdiv_int(&brushless_pwm_config, 2); // set clock divider
    pwm_config_set_wrap(&brushless_pwm_config, BRUSHED_PWM_CLK_WRAP);     // set PID period
    pwm_init(slice, &brushless_pwm_config, false);
    gpio_set_function(chanHgpio, GPIO_FUNC_PWM);
    gpio_set_function(chanLgpio, GPIO_FUNC_PWM);
    pwm_set_chan_level(slice, chanH, BRUSHED_PWM_H_OFF);
    pwm_set_chan_level(slice, chanL, BRUSHED_PWM_L_OFF);
    pwm_set_enabled(slice, true);
}

void brushless_set_throttle(int16_t newthrottle) {
    throttle = newthrottle;
    brushless_commutation_callback(true);
    return;
}


void brushless_set_encoder(int64_t val) {
    encodervalue = val;
}

long brushless_get_encoder() {
    return encodervalue;
}

void brushless_calculate_encoder(int laststate, int newstate) {
    int difference = newstate - laststate;
    if(difference > 4) {difference = 4 - difference;}
    if(difference < -4) {difference = -4 - difference;}
    encodervalue += difference;
}

uint8_t brushless_get_hall_states() {
    // uint8_t pinstates = gpio_get(HALLA) << 2 | gpio_get(HALLB) << 1 | gpio_get(HALLC); // working but risky
    uint gpiostates = sio_hw->gpio_in;
    // !! converts to boolean, e.g. !!(42) = 1, and !!(0) = 0.
    uint8_t pinstates = ((!!((1ul << HALLA) & gpiostates)) << 2) | ((!!((1ul << HALLB) & gpiostates)) << 1) | (!!((1ul << HALLC) & gpiostates));
    return pinstates;
}

void brushless_set_hbridge(uint8_t lowstate, uint8_t highstate, uint slice, uint tempthrottle) {
    if(highstate) {
        pwm_set_both_levels(slice, tempthrottle, tempthrottle);
    } else if(lowstate) {
        pwm_set_both_levels(slice, BRUSHED_PWM_L_ON, BRUSHED_PWM_H_OFF);
    } else pwm_set_both_levels(slice, BRUSHED_PWM_L_OFF, BRUSHED_PWM_H_OFF);
}

void brushless_set_off() {
    if(brakemode) {
        pwm_set_chan_level(PA_slice, AH_chan, BRUSHED_PWM_H_OFF);
        pwm_set_chan_level(PA_slice, AL_chan, BRUSHED_PWM_L_ON);
        pwm_set_chan_level(PB_slice, BH_chan, BRUSHED_PWM_H_OFF);
        pwm_set_chan_level(PB_slice, BL_chan, BRUSHED_PWM_L_ON);
        pwm_set_chan_level(PC_slice, CH_chan, BRUSHED_PWM_H_OFF);
        pwm_set_chan_level(PC_slice, CL_chan, BRUSHED_PWM_L_ON);

    } else {
        pwm_set_chan_level(PA_slice, AH_chan, BRUSHED_PWM_H_OFF);
        pwm_set_chan_level(PA_slice, AL_chan, BRUSHED_PWM_L_OFF);
        pwm_set_chan_level(PB_slice, BH_chan, BRUSHED_PWM_H_OFF);
        pwm_set_chan_level(PB_slice, BL_chan, BRUSHED_PWM_L_OFF);
        pwm_set_chan_level(PC_slice, CH_chan, BRUSHED_PWM_H_OFF);
        pwm_set_chan_level(PC_slice, CL_chan, BRUSHED_PWM_L_OFF);
    }
}

// called for commutation update. force is set to true when commutation is forced (e.g. via throttle setting)
uint8_t brushless_commutation_callback(bool force) { 
    int hallphase = 0;
    uint8_t pinstates;
    pinstates = brushless_get_hall_states();
    hallphase = hall_phase_state_FWD[pinstates][6];
    if((lastphase == hallphase || hallphase == 255) && !force) return hallphase; // if bad trigger return, unless forced to continue
    brushless_calculate_encoder(lastphase, hallphase); // update encoder
    lastphase = hallphase; // reset phase tracker
    
    if(throttle > 0 && throttle < 4096) {
        brushless_set_hbridge(hall_phase_state_FWD[pinstates][0], hall_phase_state_FWD[pinstates][1], PA_slice, throttle);
        brushless_set_hbridge(hall_phase_state_FWD[pinstates][2], hall_phase_state_FWD[pinstates][3], PB_slice, throttle);
        brushless_set_hbridge(hall_phase_state_FWD[pinstates][4], hall_phase_state_FWD[pinstates][5], PC_slice, throttle);
        return 0;
    }
    if (throttle < 0 && throttle > -4096) {
        brushless_set_hbridge(hall_phase_state_REV[pinstates][0], hall_phase_state_FWD[pinstates][1], PA_slice, -throttle);
        brushless_set_hbridge(hall_phase_state_REV[pinstates][2], hall_phase_state_FWD[pinstates][3], PB_slice, -throttle);
        brushless_set_hbridge(hall_phase_state_REV[pinstates][4], hall_phase_state_FWD[pinstates][5], PC_slice, -throttle);
        return 0;
    }

    brushless_set_off(); // if others fail, turn off motor.
    return 0;
}

