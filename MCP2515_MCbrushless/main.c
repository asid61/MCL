// Press ctrl+shift+P -> Cmake -> Quick Start to configure!
// Edit CMakeLists with include libs
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "tusb.h"
#include "hardware/watchdog.h"

#include "interrupts.h"
#include "CANstack.h"
#include "led.h"
#include "brushless_drive.h"

extern uint32_t ARBID;

int main()
{
    LED_Init();
    LED_Set_Status(LED_STATE_NOSIGNAL);
    set_sys_clock_khz(120000, true); // 120MHz
    stdio_init_all();
    watchdog_enable(250, 1);
    watchdog_update();

    // Initialize serial port

    begin_systick();
    uint32_t starttime = time_us_32();
    while (!tud_cdc_connected() && (time_us_32() - starttime) < 5000000u)
    {
        busy_wait_ms(10);
        watchdog_update();
    } // wait for usb to come up or wait seconds
    printf("USB at %5d ms\r\n", (time_us_32() - starttime) / 1000);

    init_spi_dma();
    MCP2515_Init(0); // init MCP2515 with CAN ID
    printf("MCP2515 at %5d ms\r\n", (time_us_32() - starttime) / 1000);

    CANstack_init();

    brushless_init();
    brushless_set_encoder(0);

    printf("Setup complete\r\n");
    LED_Set_Status(LED_STATE_ENABLED_BRAKE);

    int64_t txdata = 0;

    // Loop forever
    while (true)
    {
        starttime = time_us_32();
        // txdata++;
        

        // MCP2515_write_TXdata(0x1, tempdata, sizeof(tempdata)); // set data bytes
        // MCP2515_write_RTS(0x1); // trigger send
        //brushless_set_encoder(time_us_64());
        txdata = brushless_get_encoder();
        if (CANstack_get_encoder_frame_trigger() == 1)
        {
            CANstack_set_encoder_frame_trigger(0);
            uint8_t tempdata[8] = {0};
            for (int i = 0; i < 4; i++)
            {
                tempdata[i] = (txdata >> (i << 3)) & 0xff;
            }
            MCP2515_write_TXdata(CAN_TXDATABUF_ENCODERFB, tempdata, sizeof(tempdata));
            MCP2515_write_RTS(CAN_RTS_ENCODERFB);
        }
        

        uint8_t canint = MCP2515_read_reg(MCP2515_CANINTF);
        uint8_t retdata[2];
        int16_t throttle;
        if (canint & MCP2515_CANINTF_RX0IF)
        { // if message in RX0 buffer
            
            MCP2515_write_reg(MCP2515_CANINTF, 0x00); // clear interrupts
            MCP2515_read_RXbuffer(0x2, retdata, 2);
            throttle = (retdata[1] | ((uint16_t)(retdata[0]) << 8)) - 4095;
            // brushless_set_throttle(throttle);
            // printf("throt: %d\r\n", throttle);
        }
        else
        {
            int16_t sigthrottle = ((time_us_32() >> 11) & 0xfff); //0x1fff) - 4096;
            // printf("throt: %d\r\n", sigthrottle);
            brushless_set_throttle(sigthrottle);
            brushless_commutation_callback(true); // start commutation and force
        }


        uint8_t errors = MCP2515_read_reg(MCP2515_EFLG);
        if (errors & 0x1 == 1)
        {
            LED_Set_Status(LED_STATE_ENABLED_COAST);
        }
        else
        {
            LED_Set_Status(LED_STATE_ENABLED_BRAKE);
        }

        printf("%ld\r\n", brushless_get_encoder()); // print encoder for debug

        watchdog_update();
        // printf("loop takes %4dus\r\n", time_us_32() - starttime);
        busy_wait_ms(10);
        
    }
}