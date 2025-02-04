#ifndef CUST_UTIL
#define CUST_UTIL

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "MCP2515.h"

// MCP2515 TXBUF allocations
#define CAN_TXDATABUF_ENCODERFB 0x1
#define CAN_TXDATABUF_THROTTLEFB 0x3
#define CAN_TXDATABUF_FLEXFB 0x5

#define CAN_RTS_ENCODERFB 0x1
#define CAN_RTS_THROTTLEFB 0x2
#define CAN_RTS_FLEXFB 0x4

// CAN frames
#define CAN_COMMAND_NONE 0        // no command
#define CAN_COMMAND_THROTTLE 1    // set MC throttle
#define CAN_COMMAND_SETENCODER 2  // set MC encoder
#define CAN_COMMAND_SETSETTING 3  // set MC setting
#define CAN_COMMAND_GETUID 4      // get MC UID command
#define CAN_COMMAND_SETFSOFTLIM 5 // set MC forward soft limit
#define CAN_COMMAND_SETRSOFTLIM 6 // set MC reverse soft limit
#define CAN_FB_STATUS 8           // MC status frame
#define CAN_FB_THROTTLE 9         // MC throttle feedback frame
#define CAN_FB_ENCODER 10         // MC encoder feedback frame
#define CAN_FB_UID 12             // MC UID frame
#define CAN_FB_ERROR 13           // MC error frame

// CAN frame sizes
#define CAN_FB_STATUS_SIZE 6
#define CAN_FB_THROTTLE_SIZE 2
#define CAN_FB_ENCODER_SIZE 8
#define CAN_FB_UID_SIZE 8
#define CAN_FB_ERROR_SIZE 2

// CAN settings, receives 3 data bytes. 1 byte defines which setting, 2 bytes sets it.
#define CAN_SETTING_BC 1             // brake/coast mode
#define CAN_SETTING_ILIM 2           // current limit. 0.0733A/LSB
#define CAN_SETTING_VCOMP 3          // voltage comp setting. 8.86mV/LSB
#define CAN_SETTING_MOTINV 4         // motor inversion
#define CAN_SETTING_ENCINV 5         // encoder inverson (reverses direction)
#define CAN_SETTING_FRLIMSW 6        // limit switch en and NO/NC. 0000abcd, a = flim en, b = NO(0)/NC(1), c = rlim en, d = NO/NC
#define CAN_SETTING_FRSOFTLIM 7      // soft limit enable. 000000ab, a = flim en, b = rlim en
#define CAN_SETTING_MOTTEMPLIM 8     // motor temperature limit. 1C/LSB
#define CAN_SETTING_FOLLOWER 9       // MSB enables follower mode, 15 LSB for CAN ID to follow
#define CAN_SETTING_CLEARSTICKY 10    // clear sticky faults
#define CAN_SETTING_SETCANIDFLASH 11 // set CAN ID and commit flash memory
#define CAN_SETTING_FACTRESET 12     // factory reset device to defaults

#define CAN_SETTING_STATUSPER 16      // status frame period in ms
#define CAN_SETTING_THROTTLEFBPER 17  // throttle frame period in ms
#define CAN_SETTING_ENCODERFBPER 18   // encoder frame period in ms
#define CAN_SETTING_ERRORPER 19       // error frame period in ms

#define CAN_ARBID_BASE 0x02080400 // 0x02 (motor controller) 0x08 (FRC team use) 10b API, 6b CAN ID
// 00010 00001000 000001 0000 000000 Type, Manufacturer, Prod ID, Command, CAN ID
// 0 0010 0000 1000 0000 0100 0011 0001
// 02080431
#define CAN_ARBID_HEARTBEAT 0x5
#define CAN_ARBID_DISABLE 0x0

#define CAN_COMMAND_MASK 0x0003A0 // 1111 000000 -> 11 1100 0000 mask for permitting commands

// GPIO and hardware definitions
#define CANSPI_RX 0
#define CANSPI_CS 1
#define CANSPI_SCK 2
#define CANSPI_TX 3
#define CANCLK 4
#define CANCLKWRAP 5 // PWM frequency = SYSCLK / (WRAP + 1)
#define CANCLKDUTY 3

void begin_systick();
void init_spi_dma();
void MCP2515_Init(uint8_t newCANID);
uint8_t MCP2515_write_TXARBID(uint8_t startaddr, uint8_t frame, uint8_t DLC);
uint8_t MCP2515_write_RXARBID(uint8_t startaddr, uint32_t arbid);
uint8_t MCP2515_read_reg(uint8_t addr);
uint8_t MCP2515_read_RXbuffer(uint8_t startbuf, uint8_t *dest_data, uint8_t len);
uint8_t MCP2515_write_bytes(uint8_t startaddr, uint8_t *data, uint8_t len);
uint8_t MCP2515_write_reg(uint8_t addr, uint8_t data);
uint8_t MCP2515_write_bits(uint8_t addr, uint8_t data, uint8_t mask);
uint8_t MCP2515_write_RTS(uint8_t buffers);
uint8_t MCP2515_write_TXdata(uint8_t startbuf, uint8_t *data, uint8_t len);
uint8_t MCP2515_reset();

#endif