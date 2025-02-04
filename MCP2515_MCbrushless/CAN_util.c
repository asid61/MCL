#include "CAN_util.h"


uint dma_tx;
uint dma_rx;
uint8_t dma_txbuf[12] = {0};
uint8_t dma_rxbuf[12] = {0};
dma_channel_config txconfig;
dma_channel_config rxconfig;

spi_inst_t *canspi = spi0;
uint8_t CANID = 0;

void begin_systick()
{
    systick_hw->csr = 0x5; // start systick
    systick_hw->rvr = 0x00FFFFFF;
    printf("systick started\r\n");
}

void init_spi_dma()
{
     // Enable SPI at 10 MHz and connect to GPIOs
    spi_init(canspi, 10000 * 1000);
    gpio_set_function(CANSPI_RX, GPIO_FUNC_SPI); // RX
    gpio_init(CANSPI_CS);                        // CS
    gpio_set_dir(CANSPI_CS, GPIO_OUT);
    gpio_put(CANSPI_CS, true); // start CS high
    gpio_set_function(CANSPI_SCK, GPIO_FUNC_SPI); // SCK
    gpio_set_function(CANSPI_TX, GPIO_FUNC_SPI);  // TX

    dma_tx = dma_claim_unused_channel(true);
    dma_rx = dma_claim_unused_channel(true);

    txconfig = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&txconfig, DMA_SIZE_8);
    channel_config_set_dreq(&txconfig, spi_get_dreq(canspi, true));
    dma_channel_configure(dma_tx, &txconfig,
                          &spi_get_hw(canspi)->dr, // write address
                          dma_txbuf, // read address
                          sizeof(dma_txbuf), // element count (each element is of size transfer_data_size)
                          false); // don't start yet

    rxconfig = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&rxconfig, DMA_SIZE_8);
    channel_config_set_dreq(&rxconfig, spi_get_dreq(canspi, false));
    channel_config_set_read_increment(&rxconfig, false);
    channel_config_set_write_increment(&rxconfig, true);
    dma_channel_configure(dma_rx, &rxconfig,
                          dma_rxbuf, // write address
                          &spi_get_hw(canspi)->dr, // read address
                          sizeof(dma_rxbuf), // element count (each element is of size transfer_data_size)
                          false); // don't start yet

    // dma_channel_start(dma_tx); // command to start a DMA transfer

}

void MCP2515_Init(uint8_t newCANID)
{   
    CANID = newCANID;

    const int can_clk_slice = pwm_gpio_to_slice_num(CANCLK);
    const int can_clk_chan = pwm_gpio_to_channel(CANCLK);
    pwm_config can_clk_config = pwm_get_default_config(); // wraps at 0xffff with system clock speed
    pwm_config_set_wrap(&can_clk_config, CANCLKWRAP);     // set PID period
    pwm_init(can_clk_slice, &can_clk_config, false);
    gpio_set_function(CANCLK, GPIO_FUNC_PWM);
    pwm_set_chan_level(can_clk_slice, can_clk_chan, CANCLKDUTY);
    pwm_set_enabled(can_clk_slice, true);

    busy_wait_ms(2); // wait for MCP2515 to startup

    uint8_t tempdata = 0;

    MCP2515_reset();

    printf("CAN reset\r\n");
    
    MCP2515_write_reg(MCP2515_CANCTRL, 0x80); // put MCP2515 into configuration mode and disable CLKOUT
    tempdata = (0x0 << 6) | 0x00; // jump width | baud rate prescaler. See section 5.5 in MCP2515 datasheet. TQ = 2 x (BRP[5:0] + 1)/FOSC
    // bit time is 1us, so 10Tq per bit time (0.1us). Syncseg = 1, Propseg = 2, PS1 = 3, PS2 = 3
    MCP2515_write_reg(MCP2515_CNF1, tempdata);
    tempdata = (0x1 << 7) | (0x3 << 3) | 0x1; // PS2 source, PS1, PROPSEG. 1 is added to both PS1 and PROPSEG.
    MCP2515_write_reg(MCP2515_CNF2, tempdata);
    MCP2515_write_reg(MCP2515_CNF3, (uint8_t) 0x2); // PS2. 1 is added.

    printf("CAN configs written\r\n");

    MCP2515_write_bits(MCP2515_TXB0CTRL, 0x2, MCP2515_TXBCTRL_TXP0 | MCP2515_TXBCTRL_TXP1); // set write priority of TX0 to medium
    MCP2515_write_bits(MCP2515_TXB1CTRL, 0x1, MCP2515_TXBCTRL_TXP0 | MCP2515_TXBCTRL_TXP1); // set write priority of TX1 to low
    MCP2515_write_bits(MCP2515_TXB2CTRL, 0x1, MCP2515_TXBCTRL_TXP0 | MCP2515_TXBCTRL_TXP1); // set write priority of TX2 to low

    MCP2515_write_TXARBID(MCP2515_TXB0SIDH, CAN_FB_ENCODER, CAN_FB_ENCODER_SIZE);
    MCP2515_write_TXARBID(MCP2515_TXB1SIDH, CAN_FB_THROTTLE, CAN_FB_THROTTLE_SIZE);
    MCP2515_write_TXARBID(MCP2515_TXB2SIDH, CAN_FB_STATUS, CAN_FB_STATUS_SIZE); // flex, used for status, UID, and errors 

    MCP2515_write_RXARBID(MCP2515_RXF0SIDH, 0x00000000u); // write disable signal into buffer 0
    MCP2515_write_RXARBID(MCP2515_RXF1SIDH, 0xffffffffu); // unused rx buffers only accept fake id
    MCP2515_write_RXARBID(MCP2515_RXM0SIDH, 0xffffffffu); // set mask 0 for broadcast messages - all 0s is disable.
    MCP2515_write_RXARBID(MCP2515_RXF2SIDH, CAN_ARBID_BASE | CANID); // write all device-specific messages into buffer 1
    MCP2515_write_RXARBID(MCP2515_RXF3SIDH, 0xffffffffu); // unused rx buffers only accept fake arbid
    MCP2515_write_RXARBID(MCP2515_RXF4SIDH, 0xffffffffu);
    MCP2515_write_RXARBID(MCP2515_RXF5SIDH, 0xffffffffu);
    MCP2515_write_RXARBID(MCP2515_RXM1SIDH, 0xffff003fu); // set mask 1 to accept any API class and index

    MCP2515_write_reg(MCP2515_BFPCTRL, 0x0f); // set RXnBF pins to output when the Rx buffer receives a new message
    MCP2515_write_bits(MCP2515_RXB0CTRL, 0x00, MCP2515_RXB0CTRL_BUKT | MCP2515_RXB0CTRL_RXM0 | MCP2515_RXB0CTRL_RXM1); // enable RX0
    MCP2515_write_bits(MCP2515_RXB1CTRL, 0x00, MCP2515_RXB1CTRL_RXM0 | MCP2515_RXB1CTRL_RXM1); // enable RX1

    MCP2515_write_reg(MCP2515_CANINTE, 0x03); // only enable interrupt for when rx buffer full
    MCP2515_write_reg(MCP2515_CANINTF, 0x00); // clear interrupts

    MCP2515_write_reg(MCP2515_CANCTRL, (uint8_t) 0x00); // put MCP2515 into normal mode with disabled CLKOUT
    printf("MCP2515 setup complete\r\n");
}

uint8_t MCP2515_write_TXARBID(uint8_t startaddr, uint8_t frame, uint8_t DLC) {
    uint8_t data[5];
    uint32_t ARBID;
    uint8_t byteswritten;
    ARBID = CAN_ARBID_BASE | CANID | (frame << 6);
    data[0] = (ARBID >> 21) & 0xff;  // SIDH
    data[1] = ((ARBID >> 16) & 0x3U) | MCP2515_RX_IDE | (((ARBID >> 18) & 0x7) << 5);  // SIDL
    data[2] = (ARBID >> 8) & 0xff;  // EID8
    data[3] = ARBID & 0xff;  // EID0
    data[4] = DLC & 0x0F;  // DLC
    byteswritten = MCP2515_write_bytes(startaddr, data, sizeof(data)); // write arbID + DLC into memory
    return byteswritten;
}

// write filter arbid for RX buffers. Filter 0,1 go to RX0 and 2,3,4,5 go to RX1.
uint8_t MCP2515_write_RXARBID(uint8_t startaddr, uint32_t arbid) {
    uint8_t data[4];
    uint32_t ARBID;
    uint8_t byteswritten;
    ARBID = arbid;
    data[0] = (ARBID >> 21) & 0xff;  // SIDH
    data[1] = ((ARBID >> 16) & 0x3U) | MCP2515_RX_IDE | (((ARBID >> 18) & 0x7) << 5);  // SIDL
    data[2] = (ARBID >> 8) & 0xff;  // EID8
    data[3] = ARBID & 0xff;  // EID0
    byteswritten = MCP2515_write_bytes(startaddr, data, sizeof(data)); // write arbID + DLC into memory
    return byteswritten;
}

uint8_t MCP2515_read_reg(uint8_t addr)
{
    // needs to process 24 bits - 8 bits of command, 8 bits of register, 8 bits of rx data
    uint8_t writebuf[2];
    uint8_t readbuf[1];
    writebuf[0] = MCP2515_SPI_READ; // read command
    writebuf[1] = addr;             // register to read
    readbuf[0] = 0;

    sio_hw->gpio_clr = 1ul << CANSPI_CS; // bring cs low
    spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    spi_read_blocking(canspi, 0, readbuf, sizeof(readbuf));
    sio_hw->gpio_set = 1ul << CANSPI_CS; // bring cs high
    uint readval = readbuf[0];
    return readval;
}

uint8_t MCP2515_read_RXbuffer(uint8_t startbuf, uint8_t* dest_data, uint8_t len) // read rx data srtarting from startbuf into dest_data
{
    // read data starting at certain points. 000 = RXB0SIDH, 010 = RXB0D0, 100 = RXB1SIDH, 110 = RXB1D0
    uint8_t writebuf[1];
    writebuf[0] = MCP2515_SPI_READ_RXBUF | startbuf;

    sio_hw->gpio_clr = 1ul << CANSPI_CS; // bring cs low
    spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    spi_read_blocking(canspi, 0, dest_data, len);
    sio_hw->gpio_set = 1ul << CANSPI_CS; // bring cs high
    return 0;
}

uint8_t MCP2515_write_bytes(uint8_t startaddr, uint8_t* data, uint8_t len) {
    uint byteswritten = 0;
    uint8_t writebuf[2];
    writebuf[0] = MCP2515_SPI_WRITE; // write command
    writebuf[1] = startaddr;         // register to start write from

    sio_hw->gpio_clr = 1ul << CANSPI_CS;
    byteswritten = spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    byteswritten = spi_write_blocking(canspi, data, len);
    sio_hw->gpio_set = 1ul << CANSPI_CS;
}

uint8_t MCP2515_write_reg(uint8_t addr, uint8_t data)
{
    uint byteswritten = 0;
    uint8_t writebuf[3];
    writebuf[0] = MCP2515_SPI_WRITE; // write command
    writebuf[1] = addr;              // register to write
    writebuf[2] = data;

    sio_hw->gpio_clr = 1ul << CANSPI_CS;
    byteswritten = spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    sio_hw->gpio_set = 1ul << CANSPI_CS;
    return byteswritten;
}

uint8_t MCP2515_write_bits(uint8_t addr, uint8_t data, uint8_t mask)
{
    uint byteswritten = 0;
    uint8_t writebuf[4];
    writebuf[0] = MCP2515_SPI_BITMOD;   // bit modification command
    writebuf[1] = addr;                 // register to write
    writebuf[2] = mask;                 // bitmask (1 = write 0 = ignore)
    writebuf[3] = data;                 // bits

    sio_hw->gpio_clr = 1ul << CANSPI_CS;
    byteswritten = spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    sio_hw->gpio_set = 1ul << CANSPI_CS;
    return byteswritten;
}

uint8_t MCP2515_write_RTS(uint8_t buffers) // write ready-to-send on buffers. 0x1 is buffer 0, 0x2 is buffer 1, 0x4 is buffer 2. Can be OR'd.
{
    uint8_t byteswritten = 0;
    uint8_t writebuf[1];
    writebuf[0] = MCP2515_SPI_RTS | buffers; // write command
    sio_hw->gpio_clr = 1ul << CANSPI_CS;
    byteswritten = spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    sio_hw->gpio_set = 1ul << CANSPI_CS;
    return byteswritten;
}

uint8_t MCP2515_write_TXdata(uint8_t startbuf, uint8_t* data, uint8_t len) // write TX data starting from one of six addresses
{
    // 3 LSB are used to determine starting address. 
    // 001 = TXB0D0,   011 = TXB1D0,   101 = TXB2D0
    // 000 = TXB0SIDH, 010 = TXB1SIDH, 100 = TXB2SIDH
    
    uint8_t byteswritten = 0;
    uint8_t writebuf[1];
    writebuf[0] = MCP2515_SPI_LOAD_TXBUF | startbuf; // write command
    sio_hw->gpio_clr = 1ul << CANSPI_CS; // bring cs low
    byteswritten = spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    byteswritten = spi_write_blocking(canspi, data, len);
    sio_hw->gpio_set = 1ul << CANSPI_CS; // bring cs high
    return byteswritten;
}

uint8_t MCP2515_reset() // resets MCP2515 to defaults
{
    uint byteswritten = 0;
    uint8_t writebuf[1];
    writebuf[0] = MCP2515_SPI_RESET; // write command

    sio_hw->gpio_clr = 1ul << CANSPI_CS; // bring cs low
    byteswritten = spi_write_blocking(canspi, writebuf, sizeof(writebuf));
    sio_hw->gpio_set = 1ul << CANSPI_CS; // bring cs high
    return byteswritten;
}