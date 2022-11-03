#include "AIC3254.h"

void AIC3254::init_i2c(void)
{
    i2c_port_t i2c_master_port = (i2c_port_t)I2C_PORT_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

void AIC3254::set_register(uint8_t register_address, uint8_t set_value)
{
    register_address = register_address & 0x007F; // register bit mask to limit register address from 0 to 127 (see application reference guide p.94)

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                     // send start bit
    i2c_master_write_byte(cmd, (AIC3254_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN); // aic3254 7-bit address + write bit
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);                // target register
    i2c_master_write_byte(cmd, set_value, ACK_CHECK_EN);                       // target value
    i2c_master_stop(cmd);                                                      // send stop bit
    i2c_master_cmd_begin((i2c_port_t)I2C_PORT_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

uint8_t AIC3254::read_register(uint8_t register_address)
{
    uint8_t data = 0xFF;                          // initialize the buffer to 255
    register_address = register_address & 0x007F; // register bit mask to limit register address from 0 to 127 (see application reference guide p.94)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                     // send start bit
    i2c_master_write_byte(cmd, (AIC3254_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN); // aic3254 7-bit address + write bit
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);                // register to be read
    i2c_master_start(cmd);                                                     // resend start bit (see application reference guide p.80 for more info on the i2c transaction)
    i2c_master_write_byte(cmd, (AIC3254_ADDR << 1) | READ_BIT, ACK_CHECK_EN);  // aic3254 7-bit address + read bit
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);                         // read into data buffer
    i2c_master_stop(cmd);                                                      // send stop bit
    i2c_master_cmd_begin((i2c_port_t)I2C_PORT_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return data;
}

void AIC3254::test_register(uint8_t register_address, uint8_t expected_value)
{
    uint8_t read_value = read_register(register_address);

    if (read_value == expected_value)
    {
        printf("[PASS] R_0x%X -> EXPECTED: 0x%X. ACTUAL: 0x%X.\n", register_address, expected_value, read_value);
    }
    else
    {
        printf("[ERROR] R_0x%X -> EXPECTED: 0x%X. ACTUAL: 0x%X.\n", register_address, expected_value, read_value);
    }
}

void AIC3254::init_demo(void)
{
    uint8_t gain = 0x00; // input gain set to +0db gain

    // issue a soft reset
    set_register(0x00, 0x00);            // (P0_R0) go to page 0 on register map
    set_register(0x01, 0x01);            // (P0_R1) issue a software reset to the codec
    vTaskDelay(10 / portTICK_PERIOD_MS); // wait for device to initialize registers
    set_register(0x00, 0x01);            // (P1_R0) point to page 1 on register map
    set_register(0x01, 0x08);            // (P1_R1) disable crude AVDD generation from DVDD [0b00001000]
    set_register(0x02, 0x01);            // (P1_R2) enable analog block, AVDD LDO powered up [0b00000001]
    // configure PLL and clocks
    set_register(0x00, 0x00); // (P0_R0) go to page 0 on regiter map
    set_register(0x1B, 0x31); // (P0_R27) enable I2S, 32 bit depth, set BCLK and WCLK as inputs to AIC3254 (target) [0b00110001]
    set_register(0x1C, 0x00); // (P0_R28) set data offset to 0 BCLKs
    set_register(0x04, 0x03); // (P0_R4) config PLL settings: Low PLL clock range, MCLK -> PLL_CLKIN, PLL_CLK -> CODEC_CLKIN [0b00000011]
    set_register(0x06, 0x07); // (P0_R6) set J = 7 [0b00000111]
    set_register(0x07, 0x00); // (P0_R7) set D = 0 (MSB)
    set_register(0x08, 0x00); // (P0_R8) set D = 0 (LSB)
    // for 32 bit clocks per frame in Controller mode ONLY
    set_register(0x1E, 0x08); // (P0_R30) set BCKL N divider to 8. BLCK = DAC_CLK/N = 12.288 MHz/8 = 32*fs = 1.536 MHz
    set_register(0x05, 0x91); // (P0_R5) Power up PLL, set P = 1 and R = 1 [0b10010001]
    set_register(0x0D, 0x00); // (P0_R13) set DOSR = 128 (MSB), hex 0x0080, DAC Oversampling
    set_register(0x0E, 0x80); // (P0_R14) set DOSR = 128 (LSB)
    set_register(0x14, 0x80); // (P0_R20) set AOSR = 128 decimal or 0x0080 hex, for decimation filters 1 to 6, ADC Oversampling
    set_register(0x0B, 0x87); // (P0_R11) power up and set NDAC = 7 [0b10000111]
    set_register(0x0C, 0x82); // (P0_R12) power up and set MDAC = 2 [0b10000010]
    set_register(0x12, 0x87); // (P0_R18) power up and set NADC = 7 [0b10000111]
    set_register(0x13, 0x82); // (P0_R19) power up and set MADC = 2 [0b10000010]

    // DAC routing and power up
    set_register(0x00, 0x01); // (P1_R0) point to page 1 on register map
    set_register(0x0E, 0x08); // (P1_R14) LDAC AFIR routed to LOL [0b00001000]
    set_register(0x0F, 0x08); // (P1_R15) RDAC AFIR routed to LOR [0b00001000]
    set_register(0x0C, 0x08); // (P1_R12) LDAC AFIR routed to HPL [0b00001000]
    set_register(0x0D, 0x08); // (P1_R13) RDAC AFIR routed to HPR [0b00001000]

    set_register(0x00, 0x00); // (P0_R0) point to page 0 on register map
    set_register(0x40, 0x02); // (P0_R64) DAC left volume = right volumen [0b00000010]
    set_register(0x41, 0x00); // (P0_R65) set left DAC gain to 0dB DIGITAL VOL
    set_register(0x3F, 0xD4); // (P0_R63) Power up left and right DAC data paths and set channel [0b11010100]
    set_register(0x00, 0x01); // (P1_01) point to page 1 on register map
    set_register(0x09, 0x3C); // (P1_R9) power up HPL, HPR, LOL and LOR [0b00111100]
    set_register(0x10, 0x06); // (P1_R16) unmute HPL, set +6dB gain [0b00001010]
    set_register(0x11, 0x06); // (P1_R17) unmute HPR, set +6db gain [0b00001010]
    set_register(0x0A, 0x08); // (P1_R10) output common mode for LOL and LOR is 1.65 from LDOIN (= Vcc / 2) [0b00001000]
    set_register(0x12, 0x0A); // (P1_R18) unmute LOL, set +10dB gain [0b00001010]
    set_register(0x13, 0x0A); // (P1_R19) unmute LOR, set +10dB gain [0b00001010]

    // ADC routing and power up
    set_register(0x00, 0x01);            // (P1_R0) point to page 1 on register mao
    set_register(0x34, 0x50);            // (P1_R52) Route IN1L and IN2L to Left MICPGA with 10kohm resistance [0b01010000]
    set_register(0x37, 0x50);            // (P1_R55) Route IN1R and IN2R to RIght MICPGA with 10kohm resistance [0b01010000]
    set_register(0x36, 0x01);            // (P1_R54) CM is routed to Left MICPGA via CM2L with 10kohm resistance
    set_register(0x39, 0x40);            // (P1_R57) CM is routed to Right MICPGA via CM1R with 10kohm resistance [0b01000000]
    set_register(0x3B, gain);            // (P1_R59) Unmute left MICPGA  and set its gain
    set_register(0x3C, gain);            // (P1_R60) Unmute right MICPGA and set its gain
    set_register(0x00, 0x00);            // (P0_R0) point to page 0 on register map
    set_register(0x51, 0xC0);            // (P0_R81) power up left and right ADCs
    set_register(0x52, 0x00);            // (P0_R82) unmute left and right ADCs
    set_register(0x00, 0x00);            // (P0_R0) point to page 0 on register map
    vTaskDelay(10 / portTICK_PERIOD_MS); // wait for device to initialize registers
}

void AIC3254::debug_registers(void)
{
    uint8_t gain = 0x06; // input gain set to +3 dB

    set_register(0x00, 0x01); // (P1_R0) jump to page 1 on register map
    printf("===== PAGE 1 =====\n");

    test_register(0x01, 0x08); // (P1_R1) disable crude AVDD generation from DVDD [0b00001000]
    test_register(0x02, 0x01); // (P1_R2) enable analog block, AVDD LDO powered up [0b00000001]

    // configure PLL and clocks
    set_register(0x00, 0x00); // (P0_R0) go to page 0 on regiter map
    printf("===== PAGE 0 =====\n");

    test_register(0x1B, 0x31); // (P0_R27) enable I2S, 32 bit depth, set BCLK and WCLK as outputs to AIC3254 (target) [0b00100001]
    test_register(0x1C, 0x00); // (P0_R28) set data offset to 0 BCLKs
    test_register(0x04, 0x03); // (P0_R4) config PLL settings: Low PLL clock range, MCLK -> PLL_CLKIN, PLL_CLK -> CODEC_CLKIN [0b00000011]
    test_register(0x06, 0x07); // (P0_R6) set J = 7 [0b00000111]
    test_register(0x07, 0x00); // (P0_R7) set D = 0 (MSB)
    test_register(0x08, 0x00); // (P0_R8) set D = 0 (LSB)
    // for 32 bit clocks per frame in Controller mode ONLY
    test_register(0x1E, 0x08); // (P0_R30) set BCKL N divider to 8. BLCK = DAC_CLK/N = 12.288 MHz/8 = 32*fs = 1.536 MHz
    test_register(0x05, 0x91); // (P0_R5) Power up PLL, set P = 1 and R = 1 [0b10010001]
    test_register(0x0D, 0x00); // (P0_R13) set DOSR = 128 (MSB), hex 0x0080, DAC Oversampling
    test_register(0x0E, 0x80); // (P0_R14) set DOSR = 128 (LSB)
    test_register(0x14, 0x80); // (P0_R20) set AOSR = 128 decimal or 0x0080 hex, for decimation filters 1 to 6, ADC Oversampling
    test_register(0x0B, 0x87); // (P0_R11) power up and set NDAC = 7 [0b10000111]
    test_register(0x0C, 0x82); // (P0_R12) power up and set MDAC = 2 [0b10000010]
    test_register(0x12, 0x87); // (P0_R18) power up and set NADC = 7 [0b10000111]
    test_register(0x13, 0x82); // (P0_R19) power up and set MADC = 2 [0b10000010]

    // DAC routing and power up
    set_register(0x00, 0x01); // (P1_R0) point to page 1 on register map
    printf("===== PAGE 1 =====\n");

    test_register(0x0E, 0x08); // (P1_R14) LDAC AFIR routed to LOL [0b00001000]
    test_register(0x0F, 0x08); // (P1_R15) RDAC AFIR routed to LOR [0b00001000]
    test_register(0x0C, 0x08); // (P1_R12) LDAC AFIR routed to HPL [0b00001000]
    test_register(0x0D, 0x08); // (P1_R13) RDAC AFIR routed to HPR [0b00001000]

    set_register(0x00, 0x00); // (P0_R0) point to page 0 on register map
    printf("===== PAGE 0 =====\n");

    test_register(0x40, 0x02); // (P0_R64) DAC left volume = right volumen [0b00000010]
    test_register(0x41, 0x00); // (P0_R65) set left DAC gain to 0dB DIGITAL VOL
    test_register(0x3F, 0xD4); // (P0_R63) Power up left and right DAC data paths and set channel [0b11010100]

    set_register(0x00, 0x01); // (P1_01) point to page 1 on register map
    printf("===== PAGE 1 =====\n");

    test_register(0x09, 0x3C); // (P1_R9) power up HPL and HPR [0b00111100]
    test_register(0x10, 0x00); // (P1_R16) unmute HPL, set 0dB gain [0b00001010]
    test_register(0x11, 0x00); // (P1_R17) unmute HPR, set 0db gain [0b00001010]
    test_register(0x0A, 0x08); // (P1_R10) output common mode for LOL and LOR is 1.65 from LDOIN (= Vcc / 2) [0b00001000]
    test_register(0x12, 0x0F); // (P1_R18) unmute LOL, set +15dB gain [0b00001010]
    test_register(0x13, 0x0F); // (P1_R19) unmute LOR, set +15dB gain [0b00001010]

    // ADC routing and power up
    set_register(0x00, 0x01); // (P1_R0) point to page 1 on register map
    printf("===== PAGE 1 =====\n");

    test_register(0x34, 0x50); // (P1_R52) Route IN1L and IN2L to Left MICPGA with 10kohm resistance [0b01010000]
    test_register(0x37, 0x50); // (P1_R55) Route IN1R and IN2R to RIght MICPGA with 10kohm resistance [0b01010000]
    test_register(0x36, 0x01); // (P1_R54) CM is routed to Left MICPGA via CM2L with 10kohm resistance
    test_register(0x39, 0x40); // (P1_R57) CM is routed to Right MICPGA via CM1R with 10kohm resistance [0b01000000]
    test_register(0x3B, gain); // (P1_R59) Unmute left MICPGA  and set its gain
    test_register(0x3C, gain); // (P1_R60) Unmute right MICPGA and set its gain

    set_register(0x00, 0x00); // (P0_R0) point to page 0 on register map
    printf("===== PAGE 0 =====\n");

    test_register(0x51, 0xC0);           // (P0_R81) power up left and right ADCs
    test_register(0x52, 0x00);           // (P0_R82) unmute left and right ADCs
    set_register(0x00, 0x00);            // (P0_R0) point to page 0 on register map
    vTaskDelay(10 / portTICK_PERIOD_MS); // wait for device to initialize registers
}
