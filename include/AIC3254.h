#ifndef __AIC3254_H
#define __AIC3254_H

#include <stdio.h>
#include "driver/i2c.h"

#define I2C_PORT_NUM 1
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_FREQ_HZ 400000 // init i2c bus @ 400 khz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define AIC3254_ADDR 0x18 // 0b0011000 (7-bit address)
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 1

void init_codec_i2c(void);
void init_codec_demo(void);
void debug_codec_registers(void);

#endif /* __AIC3254_H */