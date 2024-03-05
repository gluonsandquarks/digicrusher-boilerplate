#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include <stdio.h>
#include "AIC3254.h"

#define I2S_SAMPLE_FREQ 48000 // 48kHz sampling frequency (f_s, also known as sampling rate, sample rate, etc)
#define I2S_PORT I2S_NUM_0
// define i2s pins
#define I2S_MCLK_IO 0 // mclk -> can only be defined to be GPIO0, GPIO1 or GPIO3
#define I2S_BCLK 23   // bit clock
#define I2S_LRC 25    // ws / left right clock
#define I2S_DIN 27    // data in
#define I2S_DOUT 26   // data out

#define DMA_BUF_LEN (32)
#define DMA_BUF_COUNT (2)

#define MAX_32BIT 2147483647.0F

// DMA buffers
int32_t rxbuf[DMA_BUF_LEN * 2], txbuf[DMA_BUF_LEN * 2];
float l_in[DMA_BUF_LEN], r_in[DMA_BUF_LEN];
float l_out[DMA_BUF_LEN], r_out[DMA_BUF_LEN];

void init_i2s()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_FREQ,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = DMA_BUF_COUNT,
        .dma_buf_len = DMA_BUF_LEN,
        .use_apll = true,
        .tx_desc_auto_clear = true};

    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_MCLK_IO,
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_DIN};

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);

    // lEGACY CODE
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1); // set GPIO0 function to CLK_OUT1 -> MCLK on GPIO0
    // REG_WRITE(PIN_CTRL, 0xFFFFFFF0);                              // route i2s0 internal CLK to CLK_OUT

    i2s_set_clk(I2S_PORT, I2S_SAMPLE_FREQ, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
    i2s_set_sample_rates(I2S_PORT, I2S_SAMPLE_FREQ);
    printf("Sampling frequency: %.2f\n", i2s_get_clk(I2S_PORT));
}

void app_main()
{

    float gain = 1.0f;

    // debug white LED
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    int ledState = 1;
    gpio_set_level(GPIO_NUM_5, ledState);

    // initialize codec
    init_codec_i2c();
    init_codec_demo();
    // debug_codec_registers(); // uncomment this line to print and look at the values written and read from each register on the codec

    // initialize i2s bus
    init_i2s();

    // state variable bytes read by dma transfer
    size_t readsize = 0;

    while (1)
    {
        // read 256 samples (128 stereo samples)
        esp_err_t rxfb = i2s_read(I2S_PORT, &rxbuf[0], sizeof(rxbuf), &readsize, 1000);
        if (rxfb == ESP_OK && readsize == sizeof(rxbuf))
        {
            // extract stereo samples to mono buffers
            int y = 0;
            for (int i = 0; i < DMA_BUF_LEN * 2; i = i + 2)
            {
                l_in[y] = ((float)rxbuf[i]) / MAX_32BIT;
                r_in[y] = ((float)rxbuf[i + 1]) / MAX_32BIT;
                y++;
            }

            // do something to samples here
            for (int i = 0; i < DMA_BUF_LEN; i++)
            {
                // perform mono mix + gain control
                l_out[i] = (l_in[i] + r_in[i]) * gain;
                r_out[i] = l_out[i];
            }

            // merge two l and r buffers into a mixed buffer and write back to hardware
            y = 0;
            for (int i = 0; i < DMA_BUF_LEN; i++)
            {
                txbuf[y] = (int32_t)(l_out[i] * MAX_32BIT);
                txbuf[y + 1] = (int32_t)(r_out[i] * MAX_32BIT);
                y = y + 2;
            }

            i2s_write(I2S_PORT, &txbuf[0], sizeof(txbuf), &readsize, 1000);
        }
    }
}
