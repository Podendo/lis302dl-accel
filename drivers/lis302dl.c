#include "./include/lis302dl.h"

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>

#include <libopencm3/stm32/f4/spi.h>
#include <libopencm3/stm32/f4/usart.h>


#define LIS_UNSELECT()      gpio_set(LIS_CS_PORT, LIS_CS_PIN);
#define LIS_SELECT()        gpio_clear(LIS_CS_PORT, LIS_CS_PIN);

void lis_spi_setup(void)
{
    /* Note that you can do it in clock_setup function (main.c) */
    //rcc_periph_clock_enable(RCC_SPI1);

    /* Configure /NSS pin for SPI interface */
    gpio_mode_setup(LIS_CS_PORT, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, LIS_CS_PIN);
    gpio_set(LIS_CS_PORT, LIS_CS_PIN);

    /* SPI CLK configuration: */
    gpio_mode_setup(LIS_SPI_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, LIS_SCLK_PIN);
    /* SPI MISO configuration: */
    gpio_mode_setup(LIS_SPI_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, LIS_MISO_PIN);
    /* SPI MOSI configuration: */
    gpio_mode_setup(LIS_SPI_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, LIS_MOSI_PIN);
    
    /* MOSI and /NSS pins output options: */
    gpio_set_output_options(LIS_SPI_PORT, GPIO_OTYPE_OD,
                    GPIO_OSPEED_100MHZ, LIS_CS_PIN);
    gpio_set_output_options(LIS_SPI_PORT, GPIO_OTYPE_OD,
                    GPIO_OSPEED_100MHZ, LIS_MOSI_PIN);
    
    /* Configure pin alternative functions for SPI(1) */
    gpio_set_af(LIS_SPI_PORT, GPIO_AF5, LIS_SCLK_PIN);
    gpio_set_af(LIS_SPI_PORT, GPIO_AF5, LIS_MISO_PIN);
    gpio_set_af(LIS_SPI_PORT, GPIO_AF5, LIS_MOSI_PIN);
    
    /**
     * Set up SPI in Master mode with:
     * clock baud rate: 168 MHz of peripheral clock freq
     * clock polarity: idle high
     * clock phase: data valid on falling edge
     * data frame format: 8-bit
     * 2frame format: MSB first
     * in RM page 879: CPHA=0 CPOL=1
    */
    spi_init_master(LIS_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_128,
                        SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                        SPI_CR1_CPHA_CLK_TRANSITION_1,
                        SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    

    spi_enable(LIS_SPI);

    return;
}


void lis_set_reg_addr(struct lis302dl_registers *lis_reg_map)
{
    lis_reg_map->whoami              = 0x0F;

    lis_reg_map->ctrl_1              = 0x20;
    lis_reg_map->ctrl_2              = 0x21;
    lis_reg_map->ctrl_3              = 0x22;

    lis_reg_map->hp_filter_reset     = 0x23;

    lis_reg_map->status              = 0x27;

    lis_reg_map->out_x               = 0x29;
    lis_reg_map->out_y               = 0x2A;
    lis_reg_map->out_z               = 0x2D;

    lis_reg_map->ff_wu_cfg_1         = 0x30;
    lis_reg_map->ff_wu_src_1         = 0x31;
    lis_reg_map->ff_wu_ths_1         = 0x32;
    lis_reg_map->ff_wu_duration_1    = 0x33;

    lis_reg_map->ff_wu_cfg_2         = 0x34;
    lis_reg_map->ff_wu_src_2         = 0x35;
    lis_reg_map->ff_wu_ths_2         = 0x36;
    lis_reg_map->ff_wu_duration_2    = 0x37;

    lis_reg_map->click_cfg           = 0x38;
    lis_reg_map->click_src           = 0x39;
    lis_reg_map->click_thsy_x        = 0x3B;
    lis_reg_map->click_thsz          = 0x3C;

    lis_reg_map->click_timlim        = 0x3D;
    lis_reg_map->click_latency       = 0x3E;
    lis_reg_map->click_window        = 0x3F;

    return;
}


//void lis_set_reg_conf(void);


void lis_accel_init(void)
{
    lis_spi_setup();
    //lis_set_reg_addr(&lis_reg_map);

    return;
}

uint8_t lis_whoami(struct lis302dl_registers *lis_reg_map)
{
    LIS_SELECT();

    uint8_t lis_reg = lis_reg_map->whoami;
    uint8_t lis_rw = 0x01;
    uint8_t lis_am = 0x00;
    uint8_t command = (lis_rw << 7 ) | (lis_am << 6) | lis_reg;
    uint8_t ans = 0x00;

    usart_send_blocking(USART2, command);

    spi_send(LIS_SPI, command);
    ans = spi_read(LIS_SPI);

    usart_send_blocking(USART2, ans);

    LIS_UNSELECT();

    return ans;
}


/* END OF FILE*/