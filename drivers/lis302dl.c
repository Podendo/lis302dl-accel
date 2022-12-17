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

#define LIS_UNSELECT()      gpio_set(LIS_CS_PORT, LIS_CS_PIN)
#define LIS_SELECT()        gpio_clear(LIS_CS_PORT, LIS_CS_PIN)


extern void sleep_ms(uint32_t delay_ms);

void lis_spi_setup(void)
{
    /* Note that you can do it in clock_setup function (main.c) */
    rcc_periph_clock_enable(LIS_RCC_SPI);

    /* Configure /NSS pin for SPI interface and set it to hight */
    gpio_mode_setup(LIS_CS_PORT, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, LIS_CS_PIN);
                    
    gpio_set_output_options(LIS_CS_PORT,  GPIO_OTYPE_PP,
                    GPIO_OSPEED_50MHZ, LIS_CS_PIN);

    gpio_set(LIS_CS_PORT, LIS_CS_PIN);

    /* SPI CLK configuration: */
    gpio_mode_setup(LIS_SPI_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_PULLDOWN, LIS_SCLK_PIN);
    /* SPI MISO configuration: */
    gpio_mode_setup(LIS_SPI_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_PULLDOWN, LIS_MISO_PIN);
    /* SPI MOSI configuration: */
    gpio_mode_setup(LIS_SPI_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_PULLDOWN, LIS_MOSI_PIN);
    
    /* MOSI and /NSS pins output options: */
    gpio_set_output_options(LIS_SPI_PORT, GPIO_OTYPE_PP,
                    GPIO_OSPEED_50MHZ, LIS_CS_PIN);

    gpio_set_output_options(LIS_SPI_PORT, GPIO_OTYPE_PP,
                    GPIO_OSPEED_50MHZ, LIS_MOSI_PIN);

    gpio_set_output_options(LIS_SPI_PORT, GPIO_OTYPE_PP,
                    GPIO_OSPEED_50MHZ, LIS_MISO_PIN);
    
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
     * in RM page 879: CPHA=1 CPOL=0
    */
    spi_reset(LIS_SPI);

    spi_init_master(LIS_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                        SPI_CR1_CPHA_CLK_TRANSITION_2,
                        SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    spi_set_clock_polarity_0(LIS_SPI);
    spi_set_clock_phase_1(LIS_SPI);
    spi_enable_software_slave_management(LIS_SPI);
    spi_enable_ss_output(LIS_SPI);
    //spi_set_nss_high(LIS_SPI);
    
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
    gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);

    /* Set IT1 and IT2 pin-outs for interrupts: */
    gpio_clear(LIS_INT_PORT, LIS_INT1_PIN | LIS_INT2_PIN);

    gpio_mode_setup(LIS_INT_PORT, GPIO_MODE_INPUT,
            GPIO_PUPD_NONE, LIS_INT1_PIN | LIS_INT2_PIN);

    gpio_set(LIS_INT_PORT, LIS_INT1_PIN | LIS_INT2_PIN);

    gpio_set_output_options(LIS_INT_PORT, GPIO_OTYPE_PP,
            GPIO_OSPEED_50MHZ, LIS_INT1_PIN | LIS_INT2_PIN);
    
    /* Setup the SPI peripherals: */
    lis_spi_setup();
    
    sleep_ms(1000);
    gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    lis_whoami();

    return;
}


uint8_t lis_whoami(void)
{
    uint8_t lis_reg = 0x0F;
    uint8_t proto = lis_reg | LIS_RW_MODE(LIS_RW_R) | LIS_MS_MODE(LIS_MS_NINC);

    usart_send_blocking(USART2, proto);

    LIS_SELECT();

    lis_transfer_byte(proto);
    
    /* Send dummy byte to generate the SPI clock to MEMS */
    proto = lis_transfer_byte(0x00);

    usart_send_blocking(USART2, proto);

    LIS_UNSELECT();

    return proto;
}


uint8_t lis_transfer_byte(uint8_t databyte)
{
    while((SPI_SR(LIS_SPI) & SPI_SR_TXE) == 0){
        __asm__("nop");
    }
    spi_send(LIS_SPI, databyte);
    while((SPI_SR(LIS_SPI) & SPI_SR_RXNE) == 0){
        __asm__("nop");
    }

    return (uint8_t)SPI_DR(LIS_SPI);
}


uint8_t lis_read(uint8_t reg_addr, uint8_t *databuff, uint8_t buffsize)
{
    uint8_t proto = 0x00;
    uint8_t byte = 0x00;

    if(buffsize > 0x01){
        proto = reg_addr | LIS_RW_MODE(LIS_RW_R) | LIS_MS_MODE(LIS_MS_INC);
    }
    else{
        proto = reg_addr | LIS_RW_MODE(LIS_RW_R) | LIS_MS_MODE(LIS_MS_NINC);
    }

    LIS_SELECT();
    lis_transfer_byte(proto);
    do{
        buffsize--;
        *databuff = lis_transfer_byte(0x00);
        databuff++;
    } while(buffsize > 0x00);
    LIS_UNSELECT();

    return 0;
}

/* END OF FILE*/