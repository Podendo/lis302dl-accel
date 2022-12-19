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

void lis302dl_spi_setup(void)
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

    spi_init_master(LIS_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                        SPI_CR1_CPHA_CLK_TRANSITION_1,
                        SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    spi_set_clock_polarity_0(LIS_SPI);
    spi_set_clock_phase_1(LIS_SPI);
    spi_enable_software_slave_management(LIS_SPI);
    spi_enable_ss_output(LIS_SPI);
    //spi_set_nss_high(LIS_SPI);
    
    spi_enable(LIS_SPI);

    return;
}


void lis302dl_set_reg_addr(struct lis302dl_registers *lis_reg_map)
{
    lis_reg_map->whoami              = 0x0F;

    lis_reg_map->ctrl_1              = 0x20;
    lis_reg_map->ctrl_2              = 0x21;
    lis_reg_map->ctrl_3              = 0x22;

    lis_reg_map->hp_filter_reset     = 0x23;

    lis_reg_map->status              = 0x27;

    lis_reg_map->out_x               = 0x29;
    lis_reg_map->out_y               = 0x2B;
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


void lis302dl_set_reg_conf(void)
{
    lis302dl_set_cr1(LIS_DATA_RATE_100HZ, LIS_PWR_NORM, LIS_SCALE_2G_16,
                        LIS_ZAXIS_ENA | LIS_YAXIS_ENA | LIS_XAXIS_ENA);

    lis302dl_set_ctr2(LIS_SPI_4_WIRE, LIS_FLTR_BYPASS, LIS_FLTR_FFWU_ENA,
                                                     LIS_FLTR_FREQM_1);

    lis302dl_set_freefall_it(LIS_IT_COMBO_OR, LIS_ITR_LATCH,
        LIS_ITR_ZH_ENA | LIS_ITR_YH_ENA | LIS_ITR_XH_ENA, 0x01);

    lis302dl_set_freefall_it(LIS_IT_COMBO_AND, LIS_ITR_LATCH,
        LIS_ITR_ZL_ENA | LIS_ITR_YL_ENA | LIS_ITR_XL_ENA, 0x02);

    lis302dl_set_freefall_ths(LIS_FFW_COUNT_DCRM, 0xFF, 0x01);
    lis302dl_set_freefall_ths(LIS_FFW_COUNT_NONE, 0xAA, 0x02);

    lis302dl_set_freefall_duration(0xAA, 0x01);
    lis302dl_set_freefall_duration(0xFF, 0x02);
    
    return;
}


uint8_t lis302dl_set_cr1(uint8_t DR, uint8_t PD, uint8_t FS, uint8_t AXES)
{
    uint8_t cfg_setup = DR | PD | FS | AXES;

    lis302dl_write(LIS_CTRL_1_ADDR, &cfg_setup, 0x01);

    return 0;
}


uint8_t lis302dl_set_ctr2(uint8_t SIM, uint8_t FDS, uint8_t FLTR, uint8_t FLTR_COEFF)
{
    uint8_t cfg_setup = SIM | FDS | FLTR | FLTR_COEFF;
    
    lis302dl_write(LIS_CTRL_2_ADDR, &cfg_setup, 0x01);

    return 0;
}


// uint8_t lis302_dl_cfg_ctr3(uint8_t IHL, uint8_t PPOD, uint8_t ITSIG);


uint8_t lis302dl_set_freefall_it(uint8_t AOI, uint8_t LIR, uint8_t IR, uint8_t regnum)
{
    uint8_t cfg_setup = AOI | LIR | IR;

    switch (regnum)
    {
    case 0x01:
        lis302dl_write(LIS_FFW_CFG_1_ADDR, &cfg_setup, 0x01);
        break;
    case 0x02:
        lis302dl_write(LIS_FFW_CFG_2_ADDR, &cfg_setup, 0x01);
    default:
        break;
    }

    return 0;
}


inline uint8_t lis302dl_set_freefall_ths(uint8_t DCRM, uint8_t THS, uint8_t regnum)
{
    uint8_t cfg_setup = DCRM | LIS_FFW_SET_THRSHD(THS);

    switch(regnum)
    {
    case 0x01:
        lis302dl_write(LIS_FFW_THS_1_ADDR, &cfg_setup, 0x01);
        break;
    case 0x02:
        lis302dl_write(LIS_FFW_THS_2_ADDR, &cfg_setup, 0x01);
        break;
    default:
        break;
    }

    return 0;
}


uint8_t lis302dl_set_freefall_duration(uint8_t DUR, uint8_t regnum)
{
    uint8_t cfg_setup = LIS_FFW_SET_DUR(DUR);

    switch (regnum)
    {
    case 0x01:
        lis302dl_write(LIS_FFW_DUR_1_ADDR, &cfg_setup, 0x01);
        break;
    case 0x02:
        lis302dl_write(LIS_FFW_DUR_2_ADDR, &cfg_setup, 0x01);
    default:
        break;
    }
    return 0;
}


/* To-Do: Click interrupt configurations blocks IC - do not use this func in init procedure */
uint8_t lis302dl_set_click_it(uint8_t LIR, uint8_t ZIR, uint8_t YIR, uint8_t XIR)
{
    uint8_t cfg_setup = LIR | ZIR | YIR | XIR;

    lis302dl_write(LIS_CLCK_CFG_ADDR, &cfg_setup, 0x01);

    return 0;
}


uint8_t lis302dl_set_click_thrshld(uint8_t ths_x, uint8_t ths_y, uint8_t ths_z)
{
    uint8_t thsyx, thsz;

    if(ths_x > 0x0F)    ths_x = 0x0F;
    if(ths_y > 0x0F)    ths_y = 0x0F;
    if(ths_z > 0x0F)    ths_z = 0x0F;

    LIS_SET_THS_XY(thsyx, ths_x, ths_y);
    LIS_SET_THS_Z(thsz, ths_z);

    lis302dl_write(LIS_CLCK_THSYX_ADDR, &thsyx, 0x01);
    lis302dl_write(LIS_CLCK_THSZ_ADDR, &thsz, 0x01);

    return 0;
}


uint8_t lis302dl_set_click_timelim(uint8_t time_limit)
{
    lis302dl_write(LIS_CLCK_TMLM_ADDR, &time_limit, 0x01);

    return 0;
}


uint8_t lis302dl_set_click_latency(uint8_t latency)
{
    /* ToDo: msb(8) is broken - 0x00 feedback from
     * all registers after 0x80 latency value */
    if(latency >= 0x80) latency = 0x79;

    lis302dl_write(LIS_CLCK_LTNC_ADDR, &latency, 0x01);

    return 0;
}


uint8_t lis302dl_set_click_windows(uint8_t window)
{
    lis302dl_write(LIS_CLCK_WNDW_ADDR, &window, 0x01);

    return 0;
}

/* To-Do: write this function for check MEMS params
 * Last command should be resetting this bit value to 0
*/
uint8_t lis302dl_selftest(void)
{
    uint8_t cfg_setup = LIS_SELFTEST_ON;

    lis302dl_write(LIS_CTRL_1_ADDR, &cfg_setup, 0x01);

    return 0;
}


uint8_t lis302dl_reboot(void)
{
    uint8_t cfg_setup = LIS_REBOOT;

    lis302dl_write(LIS_CTRL_2_ADDR, &cfg_setup, 0x01);

    return 0;
}


uint8_t lis302dl_get_status(void)
{
    uint8_t status = 0x00;

    lis302dl_read(LIS_STATUS_ADDR, &status, 0x01);

    return status;
}


uint8_t lis302dl_get_click_it(void)
{
    uint8_t status = 0x00;
    
    lis302dl_read(LIS_CLCK_SRC_ADDR, &status, 0x01);

    return status;
}


uint8_t lis302dl_get_freefall_it(uint8_t register_number)
{
    uint8_t status = 0x00;

    switch(register_number)
    {
    case 0x01:
        lis302dl_read(LIS_FFW_SRC_1_ADDR, &status, 0x01);
        break;
    case 0x02:
        lis302dl_read(LIS_FFW_SRC_2_ADDR, &status, 0x01);
        break;
    default:
        break;
    }

    return status;
}

void lis302dl_accel_init(void)
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
    lis302dl_spi_setup();

    sleep_ms(100);

    /* Check device id (debug) */
    lis302dl_whoami();

    /* Configure Control Register 1 for operating in normal mode */
    lis302dl_set_cr1(LIS_DATA_RATE_100HZ, LIS_PWR_NORM, LIS_SCALE_2G_16,
                    LIS_ZAXIS_ENA | LIS_YAXIS_ENA | LIS_XAXIS_ENA);

    gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    
    lis302dl_reboot();
    //lis302dl_selftest();

    sleep_ms(1000);

    return;
}


uint8_t lis302dl_whoami(void)
{
    uint8_t lis_reg = 0x0F;
    uint8_t proto = lis_reg | LIS_RW_MODE(LIS_RW_R) | LIS_MS_MODE(LIS_MS_NINC);

    usart_send_blocking(USART2, proto);

    LIS_SELECT();

    lis302dl_transfer_byte(proto);
    
    /* Send dummy byte to generate the SPI clock to MEMS */
    proto = lis302dl_transfer_byte(0x00);

    usart_send_blocking(USART2, proto);

    LIS_UNSELECT();

    return proto;
}


uint8_t lis302dl_transfer_byte(uint8_t databyte)
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


uint8_t lis302dl_read(uint8_t reg_addr, uint8_t *databuff_r, uint8_t buffsize)
{
    uint8_t proto = 0x00;

    if(buffsize > 0x01){
        proto = reg_addr | LIS_RW_MODE(LIS_RW_R) | LIS_MS_MODE(LIS_MS_INC);
    }
    else{
        proto = reg_addr | LIS_RW_MODE(LIS_RW_R) | LIS_MS_MODE(LIS_MS_NINC);
    }

    LIS_SELECT();

    lis302dl_transfer_byte(proto);
    do{
        buffsize--;
        *databuff_r = lis302dl_transfer_byte(0x00);
        databuff_r++;
    } while(buffsize > 0x00);

    LIS_UNSELECT();

    return 0;
}


uint8_t lis302dl_write(uint8_t reg_addr, uint8_t *databuff_w, uint8_t buffsize)
{
    uint8_t proto = 0x00;

    if(buffsize > 0x01){
        proto = reg_addr | LIS_RW_MODE(LIS_RW_W) | LIS_MS_MODE(LIS_MS_INC);
    }
    else{
        proto = reg_addr | LIS_RW_MODE(LIS_RW_W) | LIS_MS_MODE(LIS_MS_NINC);
    }

    LIS_SELECT();

    lis302dl_transfer_byte(proto);
    do{
        buffsize--;
        lis302dl_transfer_byte(*databuff_w);
        databuff_w++;
    }while(buffsize > 0x00);

    LIS_UNSELECT();

    return 0;
}


uint8_t lis302dl_read_accel(uint8_t *accel_buf)
{
    uint8_t reg_ctrl1 = 0x00;
    uint8_t scl_coeff = 0x00;

    uint8_t tmp_buf[3];

    lis302dl_read(LIS_CTRL_1_ADDR, &reg_ctrl1, 0x01);

    lis302dl_read(LIS_OUT_X_ADDR, &tmp_buf[0], 0x01);
    lis302dl_read(LIS_OUT_Y_ADDR, &tmp_buf[1], 0x01);
    lis302dl_read(LIS_OUT_Z_ADDR, &tmp_buf[2], 0x01);

    switch(reg_ctrl1 & 0x20)
    {
    case LIS_SCALE_2G_16:
        scl_coeff = 0x12;   //18
        break;
    case LIS_SCALE_8G_64:
        scl_coeff = 0x42;   //72
        break;
    default:
        break;
    }

    for(uint8_t i = 0; i < 3; i++){
        accel_buf[i] = tmp_buf[2*i] * scl_coeff;
    }

    usart_send_blocking(USART2, accel_buf[0]);
    usart_send_blocking(USART2, accel_buf[1]);
    usart_send_blocking(USART2, accel_buf[2]);

    return 0;
}


/* END OF FILE*/