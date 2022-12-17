#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>

#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/spi.h>

#include "../drivers/include/lis302dl.h"

static volatile uint32_t sys_ms;

static uint8_t usart_rx_buffer[128];
static uint8_t urxb = 0;


struct lis302dl_registers lis_r_map;
struct lis302dl_registers lis_r_cfg;


extern void sleep_ms(uint32_t delay_ms);

static void clock_setup(void);
static void systick_setup(void);
static void gpio_setup(void);

static void usart_setup(void);
//static void spi_setup(void);

void led_blinking(void);

void usart2_isr(void);

void usart_transmit(uint32_t USARTx, uint8_t *data, uint8_t size);

/* ######################################################################### */

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    systick_setup();

    lis_set_reg_addr(&lis_r_map);
    lis_accel_init();
    sleep_ms(10);

    uint8_t tmp = 0;
    uint8_t com[3] = {0x00, 0x00, 0x00};

    gpio_set(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);

    while(1)
    {
        
        led_blinking();
        sleep_ms(1000);

        lis_read(lis_r_map.out_x, com, 3);
        usart_transmit(USART2, com, 3);

    }

    return 0;
}
/* ######################################################################### */






/*___________________F U N C T I O N   P R O T O T Y P E S___________________*/

extern void sleep_ms(uint32_t delay_ms)
{
    uint32_t current_ms = sys_ms;
    while((sys_ms - current_ms) < delay_ms){
        continue;
    }
    return;
}

/*___________________________________________________________________________*/
static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);

    rcc_periph_clock_enable(RCC_USART2);

    //rcc_periph_clock_enable(RCC_SPI1);

    return;
}

/*___________________________________________________________________________*/
static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(21000 - 1);

    systick_counter_enable();
    systick_interrupt_enable();

    return;
}

void sys_tick_handler(void)
{
    sys_ms += 1;
    return;
}

/*___________________________________________________________________________*/
static void gpio_setup(void)
{
    gpio_clear(GPIOA, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

    gpio_clear(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            GPIO12 | GPIO13 | GPIO14 | GPIO15);
  
    return;
}

/*___________________________________________________________________________*/
static void usart_setup(void)
{
    nvic_enable_irq(NVIC_USART2_IRQ);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);

    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO3);

    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable_rx_interrupt(USART2);

    usart_enable(USART2);

    return;
}

void usart2_isr(void)
{
    uint32_t usart_register = 0;
    usart_register = USART_SR(USART2);

    if(usart_register & USART_SR_RXNE)
    {
        usart_rx_buffer[urxb] = USART_DR(USART2);
        if(urxb == 128) urxb = 0;
        urxb = urxb + 1;
    }
    return;
}

void usart_transmit(uint32_t USARTx, uint8_t *data, uint8_t size)
{
    for(uint8_t i = 0; i < size; i++){
        usart_send_blocking(USARTx, data[i]);
    }
    return;
}

/*___________________________________________________________________________*/
void led_blinking(void)
{
    gpio_toggle(GPIOD, GPIO12);
    sleep_ms(250);
    gpio_toggle(GPIOD, GPIO13);
    sleep_ms(250);
    gpio_toggle(GPIOD, GPIO14);
    sleep_ms(250);
    gpio_toggle(GPIOD, GPIO15);

    return;
}


/* END OF FILE */