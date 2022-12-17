#ifndef LIS302DL_H
#define LIS302DL_H

/* SET SPI PERIPHERALS ACCORDING TO PCB */

#define LIS_SPI                 SPI1
#define LIS_RCC_SPI             RCC_SPI1

#define LIS_SPI_PORT            GPIOA
#define LIS_SCLK_PIN            GPIO5
#define LIS_MISO_PIN            GPIO6
#define LIS_MOSI_PIN            GPIO7

#define LIS_CS_PORT             GPIOE
#define LIS_CS_PIN              GPIO3

#define LIS_INT_PORT            GPIOE
#define LIS_INT1_PIN            GPIO0
#define LIS_INT2_PIN            GPIO1


/* LIS302DL REGISTERS BITMASKS */

#define LIS_CR1_DR          0x80
#define LIS_CR1_PD          0x40
#define LIS_CR1_FS          0x20
#define LIS_CR1_STP         0x10
#define LIS_CR1_STM         0x08
#define LIS_CR1_ZEN         0x04
#define LIS_CR1_YEN         0x02
#define LIS_CR1_XEN         0x01

#define LIS_CR2_SIM         0x80
#define LIS_CR2_BOOT        0x20
#define LIS_CR2_FDS         0x10
#define LIS_HPF_WU2         0x08
#define LIS_HPF_WU1         0x04
#define LIS_HPF_CF2         0x02
#define LIS_HPF_CF1         0x01
/**
 *  High pass filter cut-off frequency configuration:
 * +==============+===================+===================+
 * | HP_coeff2,1  | ft DR=0 (100 Hz)  | ft DR=1 (400 Hz)  |
 * |==============+===================+===================|
 * |   0      0   |         2         |         8         |
 * +--------------+-------------------+-------------------+
 * |   0      1   |         1         |         4         |
 * +--------------+-------------------+-------------------+
 * |   1      0   |        0.5        |         2         |
 * +--------------+-------------------+-------------------+
 * |   1      1   |        0.25       |         1         |
 * +==============+===================+===================+
*/


/* CR3 - Interrupt CTRL Register */

#define LIS_CR3_IHL         0x80
#define LIS_CR3_PP_OD       0x40
#define LIS_CR3_I2CFG2      0x20
#define LIS_CR3_I2CFG1      0x10
#define LIS_CR3_I2CFG0      0x08
#define LIS_CR3_I1CFG2      0x04
#define LIS_CR3_I1CFG1      0x02
#define LIS_CR3_I1CFG0      0x01
/**
 *  Data signals on interrupt pads:
 * +=============+================+================+===============+
 * | I1(2)_CFG2  |   I1(2)_CFG1   |   I1(2)_CFG0   |   Int1(2) Pad |
 * |=============+================+================+===============|
 * |      0      |        0       |       0        |      GND      |
 * +-------------+----------------+----------------+---------------+
 * |      0      |        0       |       1        |   FF_WU_1     |
 * +-------------+----------------+----------------+---------------+
 * |      0      |        1       |       0        |   FF_WU_2     |
 * +-------------+----------------+----------------+---------------|
 * |      0      |        1       |       1        |   FF_WU_1-2   |
 * +-------------+----------------+----------------+---------------|
 * |      1      |        0       |       0        |  Data ready   |
 * +-------------+----------------+----------------+---------------+
 * |      1      |        1       |       1        |   Click IT    |
 * +=============+================+================+===============+
*/


/* Status register (Data overrun or data update) */

#define LIS_SR_ZYXOR        0x80
#define LIS_SR_ZOR          0x40
#define LIS_SR_YOR          0x20
#define LIS_SR_XOR          0x10
#define LIS_SR_ZYXDA        0x08
#define LIS_SR_ZDA          0x04
#define LIS_SR_YDA          0x02
#define LIS_SR_XDA          0x01

/* Free-fall and Wake-up registers 1-2 */

#define LIS_FWU_CFG_AOI    0x80
#define LIS_FWU_CFG_LIR    0x40
#define LIS_FWU_CFG_ZHIE   0x20
#define LIS_FWU_CFG_ZLIE   0x10
#define LIS_FWU_CFG_YHIE   0x08
#define LIS_FWU_CFG_YLIE   0x04
#define LIS_FWU_CFG_XHIE   0x02
#define LIS_FWU_CFG_XLIE   0x01


#define LIS_FWU_SRC_IA     0x40
#define LIS_FWU_SRC_ZH     0x20
#define LIS_FWU_SRC_ZL     0x10
#define LIS_FWU_SRC_YH     0x08
#define LIS_FWU_SRC_YL     0x04
#define LIS_FWU_SRC_XH     0x02
#define LIS_FWU_SRC_XL     0x01

#define LIS_FWU_THS_DCRM   0x80
#define LIS_FWU_MAX_THRES  0x7F    //!!!

#define LIS_FWU_MAX_DURAT  0xFF    //!!!

#define LIS_CLCK_CFG_LIR   0x40
#define LIS_CLCK_CFG_DZ    0x20
#define LIS_CLCK_CFG_SZ    0x10
#define LIS_CLCK_CFG_DY    0x08
#define LIS_CLCK_CFG_SY    0x04
#define LIS_CLCK_CFG_DX    0x02
#define LIS_CLCK_CFG_SX    0x01
/**
 *  CLICK_CFG (38h) truth table:
 * +==============+===================+===================+
 * | double z/y/x |   single z/y/x    |   click output:   |
 * |==============+===================+===================|
 * |       0      |         0         |         0         |
 * +--------------+-------------------+-------------------+
 * |       0      |         1         |      single       |
 * +--------------+-------------------+-------------------+
 * |       1      |         0         |      double       |
 * +--------------+-------------------+-------------------+
 * |       1      |         1         | signal or double  |
 * +==============+===================+===================+
*/


#define LIS_CLCK_SRC_IA    0x40
#define LIS_CLCK_SRC_DZ    0x20
#define LIS_CLCK_SRC_SZ    0x10
#define LIS_CLCK_SRC_DY    0x08
#define LIS_CLCK_SRC_SY    0x04
#define LIS_CLCK_SRC_DX    0x02
#define LIS_CLCK_SRC_SX    0x01


#define LIS_RW_R          0x01
#define LIS_RW_W          0x00
#define LIS_MS_INC        0x01
#define LIS_MS_NINC       0x00
#define LIS_RW_MODE(MODE)   (MODE << 7)
#define LIS_MS_MODE(MODE)   (MODE << 6)



#define LIS_SET_THS_XY(THS, X, Y)                   \
        THS = ((THS | (Y & 0xF0)) << 4) |           \
              ((THS | (X & 0x0F)) << 0)             \

#define LIS_SET_THS_Z(THS, Z)                       \
        THS = (THS << 8) | (Z & 0x0F)               \


#define LIS_CLCK_TLIM_MAX  0xFF
#define LIS_CLCK_LTNC_MAX  0xFF
#define LIS_CLCK_WINDOW    0xFF


#define BIT_SET(X, MASK)        \
        X = X | MASK

#define BIT_CLR(X, MASK)        \
        X = X & (~MASK)


#include <inttypes.h>

/** 
 * Structure for initialisation procedure with accel regs.
 * to see more info - use mems ds - Register description;
 * In source code this struction is used for setting addr
 * according to register map and accel configuration.
 */
struct lis302dl_registers
{
    uint8_t whoami;             //r

    uint8_t ctrl_1;             //rw  
    uint8_t ctrl_2;             //rw
    uint8_t ctrl_3;             //rw

    uint8_t hp_filter_reset;    //r

    uint8_t status;             //r

    uint8_t out_x;              //r
    uint8_t out_y;              //r
    uint8_t out_z;              //r

    uint8_t ff_wu_cfg_1;        //rw
    uint8_t ff_wu_src_1;        //r
    uint8_t ff_wu_ths_1;        //rw
    uint8_t ff_wu_duration_1;   //rw

    uint8_t ff_wu_cfg_2;        //rw
    uint8_t ff_wu_src_2;        //r
    uint8_t ff_wu_ths_2;        //rw
    uint8_t ff_wu_duration_2;   //rw

    uint8_t click_cfg;          //rw
    uint8_t click_src;          //r

    uint8_t click_thsy_x;       //rw
    uint8_t click_thsz;         //rw
    uint8_t click_timlim;       //rw
    uint8_t click_latency;      //rw
    uint8_t click_window;       //rw

};

/**
 * SPI initialization, MEMS` interrupt pins are
 * initialized in lis_accel_init procedure.
*/
void lis_spi_setup(void);

/**
 * Set register adresses according to datasheet
 * for configuration and communication procedure.
*/
void lis_set_reg_addr(struct lis302dl_registers *lis_reg_map);


void lis_set_reg_conf(void);


/**
 * Initialize all peripherals, register addresses,
 * get device id of MEMS-IC, Should be used once.
*/
void lis_accel_init(void);

/** 
 * This functions used in accel init procedure, use
 * it only for debug and to check IC-functionality.
*/
uint8_t lis_whoami(void);


/**
 * Base transfer one byte via SPI interface. Note
 * that this function does not have timeout check
 * in loop. It SHOULD be added in next commits.
*/
uint8_t lis_transfer_byte(uint8_t databyte);


/**
 * Read data from MEMS IC according to buffer size.
 * reg_addr - address of the register you`re reading from;
 * data_buff - data buffer where you write received data;
 * buffer_size - how many bites we are reading from MEMS;
*/
uint8_t lis_read(uint8_t reg_addr, uint8_t *databuff, uint8_t buffsize);


/**
 * Write data to MEMS-IC according to buffer size.
 * reg_addr - address of the register you`re writing to;
 * data_buff - data which you write to MEMS` register;
 * buffsize - how many bytes you write to MEMS` register;
*/
uint8_t lis_write(uint8_t reg_addr, uint8_t *databuff, uint8_t buffsize);


#endif /* LIS302DL_H*/
/* END OF FILE*/