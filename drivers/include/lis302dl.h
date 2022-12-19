#ifndef LIS302DL_H
#define LIS302DL_H


#include <inttypes.h>


/* SET SPI PERIPHERALS ACCORDING TO PCB: */

#define LIS_SPI                     SPI1
#define LIS_RCC_SPI                 RCC_SPI1

#define LIS_SPI_PORT                GPIOA
#define LIS_SCLK_PIN                GPIO5
#define LIS_MISO_PIN                GPIO6
#define LIS_MOSI_PIN                GPIO7

#define LIS_CS_PORT                 GPIOE
#define LIS_CS_PIN                  GPIO3

#define LIS_INT_PORT                GPIOE
#define LIS_INT1_PIN                GPIO0
#define LIS_INT2_PIN                GPIO1

/* LIS302DL Registers addresses for configuration functions: */
#define LIS_WHOAMI_ADDR             0x0F
#define LIS_CTRL_1_ADDR             0x20
#define LIS_CTRL_2_ADDR             0x21
#define LIS_CTRL_3_ADDR             0x22

#define LIS_HPF_RS_ADDR             0x23
#define LIS_STATUS_ADDR             0x27
#define LIS_OUT_X_ADDR              0x29
#define LIS_OUT_Y_ADDR              0x2B
#define LIS_OUT_Z_ADDR              0x2D

#define LIS_FFW_CFG_1_ADDR          0x30
#define LIS_FFW_SRC_1_ADDR          0x31
#define LIS_FFW_THS_1_ADDR          0x32
#define LIS_FFW_DUR_1_ADDR          0x33

#define LIS_FFW_CFG_2_ADDR          0x34
#define LIS_FFW_SRC_2_ADDR          0x35
#define LIS_FFW_THS_2_ADDR          0x36
#define LIS_FFW_DUR_2_ADDR          0x37

#define LIS_CLCK_CFG_ADDR           0x38
#define LIS_CLCK_SRC_ADDR           0x39
#define LIS_CLCK_THSYX_ADDR         0x3B
#define LIS_CLCK_THSZ_ADDR          0x3C

#define LIS_CLCK_TMLM_ADDR          0x3D
#define LIS_CLCK_LTNC_ADDR          0x3E
#define LIS_CLCK_WNDW_ADDR          0x3F

/* Read/Write and Address increment/decrement modes */
#define LIS_RW_R                    0x01
#define LIS_RW_W                    0x00
#define LIS_MS_INC                  0x01
#define LIS_MS_NINC                 0x00

#define LIS_RW_MODE(MODE)           (MODE << 7)
#define LIS_MS_MODE(MODE)           (MODE << 6)


/* Define setting half bit for threshold registers */
#define LIS_SET_THS_XY(THS, X, Y)                   \
        THS = ((THS | (Y & 0x0F)) << 4) |           \
              ((THS | (X & 0x0F)) << 0)             \

#define LIS_SET_THS_Z(THS, Z)                       \
        THS = (THS << 8) | (Z & 0x0F)               \

/* Macro bit setting functions for common use */
#define BIT_SET(X, MASK)        (X = X | MASK)

#define BIT_CLR(X, MASK)        (X = X & (~MASK))


/* Setting modes for LIS302DL CR1 (read-write) register */

#define LIS_DATA_RATE_100HZ         (0x00 << 7)
#define LIS_DATA_RATE_400HZ         (0x01 << 7)

#define LIS_PWR_DOWN                (0x00 << 6)
#define LIS_PWR_NORM                (0x01 << 6)

#define LIS_SCALE_2G_16             (0x00 << 5)
#define LIS_SCALE_8G_64             (0x01 << 5)

#define LIS_ZAXIS_ENA               (0x01 << 2)
#define LIS_YAXIS_ENA               (0x01 << 1)
#define LIS_XAXIS_ENA               (0x01 << 0)
#define LIS_ZAXIS_DIS               (0x00 << 2)
#define LIS_YAXIS_DIS               (0x00 << 1)
#define LIS_XAXIS_DIS               (0x00 << 0)

#define LIS_SELFTEST_ON             ((0x01 << 4) | (0x01 << 3))
#define LIS_SELFTEST_OFF            ((0x00 << 4) | (0x00 << 3))


/* Setting modes for LIS302DL CR2 (read-write) register */

#define LIS_SPI_4_WIRE              (0x00 << 7)
#define LIS_SPI_3_WIRE              (0x01 << 7)
#define LIS_REBOOT                  (0x01 << 6)
#define LIS_FLTR_BYPASS             (0x00 << 4)
#define LIS_FLTR_OUTREG             (0x01 << 4)

#define LIS_FLTR_FFWU_ENA           ((0x01 << 3) | (0x01 << 2))
#define LIS_FLTR_FFWU_DIS           ((0x00 << 3) | (0x00 << 2))

#define LIS_FLTR_FREQM_0            ((0x00 << 1) | (0x00 << 0))
#define LIS_FLTR_FREQM_1            ((0x01 << 1) | (0x00 << 0))
#define LIS_FLTR_FREQM_2            ((0x00 << 1) | (0x01 << 0))
#define LIS_FLTR_FREQM_3            ((0x01 << 1) | (0x01 << 1))

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



/* Setting modes for LIS302DL CR3 (read-write) interrupts register */

#define LIS_IT_HIGH                 (0x00 << 7)
#define LIS_IT_LOW                  (0x01 << 7)

#define LIS_IT_OUT_PP               (0x00 << 6)
#define LIS_IT_OUT_OD               (0x01 << 6)

#define LIS_IT_SIG_GND_1            ((0x00 << 0) | (0x00 << 1) | (0x00 << 2))
#define LIS_IT_SIG_CLK_1            ((0x01 << 1) | (0x01 << 1) | (0x01 << 2))

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

/* TO-DO: describe function for working with accel in interrupt mode */
/* TO-DO: write functions for interrupt mode configuration */

// uint8_t lis302_dl_cfg_ctr3(uint8_t IHL, uint8_t PPOD, uint8_t ITSIG);


/* Reading LIS302DL status register */

#define LIS_ZYX_OVERRUN             (0x01 << 7)
#define LIS_Z_OVERRUN               (0x01 << 6)
#define LIS_Y_OVERRUN               (0x01 << 5)
#define LIS_X_OVERRUN               (0x01 << 4)

#define LIS_ZYX_NEWDATA             (0x01 << 3)
#define LIS_Z_NEWDATA               (0x01 << 2)
#define LIS_Y_NEWDATA               (0x01 << 1)
#define LIS_X_NEWDATA               (0x01 << 0)


/* Setting modes for LIS302DL Free-fall / Wake-up configuration registers x1, x2 */

#define LIS_IT_COMBO_OR             (0x00 << 7)
#define LIS_IT_COMBO_AND            (0x01 << 7)

#define LIS_ITR_NO_LATCH            (0x00 << 6)
#define LIS_ITR_LATCH               (0x01 << 6)

#define LIS_ITR_ZH_DIS              (0x00 << 5)
#define LIS_ITR_YH_DIS              (0x00 << 3)
#define LIS_ITR_XH_DIS              (0x00 << 1))

#define LIS_ITR_ZL_DIS              (0x00 << 4)
#define LIS_ITR_YL_DIS              (0x00 << 2)
#define LIS_ITR_XL_DIS              (0x00 << 0)

#define LIS_ITR_ZH_ENA              (0x01 << 5)
#define LIS_ITR_YH_ENA              (0x01 << 3)
#define LIS_ITR_XH_ENA              (0x01 << 1)

#define LIS_ITR_ZL_ENA              (0x01 << 4)
#define LIS_ITR_YL_ENA              (0x01 << 2)
#define LIS_ITR_XL_ENA              (0x01 << 0)


/* Read free-fall and wake-up source register (read-only) */
#define LIS_INT_ACTIVE              (0x01 << 6)

#define LIS_ZH_ACTIVE               (0x01 << 5)
#define LIS_YH_ACTIVE               (0x01 << 3)
#define LIS_XH_ACTIVE               (0x01 << 1)

#define LIS_ZL_ACTIVE               (0x01 << 4)
#define LIS_YL_ACTIVE               (0x01 << 2)
#define LIS_XL_ACTIVE               (0x01 << 0)

/* FF_WU_THS_1 THRESHOLD REGISTERS */
#define LIS_FFW_COUNT_DCRM          (0x01 << 7)
#define LIS_FFW_COUNT_NONE          (0x00 << 7)
#define LIS_FFW_SET_THRSHD(X)       (X & 0x7F)

/* Setting FF_WU DURATION REGISTERS */
#define LIS_FFW_SET_DUR(X)          (X & 0xFF)

/* Setting LIS302DL Click configuration register defines  CLICK_CFG */
#define LIS_CLCK_LIR_ENA            (0x01 << 6)
#define LIS_CLCK_LIR_DIS            (0x00 << 6)

#define LIS_CLCK_D_Z_IR_ENA         (0x01 << 5)
#define LIS_CLCK_S_Z_IR_ENA         (0x01 << 4)

#define LIS_CLCK_D_Y_IR_ENA         (0x01 << 3)
#define LIS_CLCK_S_Y_IR_ENA         (0x01 << 2)

#define LIS_CLCK_D_X_IR_ENA         (0x01 << 1)
#define LIS_CLCK_S_X_IR_ENA         (0x01 << 0)

#define LIS_CLCK_D_Z_IR_DIS         (0x00 << 5)
#define LIS_CLCK_S_Z_IR_DIS         (0x00 << 4)

#define LIS_CLCK_D_Y_IR_DIS         (0x00 << 3)
#define LIS_CLCK_S_Y_IR_DIS         (0x00 << 2)

#define LIS_CLCK_D_X_IR_DIS         (0x00 << 1)
#define LIS_CLCK_S_X_IR_DIS         (0x00 << 0)

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


/* Reading click source register CLICK_SRC (39h) */
#define LIS_CLCK_IT_REQ             (0x01 << 6)

#define LIS_CLCK_IT_DZ_REQ          (0x01 << 5)
#define LIS_CLCK_IT_DY_REQ          (0x01 << 3)
#define LIS_CLCK_IT_DX_REQ          (0x01 << 1)

#define LIS_CLCK_IT_SZ_REQ          (0x01 << 4)
#define LIS_CLCK_IT_SY_REQ          (0x01 << 2)
#define LIS_CLCK_IT_SX_REQ          (0x01 << 0)


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
 * Set bit-parameters for CR1 register (default 0x07).
 * DR - data rate selection
 * PD - power down control
 * FS - full scale selection
 * AXES - first 3 bits for ZYX measurement
*/
uint8_t lis302dl_set_cr1(uint8_t DR, uint8_t PD, uint8_t FS, uint8_t AXES);
/* Done and checked */


/**
 * Set bit-parameters for CR2 register (31h and 35h) (default 0x00).
 * SIM - spi crotocol (3-wire or 4-wire)
 * FDS - filtered data selection
 * FLTR - enable high-pass filter for free-fall\wkup
 * FLTR_COEFF - set cut-off frequency configuration
*/
uint8_t lis302dl_set_ctr2(uint8_t SIM, uint8_t FDS, uint8_t FLTR, uint8_t FLTR_COEFF);
/* Done and checked */


// uint8_t lis302_dl_cfg_ctr3(uint8_t IHL, uint8_t PPOD, uint8_t ITSIG);


/**
 * Set bit-parameters for FW_WU_CFG_1(2) (30h and 34h) registers
 * AOI - enabled interrupts
 * LIR - latch interrupt request
 * IR - aviable interrupts for axes
*/
uint8_t lis302dl_set_freefall_it(uint8_t AOI, uint8_t LIR, uint8_t IR, uint8_t regnum);
/* Done and checked */


/**
 * Set bit-parameters for FF_WU_THS_1(2) (32h and 36h) register
 * DCRM - resetting mode selection. default value is 0
 *         0: counter resetted; 1: counter decremented
 * THS - free-fall/wake-up threshold, default: 000 000x
*/
uint8_t lis302dl_set_freefall_ths(uint8_t DCRM, uint8_t THS, uint8_t regnum);
/* Done and checked */


/**
 * Set 8-bit value to FF_WU_DURATION_1(2) (33h and 37h) register,
 * default value is 0000 0000;
 * if ODR=400Hz - duration:= [0 : 2.5 : 637.5] msec;
 * if ODR=100Hz - duration:= [0 : 10 : 2.55] msec;
 * if LIR = 1 this function is blocked in cfg.
*/
uint8_t lis302dl_set_freefall_duration(uint8_t DUR, uint8_t regnum);
/* Done and checked */


/**
 * Set bit-parameters for CLICK_CFG (38h) register.
 * LIR - latch interrupt request, default value 0
 * ZIR - enable interrupt generation on Z axis
 * YIR - enable interrupt generation on Y axis
 * XIR - eable interrupt generation on X axis
*/
uint8_t lis302dl_set_click_it(uint8_t LIR, uint8_t ZIR, uint8_t YIR, uint8_t XIR);
/* Done and checked */


/**
 * Set configuration for CLICK_THSY_X (3Bh) and CLICK_THSZ (3Ch) registers
 * Set threshold from 0.5g to 7.5 (1111) with step 0.5g
 * thsyx - first 4 bits starting from LSB - X axis; default 0x.0
 * thsyx - last 4 bits starting from LSB - Y axis; default 0x0.
 * thsz - first 5 bits starting from LSB - Z axis; defaqult 0x.0
*/
uint8_t lis302dl_set_click_thrshld(uint8_t ths_x, uint8_t ths_y, uint8_t ths_z);
/* Done and checked */


/**
 * Set configuration for CLICK_TimeLimit (3Dh)
 * From 0 to 127.5 msec with step of 0.5 msec;
*/
uint8_t lis302dl_set_click_timelim(uint8_t time_limit);
/* Done and checked */

/**
 * Set configuration for CLICK_Latency (3Eh) register
 * from 0 to 255 msec with step of 1 msec.
*/
uint8_t lis302dl_set_click_latency(uint8_t latency);
/* Done and checked - bug with MSB bit */

/**
 * Set configuration for CLICK_Window (3Fh) register
 * from 0 to 255msec with step of 1 msec 
*/
uint8_t lis302dl_set_click_windows(uint8_t window);
/* Done and checked */

/**
 * Set Init Self-test procedure, ST - CR1 bit
 * Write this bit to CR1 MEMS register to start
 * calibration procedure.
*/
uint8_t lis302dl_selftest(void);


/**
 * Refresh the content of internal registers stored in the
 * flash memory block. The content of internal flash is copied
 * inside corresponding internal registers and it is used to
 * calibrate the device and normally they have not be changed.
*/
uint8_t lis302dl_reboot(void);


/* Read lis302dl status register to check if we have new data or overrun */
uint8_t lis302dl_get_status(void);


/* Read lis 302dl source register to check if we have click interrupts events*/
uint8_t lis302dl_get_click_it(void);


/**
 * Reading at this address clears ff_wu_src_1 register 
 * and the ff, wu 1 interrupt and allows the refreshment
 * of data in the FF_WU_SRC_1 register if the latched opt is 1
*/
uint8_t lis302dl_get_freefall_it(uint8_t register_number);


/**
 * SPI initialization, MEMS` interrupt pins are
 * initialized in lis_accel_init procedure.
*/
void lis302dl_spi_setup(void);
/* Done and checked */

/**
 * Set register adresses according to datasheet
 * for configuration and communication procedure.
*/
void lis302dl_set_reg_addr(struct lis302dl_registers *lis_reg_map);
/* Done and checked */

/**
 * Set bit-values for all registers as a part of
 * initialization procedure.
*/
void lis302dl_set_reg_conf(void);


/**
 * Initialize all peripherals, register addresses,
 * get device id of MEMS-IC, Should be used once.
*/
void lis302dl_accel_init(void);
/* Done and checked; should be updated with registers configurations */


/** 
 * This functions used in accel init procedure, use
 * it only for debug and to check IC-functionality.
*/
uint8_t lis302dl_whoami(void);
/* Done and checked */


/**
 * Base transfer one byte via SPI interface. Note
 * that this function does not have timeout check
 * in loop. It SHOULD be added in next commits.
*/
uint8_t lis302dl_transfer_byte(uint8_t databyte);
/* Done and checked */


/**
 * Read data from MEMS IC according to buffer size.
 * reg_addr - address of the register you`re reading from;
 * data_buff - data buffer where you write received data;
 * buffer_size - how many bites we are reading from MEMS;
*/
uint8_t lis302dl_read(uint8_t reg_addr, uint8_t *databuff_r, uint8_t buffsize);
/* Done and checked */


/**
 * Write data to MEMS-IC according to buffer size.
 * reg_addr - address of the register you`re writing to;
 * data_buff - data which you write to MEMS` register;
 * buffsize - how many bytes you write to MEMS` register;
*/
uint8_t lis302dl_write(uint8_t reg_addr, uint8_t *databuff_w, uint8_t buffsize);
/* Done and checked */


/**
 * Read acceleration from out_x, out_y and out_z registers
 * accel_buf - 3-size buffer; 0 - axis X; 1 - axis Y; 2 - axis Z
 * before calculations, reads cr1 register for getting right constants.
*/
uint8_t lis302dl_read_accel(uint8_t *accel_buf);







#endif /* LIS302DL_H*/
/* END OF FILE*/