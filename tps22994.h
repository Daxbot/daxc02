#ifndef __TPS22994_H__
#define __TPS22994_H__

#include <linux/i2c.h>

#define TPS22994_I2C_ADDR               0x70
#define TPS22994_I2C_BUS                0x06


/******************************************************************
                TPS22994 Register Defines
*******************************************************************/

/*
 * Configuration registers
 *
 * | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 * |   X   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |
 * |   X   |    On-Delay   |       Slew Rate       | Quick Dischrg |
 *
 * Default Values:
 *      On-Delay:               0b00    (11us)
 *      Slew Rate:              0b011   (460 us/V)
 *      Quick Discharge:        0b10    (951 Ω)
 */

#define TPS22994_REG_CONFIG1        0x01
#define TPS22994_REG_CONFIG2        0x02
#define TPS22994_REG_CONFIG3        0x03
#define TPS22994_REG_CONFIG4        0x04

/*
 * Control register
 *
 * | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 * |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |
 * | Ctrl4 | Ctrl3 | Ctrl2 | Ctrl1 |  En4  |  En3  |  En2  |  En1  |
 *
 * Default Values:
 *      Channel Control:        0b0     (GPIO)
 *      Channel Enable:         0b0     (Off)
 */

#define TPS22994_REG_CONTROL        0x05

/*
 * Mode registers
 *
 * | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 * |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |
 * |   X   |   X   |   X   |   X   |  En4  |  En3  |  En2  |  En1  |
 *
 * Default Values:
 *      Channel Enable:         0b0     (Off)
 */

#define TPS22994_REG_MODE1          0x06
#define TPS22994_REG_MODE2          0x07
#define TPS22994_REG_MODE3          0x08
#define TPS22994_REG_MODE4          0x09
#define TPS22994_REG_MODE5          0x0A
#define TPS22994_REG_MODE6          0x0B
#define TPS22994_REG_MODE7          0x0C
#define TPS22994_REG_MODE8          0x0D
#define TPS22994_REG_MODE9          0x0E
#define TPS22994_REG_MODE10         0x0F
#define TPS22994_REG_MODE11         0x10
#define TPS22994_REG_MODE12         0x11


/******************************************************************
                TPS22994 Value Masks
*******************************************************************/

#define TPS22994_I2C    0xF0

#define TPS22994_CH1    0x01
#define TPS22994_CH2    0x02
#define TPS22994_CH3    0x04
#define TPS22994_CH4    0x08
#define TPS22994_ALL    0x0F


/******************************************************************
                TPS22994 Prototypes
*******************************************************************/

int tps22994_power_up(struct i2c_client *client);
int tps22994_power_down(struct i2c_client *client);

#endif
