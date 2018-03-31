/*
 * Driver for Nova Dynamics DAX-C02 dual mipi camera board.
 * Used to interface Leopard Imaging LI-M021C-MIPI cameras to the Jetson TX1/TX2.
 *
 * Copyright 2017-2018 Nova Dynamics LLC
 * Written by Wilkins White <ww@novadynamics.com>
 * 
 * Based on SoC Camera driver for Sony OV5693 written by
 * David Wang, Copyright 2013-2017 NVIDIA CORPORATION
 * 
 * Based on Aptina MT9M021 Camera driver written by
 * Prashanth Subramanya, Copyright 2013 Aptina Imaging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DAXC02_H__
#define __DAXC02_H__

/***************************************************
        MT9M021 Image Sensor Registers
****************************************************/

#define MT9M021_CHIP_ID_REG             0x3000
#define MT9M021_RESET_REG               0x301A
#define MT9M021_SEQ_CTRL_PORT           0x3088
#define MT9M021_SEQ_DATA_PORT           0x3086
#define MT9M021_TEST_RAW_MODE           0x307A
#define MT9M021_DARK_CTRL               0x3044
#define MT9M021_DATA_PEDESTAL           0x301E
#define MT9M021_COLUMN_CORRECTION       0x30D4
#define MT9M021_FLASH                   0x3046

#define MT9M021_VT_SYS_CLK_DIV          0x302A
#define MT9M021_VT_PIX_CLK_DIV          0x302C
#define MT9M021_PRE_PLL_CLK_DIV         0x302E
#define MT9M021_PLL_MULTIPLIER          0x3030
#define MT9M021_DIGITAL_TEST            0x30B0

#define MT9M021_Y_ADDR_START            0x3002
#define MT9M021_X_ADDR_START            0x3004
#define MT9M021_Y_ADDR_END              0x3006
#define MT9M021_X_ADDR_END              0x3008
#define MT9M021_FRAME_LENGTH_LINES      0x300A
#define MT9M021_LINE_LENGTH_PCK         0x300C
#define MT9M021_COARSE_INT_TIME         0x3012
#define MT9M021_FINE_INT_TIME           0x3014
#define MT9M021_COARSE_INT_TIME_CB      0x3016
#define MT9M021_FINE_INT_TIME_CB        0x3018
#define MT9M021_FRAME_LENGTH_LINES_CB   0x30AA
#define MT9M021_X_ODD_INC               0x30A2
#define MT9M021_Y_ODD_INC               0x30A6
#define MT9M021_READ_MODE               0x3040
#define MT9M021_READ_SPEED              0x3028
#define MT9M021_TEST_PATTERN            0x3070
#define MT9M021_DIGITAL_BINNING         0x3032

#define MT9M021_AE_CTRL                 0x3100
#define MT9M021_AE_LUMA_TARGET          0x3102
#define MT9M021_EMBEDDED_DATA_CTRL      0x3064
#define MT9M021_DATAPATH_SELECT         0x306E

#define MT9M021_GREEN1_GAIN             0x3056
#define MT9M021_BLUE_GAIN               0x3058
#define MT9M021_RED_GAIN                0x305A
#define MT9M021_GREEN2_GAIN             0x305C
#define MT9M021_GLOBAL_GAIN             0x305E
#define MT9M021_GREEN1_GAIN_CB          0x30BC
#define MT9M021_BLUE_GAIN_CB            0x30BE
#define MT9M021_RED_GAIN_CB             0x30C0
#define MT9M021_GREEN2_GAIN_CB          0x30C2
#define MT9M021_GLOBAL_GAIN_CB          0x30C4


/***************************************************
        MT9M021 Image Sensor Defines
****************************************************/

#define BRIDGE_I2C_ADDR                 0x0e
#define MT9M021_I2C_ADDR                0x10
#define MT9M021_CHIP_ID                 0x2401

#define MT9M021_PIXEL_ARRAY_WIDTH       1280
#define MT9M021_PIXEL_ARRAY_HEIGHT      960
#define MT9M021_LLP_RECOMMENDED         1650

#define MT9M021_EXT_FREQ                24000000
#define MT9M021_TARGET_FREQ             74250000
#define MT9M021_PLL_M                   99
#define MT9M021_PLL_N                   4
#define MT9M021_PLL_P1                  1
#define MT9M021_PLL_P2                  8

#define MT9M021_ROW_START_MIN           0
#define MT9M021_ROW_START_MAX           960
#define MT9M021_ROW_START_DEF           0x0078
#define MT9M021_WINDOW_HEIGHT_MIN       2
#define MT9M021_WINDOW_HEIGHT_MAX       960
#define MT9M021_WINDOW_HEIGHT_DEF       720
#define MT9M021_WINDOW_WIDTH_MIN        2
#define MT9M021_WINDOW_WIDTH_MAX        1280
#define MT9M021_WINDOW_WIDTH_DEF        1280
#define MT9M021_BINNING_DEF             0x0020

#define MT9M021_RESET                   0x00D9
#define MT9M021_STREAM_OFF              0x00D8
#define MT9M021_STREAM_ON               0x00DC
#define MT9M021_MASTER_MODE             0x10DC
#define MT9M021_TRIGGER_MODE            0x19D8

#define MT9M021_ANALOGUE_GAIN_MIN         0x0
#define MT9M021_ANALOGUE_GAIN_MAX         0x3
#define MT9M021_ANALOGUE_GAIN_DEF         0x0
#define MT9M021_ANALOGUE_GAIN_SHIFT       4
#define MT9M021_ANALOGUE_GAIN_MASK        0x0030

#define MT9M021_RESERVED_30EA           0x30EA
#define MT9M021_RESERVED_3180           0x3180
#define MT9M021_RESERVED_3ED6           0x3ED6
#define MT9M021_RESERVED_3ED8           0x3ED8
#define MT9M021_RESERVED_3EDA           0x3EDA
#define MT9M021_RESERVED_3EDC           0x3EDC
#define MT9M021_RESERVED_3EDE           0x3EDE
#define MT9M021_RESERVED_3EE0           0x3EE0
#define MT9M021_RESERVED_3EE2           0x3EE2
#define MT9M021_RESERVED_3EE4           0x3EE4

#endif