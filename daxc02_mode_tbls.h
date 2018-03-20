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

#ifndef __DAXC02_TABLES__
#define __DAXC02_TABLES__

#include <media/camera_common.h>
#include "daxc02.h"

#define MT9M021_TABLE_WAIT_MS   0
#define MT9M021_TABLE_END       1
#define MT9M021_MAX_RETRIES     3

static const struct reg_16 daxc02_mode_table_common[] = {
    /* Sequencer settings. */
    {MT9M021_SEQ_CTRL_PORT,     0x8000},
    {MT9M021_SEQ_CTRL_PORT,     0x3227},
    {MT9M021_SEQ_CTRL_PORT,     0x0101},
    {MT9M021_SEQ_CTRL_PORT,     0x0F25},
    {MT9M021_SEQ_CTRL_PORT,     0x0808},
    {MT9M021_SEQ_CTRL_PORT,     0x0227},
    {MT9M021_SEQ_CTRL_PORT,     0x0101},
    {MT9M021_SEQ_CTRL_PORT,     0x0837},
    {MT9M021_SEQ_CTRL_PORT,     0x2700},
    {MT9M021_SEQ_CTRL_PORT,     0x0138},
    {MT9M021_SEQ_CTRL_PORT,     0x2701},
    {MT9M021_SEQ_CTRL_PORT,     0x013A},
    {MT9M021_SEQ_CTRL_PORT,     0x2700},
    {MT9M021_SEQ_CTRL_PORT,     0x0125},
    {MT9M021_SEQ_CTRL_PORT,     0x0020},
    {MT9M021_SEQ_CTRL_PORT,     0x3C25},
    {MT9M021_SEQ_CTRL_PORT,     0x0040},
    {MT9M021_SEQ_CTRL_PORT,     0x3427},
    {MT9M021_SEQ_CTRL_PORT,     0x003F},
    {MT9M021_SEQ_CTRL_PORT,     0x2500},
    {MT9M021_SEQ_CTRL_PORT,     0x2037},
    {MT9M021_SEQ_CTRL_PORT,     0x2540},
    {MT9M021_SEQ_CTRL_PORT,     0x4036},
    {MT9M021_SEQ_CTRL_PORT,     0x2500},
    {MT9M021_SEQ_CTRL_PORT,     0x4031},
    {MT9M021_SEQ_CTRL_PORT,     0x2540},
    {MT9M021_SEQ_CTRL_PORT,     0x403D},
    {MT9M021_SEQ_CTRL_PORT,     0x6425},
    {MT9M021_SEQ_CTRL_PORT,     0x2020},
    {MT9M021_SEQ_CTRL_PORT,     0x3D64},
    {MT9M021_SEQ_CTRL_PORT,     0x2510},
    {MT9M021_SEQ_CTRL_PORT,     0x1037},
    {MT9M021_SEQ_CTRL_PORT,     0x2520},
    {MT9M021_SEQ_CTRL_PORT,     0x2010},
    {MT9M021_SEQ_CTRL_PORT,     0x2510},
    {MT9M021_SEQ_CTRL_PORT,     0x100F},
    {MT9M021_SEQ_CTRL_PORT,     0x2708},
    {MT9M021_SEQ_CTRL_PORT,     0x0802},
    {MT9M021_SEQ_CTRL_PORT,     0x2540},
    {MT9M021_SEQ_CTRL_PORT,     0x402D},
    {MT9M021_SEQ_CTRL_PORT,     0x2608},
    {MT9M021_SEQ_CTRL_PORT,     0x280D},
    {MT9M021_SEQ_CTRL_PORT,     0x1709},
    {MT9M021_SEQ_CTRL_PORT,     0x2600},
    {MT9M021_SEQ_CTRL_PORT,     0x2805},
    {MT9M021_SEQ_CTRL_PORT,     0x26A7},
    {MT9M021_SEQ_CTRL_PORT,     0x2807},
    {MT9M021_SEQ_CTRL_PORT,     0x2580},
    {MT9M021_SEQ_CTRL_PORT,     0x8029},
    {MT9M021_SEQ_CTRL_PORT,     0x1705},
    {MT9M021_SEQ_CTRL_PORT,     0x2500},
    {MT9M021_SEQ_CTRL_PORT,     0x4027},
    {MT9M021_SEQ_CTRL_PORT,     0x2222},
    {MT9M021_SEQ_CTRL_PORT,     0x1616},
    {MT9M021_SEQ_CTRL_PORT,     0x2726},
    {MT9M021_SEQ_CTRL_PORT,     0x2617},
    {MT9M021_SEQ_CTRL_PORT,     0x3626},
    {MT9M021_SEQ_CTRL_PORT,     0xA617},
    {MT9M021_SEQ_CTRL_PORT,     0x0326},
    {MT9M021_SEQ_CTRL_PORT,     0xA417},
    {MT9M021_SEQ_CTRL_PORT,     0x1F28},
    {MT9M021_SEQ_CTRL_PORT,     0x0526},
    {MT9M021_SEQ_CTRL_PORT,     0x2028},
    {MT9M021_SEQ_CTRL_PORT,     0x0425},
    {MT9M021_SEQ_CTRL_PORT,     0x2020},
    {MT9M021_SEQ_CTRL_PORT,     0x2700},
    {MT9M021_SEQ_CTRL_PORT,     0x2625},
    {MT9M021_SEQ_CTRL_PORT,     0x0000},
    {MT9M021_SEQ_CTRL_PORT,     0x171E},
    {MT9M021_SEQ_CTRL_PORT,     0x2500},
    {MT9M021_SEQ_CTRL_PORT,     0x0425},
    {MT9M021_SEQ_CTRL_PORT,     0x0020},
    {MT9M021_SEQ_CTRL_PORT,     0x2117},
    {MT9M021_SEQ_CTRL_PORT,     0x121B},
    {MT9M021_SEQ_CTRL_PORT,     0x1703},
    {MT9M021_SEQ_CTRL_PORT,     0x2726},
    {MT9M021_SEQ_CTRL_PORT,     0x2617},
    {MT9M021_SEQ_CTRL_PORT,     0x2828},
    {MT9M021_SEQ_CTRL_PORT,     0x0517},
    {MT9M021_SEQ_CTRL_PORT,     0x1A26},
    {MT9M021_SEQ_CTRL_PORT,     0x6017},
    {MT9M021_SEQ_CTRL_PORT,     0xAE25},
    {MT9M021_SEQ_CTRL_PORT,     0x0080},
    {MT9M021_SEQ_CTRL_PORT,     0x2700},
    {MT9M021_SEQ_CTRL_PORT,     0x2626},
    {MT9M021_SEQ_CTRL_PORT,     0x1828},
    {MT9M021_SEQ_CTRL_PORT,     0x002E},
    {MT9M021_SEQ_CTRL_PORT,     0x2A28},
    {MT9M021_SEQ_CTRL_PORT,     0x081E},
    {MT9M021_SEQ_CTRL_PORT,     0x4127},
    {MT9M021_SEQ_CTRL_PORT,     0x1010},
    {MT9M021_SEQ_CTRL_PORT,     0x0214},
    {MT9M021_SEQ_CTRL_PORT,     0x6060},
    {MT9M021_SEQ_CTRL_PORT,     0x0A14},
    {MT9M021_SEQ_CTRL_PORT,     0x6060},
    {MT9M021_SEQ_CTRL_PORT,     0x0B14},
    {MT9M021_SEQ_CTRL_PORT,     0x6060},
    {MT9M021_SEQ_CTRL_PORT,     0x0C14},
    {MT9M021_SEQ_CTRL_PORT,     0x6060},
    {MT9M021_SEQ_CTRL_PORT,     0x0D14},
    {MT9M021_SEQ_CTRL_PORT,     0x6060},
    {MT9M021_SEQ_CTRL_PORT,     0x0217},
    {MT9M021_SEQ_CTRL_PORT,     0x3C14},
    {MT9M021_SEQ_CTRL_PORT,     0x0060},
    {MT9M021_SEQ_CTRL_PORT,     0x0A14},
    {MT9M021_SEQ_CTRL_PORT,     0x0060},
    {MT9M021_SEQ_CTRL_PORT,     0x0B14},
    {MT9M021_SEQ_CTRL_PORT,     0x0060},
    {MT9M021_SEQ_CTRL_PORT,     0x0C14},
    {MT9M021_SEQ_CTRL_PORT,     0x0060},
    {MT9M021_SEQ_CTRL_PORT,     0x0D14},
    {MT9M021_SEQ_CTRL_PORT,     0x0060},
    {MT9M021_SEQ_CTRL_PORT,     0x0811},
    {MT9M021_SEQ_CTRL_PORT,     0x2500},
    {MT9M021_SEQ_CTRL_PORT,     0x1027},
    {MT9M021_SEQ_CTRL_PORT,     0x0010},
    {MT9M021_SEQ_CTRL_PORT,     0x2F6F},
    {MT9M021_SEQ_CTRL_PORT,     0x0F3E},
    {MT9M021_SEQ_CTRL_PORT,     0x2500},
    {MT9M021_SEQ_CTRL_PORT,     0x0827},
    {MT9M021_SEQ_CTRL_PORT,     0x0008},
    {MT9M021_SEQ_CTRL_PORT,     0x3066},
    {MT9M021_SEQ_CTRL_PORT,     0x3225},
    {MT9M021_SEQ_CTRL_PORT,     0x0008},
    {MT9M021_SEQ_CTRL_PORT,     0x2700},
    {MT9M021_SEQ_CTRL_PORT,     0x0830},
    {MT9M021_SEQ_CTRL_PORT,     0x6631},
    {MT9M021_SEQ_CTRL_PORT,     0x3D64},
    {MT9M021_SEQ_CTRL_PORT,     0x2508},
    {MT9M021_SEQ_CTRL_PORT,     0x083D},
    {MT9M021_SEQ_CTRL_PORT,     0xFF3D},
    {MT9M021_SEQ_CTRL_PORT,     0x2A27},
    {MT9M021_SEQ_CTRL_PORT,     0x083F},
    {MT9M021_SEQ_CTRL_PORT,     0x2C00},

    /* PLL settings. */
    {MT9M021_VT_PIX_CLK_DIV,    MT9M021_PLL_P1},
    {MT9M021_VT_SYS_CLK_DIV,    MT9M021_PLL_P2},
    {MT9M021_PRE_PLL_CLK_DIV,   MT9M021_PLL_N},
    {MT9M021_PLL_MULTIPLIER,    MT9M021_PLL_M},
    {MT9M021_TABLE_WAIT_MS,     200},
    {MT9M021_DIGITAL_TEST,      0x0000},

    /* Column correction. */
    {MT9M021_RESET_REG,         MT9M021_STREAM_OFF},
    {MT9M021_COLUMN_CORRECTION, 0x0007},
    {MT9M021_TABLE_WAIT_MS,     200},
    {MT9M021_RESET_REG,         MT9M021_STREAM_ON},
    {MT9M021_TABLE_WAIT_MS,     200},
    {MT9M021_RESET_REG,         MT9M021_STREAM_OFF},
    {MT9M021_TABLE_WAIT_MS,     200},
    {MT9M021_COLUMN_CORRECTION, 0xE007},
    {MT9M021_TABLE_WAIT_MS,     200},
    {MT9M021_TEST_RAW_MODE,     0x0000},
    {MT9M021_RESERVED_30EA,     0x0C00},
    {MT9M021_DARK_CTRL,         0x0404},
    {MT9M021_DATA_PEDESTAL,     0x012C},
    {MT9M021_RESERVED_3180,     0x8000},
    {MT9M021_RESERVED_3ED6,     0x00FD},
    {MT9M021_RESERVED_3ED8,     0x0FFF},
    {MT9M021_RESERVED_3EDA,     0x0003},
    {MT9M021_RESERVED_3EDC,     0xF87A},
    {MT9M021_RESERVED_3EDE,     0xE075},
    {MT9M021_RESERVED_3EE0,     0x077C},
    {MT9M021_RESERVED_3EE2,     0xA4EB},
    {MT9M021_RESERVED_3EE4,     0xD208},

    /** Disable embedded data. */
    {MT9M021_EMBEDDED_DATA_CTRL, 0x1802},

    {MT9M021_TABLE_END,         0x0000}
};

static const struct reg_16 daxc02_mode_1280x720[] = {
    /** Configure frame size. */
    {MT9M021_Y_ADDR_START,      0x0078},
    {MT9M021_X_ADDR_START,      0x0001},
    {MT9M021_Y_ADDR_END,        0x0347},
    {MT9M021_X_ADDR_END,        0x0500},
    {MT9M021_LINE_LENGTH_PCK,   MT9M021_LLP_RECOMMENDED},
    {MT9M021_X_ODD_INC,         0x0001},
    {MT9M021_Y_ODD_INC,         0x0001},
    {MT9M021_READ_MODE,         0x0000},
    {MT9M021_DIGITAL_BINNING,   MT9M021_BINNING_DEF},
    {MT9M021_READ_SPEED,        0x0010},

    {MT9M021_TABLE_END,         0x0000}
};

enum {
    MT9M021_MODE_1280X720,
};

static const struct reg_16 *mode_table[] = {
    [MT9M021_MODE_1280X720]         = daxc02_mode_1280x720,
};


static const int daxc02_60fps[] = {
    60,
};

static const struct camera_common_frmfmt daxc02_frmfmt[] = {
    {{1280, 720},    daxc02_60fps,    1, 0,    MT9M021_MODE_1280X720},
};

#endif  /* __MT9M021_TABLES__ */

