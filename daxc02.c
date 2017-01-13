/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG 1

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>

#include <media/camera_common.h>
#include <stdbool.h>
#include <linux/kernel.h>

#include "mt9m021_mode_tbls.h"

/***************************************************
        MT9M021 Defines
****************************************************/

#define MT9M021_I2C_ADDR                0x10
#define MT9M021_EXT_FREQ                20250000
#define MT9M021_TARGET_FREQ             74250000

#define MT9M021_PIXEL_ARRAY_WIDTH       1280
#define MT9M021_PIXEL_ARRAY_HEIGHT      960

#define MT9M021_ROW_START_MIN           0
#define MT9M021_ROW_START_MAX           960
#define MT9M021_ROW_START_DEF           0
#define MT9M021_COLUMN_START_MIN        0
#define MT9M021_COLUMN_START_MAX        1280
#define MT9M021_COLUMN_START_DEF        0
#define MT9M021_WINDOW_HEIGHT_MIN       2
#define MT9M021_WINDOW_HEIGHT_MAX       960
#define MT9M021_WINDOW_HEIGHT_DEF       960
#define MT9M021_WINDOW_WIDTH_MIN        2
#define MT9M021_WINDOW_WIDTH_MAX        1280
#define MT9M021_WINDOW_WIDTH_DEF        1280
#define MT9M021_ENABLE                  1
#define MT9M021_DISABLE                 0

#define MT9M021_CHIP_ID_REG             0x3000
#define MT9M021_CHIP_ID                 0x2401

#define MT9M021_RESET_REG               0x301A
#define MT9M021_SEQ_CTRL_PORT           0x3088
#define MT9M021_SEQ_DATA_PORT           0x3086
#define MT9M021_ANALOG_REG              0x3ED6
#define MT9M021_TEST_RAW_MODE           0x307A
#define MT9M021_DARK_CTRL               0x3044
#define MT9M021_DATA_PEDESTAL           0x301E
#define MT9M021_COLUMN_CORRECTION       0x30D4

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
#define MT9M021_X_ODD_INC               0x30A2
#define MT9M021_Y_ODD_INC               0x30A6
#define MT9M021_READ_MODE               0x3040
#define MT9M021_TEST_PATTERN            0x3070
#define MT9M021_LLP_RECOMMENDED         1650
#define MT9M021_DIGITAL_BINNING         0x3032
#define MT9M021_HOR_AND_VER_BIN         0x0022
#define MT9M021_HOR_BIN                 0x0011
#define MT9M021_DISABLE_BINNING         0x0000

#define MT9M021_AE_CTRL_REG             0x3100
#define MT9M021_EMBEDDED_DATA_CTRL      0x3064

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

#define MT9M021_RESET                   0x00D9
#define MT9M021_STREAM_OFF              0x00D8
#define MT9M021_STREAM_ON               0x00DC

#define MT9M021_ANALOG_GAIN_MIN         0x0
#define MT9M021_ANALOG_GAIN_MAX         0x3
#define MT9M021_ANALOG_GAIN_DEF         0x0
#define MT9M021_ANALOG_GAIN_SHIFT       4
#define MT9M021_ANALOG_GAIN_MASK        0x0030

#define MT9M021_GLOBAL_GAIN_MIN         0x00
#define MT9M021_GLOBAL_GAIN_MAX         0xFF
#define MT9M021_GLOBAL_GAIN_DEF         0x20

#define MT9M021_EXPOSURE_MIN            1
#define MT9M021_EXPOSURE_MAX            0x02A0
#define MT9M021_EXPOSURE_DEF            0x0100

#define V4L2_CID_GAIN_RED           (V4L2_CID_USER_BASE | 0x1001)
#define V4L2_CID_GAIN_GREEN1        (V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_GAIN_GREEN2        (V4L2_CID_USER_BASE | 0x1003)
#define V4L2_CID_GAIN_BLUE          (V4L2_CID_USER_BASE | 0x1004)
#define V4L2_CID_ANALOG_GAIN        (V4L2_CID_USER_BASE | 0x1005)

enum {
    MT9M021_COLOR_VERSION,
    MT9M021_MONOCHROME_VERSION,
};

struct mt9m021_frame_size {
    uint16_t width;
    uint16_t height;
};

static unsigned int mt9m021_seq_data[133] = {
    0x3227, 0x0101, 0x0F25, 0x0808, 0x0227, 0x0101, 0x0837, 0x2700,
    0x0138, 0x2701, 0x013A, 0x2700, 0x0125, 0x0020, 0x3C25, 0x0040,
    0x3427, 0x003F, 0x2500, 0x2037, 0x2540, 0x4036, 0x2500, 0x4031,
    0x2540, 0x403D, 0x6425, 0x2020, 0x3D64, 0x2510, 0x1037, 0x2520,
    0x2010, 0x2510, 0x100F, 0x2708, 0x0802, 0x2540, 0x402D, 0x2608,
    0x280D, 0x1709, 0x2600, 0x2805, 0x26A7, 0x2807, 0x2580, 0x8029,
    0x1705, 0x2500, 0x4027, 0x2222, 0x1616, 0x2726, 0x2617, 0x3626,
    0xA617, 0x0326, 0xA417, 0x1F28, 0x0526, 0x2028, 0x0425, 0x2020,
    0x2700, 0x2625, 0x0000, 0x171E, 0x2500, 0x0425, 0x0020, 0x2117,
    0x121B, 0x1703, 0x2726, 0x2617, 0x2828, 0x0517, 0x1A26, 0x6017,
    0xAE25, 0x0080, 0x2700, 0x2626, 0x1828, 0x002E, 0x2A28, 0x081E,
    0x4127, 0x1010, 0x0214, 0x6060, 0x0A14, 0x6060, 0x0B14, 0x6060,
    0x0C14, 0x6060, 0x0D14, 0x6060, 0x0217, 0x3C14, 0x0060, 0x0A14,
    0x0060, 0x0B14, 0x0060, 0x0C14, 0x0060, 0x0D14, 0x0060, 0x0811,
    0x2500, 0x1027, 0x0010, 0x2F6F, 0x0F3E, 0x2500, 0x0827, 0x0008,
    0x3066, 0x3225, 0x0008, 0x2700, 0x0830, 0x6631, 0x3D64, 0x2508,
    0x083D, 0xFF3D, 0x2A27, 0x083F, 0x2C00
};

static unsigned int mt9m021_analog_setting[8] = {
    0x00FD, 0x0FFF, 0x0003, 0xF87A, 0xE075, 0x077C, 0xA4EB, 0xD208
};

/*
 * struct mt9m021_platform_data - MT9M021 platform data
 * @ext_freq: Input clock frequency
 * @target_freq: Pixel clock frequency
 * @version: color or monochrome
 */
struct mt9m021_platform_data {
    int ext_freq;
    int target_freq;
    int version;
};

/**
 * PLL Dividers
 *
 * Calculated according to the following formula:
 *
 *    target_freq = (ext_freq x M) / (N x P1 x P2)
 *    VCO_freq    = (ext_freq x M) / N
 *
 * And subject to the following limitations:
 *
 *    Limitations of PLL parameters
 *    -----------------------------
 *    32        >=  M           >=  384
 *    1         >=  N           >=  64
 *    1         >=  P1          >=  16
 *    4         >=  P2          >=  16
 *    384MHz    >=  VCO_freq    >=  768MHz
 *
 */
struct mt9m021_pll_divs {
    uint32_t ext_freq;
    uint32_t target_freq;
    uint16_t m;
    uint16_t n;
    uint16_t p1;
    uint16_t p2;
};


static struct mt9m021_pll_divs mt9m021_divs[] = {
    /* ext_freq     target_freq     M       N       p1      p2 */
    {20250000,      74250000,       44,     2,      1,      6},
    {24000000,      48000000,       32,     2,      2,      4},
    {24000000,      66000000,       44,     2,      2,      4},
    {24000000,      74250000,       99,     2,      4,      4},
    {27000000,      74250000,       44,     2,      1,      8},
    {48000000,      48000000,       40,     5,      2,      4}
};

/*
MT9M021_TEST_PATTERN
0 = Disabled. Normal operation. Generate output data from pixel array
1 = Solid color test pattern",
2 = color bar test pattern",
3 = Fade to gray color bar test pattern",
256 = Walking 1s test pattern (12 bit)"
*/
static const char * const mt9m021_test_pattern_menu[] = {
    "0:Disabled",
    "1:Solid color test pattern",
    "2:color bar test pattern",
    "3:Fade to gray color bar test pattern",
    "256:Walking 1s test pattern (12 bit)"
};


/***************************************************
        DAX-C02 Private Structure
****************************************************/

struct daxc02 {
    struct camera_common_power_rail     power;
    int                                 numctrls;
    struct v4l2_ctrl_handler            ctrl_handler;
    struct i2c_client                   *i2c_client;
    struct v4l2_subdev                  *subdev;
    struct media_pad                    pad;

    int32_t                             group_hold_prev;
    bool                                group_hold_en;
    struct camera_common_data           *s_data;
    struct camera_common_pdata          *pdata;

    struct v4l2_rect                    crop;  /* Sensor window */
    struct v4l2_mbus_framefmt           format;
    struct mt9m021_platform_data        *mt9m021_pdata;
    struct mt9m021_pll_divs             *pll;
    int                                 power_count;
    enum v4l2_exposure_auto_type        autoexposure;

    struct v4l2_ctrl                    *ctrls[];
};


/***************************************************
        Prototypes
****************************************************/

static int daxc02_s_ctrl(struct v4l2_ctrl *ctrl);
static int daxc02_power_on(struct camera_common_data *s_data);
static int daxc02_power_off(struct camera_common_data *s_data);
static int daxc02_power_put(struct daxc02 *priv);
static int daxc02_power_get(struct daxc02 *priv);
static inline int mt9m021_read(struct i2c_client *client, uint16_t addr);
static inline int mt9m021_read_reg(struct camera_common_data *s_data, uint16_t addr, uint8_t *val);
static int mt9m021_write(struct i2c_client *client, uint16_t addr, uint16_t val);
static int mt9m021_write_reg(struct camera_common_data *s_data, uint16_t addr, uint8_t val);
static int mt9m021_sequencer_settings(struct i2c_client *client);
static int mt9m021_col_correction(struct i2c_client *client);
static int mt9m021_rev2_settings(struct i2c_client *client);
static int mt9m021_pll_setup(struct i2c_client *client);
static int mt9m021_set_size(struct i2c_client *client, struct mt9m021_frame_size *frame);
static int mt9m021_is_streaming(struct i2c_client *client);
static int mt9m021_set_autoexposure( struct i2c_client *client, enum v4l2_exposure_auto_type ae_mode );
static int mt9m021_s_stream(struct v4l2_subdev *sd, int enable);
static int daxc02_g_input_status(struct v4l2_subdev *sd, uint32_t *status);
static int mt9m021_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_mbus_code_enum *code);
static int mt9m021_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_frame_size_enum *fse);
static struct v4l2_mbus_framefmt * __mt9m021_get_pad_format(struct daxc02 *priv, struct v4l2_subdev_fh *fh, unsigned int pad, uint32_t which);
static struct v4l2_rect * __mt9m021_get_pad_crop(struct daxc02 *priv, struct v4l2_subdev_fh *fh, unsigned int pad, uint32_t which);
static int mt9m021_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt);
static int mt9m021_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format);
static int mt9m021_get_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_crop *crop);
static int mt9m021_set_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_crop *crop);
static int mt9m021_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
static int mt9m021_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
static struct camera_common_pdata *daxc02_parse_dt(struct i2c_client *client);
static int daxc02_ctrls_init(struct daxc02 *priv);
static int daxc02_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int daxc02_remove(struct i2c_client *client);


/***************************************************
        V4L2 Control Configuration
****************************************************/

static int daxc02_s_ctrl(struct v4l2_ctrl *ctrl)
{
    uint16_t reg16;
    int ret = 0;

    struct daxc02 *priv = container_of(ctrl->handler, struct daxc02, ctrl_handler);
    struct i2c_client *client = v4l2_get_subdevdata(priv->subdev);

    if (priv->power.state == SWITCH_OFF) return 0;

    switch (ctrl->id) {
        case V4L2_CID_EXPOSURE_AUTO:
            dev_dbg(&client->dev, "%s: V4L2_CID_EXPOSURE_AUTO\n", __func__);
            ret = mt9m021_set_autoexposure(client, (enum v4l2_exposure_auto_type)ctrl->val);
            if(ret < 0) return ret;
            break;

        case V4L2_CID_EXPOSURE:
            dev_dbg(&client->dev, "%s: V4L2_CID_EXPOSURE\n", __func__);
            ret = mt9m021_write(client, MT9M021_COARSE_INT_TIME, ctrl->val);
            if(ret < 0) return ret;
            return mt9m021_write(client, MT9M021_COARSE_INT_TIME_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN\n", __func__);
            ret = mt9m021_write(client, MT9M021_GLOBAL_GAIN, ctrl->val);
            if(ret < 0) return ret;
            return mt9m021_write(client, MT9M021_GLOBAL_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_GREEN1:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_GREEN1\n", __func__);
            ret = mt9m021_write(client, MT9M021_GREEN1_GAIN, ctrl->val);
            if(ret < 0) return ret;
            return mt9m021_write(client, MT9M021_GREEN1_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_RED:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_RED\n", __func__);
            ret = mt9m021_write(client, MT9M021_RED_GAIN, ctrl->val);
            if(ret < 0) return ret;
            return mt9m021_write(client, MT9M021_RED_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_BLUE:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_BLUE\n", __func__);
            ret = mt9m021_write(client, MT9M021_BLUE_GAIN, ctrl->val);
            if(ret < 0) return ret;
            return mt9m021_write(client, MT9M021_BLUE_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_GREEN2:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_GREEN2\n", __func__);
            ret = mt9m021_write(client, MT9M021_GREEN2_GAIN, ctrl->val);
            if(ret < 0) return ret;
            return mt9m021_write(client, MT9M021_GREEN2_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_ANALOG_GAIN:
            dev_dbg(&client->dev, "%s: V4L2_CID_ANALOG_GAIN\n", __func__);
            reg16 = mt9m021_read(client, MT9M021_DIGITAL_TEST);
            reg16 = ( reg16 & ~MT9M021_ANALOG_GAIN_MASK ) |
                ( ( ctrl->val << MT9M021_ANALOG_GAIN_SHIFT ) & MT9M021_ANALOG_GAIN_MASK );
            return mt9m021_write(client, MT9M021_DIGITAL_TEST, reg16);
            break;

        case V4L2_CID_HFLIP:
            dev_dbg(&client->dev, "%s: V4L2_CID_HFLIP\n", __func__);
            reg16 = mt9m021_read(client, MT9M021_READ_MODE);
            if (ctrl->val)
            {
                reg16 |= 0x4000;
                ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
                if (ret < 0) return ret;
                break;
            }
            reg16 &= 0xbfff;
            ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
            if (ret < 0) return ret;
            break;

        case V4L2_CID_VFLIP:
            dev_dbg(&client->dev, "%s: V4L2_CID_VFLIP\n", __func__);
            reg16 = mt9m021_read(client, MT9M021_READ_MODE);
            if (ctrl->val)
            {
                reg16 |= 0x8000;
                ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
                if (ret < 0) return ret;
                break;
            }
            reg16 &= 0x7fff;
            ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
            if (ret < 0) return ret;
            break;

        case V4L2_CID_TEST_PATTERN:
            dev_dbg(&client->dev, "%s: V4L2_CID_TEST_PATTERN\n", __func__);
            if (!ctrl->val)
            {
                ret = mt9m021_write(client, MT9M021_TEST_PATTERN, 0x0000);
                if(ret < 0) return ret;
            }
            ret = mt9m021_write(client, MT9M021_TEST_PATTERN, ctrl->val);
            if(ret < 0) return ret;
            break;

        case V4L2_CID_FRAME_LENGTH:
            dev_err(&client->dev, "%s: V4L2_CID_FRAME_LENGTH not implemented.\n", __func__);
            break;

        case V4L2_CID_COARSE_TIME:
            dev_err(&client->dev, "%s: V4L2_CID_COARSE_TIME not implemented.\n", __func__);
            break;

        case V4L2_CID_COARSE_TIME_SHORT:
            dev_err(&client->dev, "%s: V4L2_CID_COARSE_TIME_SHORT not implemented.\n", __func__);
            break;

        case V4L2_CID_GROUP_HOLD:
            dev_err(&client->dev, "%s: V4L2_CID_GROUP_HOLD not implemented.\n", __func__);
            break;

        case V4L2_CID_HDR_EN:
            dev_err(&client->dev, "%s: V4L2_CID_HDR_EN not implemented.\n", __func__);
            break;

        case V4L2_CID_OTP_DATA:
            dev_err(&client->dev, "%s: V4L2_CID_OTP_DATA not implemented.\n", __func__);
            break;

        case V4L2_CID_FUSE_ID:
            dev_err(&client->dev, "%s: V4L2_CID_FUSE_ID not implemented.\n", __func__);
            break;

        default:
            dev_err(&client->dev, "%s: unknown ctrl id.\n", __func__);
            return -EINVAL;
    }

    return ret;
}

static int daxc02_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
    struct daxc02 *priv = container_of(ctrl->handler, struct daxc02, ctrl_handler);
    int err = 0;
    struct i2c_client *client = v4l2_get_subdevdata(priv->subdev);

    if (priv->power.state == SWITCH_OFF)
        return 0;

    switch (ctrl->id) {
    case V4L2_CID_EEPROM_DATA:
        dev_err(&client->dev, "%s: V4L2_CID_EEPROM_DATA not implemented.\n", __func__);
        break;

    default:
            dev_err(&client->dev, "%s: unknown ctrl id.\n", __func__);
            return -EINVAL;
    }

    return err;
}

static const struct v4l2_ctrl_ops daxc02_ctrl_ops = {
    .g_volatile_ctrl    = daxc02_g_volatile_ctrl,
    .s_ctrl             = daxc02_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN,
        .name           = "Gain",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = V4L2_CTRL_FLAG_SLIDER,
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_TEST_PATTERN,
        .type           = V4L2_CTRL_TYPE_MENU,
        .name           = "Test Pattern",
        .min            = 0,
        .max            = ARRAY_SIZE(mt9m021_test_pattern_menu) - 1,
        .step           = 0,
        .def            = 0,
        .flags          = 0,
        .menu_skip_mask = 0,
        .qmenu          = mt9m021_test_pattern_menu,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN_GREEN1,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Green (R)",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .step           = 1,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .flags          = 0,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN_RED,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Red",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .step           = 1,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .flags          = 0,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN_BLUE,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Blue",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .step           = 1,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .flags          = 0,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN_GREEN2,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Green (B)",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .step           = 1,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .flags          = 0,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_ANALOG_GAIN,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Column",
        .min            = MT9M021_ANALOG_GAIN_MIN,
        .max            = MT9M021_ANALOG_GAIN_MAX,
        .step           = 1,
        .def            = MT9M021_ANALOG_GAIN_DEF,
        .flags          = 0,
    },
    // Begin not implemented controls
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_FRAME_LENGTH,
        .name = "Frame Length",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0,
        .max = 0x7fff,
        .def = 0x07C0,
        .step = 1,
    },
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_COARSE_TIME,
        .name = "Coarse Time",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0x0002,
        .max = 0x7ff9,
        .def = 0x7fba,
        .step = 1,
    },
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_COARSE_TIME_SHORT,
        .name = "Coarse Time Short",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0x0002,
        .max = 0x7ff9,
        .def = 0x7fba,
        .step = 1,
    },
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_GROUP_HOLD,
        .name = "Group Hold",
        .type = V4L2_CTRL_TYPE_INTEGER_MENU,
        .min = 0,
        .max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
        .menu_skip_mask = 0,
        .def = 0,
        .qmenu_int = switch_ctrl_qmenu,
    },
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_HDR_EN,
        .name = "HDR enable",
        .type = V4L2_CTRL_TYPE_INTEGER_MENU,
        .min = 0,
        .max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
        .menu_skip_mask = 0,
        .def = 0,
        .qmenu_int = switch_ctrl_qmenu,
    },
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_EEPROM_DATA,
        .name = "EEPROM Data",
        .type = V4L2_CTRL_TYPE_STRING,
        .flags = V4L2_CTRL_FLAG_VOLATILE,
        .min = 0,
        .max = 2048,
        .step = 2,
    },
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_OTP_DATA,
        .name = "OTP Data",
        .type = V4L2_CTRL_TYPE_STRING,
        .flags = V4L2_CTRL_FLAG_READ_ONLY,
        .min = 0,
        .max = 1024,
        .step = 2,
    },
    {
        .ops = &daxc02_ctrl_ops,
        .id = V4L2_CID_FUSE_ID,
        .name = "Fuse ID",
        .type = V4L2_CTRL_TYPE_STRING,
        .flags = V4L2_CTRL_FLAG_READ_ONLY,
        .min = 0,
        .max = 16,
        .step = 2,
    },
};


/***************************************************
        DAX-C02 Power Functions
****************************************************/

static int daxc02_power_on(struct camera_common_data *s_data)
{
    int err = 0;
    struct daxc02 *priv = (struct daxc02 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;
    struct i2c_client *client = s_data->i2c_client;

    dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

    if (priv->pdata && priv->pdata->power_on)
    {
        err = priv->pdata->power_on(pw);
        if (err) pr_err("%s failed.\n", __func__);
        else pw->state = SWITCH_ON;
        return err;
    }


    /* sleeps calls in the sequence below are for internal device
     * signal propagation as specified by sensor vendor */

    if (pw->dvdd) err = regulator_enable(pw->dvdd);     // 1.2V
    if (err) goto daxc02_dvdd_fail;

    usleep_range(5, 10);
    if (pw->avdd) err = regulator_enable(pw->avdd);     // 2.8V
    if (err) goto daxc02_avdd_fail;

    usleep_range(5, 10);
    if (pw->iovdd) err = regulator_enable(pw->iovdd);   // 1.8V
    if (err) goto daxc02_iovdd_fail;

    /* a power on reset is generated after core power becomes stable */
    usleep_range(2000, 2010);

    /* soft reset */
    err = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_RESET);
    if(err < 0) return err;
    msleep(200);

    usleep_range(1350, 1360);

    pw->state = SWITCH_ON;
    return 0;

    daxc02_iovdd_fail:
        regulator_disable(pw->avdd);

    daxc02_avdd_fail:
        regulator_disable(pw->dvdd);

    daxc02_dvdd_fail:
        pr_err("%s failed.\n", __func__);
        return -ENODEV;
}

static int daxc02_power_off(struct camera_common_data *s_data)
{
    int err = 0;
    struct daxc02 *priv = (struct daxc02 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;

    dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

    if (priv->pdata && priv->pdata->power_on)
    {
        err = priv->pdata->power_off(pw);
        if (!err) pw->state = SWITCH_OFF;
        else pr_err("%s failed.\n", __func__);
        return err;
    }

    usleep_range(2000, 2010);

    if (pw->iovdd) regulator_disable(pw->iovdd);
    usleep_range(5, 10);
    if (pw->avdd) regulator_disable(pw->avdd);
    usleep_range(5, 10);
    if (pw->dvdd) regulator_disable(pw->dvdd);

    return 0;
}

static int daxc02_power_put(struct daxc02 *priv)
{
    struct camera_common_power_rail *pw = &priv->power;
    dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

    if (unlikely(!pw)) return -EFAULT;

    if (likely(pw->iovdd)) regulator_put(pw->iovdd);

    if (likely(pw->avdd)) regulator_put(pw->avdd);

    if (likely(pw->dvdd)) regulator_put(pw->dvdd);

    pw->avdd = NULL;
    pw->iovdd = NULL;
    pw->dvdd = NULL;

    return 0;
}

static int daxc02_power_get(struct daxc02 *priv)
{
    struct camera_common_power_rail *pw = &priv->power;
    struct camera_common_pdata *pdata = priv->pdata;
    const char *mclk_name;
    const char *parentclk_name;
    struct clk *parent;
    int err = 0;

    dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

    mclk_name = priv->pdata->mclk_name ? priv->pdata->mclk_name : "cam_mclk1";
    pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
    if (IS_ERR(pw->mclk))
    {
        dev_err(&priv->i2c_client->dev, "unable to get clock %s\n", mclk_name);
        return PTR_ERR(pw->mclk);
    }

    parentclk_name = priv->pdata->parentclk_name;
    if (parentclk_name)
    {
        parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
        if (IS_ERR(parent)) dev_err(&priv->i2c_client->dev, "unable to get parent clcok %s", parentclk_name);
        else clk_set_parent(pw->mclk, parent);
    }

    /* 1.2v */
    err |= camera_common_regulator_get(priv->i2c_client, &pw->dvdd, pdata->regulators.dvdd);
    /* analog 2.8v */
    err |= camera_common_regulator_get(priv->i2c_client, &pw->avdd, pdata->regulators.avdd);
    /* IO 1.8v */
    err |= camera_common_regulator_get(priv->i2c_client, &pw->iovdd, pdata->regulators.iovdd);

    pw->state = SWITCH_OFF;
    return err;
}


/***************************************************
        MT9M021 Helper Functions
****************************************************/

static inline int mt9m021_read(struct i2c_client *client, uint16_t addr)
{
    struct i2c_msg msg[2];
    uint8_t buf[2];
    uint16_t __addr;
    uint16_t ret;

    /* 16 bit addressable register */
    __addr = cpu_to_be16(addr);

    msg[0].addr     = client->addr;
    msg[0].flags    = 0;
    msg[0].len      = 2;
    msg[0].buf      =(uint8_t *)&__addr;

    msg[1].addr     = client->addr;
    msg[1].flags    = I2C_M_RD; // 1
    msg[1].len      = 2;
    msg[1].buf      = buf;

    ret = i2c_transfer(client->adapter, msg, 2);

    if (ret < 0)
    {
        dev_err(&client->dev, "Read from offset 0x%x error %d", addr, ret);
        return ret;
    }

    dev_dbg(&client->dev, "daxc02: read offset 0x%x from %x@i2c%d, result 0x%x%x\n", addr, client->addr, i2c_adapter_id(client->adapter), buf[0], buf[1]);

    return (buf[0] << 8) | buf[1];
}


static inline int mt9m021_read_reg(struct camera_common_data *s_data, uint16_t addr, uint8_t *val)
{
    struct i2c_client *client = s_data->i2c_client;
    dev_dbg(&client->dev, "%s\n", __func__);
    *val = (uint8_t)mt9m021_read(client, addr);
    return 0;
}

static int mt9m021_write(struct i2c_client *client, uint16_t addr, uint16_t data)
{
    struct i2c_msg msg;
    uint8_t buf[4];
    uint16_t __addr, __data;
    int ret;

    dev_dbg(&client->dev, "%s: 0x%x to 0x%x\n", __func__, data, addr);

    /* 16-bit addressable register */

    __addr = cpu_to_be16(addr);
    __data = cpu_to_be16(data);

    buf[0] = __addr & 0xff;
    buf[1] = __addr >> 8;
    buf[2] = __data & 0xff;
    buf[3] = __data >> 8;
    msg.addr  = client->addr;
    msg.flags = 0;
    msg.len   = 4;
    msg.buf   = buf;

    /* i2c_transfer returns message length, but function should return 0 */
    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret == 1) return 0;

    v4l_err(client, "Write failed at 0x%x error %d\n", addr, ret);
    return ret;
}


static inline int mt9m021_write_reg(struct camera_common_data *s_data, uint16_t addr, uint8_t val)
{
    struct i2c_client *client = s_data->i2c_client;
    dev_dbg(&client->dev, "%s\n", __func__);
    return mt9m021_write(client, addr, val);
}

/**
 * mt9m021_sequencer_settings
 * @client: pointer to the i2c client
 *
 */
static int mt9m021_sequencer_settings(struct i2c_client *client)
{
    int i, ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    ret = mt9m021_write(client, MT9M021_SEQ_CTRL_PORT, 0x8000);
    if (ret < 0) return ret;

    for(i = 0; i < ARRAY_SIZE(mt9m021_seq_data); i++)
    {
        ret = mt9m021_write(client, MT9M021_SEQ_DATA_PORT, mt9m021_seq_data[i]);
        if (ret < 0) return ret;
    }

    return ret;
}

/**
 * mt9m021_col_correction - retrigger column correction
 * @client: pointer to the i2c client
 *
 */
static int mt9m021_col_correction(struct i2c_client *client)
{
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    /* Disable Streaming */
    ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
    if (ret < 0) return ret;

    /* Disable column correction */
    ret = mt9m021_write(client, MT9M021_COLUMN_CORRECTION, 0x0000);
    if (ret < 0) return ret;
    msleep(200);

    /* Enable Streaming */
    ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_ON);
    if (ret < 0) return ret;
    msleep(200);

    /* Disable Streaming */
    ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
    if (ret < 0) return ret;

    /* Enable column correction */
    ret = mt9m021_write(client, MT9M021_COLUMN_CORRECTION, 0x0001);
    if (ret < 0) return ret;
    msleep(200);

    return ret;
}

/**
 * mt9m021_rev2_settings
 * @client: pointer to the i2c client
 *
 */
static int mt9m021_rev2_settings(struct i2c_client *client)
{
    int ret;
    int i;

    dev_dbg(&client->dev, "%s\n", __func__);

    ret = mt9m021_write(client, MT9M021_TEST_RAW_MODE, 0x0000);
    if (ret < 0) return ret;

    ret = mt9m021_write(client, 0x30EA, 0x0C00);
    if (ret < 0) return ret;

    ret = mt9m021_write(client, MT9M021_DARK_CTRL, 0x0404);
    if (ret < 0) return ret;

    ret = mt9m021_write(client, MT9M021_DATA_PEDESTAL, 0x012C);
    if (ret < 0) return ret;

    ret = mt9m021_write(client, 0x3180, 0x8000);
    if (ret < 0) return ret;

    ret = mt9m021_write(client, MT9M021_COLUMN_CORRECTION, 0xE007);
    if (ret < 0) return ret;

    ret = mt9m021_write(client, MT9M021_FINE_INT_TIME, 0x0000);
    if (ret < 0) return ret;

    for(i = 0; i < ARRAY_SIZE(mt9m021_analog_setting); i++)
    {
        ret = mt9m021_write(client, MT9M021_ANALOG_REG + 2*i, mt9m021_analog_setting[i]);
        if (ret < 0) return ret;
    }

    return ret;
}

/**
 * mt9m021_pll_setup - enable the sensor pll
 * @client: pointer to the i2c client
 *
 */
static int mt9m021_pll_setup(struct i2c_client *client)
{
    int ret;
    int i;
    struct camera_common_data *common_data = to_camera_common_data(client);
    struct daxc02 *priv = common_data->priv;
    struct mt9m021_platform_data *mt9m021 = priv->mt9m021_pdata;

    dev_dbg(&client->dev, "%s\n", __func__);

    for (i = 0; i < ARRAY_SIZE(mt9m021_divs); i++)
    {
        if (mt9m021_divs[i].ext_freq == mt9m021->ext_freq &&
            mt9m021_divs[i].target_freq == mt9m021->target_freq)
        {
            priv->pll = &mt9m021_divs[i];
            goto out;
        }
    }
    dev_err(&client->dev, "Couldn't find PLL dividers for ext_freq = %d, target_freq = %d\n",
            mt9m021->ext_freq, mt9m021->target_freq);
    return -EINVAL;

    out:
        dev_dbg(&client->dev, "PLL settings:M = %d, N = %d, P1 = %d, P2 = %d",
                priv->pll->m, priv->pll->n, priv->pll->p1, priv->pll->p2);
        ret = mt9m021_write(client, MT9M021_VT_SYS_CLK_DIV, priv->pll->p1);
        if (ret < 0) return ret;
        ret = mt9m021_write(client, MT9M021_VT_PIX_CLK_DIV, priv->pll->p2);
        if (ret < 0) return ret;
        ret = mt9m021_write(client, MT9M021_PRE_PLL_CLK_DIV, priv->pll->n);
        if (ret < 0) return ret;
        ret = mt9m021_write(client, MT9M021_PLL_MULTIPLIER, priv->pll->m);
        if (ret < 0) return ret;

        if (mt9m021->version == MT9M021_COLOR_VERSION)
            ret = mt9m021_write(client, MT9M021_DIGITAL_TEST, 0x0000);
        else
            ret = mt9m021_write(client, MT9M021_DIGITAL_TEST, 0x0080);

        if (ret < 0) return ret;

        msleep(100);

        return ret;
}

/**
 * mt9m021_set_size - set the frame resolution
 * @client: pointer to the i2c client
 *
 */
static int mt9m021_set_size(struct i2c_client *client, struct mt9m021_frame_size *frame)
{
    struct camera_common_data *common_data = to_camera_common_data(client);
    struct daxc02 *priv = common_data->priv;
    int ret;
    int hratio;
    int vratio;

    dev_dbg(&client->dev, "%s\n", __func__);


    hratio = DIV_ROUND_CLOSEST(priv->crop.width, priv->format.width);
    vratio = DIV_ROUND_CLOSEST(priv->crop.height, priv->format.height);
    if (hratio == 2)
    {
        if (vratio == 2)
        {
            ret = mt9m021_write(client, MT9M021_DIGITAL_BINNING, MT9M021_HOR_AND_VER_BIN);
            if (ret < 0) return ret;
            dev_dbg(&client->dev, "mt9m021: Horizontal and Vertical binning enabled\n");
        }
        else if (vratio < 2)
        {
            ret = mt9m021_write(client, MT9M021_DIGITAL_BINNING, MT9M021_HOR_BIN);
            if (ret < 0) return ret;
            dev_dbg(&client->dev, "mt9m021: Horizontal binning enabled\n");
        }
    }
    else
    {
        ret = mt9m021_write(client, MT9M021_DIGITAL_BINNING, MT9M021_DISABLE_BINNING);
        if (ret < 0) return ret;
        dev_dbg(&client->dev, "mt9m021: Binning disabled\n");
    }

    ret = mt9m021_write(client, MT9M021_Y_ADDR_START, priv->crop.top);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_X_ADDR_START, priv->crop.left);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_Y_ADDR_END, priv->crop.top + priv->crop.height - 1);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_X_ADDR_END, priv->crop.left + priv->crop.width - 1);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_FRAME_LENGTH_LINES, priv->crop.height + 37);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_LINE_LENGTH_PCK, MT9M021_LLP_RECOMMENDED);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_COARSE_INT_TIME, 0x01C2);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_X_ODD_INC, 0x0001);
    if(ret < 0) return ret;
    return mt9m021_write(client, MT9M021_Y_ODD_INC, 0x0001);
}

static int mt9m021_is_streaming(struct i2c_client *client)
{
    uint16_t streaming;

    dev_dbg(&client->dev, "%s\n", __func__);

    streaming = mt9m021_read(client, MT9M021_RESET_REG);
    streaming = ( (streaming >> 2) & 0x0001);

    return (streaming != 0);
}

static int mt9m021_set_autoexposure( struct i2c_client *client, enum v4l2_exposure_auto_type ae_mode )

{
    struct camera_common_data *common_data = to_camera_common_data(client);
    struct daxc02 *priv = common_data->priv;
    int streaming;
    int ret = 0;

    dev_dbg(&client->dev, "%s\n", __func__);

    /* Save the current streaming state. Used later to restore it */
    streaming = mt9m021_is_streaming(client);

    switch(ae_mode)
    {
        case V4L2_EXPOSURE_AUTO: /* Shutter and Aperture */
            dev_err(&client->dev, "Unsupported auto-exposure mode requested: %d\n", ae_mode);
            ret = -EINVAL;
            break;

        case V4L2_EXPOSURE_MANUAL:
            if (streaming)
            {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
                if (ret < 0) return ret;
            }

            ret = mt9m021_write(client, MT9M021_EMBEDDED_DATA_CTRL, 0x1802);
            if (ret < 0) return ret;
            ret = mt9m021_write(client, MT9M021_AE_CTRL_REG, 0x0000);
            if (ret < 0) return ret;
            if (streaming) {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_ON);
                if (ret < 0) return ret;
            }
            break;

        case V4L2_EXPOSURE_SHUTTER_PRIORITY:
            if (streaming)
            {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
                if (ret < 0) return ret;
            }
            ret = mt9m021_write(client, MT9M021_EMBEDDED_DATA_CTRL, 0x1982);
            if (ret < 0) return ret;
            ret = mt9m021_write(client, MT9M021_AE_CTRL_REG, 0x0013);
            if (ret < 0) return ret;

            if (streaming)
            {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_ON);
                if (ret < 0) return ret;
            }
            break;

        case V4L2_EXPOSURE_APERTURE_PRIORITY:
            dev_err(&client->dev, "Unsupported auto-exposure mode requested: %d\n", ae_mode);
            ret = -EINVAL;
            break;

        default:
            dev_err(&client->dev, "Auto Exposure mode out of range: %d\n", ae_mode);
            ret = -ERANGE;
            break;
    }
    if(ret == 0) priv->autoexposure = ae_mode;

    return ret;
}


/***************************************************
        V4L2 Subdev Video Operations
****************************************************/

static int mt9m021_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct mt9m021_frame_size frame;
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    if (!enable) return mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);

    /* soft reset */
        /*
    ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_RESET);
    if(ret < 0)
        return ret;

    msleep(200);
        */
    ret = mt9m021_sequencer_settings(client);
    if (ret < 0)
    {
        printk(KERN_ERR"%s: Failed to setup sequencer\n",__func__);
        return ret;
    }

    ret = mt9m021_col_correction(client);
    if (ret < 0)
    {
        printk(KERN_ERR"%s: Failed to setup column correction\n",__func__);
        return ret;
    }

    ret = mt9m021_rev2_settings(client);
    if (ret < 0)
    {
        printk(KERN_ERR"%s: Failed to setup Rev2 optimised settings\n",__func__);
        return ret;
    }

    ret = mt9m021_pll_setup(client);
    if (ret < 0)
    {
        printk(KERN_ERR"%s: Failed to setup pll\n",__func__);
        return ret;
    }

    ret = mt9m021_set_size(client, &frame);
    if (ret < 0)
    {
        printk(KERN_ERR"%s: Failed to setup resolution\n",__func__);
        return ret;
    }

    /* start streaming */
    return mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_ON);
}

static int daxc02_g_input_status(struct v4l2_subdev *sd, uint32_t *status)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct camera_common_data *s_data = to_camera_common_data(client);
    struct daxc02 *priv = (struct daxc02 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;

    dev_dbg(&client->dev, "%s\n", __func__);

    *status = pw->state == SWITCH_ON;
    return 0;
}

static struct v4l2_subdev_video_ops daxc02_subdev_video_ops = {
    .s_stream               = mt9m021_s_stream,
    .s_mbus_fmt             = camera_common_s_fmt,
    .g_mbus_fmt             = camera_common_g_fmt,
    .try_mbus_fmt           = camera_common_try_fmt,
    .enum_mbus_fmt          = camera_common_enum_fmt,
    .g_mbus_config          = camera_common_g_mbus_config,
    .g_input_status         = daxc02_g_input_status,
    .enum_framesizes        = camera_common_enum_framesizes,
    .enum_frameintervals    = camera_common_enum_frameintervals,
};


/***************************************************
        V4L2 Subdev Core Operations
****************************************************/

static struct v4l2_subdev_core_ops daxc02_subdev_core_ops = {
    .s_power                = camera_common_s_power,
};


/***************************************************
        V4L2 Subdev Pad Operations
****************************************************/
static int mt9m021_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_mbus_code_enum *code)
{
    struct camera_common_data *common_data = container_of(sd, struct camera_common_data, subdev);
    struct daxc02 *priv = common_data->priv;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    dev_dbg(&client->dev, "%s\n", __func__);

    if (code->pad || code->index) return -EINVAL;

    code->code = priv->format.code;
    return 0;
}

static int mt9m021_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_frame_size_enum *fse)
{
    struct camera_common_data *common_data = container_of(sd, struct camera_common_data, subdev);
    struct daxc02 *priv = common_data->priv;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s\n", __func__);

    if (fse->index != 0 || fse->code != priv->format.code) return -EINVAL;

    fse->min_width = MT9M021_WINDOW_WIDTH_MIN;
    fse->max_width = MT9M021_WINDOW_WIDTH_MAX;
    fse->min_height = MT9M021_WINDOW_HEIGHT_MIN;
    fse->max_height = MT9M021_WINDOW_HEIGHT_MAX;

    return 0;
}

static struct v4l2_mbus_framefmt * __mt9m021_get_pad_format(struct daxc02 *priv, struct v4l2_subdev_fh *fh, unsigned int pad, uint32_t which)
{
    switch (which)
    {
        case V4L2_SUBDEV_FORMAT_TRY:
            if(!fh) break;
            return v4l2_subdev_get_try_format(fh, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
            if(!priv) break;
            return &priv->format;
        default:
            break;
    }
    return NULL;
}

static struct v4l2_rect * __mt9m021_get_pad_crop(struct daxc02 *priv, struct v4l2_subdev_fh *fh, unsigned int pad, uint32_t which)
{
    switch (which)
    {
        case V4L2_SUBDEV_FORMAT_TRY:
            if(!fh) break;
            return v4l2_subdev_get_try_crop(fh, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
            if(!priv) break;
            return &(priv->crop);
        default:
            break;
    }
    return NULL;
}

static int mt9m021_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format)
{

    return camera_common_g_fmt(sd, &format->format);
    /*
    struct camera_common_data *common_data = container_of(&sd, struct camera_common_data, subdev);
    struct daxc02 *priv = common_data->priv;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s\n", __func__);

    format->format = *__mt9m021_get_pad_format(priv, fh, format->pad, format->which);

    return 0;
    */
}

static int mt9m021_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format)
{
    int ret;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s\n", __func__);

    if (format->which == V4L2_SUBDEV_FORMAT_TRY)
        ret = camera_common_try_fmt(sd, &format->format);
    else
        ret = camera_common_s_fmt(sd, &format->format);

    return ret;

    /*
    struct camera_common_data *common_data = container_of(&sd, struct camera_common_data, subdev);
    struct daxc02 *priv = common_data->priv;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct mt9m021_frame_size size;
    struct v4l2_mbus_framefmt *__format;
    struct v4l2_rect *__crop;
    unsigned int wratio;
    unsigned int hratio;

    dev_dbg(&client->dev, "%s\n", __func__);

    __crop = __mt9m021_get_pad_crop(priv, fh, format->pad, format->which);
    if(!__crop) return -EFAULT;

    // Clamp the width and height to avoid dividing by zero.
    size.width = clamp_t(unsigned int, ALIGN(format->format.width, 2),
            MT9M021_WINDOW_WIDTH_MIN,
            MT9M021_WINDOW_WIDTH_MAX);
    size.height = clamp_t(unsigned int, ALIGN(format->format.height, 2),
            MT9M021_WINDOW_HEIGHT_MIN,
            MT9M021_WINDOW_WIDTH_MAX);

    wratio = DIV_ROUND_CLOSEST(__crop->width, size.width);
    hratio = DIV_ROUND_CLOSEST(__crop->height, size.height);

    __format = __mt9m021_get_pad_format(priv, fh, format->pad, format->which);

    __format->width             = __crop->width / wratio;
    __format->height            = __crop->height / hratio;

    printk(KERN_INFO"mt9m021: crop = %dx%d format = %dx%d\n",
    __crop->width, __crop->height, __format->width, __format->height);

    format->format              = *__format;

    priv->format.width       = format->format.width;
    priv->format.height      = format->format.height;
    priv->format.code        = V4L2_MBUS_FMT_SGRBG12_1X12;

    return 0;
    */
}

static int mt9m021_get_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_crop *crop)
{
    struct camera_common_data *common_data = container_of(sd, struct camera_common_data, subdev);
    struct daxc02 *priv = common_data->priv;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s\n", __func__);

    crop->rect = *__mt9m021_get_pad_crop(priv, fh, crop->pad, crop->which);

    return 0;
}

static int mt9m021_set_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_crop *crop)
{
    struct camera_common_data *common_data = container_of(sd, struct camera_common_data, subdev);
    struct daxc02 *priv = common_data->priv;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct v4l2_mbus_framefmt *__format;
    struct v4l2_rect *__crop;
    struct v4l2_rect rect;

    dev_dbg(&client->dev, "%s\n", __func__);

    /* Clamp the crop rectangle boundaries and align them to a multiple of 2
    * pixels to ensure a GRBG Bayer pattern.
    */
    rect.left = clamp(ALIGN(crop->rect.left, 2), MT9M021_COLUMN_START_MIN,
            MT9M021_COLUMN_START_MAX);
    rect.top = clamp(ALIGN(crop->rect.top, 2), MT9M021_ROW_START_MIN,
            MT9M021_ROW_START_MAX);
    rect.width = clamp(ALIGN(crop->rect.width, 2),
            MT9M021_WINDOW_WIDTH_MIN,
            MT9M021_WINDOW_WIDTH_MAX);
    rect.height = clamp(ALIGN(crop->rect.height, 2),
            MT9M021_WINDOW_HEIGHT_MIN,
            MT9M021_WINDOW_HEIGHT_MAX);

    rect.width = min(rect.width, MT9M021_PIXEL_ARRAY_WIDTH - rect.left);
    rect.height = min(rect.height, MT9M021_PIXEL_ARRAY_HEIGHT - rect.top);

    __crop = __mt9m021_get_pad_crop(priv, fh, crop->pad, crop->which);

    /* Reset the output image size if the crop rectangle size has
    * been modified.
    */
    if (rect.width != __crop->width || rect.height != __crop->height)
    {
        __format = __mt9m021_get_pad_format(priv, fh, crop->pad, crop->which);
        __format->width = rect.width;
        __format->height = rect.height;
    }

    *__crop = rect;
    crop->rect = rect;

    priv->crop.left      = crop->rect.left;
    priv->crop.top       = crop->rect.top;
    priv->crop.width     = crop->rect.width;
    priv->crop.height    = crop->rect.height;

    return 0;
}


static struct v4l2_subdev_pad_ops mt9m021_subdev_pad_ops = {
    .enum_mbus_code         = mt9m021_enum_mbus_code,
    .enum_frame_size        = mt9m021_enum_frame_size,
    .get_fmt                = mt9m021_get_format,
    .set_fmt                = mt9m021_set_format,
    .get_crop               = mt9m021_get_crop,
    .set_crop               = mt9m021_set_crop,
};


/***************************************************
        V4L2 Subdev Operations
****************************************************/

static struct v4l2_subdev_ops daxc02_subdev_ops = {
    .core                   = &daxc02_subdev_core_ops,
    .video                  = &daxc02_subdev_video_ops,
    .pad                    = &mt9m021_subdev_pad_ops,
};


/***************************************************
        Camera Common Operations
****************************************************/

static struct camera_common_sensor_ops daxc02_common_ops = {
    .power_on               = daxc02_power_on,
    .power_off              = daxc02_power_off,
    .write_reg              = mt9m021_write_reg,
    .read_reg               = mt9m021_read_reg,
};


/***********************************************************
    V4L2 subdev internal operations
************************************************************/
static int mt9m021_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    dev_dbg(&client->dev, "%s\n", __func__);
    //return camera_common_s_power(sd, 1);
    return 0;
}

static int mt9m021_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    dev_dbg(&client->dev, "%s\n", __func__);
    //return camera_common_s_power(sd, 0);
    return 0;
}

/*
 * Internal ops. Never call this from drivers, only the v4l2 framework can call
 * these ops.
 */
static const struct v4l2_subdev_internal_ops mt9m021_subdev_internal_ops = {
    .open               = mt9m021_open,
    .close              = mt9m021_close,
};


/***************************************************
        Media Entity Operations
****************************************************/

static const struct media_entity_operations daxc02_media_ops = {
    .link_validate      = v4l2_subdev_link_validate,
};


/***************************************************
        I2C Driver Setup
****************************************************/

static struct of_device_id daxc02_of_match[] = {
        { .compatible = "nova,daxc02", },
        { },
};

static struct camera_common_pdata *daxc02_parse_dt(struct i2c_client *client)
{
    struct device_node *node = client->dev.of_node;
    struct camera_common_pdata *board_priv_pdata;
    const struct of_device_id *match;
    //int gpio;
    int err;

    dev_dbg(&client->dev, "%s\n", __func__);

    if (!node) return NULL;

    match = of_match_device(daxc02_of_match, &client->dev);
    if (!match)
    {
        dev_err(&client->dev, "Failed to find matching dt id\n");
        return NULL;
    }

    board_priv_pdata = devm_kzalloc(&client->dev,
               sizeof(*board_priv_pdata), GFP_KERNEL);
    if (!board_priv_pdata) return NULL;

    err = camera_common_parse_clocks(client, board_priv_pdata);
    if (err)
    {
        dev_err(&client->dev, "Failed to find clocks\n");
        goto error;
    }

    /*
    gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
    if (gpio < 0) {
        dev_err(&client->dev, "pwdn gpios not in DT\n");
        goto error;
    }
    board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

    gpio = of_get_named_gpio(node, "reset-gpios", 0);
    if (gpio < 0) {
        dev_dbg(&client->dev, "reset gpios not in DT\n");
        gpio = 0;
    }
    board_priv_pdata->reset_gpio = (unsigned int)gpio;
    */

    board_priv_pdata->use_cam_gpio =
        of_property_read_bool(node, "cam,use-cam-gpio");

    err = of_property_read_string(node, "avdd-reg",
            &board_priv_pdata->regulators.avdd);
    if (err)
    {
        dev_err(&client->dev, "avdd-reg not in DT\n");
        goto error;
    }

    err = of_property_read_string(node, "iovdd-reg",
            &board_priv_pdata->regulators.iovdd);
    if (err)
    {
        dev_err(&client->dev, "iovdd-reg not in DT\n");
        goto error;
    }

    err = of_property_read_string(node, "dvdd-reg",
            &board_priv_pdata->regulators.dvdd);
    if (err)
    {
        dev_err(&client->dev, "dvdd-reg not in DT\n");
        goto error;
    }

    board_priv_pdata->has_eeprom =
        of_property_read_bool(node, "has-eeprom");

    return board_priv_pdata;

    error:
        devm_kfree(&client->dev, board_priv_pdata);
        return NULL;
}

static int daxc02_ctrls_init(struct daxc02 *priv)
{
    struct i2c_client *client = priv->i2c_client;
    struct camera_common_data *common_data = priv->s_data;
    struct v4l2_ctrl *ctrl;
    int numctrls;
    int err;
    int i;

    dev_dbg(&client->dev, "%s\n", __func__);

    numctrls = ARRAY_SIZE(ctrl_config_list);
    dev_dbg(&client->dev, "initializing %d controls\n", numctrls);
    v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

    for (i = 0; i < numctrls; i++)
    {
        /* Skip control 'V4L2_CID_EEPROM_DATA' */
        if (ctrl_config_list[i].id == V4L2_CID_EEPROM_DATA) {
            common_data->numctrls -= 1;
            continue;
        }

        dev_dbg(&client->dev, "control %d: %s\n", i, ctrl_config_list[i].name);
        ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl_config_list[i], NULL);
        if (ctrl == NULL)
        {
            dev_err(&client->dev, "Failed to init %s ctrl\n", ctrl_config_list[i].name);
            continue;
        }

        if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
            ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY)
        {
            ctrl->string = devm_kzalloc(&client->dev, ctrl_config_list[i].max + 1, GFP_KERNEL);
            if (!ctrl->string) return -ENOMEM;
        }
        priv->ctrls[i] = ctrl;
    }

    priv->numctrls = numctrls;
    priv->subdev->ctrl_handler = &priv->ctrl_handler;
    if (priv->ctrl_handler.error)
    {
        dev_err(&client->dev, "Error %d adding controls\n", priv->ctrl_handler.error);
        err = priv->ctrl_handler.error;
        goto error;
    }

    err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
    if (err)
    {
        dev_err(&client->dev, "Error %d setting default controls\n", err);
        goto error;
    }

    return 0;

    error:
        v4l2_ctrl_handler_free(&priv->ctrl_handler);
        return err;
}

static int daxc02_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct camera_common_data *common_data;
    struct mt9m021_platform_data *mt9m021_pdata;
    struct daxc02 *priv;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    char debugfs_name[10];
    int32_t data;
    uint8_t i;
    int err;

    dev_dbg(&client->dev, "%s\n", __func__);

    mt9m021_pdata = devm_kzalloc(&client->dev, sizeof(struct mt9m021_platform_data), GFP_KERNEL);
    if (!mt9m021_pdata) return -ENOMEM;

    common_data = devm_kzalloc(&client->dev, sizeof(struct camera_common_data), GFP_KERNEL);
    if (!common_data) return -ENOMEM;

    priv = devm_kzalloc(&client->dev,
                sizeof(struct daxc02) + sizeof(struct v4l2_ctrl *) *
                ARRAY_SIZE(ctrl_config_list),
                GFP_KERNEL);
    if (!priv) return -ENOMEM;

    priv->pdata = daxc02_parse_dt(client);
    if (!priv->pdata)
    {
        dev_err(&client->dev, "unable to get device tree platform data\n");
        return -EFAULT;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
    {
        dev_warn(&client->dev, "i2c-adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
        return -EIO;
    }

    mt9m021_pdata->ext_freq     = 24000000;
    mt9m021_pdata->target_freq  = 74250000;
    mt9m021_pdata->version      = MT9M021_COLOR_VERSION;

    common_data->ops            = &daxc02_common_ops;
    common_data->ctrl_handler   = &priv->ctrl_handler;
    common_data->i2c_client     = client;
    common_data->frmfmt         = mt9m021_frmfmt;
    common_data->colorfmt       = camera_common_find_datafmt(V4L2_MBUS_FMT_SRGGB12_1X12);
    common_data->numfmts        = ARRAY_SIZE(mt9m021_frmfmt);
    common_data->power          = &priv->power;
    common_data->ctrls          = priv->ctrls;
    common_data->priv           = (void *)priv;
    common_data->numctrls       = ARRAY_SIZE(ctrl_config_list);
    common_data->def_mode       = MT9M021_DEFAULT_MODE;
    common_data->def_width      = MT9M021_PIXEL_ARRAY_WIDTH;
    common_data->def_height     = MT9M021_PIXEL_ARRAY_HEIGHT;
    common_data->fmt_width      = common_data->def_width;
    common_data->fmt_height     = common_data->def_height;
    common_data->def_clk_freq   = MT9M021_TARGET_FREQ;

    priv->i2c_client            = client;
    priv->s_data                = common_data;
    priv->subdev                = &common_data->subdev;
    priv->subdev->dev           = &client->dev;
    priv->s_data->dev           = &client->dev;
    priv->group_hold_prev       = 0;

    priv->mt9m021_pdata         = mt9m021_pdata;
    priv->crop.width            = MT9M021_WINDOW_WIDTH_MAX;
    priv->crop.height           = MT9M021_WINDOW_HEIGHT_MAX;
    priv->crop.left             = MT9M021_COLUMN_START_DEF;
    priv->crop.top              = MT9M021_ROW_START_DEF;

    priv->format.code           = V4L2_MBUS_FMT_SGRBG12_1X12;
    priv->format.width          = MT9M021_WINDOW_WIDTH_DEF;
    priv->format.height         = MT9M021_WINDOW_HEIGHT_DEF;
    priv->format.field          = V4L2_FIELD_NONE;
    priv->format.colorspace     = V4L2_COLORSPACE_SRGB;

    err = daxc02_power_get(priv);
    if (err) return err;

    daxc02_power_on(common_data);
    data = mt9m021_read(client, MT9M021_CHIP_ID_REG);
    if (data != MT9M021_CHIP_ID)
    {
        for(i=0; i < 5; i++)
        {
            data = mt9m021_read(client, MT9M021_CHIP_ID_REG);
            msleep(5);
        }
        dev_err(&client->dev, "Aptina MT9M021 not detected, chip ID read:0x%4.4x\n", data);
        return -ENODEV;
    }
    dev_info(&client->dev, "Aptina MT9M021 detected at address 0x%02x\n", client->addr);

    err = camera_common_parse_ports(client, common_data);
    if (err)
    {
        dev_err(&client->dev, "Failed to find port info\n");
        return err;
    }
    sprintf(debugfs_name, "daxc02_%c", common_data->csi_port + 'a');
    dev_dbg(&client->dev, "name %s\n", debugfs_name);
    camera_common_create_debugfs(common_data, debugfs_name);

    v4l2_i2c_subdev_init(priv->subdev, client, &daxc02_subdev_ops);

    err = daxc02_ctrls_init(priv);
    if (err) return err;

    priv->subdev->internal_ops = &mt9m021_subdev_internal_ops;
    priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

    #if defined(CONFIG_MEDIA_CONTROLLER)
    pr_info("daxc02: initializing media entity.\n");
    priv->pad.flags = MEDIA_PAD_FL_SOURCE;
    priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
    priv->subdev->entity.ops = &daxc02_media_ops;
    err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
    if (err < 0)
    {
        dev_err(&client->dev, "unable to init media entity\n");
        return err;
    }
    #endif

    err = v4l2_async_register_subdev(priv->subdev);
    if (err) return err;

    pr_info("daxc02: probe successful.\n");
    return 0;
}

static int daxc02_remove(struct i2c_client *client)
{
    struct camera_common_data *common_data = to_camera_common_data(client);
    struct daxc02 *priv = (struct daxc02 *)common_data->priv;

    dev_dbg(&client->dev, "%s\n", __func__);

    v4l2_async_unregister_subdev(priv->subdev);

    #if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&priv->subdev->entity);
    #endif

    v4l2_ctrl_handler_free(&priv->ctrl_handler);
    daxc02_power_off(common_data);
    daxc02_power_put(priv);
    camera_common_remove_debugfs(common_data);

    return 0;
}

static const struct i2c_device_id daxc02_id[] = {
    { "daxc02", 0 },
    { }
};

static struct i2c_driver daxc02_i2c_driver = {
    .driver = {
        .name = "daxc02",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(daxc02_of_match),
    },
    .probe = daxc02_probe,
    .remove = daxc02_remove,
    .id_table = daxc02_id,
};


/***************************************************
        Module Setup
****************************************************/

module_i2c_driver(daxc02_i2c_driver);
MODULE_DEVICE_TABLE(of, daxc02_of_match);
MODULE_DEVICE_TABLE(i2c, daxc02_id);
MODULE_DESCRIPTION("Nova Dynamics DAX-C02 dual MIPI camera driver");
MODULE_AUTHOR("Wilkins White <ww@novadynamics.com>");
MODULE_LICENSE("GPL v2");

