//#define USE_RAW8

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


/***************************************************
        MT9M021 Image Sensor Registers
****************************************************/

#define MT9M021_CHIP_ID_REG             0x3000
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
#define MT9M021_FRAME_LENGTH_LINES_CB   0x30AA
#define MT9M021_X_ODD_INC               0x30A2
#define MT9M021_Y_ODD_INC               0x30A6
#define MT9M021_READ_MODE               0x3040
#define MT9M021_TEST_PATTERN            0x3070
#define MT9M021_DIGITAL_BINNING         0x3032

#define MT9M021_AE_CTRL_REG             0x3100
#define MT9M021_AE_LUMA_TARGET_REG      0x3102
#define MT9M021_EMBEDDED_DATA_CTRL      0x3064
#define MT9M021_DATAPATH_SELECT         0X306E

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
#define MT9M021_WINDOW_HEIGHT_MAX       720
#define MT9M021_WINDOW_HEIGHT_DEF       720
#define MT9M021_WINDOW_WIDTH_MIN        2
#define MT9M021_WINDOW_WIDTH_MAX        1280
#define MT9M021_WINDOW_WIDTH_DEF        1280
#define MT9M021_BINNING_DEF             0x0020

#define MT9M021_RESET                   0x00D9
#define MT9M021_STREAM_OFF              0x00D8
#define MT9M021_STREAM_ON               0x00DC

#define MT9M021_ANALOG_GAIN_MIN         0x0
#define MT9M021_ANALOG_GAIN_MAX         0x3
#define MT9M021_ANALOG_GAIN_DEF         0x0
#define MT9M021_ANALOG_GAIN_SHIFT       4
#define MT9M021_ANALOG_GAIN_MASK        0x0030

#define MT9M021_GLOBAL_GAIN_MIN         0x01
#define MT9M021_GLOBAL_GAIN_MAX         0xFF
#define MT9M021_GLOBAL_GAIN_DEF         0x10

#define MT9M021_EXPOSURE_MIN            0x0001
#define MT9M021_EXPOSURE_MAX            0x02A0
#define MT9M021_EXPOSURE_DEF            0x01C2

#define MT9M021_FINE_INT_TIME_MIN       0x0000
#define MT9M021_FINE_INT_TIME_MAX       0xFFFF
#define MT9M021_FINE_INT_TIME_DEF       0x0380

#define MT9M021_LLP_MIN                 0x0672
#define MT9M021_LLP_MAX                 0xFFFF
#define MT9M021_LLP_DEF                 0x0672

static uint16_t mt9m021_seq_data[] = {
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

static uint16_t mt9m021_analog_setting[] = {
    0x00FD, 0x0FFF, 0x0003, 0xF87A, 0xE075, 0x077C, 0xA4EB, 0xD208
};

/***************************************************
        NVIDIA Camera Common Defines
****************************************************/

enum mt9m021_modes{
    MT9M021_DEFAULT_MODE
};

static const int mt9m021_30fps[] = {
    30,
};

static const struct camera_common_frmfmt mt9m021_frmfmt[] = {
    {{MT9M021_WINDOW_WIDTH_DEF, MT9M021_WINDOW_HEIGHT_DEF},   mt9m021_30fps,  1, 0,   MT9M021_DEFAULT_MODE},
};

#ifdef USE_RAW8
static const struct camera_common_colorfmt mt9m021_colorfmt[] = {
    {
        V4L2_MBUS_FMT_SRGGB8_1X8,
        V4L2_COLORSPACE_SRGB,
        V4L2_PIX_FMT_SRGGB8,
    },
};
#else
static const struct camera_common_colorfmt mt9m021_colorfmt[] = {
    {
        V4L2_MBUS_FMT_SRGGB12_1X12,
        V4L2_COLORSPACE_SRGB,
        V4L2_PIX_FMT_SRGGB12,
    },
};
#endif



/***************************************************
        TC358746AXBG MIPI Converter Defines
****************************************************/

struct daxc02_mipi_settings {
    uint8_t len;
    uint16_t addr;
    uint32_t data;
};

struct daxc02_mipi_settings daxc02_mipi_output[] = {
   {2, 0x0004, 0x0004},
   {2, 0x0002, 0x0001}, // reset 1
   {2, 0x0002, 0x0000}, // reset 0
   {2, 0x0016, 0x50F9}, // set the input and feedback frequency division ratio
   {2, 0x0018, 0x0213}, // 50% maximum loop bandwidth + PLL clock enable + normal operation + PLL enable
   {2, 0x0006, 0x0030}, // FIFO level 3

#ifdef USE_RAW8
   {2, 0x0008, 0x0000}, // data format RAW8
#else
   {2, 0x0008, 0x0020}, // data format RAW12
#endif

   {2, 0x0022, 0x0780}, // word count (bytes per line)
   {4, 0x0210, 0x00002C00},
   {4, 0x0214, 0x00000005},
   {4, 0x0218, 0x00001E06},
   {4, 0x021C, 0x00000004},
   {4, 0x0220, 0x00000406},
   {4, 0x0224, 0x00004988},
   {4, 0x0228, 0x0000000C},
   {4, 0x022C, 0x00000006},
   {4, 0x0234, 0x0000001F}, // Voltage regulator enable for data 0-3 and clock lanes.
   //{4, 0x0238, 0x00000001}, // Continuous clock mode. Maintains the clock lane output regardless of data lane operation
   {4, 0x0238, 0x00000000}, // Discontinuous clock mode.
   {4, 0x0518, 0x00000001}, // CSI start

   {4, 0x0500, 0xA30080A1}, // 1 data lane
   //{4, 0x0500, 0xA30080A3}, // 2 data lanes
   //{4, 0x0500, 0xA30080A7}, // 4 data lanes

   {4, 0x0204, 0x00000001}, // TX PPI starts
   {2, 0x0004, 0x0044},
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

    struct camera_common_data           *s_data;
    struct camera_common_pdata          *pdata;

    struct v4l2_mbus_framefmt           format;
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
static int mt9m021_write(struct i2c_client *client, uint16_t addr, uint16_t val);
static int daxc02_bridge_setup(struct i2c_client *client);
static int mt9m021_sequencer_settings(struct i2c_client *client);
static int mt9m021_col_correction(struct i2c_client *client);
static int mt9m021_rev2_settings(struct i2c_client *client);
static int mt9m021_pll_setup(struct i2c_client *client, uint16_t m, uint8_t n, uint8_t p1, uint8_t p2);
static int mt9m021_set_size(struct i2c_client *client);
static int mt9m021_is_streaming(struct i2c_client *client);
static int mt9m021_set_autoexposure( struct i2c_client *client, enum v4l2_exposure_auto_type ae_mode );
static int mt9m021_s_stream(struct v4l2_subdev *sd, int enable);
static int daxc02_g_input_status(struct v4l2_subdev *sd, uint32_t *status);
static int mt9m021_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_mbus_code_enum *code);
static int mt9m021_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_frame_size_enum *fse);
static int mt9m021_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt);
static int mt9m021_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format);
static int daxc02_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
static int daxc02_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
static struct camera_common_pdata *daxc02_parse_dt(struct i2c_client *client);
static int daxc02_ctrls_init(struct daxc02 *priv);
static int daxc02_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int daxc02_remove(struct i2c_client *client);


/***************************************************
        V4L2 Control Configuration
****************************************************/

/*
 * Non-standard control definitions.
 */
#define V4L2_CID_GAIN_RED           (V4L2_CID_USER_BASE | 0x1001)
#define V4L2_CID_GAIN_GREEN1        (V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_GAIN_GREEN2        (V4L2_CID_USER_BASE | 0x1003)
#define V4L2_CID_GAIN_BLUE          (V4L2_CID_USER_BASE | 0x1004)
#define V4L2_CID_ANALOG_GAIN        (V4L2_CID_USER_BASE | 0x1005)

/*
 * Extra test pattern information to display to the user.
 */
static const char * const mt9m021_test_pattern_menu[] = {
    "0:Disabled",
    "1:Solid color test pattern",
    "2:color bar test pattern",
    "3:Fade to gray color bar test pattern",
    "256:Walking 1s test pattern (12 bit)"
};

/** daxc02_s_ctrl - Called by the V4L2 framework to set a control.
  * @ctrl:  struct containing the control id to switch off of and
  *         value to set from the v4l2 framework.
  */
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
            if(ret < 0) break;
            ret = mt9m021_write(client, MT9M021_COARSE_INT_TIME, ctrl->val);
            break;

        case V4L2_CID_COARSE_TIME:
            dev_dbg(&client->dev, "%s: V4L2_CID_COARSE_TIME\n", __func__);
            ret = mt9m021_write(client, MT9M021_FINE_INT_TIME, ctrl->val);
            if(ret < 0) break;
            ret = mt9m021_write(client, MT9M021_FINE_INT_TIME_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN\n", __func__);
            ret = mt9m021_write(client, MT9M021_GLOBAL_GAIN, ctrl->val);
            if(ret < 0) break;
            ret = mt9m021_write(client, MT9M021_GLOBAL_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_GREEN1:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_GREEN1\n", __func__);
            ret = mt9m021_write(client, MT9M021_GREEN1_GAIN, ctrl->val);
            if(ret < 0) break;
            ret = mt9m021_write(client, MT9M021_GREEN1_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_RED:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_RED\n", __func__);
            ret = mt9m021_write(client, MT9M021_RED_GAIN, ctrl->val);
            if(ret < 0) break;
            ret = mt9m021_write(client, MT9M021_RED_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_BLUE:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_BLUE\n", __func__);
            ret = mt9m021_write(client, MT9M021_BLUE_GAIN, ctrl->val);
            if(ret < 0) break;
            ret = mt9m021_write(client, MT9M021_BLUE_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_GAIN_GREEN2:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_GREEN2\n", __func__);
            ret = mt9m021_write(client, MT9M021_GREEN2_GAIN, ctrl->val);
            if(ret < 0) break;
            ret = mt9m021_write(client, MT9M021_GREEN2_GAIN_CB, ctrl->val);
            break;

        case V4L2_CID_ANALOG_GAIN:
            dev_dbg(&client->dev, "%s: V4L2_CID_ANALOG_GAIN\n", __func__);
            reg16 = mt9m021_read(client, MT9M021_DIGITAL_TEST);
            reg16 = ( reg16 & ~MT9M021_ANALOG_GAIN_MASK ) |
                ( ( ctrl->val << MT9M021_ANALOG_GAIN_SHIFT ) & MT9M021_ANALOG_GAIN_MASK );
            ret = mt9m021_write(client, MT9M021_DIGITAL_TEST, reg16);
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
            break;

        case V4L2_CID_TEST_PATTERN:
            dev_dbg(&client->dev, "%s: V4L2_CID_TEST_PATTERN\n", __func__);
            if (!ctrl->val)
            {
                ret = mt9m021_write(client, MT9M021_TEST_PATTERN, 0x0000);
                if(ret < 0) return ret;
            }
            ret = mt9m021_write(client, MT9M021_TEST_PATTERN, ctrl->val);
            break;

        case V4L2_CID_FRAME_LENGTH:
            dev_dbg(&client->dev, "%s: V4L2_CID_FRAME_LENGTH\n", __func__);
            ret = mt9m021_write(client, MT9M021_LINE_LENGTH_PCK, ctrl->val);
            break;

        default:
            dev_err(&client->dev, "%s: unknown ctrl id.\n", __func__);
            return -EINVAL;
    }

    return ret;
}

/*
 * Registers the control operations with the v4l2 framework.
 */
static const struct v4l2_ctrl_ops daxc02_ctrl_ops = {
    .s_ctrl             = daxc02_s_ctrl,
};

/*
 * List of controls and limits that can be set through the v4l2 framework.
 */
static struct v4l2_ctrl_config ctrl_config_list[] = {
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
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_COARSE_TIME,
        .name           = "Coarse Time",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = V4L2_CTRL_FLAG_SLIDER,
        .min            = MT9M021_FINE_INT_TIME_MIN,
        .max            = MT9M021_FINE_INT_TIME_MAX,
        .def            = MT9M021_FINE_INT_TIME_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_EXPOSURE_AUTO,
        .name           = "Auto Exposure",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = 0,
        .min            = V4L2_EXPOSURE_MANUAL,
        .max            = V4L2_EXPOSURE_SHUTTER_PRIORITY,
        .def            = V4L2_EXPOSURE_MANUAL,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_HFLIP,
        .name           = "Horizontal Flip",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = 0,
        .min            = 0,
        .max            = 1,
        .def            = 0,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_VFLIP,
        .name           = "Vertical Flip",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = 0,
        .min            = 0,
        .max            = 1,
        .def            = 0,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_FRAME_LENGTH,
        .name           = "Frame Length",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = V4L2_CTRL_FLAG_SLIDER,
        .min            = MT9M021_LLP_MIN,
        .max            = MT9M021_LLP_MAX,
        .def            = MT9M021_LLP_DEF,
        .step           = 1,
    },
};


/***************************************************
        DAX-C02 Power Functions
****************************************************/

/** daxc02_power_on - Turns on the needed TX1 voltage regulators.
  * @s_data: Nvidia camera common data struct.
  */
static int daxc02_power_on(struct camera_common_data *s_data)
{
    int err = 0;
    struct daxc02 *priv = (struct daxc02 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;
    struct i2c_client *client = s_data->i2c_client;

    dev_dbg(&client->dev, "%s\n", __func__);

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

    usleep_range(50, 100);
    if (pw->avdd) err = regulator_enable(pw->avdd);     // 2.8V
    if (err) goto daxc02_avdd_fail;

    usleep_range(50, 100);
    if (pw->iovdd) err = regulator_enable(pw->iovdd);   // 1.8V
    if (err) goto daxc02_iovdd_fail;

    msleep(200);

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

/** daxc02_power_off - Turns off the TX1 voltage regulators.
  * @s_data: Nvidia camera common data struct.
  */
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
    usleep_range(50, 100);
    if (pw->avdd) regulator_disable(pw->avdd);
    usleep_range(50, 100);
    if (pw->dvdd) regulator_disable(pw->dvdd);

    return 0;
}

/** daxc02_power_put - Registers needed voltage regulators.
 * @priv: Dax-C02 private data structure.
 */
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

/** daxc02_power_put - Frees the voltage regulators.
 * @priv: Dax-C02 private data structure.
 */
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


/***************************************************
        MT9M021 Helper Functions
****************************************************/

/** mt9m021_read - Reads a MT9M021 register.
  * @client:    pointer to the i2c client.
  * @addr:      address of the register to read.
  */
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
        dev_err(&client->dev, "read failed at 0x%04x error %d\n", addr, ret);
        return ret;
    }

    dev_dbg(&client->dev, "%s: 0x%02x%02x from 0x%04x\n", __func__, buf[0], buf[1], addr);

    return (buf[0] << 8) | buf[1];
}

/** mt9m021_write - Writes to a MT9M021 register.
  * @client:    pointer to the i2c client.
  * @addr:      address of the register to write.
  * @data:      data to write to the register.
  */
static inline int mt9m021_write(struct i2c_client *client, uint16_t addr, uint16_t data)
{
    struct i2c_msg msg;
    uint8_t buf[4];
    uint16_t __addr, __data;
    int ret;

    dev_dbg(&client->dev, "%s: 0x%04x to 0x%04x\n", __func__, data, addr);

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

    dev_err(&client->dev, "write failed at 0x%04x error %d\n", addr, ret);
    return ret;
}

/** daxc02_bridge_setup - Configures the MIPI bridge.
  * @client: pointer to the i2c client.
  */
static int daxc02_bridge_setup(struct i2c_client *client)
{
    struct i2c_msg msg[2];
    uint8_t buf[6];
    struct daxc02_mipi_settings settings;
    uint16_t __addr;
    uint32_t __data;
    int ret;
    uint8_t i;

    for(i = 0; i < ARRAY_SIZE(daxc02_mipi_output); i++)
    {
        settings = daxc02_mipi_output[i];

        __addr = cpu_to_be16(settings.addr);
        __data = cpu_to_be32(settings.data);

        buf[0] = (uint8_t)(__addr);
        buf[1] = (uint8_t)(__addr >> 8);

        buf[2] = (uint8_t)(__data >> 16);
        buf[3] = (uint8_t)(__data >> 24);

        buf[4] = (uint8_t)(__data);
        buf[5] = (uint8_t)(__data >> 8);

        msg[0].addr  = BRIDGE_I2C_ADDR;
        msg[0].flags = 0;
        msg[0].len   = settings.len + 2;
        msg[0].buf   = buf;

        ret = i2c_transfer(client->adapter, msg, 1);

        if(settings.len == 2) dev_dbg(&client->dev, "%s: 0x%04x to 0x%04x\n", __func__, settings.data, settings.addr);
        else dev_dbg(&client->dev, "%s: 0x%08x to 0x%04x\n", __func__, settings.data, settings.addr);

        if (ret < 0)
        {
            dev_err(&client->dev, "%s failed at 0x%04x error %d\n", __func__, settings.addr, ret);
            break;
        }
    }

    return ret;
}

/** mt9m021_sequencer_settings - Loads the MT9M021 sequencer settings.
 * @client: pointer to the i2c client.
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

/** mt9m021_col_correction - sets up column correction.
 * @client: pointer to the i2c client.
 */
static int mt9m021_col_correction(struct i2c_client *client)
{
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    /* Disable Streaming */
    ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
    if (ret < 0) return ret;

    /* Disable column correction */
    ret = mt9m021_write(client, MT9M021_COLUMN_CORRECTION, 0x0007);
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
    ret = mt9m021_write(client, MT9M021_COLUMN_CORRECTION, 0xE007);
    if (ret < 0) return ret;
    msleep(200);

    return ret;
}

/** mt9m021_rev2_settings - Additional setup from Leopard and Aptina.
 * @client: pointer to the i2c client.
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

    //ret = mt9m021_write(client, MT9M021_FINE_INT_TIME, 0x0380);
    //if (ret < 0) return ret;

    for(i = 0; i < ARRAY_SIZE(mt9m021_analog_setting); i++)
    {
        ret = mt9m021_write(client, MT9M021_ANALOG_REG + 2*i, mt9m021_analog_setting[i]);
        if (ret < 0) return ret;
    }

    return ret;
}

/** mt9m021_pll_setup - sets up the PLL.
 * @client:     pointer to the i2c client.
 * @m:          PLL multiplier, M.
 * @n:          PLL pre clock divider, N.
 * @p1:         PLL pixel clock divider, P1.
 * @p2:         PLL system clock divider, P2.
 *
 *    target_freq = (ext_freq x M) / (N x P1 x P2)
 *    VCO_freq    = (ext_freq x M) / N
 *
 *    Limitations of PLL parameters
 *    -----------------------------
 *    0x20      >=  M           >=  0x180
 *    0x01      >=  N           >=  0x40
 *    0x01      >=  P1          >=  0x10
 *    0x04      >=  P2          >=  0x10
 *    384MHz    >=  VCO_freq    >=  768MHz
 *
 */
static int mt9m021_pll_setup(struct i2c_client *client, uint16_t m, uint8_t n, uint8_t p1, uint8_t p2)
{
    int ret;
    dev_dbg(&client->dev, "%s: M=%d, N=%d, P1=%d, P2=%d" ,__func__, m, n, p1, p2);

    if((m <  0x20) && (m > 0x180)) return -EINVAL;
    if((n <  0x01) && (n > 0x40)) return -EINVAL;
    if((p1 <  0x01) && (p1 > 0x10)) return -EINVAL;
    if((p2 <  0x04) && (p2 > 0x10)) return -EINVAL;

    ret = mt9m021_write(client, MT9M021_VT_PIX_CLK_DIV, p1);
    if (ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_VT_SYS_CLK_DIV, p2);
    if (ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_PRE_PLL_CLK_DIV, n);
    if (ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_PLL_MULTIPLIER, m);
    if (ret < 0) return ret;

    ret = mt9m021_write(client, MT9M021_DIGITAL_TEST, 0x0000);
    if (ret < 0) return ret;

    msleep(100);

    return ret;
}

/** mt9m021_set_size - set the frame resolution to 1280x720.
 * @client: pointer to the i2c client.
 */
static int mt9m021_set_size(struct i2c_client *client)
{
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    ret = mt9m021_write(client, MT9M021_DIGITAL_BINNING, MT9M021_BINNING_DEF);
    if (ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_Y_ADDR_START, 0x0078);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_X_ADDR_START, 1);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_Y_ADDR_END, 0x0347);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_X_ADDR_END, 0x0500);
    if(ret < 0) return ret;
    //ret = mt9m021_write(client, MT9M021_LINE_LENGTH_PCK, MT9M021_LLP_DEF);
    //if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_COARSE_INT_TIME, MT9M021_EXPOSURE_DEF);
    if(ret < 0) return ret;
    ret = mt9m021_write(client, MT9M021_X_ODD_INC, 0x0001);
    if(ret < 0) return ret;
    return mt9m021_write(client, MT9M021_Y_ODD_INC, 0x0001);
}

/** mt9m021_is_streaming - returns if the sensor is streaming.
 * @client: pointer to the i2c client.
 */
static int mt9m021_is_streaming(struct i2c_client *client)
{
    uint16_t streaming;

    dev_dbg(&client->dev, "%s\n", __func__);

    streaming = mt9m021_read(client, MT9M021_RESET_REG);
    streaming = ( (streaming >> 2) & 0x0001);

    return (streaming != 0);
}

/** mt9m021_set_autoexposure - enables or disables autoexposure.
 * @client: pointer to the i2c client.
 * @ae_mode: v4l2 autoexposure mode.
 */
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

/** mt9m021_s_stream - starts or disables streaming.
 * @sd:     pointer to the v4l2 sub-device.
 * @enable: enable or disable stream.
 */
static int mt9m021_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    if (!enable) return mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);

    ret = daxc02_bridge_setup(client);
    if (ret < 0)
    {
        dev_dbg(&client->dev, "%s: Failed to setup mipi bridge\n",__func__);
        return ret;
    }

    ret = mt9m021_sequencer_settings(client);
    if (ret < 0)
    {
        dev_dbg(&client->dev, "%s: Failed to setup sequencer\n",__func__);
        return ret;
    }

    ret = mt9m021_pll_setup(client, MT9M021_PLL_M, MT9M021_PLL_N, MT9M021_PLL_P1, MT9M021_PLL_P2);
    if (ret < 0)
    {
        dev_dbg(&client->dev, "%s: Failed to setup pll\n",__func__);
        return ret;
    }

    ret = mt9m021_col_correction(client);
    if (ret < 0)
    {
        dev_dbg(&client->dev, "%s: Failed to setup column correction\n",__func__);
        return ret;
    }

    ret = mt9m021_rev2_settings(client);
    if (ret < 0)
    {
        dev_dbg(&client->dev, "%s: Failed to setup Rev2 optimised settings\n",__func__);
        return ret;
    }

    ret = mt9m021_write(client, MT9M021_EMBEDDED_DATA_CTRL, 0x1802);
    if (ret < 0)
    {
        dev_dbg(&client->dev, "%s: Failed to disable embedded data\n",__func__);
        return ret;
    }

    ret = mt9m021_set_size(client);
    if (ret < 0)
    {
        dev_dbg(&client->dev, "%s: Failed to setup resolution\n",__func__);
        return ret;
    }

    /* start streaming */
    return mt9m021_write(client, MT9M021_RESET_REG, 0x10DC);
}

/** daxc02_g_input_status - get input status.
 * @sd:     pointer to the v4l2 sub-device.
 * @status: where to store the status.
 */
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

/*
 * Registers the v4l2 sub device video operations.
 */
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

/** mt9m021_enum_mbus_code - Handles the %VIDIOC_SUBDEV_ENUM_MBUS_CODE ioctl.
 * @sd:     pointer to the v4l2 sub-device.
 * @fh:     pointer to the v4l2 sub-device file handle.
 * @code:   sub-device mbus code enum.
 */
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

/** mt9m021_enum_mbus_code - Handles the %VIDIOC_SUBDEV_ENUM_FRAME_SIZE ioctl.
 * @sd:     pointer to the v4l2 sub-device.
 * @fh:     pointer to the v4l2 sub-device file handle.
 * @fse:    sub-device frame size enum.
 */
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

/** mt9m021_get_format - Gets the sub-device format.
 * @sd:         pointer to the v4l2 sub-device.
 * @fh:         pointer to the v4l2 sub-device file handle.
 * @format:     where to store the format.
 */
static int mt9m021_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format)
{
    struct camera_common_data *common_data = container_of(sd, struct camera_common_data, subdev);
    struct daxc02 *priv = common_data->priv;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    format->format = priv->format;

    dev_dbg(&client->dev, "%s\n\twidth: %u\n\theight: %u\n\tcode: %u\n",
            __func__,
            format->format.width,
            format->format.height,
            format->format.code);

    return 0;
}

/** mt9m021_set_format - Sets the sub-device format.
 * @sd:         pointer to the v4l2 sub-device.
 * @fh:         pointer to the v4l2 sub-device file handle.
 * @format:     new format to set.
 */
static int mt9m021_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;

    dev_dbg(&client->dev, "%s\n\twidth: %u\n\theight: %u\n\tcode: %u\n",
            __func__,
            format->format.width,
            format->format.height,
            format->format.code);

    switch(format->which)
    {
        case V4L2_SUBDEV_FORMAT_TRY:
            ret = camera_common_try_fmt(sd, &format->format);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
            ret = camera_common_s_fmt(sd, &format->format);
        default:
            ret = 0;
    }

    return ret;
}

/*
 * Registers the v4l2 sub-device pad operations.
 */
static struct v4l2_subdev_pad_ops mt9m021_subdev_pad_ops = {
    .enum_mbus_code         = mt9m021_enum_mbus_code,
    .enum_frame_size        = mt9m021_enum_frame_size,
    .get_fmt                = mt9m021_get_format,
    .set_fmt                = mt9m021_set_format,
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
};


/***********************************************************
    V4L2 subdev internal operations
************************************************************/

/** mt9m021_open - Called by the v4l2 framework when the device is opened.
 * @sd:     pointer to the v4l2 sub-device.
 * @fh:     pointer to the v4l2 sub-device file handle.
 */
static int daxc02_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    dev_dbg(&client->dev, "%s\n", __func__);
    return camera_common_s_power(sd, 1);
}

/** mt9m021_open - Called by the v4l2 framework when the device is closed.
 * @sd:     pointer to the v4l2 sub-device.
 * @fh:     pointer to the v4l2 sub-device file handle.
 */
static int daxc02_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    dev_dbg(&client->dev, "%s\n", __func__);
    return camera_common_s_power(sd, 0);
}

/*
 * Registers the v4l2 sub-device internal operations.
 */
static const struct v4l2_subdev_internal_ops mt9m021_subdev_internal_ops = {
    .open               = daxc02_open,
    .close              = daxc02_close,
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

/*
 * Device tree ID for matching.
 */
static struct of_device_id daxc02_of_match[] = {
        { .compatible = "nova,daxc02", },
        { },
};

/** camera_common_pdata - Parses the device tree to load camera common data.
 * @client: pointer to the i2c client.
 */
static struct camera_common_pdata *daxc02_parse_dt(struct i2c_client *client)
{
    struct device_node *node = client->dev.of_node;
    struct camera_common_pdata *board_priv_pdata;
    const struct of_device_id *match;

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

/** daxc02_ctrls_init - Registers and initializes controls with the v4l2 framework.
 * @priv: Dax-C02 private data structure.
 */
static int daxc02_ctrls_init(struct daxc02 *priv)
{
    struct i2c_client *client = priv->i2c_client;
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
        dev_dbg(&client->dev, "control %d: %s\n", i, ctrl_config_list[i].name);
        ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl_config_list[i], NULL);
        if (ctrl == NULL)
        {
            dev_err(&client->dev, "Failed to init %s ctrl\n", ctrl_config_list[i].name);
            continue;
        }

        if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
            (ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY))
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

/** daxc02_probe - Checks if the device is present and performs one time initialization.
 * @client:     pointer to the i2c client.
 * @id:         i2c device id.
 */
static int daxc02_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct camera_common_data *common_data;
    struct daxc02 *priv;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    char debugfs_name[10];
    int32_t data;
    uint8_t i;
    int err;

    dev_dbg(&client->dev, "%s\n", __func__);

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

    common_data->ops                = &daxc02_common_ops;
    common_data->ctrl_handler       = &priv->ctrl_handler;
    common_data->i2c_client         = client;
    common_data->frmfmt             = mt9m021_frmfmt;
    common_data->colorfmt           = mt9m021_colorfmt;
    common_data->numfmts            = ARRAY_SIZE(mt9m021_frmfmt);
    common_data->power              = &priv->power;
    common_data->ctrls              = priv->ctrls;
    common_data->priv               = (void *)priv;
    common_data->numctrls           = ARRAY_SIZE(ctrl_config_list);
    common_data->def_mode           = MT9M021_DEFAULT_MODE;
    common_data->def_width          = MT9M021_PIXEL_ARRAY_WIDTH;
    common_data->def_height         = MT9M021_PIXEL_ARRAY_HEIGHT;
    common_data->def_maxfps         = 60;
    common_data->fmt_width          = common_data->def_width;
    common_data->fmt_height         = common_data->def_height;
    common_data->fmt_maxfps         = common_data->def_maxfps;
    common_data->def_clk_freq       = MT9M021_TARGET_FREQ;
    common_data->sensor_mode_id     = 0;
    common_data->use_sensor_mode_id = true;

    priv->i2c_client            = client;
    priv->s_data                = common_data;
    priv->subdev                = &common_data->subdev;
    priv->subdev->dev           = &client->dev;
    priv->s_data->dev           = &client->dev;

#ifdef USE_RAW8
    priv->format.code           = V4L2_MBUS_FMT_SRGGB8_1X8;
#else
    priv->format.code           = V4L2_MBUS_FMT_SRGGB12_1X12;
#endif

    priv->format.width          = MT9M021_WINDOW_WIDTH_DEF;
    priv->format.height         = MT9M021_WINDOW_HEIGHT_DEF;
    priv->format.field          = V4L2_FIELD_NONE;
    priv->format.colorspace     = V4L2_COLORSPACE_SRGB;

    err = daxc02_power_get(priv);
    if (err) return err;

    daxc02_power_on(common_data);

    /* soft reset */
    err = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_RESET);
    if(err < 0) return err;
    msleep(200);

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

    dev_info(&client->dev, "probe successful.\n");
    return 0;
}

/** daxc02_remove - Called when the driver is removed.
 * @client:     pointer to the i2c client.
 */
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

/*
 * I2C driver setup for registration.
 */
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
MODULE_DESCRIPTION("Nova Dynamics DAX-C02 MIPI camera driver");
MODULE_AUTHOR("Wilkins White <ww@novadynamics.com>");
MODULE_LICENSE("GPL v2");

