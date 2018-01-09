/*
 * daxc02.c
 *
 * Driver for Nova Dynamics DAX-C02 dual mipi camera board.
 * Used to control Leopard Imaging LI-M021C-MIPI cameras.
 *
 * Copyright (c) 2017 Wilkins White <ww@novadynamics.com>
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

#define DEBUG

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

#include "daxc02.h"
#include "daxc02_mode_tbls.h"
#include "../platform/tegra/camera/camera_gpio.h"

/***************************************************
        TC358746AXBG MIPI Converter Defines
****************************************************/

struct daxc02_mipi_settings {
    uint8_t len;
    uint16_t addr;
    uint32_t data;
};

const struct daxc02_mipi_settings daxc02_single_lane[] = {
    {2, 0x0004, 0x0004}, // parallel port disable
    {2, 0x0002, 0x0001}, // reset 1
    {2, 0x0002, 0x0000}, // reset 0
    {2, 0x0016, 0x50F9}, // set the input and feedback frequency division ratio
    {2, 0x0018, 0x0213}, // 50% maximum loop bandwidth + PLL clock enable + normal operation + PLL enable

    {2, 0x0006, 0x0030}, // FIFO level 3
    {2, 0x0008, 0x0020}, // data format RAW12
    {2, 0x0022, 0x0780}, // word count (bytes per line)

    {4, 0x0140, 0x00000000}, // clock lane enable
    {4, 0x0144, 0x00000000}, // data lane 0 enable
    {4, 0x0148, 0x00000001}, // data lane 1 disable
    {4, 0x014C, 0x00000001}, // data lane 2 disable
    {4, 0x0150, 0x00000001}, // data lane 3 disable

    {4, 0x0210, 0x00002C00}, // line intialization wait counter
    {4, 0x0214, 0x00000005}, // timing generation counter
    {4, 0x0218, 0x00001E06}, // clock header counter
    {4, 0x021C, 0x00000004}, // clock trail counter
    {4, 0x0220, 0x00000406}, // data header counter
    {4, 0x0224, 0x00004988}, // wakeup counter
    {4, 0x0228, 0x0000000C}, // clock post counter
    {4, 0x022C, 0x00000006}, // data trail counter
    {4, 0x0234, 0x00000003}, // voltage regulator enable for clock data 0
    {4, 0x0238, 0x00000000}, // discontinuous clock mode.
    {4, 0x0204, 0x00000001}, // TX PPI start

    {4, 0x0518, 0x00000001}, // CSI start
    {4, 0x0500, 0xA30080A1}, // 1 data lane

    {2, 0x0004, 0x0044}, // increment I2C, parallel port enable, 1 csi lane
};

const struct daxc02_mipi_settings daxc02_double_lane[] = {
    {2, 0x0004, 0x0004}, // parallel port disable
    {2, 0x0002, 0x0001}, // reset 1
    {2, 0x0002, 0x0000}, // reset 0
    {2, 0x0016, 0x50C6}, // set the input and feedback frequency division ratio
    {2, 0x0018, 0x0213}, // 50% maximum loop bandwidth + PLL clock enable + normal operation + PLL enable

    {2, 0x0006, 0x0030}, // FIFO level 3
    {2, 0x0008, 0x0020}, // data format RAW12
    {2, 0x0022, 0x0780}, // word count (bytes per line)

    {4, 0x0140, 0x00000000}, // clock lane enable
    {4, 0x0144, 0x00000000}, // data lane 0 enable
    {4, 0x0148, 0x00000000}, // data lane 1 enable
    {4, 0x014C, 0x00000001}, // data lane 2 disable
    {4, 0x0150, 0x00000001}, // data lane 3 disable

    {4, 0x0210, 0x00002C00}, // line initialization wait counter
    {4, 0x0214, 0x00000005}, // timing generation counter
    {4, 0x0218, 0x00001E05}, // clock header counter
    {4, 0x021C, 0x00000002}, // clock trail counter
    {4, 0x0220, 0x00000204}, // data header counter
    {4, 0x0224, 0x00004988}, // wakeup counter
    {4, 0x0228, 0x00000009}, // clock post counter
    {4, 0x022C, 0x00000003}, // data trail counter
    {4, 0x0234, 0x00000007}, // voltage regulator enable for clock and data lanes 0-1
    {4, 0x0238, 0x00000000}, // discontinuous clock mode.
    {4, 0x0204, 0x00000001}, // TX PPI start

    {4, 0x0518, 0x00000001}, // CSI start
    {4, 0x0500, 0xA30080A3}, // 2 data lanes

    {2, 0x0004, 0x0045}, // increment I2C, parallel port enable, 2 csi lanes
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

    enum v4l2_exposure_auto_type        autoexposure;
    const char*                         trigger_mode;

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
static int mt9m021_write_table(struct i2c_client *client, const struct reg_16 table[]);
static int daxc02_bridge_setup(struct i2c_client *client);
static int mt9m021_is_streaming(struct i2c_client *client);
static uint8_t mt9m021_calculate_gain(struct i2c_client *client, uint8_t value);
static int mt9m021_set_autoexposure(struct i2c_client *client, enum v4l2_exposure_auto_type ae_mode);
static int mt9m021_set_flash(struct i2c_client *client, enum v4l2_flash_led_mode flash_mode);
static int mt9m021_s_stream(struct v4l2_subdev *sd, int enable);
static int daxc02_g_input_status(struct v4l2_subdev *sd, uint32_t *status);
static int mt9m021_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format);
static int mt9m021_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format);
static int daxc02_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
static int daxc02_get_trigger_mode(struct daxc02 *priv);
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

    if(priv->power.state == SWITCH_OFF) return 0;

    switch (ctrl->id)
    {
        case V4L2_CID_FLASH_LED_MODE:
            dev_dbg(&client->dev, "%s: V4L2_CID_FLASH_LED_MODE - %d\n", __func__, ctrl->val);
            ret = mt9m021_set_flash(client, (enum v4l2_flash_led_mode)ctrl->val);
            break;

        case V4L2_CID_EXPOSURE_AUTO:
            dev_dbg(&client->dev, "%s: V4L2_CID_EXPOSURE_AUTO - %d\n", __func__, ctrl->val);
            ret = mt9m021_set_autoexposure(client, (enum v4l2_exposure_auto_type)ctrl->val);
            if(ret < 0) return ret;
            break;

        case V4L2_CID_COARSE_TIME:
            dev_dbg(&client->dev, "%s: V4L2_CID_COARSE_TIME - %d\n", __func__, ctrl->val);
            ret = mt9m021_write(client, MT9M021_COARSE_INT_TIME, ctrl->val);
            break;

        case V4L2_CID_COARSE_TIME_SHORT:
            dev_dbg(&client->dev, "%s: V4L2_CID_COARSE_TIME_SHORT - %d\n", __func__, ctrl->val);
            ret = mt9m021_write(client, MT9M021_FINE_INT_TIME, ctrl->val);
            break;

        case V4L2_CID_GAIN:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_calculate_gain(client, ctrl->val);
            ret = mt9m021_write(client, MT9M021_GLOBAL_GAIN, reg16);
            break;

        case V4L2_CID_GAIN_GREEN1:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_GREEN1 - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_calculate_gain(client, ctrl->val);
            ret = mt9m021_write(client, MT9M021_GREEN1_GAIN, reg16);
            break;

        case V4L2_CID_GAIN_RED:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_RED - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_calculate_gain(client, ctrl->val);
            ret = mt9m021_write(client, MT9M021_RED_GAIN, reg16);
            break;

        case V4L2_CID_GAIN_BLUE:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_BLUE - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_calculate_gain(client, ctrl->val);
            ret = mt9m021_write(client, MT9M021_BLUE_GAIN, reg16);
            break;

        case V4L2_CID_GAIN_GREEN2:
            dev_dbg(&client->dev, "%s: V4L2_CID_GAIN_GREEN2 - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_calculate_gain(client, ctrl->val);
            ret = mt9m021_write(client, MT9M021_GREEN2_GAIN, reg16);
            break;

        case V4L2_CID_ANALOG_GAIN:
            dev_dbg(&client->dev, "%s: V4L2_CID_ANALOG_GAIN - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_read(client, MT9M021_DIGITAL_TEST);
            reg16 &= ~MT9M021_ANALOG_GAIN_MASK;
            reg16 |= ((ctrl->val << MT9M021_ANALOG_GAIN_SHIFT) & MT9M021_ANALOG_GAIN_MASK);
            ret = mt9m021_write(client, MT9M021_DIGITAL_TEST, reg16);
            break;

        case V4L2_CID_HFLIP:
            dev_dbg(&client->dev, "%s: V4L2_CID_HFLIP - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_read(client, MT9M021_READ_MODE);
            if(ctrl->val)
            {
                reg16 |= 0x4000;
                ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
                if(ret < 0) return ret;
                break;
            }
            reg16 &= 0xbfff;
            ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
            break;

        case V4L2_CID_VFLIP:
            dev_dbg(&client->dev, "%s: V4L2_CID_VFLIP - %d\n", __func__, ctrl->val);
            reg16 = mt9m021_read(client, MT9M021_READ_MODE);
            if(ctrl->val)
            {
                reg16 |= 0x8000;
                ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
                if(ret < 0) return ret;
                break;
            }
            reg16 &= 0x7fff;
            ret = mt9m021_write(client, MT9M021_READ_MODE, reg16);
            break;

        case V4L2_CID_TEST_PATTERN:
            dev_dbg(&client->dev, "%s: V4L2_CID_TEST_PATTERN - %d\n", __func__, ctrl->val);
            if(!ctrl->val)
            {
                ret = mt9m021_write(client, MT9M021_TEST_PATTERN, 0x0000);
                if(ret < 0) return ret;
            }
            ret = mt9m021_write(client, MT9M021_TEST_PATTERN, ctrl->val);
            break;

        case V4L2_CID_FRAME_LENGTH:
            dev_dbg(&client->dev, "%s: V4L2_CID_FRAME_LENGTH - %d\n", __func__, ctrl->val);
            ret = mt9m021_write(client, MT9M021_FRAME_LENGTH_LINES, ctrl->val);
            break;

        default:
            dev_err(&client->dev, "%s: UNKNOWN CTRL ID - %d\n", __func__, ctrl->val);
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
        .id             = V4L2_CID_GAIN_GREEN1,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Green (R)",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN_RED,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Red",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN_BLUE,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Blue",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_GAIN_GREEN2,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Green (B)",
        .min            = MT9M021_GLOBAL_GAIN_MIN,
        .max            = MT9M021_GLOBAL_GAIN_MAX,
        .def            = MT9M021_GLOBAL_GAIN_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_ANALOG_GAIN,
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .name           = "Gain, Column",
        .min            = MT9M021_ANALOG_GAIN_MIN,
        .max            = MT9M021_ANALOG_GAIN_MAX,
        .def            = MT9M021_ANALOG_GAIN_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_COARSE_TIME,
        .name           = "Coarse Time",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = V4L2_CTRL_FLAG_SLIDER,
        .min            = 0x1,
        .max            = 0x5DC,
        .def            = MT9M021_COARSE_INT_TIME_DEF,
        .step           = 1,
    },
    {
        .ops            = &daxc02_ctrl_ops,
        .id             = V4L2_CID_COARSE_TIME_SHORT,
        .name           = "Coarse Time Short",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = V4L2_CTRL_FLAG_SLIDER,
        .min            = 0x0,
        .max            = MT9M021_LLP_RECOMMENDED-750,
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
        .id             = V4L2_CID_FLASH_LED_MODE,
        .name           = "Flash",
        .type           = V4L2_CTRL_TYPE_INTEGER,
        .flags          = 0,
        .min            = V4L2_FLASH_LED_MODE_NONE,
        .max            = V4L2_FLASH_LED_MODE_FLASH,
        .def            = V4L2_FLASH_LED_MODE_FLASH,
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
        .min            = MT9M021_WINDOW_HEIGHT_MIN + 37,
        .max            = 0x5DC,
        .def            = MT9M021_WINDOW_HEIGHT_DEF + 37,
        .step           = 1,
    },
};


/***************************************************
        DAX-C02 Power Functions
****************************************************/

/** daxc02_gpio_set - Turns on or off gpio
  * @priv: DAX-C02 private data struct.
  * @gpio: Gpio to target.
  * @val:  Value to set gpio.
  */
  static void daxc02_gpio_set(struct daxc02 *priv, unsigned int gpio, int val)
  {
    if(gpio_cansleep(gpio)) gpio_set_value_cansleep(gpio, val);
    else gpio_set_value(gpio, val);
  }

/** daxc02_power_on - Turns on the needed TX1 voltage regulators.
  * @s_data: Nvidia camera common data struct.
  */
static int daxc02_power_on(struct camera_common_data *s_data)
{
    int ret = 0;
    struct daxc02 *priv = (struct daxc02 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;
    struct i2c_client *client = s_data->i2c_client;

    dev_dbg(&client->dev, "%s\n", __func__);

    if(priv->pdata && priv->pdata->power_on)
    {
        ret = priv->pdata->power_on(pw);
        if(ret) pr_err("%s failed.\n", __func__);
        else pw->state = SWITCH_ON;
        return ret;
    }

    /* sleeps calls in the sequence below are for internal device
     * signal propagation as specified by sensor vendor */

    if(pw->dvdd) ret = regulator_enable(pw->dvdd);     // 1.2V
    if(ret) goto daxc02_dvdd_fail;

    usleep_range(50, 100);
    if(pw->avdd) ret = regulator_enable(pw->avdd);     // 2.8V
    if(ret) goto daxc02_avdd_fail;

    usleep_range(50, 100);
    if(pw->iovdd) ret = regulator_enable(pw->iovdd);   // 1.8V
    if(ret) goto daxc02_iovdd_fail;

    msleep(30);
    if(pw->reset_gpio) daxc02_gpio_set(priv, pw->reset_gpio, 1);
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
    int ret = 0;
    struct daxc02 *priv = (struct daxc02 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;

    dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

    if(priv->pdata && priv->pdata->power_on)
    {
        ret = priv->pdata->power_off(pw);
        if(!ret) pw->state = SWITCH_OFF;
        else pr_err("%s failed.\n", __func__);
        return ret;
    }

    if(pw->reset_gpio) daxc02_gpio_set(priv, pw->reset_gpio, 0);
    usleep_range(50, 100);
    if(pw->iovdd) regulator_disable(pw->iovdd);
    usleep_range(50, 100);
    if(pw->avdd) regulator_disable(pw->avdd);
    usleep_range(50, 100);
    if(pw->dvdd) regulator_disable(pw->dvdd);

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
    int ret = 0;

    dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

    if (!pdata)
    {
        dev_err(&priv->i2c_client->dev, "pdata missing\n");
        return -EFAULT;
    }

    mclk_name = pdata->mclk_name ? pdata->mclk_name : "cam_mclk1";
    pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
    if(IS_ERR(pw->mclk))
    {
        dev_err(&priv->i2c_client->dev, "unable to get clock %s\n", mclk_name);
        return PTR_ERR(pw->mclk);
    }

    parentclk_name = pdata->parentclk_name;
    if (parentclk_name)
    {
        parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
        if(IS_ERR(parent)) dev_err(&priv->i2c_client->dev, "unable to get parent clock %s", parentclk_name);
        else clk_set_parent(pw->mclk, parent);
    }

    /* 1.2v */
    ret |= camera_common_regulator_get(priv->i2c_client, &pw->dvdd, pdata->regulators.dvdd);
    /* analog 2.8v */
    ret |= camera_common_regulator_get(priv->i2c_client, &pw->avdd, pdata->regulators.avdd);
    /* IO 1.8v */
    ret |= camera_common_regulator_get(priv->i2c_client, &pw->iovdd, pdata->regulators.iovdd);

    if(!ret)
    {
        pw->reset_gpio = pdata->reset_gpio;
        gpio_request(pw->reset_gpio, "cam_reset_gpio");

        // TODO: Figure out why this always returns EBUSY
        //err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
        //if(err < 0) dev_dbg(&priv->i2c_client->dev, "%s: can't request reset_gpio %d\n", __func__, err);
    }

    pw->state = SWITCH_OFF;
    return ret;
}

/** daxc02_power_put - Frees the voltage regulators.
 * @priv: Dax-C02 private data structure.
 */
static int daxc02_power_put(struct daxc02 *priv)
{
    struct camera_common_power_rail *pw = &priv->power;
    dev_dbg(&priv->i2c_client->dev, "%s\n", __func__);

    if(unlikely(!pw)) return -EFAULT;

    if(likely(pw->iovdd)) regulator_put(pw->iovdd);

    if(likely(pw->avdd)) regulator_put(pw->avdd);

    if(likely(pw->dvdd)) regulator_put(pw->dvdd);

    pw->avdd = NULL;
    pw->iovdd = NULL;
    pw->dvdd = NULL;

    if(pw->reset_gpio) gpio_free(pw->reset_gpio);

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

    if(ret < 0)
    {
        dev_err(&client->dev, "read failed at 0x%04x error %d\n", addr, ret);
        return ret;
    }

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
    if(ret == 1) return 0;

    dev_err(&client->dev, "write failed at 0x%04x error %d\n", addr, ret);
    return ret;
}

static int mt9m021_write_table(struct i2c_client *client, const struct reg_16 table[])
{
    const struct reg_16 *next;
    int ret = 0;

    for(next = table;; next++)
    {
        if(next->addr == MT9M021_TABLE_END) break;
        else if(next->addr == MT9M021_TABLE_SLEEP)
        {
            msleep_range(next->val);
        }
        else
        {
            ret = mt9m021_write(client, next->addr, next->val);
            if(ret < 0) break;
        }
    }

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

    for(i = 0; i < ARRAY_SIZE(daxc02_single_lane); i++)
    {
        settings = daxc02_single_lane[i];

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

        if(ret < 0)
        {
            dev_err(&client->dev, "%s failed at 0x%04x error %d\n", __func__, settings.addr, ret);
            break;
        }
    }

    return ret;
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

/** mt9m021_calculate_gain - sets the digital gain.
 * @client: pointer to the i2c client.
 * @value: gain to set [1-224].
 */
static uint8_t mt9m021_calculate_gain(struct i2c_client *client, uint8_t value)
{
    uint16_t integer_gain, fractional_gain;

    if(value < MT9M021_GLOBAL_GAIN_MIN || value > MT9M021_GLOBAL_GAIN_MAX)
        return -EINVAL;

    integer_gain = ((value-1) >> 5) + 1;
    fractional_gain = (value-1) % (1<<5);

    dev_dbg(&client->dev, "%s: %u + %u/32", __func__, integer_gain, fractional_gain);

    return (integer_gain << 5) | fractional_gain;
}

/** mt9m021_set_flash - enables or disables flash.
 * @client: pointer to the i2c client.
 * @flash_mode: v4l2 flash mode.
 */
static int mt9m021_set_flash(struct i2c_client *client, enum v4l2_flash_led_mode flash_mode )
{
    int ret = 0;

    dev_dbg(&client->dev, "%s\n", __func__);

    switch(flash_mode)
    {
        case V4L2_FLASH_LED_MODE_NONE:
            ret = mt9m021_write(client, MT9M021_FLASH, 0x0000);
            break;

        case V4L2_FLASH_LED_MODE_FLASH:
            ret = mt9m021_write(client, MT9M021_FLASH, 0x0180);
            break;

        case V4L2_FLASH_LED_MODE_TORCH:
            dev_err(&client->dev, "Unsupported flash mode requested: %d\n", flash_mode);
            ret = -EINVAL;
            break;

        default:
            dev_err(&client->dev, "Flash mode out of range: %d\n", flash_mode);
            break;
    }

    return ret;
}

/** mt9m021_set_autoexposure - enables or disables autoexposure.
 * @client: pointer to the i2c client.
 * @ae_mode: v4l2 autoexposure mode.
 */
static int mt9m021_set_autoexposure(struct i2c_client *client, enum v4l2_exposure_auto_type ae_mode )
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
            if(streaming)
            {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
                if(ret < 0) return ret;
            }

            ret = mt9m021_write(client, MT9M021_EMBEDDED_DATA_CTRL, 0x1802);
            if(ret < 0) return ret;
            ret = mt9m021_write(client, MT9M021_AE_CTRL, 0x0000);
            if(ret < 0) return ret;

            if(streaming)
            {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_ON);
                if(ret < 0) return ret;
            }
            break;

        case V4L2_EXPOSURE_SHUTTER_PRIORITY:
            if(streaming)
            {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
                if(ret < 0) return ret;
            }

            ret = mt9m021_write(client, MT9M021_EMBEDDED_DATA_CTRL, 0x1982);
            if(ret < 0) return ret;
            ret = mt9m021_write(client, MT9M021_AE_CTRL, 0x0013);
            if(ret < 0) return ret;

            if(streaming)
            {
                ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_ON);
                if(ret < 0) return ret;
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
    struct camera_common_data *common_data = to_camera_common_data(client);
    struct daxc02 *priv = (struct daxc02 *)common_data->priv;
    struct v4l2_control control;
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    if(!enable)
    {
        dev_info(&client->dev, "Ending stream\n");
        return mt9m021_write(client, MT9M021_RESET_REG, MT9M021_STREAM_OFF);
    }
    else dev_info(&client->dev, "Starting stream\n");

    ret = daxc02_bridge_setup(client);
    if(ret < 0)
    {
        dev_err(&client->dev, "%s: Failed to setup mipi bridge\n", __func__);
        return ret;
    }

    ret = mt9m021_write_table(client, daxc02_mode_table_common);
    if(ret < 0)
    {
        dev_err(&client->dev, "%s: failed to configure mt9m021.\n", __func__);
        return ret;
    }

    ret = mt9m021_write_table(client, mode_table[priv->s_data->mode]);
    if(ret < 0)
    {
        dev_err(&client->dev, "%s: failed to set mode.\n", __func__);
        return ret;
    }

    /* write list of override regs for the asking frame length,
     * coarse integration time, and gain. Failures to write
     * overrides are non-fatal */
    control.id = V4L2_CID_GAIN;
    ret = v4l2_g_ctrl(&priv->ctrl_handler, &control);
    ret |= mt9m021_write(client, MT9M021_GLOBAL_GAIN, mt9m021_calculate_gain(client, control.value));
    if(ret < 0) dev_dbg(&client->dev, "%s: warning gain override failed\n", __func__);

    control.id = V4L2_CID_FRAME_LENGTH;
    ret = v4l2_g_ctrl(&priv->ctrl_handler, &control);
    ret |= mt9m021_write(client, MT9M021_FRAME_LENGTH_LINES, control.value);
    if(ret < 0) dev_dbg(&client->dev, "%s: warning frame length override failed\n", __func__);

    control.id = V4L2_CID_COARSE_TIME;
    ret = v4l2_g_ctrl(&priv->ctrl_handler, &control);
    ret |= mt9m021_write(client, MT9M021_COARSE_INT_TIME, control.value);
    if(ret < 0) dev_dbg(&client->dev, "%s: warning coarse integration time override failed\n", __func__);

    control.id = V4L2_CID_COARSE_TIME_SHORT;
    ret = v4l2_g_ctrl(&priv->ctrl_handler, &control);
    ret |= mt9m021_write(client, MT9M021_FINE_INT_TIME, control.value);
    if(ret < 0) dev_dbg(&client->dev, "%s: warning fine integration time override failed\n", __func__);

    /* start streaming */
    if(strstr(priv->trigger_mode, "slave") != NULL)
    {
        ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_TRIGGER_MODE);
    }
    else ret = mt9m021_write(client, MT9M021_RESET_REG, MT9M021_MASTER_MODE);

    return ret;
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
    .g_mbus_config          = camera_common_g_mbus_config,
    .g_input_status         = daxc02_g_input_status,
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

static int mt9m021_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format)
{
    return camera_common_g_fmt(sd, &format->format);
}

static int mt9m021_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format)
{
    if (format->which == V4L2_SUBDEV_FORMAT_TRY) return camera_common_try_fmt(sd, &format->format);
    else return camera_common_s_fmt(sd, &format->format);
}

/*
 * Registers the v4l2 sub-device pad operations.
 */
static struct v4l2_subdev_pad_ops mt9m021_subdev_pad_ops = {
    .get_fmt                = mt9m021_get_format,
    .set_fmt                = mt9m021_set_format,
    .enum_mbus_code         = camera_common_enum_mbus_code,
    .enum_frame_size        = camera_common_enum_framesizes,
    .enum_frame_interval     = camera_common_enum_frameintervals,
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

/** daxc02_open - Called by the v4l2 framework when the device is opened.
 * @sd:     pointer to the v4l2 sub-device.
 * @fh:     pointer to the v4l2 sub-device file handle.
 */
static int daxc02_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    dev_dbg(&client->dev, "%s\n", __func__);
    return 0;
}

/*
 * Registers the v4l2 sub-device internal operations.
 */
static const struct v4l2_subdev_internal_ops mt9m021_subdev_internal_ops = {
    .open               = daxc02_open,
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

/** daxc02_parse_dt - Fetch the trigger mode from the device tree
 * @priv: pointer to the daxc02 private structure
 */
 static int daxc02_get_trigger_mode(struct daxc02 *priv)
 {
    int ret = 0;
    struct i2c_client *client = priv->i2c_client;
    struct device_node *node = client->dev.of_node;
    const struct of_device_id *match;

    dev_dbg(&client->dev, "%s\n", __func__);

    if(!node) return -EFAULT;

    match = of_match_device(daxc02_of_match, &client->dev);
    if(!match)
    {
        dev_err(&client->dev, "Failed to find matching dt id\n");
        return -EFAULT;
    }

    ret = of_property_read_string(node, "trigger_mode", &priv->trigger_mode);
    if(ret == -EINVAL)
    {
        dev_warn(&client->dev, "trigger_mode not in device tree\n");
        *(&priv->trigger_mode) = "master";
    }

    return 0;
 }

/** daxc02_parse_dt - Parses the device tree to load camera common data.
 * @client: pointer to the i2c client.
 */
static struct camera_common_pdata *daxc02_parse_dt(struct i2c_client *client)
{
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    struct camera_common_pdata *board_priv_pdata = NULL;
    int gpio;
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    if(!node) return NULL;

    match = of_match_device(daxc02_of_match, &client->dev);
    if(!match)
    {
        dev_err(&client->dev, "Failed to find matching dt id\n");
        return NULL;
    }

    board_priv_pdata = devm_kzalloc(&client->dev,
               sizeof(*board_priv_pdata), GFP_KERNEL);
    if(!board_priv_pdata) return NULL;

    ret = camera_common_parse_clocks(client, board_priv_pdata);
    if(ret)
    {
        dev_err(&client->dev, "Failed to find clocks\n");
        goto error;
    }

    gpio = of_get_named_gpio(node, "reset-gpios", 0);
    if(gpio == -EPROBE_DEFER)
    {
        board_priv_pdata = ERR_PTR(-EPROBE_DEFER);
        goto error;
    }
    else if (gpio < 0)
    {
            /* reset-gpio is not absoluctly needed */
            dev_dbg(&client->dev, "reset gpios not in DT\n");
            gpio = 0;
    }
    board_priv_pdata->reset_gpio = (unsigned int)gpio;

    ret = of_property_read_string(node, "avdd-reg", &board_priv_pdata->regulators.avdd);
    if(ret)
    {
        dev_err(&client->dev, "avdd-reg not in DT\n");
        goto error;
    }

    ret = of_property_read_string(node, "iovdd-reg", &board_priv_pdata->regulators.iovdd);
    if(ret)
    {
        dev_err(&client->dev, "iovdd-reg not in DT\n");
        goto error;
    }

    ret = of_property_read_string(node, "dvdd-reg", &board_priv_pdata->regulators.dvdd);
    if(ret)
    {
        dev_err(&client->dev, "dvdd-reg not in DT\n");
        goto error;
    }

    board_priv_pdata->has_eeprom = of_property_read_bool(node, "has-eeprom");

    return board_priv_pdata;

    error:
        devm_kfree(&client->dev, board_priv_pdata);
        return board_priv_pdata;
}

/** daxc02_ctrls_init - Registers and initializes controls with the v4l2 framework.
 * @priv: Dax-C02 private data structure.
 */
static int daxc02_ctrls_init(struct daxc02 *priv)
{
    struct i2c_client *client = priv->i2c_client;
    struct v4l2_ctrl *ctrl;
    int numctrls;
    int ret;
    int i;

    dev_dbg(&client->dev, "%s\n", __func__);

    numctrls = ARRAY_SIZE(ctrl_config_list);
    dev_dbg(&client->dev, "initializing %d controls\n", numctrls);
    v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

    for(i = 0; i < numctrls; i++)
    {
        dev_dbg(&client->dev, "control %d: %s\n", i, ctrl_config_list[i].name);
        ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl_config_list[i], NULL);
        if(ctrl == NULL)
        {
            dev_err(&client->dev, "Failed to init %s ctrl\n", ctrl_config_list[i].name);
            continue;
        }

        if(ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
          (ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY))
        {
            ctrl->p_new.p_char = devm_kzalloc(&client->dev, ctrl_config_list[i].max + 1, GFP_KERNEL);
            if (!ctrl->p_new.p_char) return -ENOMEM;
        }
        priv->ctrls[i] = ctrl;
    }

    priv->numctrls = numctrls;
    priv->subdev->ctrl_handler = &priv->ctrl_handler;
    if(priv->ctrl_handler.error)
    {
        dev_err(&client->dev, "Error %d adding controls\n", priv->ctrl_handler.error);
        ret = priv->ctrl_handler.error;
        goto error;
    }

    ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
    if(ret)
    {
        dev_err(&client->dev, "Error %d setting default controls\n", ret);
        goto error;
    }

    return 0;

    error:
        v4l2_ctrl_handler_free(&priv->ctrl_handler);
        return ret;
}

/** daxc02_probe - Checks if the device is present and performs one time initialization.
 * @client:     pointer to the i2c client.
 * @id:         i2c device id.
 */
static int daxc02_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct camera_common_data *common_data;
    struct device_node *node = client->dev.of_node;
    struct daxc02 *priv;
    char debugfs_name[10];
    uint16_t reg16;
    int ret;

    dev_dbg(&client->dev, "%s\n", __func__);

    if(!IS_ENABLED(CONFIG_OF) || !node) return -EINVAL;

    common_data = devm_kzalloc(&client->dev, sizeof(struct camera_common_data), GFP_KERNEL);
    if(!common_data) return -ENOMEM;

    priv = devm_kzalloc(&client->dev,
                sizeof(struct daxc02) + sizeof(struct v4l2_ctrl *) *
                ARRAY_SIZE(ctrl_config_list),
                GFP_KERNEL);
    if(!priv) return -ENOMEM;

    priv->pdata = daxc02_parse_dt(client);
    if(PTR_ERR(priv->pdata) == -EPROBE_DEFER) return -EPROBE_DEFER;
    else if (!priv->pdata)
    {
        dev_err(&client->dev, "unable to get device tree platform data\n");
        return -EFAULT;
    }

    common_data->ops                = &daxc02_common_ops;
    common_data->ctrl_handler       = &priv->ctrl_handler;
    common_data->i2c_client         = client;
    common_data->frmfmt             = daxc02_frmfmt;
    common_data->colorfmt           = camera_common_find_datafmt(MEDIA_BUS_FMT_SRGGB12_1X12);
    common_data->power              = &priv->power;
    common_data->ctrls              = priv->ctrls;
    common_data->priv               = (void *)priv;
    common_data->numctrls           = ARRAY_SIZE(ctrl_config_list);
    common_data->def_mode           = MT9M021_MODE_1280X720;
    common_data->def_width          = MT9M021_PIXEL_ARRAY_WIDTH;
    common_data->def_height         = MT9M021_PIXEL_ARRAY_HEIGHT;
    common_data->fmt_width          = common_data->def_width;
    common_data->fmt_height         = common_data->def_height;
    common_data->def_clk_freq       = MT9M021_TARGET_FREQ;

    priv->i2c_client                = client;
    priv->s_data                    = common_data;
    priv->subdev                    = &common_data->subdev;
    priv->subdev->dev               = &client->dev;
    priv->s_data->dev               = &client->dev;

    ret = daxc02_get_trigger_mode(priv);
    if(ret) return ret;

    if(strstr(priv->trigger_mode, "slave") != NULL)
    {
        dev_info(&client->dev, "slave mode activated\n");
    }

    ret = daxc02_power_get(priv);
    if(ret) return ret;

    ret = daxc02_power_on(common_data);
    if(ret) return ret;

    reg16 = mt9m021_read(client, MT9M021_CHIP_ID_REG);
    if(ret || reg16 != MT9M021_CHIP_ID)
    {
        dev_err(&client->dev, "Aptina MT9M021 not detected.\n");
        return -ENODEV;
    }
    else dev_info(&client->dev, "Aptina MT9M021 detected!\n");

    ret = camera_common_parse_ports(client, common_data);
    if(ret)
    {
        dev_err(&client->dev, "Failed to find port info\n");
        return ret;
    }
    sprintf(debugfs_name, "daxc02_%c", common_data->csi_port + 'a');
    camera_common_create_debugfs(common_data, debugfs_name);

    v4l2_i2c_subdev_init(priv->subdev, client, &daxc02_subdev_ops);

    ret = daxc02_ctrls_init(priv);
    if(ret) return ret;

    priv->subdev->internal_ops = &mt9m021_subdev_internal_ops;
    priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

    #ifdef CONFIG_MEDIA_CONTROLLER
    dev_dbg(&client->dev, "initializing media entity.\n");
    priv->pad.flags = MEDIA_PAD_FL_SOURCE;
    priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
    priv->subdev->entity.ops = &daxc02_media_ops;
    ret = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
    if(ret < 0)
    {
        dev_err(&client->dev, "unable to init media entity\n");
        return ret;
    }
    #endif

    ret = v4l2_async_register_subdev(priv->subdev);
    if(ret) return ret;

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
