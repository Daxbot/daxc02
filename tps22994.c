/* Texas Instruments TPS22994 power switch driver */


#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/module.h>
#include "tps22994.h"


/********************************************************************
        Power Switch Functions
*********************************************************************/

/* Enable the specified channel.
 *    0 = All channels
 *    1 = Channel 1
 *    2 = Channel 2
 *    3 = Channel 3
 *    4 = Channel 4
 */
static int tps22994_enable_channel(struct i2c_client *client, u8 channel)
{
    int retval;
    u8 write_data;

    if(channel == 0) pr_info("tps22994: enabling all channels.");
    else pr_info("tps22994: enabling channel %d.", channel);

    switch(channel)
    {
        case 0:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(~(retval & TPS22994_ALL))
                {
                    write_data = (u8)(retval | TPS22994_ALL);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote 0x%X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 1:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(~(retval & TPS22994_CH1))
                {
                    write_data = (u8)(retval | TPS22994_CH1);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote 0x%X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 2:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(~(retval & TPS22994_CH2))
                {
                    write_data = (u8)(retval | TPS22994_CH2);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote 0x%X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 3:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(~(retval & TPS22994_CH3))
                {
                    write_data = (u8)(retval | TPS22994_CH3);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote 0x%X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 4:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(~(retval & TPS22994_CH4))
                {
                    write_data = (u8)(retval | TPS22994_CH4);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote 0x%X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        default:
            retval = -1;
            break;

    }

    return retval;
}

/* Disable the specified channel.
 *    0 = All channels
 *    1 = Channel 1
 *    2 = Channel 2
 *    3 = Channel 3
 *    4 = Channel 4
 */
static int tps22994_disable_channel(struct i2c_client *client, u8 channel)
{
    int retval;
    u8 write_data;

    if(channel == 0) pr_info("tps22994: disabling all channels.");
        else pr_info("tps22994: disabling channel %d.", channel);

    switch(channel)
    {
        case 0:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(retval & TPS22994_ALL)
                {
                    write_data = (u8)(retval & ~TPS22994_ALL);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote %X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 1:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(retval & TPS22994_CH1)
                {
                    write_data = (u8)(retval & ~TPS22994_CH1);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote %X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 2:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(retval & TPS22994_CH2)
                {
                    write_data = (u8)(retval & ~TPS22994_CH2);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote 0x%X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 3:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(retval & TPS22994_CH3)
                {
                    write_data = (u8)(retval & ~TPS22994_CH3);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote %X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        case 4:
            retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
            pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);

            if(retval >= 0)
            {
                if(retval & TPS22994_CH4)
                {
                    write_data = (u8)(retval & ~TPS22994_CH4);
                    retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
                    if(retval == 0)
                            pr_info("tps22994: wrote %X to R%d.", write_data, TPS22994_REG_CONTROL);
                }
                else retval = 0;
            }
            break;

        default:
            retval = -1;
            break;

    }

    return retval;
}


/* Follow the power up sequence defined in the LI-M021C-MIPI Camera datasheet.
 * Channel on order: 2, 1, 3, 4
 */
int tps22994_power_up(struct i2c_client *client)
{
    int retval;

    pr_info("tps22994: starting power up sequence.\n");

    retval = tps22994_enable_channel(client, 2); // 2.8V PLL
    if(retval == 0)
    {
        usleep_range(10, 20);
        retval = tps22994_enable_channel(client, 3); // 2.8V Analog
        if(retval == 0)
        {
            usleep_range(10, 20);
            retval = tps22994_enable_channel(client, 1); // 1.8V
            if(retval == 0)
            {
                usleep_range(10, 20);
                retval = tps22994_enable_channel(client, 4); // 3.3V Enable Clock
            }
        }
    }

    return retval;
}

/* Follow the power down sequence defined in the LI-M021C-MIPI Camera datasheet.
 * Channel off order: 2, 3, 1, 4
 */
int tps22994_power_down(struct i2c_client *client)
{
    int retval;

    pr_info("tps22994: starting power down sequence.\n");

    retval = tps22994_disable_channel(client, 1); // 1.8V
    if(retval == 0)
    {
        usleep_range(10, 20);
        retval = tps22994_disable_channel(client, 3); // 2.8V Analog
        if(retval == 0)
        {
            usleep_range(10, 20);
            retval = tps22994_disable_channel(client, 2); // 2.8V PLL
            if(retval == 0)
            {
                usleep_range(10, 20);
                retval = tps22994_disable_channel(client, 4);
            }
        }
    }

    return retval;
}

/********************************************************************
        I2C Driver Setup
*********************************************************************/

/* Called on initialization to make sure the device exists and is responding.
 * Checks reading and writing the control register and enables the channels.
 */
static int tps22994_probe(struct i2c_client *client,
            const struct i2c_device_id *did)
{
    int retval;
    u8 write_data;

    retval = i2c_smbus_read_byte_data(client, TPS22994_REG_CONTROL);
    pr_info("tps22994: R%d is 0x%X", TPS22994_REG_CONTROL, retval);
    if(retval >= 0)
    {
        write_data = (u8)(retval | TPS22994_I2C);
        retval = i2c_smbus_write_byte_data(client, TPS22994_REG_CONTROL, write_data);
        if(retval == 0)
        {
            pr_info("tps22994: wrote %X to R%d.", write_data, TPS22994_REG_CONTROL);
            //retval = tps22994_power_up(client);
            retval = tps22994_enable_channel(client, 0); // All Channels to see if the tps22994 is even needed
        }
    }

    if(retval == 0) pr_info("tps22994: probe successful.\n");
    return retval;
}

/* Called when the module is removed.  Disables the power channels.
 */
static int tps22994_remove(struct i2c_client *client)
{
    int retval;
    retval = tps22994_power_down(client);
    if(retval == 0) pr_info("tps22994: remove successful.\n");
    return 0;
}

/* Lists the devices that this driver supports.
 */
static const struct i2c_device_id tps22994_id[] = {
    { "tps22994", 0 },
    { }
};

/* Link to the device tree.
 */
static const struct of_device_id tps22994_of_match[] = {
        { .compatible = "nova,tps22994", },
        { },
};

/* Driver struct used to access all the other functions.
 */
static struct i2c_driver tps22994_i2c_driver = {
    .driver = {
         .name    =        "tps22994",
         .owner =        THIS_MODULE,
    },
    .probe  = tps22994_probe,
    .remove   = tps22994_remove,
    .id_table = tps22994_id,
};

/********************************************************************
        Kernel Module Setup
*********************************************************************/

module_i2c_driver(tps22994_i2c_driver);
MODULE_DEVICE_TABLE(i2c, tps22994_id);
MODULE_DEVICE_TABLE(of, tps22994_of_match);
MODULE_DESCRIPTION("Texas Instruments TPS22994 Switch driver");
MODULE_AUTHOR("Wilkins White <ww@novadynamics.com>");
MODULE_LICENSE("GPL v2");

