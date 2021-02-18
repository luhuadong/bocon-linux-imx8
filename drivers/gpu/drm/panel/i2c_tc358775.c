#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>


static struct i2c_client *this_client;



int  tc358_read_reg(u16 reg,u32 *value)
{
	int r;
        u8 tx_data[] = {
                (reg >> 8) & 0xff,
                reg & 0xff,
        };
        u8 rx_data[4];

        struct i2c_msg msgs[] = {
                {
                        .addr = this_client->addr,
                        .flags = 0,
                        .buf = tx_data,
                        .len = ARRAY_SIZE(tx_data), //写地址
                },
                {
                        .addr = this_client->addr,
                        .flags = I2C_M_RD,
                        .buf = rx_data,
                        .len = ARRAY_SIZE(rx_data), //读数据
                 },
        };

        r = i2c_transfer(this_client->adapter, msgs, ARRAY_SIZE(msgs));
        if (r < 0) {
                printk("%s: reg 0x%04x error %d\n", __func__,
                        reg, r);
                return r;
        }

        if (r < ARRAY_SIZE(msgs)) {
                printk("%s: reg 0x%04x msgs %d\n", __func__,
                        reg, r);
                return -EAGAIN;
        }

        *value = rx_data[0] << 0 | rx_data[1] << 8 |
                rx_data[2] << 16 | rx_data[3] <<24;
		
		printk("%s: reg 0x%04x *value 0x%08x\n", __func__,
                reg, *value);
	
	return 0;

}



int tc358_write_reg(u16 reg, u32 value)
{
        int r;
        u8 tx_data[] = {
                /* NOTE: Register address big-endian, data little-endian. */
                (reg >> 8) & 0xff,
                reg & 0xff,
                value & 0xff,
                (value >> 8) & 0xff,
                (value >> 16) & 0xff,
                (value >> 24) & 0xff,
        };

        struct i2c_msg msgs[] = {
                {
                        .addr = this_client->addr,
                        .flags = 0,
                        .buf = tx_data,
                        .len = ARRAY_SIZE(tx_data),
                },
        };

		if(reg == 0x0000) {
				udelay(value);
				return 0;
		}

        r = i2c_transfer(this_client->adapter, msgs, ARRAY_SIZE(msgs));
        if (r < 0) {
                dev_err(&this_client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
                        __func__, reg, value, r);
                return r;
        }

        if (r < ARRAY_SIZE(msgs)) {
                dev_err(&this_client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
                        __func__, reg, value, r);
                return -EAGAIN;
        }

        printk("%s: reg 0x%04x val 0x%08x\n",
                        __func__, reg, value);

        return 0;
}


EXPORT_SYMBOL(tc358_write_reg);
EXPORT_SYMBOL(tc358_read_reg);

struct cmd_list {
	u16 reg;
	u32 val;
};


#if 0
static struct cmd_list cmd_lists[] = {
	

	{0x013C,0x0008000B},
	{0x0114,0x00000007},
	
	{0x0164,0x0000000B},
	{0x0168,0x0000000B},
	{0x016C,0x0000000B},
	{0x0170,0x0000000B}, 

	{0x0134,0x0000001f},
	{0x0210,0x0000001f},
	{0x0104,0x00000001},
	{0x0204,0x00000001},
	
	{0x0450,0x03200100}, 
	{0x0454,0x003C0050},  
	{0x0458,0x00140780},
	{0x045C,0x00140032}, 
	{0x0460,0x00140438},

  	{0x0464,0x00000001}, 
	{0x04a0,0x00448006},
	{0x0000,100},
	{0x04a0,0x00048006},
	{0x0504,0x00000004},	//这个用了会花屏
/* 用这个 */
 	{0x0480,0x03020100},
	{0x0484,0x08050704},
	{0x0488,0x0F0E0A09},
	{0x048C,0x100d0c0b},
	{0x0490,0x12111716},
	{0x0494,0x1b151413},
	{0x0498,0x061a1918},	

	{0x049c,0x00000063},	
};

#endif



static int tc358775_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u32 val = 0;
	printk("tc358775_probe ok\n");
	
	//检查是否支持I2C功能
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk("i2c_check_functionality fail\n");
		
		return -ENODEV;
	}	


	this_client = client;

	tc358_read_reg(0x13c,&val);
	printk("%s read 0x13c =%x \n",__func__,val);
	return 0;
}

static int  tc358775_remove(struct i2c_client *client)
{

	return 0;
}


static const struct i2c_device_id tc358775_id[] = {
	{ "tc358775", 0 },	//设备类型，ID号
	{ }							//标识结束
};

//添加到内核i2c设备列表
MODULE_DEVICE_TABLE(i2c, tc358775_id);

static const struct of_device_id tc358775_of_match[] = {
	 { .compatible = "TC358775" },
	 { }
};

MODULE_DEVICE_TABLE(of, tc358775_of_match);

// 分配/设置i2c_driver
static struct i2c_driver tc358775_driver = {
	.driver = 
	{
		.name = "tc358775",						
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tc358775_of_match),
	},
	.probe 		= tc358775_probe,					
	.remove 	= tc358775_remove,	
	.id_table = tc358775_id,					
};



static int __init tc358775_drv_init(void)
{
	printk("tc358775_init\n");
	
	
	return i2c_add_driver(&tc358775_driver);
}



static void __exit tc358775_drv_exit(void)
{
	
	i2c_del_driver(&tc358775_driver);
	
	printk("tc358775  exit\n");
}


module_init(tc358775_drv_init);
module_exit(tc358775_drv_exit)




MODULE_AUTHOR("stephenwen88@163.com");			
MODULE_DESCRIPTION("tc358775_init  driver");	
MODULE_LICENSE("GPL");						
MODULE_ALIAS("platform:tc358775_init"); 
