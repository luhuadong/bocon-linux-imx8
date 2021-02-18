#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/pwm.h>


/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77


/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};


static const struct cmd_set_entry manufacturer_cmd_set[] = {
{0x20 , 0x00},
{0x21 , 0x00},
{0x22 , 0x34},
{0x23 , 0x50},
{0x24 , 0x14},
{0x25 , 0x50},
{0x26 , 0x00},
{0x27 , 0x14},
{0x28 , 0x0A},
{0x29 , 0x14},
{0x34 , 0x80},
{0x36 , 0x50},
{0xB5 , 0xA0},
{0x5C , 0xFF},
{0x56 , 0x92},
{0x6B , 0x32},
{0x69 , 0x3B},
{0xB6 , 0x20},
{0x51 , 0x20},
{0x09 , 0x10},
};

//28,49s/=/,/g  	28,49s/^/{/g	28,49s/$/},/g

static const u32 ct_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 ct_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_NEGEDGE;


struct ct_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *pwr; 
	struct gpio_desc *reset;  /*复位*/
	struct gpio_desc *blken;  /*背光使能*/
	struct gpio_desc *stby;  /*背光使能*/
	
	struct backlight_device *backlight;
	struct pwm_device	*bl;//背光
	struct pwm_args pargs; 
	
	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

//时序参数 内核源码有注解意义
static const struct drm_display_mode default_mode = {
	.clock = 128000, //RGB频率
	.hdisplay = 1920,
	.hsync_start = 1920 + 20, //+fp
	.hsync_end = 1920 + 20 + 80,//+fp +sync
	.htotal = 1920 + 20 + 80 + 60,
	.vdisplay = 1080,
	.vsync_start = 1080 + 20,
	.vsync_end = 1080 + 20 + 50,
	.vtotal = 1080 + 20 + 50 + 20,
	.vrefresh = 60, //刷屏速度
	.width_mm = 68,
	.height_mm = 121,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static inline struct ct_panel *to_ct_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ct_panel, panel);
}


/*读寄存器*/
static int mipi_dsi_read_reg(struct mipi_dsi_device *dsi,u8 addr,uint8_t *val)
{
	u8 date;
	ssize_t ret;
	u8 reg[2] ={addr,0x01}; //第二个是读的长度
	ret =   mipi_dsi_generic_read(dsi,reg,2,&date,1);

	if(ret <0) {
		printk("ret :%zd addr:%02x date:%02x\n",ret,addr,date);
		return ret;
	}
	*val = date;
	return ret;
}


/*
写8位的寄存器
*/
static int mipi_dsi_write_reg(struct mipi_dsi_device *dsi,uint8_t addr, uint8_t val)
{
	int ret; 
	uint8_t buffer[2] = {addr,val};
	ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer)); 
	if (ret < 0) {
		printk("%s write addr :%02x fail\n",__func__,addr);
		return ret;
	}
	//printk("addr:%02x val:%02x\n",addr,val);
	return  0;
}



static int ct_panel_push_cmd_list(struct mipi_dsi_device *dsi)
{
	int ret = 0;
	size_t i;
	size_t count = ARRAY_SIZE(manufacturer_cmd_set);/* 初始化命令列表 */
	uint8_t ic_type[2];
	uint8_t val;
	
	
	/*写入配置*/
	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = &manufacturer_cmd_set[i];
		u8 buffer[2] = { entry->cmd, entry->param };
		
		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0) {
			printk("%s %d write fail ADDR:%02x, VAL:%02x \n",__func__,__LINE__,
			entry->cmd, entry->param);
			return ret;
		}else
		{
			printk("mipi write ADDR:%02x, VAL:%02x \n",
			entry->cmd, entry->param);
		}
	}
	
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_read_reg(dsi,0x01,&ic_type[0]);
	mipi_dsi_read_reg(dsi,0x02,&ic_type[1]);
	printk("dsi bridge ic_type %02x%02x",ic_type[0],ic_type[1]);

	mipi_dsi_write_reg(dsi,0x51,0x20);
	mipi_dsi_read_reg(dsi,0x51,&val);
	printk("read 0x51 = %02x",val);
	mipi_dsi_read_reg(dsi,0x27,&val);
	printk("read 0x27 = %02x",val);

	ret = 2;
	return ret;
};


//设置格式
static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return COL_FMT_16BPP;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return COL_FMT_18BPP;
	case MIPI_DSI_FMT_RGB888:
		return COL_FMT_24BPP;
	default:
		return COL_FMT_24BPP; /* for backward compatibility */
	}
};

static int ct_panel_prepare(struct drm_panel *panel)
{
	struct ct_panel *ct = to_ct_panel(panel);
	int ret;
	
	printk("%s %d\n",__func__,__LINE__);
	
	if (ct->prepared)
		return 0;

	ret = regulator_bulk_enable(ct->num_supplies, ct->supplies);
	if (ret)
		return ret;
	printk("............%s \n",__func__);
	
	if (ct->pwr != NULL)
	gpiod_set_value(ct->pwr, 1);/* --电源 */
	msleep(20);	
	
	if (ct->stby != NULL)
	gpiod_set_value(ct->stby, 1);  /* -- stby */
	udelay(20);	
	
	if (ct->blken != NULL)
	gpiod_set_value(ct->blken, 1);  /* -- blk */
	udelay(20);

	if (ct->reset != NULL)
	gpiod_set_value(ct->reset, 0); /*--reset low*/
	msleep(20);	
	gpiod_set_value(ct->reset, 1); /*--reset high*/

	ct->prepared = true;

	return 0;
}

static int ct_panel_unprepare(struct drm_panel *panel)
{
	struct ct_panel *ct = to_ct_panel(panel);
	struct device *dev = &ct->dsi->dev;
	int ret;
	
	printk("%s %d\n",__func__,__LINE__);
	if (!ct->prepared)
		return 0;

	if (ct->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	/*关闭电源 */
	if (ct->pwr != NULL)
		gpiod_set_value(ct->pwr, 0);

	ret = regulator_bulk_disable(ct->num_supplies, ct->supplies);
	if (ret)
		return ret;

	ct->prepared = false;

	return 0;
}





static int ct_panel_enable(struct drm_panel *panel)
{
	struct ct_panel *ct = to_ct_panel(panel);
	struct mipi_dsi_device *dsi = ct->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;

	if (ct->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = ct_panel_push_cmd_list(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	}
	

	
	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to do Software Reset (%d)\n", ret);
	//	goto fail;
	}
	
	usleep_range(15000, 17000);
	
	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set tear ON (%d)\n", ret);
		goto fail;
	}
	/* Set tear scanline */
	ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0x380);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set tear scanline (%d)\n", ret);
		goto fail;
	}
	
	/* Set pixel format */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
	DRM_DEV_DEBUG_DRIVER(dev, "Interface color format set to 0x%x\n",
			     color_format);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}
	
	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	usleep_range(5000, 7000);
	
	
	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	backlight_enable(ct->backlight);
	
	ct->enabled = true;

	return 0;

fail:
	if (ct->pwr != NULL)
		gpiod_set_value(ct->pwr, 0);

	return ret;
}

static int ct_panel_disable(struct drm_panel *panel)
{
	struct ct_panel *ct = to_ct_panel(panel);
	struct mipi_dsi_device *dsi = ct->dsi;
	struct device *dev = &dsi->dev;
	int ret;
	
	if (!ct->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	backlight_disable(ct->backlight);
	usleep_range(10000, 12000);
	
	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}
	usleep_range(5000, 10000);
	
	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}
	
	ct->enabled = false;

	return 0;
}

static int ct_panel_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_DEV_ERROR(panel->dev, "failed to add mode %ux%ux@%u\n",
			      default_mode.hdisplay, default_mode.vdisplay,
			      default_mode.vrefresh);
		return -ENOMEM;
	}
	
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = ct_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 ct_bus_formats,
					 ARRAY_SIZE(ct_bus_formats));
	return 1;
}

//TEST: cat /sys/class/backlight/rgb_backlight/actual_brightness
static int ct_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ct_panel *ct = mipi_dsi_get_drvdata(dsi);
	u16 brightness = 0;
	
	if (!ct->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM; 

	brightness = bl->props.brightness;

	return brightness & 0xff;
}


//TEST: echo 55 > /sys/class/backlight/rgb_backlight/brightness
static int ct_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ct_panel *ct = mipi_dsi_get_drvdata(dsi);

	unsigned long long duty =  ct->pargs.period; 
	unsigned int max = bl->props.max_brightness; 
	
	if (!ct->prepared) 
		return 0;

	duty *= bl->props.brightness;  
	do_div(duty, max);

	pwm_config(ct->bl, duty, ct->pargs.period);

	if (duty == 0)
		pwm_disable(ct->bl);
	else
		pwm_enable(ct->bl);


	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM; 

	return 0;
}

/* backlight */
static const struct backlight_ops ct_bl_ops = {
	.update_status = ct_bl_update_status, 
	.get_brightness = ct_bl_get_brightness, 
};

static const struct drm_panel_funcs ct_panel_funcs = {
	.prepare = ct_panel_prepare,
	.unprepare = ct_panel_unprepare,
	.enable = ct_panel_enable,
	.disable = ct_panel_disable,
	.get_modes = ct_panel_get_modes,
};

static const char * const ct_supply_names[] = {
	"v3p3",
	"v1p8",
};

static int ct_init_regulators(struct ct_panel *ct)
{
	struct device *dev = &ct->dsi->dev;
	int i;

	ct->num_supplies = ARRAY_SIZE(ct_supply_names);//3.3V和1.8V
	ct->supplies = devm_kcalloc(dev, ct->num_supplies,
				     sizeof(*ct->supplies), GFP_KERNEL);
	if (!ct->supplies)
		return -ENOMEM;

	for (i = 0; i < ct->num_supplies; i++)
		ct->supplies[i].supply = ct_supply_names[i];

	return devm_regulator_bulk_get(dev, ct->num_supplies, ct->supplies);
};

static int ct_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct ct_panel *panel;
	struct backlight_properties bl_props; 
	int ret;
	u32 video_mode;

	printk("%s %d \n",__func__,__LINE__);

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;
	
	mipi_dsi_set_drvdata(dsi, panel);
	
	panel->dsi = dsi;
	
	dsi->format = MIPI_DSI_FMT_RGB888;  
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO;
	
	ret = of_property_read_u32(np, "video-mode", &video_mode);
	
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;

		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}
	
	
	panel->bl = devm_of_pwm_get(dev, np, 0);
	
	if (IS_ERR(panel->bl)) {
		ret = PTR_ERR(panel->bl);
		printk("unable to request PWM for : %d\n",ret);
		return ret;
	}
	
	pwm_apply_args(panel->bl);
	pwm_get_args(panel->bl, &panel->pargs); 
	
	printk("panel->pargs.period =%u\n",panel->pargs.period);
	printk("panel->pargs.polarity =%u\n",panel->pargs.polarity);
	
	pwm_disable(panel->bl);
	

	/* pwr */
	panel->pwr = devm_gpiod_get(dev, "pwr", GPIOD_OUT_HIGH);	
	if (IS_ERR(panel->pwr)) {
		panel->pwr = NULL;
		printk("%d \n",__LINE__);
	}
	else {
		gpiod_direction_output(panel->pwr,1);
//		gpiod_set_value(panel->pwr, 0); 
		printk("panel->pwr =%d\n",desc_to_gpio(panel->pwr));
	}
	
	/* blken */
	panel->blken = devm_gpiod_get(dev, "blk", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->blken)) {
		panel->blken = NULL;
		printk("%d \n",__LINE__);
	}	
	else {
		gpiod_direction_output(panel->blken,1);
//		gpiod_set_value(panel->blken, 0); 
		printk("panel->blken =%d\n",desc_to_gpio(panel->blken));

	}
	
	/* reset */
	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(panel->reset)) {
		panel->reset = NULL;
		printk("%d \n",__LINE__);
	}
	else {
		gpiod_direction_output(panel->reset,1);
//		gpiod_set_value(panel->reset, 0); 
		printk("panel->reset =%d\n",desc_to_gpio(panel->reset));
	}

	/* stby */
	panel->stby = devm_gpiod_get(dev, "stby", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->stby)) {
		panel->stby = NULL;
		printk("%d \n",__LINE__);
	}
	else {
		gpiod_direction_output(panel->stby,1);
		printk("panel->stby =%d\n",desc_to_gpio(panel->stby));

	}


	memset(&bl_props, 0, sizeof(bl_props)); 
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;
	
	panel->backlight = devm_backlight_device_register(
				dev, "rgb_backlight",
				dev, dsi,
				&ct_bl_ops, &bl_props);
				
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}
	
	ret = ct_init_regulators(panel);//初始化调压器
	if (ret)
		return ret;
	
	drm_panel_init(&panel->panel); 
	
	
	panel->panel.funcs = &ct_panel_funcs;
	panel->panel.dev = dev;
	dev_set_drvdata(dev, panel); 

	ret = drm_panel_add(&panel->panel);
	if (ret < 0)
		return ret;
	
	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&panel->panel);


	return ret;
}

static int ct_panel_remove(struct mipi_dsi_device *dsi)
{
	struct ct_panel *ct = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			      ret);

	drm_panel_remove(&ct->panel);

	return 0;
}

static void ct_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct ct_panel *ct = mipi_dsi_get_drvdata(dsi);
	ct_panel_disable(&ct->panel);
	ct_panel_unprepare(&ct->panel);
}


static const struct of_device_id ct_of_match[] = {
	{ .compatible = "ct,icn6201", },
	{ }
};

MODULE_DEVICE_TABLE(of, ct_of_match);

static struct mipi_dsi_driver ct_panel_driver = {
	.driver = {
		.name = "panel-ct-icn6201",
		.of_match_table = ct_of_match,
	},
	.probe = ct_panel_probe,
	.remove = ct_panel_remove,
	.shutdown = ct_panel_shutdown,
};
module_mipi_dsi_driver(ct_panel_driver);

MODULE_AUTHOR("Robert Chiras <992770449@qq.com>");
MODULE_DESCRIPTION("DRM Driver for ICN6211 MIPI DSI TO RGB");
MODULE_LICENSE("GPL v2");
