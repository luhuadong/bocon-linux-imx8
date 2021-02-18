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



static const u32 tc_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 tc_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_NEGEDGE;

struct cmd_list {
	u16 reg;
	u32 val;
};


struct tc_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *pwr; /*电源*/
	struct gpio_desc *reset;  /*复位*/
	struct gpio_desc *blken;  /*背光使能*/
	struct gpio_desc *stby;  /*背光使能*/
	struct backlight_device *backlight;
	struct pwm_device	*bl;//背光
	struct pwm_args pargs; 

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
	u32 init_len; 
	u32 *init_reg;
};





//时序参数 内核源码有注解意义
static  struct drm_display_mode default_mode = {
	.clock = 160000, //RGB频率
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



static inline struct tc_panel *to_tc_panel(struct drm_panel *panel)
{
	return container_of(panel, struct tc_panel, panel);
}


//写寄存器
size_t tc358775_dsi_write_reg(struct mipi_dsi_device *dsi,u16 addr,u32 val)
{
	ssize_t ret;
	u8 buf[6];
	buf[0] = addr & 0xff;
	buf[1] = (addr>>8) & 0xff;
	buf[2] = val &  0xff;
	buf[3] = (val>>8) &  0xff;
	buf[4] = (val>>16) & 0xff;
	buf[5] = (val>>24) & 0xff;

	
	ret =   mipi_dsi_generic_write(dsi,buf,6);
	printk("write ret:%zd addr:0x%04x val:0x%08x\n",ret,addr,val);
	return ret;
}


size_t tc358775_dsi_read_reg(struct mipi_dsi_device *dsi,u16 addr,u32 *val)     
{
        ssize_t ret;
        *val = 0;
        ret =   mipi_dsi_generic_read(dsi,&addr,2,val,4);
        printk("read ret:%zd addr:0x%04x val:0x%08x\n",ret,addr,*val);
        return ret;
}






static int tc_panel_push_cmd_list(struct mipi_dsi_device *dsi)
{
	
	int ret = 0;
	struct tc_panel *tc = (struct tc_panel *)mipi_dsi_get_drvdata(dsi);
	u32 *reg_init = tc->init_reg;
	u32 reg_len   = tc->init_len;
	
	if(reg_len > 0 && reg_init) {
		int i;
		for(i = 0;i < reg_len; i+=2) {
			if(reg_init[i]!=0xffff)	
				tc358775_dsi_write_reg(dsi,reg_init[i],reg_init[i+1]);
			else
				udelay(reg_init[i+1]);
		}	
		
	}
	

	ret = 2;
	return ret;
};

#if 0
extern int  tc358_read_reg(u16 reg,u32 *value);
extern int tc358_write_reg(u16 reg, u32 value); //i2c写寄存器

static int tc_panel_i2c_cmd_list(struct mipi_dsi_device *dsi)
{
	
	int ret = 0;
	struct tc_panel *tc = (struct tc_panel *)mipi_dsi_get_drvdata(dsi);
	u32 *reg_init = tc->init_reg;
	u32 reg_len   = tc->init_len;
	
	if(reg_len > 0 && reg_init) {
		int i;
		for(i = 0;i < reg_len; i+=2) {
			if(reg_init[i]!=0xffff)	
				tc358_write_reg(reg_init[i],reg_init[i+1]);
			else
				udelay(reg_init[i+1]);
		}	
		
	}
	

	ret = 2;
	return ret;
};
#endif

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



/*这个之后 到 enable , drm_panel.c   panel->funcs->prepare*/
/* drm_panel_prepare->drm_panel_prepare */
static int tc_panel_prepare(struct drm_panel *panel)
{

	struct tc_panel *tc = to_tc_panel(panel);
	int ret;
	struct mipi_dsi_device *dsi = tc->dsi;
	printk("%s %d\n",__func__,__LINE__);
	
	if (tc->prepared)
		return 0;


	if (tc->pwr != NULL) {
		gpiod_set_value(tc->pwr, 1);/* --电源 */
		udelay(5);
	}
		
	//添加时序到有时钟之后
	if (tc->stby != NULL) {
		gpiod_set_value(tc->stby, 1);  /* -- stby */
		usleep_range(5000, 10000);
	}


	if (tc->reset != NULL) {
		gpiod_set_value(tc->reset, 1); /*--reset high*/	
		usleep_range(5000, 10000);	
	}
		

	backlight_enable(tc->backlight);
	pwm_config(tc->bl, 1000000, 1000000);
	pwm_enable(tc->bl);
	usleep_range(5000, 10000);
	

	
	if (tc->blken != NULL)
	gpiod_set_value(tc->blken, 1);  /* -- blk */
	usleep_range(5000, 10000);



	tc->prepared = true;

	return 0;

}

static int tc_panel_unprepare(struct drm_panel *panel)
{
	struct tc_panel *tc = to_tc_panel(panel);
	struct device *dev = &tc->dsi->dev;
	printk("%s %d\n",__func__,__LINE__);
	if (!tc->prepared)
		return 0;

	if (tc->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	/*关闭电源 */
	if (tc->pwr != NULL)
		gpiod_set_value(tc->pwr, 0);

	tc->prepared = false;

	return 0;
}




static int tc_panel_enable(struct drm_panel *panel)
{
	struct tc_panel *tc = to_tc_panel(panel);
	struct mipi_dsi_device *dsi = tc->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;
//	u32 val;
	printk("%s ...\n",__func__);

	if (tc->enabled)
		return 0;

	if (!tc->prepared) {
		DRM_DEV_ERROR(dev, "Panel not prepared!\n");
		return -EPERM;
	}


	

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

 	ret = tc_panel_push_cmd_list(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	} 

//	tc358_read_reg(0x13c,&val);
//	printk("%s read reg:0x13c val =%x\n",__func__,val);
//	tc_panel_i2c_cmd_list(dsi);
	
	
	/* Set DSI mode */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xC2, 0x0B }, 2);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set DSI mode (%d)\n", ret);
		goto fail;
	}	
	
	
	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
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
	backlight_enable(tc->backlight);
	msleep(5);
	tc->enabled = true;

	return 0;

fail:
	if (tc->pwr != NULL)
		gpiod_set_value(tc->pwr, 0);

	return ret;
}

static int tc_panel_disable(struct drm_panel *panel)
{
	struct tc_panel *tc = to_tc_panel(panel);
	struct mipi_dsi_device *dsi = tc->dsi;

	if (!tc->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	backlight_disable(tc->backlight);

	tc->enabled = false;

	return 0;
}

static int tc_panel_get_modes(struct drm_panel *panel)
{

	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	printk("%s %d default_mode.clock=%d khz\n",__func__,__LINE__,default_mode.clock);

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
	connector->display_info.bus_flags = tc_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 tc_bus_formats,
					 ARRAY_SIZE(tc_bus_formats));
	return 1;
}

//TEST: cat /sys/class/backlight/rgb_backlight/actual_brightness
static int tc_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct tc_panel *tc = mipi_dsi_get_drvdata(dsi);
	u16 brightness = 0;
	
	if (!tc->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM; 

	brightness = bl->props.brightness;

	return brightness & 0xff;
}


//TEST: echo 55 > /sys/class/backlight/rgb_backlight/brightness
static int tc_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct tc_panel *tc = mipi_dsi_get_drvdata(dsi);

	unsigned long long duty =  tc->pargs.period; 
	unsigned int max = bl->props.max_brightness; 
	
	if (!tc->prepared) 
		return 0;

	duty *= bl->props.brightness;  
	do_div(duty, max);

	printk("bl->props.brightness =%d \n",bl->props.brightness);
	if (duty == 0)
		pwm_disable(tc->bl);
	else
		pwm_enable(tc->bl);


	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM; 

	return 0;
}


/* backlight */
static const struct backlight_ops tc_bl_ops = {
	.update_status = tc_bl_update_status, 
	.get_brightness = tc_bl_get_brightness, 
};

static const struct drm_panel_funcs tc_panel_funcs = {
	.prepare = tc_panel_prepare,
	.unprepare = tc_panel_unprepare,
	.enable = tc_panel_enable,
	.disable = tc_panel_disable,
	.get_modes = tc_panel_get_modes,
};

static const char * const ct_supply_names[] = {
	"v3p3",
	"v1p8",
};



static int tc_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct tc_panel *panel;
	struct backlight_properties bl_props; 
	int ret ,cmd_len = 0;
	u32 video_mode;
	u32 clock;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;
	
	mipi_dsi_set_drvdata(dsi, panel);
	
	panel->dsi = dsi;
	
	dsi->format = MIPI_DSI_FMT_RGB888; 
//	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO; //支持模式
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
		| MIPI_DSI_MODE_VIDEO_AUTO_VERT | MIPI_DSI_MODE_LPM |MIPI_DSI_MODE_VIDEO_HSE;
	
	
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
	
	
	ret = of_property_read_u32(np, "clock", &clock);
	if (ret < 0) {
		dev_err(dev, "Failed to get clock property (%d) use %d khz\n", ret,default_mode.clock);
	}else {
		default_mode.clock = clock;
	}
	
	
	printk("%s  \n",__func__);
	
	cmd_len = of_property_count_u32_elems(np, "reg_init"); //获取数组长度
	
	if(cmd_len == 0) {
		panel->init_reg = NULL;
		panel->init_len = 0;
		printk("init reg_init is not find \n");
	}else {
		panel->init_reg = devm_kzalloc(&dsi->dev, cmd_len * sizeof(u32), GFP_KERNEL);
		if (!panel->init_reg)
			return -ENOMEM;
		
		ret = of_property_read_u32_array(np, "reg_init", panel->init_reg, cmd_len);
		
		if(ret < 0) {
			dev_err(dev, "Failed to get reg_init property (%d)\n", ret);
			return ret;	
		}

		panel->init_len = cmd_len;	
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
	
	/*gdev->panel 对于gpio1 都是0 ,对于gpio2 都是32 ,对于gpio3 都是64 */
	/*gdev->ngpio 对于gpio1 ,gpio2 ,gpio3 都是32 */
	
	#if 0
	/* pwr */
	panel->pwr = devm_gpiod_get(dev, "pwr", GPIOD_OUT_HIGH);	
	if (IS_ERR(panel->pwr)) {
		panel->pwr = NULL;
		printk("%d \n",__LINE__);
	}
	else {
		gpiod_direction_output(panel->pwr,0);
		printk("panel->pwr =%d\n",desc_to_gpio(panel->pwr));
	}
	
	/* blken */
	panel->blken = devm_gpiod_get(dev, "blk", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->blken)) {
		panel->blken = NULL;
		printk("%d \n",__LINE__);
	}	
	else {
		gpiod_direction_output(panel->blken,0);
		printk("panel->blken =%d\n",desc_to_gpio(panel->blken));

	}
	
	/* reset */
	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(panel->reset)) {
		panel->reset = NULL;
		printk("%d \n",__LINE__);
	}
	else {
	    gpiod_direction_output(panel->reset,0);
		printk("panel->reset =%d\n",desc_to_gpio(panel->reset));
	}

	/* stby */
	panel->stby = devm_gpiod_get(dev, "stby", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->stby)) {
		panel->stby = NULL;
		printk("%d \n",__LINE__);
	}
	else {
		gpiod_direction_output(panel->stby,0);
		printk("panel->stby =%d\n",desc_to_gpio(panel->stby));
	}
	#else 	//更改断电

	/* stby */
	panel->stby = devm_gpiod_get(dev, "stby", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->stby)) {
		panel->stby = NULL;
	}	
	
	
	/* reset */
	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->reset)) {
		panel->reset = NULL;
	}


	/* blken */
	panel->blken = devm_gpiod_get(dev, "blk", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->blken)) {
		panel->blken = NULL;
		printk("%d \n",__LINE__);
	}	

	
	/* pwr */
	panel->pwr = devm_gpiod_get(dev, "pwr", GPIOD_OUT_HIGH);	
	if (IS_ERR(panel->pwr)) {
		panel->pwr = NULL;
		printk("%d \n",__LINE__);
	}

	#endif 

	memset(&bl_props, 0, sizeof(bl_props)); 
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;
	
	/*注册背光*/
	panel->backlight = devm_backlight_device_register(
				dev, "rgb_backlight",
				dev, dsi,
				&tc_bl_ops, &bl_props);
				
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}
	
	drm_panel_init(&panel->panel); 
	
	panel->panel.funcs = &tc_panel_funcs;/* 操作集 */
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

static int tc_panel_remove(struct mipi_dsi_device *dsi)
{
	
	return 0;
}

static void tc_panel_shutdown(struct mipi_dsi_device *dsi)
{
	
}


static const struct of_device_id tc_of_match[] = {
	{ .compatible = "tc,tc358775", },
	{ }
};

MODULE_DEVICE_TABLE(of, tc_of_match);

static struct mipi_dsi_driver tc_panel_driver = {
	.driver = {
		.name = "panel-tc-tc358775",
		.of_match_table = tc_of_match,
	},
	.probe = tc_panel_probe,
	.remove = tc_panel_remove,
	.shutdown = tc_panel_shutdown,
};
module_mipi_dsi_driver(tc_panel_driver);

MODULE_AUTHOR("Robert Chiras <992770449@qq.com>");
MODULE_DESCRIPTION("DRM Driver for TC358775 MIPI DSI TO LVDS");
MODULE_LICENSE("GPL v2");
