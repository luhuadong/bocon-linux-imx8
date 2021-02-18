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
#include <drm/drm_dp_helper.h> //add 

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77

#define SN_DEVICE_REV_REG			0x08
#define SN_DPPLL_SRC_REG			0x0A
#define  DPPLL_CLK_SRC_DSICLK			BIT(0)
#define  REFCLK_FREQ_MASK			GENMASK(3, 1)
#define  REFCLK_FREQ(x)				((x) << 1)
#define  DPPLL_SRC_DP_PLL_LOCK			BIT(7)
#define SN_PLL_ENABLE_REG			0x0D
#define SN_DSI_LANES_REG			0x10
#define  CHA_DSI_LANES_MASK			GENMASK(4, 3)
#define  CHA_DSI_LANES(x)			((x) << 3)
#define SN_DSIA_CLK_FREQ_REG			0x12
#define SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG	0x20
#define SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG	0x24
#define SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG	0x2C
#define SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG	0x2D
#define  CHA_HSYNC_POLARITY			BIT(7)
#define SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG	0x30
#define SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG	0x31
#define  CHA_VSYNC_POLARITY			BIT(7)
#define SN_CHA_HORIZONTAL_BACK_PORCH_REG	0x34
#define SN_CHA_VERTICAL_BACK_PORCH_REG		0x36
#define SN_CHA_HORIZONTAL_FRONT_PORCH_REG	0x38
#define SN_CHA_VERTICAL_FRONT_PORCH_REG		0x3A
#define SN_ENH_FRAME_REG			0x5A
#define  VSTREAM_ENABLE				BIT(3)
#define SN_DATA_FORMAT_REG			0x5B
#define SN_HPD_DISABLE_REG			0x5C
#define  HPD_DISABLE				BIT(0)
#define SN_AUX_WDATA_REG(x)			(0x64 + (x))
#define SN_AUX_ADDR_19_16_REG			0x74
#define SN_AUX_ADDR_15_8_REG			0x75
#define SN_AUX_ADDR_7_0_REG			0x76
#define SN_AUX_LENGTH_REG			0x77
#define SN_AUX_CMD_REG				0x78
#define  AUX_CMD_SEND				BIT(0)
#define  AUX_CMD_REQ(x)				((x) << 4)
#define SN_AUX_RDATA_REG(x)			(0x79 + (x))
#define SN_SSC_CONFIG_REG			0x93
#define  DP_NUM_LANES_MASK			GENMASK(5, 4)
#define  DP_NUM_LANES(x)			((x) << 4)
#define SN_DATARATE_CONFIG_REG			0x94
#define  DP_DATARATE_MASK			GENMASK(7, 5)
#define  DP_DATARATE(x)				((x) << 5)
#define SN_ML_TX_MODE_REG			0x96
#define  ML_TX_MAIN_LINK_OFF			0
#define  ML_TX_NORMAL_MODE			BIT(0)
#define SN_AUX_CMD_STATUS_REG			0xF4
#define  AUX_IRQ_STATUS_AUX_RPLY_TOUT		BIT(3)
#define  AUX_IRQ_STATUS_AUX_SHORT		BIT(5)
#define  AUX_IRQ_STATUS_NAT_I2C_FAIL		BIT(6)

#define MIN_DSI_CLK_FREQ_MHZ	40

/* fudge factor required to account for 8b/10b encoding */
#define DP_CLK_FUDGE_NUM	10
#define DP_CLK_FUDGE_DEN	8

/* Matches DP_AUX_MAX_PAYLOAD_BYTES (for now) */
#define SN_AUX_MAX_PAYLOAD_BYTES	16

#define SN_REGULATOR_SUPPLY_NUM		4

#define DP_AUX_NATIVE_REPLY_MASK	(0x3 << 0)
#define DP_AUX_NATIVE_REPLY_ACK		(0x0 << 0)
#define DP_AUX_NATIVE_WRITE		0x8

/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};



static const u32 ti_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 ti_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_NEGEDGE;


struct ti_panel {
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

static inline struct ti_panel *to_ti_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ti_panel, panel);
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

#define mipi_dsi_read_poll_timeout(dsi, addr, val, cond, sleep_us, timeout_us) \
({ \
	u64 __timeout_us = (timeout_us); \
	unsigned long __sleep_us = (sleep_us); \
	ktime_t __timeout = ktime_add_us(ktime_get(), __timeout_us); \
	int __ret; \
	might_sleep_if(__sleep_us); \
	for (;;) { \
		__ret = mipi_dsi_read_reg((dsi), (addr), &(val)); \
		if (__ret) \
			break; \
		if (cond) \
			break; \
		if ((__timeout_us) && \
		    ktime_compare(ktime_get(), __timeout) > 0) { \
			__ret = mipi_dsi_read_reg((dsi), (addr), &(val)); \
			break; \
		} \
		if (__sleep_us) \
			usleep_range((__sleep_us >> 2) + 1, __sleep_us); \
	} \
	__ret ?: ((cond) ? 0 : -ETIMEDOUT); \
})


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

/*
写16位到连续地址的寄存器
*/
static int mipi_dsi_write_u16(struct mipi_dsi_device *dsi,uint8_t reg,uint16_t val)
{
	int rl,rh;
	rl = mipi_dsi_write_reg(dsi, reg, val & 0xFF);//写低8位
	rh = mipi_dsi_write_reg(dsi, reg + 1, val >> 8);//地址加1写高8位
	if(rl < 0 || rh < 0) {
		printk("rl =%d rh =%d \n",rl,rh);
		return -1;
	}
	return 0;
}

/*更新寄存器的某些位*/
static int mipi_dsi_reg_up_bit(struct mipi_dsi_device *dsi,uint8_t addr,uint8_t mask,
uint8_t val)
{
	int ret;
	uint8_t reg_val = 0x00;
	uint8_t buffer[2] = {0};
	ret = mipi_dsi_read_reg(dsi,addr,&reg_val);
	if(ret < 0) {
		printk("%s read addr :%02x fail\n",__func__,addr);
		return -1;
	}
	reg_val &=(~mask);
	reg_val |=val;
	buffer[0] = addr;
	buffer[1] = reg_val;
	ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer)); 
	if (ret < 0) {
		printk("%s write addr :%02x fail\n",__func__,addr);
		return -2;
	}
	return  0;
}



static ssize_t cur_sn_aux_transfer(struct mipi_dsi_device *dsi,struct drm_dp_aux_msg *msg)
{
	u32 request = msg->request & ~DP_AUX_I2C_MOT;
	u32 request_val = AUX_CMD_REQ(msg->request);
	u8 *buf = (u8 *)msg->buffer;
	uint8_t val;
	int ret, i;

	if (msg->size > SN_AUX_MAX_PAYLOAD_BYTES)
		return -EINVAL;

	switch (request) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		mipi_dsi_write_reg(dsi, SN_AUX_CMD_REG, request_val); //写入请求的命令
		break;
	default:
		return -EINVAL;
	}


	mipi_dsi_write_reg(dsi, SN_AUX_ADDR_19_16_REG, (msg->address >> 16) & 0xF);
	mipi_dsi_write_reg(dsi, SN_AUX_ADDR_15_8_REG, (msg->address >> 8) & 0xFF);
	mipi_dsi_write_reg(dsi, SN_AUX_ADDR_7_0_REG, msg->address & 0xFF);
	mipi_dsi_write_reg(dsi, SN_AUX_LENGTH_REG, msg->size);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE) {
		for (i = 0; i < msg->size; i++)
			mipi_dsi_write_reg(dsi, SN_AUX_WDATA_REG(i),
				     buf[i]);
	}

	mipi_dsi_write_reg(dsi, SN_AUX_CMD_REG, request_val | AUX_CMD_SEND);

	#if 0
	
	ret = mipi_dsi_read_poll_timeout(dsi, SN_AUX_CMD_REG, val,
				       !(val & AUX_CMD_SEND), 2000,
				       50 * 1000);
	if (ret) {
		printk("mipi_dsi_read_poll_timeout AUX_CMD is not OK ret= %d \n",ret);
		
	}else {
		printk("mipi_dsi_read_poll_timeout AUX_CMD is  OK\n");
	}
	#else
	msleep(50);
	ret = mipi_dsi_read_reg(dsi,SN_AUX_CMD_REG,&val); //读状态位是否被清除
	if(val & AUX_CMD_SEND) {
		printk("AUX_CMD is not OK\n");
	}else {
		printk("AUX_CMD is  OK\n");
	}
	#endif

	if (ret)
		return ret;

	ret = mipi_dsi_read_reg(dsi, SN_AUX_CMD_STATUS_REG, &val);
	if (ret)
		return ret;
	else if ((val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
		 || (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
		 || (val & AUX_IRQ_STATUS_AUX_SHORT))
		return -ENXIO;

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE)
		return msg->size;

	for (i = 0; i < msg->size; i++) {
		uint8_t val;
		ret = mipi_dsi_read_reg(dsi, SN_AUX_RDATA_REG(i),
				  &val);
		if (ret)
			return ret;

		WARN_ON(val & ~0xFF);
		buf[i] = (u8)(val & 0xFF);
	}

	return msg->size;
}

static inline ssize_t cur_dp_dpcd_writeb(struct mipi_dsi_device *dsi,unsigned int offset, u8 value)
{
	u8 request = DP_AUX_NATIVE_WRITE;
	struct drm_dp_aux_msg msg = {
		.address = offset,
		.request = request,
		.buffer	= &value,
		.size	= 1,
	};
	
	unsigned int retry, native_reply;
	int err = 0, ret = 0;
	size_t size = msg.size;
	
	for (retry = 0; retry < 32; retry++) { //重试32次
		if (ret != 0 && ret != -ETIMEDOUT) {
			usleep_range(500,500 + 100);
		}

		ret = cur_sn_aux_transfer(dsi,&msg);//传输消息

		if (ret >= 0) { //返回值大于等于0
			native_reply = msg.reply & DP_AUX_NATIVE_REPLY_MASK;
			if (native_reply == DP_AUX_NATIVE_REPLY_ACK) {
				if (ret == size)
					goto unlock;

				ret = -EPROTO;
			} else
				ret = -EIO;
		}

		if (!err)
			err = ret;//保存返回值到err继续尝试
	}

unlock:	
	return ret;
}





static int ti_panel_push_cmd_list(struct mipi_dsi_device *dsi)
{
	int ret = 0;
	uint8_t val;

	/*获取硬件检测到的时钟源 并设置时钟源频率 384 M [REG:0x0A] */
	ret = mipi_dsi_read_reg(dsi,SN_DPPLL_SRC_REG,&val);
	if (!(val & DPPLL_CLK_SRC_DSICLK)) {
		printk("use extern clk\n");
		val |= (0x3 <<1); //27M
	}else{
		printk("use DSI clk\n");
		val |= (0x2 <<1); //416M=2   486M=3 
	}
	
	mipi_dsi_write_reg(dsi, SN_DPPLL_SRC_REG, val); //更新[3:1]

	/* 关闭HDP [REG:0x5C] */
	mipi_dsi_reg_up_bit(dsi,SN_HPD_DISABLE_REG,HPD_DISABLE,HPD_DISABLE); 

	/*设置 DSI A通道 4lanes  , DP 2lanes */
	mipi_dsi_reg_up_bit(dsi,SN_DSI_LANES_REG,CHA_DSI_LANES_MASK,0x00); /*[REG:0x10]*/
	mipi_dsi_reg_up_bit(dsi,SN_SSC_CONFIG_REG,DP_NUM_LANES_MASK,0x20); /*2 DP lanes no SSC [REG:0x93]*/

	/* 设置DSI时钟范围 DP 时钟范围 */
	mipi_dsi_write_reg(dsi, SN_DSIA_CLK_FREQ_REG, 0x4c); /*[REG:0x12] 416=0x53  486=0x61*/				
	mipi_dsi_reg_up_bit(dsi,SN_DATARATE_CONFIG_REG,DP_DATARATE_MASK,0x80); /*每个lanes范围 [REG:0x94]*/

	/*使能 EDP 的PLL,并等待稳定 */
	mipi_dsi_write_reg(dsi, SN_PLL_ENABLE_REG, 1); /*[REG:0x0D]*/
	msleep(20);
	mipi_dsi_read_reg(dsi,SN_DPPLL_SRC_REG,&val);/*[REG:0x0A]*/
	if(val & DPPLL_SRC_DP_PLL_LOCK) {
		printk("DP PLL is OK\n");
	}else {
		printk("DP PLL is not OK\n");
	}


	//在 DisplayPort 地址0x0010A的eDP面板中启用 
	cur_dp_dpcd_writeb(dsi,0x10a,0x1);//0x10a 0x1


	/* 设置模式并等待设置完成 ,半自动 Semi auto link training mode */
	mipi_dsi_write_reg(dsi, SN_ML_TX_MODE_REG, 0x0A); /*[REG:0x96]*/
	msleep(20);
	mipi_dsi_read_reg(dsi,SN_ML_TX_MODE_REG,&val);
	if(val == ML_TX_MAIN_LINK_OFF ||val == ML_TX_NORMAL_MODE) {
		printk("ML_TX_MODE is OK\n");
	}else {
		printk("ML_TX_MODE is not OK\n");
	}


	/* 配置 视频 参数 */
	mipi_dsi_write_u16(dsi,SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG,1920); /*行长度 [REG:0x20]*/
	mipi_dsi_write_u16(dsi,SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG,1080); /*列长度 [REG:0x24]*/
	mipi_dsi_write_reg(dsi, SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG,80);//HSYNC 低8位 [REG:0x2C]
	mipi_dsi_write_reg(dsi, SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG,0x80);//HSYNC 高7位 bit7是HSYNC极性 =0是高电平默认 [REG:0x2D]
	mipi_dsi_write_reg(dsi, SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG,50);//VSYNC 低8位 [REG:0x30]
	mipi_dsi_write_reg(dsi, SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG,0x80);//VSYNC 高7位 bit7是VSYNC极性 =0是高电平默认 [REG:0x31]
	mipi_dsi_write_reg(dsi, SN_CHA_HORIZONTAL_BACK_PORCH_REG,60);//HBP [REG:0x34]
	mipi_dsi_write_reg(dsi, SN_CHA_VERTICAL_BACK_PORCH_REG,20);//VBP	[REG:0x36]
	mipi_dsi_write_reg(dsi, SN_CHA_HORIZONTAL_FRONT_PORCH_REG,20); //HFP	[REG:0x38]
	mipi_dsi_write_reg(dsi, SN_CHA_VERTICAL_FRONT_PORCH_REG,20);//VFP	[REG:0x3A]
	usleep_range(10000, 10500); /* 10ms delay recommended by spec */


//	mipi_dsi_write_reg(dsi,0x3C,0x10); //彩虹条
	/* 使能 视频 流出 */
	mipi_dsi_reg_up_bit(dsi, SN_ENH_FRAME_REG, VSTREAM_ENABLE,0x1<<3);	
	
	#if 0
	mipi_dsi_read_reg(dsi,0x10,&val);
	printk("addr 0x10 val =%02x\n",val);
	mipi_dsi_read_reg(dsi,SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG + 1,&val);
	printk("addr SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG + 1 val =%02x\n",val);
	mipi_dsi_read_reg(dsi,0x02,&val);
	printk("addr 0x02 val =%02x\n",val);
	mipi_dsi_read_reg(dsi,0x5a,&val);
	printk("addr 0x5a val =%02x\n",val);
	#endif
	
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

static int ti_panel_prepare(struct drm_panel *panel)
{
	struct ti_panel *ct = to_ti_panel(panel);
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
	msleep(10);	
	
	if (ct->stby != NULL)
	gpiod_set_value(ct->stby, 1);  /* -- stby */
	udelay(20);	
	
	if (ct->blken != NULL)
	gpiod_set_value(ct->blken, 1);  /* -- blk */
	udelay(20);

	if (ct->reset != NULL)
	gpiod_set_value(ct->reset, 0); /*--reset low*/
	msleep(10);	
	gpiod_set_value(ct->reset, 1); /*--reset high*/

	ct->prepared = true;

	return 0;
}

static int ti_panel_unprepare(struct drm_panel *panel)
{
	struct ti_panel *ct = to_ti_panel(panel);
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





static int ti_panel_enable(struct drm_panel *panel)
{
	struct ti_panel *ct = to_ti_panel(panel);
	struct mipi_dsi_device *dsi = ct->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;

	if (ct->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;


	ret = ti_panel_push_cmd_list(dsi);
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

static int ti_panel_disable(struct drm_panel *panel)
{
	struct ti_panel *ct = to_ti_panel(panel);
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

static int ti_panel_get_modes(struct drm_panel *panel)
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
	connector->display_info.bus_flags = ti_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 ti_bus_formats,
					 ARRAY_SIZE(ti_bus_formats));
					 
	printk("DSI output  Pixel clock =%d Khz\n",mode->clock);				 
					 
	return 1;
}

//TEST: cat /sys/class/backlight/rgb_backlight/actual_brightness
static int ct_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ti_panel *ct = mipi_dsi_get_drvdata(dsi);
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
	struct ti_panel *ct = mipi_dsi_get_drvdata(dsi);

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

static const struct drm_panel_funcs ti_panel_funcs = {
	.prepare = ti_panel_prepare,
	.unprepare = ti_panel_unprepare,
	.enable = ti_panel_enable,
	.disable = ti_panel_disable,
	.get_modes = ti_panel_get_modes,
};

static const char * const ct_supply_names[] = {
	"v3p3",
	"v1p8",
};

static int ti_init_regulators(struct ti_panel *ct)
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

static int ti_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct ti_panel *panel;
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
	}
	
	/* blken */
	panel->blken = devm_gpiod_get(dev, "blk", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->blken)) {
		panel->blken = NULL;
		printk("%d \n",__LINE__);
	}	
	else {
		gpiod_direction_output(panel->blken,1);
	}
	
	/* reset */
	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(panel->reset)) {
		panel->reset = NULL;
		printk("%d \n",__LINE__);
	}
	else {
		gpiod_direction_output(panel->reset,1);
	}

	/* stby */
	panel->stby = devm_gpiod_get(dev, "stby", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->stby)) {
		panel->stby = NULL;
		printk("%d \n",__LINE__);
	}
	else {
		gpiod_direction_output(panel->stby,1);
//		printk("panel->stby =%d\n",desc_to_gpio(panel->stby));
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
	
	ret = ti_init_regulators(panel);//初始化调压器
	if (ret)
		return ret;
	
	drm_panel_init(&panel->panel); 
	
	
	panel->panel.funcs = &ti_panel_funcs;
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

static int ti_panel_remove(struct mipi_dsi_device *dsi)
{
	struct ti_panel *ti = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			      ret);

	drm_panel_remove(&ti->panel);

	return 0;
}

static void ti_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct ti_panel *ti = mipi_dsi_get_drvdata(dsi);
	ti_panel_disable(&ti->panel);
	ti_panel_unprepare(&ti->panel);
}


static const struct of_device_id ti_of_match[] = {
	{ .compatible = "edp,sn65dsi86", },
	{ }
};

MODULE_DEVICE_TABLE(of, ti_of_match);

static struct mipi_dsi_driver ti_panel_driver = {
	.driver = {
		.name = "panel-ti-sn65dsi86",
		.of_match_table = ti_of_match,
	},
	.probe = ti_panel_probe,
	.remove = ti_panel_remove,
	.shutdown = ti_panel_shutdown,
};
module_mipi_dsi_driver(ti_panel_driver);

MODULE_AUTHOR("Robert Chiras <992770449@qq.com>");
MODULE_DESCRIPTION("DRM Driver for sn65dsi86 MIPI DSI TO EDP");
MODULE_LICENSE("GPL v2");
