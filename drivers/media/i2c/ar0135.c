// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the AR0135 Image Sensor Processor
 *
 * Copyright (C) 2018 eyeSight Technologies Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define CIPIA_CID_CUSTOM_BASE  			(V4L2_CID_USER_BASE | 0xf000)
#define CIPIA_CID_AE_ROI       			(CIPIA_CID_CUSTOM_BASE + 0)
#define CIPIA_CID_LUMA_TARGET  			(CIPIA_CID_CUSTOM_BASE + 1)
#define CIPIA_CID_FLASH					(CIPIA_CID_CUSTOM_BASE + 2)

#define AR0135_R3012_COARSE_INTEGRATION_TIME    0x3012
#define AR0135_R3040_READ_MODE                  0x3040
#define AR0135_FLASH_REG						0x3046
#define AR0135_DIGITAL_TEST						0x30B0
#define AR0135_R305E_ANALOG_GAIN                0x305E
#define AR0135_R3100_AECTRLREG                  0x3100
#define AR0135_R3100_LUMA_TARGET				0x3102
#define AR0135_R3140_AE_ROI_X_START_OFFSET      0x3140
#define AR0135_R3142_AE_ROI_Y_START_OFFSET      0x3142
#define AR0135_R3144_AE_ROI_X_SIZE              0x3144
#define AR0135_R3146_AE_ROI_Y_SIZE              0x3146

#define AR0135_EXPOSURE_DEFAULT			0x016
#define AR0135_FLASH_ENABLE				0x0100
#define MAX_EXPOSURE_TIME				0x0342

#define AR0135_I2C_ADDR      0x10
#define AR0135_ID_REG        0x3000
#define AR0135_ID_VAL        0x0554
#define AR0135_ANALOG_GAIN_BIT 0x3

struct ar0135_reg_value {
	u16 reg;
	u16 val;
};

struct AR0135 {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;

	struct gpio_desc *rst_gpio;
	struct mutex lock;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;

	unsigned hflip:1;
	unsigned vflip:1;
	unsigned ae:1;
	u16 global_gain;
	u16 exposure;
	u16 luma_target;
	u8 flash_enable;
	u32 coarse_integration_time;

	u16 ae_roi_x_start_offset;
	u16 ae_roi_y_start_offset;
	u16 ae_roi_x_size;
	u16 ae_roi_y_size;	

	bool streaming;
};

static inline struct AR0135 *to_ar0135(struct v4l2_subdev *sd)
{
	return container_of(sd, struct AR0135, sd);
}

#define SLEEP 0x0000

static const struct ar0135_reg_value ar0135at_recommended_setting[] = {
	/* PLL 27Mhz */
	{0x302A, 0x0004}, // VT_PIX_CLK_DIV
	{0x302C, 0x0002}, // VT_SYS_CLK_DIV
	{0x302E, 0x0002}, // PRE_PLL_CLK_DIV
	{0x3030, 0x0030}, // PLL_MULTIPLIER
	{0x30B0, 0x00A0}, // DIGITAL_TEST    
};

static const struct ar0135_reg_value AR0135at_1280x960_30fps[] = {
	{0x3002, 0x0000},
	{0x3004, 0x0000},
	{0x3006, 0x03C0},
	{0x3008, 0x04FF},
	{0x300A, 0x05B0}, 
	{0x300C, 0x0672},
	//{0x30B0, 0x04A0}, // DIGITAL_TEST
	{0x3012, 0x0020},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000}, // READ_MODE - HFLIP OFF , VFLIP 0FF
	{0x3028, 0x0010}, // ROW_SPEED
};

static const struct ar0135_reg_value AR0135at_embedded_data_stats[] = {
	// {0x3064, 0x1982}, // Embedded data enabled
	{0x3064, 0x1802}, // Embedded data disabled
};

static const struct ar0135_reg_value AR0135at_auto_exposure[] = {
	{AR0135_FLASH_REG, AR0135_FLASH_ENABLE}, // LED_FLASH_EN = 1 -->should sleep for 500ms?
	{0x311E, 0x0002}, // AE_MIN_EXPOSURE_REG
	{0x311C, MAX_EXPOSURE_TIME}, // AE_MAX_EXPOSURE_REG (in rows)
	{0x3108, 0x0010}, // AE_MIN_EV_STEP_REG
	{0x310A, 0x0008}, // AE_MAX_EV_STEP_REG
	{0x310C, 0x0200}, // AE_DAMP_OFFSET_REG
	{0x310E, 0x0200}, // AE_DAMP_GAIN_REG
	{0x3110, 0x0080}, // AE_DAMP_MAX_REG
//	{0x3166, MAX_EXPOSURE_TIME}, // AE_AG_EXPOSURE_HI
//	{0x3168, 0x01A3}, // AE_AG_EXPOSURE_LO
	{0x3040, 0x0000}, // READ_MODE READ_MODE - HFLIP OFF , VFLIP 0FF
    {0x3064, 0x1982}, // EMBEDDED_DATA_CTRL
	{0x306E, 0x9010}, // DATAPATH_SELECT
	{AR0135_R3100_LUMA_TARGET, 0x0550}, // AE_LUMA_TARGET  - change according dynamic ROI fix
	{0x3100, 0x0001}  // AG*4 static, only integration time is variable 
};

static const struct ar0135_reg_value AR0135at_start_stream[] = {
	// {0x3070, 0x0002}, /* generate color bar pattern */
	{0x301A, 0x10DC}, // Stream on
};

static const struct ar0135_reg_value AR0135at_stop_stream[] = {
	{0x301A, 0x10D8}, // Stream off
};

static const s64 AR0135_link_freq[] = {
	//800000000,
	 1600000000,
};

static u8 ar0144_fpd_link_i2c_read(struct i2c_client *ar0144_i2c_client, u8 id, u8 addr);

static int ar0135_write_reg(struct AR0135 *ar0135, u16 reg, u16 val)
{
	u8 regbuf[4];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val >> 8;
	regbuf[3] = val & 0xff;

	ret = i2c_master_send(ar0135->i2c_client, regbuf, 4);
	if (ret < 0)
		dev_err(&ar0135->i2c_client->dev,
			"%s: write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, val);

	return ret;
}

static int ar0135_read_reg(struct AR0135 *ar0135, u16 reg, u16 *val)
{
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	ret = i2c_master_send(ar0135->i2c_client, buf, 2);
	if (ret < 0) {
		dev_err(&ar0135->i2c_client->dev,
			"%s: write reg error %d: reg=0x%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ar0135->i2c_client, buf, 2);
	if (ret < 0) {
		dev_err(&ar0135->i2c_client->dev,
			"%s: read reg error %d: reg=0x%x\n",
			__func__, ret, reg);
		return ret;
	}
	*val = buf[0] << 8;
	*val |= (buf[1] & 0xff);

	return 0;
}

static int ar0135_set_register_array(struct AR0135 *ar0135,
				     const struct ar0135_reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		if (settings->reg == SLEEP)
		{
			msleep(settings->val);
		}
		else
		{
			ret = ar0135_write_reg(ar0135, settings->reg, settings->val);
			//printk(KERN_ALERT "-------->ar0135_set_register_array %d reg: 0x%x val 0x%x, ret: %d\n",i,settings->reg,settings->val, ret);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

static int ar0135_s_power(struct v4l2_subdev *sd, int on)
{
	struct AR0135 *ar0135 = to_ar0135(sd);
	u16 reg_val;
	int ret = 0;

	if (!on)
		return 0;

	mutex_lock(&ar0135->lock);

	ret = ar0135_read_reg(ar0135, AR0135_ID_REG, &reg_val);
	if (ret < 0)
		goto out;
	if (reg_val != AR0135_ID_VAL) {
		dev_err(&ar0135->i2c_client->dev,
			"wrong chip id (%u), expected %u\n", reg_val,
			AR0135_ID_VAL);
		ret = -ENODEV;
		goto out;
	}

	/* reset */
	ret = ar0135_write_reg(ar0135, 0x301A, 0x00D9);
	if (ret < 0)
		goto out;
	msleep(200);
#if 0
	ret = ar0135_set_register_array(
		AR0135, AR0135at_rev4_recommended_setting,
		ARRAY_SIZE(AR0135at_rev4_recommended_setting));
#endif
out:
	mutex_unlock(&ar0135->lock);
	return ret;
}

static int ar0135_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_Y12_1X12;;

	return 0;
}

static int ar0135_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_Y12_1X12)
		return -EINVAL;

	if (fse->index >= 1)
		return -EINVAL;

	fse->min_width = 1280;
	fse->max_width = 1280;
	fse->min_height = 960;
	fse->max_height = 960;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ar0135_get_pad_format(struct AR0135 *ar0135,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0135->fmt;
	default:
		return NULL;
	}
}

static int ar0135_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct AR0135 *ar0135 = to_ar0135(sd);

	mutex_lock(&ar0135->lock);
	format->format = *__ar0135_get_pad_format(ar0135, cfg, format->pad,
						  format->which);
	mutex_unlock(&ar0135->lock);

	return 0;
}

static struct v4l2_rect *
__ar0135_get_pad_crop(struct AR0135 *ar0135, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0135->crop;
	default:
		return NULL;
	}
}

static int ar0135_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct AR0135 *ar0135 = to_ar0135(sd);
	struct v4l2_mbus_framefmt *__format;

	mutex_lock(&ar0135->lock);

	__format = __ar0135_get_pad_format(ar0135, cfg, format->pad,
			format->which);
	__format->width = 1280;
	__format->height = 960;
	__format->code = MEDIA_BUS_FMT_Y12_1X12;;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	mutex_unlock(&ar0135->lock);

	return 0;
}

static int ar0135_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;

	ar0135_set_format(subdev, cfg, &fmt);

	return 0;
}

static int ar0135_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	struct AR0135 *ar0135 = to_ar0135(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ar0135_get_pad_crop(ar0135, cfg, sel->pad,
					sel->which);

	return 0;
}

static int set_read_mode(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
    u16 mode = 0x0000;
	int ret = 0 ;

    if (core->hflip)
	{
        mode |= 0x4000;
	}

    if (core->vflip)
	{
        mode |= 0x8000;
	}

	//printk(KERN_ALERT "AR0135: set_read_mode read_mode=0x%x",mode);
    ret = ar0135_write_reg(core, AR0135_R3040_READ_MODE, mode);
	if (ret < 0)
	{
		printk(KERN_ALERT "set_read_mode error ret val: %d\n", ret);
	}

	return ret;
}

static int set_ae(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
    u16 val = 0x0000;
	int ret = 0;
    if (core->ae)
        val = 0x0003;

	//printk(KERN_ALERT "AR0135-------->set_ae to  0x%x\n",core->ae);
    ret = ar0135_write_reg(core, AR0135_R3100_AECTRLREG, val);
	if (ret < 0)
	{
		printk(KERN_ALERT "set_ae error ret val: %d\n", ret);
	}

	ar0135_read_reg(core, AR0135_R3100_AECTRLREG, &val);
	//printk(KERN_ALERT "-------->set_ae read val: 0x%x\n", val);
	return ret;
}

static int set_gain(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
	int ret = 0 ;
	u16 mask = 0xFFE7;
	u16 old_val = 0;
	u16 new_val = 0;
	u16 shifted_val = 0;
	printk(KERN_ALERT "AR0135-------->set_gain-------------\n");
	shifted_val = (core->global_gain << AR0135_ANALOG_GAIN_BIT);
	ar0135_read_reg(core, AR0135_DIGITAL_TEST, &old_val);
	printk(KERN_ALERT "AR0135-------->set_gain old value 0x%x\n",old_val);
	new_val = (old_val&mask)|shifted_val;
	printk(KERN_ALERT "AR0135-------->set_gain global_gain: 0x%x shifted 0x%x write: 0x%x\n",core->global_gain, shifted_val, new_val);
    ret = ar0135_write_reg(core, AR0135_DIGITAL_TEST, new_val);
	if (ret < 0)
	{
		printk(KERN_ALERT "set_gain error ret val: %d\n", ret);
	}

	ar0135_read_reg(core, AR0135_DIGITAL_TEST, &old_val);
	printk(KERN_ALERT "-------->set_gain read val: 0x%x\n", old_val);
	printk(KERN_ALERT "AR0135---------------------\n");
	return ret;
}

static int set_exposure(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
	int ret = 0;
	u16 val;

	printk(KERN_ALERT "AR0135-------->set_exposure to  0x%x\n",core->exposure);
    ret = ar0135_write_reg(core, AR0135_R3012_COARSE_INTEGRATION_TIME, core->exposure);
	if (ret < 0)
	{
		printk(KERN_ALERT "set_exposure error ret val: %d\n", ret);
	}

	ar0135_read_reg(core, AR0135_R3012_COARSE_INTEGRATION_TIME, &val);
	printk(KERN_ALERT "-------->set_exposure read val: 0x%x\n", val);
	return ret;
}

static int set_luma_target(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
	int ret = 0;
	//u16 val;

	//printk(KERN_ALERT "AR0135-------->set_luma_target set luma target to  0x%x\n",core->luma_target);
    ret = ar0135_write_reg(core, AR0135_R3100_LUMA_TARGET, core->luma_target);
	if (ret < 0)
	{
		printk(KERN_ALERT "set_luma_target error ret val: %d\n", ret);
	}

	//ar0135_read_reg(core, AR0135_R3012_COARSE_INTEGRATION_TIME, &val);
	//printk(KERN_ALERT "-------->set_luma_target read val: 0x%x\n", val);
	return ret;
}

static int set_flash(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
	u16 val = 0x0000;
	int ret = 0;
	ar0135_read_reg(core, AR0135_FLASH_REG, &val);
	//printk(KERN_ALERT "-------->set_flash read-------------------");
	//printk(KERN_ALERT "-------->set_flash read val: 0x%x\n", val);
	if (core->flash_enable)
	{
		val |= AR0135_FLASH_ENABLE;
	}
	else
	{
		val &= ~AR0135_FLASH_ENABLE;
	}

	//printk(KERN_ALERT "-------->set_flash flash enable: 0x%x\n", core->flash_enable);
	//printk(KERN_ALERT "-------->set_flash write val: 0x%x\n", val);
    ret = ar0135_write_reg(core, AR0135_FLASH_REG, val);
	if (ret < 0)
	{
		printk(KERN_ALERT "set_flash error ret val: %d\n", ret);
	}

	//ar0135_read_reg(core, AR0135_FLASH_REG, &val);
	//printk(KERN_ALERT "-------->set_flash read val: 0x%x\n", val);
	//printk(KERN_ALERT "------------------------------------------------");
	return ret;
}

 static int set_ae_roi(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
	int ret = 0;
	//u16 addr = 0, value = 0;

	//printk(KERN_ALERT "AR0135-------->set_ae_roi calledx= 0x%x(0x%x) y=0x%x(0x%x)\n",core->ae_roi_x_start_offset,core->ae_roi_x_size, core->ae_roi_y_start_offset,core->ae_roi_y_size);
    ret = ar0135_write_reg(core, AR0135_R3140_AE_ROI_X_START_OFFSET , core->ae_roi_x_start_offset);
	if (ret < 0)
	{
		printk(KERN_ALERT "-------->set_ae_roi cannot set  AR0135_R3140_AE_ROI_X_START_OFFSET ret val: %d\n", ret);
		return ret;
	}
	ret = ar0135_write_reg(core, AR0135_R3142_AE_ROI_Y_START_OFFSET, core->ae_roi_y_start_offset);
	if (ret < 0)
	{
		printk(KERN_ALERT "-------->set_ae_roi cannot set  AR0135_R3142_AE_ROI_Y_START_OFFSET ret val: %d\n", ret);
		return ret;
	}

    ret = ar0135_write_reg(core, AR0135_R3144_AE_ROI_X_SIZE, core->ae_roi_x_size);
	if (ret < 0)
	{
		printk(KERN_ALERT "-------->set_ae_roi cannot set  AR0135_R3144_AE_ROI_X_SIZE ret val: %d\n", ret);
		return ret;
	}

	ret = ar0135_write_reg(core, AR0135_R3146_AE_ROI_Y_SIZE, core->ae_roi_y_size);
	if (ret < 0)
	{
		printk(KERN_ALERT "-------->set_ae_roi cannot set  AR0135_R3146_AE_ROI_Y_SIZE ret val: %d\n", ret);
		return ret;
	}
#if 0
	addr = 0x3102;
	value = 0;
	ret = ar0135_read_reg(core, addr, &value);
	printk(KERN_ALERT "0x%x value 0x%x\n", addr, value);

	addr = 0x3152;
	value = 0;
	ret = ar0135_read_reg(core, addr, &value);
	printk(KERN_ALERT "0x%x value 0x%x\n", addr, value);

	addr = 0x3100;
	value = 0;
	ret = ar0135_read_reg(core, addr, &value);
	printk(KERN_ALERT "0x%x value 0x%x\n", addr, value);

	addr = 0x30B0;
	value = 0;
	ret = ar0135_read_reg(core, addr, &value);
	printk(KERN_ALERT "0x%x value 0x%x\n", addr, value);

	addr = 0x3164;	
	value = 0;
	ret = ar0135_read_reg(core, addr, &value);
	printk(KERN_ALERT "0x%x value 0x%x\n", addr, value);
#endif
	return 0;
}

static int ar0135_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct AR0135 *ar0135 = to_ar0135(subdev);
	int ret;

	mutex_lock(&ar0135->lock);

	if (enable == 0) {
		ret = ar0135_set_register_array(
			ar0135, AR0135at_stop_stream,
			ARRAY_SIZE(AR0135at_stop_stream));
		goto out;
	}

	/* Start stream */
	ret = ar0135_set_register_array(ar0135, ar0135at_recommended_setting,
					ARRAY_SIZE(ar0135at_recommended_setting));
	if (ret < 0)
		goto out;

	msleep(100);

	ret = ar0135_set_register_array(ar0135, AR0135at_1280x960_30fps,
					ARRAY_SIZE(AR0135at_1280x960_30fps));
	if (ret < 0)
		goto out;

	ret = ar0135_set_register_array(ar0135, AR0135at_embedded_data_stats,
					ARRAY_SIZE(AR0135at_embedded_data_stats));
	if (ret < 0)
		goto out;
#if 1 
    printk(KERN_ALERT "r0135_s_stream setting auto exposure\n");
	ret = ar0135_set_register_array(ar0135, AR0135at_auto_exposure,
					ARRAY_SIZE(AR0135at_auto_exposure));
#else
    printk(KERN_ALERT "ar0135_s_stream NO auto exposure - open LEDs\n");   
    ar0135_write_reg(ar0135, 0x3046, 0x0100);
#endif


	if (ret < 0)
		goto out;

	ret = ar0135_set_register_array(ar0135, AR0135at_start_stream,
					ARRAY_SIZE(AR0135at_start_stream));

out:
	mutex_unlock(&ar0135->lock);

	return ret;
}

static int ar0135_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct AR0135 *core =
	container_of(ctrl->handler, struct AR0135, ctrls);
	struct v4l2_subdev *sd = &core->sd;
	int ret = -EINVAL;

	//printk(KERN_ALERT "-------->ar0135_s_ctrl called with id: 0x%x\n", ctrl->id);
	switch (ctrl->id) {
    case V4L2_CID_EXPOSURE_AUTO:
        core->ae = (ctrl->val == V4L2_EXPOSURE_AUTO) ? 1 : 0;
        ret = set_ae(sd);
        break;
	case V4L2_CID_GAIN:
		core->global_gain = ctrl->val;
		ret = set_gain(sd);
		break;
	case V4L2_CID_EXPOSURE:
		core->exposure = ctrl->val;
		//printk(KERN_ALERT "AR0135::ar0135_s_ctrl V4L2_CID_EXPOSURE 0x%x ctrl->p_new.p_u32: 0x%x\n", ctrl->val,*(ctrl->p_new.p_u32));
		ret = set_exposure(sd);
		break;
	case V4L2_CID_HFLIP:
		core->hflip = ctrl->val;
		//printk(KERN_ALERT "AR0135::ar0135_s_ctrl V4L2_CID_HFLIP 0x%x\n", core->hflip);
		ret = set_read_mode(sd);
		break;
	case V4L2_CID_VFLIP:
		core->vflip = ctrl->val;
		//printk(KERN_ALERT "AR0135::ar0135_s_ctrl V4L2_CID_VFLIP 0x%x\n", core->hflip);
		ret = set_read_mode(sd);
		break;
    case CIPIA_CID_AE_ROI:
        core->ae_roi_x_start_offset = ctrl->p_new.p_u16[0];
        core->ae_roi_y_start_offset = ctrl->p_new.p_u16[1];
        core->ae_roi_x_size = ctrl->p_new.p_u16[2];
        core->ae_roi_y_size = ctrl->p_new.p_u16[3];
		//printk(KERN_ALERT "-------->ar0135_s_ctrl ROI: Top-left=(0x%x, 0x%x) Size=(0x%x, 0x%x)\n",ctrl->p_cur.p_u16[0],ctrl->p_cur.p_u16[1], ctrl->p_cur.p_u16[2],ctrl->p_cur.p_u16[3]);
        ret = set_ae_roi(sd);
		break;
	case CIPIA_CID_LUMA_TARGET:    
		core->luma_target=*(ctrl->p_new.p_u16);
		//printk(KERN_ALERT "-------->ar0135_s_ctrl LUMA:  0x%x\n",core->luma_target);
		ret = set_luma_target(sd);
		break;
	case CIPIA_CID_FLASH:
		core->flash_enable=*(ctrl->p_new.p_u8);
		//printk(KERN_ALERT "-------->ar0135_s_ctrl FLASH:  0x%x\n",core->flash_enable);
		ret = set_flash(sd);
		break;
	default:
		printk(KERN_ALERT "AR0135::ar0135_s_ctrl called with invalid ID (id=%x)\n", ctrl->id);
	}

 	return ret;
}

static const struct v4l2_ctrl_ops AR0135_ctrl_ops = {
	.s_ctrl = ar0135_s_ctrl,
};

static const struct v4l2_ctrl_config cipia_ae_roi = {
    .ops = &AR0135_ctrl_ops,
    .id = CIPIA_CID_AE_ROI,
    .name = "CIPIA AE ROI",
    .type = V4L2_CTRL_TYPE_U16,
    .def = 0x0,
    .min = 0x0,
    .max = 0x1280,
    .step = 1,
    .dims = { 4 },
};

static const struct v4l2_ctrl_config cipia_luma_target = {
    .ops = &AR0135_ctrl_ops,
    .id = CIPIA_CID_LUMA_TARGET,
    .name = "CIPIA LUMA TARGET",
    .type = V4L2_CTRL_TYPE_U16,
    .def = 0x0,
    .min = 0x0,
    .max = 0xFFFF,
    .step = 1,
    .dims = { 1 },
};

static const struct v4l2_ctrl_config cipia_flash = {
    .ops = &AR0135_ctrl_ops,
    .id = CIPIA_CID_FLASH,
    .name = "CIPIA FLASH SET",
    .type = V4L2_CTRL_TYPE_U8,
    .def = 0x0,
    .min = 0x0,
    .max = 0xFF,
    .step = 1,
    .dims = { 1 },
};

static const struct v4l2_subdev_core_ops AR0135_core_ops = {
	.s_power = ar0135_s_power,
};

static const struct v4l2_subdev_video_ops AR0135_video_ops = {
	.s_stream = ar0135_s_stream,
};

static const struct v4l2_subdev_pad_ops AR0135_subdev_pad_ops = {
	.init_cfg = ar0135_entity_init_cfg,
	.enum_mbus_code = ar0135_enum_mbus_code,
	.enum_frame_size = ar0135_enum_frame_size,
	.get_fmt = ar0135_get_format,
	.set_fmt = ar0135_set_format,
	.get_selection = ar0135_get_selection,
};

static const struct v4l2_subdev_ops AR0135_subdev_ops = {
	.core = &AR0135_core_ops,
	.video = &AR0135_video_ops,
	.pad = &AR0135_subdev_pad_ops,
};

struct AR0135_fdp_link_init_t {
	u8 id;
	u8 addr;
	u8 val;
}; 

#if 1
struct AR0135_fdp_link_init_t AR0135_fdp_link_init_arr[] = {
	{0x3d, 0x4C, 0x01}, // Enable access to RX port 0 registers
	{0x3d, 0x58, 0x58}, // Back-channel enabled
	{0x3d, 0x6D, 0x7e}, // Coax-cable; RAW12-HS mode
	{0x3d, 0x5C, 0xb4}, // Serializer alias I2C address
	{0x3d, 0x5D, 0x20}, // AR0135 I2c address
	{0x3d, 0x65, 0x20}, // AR0135 I2c alias
	{0x3d, 0x20, 0x20}, // Forwarding enabled for RX Port 0
	{0x3d, 0x33, 0x23}, // Enable CSI TX, 2 lanes, continouos clock mode enabled	
};
#else 
struct AR0135_fdp_link_init_t AR0135_fdp_link_init_arr[] = {
	{0x3d, 0x4C, 0x01}, // Enable access to RX port 0 registers
	{0x3d, 0x58, 0x58}, // Back-channel enabled
	{0x3d, 0x6D, 0x7e}, // Coax-cable; RAW12-HS mode
	{0x3d, 0x6E, 0x28}, // BC_GPIO
	{0x3d, 0x5C, 0xb4}, // Serializer alias I2C address
	{0x3d, 0x5D, 0x20}, // AR0135 I2c address
	{0x3d, 0x65, 0x20}, // AR0135 I2c alias
	{0x3d, 0x20, 0x20}, // Forwarding enabled for RX Port 0
	{0x3d, 0x33, 0x23}, // Enable CSI TX, 2 lanes, continouos clock mode enabled	
};
#endif

static u8 ar0144_fpd_link_i2c_read(struct i2c_client *ar0144_i2c_client, u8 id, u8 addr)
{
	struct i2c_msg msg[2];
	u8 reg_addr = addr;
	int ret = 0;
	u8 result;

	msg[0].addr = id;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	//read command
	msg[1].addr = id;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &result;

	ret = i2c_transfer(ar0144_i2c_client->adapter, msg, 2);
	if (ret > 0)
	{
		printk(KERN_ALERT "ar0135_fpd_link_i2c_read register 0x%x=0x%x\n", msg[1].addr, result);
	}
	else
	{
		printk(KERN_ALERT "ar0135_fpd_link_i2c_read error: %d\n", ret);
		return -1;
	}
	
	return result;
}

static int ar0135_fpd_link_i2c_write(struct i2c_adapter *i2c_master,
				     struct AR0135_fdp_link_init_t *reg_item)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	msg.addr = reg_item->id;
	msg.flags = 0;
	msg.len = 2;
	buf[0] = reg_item->addr;
	buf[1] = reg_item->val;
	msg.buf = buf;

	ret = i2c_transfer(i2c_master, &msg, 1);
	if (ret < 0)
		return ret;
	if (ret < 1)
		return -EIO;

	return 0;
}

/**
 * This is a local board specific hack to initialize the FPD Link
 * serializer/deserialzer bridge to the actual AR0135 sensor. Since we need to
 * talk to more than one device, this routine uses the i2c master directly.
 */
static int ar0135_fpd_link_init(struct i2c_client *ar0135_i2c_client)
{
	int i, ret;
	// struct AR0135_fdp_link_init_t read_val;

	for (i = 0; i < ARRAY_SIZE(AR0135_fdp_link_init_arr); i++) {
		ret = ar0135_fpd_link_i2c_write(ar0135_i2c_client->adapter,
						&AR0135_fdp_link_init_arr[i]);
		if (ret < 0)
		{
			printk(KERN_ALERT "ar0135_fpd_link_init adapter: ERROR WRITING: 0x%x addr 0x%x val 0x%x\n", AR0135_fdp_link_init_arr[i].id, AR0135_fdp_link_init_arr[i].addr, AR0135_fdp_link_init_arr[i].val);					 
			return ret;
		}
	}

	return 0;
}

static int ar0135_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct AR0135 *ar0135;
	int ret;
	u8 camera_id_res = 0;
	//u8 read_value = 0;

	printk(KERN_ALERT "AR0135_probe\n");

	camera_id_res = ar0144_fpd_link_i2c_read(client, 0x3d, 0x5b);
	printk(KERN_ALERT "camera id: 0x%x\n", camera_id_res);
	if (camera_id_res != 0xb4)
	{
		printk(KERN_ALERT "AR0135_probe did not find AR0135 exiting\n");
		return -EINVAL;
	}

	ret = ar0135_fpd_link_init(client);
	if (ret < 0)
	{
		printk(KERN_ALERT "ar0135_probe ar0135_fpd_link_init error\n");
		return ret;	
	}

	ar0135 = devm_kzalloc(dev, sizeof(struct AR0135), GFP_KERNEL);
	if (!ar0135)
		return -ENOMEM;

	ar0135->i2c_client = client;
	ar0135->dev = dev;
	mutex_init(&ar0135->lock);
	dev_info(dev, "AR0135 RPV camera detected version 1.14.1\n");

	// default values //
	ar0135->ae = 1;
	ar0135->ae_roi_x_start_offset = 0;
	ar0135->ae_roi_y_start_offset = 0;
	ar0135->ae_roi_x_size = 1280;
	ar0135->ae_roi_y_size = 960;		

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ar0135->ep);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	of_node_put(endpoint);

	if (ar0135->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be parallel\n");
		return -EINVAL;
	}

#if 0
	AR0135->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(AR0135->rst_gpio)) {
		if (PTR_ERR(AR0135->rst_gpio) != -EPROBE_DEFER)
			dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(AR0135->rst_gpio);
	}
#endif

	v4l2_ctrl_handler_init(&ar0135->ctrls, 1);
	ar0135->link_freq = v4l2_ctrl_new_int_menu(&ar0135->ctrls, NULL,
						   V4L2_CID_LINK_FREQ, 0, 0, AR0135_link_freq);

	if (ar0135->link_freq)
		ar0135->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (ar0135->ctrls.error) {
		dev_err(dev, "control initialization error\n");
		ret = ar0135->ctrls.error;
		goto free_ctrls;
	}

    v4l2_ctrl_new_std_menu(&ar0135->ctrls, &AR0135_ctrl_ops,
              V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	v4l2_ctrl_new_std(&ar0135->ctrls, &AR0135_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0x40, 1, 0x0E);
	v4l2_ctrl_new_std(&ar0135->ctrls, &AR0135_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, MAX_EXPOSURE_TIME	, 1, AR0135_EXPOSURE_DEFAULT);
	ar0135->exposure = AR0135_EXPOSURE_DEFAULT;
	v4l2_ctrl_new_std(&ar0135->ctrls, &AR0135_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ar0135->ctrls, &AR0135_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_custom(&ar0135->ctrls, &cipia_ae_roi, NULL);
	v4l2_ctrl_new_custom(&ar0135->ctrls, &cipia_luma_target, NULL);
	v4l2_ctrl_new_custom(&ar0135->ctrls, &cipia_flash, NULL);
	
	ar0135->sd.ctrl_handler = &ar0135->ctrls;

	v4l2_i2c_subdev_init(&ar0135->sd, client, &AR0135_subdev_ops);
	ar0135->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar0135->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ar0135->pad.flags = MEDIA_PAD_FL_SOURCE;
	ar0135->sd.dev = &client->dev;

	ret = media_entity_init(&ar0135->sd.entity, 1, &ar0135->pad, 0);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrls;
	}

	ret = ar0135_s_power(&ar0135->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up AR0135\n");
		goto free_entity;
	}

	dev_info(dev, "AR0135 RPV detected at address 0x%02x\n", client->addr);

	ret = v4l2_async_register_subdev(&ar0135->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ar0135_entity_init_cfg(&ar0135->sd, NULL);

	/*read_value = ar0144_fpd_link_i2c_read(client, 0x5A, 0x0);	
	printk(KERN_ALERT "----->ar0135_probe register read 0x5A00=0x%x\n", read_value);
	read_value = ar0144_fpd_link_i2c_read(client, 0x5A, 0x0A);	
	printk(KERN_ALERT "----->ar0135_probe register read 0x5A0A=0x%x\n", read_value);
	read_value = ar0144_fpd_link_i2c_read(client, 0x5A, 0x0B);	
	printk(KERN_ALERT "----->ar0135_probe register read 0x5A0B=0x%x\n", read_value);*/
	return 0;

free_entity:
	media_entity_cleanup(&ar0135->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&ar0135->ctrls);

	return ret;
}

static int ar0135_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ar0135_id[] = {
	{ "ar0135", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ar0135_id);

static const struct of_device_id ar0135_of_match[] = {
	{ .compatible = "onnn,ar0135" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0135_of_match);

static struct i2c_driver ar0135_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ar0135_of_match),
		.name  = "ar0135",
	},
	.probe  = ar0135_probe,
	.remove = ar0135_remove,
	.id_table = ar0135_id,
};

module_i2c_driver(ar0135_i2c_driver);

MODULE_DESCRIPTION("ON Semiconductor AR0135 Camera Sensor");
MODULE_LICENSE("GPL v2");
