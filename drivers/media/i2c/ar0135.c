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

#define AR0135_CID_CUSTOM_BASE  (V4L2_CID_USER_BASE | 0xf000)
#define AR0135_CID_AE_ROI       (AR0135_CID_CUSTOM_BASE + 0)

#define AR0135_R3012_COARSE_INTEGRATION_TIME    0x3012
#define AR0135_R301D_IMAGE_ORIENTATION          0x3040
#define AR0135_R3040_READ_MODE                  0x3040
#define AR0135_R3060_ANALOG_GAIN                0x305E
#define AR0135_R3100_AECTRLREG                  0x3100
#define AR0135_R3140_AE_ROI_X_START_OFFSET      0x3140
#define AR0135_R3142_AE_ROI_Y_START_OFFSET      0x3142
#define AR0135_R3144_AE_ROI_X_SIZE              0x3144
#define AR0135_R3146_AE_ROI_Y_SIZE              0x3146

#define AR0135_EXPOSURE_DEFAULT			0x0160

#define AR0135_I2C_ADDR      0x10
#define AR0135_ID_REG        0x3000
#define AR0135_ID_VAL        0x0554

//#define BIT(n) (1 << (n))

struct AR0135_reg_value {
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

#if 0
static const struct AR0135_reg_value AR0135at_rev4_recommended_setting[] = {
	{0x3ED6, 0x3CB5},
	{0x3ED8, 0x8765},
	{0x3EDA, 0x8888},
	{0x3EDC, 0x97FF},
	{0x3EF8, 0x6522},
	{0x3EFA, 0x2222},
	{0x3EFC, 0x6666},
	{0x3F00, 0xAA05},
	{0x3EE2, 0x180E},
	{0x3EE4, 0x0808},
	{0X3EEA, 0x2A09},
	{0x3060, 0x000D},
	{0x3092, 0x00CF},
	{0x3268, 0x0030},
	{0x3786, 0x0006},
	{0x3F4A, 0x0F70},
	{0x306E, 0x4810},
	{0x3064, 0x1802},
	{0x3EF6, 0x804D},
	{0x3180, 0xC08F},
	{0x30BA, 0x7623},
	{0x3176, 0x0480},
	{0x3178, 0x0480},
	{0x317A, 0x0480},
	{0x317C, 0x0480},
};
#endif
static const struct AR0135_reg_value AR0135at_pll_27mhz[] = {
	{0x302A, 0x0004},
	{0x302C, 0x0002},
	{0x302E, 0x0002},
	{0x3030, 0x0030},
	{0x30B0, 0x0480},
};

#if 0
static const struct AR0135_reg_value AR0135at_mipi_2lane_12bit[] = {
	{0x31AC, 0x0C0C},
	{0x31B0, 0x0042},
	{0x31B2, 0x002E},
	{0x31B4, 0x1665},
	{0x31B6, 0x110E},
	{0x31B8, 0x2047},
	{0x31BA, 0x0105},
	{0x31BC, 0x0004},
};
#endif
static const struct AR0135_reg_value AR0135at_1280x960_30fps[] = {
	{0x3002, 0x0000},
	{0x3004, 0x0000},
	{0x3006, 0x03C0},
	{0x3008, 0x04FF},
	{0x300A, 0x05B0}, //ambrealla : 0x03E5
	{0x300C, 0x0672},
	{0x3012, 0x0122},//ambrella 0x05DA
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x4000},//ambrella 0x0000
};

/*
static const struct AR0135_reg_value AR0135at_context_b_2x2_binning[] = {
	{0x3040, 0x0400},
	{0x30A8, 0x0003},
	{0x3040, 0x0C00},
};
*/
static const struct AR0135_reg_value AR0135at_embedded_data_stats[] = {
	{0x3064, 0x1882}, //amrella 0x1802
	{0x3064, 0x1982}, //appear twice
};

static const struct AR0135_reg_value AR0135at_auto_exposure[] = {
	{0x3046, 0x0100}, // LED_FLASH_EN = 1
	{0x3100, 0x0007}, // amrella 0x0000 appeares twice AE_ENABLE=1; AUTO_AG_EN=1 (Analog gain); Digital gain enabled 
	{0x311C, 0x0342}, // amrella 0x05DC AE_MAX_EXPOSURE (in rows)
	{0x3102, 0x0500}, // amgrella 0x0708 AE_LUMA_TARGET
	{0x3108, 0x0001}, // amgrella N\A AE_MIN_EV_STEP_REG
	{0x310A, 0x0100}, // ambrells 0x10FC - appear twicwAE_MAX_EV_STEP_REG
	{0x310C, 0x0100}, // amgrella N\A AE_DAMP_OFFSET_REG
	{0x310E, 0x0100}, // amgrella N\A AE_DAMP_GAIN_REG
	{0x3110, 0x00E0}, // amgrella N\A AE_DAMP_MAX_REG
};

static const struct AR0135_reg_value AR0135at_start_stream[] = {
	//{0x3070, 0x0002}, /* color bar pattern */
	{0x3028, 0x0010},
	//{0x301A, 0x005C},
	{0x301A, 0x10FC}, //appear 4 times 0x10DC,0x0018, 0x1098,0x10F8,0x10FC
};

static const struct AR0135_reg_value AR0135at_stop_stream[] = {
	{0x301A, 0x0058},
};

static const s64 AR0135_link_freq[] = {
		//600000000
	72000000
};

static int ar0135_write_reg8(struct AR0135 *ar0135, u16 reg, u8 val)
{
    u8 regbuf[3];
    int ret;

    regbuf[0] = reg >> 8;
    regbuf[1] = reg & 0xff;
    regbuf[2] = val;

     ret = i2c_master_send(ar0135->i2c_client, regbuf, 3);
    if (ret < 0)
        dev_err(&ar0135->i2c_client->dev,
            "%s: write reg error %d: reg=%x, val=%x\n",
            __func__, ret, reg, val);

     return ret;
}

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
			"%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ar0135->i2c_client, buf, 2);
	if (ret < 0) {
		dev_err(&ar0135->i2c_client->dev,
			"%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}
	*val = buf[0] << 8;
	*val |= (buf[1] & 0xff);

	return 0;
}

#if 0
static int ar0135_read_reg8(struct AR0135 *ar0135, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_master_send(ar0135->i2c_client, reg, 1);
	if (ret < 0) {
		dev_err(&ar0135->i2c_client->dev,
			"%s: write reg 8 bit error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ar0135->i2c_client, val, 1);
	if (ret < 0) {
		dev_err(&ar0135->i2c_client->dev,
			"%s: read reg 8 bit error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	return 0;
}
#endif
static int ar0135_set_register_array(struct AR0135 *ar0135,
				     const struct AR0135_reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = ar0135_write_reg(ar0135, settings->reg, settings->val);
		printk(KERN_ALERT "-------->ar0135_set_register_array %d reg: 0x%x val 0x%x\n",i,settings->reg,settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ar0135_s_power(struct v4l2_subdev *sd, int on)
{
	struct AR0135 *ar0135 = to_ar0135(sd);
	u16 reg_val;
	int ret = 0;

	printk(KERN_ALERT "-------->ar0135_s_power start");
	if (!on)
		return 0;

	mutex_lock(&ar0135->lock);

#if 0
	gpiod_direction_output(AR0135->rst_gpio, 1);
	if (!on)
		goto out;
	msleep(2); /* more than 1ms */
	gpiod_set_value_cansleep(AR0135->rst_gpio, 0);
	msleep(10); /* more than 160000 clocks at 24MHz; FIXME: use clk rate */
#endif

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
	printk(KERN_ALERT "-------->ar0135_s_power stop");
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

	printk(KERN_ALERT "-------->ar0135_get_format start");

	mutex_lock(&ar0135->lock);
	format->format = *__ar0135_get_pad_format(ar0135, cfg, format->pad,
						  format->which);
	mutex_unlock(&ar0135->lock);
	printk(KERN_ALERT "-------->ar0135_get_format end");
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

	printk(KERN_ALERT "-------->ar0135_set_format start\n");

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
	printk(KERN_ALERT "-------->ar0135_set_format end\n");
	return 0;
}

static int ar0135_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	printk(KERN_ALERT "-------->ar0135_entity_init_cfg start\n");

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;

	ar0135_set_format(subdev, cfg, &fmt);
	printk(KERN_ALERT "-------->ar0135_entity_init_cfg stop\n");
	return 0;
}

static int ar0135_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	struct AR0135 *ar0135 = to_ar0135(sd);

	printk(KERN_ALERT "-------->ar0135_get_selection start\n");
	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ar0135_get_pad_crop(ar0135, cfg, sel->pad,
					sel->which);
	printk(KERN_ALERT "-------->ar0135_get_selection end\n");
	return 0;
}

static void set_read_mode(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
    u8 mode = 0x00;

    if (core->hflip)
        mode |= 0x01;

    if (core->vflip)
        mode |= 0x02;

    ar0135_write_reg8(core, AR0135_R301D_IMAGE_ORIENTATION, mode);
}

static void set_ae(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);
    u16 val = 0x00;

    if (core->ae)
        val = 0x03;

    ar0135_write_reg(core, AR0135_R3100_AECTRLREG, val);
}

 static void set_gain(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);

    ar0135_write_reg(core, AR0135_R3060_ANALOG_GAIN, core->global_gain);
}

 static void set_exposure(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);

    ar0135_write_reg(core, AR0135_R3012_COARSE_INTEGRATION_TIME, core->exposure);
}

 static void set_ae_roi(struct v4l2_subdev *sd)
{
    struct AR0135 *core = to_ar0135(sd);

    ar0135_write_reg(core, AR0135_R3140_AE_ROI_X_START_OFFSET , core->ae_roi_x_start_offset);
    ar0135_write_reg(core, AR0135_R3142_AE_ROI_Y_START_OFFSET, core->ae_roi_y_start_offset);
    ar0135_write_reg(core, AR0135_R3144_AE_ROI_X_SIZE, core->ae_roi_x_size);
    ar0135_write_reg(core, AR0135_R3146_AE_ROI_Y_SIZE, core->ae_roi_y_size);
}

static int ar0135_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct AR0135 *ar0135 = to_ar0135(subdev);
	struct v4l2_subdev *sd = &ar0135->sd;
	int ret;
	//u16 reg_val;

	printk(KERN_ALERT "-------->ar0135_s_stream 1\n");

	mutex_lock(&ar0135->lock);

	printk(KERN_ALERT "-------->ar0135_s_stream 2\n");
	if (enable == 0) {
		ret = ar0135_set_register_array(
			ar0135, AR0135at_stop_stream,
			ARRAY_SIZE(AR0135at_stop_stream));
		goto out;
	}
	printk(KERN_ALERT "-------->ar0135_s_stream 3\n");
	ret = ar0135_set_register_array(ar0135, AR0135at_pll_27mhz,
					ARRAY_SIZE(AR0135at_pll_27mhz));
	if (ret < 0)
		goto out;
	printk(KERN_ALERT "-------->ar0135_s_stream 4\n");	
	msleep(100);
#if 0
	ret = ar0135_set_register_array(AR0135, AR0135at_mipi_2lane_12bit,
					ARRAY_SIZE(AR0135at_mipi_2lane_12bit));
	if (ret < 0)
		goto out;
#endif
	printk(KERN_ALERT "-------->ar0135_s_stream 5\n");
	ret = ar0135_set_register_array(ar0135, AR0135at_1280x960_30fps,
					ARRAY_SIZE(AR0135at_1280x960_30fps));
	if (ret < 0)
		goto out;
	printk(KERN_ALERT "-------->ar0135_s_stream 6\n");
	// ret = ar0135_set_register_array(
	// 	AR0135, AR0135at_context_b_2x2_binning,
	// 	ARRAY_SIZE(AR0135at_context_b_2x2_binning));
	// if (ret < 0)
	// 	goto out;

	ret = ar0135_set_register_array(
		ar0135, AR0135at_embedded_data_stats,
		ARRAY_SIZE(AR0135at_embedded_data_stats));
	if (ret < 0)
		goto out;
	printk(KERN_ALERT "-------->ar0135_s_stream 7\n");
	ret = ar0135_set_register_array(ar0135, AR0135at_auto_exposure,
						ARRAY_SIZE(AR0135at_auto_exposure));

	if (ret < 0)
		goto out;
	printk(KERN_ALERT "-------->ar0135_s_stream 8\n");
	set_read_mode(subdev);
    set_ae(subdev);
    set_gain(subdev);
    set_exposure(subdev);	
        set_ae_roi(sd);
	printk(KERN_ALERT "-------->ar0135_s_stream 9 - start streaming\n");
	ret = ar0135_set_register_array(ar0135, AR0135at_start_stream,
					ARRAY_SIZE(AR0135at_start_stream));

	//ar0135_read_reg(ar0135, 0x301A, &reg_val);
	//printk(KERN_ALERT "-------->ar0135_s_stream 9 - start streaming read value: 0x%x\n",reg_val);
	//reg_val |= BIT(2);
	//printk(KERN_ALERT "-------->ar0135_s_stream 9 - start streaming write value: 0x%x\n",reg_val);
	//ar0135_write_reg(ar0135, 0x301A, reg_val);
	printk(KERN_ALERT "-------->ar0135_s_stream 10\n");				

out:
	mutex_unlock(&ar0135->lock);

	printk(KERN_ALERT "-------->ar0135_s_stream done. ret: 0x%x\n",ret);
	return ret;
}

static int ar0135_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct AR0135 *core =
		container_of(ctrl->handler, struct AR0135, ctrls);
	struct v4l2_subdev *sd = &core->sd;

 	switch (ctrl->id) {
    case V4L2_CID_EXPOSURE_AUTO:
        core->ae = (ctrl->val == V4L2_EXPOSURE_AUTO) ? 1 : 0;
        break;
	case V4L2_CID_GAIN:
		core->global_gain = ctrl->val;
		break;
	case V4L2_CID_EXPOSURE:
		core->exposure = ctrl->val;
		break;
	case V4L2_CID_HFLIP:
		core->hflip = ctrl->val;
		return 0;
	case V4L2_CID_VFLIP:
		core->vflip = ctrl->val;
		return 0;
    case AR0135_CID_AE_ROI:
        core->ae_roi_x_start_offset = ctrl->p_cur.p_u16[0];
        core->ae_roi_y_start_offset = ctrl->p_cur.p_u16[1];
        core->ae_roi_x_size = ctrl->p_cur.p_u16[2];
        core->ae_roi_y_size = ctrl->p_cur.p_u16[3];
        set_ae_roi(sd);
        return 0;		
	default:
		return -EINVAL;
	}

 	return 0;
}

static const struct v4l2_ctrl_ops AR0135_ctrl_ops = {
	.s_ctrl = ar0135_s_ctrl,
};

static const struct v4l2_ctrl_config AR0135_ae_roi = {
    .ops = &AR0135_ctrl_ops,
    .id = AR0135_CID_AE_ROI,
    .name = "AR0135 AE ROI",
    .type = V4L2_CTRL_TYPE_U16,
    .def = 0x0,
    .min = 0x0,
    .max = 0x1280,
    .step = 1,
    .dims = { 4 },
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

#if 0
struct AR0135_fdp_link_init_t AR0135_fdp_link_init_arr[] = {
	{0x3d, 0x4C, 0x01},
	{0x3d, 0x33, 0x23},
	{0x3d, 0x20, 0x20},
	{0x3d, 0x5D, 0x20},
	{0x3d, 0x65, 0x20},
	{0x3d, 0x6E, 0x88},
	{0x3d, 0x58, 0x58},
	//{0x3d, 0x6D, 0x7E},
	{0x3d, 0x6D, 0x01},
	{0x3d, 0xD5, 0xF0},	
	{0x3d, 0x5C, 0xB4},	
	{0x5a, 0x0d, 0x59},	
	//{0x5a, 0x05, 0x18},
	{0x5a, 0x05, 0x30},
	{0x5a, 0x01, 0x30},
	{0x5a, 0x02, 0x20},
	{0x5a, 0x03, 0xC5},
	{0x5a, 0x05, 0x18},
	{0x5a, 0x06, 0x60																																							},
};
#endif

struct AR0135_fdp_link_init_t AR0135_fdp_link_init_arr[] = {
	{0x3d, 0x4C, 0x01},
	{0x3d, 0x33, 0x01},
	{0x3d, 0x20, 0xe0},
	{0x3d, 0x5D, 0x00},
	{0x3d, 0x65, 0x00},
	{0x3d, 0x6E, 0x88},
	{0x3d, 0x58, 0xd8},
	{0x3d, 0x6D, 0x7f},
	{0x3d, 0xD5, 0xF8},	
	{0x3d, 0x5C, 0x00}
};

static int ar0135_fpd_link_i2c_read(struct i2c_adapter *i2c_master,
				     struct AR0135_fdp_link_init_t *reg_item)
{
	struct i2c_msg msg[2];
	//u8 buf[2];
	int ret;
	u8 register_offset = reg_item->addr;

	reg_item->val = 0; //sets the value to 0

	//-------------------------------
	//write command
	//------------------------------
	msg[0].addr = reg_item->id;
	msg[0].flags = 0;	//write
	msg[0].buf = &register_offset;
	msg[0].len = 1;

	//-------------------------------
	//Read command
	//-------------------------------
	msg[1].addr = reg_item->id;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &(reg_item->val);
	msg[1].len = 1;

	ret = i2c_transfer(i2c_master, msg, 2);
	if (ret < 0)
		return ret;
	if (ret < 1)
		return -EIO;

	
	return 0;
}

static void dump_fpd_link(struct i2c_adapter *i2c_master, const char *msg)
{
	int ret = 0;
	int addr;
	for (addr = 0 ; addr < 0xff ; addr++)
	{
		struct AR0135_fdp_link_init_t read_val;
		read_val.id = 0x3d;
		read_val.addr = addr;
		read_val.val = 0;
		ret  = ar0135_fpd_link_i2c_read(/*i2c_master->adapter*/i2c_master, &read_val);
		//printk(KERN_ALERT "-------->%s: reading id: 0x%x addr 0x%x val 0x%x\n", msg, read_val.id, read_val.addr, read_val.val);	
		if (ret < 0)
		{
			printk(KERN_ALERT "-------->ERROR %s: reading id: 0x%x addr 0x%x val 0x%x\n", msg, read_val.id, read_val.addr, read_val.val);	
		}				
		else
		{
			printk(KERN_ALERT "-------->%s: reading id: 0x%x addr 0x%x val 0x%x\n", msg, read_val.id, read_val.addr, read_val.val);	
		}
	}
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
	struct AR0135_fdp_link_init_t read_val;

	for (i = 0; i < ARRAY_SIZE(AR0135_fdp_link_init_arr); i++) {
		ret = ar0135_fpd_link_i2c_write(ar0135_i2c_client->adapter,
						&AR0135_fdp_link_init_arr[i]);
		if (ret < 0)
		{
			printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: ERROR WRITING: 0x%x addr 0x%x val 0x%x\n", AR0135_fdp_link_init_arr[i].id, AR0135_fdp_link_init_arr[i].addr, AR0135_fdp_link_init_arr[i].val);					 
			return ret;
		}
		printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: writing id: 0x%x addr 0x%x val 0x%x\n", AR0135_fdp_link_init_arr[i].id, AR0135_fdp_link_init_arr[i].addr, AR0135_fdp_link_init_arr[i].val);					
	}

	printk(KERN_ALERT "----------------------------------->");		
	printk(KERN_ALERT "sleep 1 sec");		
	msleep(1000);			
	dump_fpd_link(ar0135_i2c_client->adapter, "ar0135_fpd_link_init adapter oren");
	
	read_val.id = 0x3d;
	read_val.addr = 0x00;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x5a;
	read_val.addr = 0x00;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0x70;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0x71;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0x7a;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0x7b;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0x7c;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0xb0;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0xba;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	read_val.id = 0x3d;
	read_val.addr = 0x6d;
	read_val.val = 0;
	ret  = ar0135_fpd_link_i2c_read(ar0135_i2c_client->adapter, &read_val);
	printk(KERN_ALERT "-------->ar0135_fpd_link_init adapter: reading id: 0x%x addr 0x%x val 0x%x\n", read_val.id, read_val.addr, read_val.val);					

	printk(KERN_ALERT "<-----------------------------------");					

	return 0;
}

static int ar0135_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct AR0135 *ar0135;
	int ret;

	//u8 addr;

	printk(KERN_ALERT "--------> AR0135_probe 1\n");
	ret = ar0135_fpd_link_init(client);
	if (ret < 0)
		return ret;	

	printk(KERN_ALERT "--------> AR0135_probe 2\n");
	ar0135 = devm_kzalloc(dev, sizeof(struct AR0135), GFP_KERNEL);
	if (!ar0135)
		return -ENOMEM;

	ar0135->i2c_client = client;
	ar0135->dev = dev;
	mutex_init(&ar0135->lock);

	dev_info(dev, "Detected Primax AR135 version of camera\n");

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

	printk(KERN_ALERT "--------> AR0135_probe 3\n");
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ar0135->ep);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	printk(KERN_ALERT "--------> AR0135_probe 4\n");
	of_node_put(endpoint);

	if (ar0135->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be parallel\n");
		return -EINVAL;
	}

	printk(KERN_ALERT "--------> AR0135_probe 5\n");
	
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
			  V4L2_CID_EXPOSURE, 0, 0x01CF, 1, AR0135_EXPOSURE_DEFAULT);
	ar0135->exposure = AR0135_EXPOSURE_DEFAULT;
	v4l2_ctrl_new_std(&ar0135->ctrls, &AR0135_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ar0135->ctrls, &AR0135_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_custom(&ar0135->ctrls, &AR0135_ae_roi, NULL);

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

	dev_info(dev, "AR0135 detected at address 0x%02x\n", client->addr);

	ret = v4l2_async_register_subdev(&ar0135->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	printk(KERN_ALERT "--------> AR0135_probe 6\n");

	ar0135_entity_init_cfg(&ar0135->sd, NULL);

	printk(KERN_ALERT "--------> AR0135_probe 7\n");
	dump_fpd_link(client->adapter, "ar0135_probe read");
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
