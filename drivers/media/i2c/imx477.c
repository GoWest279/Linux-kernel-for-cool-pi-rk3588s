// SPDX-License-Identifier: GPL-2.0
/*
 * imx477 driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 * V0.0X01.0X01 add imx477 driver.
 * V0.0X01.0X02 add imx477 support mirror and flip.
 * V0.0X01.0X03 add quick stream on/off
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/rk-preisp.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define IMX477_LINK_FREQ_848		840000000// 1696Mbps

#define IMX477_LANES			2

#define PIXEL_RATE_WITH_848M_10BIT	(IMX477_LINK_FREQ_848 * 2 / 10 * 4)
#define PIXEL_RATE_WITH_848M_12BIT	(IMX477_LINK_FREQ_848 * 2 / 12 * 4)

#define IMX477_XVCLK_FREQ		24000000

#define CHIP_ID				0x0477
#define IMX477_REG_CHIP_ID_H		0x0016
#define IMX477_REG_CHIP_ID_L		0x0017 // ?

#define IMX477_REG_CTRL_MODE		0x0100
#define IMX477_MODE_SW_STANDBY		0x0
#define IMX477_MODE_STREAMING		0x1

#define IMX477_REG_EXPOSURE_H		0x0202
#define IMX477_REG_EXPOSURE_L		0x0203//?
#define IMX477_EXPOSURE_MIN		    20
#define IMX477_EXPOSURE_STEP		1
#define IMX477_VTS_MAX			0x7fff//?

#define IMX477_REG_GAIN_H		0x0204
#define IMX477_REG_GAIN_L		0x0205//?
#define IMX477_GAIN_MIN			0x00
#define IMX477_GAIN_MAX			978 
#define IMX477_GAIN_STEP		1
#define IMX477_GAIN_DEFAULT		0x0
// ? DGAIN 
#define IMX477_REG_DGAIN		0x020E
#define IMX477_DGAIN_MODE		1
#define IMX477_REG_DGAINGR_H		0x020e
#define IMX477_REG_DGAINGR_L		0x020f
#define IMX477_REG_DGAINR_H		0x0210
#define IMX477_REG_DGAINR_L		0x0211
#define IMX477_REG_DGAINB_H		0x0212
#define IMX477_REG_DGAINB_L		0x0213
#define IMX477_REG_DGAINGB_H		0x0214
#define IMX477_REG_DGAINGB_L		0x0215
#define IMX477_REG_GAIN_GLOBAL_H	0x3ffc
#define IMX477_REG_GAIN_GLOBAL_L	0x3ffd

//#define IMX477_REG_TEST_PATTERN_H	0x0600
#define IMX477_REG_TEST_PATTERN	0x0600
#define IMX477_TEST_PATTERN_ENABLE	0x1
#define IMX477_TEST_PATTERN_DISABLE	0x0


//
#define IMX477_REG_VTS_H		0x0340
#define IMX477_REG_VTS_L		0x0341

#define IMX477_FLIP_MIRROR_REG		0x0101
#define IMX477_MIRROR_BIT_MASK		BIT(0)
#define IMX477_FLIP_BIT_MASK		BIT(1)

#define IMX477_FETCH_EXP_H(VAL)		(((VAL) >> 8) & 0xFF)
#define IMX477_FETCH_EXP_L(VAL)		((VAL) & 0xFF)

#define IMX477_FETCH_AGAIN_H(VAL)		(((VAL) >> 8) & 0x03)
#define IMX477_FETCH_AGAIN_L(VAL)		((VAL) & 0xFF)

#define IMX477_FETCH_DGAIN_H(VAL)		(((VAL) >> 8) & 0x0F)
#define IMX477_FETCH_DGAIN_L(VAL)		((VAL) & 0xFF)

#define IMX477_FETCH_RHS1_H(VAL)	(((VAL) >> 16) & 0x0F)
#define IMX477_FETCH_RHS1_M(VAL)	(((VAL) >> 8) & 0xFF)
#define IMX477_FETCH_RHS1_L(VAL)	((VAL) & 0xFF)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF
//   
#define IMX477_REG_VALUE_08BIT		1
#define IMX477_REG_VALUE_16BIT		2
//#define IMX477_REG_VALUE_24BIT		3

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define IMX477_NAME			"imx477"

static const char * const imx477_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX477_NUM_SUPPLIES ARRAY_SIZE(imx477_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct imx477_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct imx477 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[IMX477_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct imx477_mode *cur_mode;
	u32			cfg_num;
	u32			cur_pixel_rate;
	u32			cur_link_freq;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			has_init_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	u8			flip;
};

#define to_imx477(sd) container_of(sd, struct imx477, subdev)



/*
 *IMX477LQR All-pixel scan CSI-2_4lane 24Mhz
 *AD:12bit Output:12bit 1696Mbps Master Mode 30fps
 *Tool ver : Ver4.0
 */
static const struct regval imx477_linear_12_4056x3040_regs[] = {
	{0x0101, 0x00},
		{0x0136, 0x18},
	{0x0137, 0x00},
	{0xe000, 0x00},
	{0xe07a, 0x01},
	{0x0808, 0x02},
	{0x4ae9, 0x18},
	{0x4aea, 0x08},
	{0xf61c, 0x04},
	{0xf61e, 0x04},
	{0x4ae9, 0x21},
	{0x4aea, 0x80},
	{0x38a8, 0x1f},
	{0x38a9, 0xff},
	{0x38aa, 0x1f},
	{0x38ab, 0xff},
	{0x55d4, 0x00},
	{0x55d5, 0x00},
	{0x55d6, 0x07},
	{0x55d7, 0xff},
	{0x55e8, 0x07},
	{0x55e9, 0xff},
	{0x55ea, 0x00},
	{0x55eb, 0x00},
	{0x574c, 0x07},
	{0x574d, 0xff},
	{0x574e, 0x00},
	{0x574f, 0x00},
	{0x5754, 0x00},
	{0x5755, 0x00},
	{0x5756, 0x07},
	{0x5757, 0xff},
	{0x5973, 0x04},
	{0x5974, 0x01},
	{0x5d13, 0xc3},
	{0x5d14, 0x58},
	{0x5d15, 0xa3},
	{0x5d16, 0x1d},
	{0x5d17, 0x65},
	{0x5d18, 0x8c},
	{0x5d1a, 0x06},
	{0x5d1b, 0xa9},
	{0x5d1c, 0x45},
	{0x5d1d, 0x3a},
	{0x5d1e, 0xab},
	{0x5d1f, 0x15},
	{0x5d21, 0x0e},
	{0x5d22, 0x52},
	{0x5d23, 0xaa},
	{0x5d24, 0x7d},
	{0x5d25, 0x57},
	{0x5d26, 0xa8},
	{0x5d37, 0x5a},
	{0x5d38, 0x5a},
	{0x5d77, 0x7f},
	{0x7b75, 0x0e},
	{0x7b76, 0x0b},
	{0x7b77, 0x08},
	{0x7b78, 0x0a},
	{0x7b79, 0x47},
	{0x7b7c, 0x00},
	{0x7b7d, 0x00},
	{0x8d1f, 0x00},
	{0x8d27, 0x00},
	{0x9004, 0x03},
	{0x9200, 0x50},
	{0x9201, 0x6c},
	{0x9202, 0x71},
	{0x9203, 0x00},
	{0x9204, 0x71},
	{0x9205, 0x01},
	{0x9371, 0x6a},
	{0x9373, 0x6a},
	{0x9375, 0x64},
	{0x991a, 0x00},
	{0x996b, 0x8c},
	{0x996c, 0x64},
	{0x996d, 0x50},
	{0x9a4c, 0x0d},
	{0x9a4d, 0x0d},
	{0xa001, 0x0a},
	{0xa003, 0x0a},
	{0xa005, 0x0a},
	{0xa006, 0x01},
	{0xa007, 0xc0},
	{0xa009, 0xc0},
	{0x3d8a, 0x01},
	{0x4421, 0x04},
	{0x7b3b, 0x01},
	{0x7b4c, 0x00},
	{0x9905, 0x00},
	{0x9907, 0x00},
	{0x9909, 0x00},
	{0x990b, 0x00},
	{0x9944, 0x3c},
	{0x9947, 0x3c},
	{0x994a, 0x8c},
	{0x994b, 0x50},
	{0x994c, 0x1b},
	{0x994d, 0x8c},
	{0x994e, 0x50},
	{0x994f, 0x1b},
	{0x9950, 0x8c},
	{0x9951, 0x1b},
	{0x9952, 0x0a},
	{0x9953, 0x8c},
	{0x9954, 0x1b},
	{0x9955, 0x0a},
	{0x9a13, 0x04},
	{0x9a14, 0x04},
	{0x9a19, 0x00},
	{0x9a1c, 0x04},
	{0x9a1d, 0x04},
	{0x9a26, 0x05},
	{0x9a27, 0x05},
	{0x9a2c, 0x01},
	{0x9a2d, 0x03},
	{0x9a2f, 0x05},
	{0x9a30, 0x05},
	{0x9a41, 0x00},
	{0x9a46, 0x00},
	{0x9a47, 0x00},
	{0x9c17, 0x35},
	{0x9c1d, 0x31},
	{0x9c29, 0x50},
	{0x9c3b, 0x2f},
	{0x9c41, 0x6b},
	{0x9c47, 0x2d},
	{0x9c4d, 0x40},
	{0x9c6b, 0x00},
	{0x9c71, 0xc8},
	{0x9c73, 0x32},
	{0x9c75, 0x04},
	{0x9c7d, 0x2d},
	{0x9c83, 0x40},
	{0x9c94, 0x3f},
	{0x9c95, 0x3f},
	{0x9c96, 0x3f},
	{0x9c97, 0x00},
	{0x9c98, 0x00},
	{0x9c99, 0x00},
	{0x9c9a, 0x3f},
	{0x9c9b, 0x3f},
	{0x9c9c, 0x3f},
	{0x9ca0, 0x0f},
	{0x9ca1, 0x0f},
	{0x9ca2, 0x0f},
	{0x9ca3, 0x00},
	{0x9ca4, 0x00},
	{0x9ca5, 0x00},
	{0x9ca6, 0x1e},
	{0x9ca7, 0x1e},
	{0x9ca8, 0x1e},
	{0x9ca9, 0x00},
	{0x9caa, 0x00},
	{0x9cab, 0x00},
	{0x9cac, 0x09},
	{0x9cad, 0x09},
	{0x9cae, 0x09},
	{0x9cbd, 0x50},
	{0x9cbf, 0x50},
	{0x9cc1, 0x50},
	{0x9cc3, 0x40},
	{0x9cc5, 0x40},
	{0x9cc7, 0x40},
	{0x9cc9, 0x0a},
	{0x9ccb, 0x0a},
	{0x9ccd, 0x0a},
	{0x9d17, 0x35},
	{0x9d1d, 0x31},
	{0x9d29, 0x50},
	{0x9d3b, 0x2f},
	{0x9d41, 0x6b},
	{0x9d47, 0x42},
	{0x9d4d, 0x5a},
	{0x9d6b, 0x00},
	{0x9d71, 0xc8},
	{0x9d73, 0x32},
	{0x9d75, 0x04},
	{0x9d7d, 0x42},
	{0x9d83, 0x5a},
	{0x9d94, 0x3f},
	{0x9d95, 0x3f},
	{0x9d96, 0x3f},
	{0x9d97, 0x00},
	{0x9d98, 0x00},
	{0x9d99, 0x00},
	{0x9d9a, 0x3f},
	{0x9d9b, 0x3f},
	{0x9d9c, 0x3f},
	{0x9d9d, 0x1f},
	{0x9d9e, 0x1f},
	{0x9d9f, 0x1f},
	{0x9da0, 0x0f},
	{0x9da1, 0x0f},
	{0x9da2, 0x0f},
	{0x9da3, 0x00},
	{0x9da4, 0x00},
	{0x9da5, 0x00},
	{0x9da6, 0x1e},
	{0x9da7, 0x1e},
	{0x9da8, 0x1e},
	{0x9da9, 0x00},
	{0x9daa, 0x00},
	{0x9dab, 0x00},
	{0x9dac, 0x09},
	{0x9dad, 0x09},
	{0x9dae, 0x09},
	{0x9dc9, 0x0a},
	{0x9dcb, 0x0a},
	{0x9dcd, 0x0a},
	{0x9e17, 0x35},
	{0x9e1d, 0x31},
	{0x9e29, 0x50},
	{0x9e3b, 0x2f},
	{0x9e41, 0x6b},
	{0x9e47, 0x2d},
	{0x9e4d, 0x40},
	{0x9e6b, 0x00},
	{0x9e71, 0xc8},
	{0x9e73, 0x32},
	{0x9e75, 0x04},
	{0x9e94, 0x0f},
	{0x9e95, 0x0f},
	{0x9e96, 0x0f},
	{0x9e97, 0x00},
	{0x9e98, 0x00},
	{0x9e99, 0x00},
	{0x9ea0, 0x0f},
	{0x9ea1, 0x0f},
	{0x9ea2, 0x0f},
	{0x9ea3, 0x00},
	{0x9ea4, 0x00},
	{0x9ea5, 0x00},
	{0x9ea6, 0x3f},
	{0x9ea7, 0x3f},
	{0x9ea8, 0x3f},
	{0x9ea9, 0x00},
	{0x9eaa, 0x00},
	{0x9eab, 0x00},
	{0x9eac, 0x09},
	{0x9ead, 0x09},
	{0x9eae, 0x09},
	{0x9ec9, 0x0a},
	{0x9ecb, 0x0a},
	{0x9ecd, 0x0a},
	{0x9f17, 0x35},
	{0x9f1d, 0x31},
	{0x9f29, 0x50},
	{0x9f3b, 0x2f},
	{0x9f41, 0x6b},
	{0x9f47, 0x42},
	{0x9f4d, 0x5a},
	{0x9f6b, 0x00},
	{0x9f71, 0xc8},
	{0x9f73, 0x32},
	{0x9f75, 0x04},
	{0x9f94, 0x0f},
	{0x9f95, 0x0f},
	{0x9f96, 0x0f},
	{0x9f97, 0x00},
	{0x9f98, 0x00},
	{0x9f99, 0x00},
	{0x9f9a, 0x2f},
	{0x9f9b, 0x2f},
	{0x9f9c, 0x2f},
	{0x9f9d, 0x00},
	{0x9f9e, 0x00},
	{0x9f9f, 0x00},
	{0x9fa0, 0x0f},
	{0x9fa1, 0x0f},
	{0x9fa2, 0x0f},
	{0x9fa3, 0x00},
	{0x9fa4, 0x00},
	{0x9fa5, 0x00},
	{0x9fa6, 0x1e},
	{0x9fa7, 0x1e},
	{0x9fa8, 0x1e},
	{0x9fa9, 0x00},
	{0x9faa, 0x00},
	{0x9fab, 0x00},
	{0x9fac, 0x09},
	{0x9fad, 0x09},
	{0x9fae, 0x09},
	{0x9fc9, 0x0a},
	{0x9fcb, 0x0a},
	{0x9fcd, 0x0a},
	{0xa14b, 0xff},
	{0xa151, 0x0c},
	{0xa153, 0x50},
	{0xa155, 0x02},
	{0xa157, 0x00},
	{0xa1ad, 0xff},
	{0xa1b3, 0x0c},
	{0xa1b5, 0x50},
	{0xa1b9, 0x00},
	{0xa24b, 0xff},
	{0xa257, 0x00},
	{0xa2ad, 0xff},
	{0xa2b9, 0x00},
	{0xb21f, 0x04},
	{0xb35c, 0x00},
	{0xb35e, 0x08},
	{0x0112, 0x0c},
	{0x0113, 0x0c},
	{0x0114, 0x01},
	{0x0350, 0x00},
	{0xbcf1, 0x02},
	{0x3ff9, 0x01},
	
	{0x0342, 0x5d},
	{0x0343, 0xc0},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b75, 0x0a},
	{0x7b76, 0x0c},
	{0x7b77, 0x07},
	{0x7b78, 0x06},
	{0x7b79, 0x3c},
	{0x7b53, 0x01},
	{0x9369, 0x5a},
	{0x936b, 0x55},
	{0x936d, 0x28},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x0f},
	{0x034d, 0xd8},
	{0x034e, 0x0b},
	{0x034f, 0xe0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x02},
	{0x3f57, 0xae},

	{REG_NULL, 0x00},
};


static const struct imx477_mode supported_modes[] = {
{
		.width = 4056,
		.height = 3040,
		.max_fps = {
			.numerator = 100,
			.denominator = 100000,
		},
		.exp_def = 0x0600,
		.hts_def = 0x1BD8,
		.vts_def = 0x0F57,
		.bus_fmt = MEDIA_BUS_FMT_SRGGB12_1X12,
		.reg_list = imx477_linear_12_4056x3040_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	IMX477_LINK_FREQ_848,
};

static const char * const imx477_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int imx477_write_reg(struct i2c_client *client, u16 reg,
			    int len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int imx477_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = imx477_write_reg(client, regs[i].addr,
					       IMX477_REG_VALUE_08BIT,
					       regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int imx477_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret, i;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs))
			break;
	}
	if (ret != ARRAY_SIZE(msgs) && i == 3)
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int imx477_get_reso_dist(const struct imx477_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct imx477_mode *
imx477_find_best_fit(struct imx477 *imx477, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < imx477->cfg_num; i++) {
		dist = imx477_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx477_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&imx477->mutex);

	mode = imx477_find_best_fit(imx477, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx477->mutex);
		return -ENOTTY;
#endif
	} else {
		imx477->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx477->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx477->vblank, vblank_def,
					 IMX477_VTS_MAX - mode->height,
					 1, vblank_def);

		if (imx477->cur_mode->bus_fmt == MEDIA_BUS_FMT_SRGGB10_1X10) {
			imx477->cur_link_freq = 0;
			imx477->cur_pixel_rate = PIXEL_RATE_WITH_848M_10BIT;
		} else if (imx477->cur_mode->bus_fmt ==
			   MEDIA_BUS_FMT_SRGGB12_1X12) {
			imx477->cur_link_freq = 0;
			imx477->cur_pixel_rate = PIXEL_RATE_WITH_848M_12BIT;
		}

		__v4l2_ctrl_s_ctrl_int64(imx477->pixel_rate,
					 imx477->cur_pixel_rate);
		__v4l2_ctrl_s_ctrl(imx477->link_freq,
				   imx477->cur_link_freq);
	}

	mutex_unlock(&imx477->mutex);

	return 0;
}

static int imx477_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode = imx477->cur_mode;

	mutex_lock(&imx477->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx477->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		if (imx477->flip & IMX477_MIRROR_BIT_MASK) {
			fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
			if (imx477->flip & IMX477_FLIP_BIT_MASK)
				fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
		} else if (imx477->flip & IMX477_FLIP_BIT_MASK) {
			fmt->format.code = MEDIA_BUS_FMT_SGBRG10_1X10;
		} else {
			fmt->format.code = mode->bus_fmt;
		}
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&imx477->mutex);

	return 0;
}

static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx477 *imx477 = to_imx477(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = imx477->cur_mode->bus_fmt;

	return 0;
}

static int imx477_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx477 *imx477 = to_imx477(sd);

	if (fse->index >= imx477->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int imx477_enable_test_pattern(struct imx477 *imx477, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | IMX477_TEST_PATTERN_ENABLE;
	else
		val = IMX477_TEST_PATTERN_DISABLE;

	return imx477_write_reg(imx477->client,
				IMX477_REG_TEST_PATTERN,
				IMX477_REG_VALUE_08BIT,
				val);
}

static int imx477_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode = imx477->cur_mode;

	mutex_lock(&imx477->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&imx477->mutex);

	return 0;
}

static int imx477_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode = imx477->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (IMX477_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	if (mode->hdr_mode == HDR_X2)
		val = 1 << (IMX477_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void imx477_get_module_inf(struct imx477 *imx477,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX477_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx477->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx477->len_name, sizeof(inf->base.lens));
}

static long imx477_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 i, h, w;
	u32 stream = 0;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_GET_MODULE_INFO:
		imx477_get_module_inf(imx477, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = imx477->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = imx477->cur_mode->width;
		h = imx477->cur_mode->height;
		for (i = 0; i < imx477->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				imx477->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == imx477->cfg_num) {
			dev_err(&imx477->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = imx477->cur_mode->hts_def -
			    imx477->cur_mode->width;
			h = imx477->cur_mode->vts_def -
			    imx477->cur_mode->height;
			__v4l2_ctrl_modify_range(imx477->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(imx477->vblank, h,
						 IMX477_VTS_MAX -
						 imx477->cur_mode->height,
						 1, h);

			if (imx477->cur_mode->bus_fmt ==
			    MEDIA_BUS_FMT_SRGGB10_1X10) {
				imx477->cur_link_freq = 0;
				imx477->cur_pixel_rate =
				PIXEL_RATE_WITH_848M_10BIT;
			} else if (imx477->cur_mode->bus_fmt ==
				   MEDIA_BUS_FMT_SRGGB12_1X12) {
				imx477->cur_link_freq = 0;
				imx477->cur_pixel_rate =
				PIXEL_RATE_WITH_848M_12BIT;
			}

			__v4l2_ctrl_s_ctrl_int64(imx477->pixel_rate,
						 imx477->cur_pixel_rate);
			__v4l2_ctrl_s_ctrl(imx477->link_freq,
					   imx477->cur_link_freq);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = imx477_write_reg(imx477->client, IMX477_REG_CTRL_MODE,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
		else
			ret = imx477_write_reg(imx477->client, IMX477_REG_CTRL_MODE,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx477_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx477_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = imx477_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx477_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = imx477_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = imx477_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = imx477_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int imx477_set_flip(struct imx477 *imx477)
{
	int ret = 0;
	u32 val = 0;

	ret = imx477_read_reg(imx477->client, IMX477_FLIP_MIRROR_REG,
			      IMX477_REG_VALUE_08BIT, &val);
	if (imx477->flip & IMX477_MIRROR_BIT_MASK)
		val |= IMX477_MIRROR_BIT_MASK;
	else
		val &= ~IMX477_MIRROR_BIT_MASK;
	if (imx477->flip & IMX477_FLIP_BIT_MASK)
		val |= IMX477_FLIP_BIT_MASK;
	else
		val &= ~IMX477_FLIP_BIT_MASK;
	ret |= imx477_write_reg(imx477->client, IMX477_FLIP_MIRROR_REG,
				IMX477_REG_VALUE_08BIT, val);

	return ret;
}

static int __imx477_start_stream(struct imx477 *imx477)
{
	int ret;

	ret = imx477_write_array(imx477->client, imx477->cur_mode->reg_list);
	if (ret)
		return ret;
	imx477->cur_vts = imx477->cur_mode->vts_def;
	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&imx477->ctrl_handler);
	if (ret)
		return ret;
	if (imx477->has_init_exp && imx477->cur_mode->hdr_mode != NO_HDR) {
		ret = imx477_ioctl(&imx477->subdev, PREISP_CMD_SET_HDRAE_EXP,
			&imx477->init_hdrae_exp);
		if (ret) {
			dev_err(&imx477->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	imx477_set_flip(imx477);

	return imx477_write_reg(imx477->client, IMX477_REG_CTRL_MODE,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
}

static int __imx477_stop_stream(struct imx477 *imx477)
{
	return imx477_write_reg(imx477->client, IMX477_REG_CTRL_MODE,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_SW_STANDBY);
}

static int imx477_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct i2c_client *client = imx477->client;
	int ret = 0;

	mutex_lock(&imx477->mutex);
	on = !!on;
	if (on == imx477->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx477_start_stream(imx477);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__imx477_stop_stream(imx477);
		pm_runtime_put(&client->dev);
	}

	imx477->streaming = on;

unlock_and_return:
	mutex_unlock(&imx477->mutex);

	return ret;
}

static int imx477_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct i2c_client *client = imx477->client;
	int ret = 0;

	mutex_lock(&imx477->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx477->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		imx477->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx477->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx477->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx477_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, IMX477_XVCLK_FREQ / 1000 / 1000);
}

static int __imx477_power_on(struct imx477 *imx477)
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx477->client->dev;

	ret = clk_set_rate(imx477->xvclk, IMX477_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(imx477->xvclk) != IMX477_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 37.125MHz\n");
	ret = clk_prepare_enable(imx477->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(imx477->reset_gpio))
		gpiod_set_value_cansleep(imx477->reset_gpio, 0);

	ret = regulator_bulk_enable(IMX477_NUM_SUPPLIES, imx477->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx477->reset_gpio))
		gpiod_set_value_cansleep(imx477->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(imx477->pwdn_gpio))
		gpiod_set_value_cansleep(imx477->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx477_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(imx477->xvclk);

	return ret;
}

static void __imx477_power_off(struct imx477 *imx477)
{
	if (!IS_ERR(imx477->pwdn_gpio))
		gpiod_set_value_cansleep(imx477->pwdn_gpio, 0);
	clk_disable_unprepare(imx477->xvclk);
	if (!IS_ERR(imx477->reset_gpio))
		gpiod_set_value_cansleep(imx477->reset_gpio, 0);
	regulator_bulk_disable(IMX477_NUM_SUPPLIES, imx477->supplies);
}

static int imx477_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	return __imx477_power_on(imx477);
}

static int imx477_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	__imx477_power_off(imx477);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx477_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx477_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx477->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx477->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int imx477_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx477 *imx477 = to_imx477(sd);

	if (fie->index >= imx477->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops imx477_pm_ops = {
	SET_RUNTIME_PM_OPS(imx477_runtime_suspend,
			   imx477_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx477_internal_ops = {
	.open = imx477_open,
};
#endif

static const struct v4l2_subdev_core_ops imx477_core_ops = {
	.s_power = imx477_s_power,
	.ioctl = imx477_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx477_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx477_video_ops = {
	.s_stream = imx477_s_stream,
	.g_frame_interval = imx477_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx477_pad_ops = {
	.enum_mbus_code = imx477_enum_mbus_code,
	.enum_frame_size = imx477_enum_frame_sizes,
	.enum_frame_interval = imx477_enum_frame_interval,
	.get_fmt = imx477_get_fmt,
	.set_fmt = imx477_set_fmt,
	.get_mbus_config = imx477_g_mbus_config,
};

static const struct v4l2_subdev_ops imx477_subdev_ops = {
	.core	= &imx477_core_ops,
	.video	= &imx477_video_ops,
	.pad	= &imx477_pad_ops,
};

static int imx477_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *imx477 = container_of(ctrl->handler,
					     struct imx477, ctrl_handler);
	struct i2c_client *client = imx477->client;
	s64 max;
	int ret = 0;
	u32 again = 0;
	u32 dgain = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx477->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx477->exposure,
					 imx477->exposure->minimum, max,
					 imx477->exposure->step,
					 imx477->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = imx477_write_reg(imx477->client,
				       IMX477_REG_EXPOSURE_H,
				       IMX477_REG_VALUE_08BIT,
				       IMX477_FETCH_EXP_H(ctrl->val));
		ret |= imx477_write_reg(imx477->client,
					IMX477_REG_EXPOSURE_L,
					IMX477_REG_VALUE_08BIT,
					IMX477_FETCH_EXP_L(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		again = ctrl->val > 978 ? 978 : ctrl->val;
		dgain = ctrl->val > 978 ? ctrl->val - 978 : 256;
		ret = imx477_write_reg(imx477->client, IMX477_REG_GAIN_H,
				       IMX477_REG_VALUE_08BIT,
				       IMX477_FETCH_AGAIN_H(again));
		ret |= imx477_write_reg(imx477->client, IMX477_REG_GAIN_L,
					IMX477_REG_VALUE_08BIT,
					IMX477_FETCH_AGAIN_L(again));
		ret |= imx477_write_reg(imx477->client, IMX477_REG_DGAIN,
					IMX477_REG_VALUE_08BIT,
					IMX477_DGAIN_MODE);
		if (IMX477_DGAIN_MODE && dgain > 0) {
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINGR_H,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_H(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINGR_L,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_L(dgain));
		} else if (dgain > 0) {
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINR_H,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_H(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINR_L,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_L(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINB_H,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_H(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINB_L,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_L(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINGB_H,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_H(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_DGAINGB_L,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_L(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_GAIN_GLOBAL_H,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_H(dgain));
			ret |= imx477_write_reg(imx477->client,
						IMX477_REG_GAIN_GLOBAL_L,
						IMX477_REG_VALUE_08BIT,
						IMX477_FETCH_DGAIN_L(dgain));
		}
		break;
	case V4L2_CID_VBLANK:
		ret = imx477_write_reg(imx477->client,
				       IMX477_REG_VTS_H,
				       IMX477_REG_VALUE_08BIT,
				       (ctrl->val + imx477->cur_mode->height)
				       >> 8);
		ret |= imx477_write_reg(imx477->client,
					IMX477_REG_VTS_L,
					IMX477_REG_VALUE_08BIT,
					(ctrl->val + imx477->cur_mode->height)
					& 0xff);
		imx477->cur_vts = ctrl->val + imx477->cur_mode->height;
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			imx477->flip |= IMX477_MIRROR_BIT_MASK;
		else
			imx477->flip &= ~IMX477_MIRROR_BIT_MASK;
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			imx477->flip |= IMX477_FLIP_BIT_MASK;
		else
			imx477->flip &= ~IMX477_FLIP_BIT_MASK;
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx477_enable_test_pattern(imx477, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_set_ctrl,
};

static int imx477_initialize_controls(struct imx477 *imx477)
{
	const struct imx477_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &imx477->ctrl_handler;
	mode = imx477->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &imx477->mutex;

	imx477->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						   V4L2_CID_LINK_FREQ,
						   0, 0, link_freq_menu_items);

	if (imx477->cur_mode->bus_fmt == MEDIA_BUS_FMT_SRGGB10_1X10) {
		imx477->cur_link_freq = 0;
		imx477->cur_pixel_rate = PIXEL_RATE_WITH_848M_10BIT;
	} else if (imx477->cur_mode->bus_fmt == MEDIA_BUS_FMT_SRGGB12_1X12) {
		imx477->cur_link_freq = 0;
		imx477->cur_pixel_rate = PIXEL_RATE_WITH_848M_12BIT;
	}

	imx477->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE,
					       0, PIXEL_RATE_WITH_848M_10BIT,
					       1, imx477->cur_pixel_rate);
	v4l2_ctrl_s_ctrl(imx477->link_freq,
			   imx477->cur_link_freq);

	h_blank = mode->hts_def - mode->width;
	imx477->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (imx477->hblank)
		imx477->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	imx477->vblank = v4l2_ctrl_new_std(handler, &imx477_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   IMX477_VTS_MAX - mode->height,
					   1, vblank_def);
	imx477->cur_vts = mode->vts_def;
	exposure_max = mode->vts_def - 4;
	imx477->exposure = v4l2_ctrl_new_std(handler, &imx477_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX477_EXPOSURE_MIN,
					     exposure_max,
					     IMX477_EXPOSURE_STEP,
					     mode->exp_def);
	imx477->anal_gain = v4l2_ctrl_new_std(handler, &imx477_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      IMX477_GAIN_MIN,
					      IMX477_GAIN_MAX,
					      IMX477_GAIN_STEP,
					      IMX477_GAIN_DEFAULT);
	imx477->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &imx477_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(imx477_test_pattern_menu) - 1,
				0, 0, imx477_test_pattern_menu);

	imx477->h_flip = v4l2_ctrl_new_std(handler, &imx477_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx477->v_flip = v4l2_ctrl_new_std(handler, &imx477_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	imx477->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&imx477->client->dev,
			"Failed to init controls(  %d  )\n", ret);
		goto err_free_handler;
	}

	imx477->subdev.ctrl_handler = handler;
	imx477->has_init_exp = false;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int imx477_check_sensor_id(struct imx477 *imx477,
				  struct i2c_client *client)
{
	struct device *dev = &imx477->client->dev;
	u16 id = 0;
	u32 reg_H = 0;
	u32 reg_L = 0;
	int ret;

	ret = imx477_read_reg(client, IMX477_REG_CHIP_ID_H,
			      IMX477_REG_VALUE_08BIT, &reg_H);
	ret |= imx477_read_reg(client, IMX477_REG_CHIP_ID_L,
			       IMX477_REG_VALUE_08BIT, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (!(reg_H == (CHIP_ID >> 8) || reg_L == (CHIP_ID & 0xff))) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected imx477 %04x sensor\n", id);
	return 0;
}

static int imx477_configure_regulators(struct imx477 *imx477)
{
	unsigned int i;

	for (i = 0; i < IMX477_NUM_SUPPLIES; i++)
		imx477->supplies[i].supply = imx477_supply_names[i];

	return devm_regulator_bulk_get(&imx477->client->dev,
				       IMX477_NUM_SUPPLIES,
				       imx477->supplies);
}

static int imx477_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx477 *imx477;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	imx477 = devm_kzalloc(dev, sizeof(*imx477), GFP_KERNEL);
	if (!imx477)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx477->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx477->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx477->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx477->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}

	imx477->client = client;
	imx477->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < imx477->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			imx477->cur_mode = &supported_modes[i];
			break;
		}
	}

	if (i == imx477->cfg_num)
		imx477->cur_mode = &supported_modes[0];

	imx477->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx477->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	imx477->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx477->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	imx477->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx477->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = imx477_configure_regulators(imx477);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&imx477->mutex);

	sd = &imx477->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx477_subdev_ops);

	ret = imx477_initialize_controls(imx477);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx477_power_on(imx477);
	if (ret)
		goto err_free_handler;

	ret = imx477_check_sensor_id(imx477, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx477_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	imx477->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx477->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx477->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx477->module_index, facing,
		 IMX477_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__imx477_power_off(imx477);
err_free_handler:
	v4l2_ctrl_handler_free(&imx477->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx477->mutex);

	return ret;
}

static int imx477_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx477->ctrl_handler);
	mutex_destroy(&imx477->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx477_power_off(imx477);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx477_of_match[] = {
	{ .compatible = "sony,imx477" },
	{},
};
MODULE_DEVICE_TABLE(of, imx477_of_match);
#endif

static const struct i2c_device_id imx477_match_id[] = {
	{ "sony,imx477", 0 },
	{ },
};

static struct i2c_driver imx477_i2c_driver = {
	.driver = {
		.name = IMX477_NAME,
		.pm = &imx477_pm_ops,
		.of_match_table = of_match_ptr(imx477_of_match),
	},
	.probe		= &imx477_probe,
	.remove		= &imx477_remove,
	.id_table	= imx477_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx477_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx477_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx477 sensor driver");
MODULE_LICENSE("GPL v2");
