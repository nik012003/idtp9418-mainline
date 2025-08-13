#include "linux/printk.h"
#include <linux/module.h>
#include <linux/alarmtimer.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include "idtp9418.h"
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/memory.h>
#include <linux/pinctrl/consumer.h>

#define LIMIT_SOC 85

#define MAC_LEN 6

struct idtp9418_device_info;

struct idtp9418_dt_props
{
	unsigned int irq_gpio;
	unsigned int pen_det_active_low;
	unsigned int pen_det_active_high;
	unsigned int reverse_gpio;
	unsigned int rx_ovp_ctl_gpio;
	unsigned int reverse_boost_enable_gpio;
};

struct idtp9418_access_func
{
	int (*read)(struct idtp9418_device_info *di, uint16_t reg, uint8_t *val);
	int (*write)(struct idtp9418_device_info *di, uint16_t reg, uint8_t val);
	int (*read_buf)(struct idtp9418_device_info *di, uint16_t reg, uint8_t *buf,
					uint32_t size);
	int (*write_buf)(struct idtp9418_device_info *di, uint16_t reg, uint8_t *buf,
					 uint32_t size);
};

struct idtp9418_device_info
{
	int chip_enable;
	char *name;
	struct device *dev;
	struct idtp9418_access_func bus;
	struct regmap *regmap;
	struct idtp9418_dt_props dt_props;
	int irq;
	int hall3_irq;
	int hall4_irq;
	struct delayed_work irq_work;
	struct delayed_work hall_irq_work;
	struct delayed_work wpc_det_work;
	struct pinctrl *idt_pinctrl;
	struct pinctrl_state *idt_gpio_active;
	// struct pinctrl_state *idt_gpio_suspend;
	struct power_supply *wireless_psy;
	struct mutex i2c_lock;
};

void reverse_clrInt(struct idtp9418_device_info *di);

int idtp9418_read(struct idtp9418_device_info *di, uint16_t reg, uint8_t *val)
{
	unsigned int temp;
	int rc;
	msleep(100);

	mutex_lock(&di->i2c_lock);
	rc = regmap_read(di->regmap, reg, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	mutex_unlock(&di->i2c_lock);
	if (rc < 0)
	{
		dev_err(di->dev, "read error: %d. while reading reg: 0x%04x val: 0x%02x\n",
				rc, reg, *val);
		return rc;
	}
	return rc;
}

int idtp9418_write(struct idtp9418_device_info *di, uint16_t reg, uint8_t val)
{
	int rc = 0;

	mutex_lock(&di->i2c_lock);
	rc = regmap_write(di->regmap, reg, val);
	if (rc < 0)
	{
		dev_err(di->dev, "write error: %d at reg: 0x%04x val: 0x%02x\n",
				rc, reg, val);
	}
	mutex_unlock(&di->i2c_lock);
	return rc;
}

int idtp9418_read_buffer(struct idtp9418_device_info *di, uint16_t reg, uint8_t *buf,
						 uint32_t size)
{
	int rc = 0;

	while (size--)
	{
		rc = di->bus.read(di, reg++, buf++);
		if (rc < 0)
		{
			dev_err(di->dev, "[idt] read buf error: %d\n", rc);
			return rc;
		}
	}

	return rc;
}

int idtp9418_write_buffer(struct idtp9418_device_info *di, uint16_t reg, uint8_t *buf,
						  uint32_t size)
{
	int rc = 0;

	while (size--)
	{
		rc = di->bus.write(di, reg++, *buf++);
		if (rc < 0)
		{
			dev_err(di->dev, "[idt] write error: %d\n", rc);
			return rc;
		}
	}

	return rc;
}

uint32_t ExtractPacketSize(uint8_t hdr)
{
	if (hdr < 0x20)
		return 1;
	if (hdr < 0x80)
		return (2 + ((hdr - 0x20) >> 4));
	if (hdr < 0xe0)
		return (8 + ((hdr - 0x80) >> 3));
	return (20 + ((hdr - 0xe0) >> 2));
}

void idtp922x_sendPkt(struct idtp9418_device_info *di, ProPkt_Type *pkt)
{
	uint32_t size = ExtractPacketSize(pkt->header) + 1;
	di->bus.write_buf(di, REG_PROPPKT, (uint8_t *)pkt,
					  size);				  // write data into proprietary packet buffer
	di->bus.write(di, REG_SSCMND, SENDPROPP); // send proprietary packet

	dev_info(di->dev, "pkt header: 0x%x and cmd: 0x%x\n", pkt->header,
			 pkt->cmd);
}

void idtp922x_receivePkt(struct idtp9418_device_info *di, uint8_t *buf)
{
	uint8_t header;
	int rc;
	uint32_t size;

	rc = di->bus.read(di, REG_BCHEADER, &header);
	if (rc < 0)
	{
		dev_err(di->dev, "read header error: %d\n", rc);
		return;
	}
	size = ExtractPacketSize(header) + 1;
	rc = di->bus.read_buf(di, REG_BCDATA, buf, size);
	if (rc < 0)
		dev_err(di->dev, "[idt] read Tx data error: %d\n", rc);
}

void idtp922x_receivePkt2(struct idtp9418_device_info *di, uint8_t *buf)
{
	int rc;

	rc = di->bus.read_buf(di, REG_PROPPKT, buf, 2);
	if (rc < 0)
		dev_err(di->dev, "[idt] read Tx data error: %d\n", rc);
}

void idtp922x_get_tx_vin(struct idtp9418_device_info *di)
{
	ProPkt_Type pkt;
	pkt.header = PROPRIETARY18;
	pkt.cmd = BC_READ_Vin;

	idtp922x_sendPkt(di, &pkt);
}

/*
	Turn on/off the power to the chip
	This turn on "reverse-enable" gpio which contorls the
	mosfet to power the TPS22916 boost converter
	By default, vout of the TPS22916 is the same as the input voltage
*/
static int idtp9418_set_chip_power(struct idtp9418_device_info *di,
								   int enable)
{
	int ret;

	if (gpio_is_valid(di->dt_props.reverse_gpio))
	{
		ret = gpio_request(di->dt_props.reverse_gpio,
						   "reverse-enable-gpio");
		if (ret)
		{
			dev_err(di->dev,
					"%s: unable to request reverse gpio [%d]\n",
					__func__, di->dt_props.reverse_gpio);
			return ret;
		}

		ret = gpio_direction_output(di->dt_props.reverse_gpio,
									!!enable);
		if (ret)
		{
			dev_err(di->dev,
					"%s: cannot set direction for reverse enable gpio [%d]\n",
					__func__, di->dt_props.reverse_gpio);
		}

		gpio_free(di->dt_props.reverse_gpio);
	}
	if (!ret)
	{
		di->chip_enable = enable;
	}
	return ret;
}

void idtp9418_retry_id_auth(struct idtp9418_device_info *di)
{
	ProPkt_Type pkt;
	pkt.header = PROPRIETARY38;
	pkt.cmd = BC_RX_ID_AUTH;

	pkt.data[0] = 0x02;
	pkt.data[1] = 0xbb;

	idtp922x_sendPkt(di, &pkt);
}

static int idtp9418_get_vout(struct idtp9418_device_info *di)
{
	uint8_t vout_l, vout_h;
	int vout = -1;
	if (!di)
		return 0;
	di->bus.read(di, REG_ADC_VOUT_L, &vout_l);
	di->bus.read(di, REG_ADC_VOUT_H, &vout_h);
	vout = vout_l | ((vout_h & 0xf) << 8);
	vout = vout * 10 * 21 * 1000 / 40950 + ADJUST_METE_MV; // vout = val/4095*10*2.1
	dev_info(di->dev, "%s: vout is %d\n", __func__, vout);
	return vout;
}

static void idtp9418_set_vout(struct idtp9418_device_info *di, int mv)
{
	uint16_t val;
	uint8_t vout_l, vout_h;
	if (!di)
		return;
	// if (!di->power_good_flag)
	// 	return;
	val = (mv - 2800) * 10 / 84;
	vout_l = val & 0xff;
	vout_h = val >> 8;
	di->bus.write(di, REG_VOUT_SET, vout_l);
	di->bus.write(di, REG_VRECT_ADJ, vout_h);
	dev_info(di->dev, "[idtp9418]: set vout voltage: %d %d\n", mv, val);
}

static void idt_set_reverse_fod(struct idtp9418_device_info *di, int mw)
{
	uint8_t mw_l, mw_h;
	if (!di)
		return;
	mw_l = mw & 0xff;
	mw_h = mw >> 8;
	di->bus.write(di, REG_FOD_LOW, mw_l);
	di->bus.write(di, REG_FOD_HIGH, mw_h);
	dev_info(di->dev, "set reverse fod: %d\n", mw);
}

static int idtp9418_get_iout(struct idtp9418_device_info *di)
{
	uint8_t cout_l, cout_h;
	int iout = -1;
	if (!di)
		return 0;

	di->bus.read(di, REG_RX_LOUT_L, &cout_l);
	di->bus.read(di, REG_RX_LOUT_H, &cout_h);
	iout = cout_l | (cout_h << 8);

	return iout;
}

static int idtp9418_get_power_profile(struct idtp9418_device_info *di)
{
	uint8_t mode;
	int ret;

	di->bus.read(di, REG_WPC_MODE, &mode);
	ret = mode & BIT(3);
	dev_info(di->dev, "tx is epp ? ret is 0x%x\n", mode);

	return ret;
}

static int idtp9418_get_vrect(struct idtp9418_device_info *di)
{
	uint8_t data_list[2];
	int vrect;
	di->bus.read_buf(di, REG_ADC_VRECT, data_list, 2);
	vrect = data_list[0] | ((data_list[1] & 0xf) << 8);
	vrect = vrect * 125 * 21 * 100 / 40950; // vrect = val/4095*12.5*2.1

	return vrect;
}

static int idtp9418_get_power_max(struct idtp9418_device_info *di)
{
	int power_max;
	uint8_t val;

	di->bus.read(di, REG_POWER_MAX, &val);
	power_max = (val / 2) * 1000;
	dev_info(di->dev, "rx power max is %dmW\n", power_max);

	return power_max;
}

void idtp922x_request_low_addr(struct idtp9418_device_info *di)
{
	ProPkt_Type pkt;
	pkt.header = PROPRIETARY18;
	pkt.cmd = CMD_GET_BLEMAC_2_0;

	idtp922x_sendPkt(di, &pkt);
}

void idtp922x_request_high_addr(struct idtp9418_device_info *di)
{
	ProPkt_Type pkt;
	pkt.header = PROPRIETARY18;
	pkt.cmd = CMD_GET_BLEMAC_5_3;

	idtp922x_sendPkt(di, &pkt);
}

/*
	Use the reverse-boost-enable-gpio gpio to turn on/off the voltage boosting.
	This needs to be turned on after the chip is powered.
*/
static int idtp9418_enable_boost_converter(struct idtp9418_device_info *di, int enable)
{
	int ret = 0;

	if (!gpio_is_valid(di->dt_props.reverse_boost_enable_gpio))
	{
		dev_err(di->dev, "%s: reverse_boost_enable_gpio is invalid\n", __func__);
		return -EINVAL;
	}

	ret = gpio_request(di->dt_props.reverse_boost_enable_gpio, "reverse-boost-enable-gpio");
	if (ret)
	{
		dev_err(di->dev, "%s: unable to request reverse_boost_enable_gpio [%d]\n",
				__func__, di->dt_props.reverse_boost_enable_gpio);
		return ret;
	}

	ret = gpio_direction_output(di->dt_props.reverse_boost_enable_gpio, !!enable);
	if (ret)
	{
		dev_err(di->dev, "%s: cannot set direction for reverse_boost_enable_gpio [%d]\n",
				__func__, di->dt_props.reverse_boost_enable_gpio);
	}

	gpio_free(di->dt_props.reverse_boost_enable_gpio);
	return ret;
}

static ssize_t chip_version_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	uint8_t chip_id_l, chip_id_h, chip_rev, cust_id, status, vset;
	uint8_t fw_otp_ver[4], fw_app_ver[4];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct idtp9418_device_info *di = i2c_get_clientdata(client);
	msleep(100);
	di->bus.read(di, REG_STATUS_L, &status);
	di->bus.read(di, REG_VOUT_SET, &vset);

	di->bus.read(di, REG_CHIP_ID_L, &chip_id_l);
	di->bus.read(di, REG_CHIP_ID_H, &chip_id_h);
	di->bus.read(di, REG_CHIP_REV, &chip_rev);
	chip_rev = chip_rev >> 4;
	di->bus.read(di, REG_CTM_ID, &cust_id);
	di->bus.read_buf(di, REG_OTPFWVER_ADDR, fw_otp_ver, 4);
	di->bus.read_buf(di, REG_EPRFWVER_ADDR, fw_app_ver, 4);

	return snprintf(
		buf, PAGE_SIZE,
		"chip_id_l:%02x\nchip_id_h:%02x\nchip_rev:%02x\ncust_id:%02x status:%02x vset:%02x\n otp_ver:%x.%x.%x.%x\n app_ver:%x.%x.%x.%x\n",
		chip_id_l, chip_id_h, chip_rev, cust_id, status, vset,
		fw_otp_ver[0], fw_otp_ver[1], fw_otp_ver[2],
		fw_otp_ver[3], fw_app_ver[0], fw_app_ver[1],
		fw_app_ver[2], fw_app_ver[3]);
}

static int idt_get_reverse_vout(struct idtp9418_device_info *di)
{
	u8 vout_l, vout_h;
	int vout = -1;

	if (!di)
		return vout;

	di->bus.read(di, REG_REVERSE_VIN_LOW, &vout_l);
	di->bus.read(di, REG_REVERSE_VIN_HIGH, &vout_h);
	vout = vout_l | (vout_h << 8);
	dev_info(di->dev, "[reverse] vout is %d\n", vout);
	return vout;
}

static int idt_get_reverse_soc(struct idtp9418_device_info *di)
{
	u8 soc = -1;

	if (!di)
		return soc;

	di->bus.read(di, REG_CHG_STATUS, &soc);
	if ((soc < 0) || (soc > 0x64))
	{
		if (soc == 0xFF)
		{
			dev_info(di->dev, "[reverse] soc is default 0xFF\n");
			return -1;
		}
		else
		{
			dev_info(di->dev, "[reverse] soc illegal: %d\n", soc);
			return -1;
		}
	}

	dev_info(di->dev, "[reverse] soc is %d\n", soc);
	return soc;
}

static int idt_get_reverse_iin(struct idtp9418_device_info *di)
{
	u8 iin_l, iin_h;
	int iin = -1;

	if (!di)
		return iin;

	di->bus.read(di, REG_REVERSE_IIN_LOW, &iin_l);
	di->bus.read(di, REG_REVERSE_IIN_HIGH, &iin_h);
	iin = iin_l | (iin_h << 8);
	dev_info(di->dev, "[reverse] iin is %d\n", iin);
	return iin;
}

static int idt_get_reverse_temp(struct idtp9418_device_info *di)
{
	u8 temp = -1;

	if (!di)
		return temp;

	di->bus.read(di, REG_REVERSE_TEMP, &temp);
	dev_info(di->dev, "[reverse] temp is %d\n", temp);
	return temp;
}

/* voltage limit attrs */
static ssize_t chip_vout_show(struct device *dev, struct device_attribute *attr,
							  char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct idtp9418_device_info *di = i2c_get_clientdata(client);
	int vout;

	vout = idtp9418_get_vout(di);

	idt_get_reverse_vout(di);
	idt_get_reverse_iin(di);
	idt_get_reverse_temp(di);
	idt_get_reverse_soc(di);

	return snprintf(buf, PAGE_SIZE, "%d\n", vout);
}

static ssize_t chip_powered_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct idtp9418_device_info *di = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%d\n", di->chip_enable);
}

static ssize_t chip_power_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	bool enable;
	int ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct idtp9418_device_info *di = i2c_get_clientdata(client);

	ret = kstrtobool(buf, &enable);
	if (ret < 0)
		return ret;

	idtp9418_set_chip_power(di, enable);

	return count;
}

static DEVICE_ATTR(chip_version, S_IRUGO, chip_version_show, NULL);
static DEVICE_ATTR(chip_powered, S_IWUSR | S_IRUGO, chip_powered_show, chip_power_store);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_chip_version.attr,
	&dev_attr_chip_powered.attr,
	NULL,
};

static const struct attribute_group sysfs_group_attrs = {
	.attrs = sysfs_attrs,
};

static int idtp9418_parse_dt(struct idtp9418_device_info *di)
{
	struct device_node *node = di->dev->of_node;

	if (!node)
	{
		dev_err(di->dev, "device tree node missing\n");
		return -EINVAL;
	}
	struct
	{
		const char *name;
		unsigned int *prop;
	} gpio_map[] = {
		{"idt,irq", &di->dt_props.irq_gpio},
		{"hall,int3", &di->dt_props.pen_det_active_low},
		{"hall,int4", &di->dt_props.pen_det_active_high},
		{"idt,reverse-enable", &di->dt_props.reverse_gpio},
		{"idt,reverse-booset-enable", &di->dt_props.reverse_boost_enable_gpio}};

	for (int i = 0; i < ARRAY_SIZE(gpio_map); i++)
	{
		*gpio_map[i].prop = of_get_named_gpio(node, gpio_map[i].name, 0);
		if (!gpio_is_valid(*gpio_map[i].prop))
		{
			dev_err(di->dev, "dt parse error, missing %s\n", gpio_map[i].name);
			return -ENODEV;
		}
	}

	return 0;
}

static int idtp9418_gpio_init(struct idtp9418_device_info *di)
{
	int ret = 0;
	int irqn = 0;

	di->idt_pinctrl = devm_pinctrl_get(di->dev);
	if (IS_ERR_OR_NULL(di->idt_pinctrl))
	{
		dev_err(di->dev, "No pinctrl config specified\n");
		ret = PTR_ERR(di->dev);
		return ret;
	}
	di->idt_gpio_active =
		pinctrl_lookup_state(di->idt_pinctrl, "idt_active");
	if (IS_ERR_OR_NULL(di->idt_gpio_active))
	{
		dev_err(di->dev, "No active config specified\n");
		ret = PTR_ERR(di->idt_gpio_active);
		return ret;
	}

	ret = pinctrl_select_state(di->idt_pinctrl, di->idt_gpio_active);
	if (ret < 0)
	{
		dev_err(di->dev, "fail to select pinctrl active rc=%d\n", ret);
		return ret;
	}

	if (gpio_is_valid(di->dt_props.irq_gpio))
	{
		irqn = gpio_to_irq(di->dt_props.irq_gpio);
		if (irqn < 0)
		{
			ret = irqn;
			goto err_irq_gpio;
		}
		di->irq = irqn;
	}
	else
	{
		dev_err(di->dev, "%s: irq gpio not provided\n", __func__);
		goto err_irq_gpio;
	}

	if (gpio_is_valid(di->dt_props.pen_det_active_low))
	{
		irqn = gpio_to_irq(di->dt_props.pen_det_active_low);
		if (irqn < 0)
		{
			ret = irqn;
			goto err_irq_gpio;
		}
		di->hall3_irq = irqn;
	}
	else
	{
		dev_err(di->dev, "%s:hall3 irq gpio not provided\n", __func__);
		goto err_hall3_irq_gpio;
	}

	if (gpio_is_valid(di->dt_props.pen_det_active_high))
	{
		irqn = gpio_to_irq(di->dt_props.pen_det_active_high);
		if (irqn < 0)
		{
			ret = irqn;
			goto err_irq_gpio;
		}
		di->hall4_irq = irqn;
	}
	else
	{
		dev_err(di->dev, "%s:hall4 irq gpio not provided\n", __func__);
		goto err_hall4_irq_gpio;
	}

err_hall4_irq_gpio:
	gpio_free(di->dt_props.pen_det_active_high);
err_hall3_irq_gpio:
	gpio_free(di->dt_props.pen_det_active_low);
err_irq_gpio:
	gpio_free(di->dt_props.irq_gpio);
	return ret;
}

static bool need_irq_cleared(struct idtp9418_device_info *di)
{
	uint8_t int_buf[4];
	uint32_t int_val;
	int rc = -1;

	rc = di->bus.read_buf(di, REG_SYS_INT, int_buf, 4);
	if (rc < 0)
	{
		dev_err(di->dev, "%s: read int state error\n", __func__);
		return true;
	}
	int_val = int_buf[0] | (int_buf[1] << 8) | (int_buf[2] << 16) |
			  (int_buf[3] << 24);
	if (int_val != 0)
	{
		dev_info(di->dev, "irq not clear right: 0x%08x\n", int_val);
		return true;
	}

	if (gpio_is_valid(di->dt_props.irq_gpio))
		rc = gpio_get_value(di->dt_props.irq_gpio);
	else
	{
		dev_err(di->dev, "%s: irq gpio not provided\n", __func__);
		rc = -1;
	}
	if (!rc)
	{
		dev_info(di->dev, "irq low, need clear int: %d\n", rc);
		return true;
	}
	return false;
}

static bool reverse_need_irq_cleared(struct idtp9418_device_info *di, uint32_t val)
{
	uint8_t int_buf[4];
	uint32_t int_val;
	int rc = -1;

	rc = di->bus.read_buf(di, REG_SYS_INT, int_buf, 4);
	if (rc < 0)
	{
		dev_err(di->dev, "%s: read int state error\n", __func__);
		return false;
	}
	int_val = int_buf[0] | (int_buf[1] << 8) | (int_buf[2] << 16) |
			  (int_buf[3] << 24);
	if (int_val && (int_val == val))
	{
		dev_info(di->dev, "irq clear wrong, retry: 0x%08x\n", int_val);
		return true;
	}

	return false;
}

static int idtp9418_reverse_charge_enable(struct idtp9418_device_info *di)
{
	u8 mode = 0;
	int i = 0;
	int fod_set = REVERSE_FOD;

	/* set reverse fod to REVERSE_FOD */
	idt_set_reverse_fod(di, fod_set);
	for (i = 0; i < 3; i++)
	{
		di->bus.write(di, REG_TX_CMD, TX_EN | TX_FOD_EN);
		msleep(50);
		di->bus.read(di, REG_TX_DATA, &mode);
		dev_info(di->dev, "tx data(0078): 0x%x\n", mode);
		if (mode & BIT(0))
		{
			dev_info(di->dev, "start reverse charging\n");
			break;
		}
		else
		{
			dev_err(di->dev, "idt set reverse charge failed, retry: %d\n", i);
		}
	}

	if (i < 3)
	{
		dev_info(di->dev, "reverse charging start success\n");
		return 1;
	}
	else
	{
		// Clean up
		dev_info(di->dev, "reverse charging failed start\n");
		idtp9418_enable_boost_converter(di, false);
		idtp9418_set_chip_power(di, false);
		return 0;
	}
}

static void idtp9418_irq_hall_work(struct work_struct *work)
{
	struct idtp9418_device_info *di = container_of(work, struct idtp9418_device_info, hall_irq_work.work);
	bool attached = false;

	if (gpio_is_valid(di->dt_props.pen_det_active_low) && gpio_is_valid(di->dt_props.pen_det_active_high))
	{
		if (!gpio_get_value(di->dt_props.pen_det_active_low) && gpio_get_value(di->dt_props.pen_det_active_high))
		{
			dev_info(di->dev, "[idtp] Pen attached! Turning on reverse charging... \n");
			attached = true;
		}
		else
		{
			dev_info(di->dev, "[idtp] Pen detached! Turning off reverse charging... \n");
		}
	}

	idtp9418_set_chip_power(di, false);
	idtp9418_enable_boost_converter(di, false);
	msleep(100);
	if (attached)
	{
		idtp9418_enable_boost_converter(di, true);
		msleep(600);
		idtp9418_set_chip_power(di, true);
		msleep(3000);
		idtp9418_reverse_charge_enable(di);
	}
	pm_relax(di->dev);
}

static void idtp9418_irq_work(struct work_struct *work)
{
	struct idtp9418_device_info *di = container_of(work, struct idtp9418_device_info, irq_work.work);

	dev_err(di->dev, "%s: Entered irq work\n", __func__);

	u8 int_buf[4] = {0};
	u32 int_val = 0;
	int rc = 0;
	u8 recive_data[5] = {0};
	int irq_level;

	if (gpio_is_valid(di->dt_props.irq_gpio))
		irq_level = gpio_get_value(di->dt_props.irq_gpio);
	else
	{
		dev_err(di->dev, "%s: irq gpio not provided\n", __func__);
		irq_level = -1;
		pm_relax(di->dev);
		return;
	}
	if (irq_level)
	{
		dev_info(di->dev, "irq is high level, ignore%d\n", irq_level);
		pm_relax(di->dev);
		return;
	}

	rc = di->bus.read_buf(di, REG_SYS_INT, int_buf, 4);
	if (rc < 0)
	{
		dev_err(di->dev, "[idt]read int state error: %d\n", rc);
	}
	int_val =
		int_buf[0] | (int_buf[1] << 8) | (int_buf[2] << 16) | (int_buf[3] << 24);

	dev_info(di->dev, "Irq got: %d\n", int_val);

	msleep(5);
	if (reverse_need_irq_cleared(di, int_val))
	{
		reverse_clrInt(di);
		msleep(5);
	}
	pm_relax(di->dev);
}

static irqreturn_t idtp9418_irq_handler(int irq, void *dev_id)
{
	struct idtp9418_device_info *di = dev_id;
	pm_stay_awake(di->dev);
	schedule_delayed_work(&di->irq_work, msecs_to_jiffies(10));
	return IRQ_HANDLED;
}

static irqreturn_t idtp9418_hall_irq_handler(int irq, void *dev_id)
{
	struct idtp9418_device_info *di = dev_id;
	pm_stay_awake(di->dev);
	schedule_delayed_work(&di->hall_irq_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static int idtp9418_request_and_enable_irq(struct device *dev,
										   int irq,
										   irq_handler_t handler,
										   unsigned long flags,
										   const char *name,
										   void *dev_id)
{
	int ret;

	if (!irq)
	{
		dev_err(dev, "%s: %s irq is invalid\n", __func__, name);
		return -EINVAL;
	}

	ret = request_irq(irq, handler, flags, name, dev_id);
	if (ret)
	{
		dev_err(dev, "%s: request_irq(%s) failed, ret=%d\n", __func__, name, ret);
		return ret;
	}

	ret = enable_irq_wake(irq);
	if (ret)
	{
		dev_err(dev, "%s: enable_irq_wake(%s) failed, ret=%d\n", __func__, name, ret);
		free_irq(irq, dev_id); // cleanup on failure
		return ret;
	}

	return 0;
}

static int idtp9418_setup_interrupts(struct idtp9418_device_info *di)
{
	int ret;

	ret = idtp9418_request_and_enable_irq(di->dev,
										  di->irq,
										  idtp9418_irq_handler,
										  IRQF_TRIGGER_FALLING,
										  di->name,
										  di);
	if (ret)
		return ret;

	ret = idtp9418_request_and_enable_irq(di->dev,
										  di->hall3_irq,
										  idtp9418_hall_irq_handler,
										  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
										  "hall3_irq",
										  di);
	if (ret)
		return ret;

	ret = idtp9418_request_and_enable_irq(di->dev,
										  di->hall4_irq,
										  idtp9418_hall_irq_handler,
										  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
										  "hall4_irq",
										  di);
	if (ret)
		return ret;

	return 0;
}

static struct regmap_config i2c_idtp9418_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static const struct i2c_device_id idtp9418_id[] = {
	{IDT_DRIVER_NAME, 0},
	{},
};

static const struct of_device_id idt_match_table[] = {{.compatible =
														   "idt,p9418"},
													  {}};

MODULE_DEVICE_TABLE(i2c, idtp9418_id);

void idtp9418_clrInt(struct idtp9418_device_info *di)
{
	uint8_t clr_buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	di->bus.write_buf(di, REG_SYS_INT_CLR, clr_buf, sizeof(clr_buf));
	di->bus.write(di, REG_SSCMND, CLRINT);
}

void reverse_clrInt(struct idtp9418_device_info *di)
{
	uint8_t clr_buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	di->bus.write_buf(di, REG_SYS_INT_CLR, clr_buf, sizeof(clr_buf));
	di->bus.write(di, REG_TX_CMD, TX_FOD_EN | TX_CLRINT);

	dev_err(di->dev, "Interrupt cleared!\n");
}

static int idtp9418_probe(struct i2c_client *client)
{
	struct idtp9418_device_info *di;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct power_supply_config idtp_cfg = {};

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
	{
		dev_err(&client->dev, "i2c check functionality failed!\n");
		return -EIO;
	}

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
	{
		dev_err(&client->dev,
				"i2c allocated device info data failed!\n");
		return -ENOMEM;
	}

	di->name = IDT_DRIVER_NAME;
	di->dev = &client->dev;
	di->chip_enable = 0;

	di->regmap = devm_regmap_init_i2c(client, &i2c_idtp9418_regmap_config);
	if (!di->regmap)
		return -ENODEV;
	di->bus.read = idtp9418_read;
	di->bus.write = idtp9418_write;
	di->bus.read_buf = idtp9418_read_buffer;
	di->bus.write_buf = idtp9418_write_buffer;

	mutex_init(&di->i2c_lock);
	device_init_wakeup(&client->dev, true);
	i2c_set_clientdata(client, di);

	int ret = idtp9418_parse_dt(di);
	if (ret < 0)
	{
		dev_err(di->dev, "%s: parse dt error [%d]\n", __func__, ret);
		return ret;
	}

	ret = idtp9418_gpio_init(di);
	if (ret < 0)
	{
		dev_err(di->dev, "%s: gpio init error [%d]\n", __func__, ret);
		return ret;
	}

	if (sysfs_create_group(&client->dev.kobj, &sysfs_group_attrs))
	{
		dev_err(&client->dev, "create sysfs attrs failed!\n");
		return -EIO;
	}

	idtp_cfg.drv_data = di;

	INIT_DELAYED_WORK(&di->irq_work, idtp9418_irq_work);
	INIT_DELAYED_WORK(&di->hall_irq_work, idtp9418_irq_hall_work);

	idtp9418_setup_interrupts(di);
	idtp9418_set_chip_power(di, false);

	return 0;
}

static void idtp9418_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &sysfs_group_attrs);

	struct idtp9418_device_info *di = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);

	cancel_delayed_work_sync(&di->irq_work);
	cancel_delayed_work_sync(&di->hall_irq_work);

	free_irq(di->hall3_irq, di);
	free_irq(di->hall4_irq, di);
	free_irq(di->irq, di);

	gpio_free(di->dt_props.irq_gpio);
	gpio_free(di->dt_props.pen_det_active_low);
	gpio_free(di->dt_props.pen_det_active_high);
}

static void idtp9418_shutdown(struct i2c_client *client)
{
}

static struct i2c_driver idtp9418_driver = {
	.driver = {
		.name = IDT_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = idt_match_table,
	},
	.probe = idtp9418_probe,
	.remove = idtp9418_remove,
	.shutdown = idtp9418_shutdown,
	.id_table = idtp9418_id,
};

static int __init idt_init(void)
{
	int ret;

	ret = i2c_add_driver(&idtp9418_driver);
	if (ret)
		printk(KERN_ERR "idt i2c driver init failed!\n");

	return ret;
}

static void __exit idt_exit(void)
{
	i2c_del_driver(&idtp9418_driver);
}

module_init(idt_init);
module_exit(idt_exit);

MODULE_AUTHOR("bsp@xiaomi.com");
MODULE_DESCRIPTION("idtp9418 Wireless Power Charger Monitor driver");
MODULE_LICENSE("GPL v2");