// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale i.MX28 GPIO control code
 *
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 */

#include <common.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>

#if	defined(CONFIG_MX23)
#define	PINCTRL_BANKS		3
#define	PINCTRL_DOUT(n)		(0x0500 + ((n) * 0x10))
#define	PINCTRL_DIN(n)		(0x0600 + ((n) * 0x10))
#define	PINCTRL_DOE(n)		(0x0700 + ((n) * 0x10))
#define	PINCTRL_PIN2IRQ(n)	(0x0800 + ((n) * 0x10))
#define	PINCTRL_IRQEN(n)	(0x0900 + ((n) * 0x10))
#define	PINCTRL_IRQSTAT(n)	(0x0c00 + ((n) * 0x10))
#elif	defined(CONFIG_MX28)
#define	PINCTRL_BANKS		5
#define	PINCTRL_DOUT(n)		(0x0700 + ((n) * 0x10))
#define	PINCTRL_DIN(n)		(0x0900 + ((n) * 0x10))
#define	PINCTRL_DOE(n)		(0x0b00 + ((n) * 0x10))
#define	PINCTRL_PIN2IRQ(n)	(0x1000 + ((n) * 0x10))
#define	PINCTRL_IRQEN(n)	(0x1100 + ((n) * 0x10))
#define	PINCTRL_IRQSTAT(n)	(0x1400 + ((n) * 0x10))
#else
#error "Please select CONFIG_MX23 or CONFIG_MX28"
#endif

#define GPIO_INT_FALL_EDGE	0x0
#define GPIO_INT_LOW_LEV	0x1
#define GPIO_INT_RISE_EDGE	0x2
#define GPIO_INT_HIGH_LEV	0x3
#define GPIO_INT_LEV_MASK	(1 << 0)
#define GPIO_INT_POL_MASK	(1 << 1)

void mxs_gpio_init(void)
{
	int i;

	for (i = 0; i < PINCTRL_BANKS; i++) {
		writel(0, MXS_PINCTRL_BASE + PINCTRL_PIN2IRQ(i));
		writel(0, MXS_PINCTRL_BASE + PINCTRL_IRQEN(i));
		/* Use SCT address here to clear the IRQSTAT bits */
		writel(0xffffffff, MXS_PINCTRL_BASE + PINCTRL_IRQSTAT(i) + 8);
	}
}

#ifndef CONFIG_DM_GPIO
int gpio_get_value(unsigned gpio)
{
	uint32_t bank = PAD_BANK(gpio);
	uint32_t offset = PINCTRL_DIN(bank);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE + offset);

	return (readl(&reg->reg) >> PAD_PIN(gpio)) & 1;
}

void gpio_set_value(unsigned gpio, int value)
{
	uint32_t bank = PAD_BANK(gpio);
	uint32_t offset = PINCTRL_DOUT(bank);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE + offset);

	if (value)
		writel(1 << PAD_PIN(gpio), &reg->reg_set);
	else
		writel(1 << PAD_PIN(gpio), &reg->reg_clr);
}

int gpio_direction_input(unsigned gpio)
{
	uint32_t bank = PAD_BANK(gpio);
	uint32_t offset = PINCTRL_DOE(bank);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE + offset);

	writel(1 << PAD_PIN(gpio), &reg->reg_clr);

	return 0;
}

int gpio_direction_output(unsigned gpio, int value)
{
	uint32_t bank = PAD_BANK(gpio);
	uint32_t offset = PINCTRL_DOE(bank);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE + offset);

	gpio_set_value(gpio, value);

	writel(1 << PAD_PIN(gpio), &reg->reg_set);

	return 0;
}

int gpio_request(unsigned gpio, const char *label)
{
	if (PAD_BANK(gpio) >= PINCTRL_BANKS)
		return -1;

	return 0;
}

int gpio_free(unsigned gpio)
{
	return 0;
}

int name_to_gpio(const char *name)
{
	unsigned bank, pin;
	char *end;

	bank = simple_strtoul(name, &end, 10);

	if (!*end || *end != ':')
		return bank;

	pin = simple_strtoul(end + 1, NULL, 10);

	return (bank << MXS_PAD_BANK_SHIFT) | (pin << MXS_PAD_PIN_SHIFT);
}
#else /* CONFIG_DM_GPIO */
#include <dm.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#ifdef CONFIG_MX28
const struct mxs_gpio_platdata mxs_gpio_def = {
	.gpio_nr_of_bank_pins[0] = IMX28_GPIO_BANK0_PIN_NR,
	.gpio_nr_of_bank_pins[1] = IMX28_GPIO_BANK1_PIN_NR,
	.gpio_nr_of_bank_pins[2] = IMX28_GPIO_BANK2_PIN_NR,
	.gpio_nr_of_bank_pins[3] = IMX28_GPIO_BANK3_PIN_NR,
	.gpio_nr_of_bank_pins[4] = IMX28_GPIO_BANK4_PIN_NR,
	.gpio_base_nr[0] = 0,
	.gpio_base_nr[1] = IMX28_GPIO_BANK0_PIN_NR,
	.gpio_base_nr[2] = (IMX28_GPIO_BANK0_PIN_NR + \
			   IMX28_GPIO_BANK1_PIN_NR),
	.gpio_base_nr[3] = (IMX28_GPIO_BANK0_PIN_NR + \
			   IMX28_GPIO_BANK1_PIN_NR + \
			   IMX28_GPIO_BANK2_PIN_NR),
	.gpio_base_nr[4] = (IMX28_GPIO_BANK0_PIN_NR + \
			   IMX28_GPIO_BANK1_PIN_NR + \
			   IMX28_GPIO_BANK2_PIN_NR + \
			   IMX28_GPIO_BANK3_PIN_NR),
	.ngpio = (IMX28_GPIO_BANK0_PIN_NR + \
		  IMX28_GPIO_BANK1_PIN_NR + \
		  IMX28_GPIO_BANK2_PIN_NR + \
		  IMX28_GPIO_BANK3_PIN_NR + \
		  IMX28_GPIO_BANK4_PIN_NR),
};

#else
#error "Only i.MX28 supported with DM_GPIO"
#endif

struct mxs_gpio_priv {
	unsigned int bank;
};

static int mxs_gpio_get_value(struct udevice *dev, unsigned offset)
{
	struct mxs_gpio_priv *priv = dev_get_priv(dev);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE +
					   PINCTRL_DIN(priv->bank));

	return (readl(&reg->reg) >> offset) & 1;
}

static int mxs_gpio_set_value(struct udevice *dev, unsigned offset,
			      int value)
{
	struct mxs_gpio_priv *priv = dev_get_priv(dev);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE +
					   PINCTRL_DOUT(priv->bank));
	if (value)
		writel(1 << offset, &reg->reg_set);
	else
		writel(1 << offset, &reg->reg_clr);

	return 0;
}

static int mxs_gpio_direction_input(struct udevice *dev, unsigned offset)
{
	struct mxs_gpio_priv *priv = dev_get_priv(dev);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE +
					   PINCTRL_DOE(priv->bank));

	writel(1 << offset, &reg->reg_clr);

	return 0;
}

static int mxs_gpio_direction_output(struct udevice *dev, unsigned offset,
				     int value)
{
	struct mxs_gpio_priv *priv = dev_get_priv(dev);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE +
					   PINCTRL_DOE(priv->bank));

	mxs_gpio_set_value(dev, offset, value);

	writel(1 << offset, &reg->reg_set);

	return 0;
}

static int mxs_gpio_get_function(struct udevice *dev, unsigned offset)
{
	struct mxs_gpio_priv *priv = dev_get_priv(dev);
	struct mxs_register_32 *reg =
		(struct mxs_register_32 *)(MXS_PINCTRL_BASE +
					   PINCTRL_DOE(priv->bank));
	bool is_output = !!(readl(&reg->reg) >> offset);

	return is_output ? GPIOF_OUTPUT : GPIOF_INPUT;
}

static const struct dm_gpio_ops gpio_mxs_ops = {
	.direction_input	= mxs_gpio_direction_input,
	.direction_output	= mxs_gpio_direction_output,
	.get_value		= mxs_gpio_get_value,
	.set_value		= mxs_gpio_set_value,
	.get_function		= mxs_gpio_get_function,
};

static int mxs_gpio_probe(struct udevice *dev)
{
	struct mxs_gpio_priv *priv = dev_get_priv(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct mxs_gpio_platdata *pdata =
		(struct mxs_gpio_platdata *)dev_get_driver_data(dev);
	char name[18], *str;
	int ret;

	ret = dev_read_u32(dev, "reg", &priv->bank);
	if (ret) {
		printf("%s: No 'reg' property defined!\n", __func__);
		return ret;
	}

	sprintf(name, "GPIO%d_", priv->bank);
	str = strdup(name);
	if (!str)
		return -ENOMEM;

	uc_priv->bank_name = str;
	uc_priv->gpio_count = pdata->gpio_nr_of_bank_pins[priv->bank];

	return 0;
}

static const struct udevice_id mxs_gpio_ids[] = {
	{ .compatible = "fsl,imx28-gpio", .data = (ulong)&mxs_gpio_def },
	{ }
};

U_BOOT_DRIVER(gpio_mxs) = {
	.name	= "gpio_mxs",
	.id	= UCLASS_GPIO,
	.ops	= &gpio_mxs_ops,
	.probe	= mxs_gpio_probe,
	.priv_auto_alloc_size = sizeof(struct mxs_gpio_priv),
	.of_match = mxs_gpio_ids,
};
#endif
