// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (C) 2018 Amarula Solutions B.V.
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <dt-bindings/clock/imx6ul-clock.h>

struct imx6ul_clk_priv {
	void *base;
};

static ulong imx6ul_clk_get_rate(struct clk *clk)
{
	debug("%s(#%ld)\n", __func__, clk->id);

	debug("  unhandled\n");
	return -EINVAL;
}

static ulong imx6ul_clk_set_rate(struct clk *clk, ulong rate)
{
	debug("%s(#%ld)\n", __func__, clk->id);

	debug("  unhandled\n");
	return -EINVAL;
}

static int imx6ul_clk_enable(struct clk *clk)
{
	struct imx6ul_clk_priv *priv = dev_get_priv(clk->dev);

	debug("%s(#%ld)\n", __func__, clk->id);

	switch (clk->id) {
	case IMX6UL_CLK_ENET_PTP:
		/* ref clock handling done via IMX6UL_CLK_ENET_REF */
		clk = NULL;
		return 0;
	case IMX6UL_CLK_ENET_REF:
		return enable_fec_anatop_clock(0, ENET_50MHZ);
	case IMX6UL_CLK_ENET2_REF_125M:
		return enable_fec_anatop_clock(1, ENET_50MHZ);
	case IMX6UL_CLK_ENET_AHB:
		setbits_le32(priv->base + 0x74, GENMASK(5, 4));
		return 0;
	default:
		printf("  unhandled\n");
		return -ENODEV;
	}
}

static struct clk_ops imx6ul_clk_ops = {
	.get_rate = imx6ul_clk_get_rate,
	.set_rate = imx6ul_clk_set_rate,
	.enable = imx6ul_clk_enable,
};

static int imx6ul_clk_probe(struct udevice *dev)
{
	return 0;
}

static int imx6ul_clk_ofdata_to_platdata(struct udevice *dev)
{
	struct imx6ul_clk_priv *priv = dev_get_priv(dev);

	priv->base = dev_read_addr_ptr(dev);

	return 0;
}

static const struct udevice_id imx6ul_clk_ids[] = {
	{ .compatible = "fsl,imx6ul-ccm" },
	{ }
};

U_BOOT_DRIVER(fsl_imx6ul_ccm) = {
	.name		= "fsl_imx6ul_ccm",
	.id		= UCLASS_CLK,
	.of_match	= imx6ul_clk_ids,
	.priv_auto_alloc_size	= sizeof(struct imx6ul_clk_priv),
	.ofdata_to_platdata	= imx6ul_clk_ofdata_to_platdata,
	.ops		= &imx6ul_clk_ops,
	.probe		= imx6ul_clk_probe,
};
