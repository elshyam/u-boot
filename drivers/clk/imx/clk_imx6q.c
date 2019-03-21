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
#include <dt-bindings/clock/imx6qdl-clock.h>

struct imx6q_clk_priv {
	void *base;
};

static ulong imx6q_clk_get_rate(struct clk *clk)
{
	debug("%s(#%ld)\n", __func__, clk->id);

	debug("  unhandled\n");
	return -EINVAL;
}

static ulong imx6q_clk_set_rate(struct clk *clk, ulong rate)
{
	debug("%s(#%ld, rate: %lu)\n", __func__, clk->id, rate);

	debug("  unhandled\n");
	return -EINVAL;
}

static int imx6q_clk_enable(struct clk *clk)
{
	debug("%s(#%ld)\n", __func__, clk->id);

	debug("  unhandled\n");
	return -EINVAL;
}

static struct clk_ops imx6q_clk_ops = {
	.get_rate = imx6q_clk_get_rate,
	.set_rate = imx6q_clk_set_rate,
	.enable = imx6q_clk_enable,
};

static int imx6q_clk_probe(struct udevice *dev)
{
	return 0;
}

static int imx6q_clk_ofdata_to_platdata(struct udevice *dev)
{
	struct imx6q_clk_priv *priv = dev_get_priv(dev);

	priv->base = dev_read_addr_ptr(dev);

	return 0;
}

static const struct udevice_id imx6q_clk_ids[] = {
	{ .compatible = "fsl,imx6q-ccm" },
	{ }
};

U_BOOT_DRIVER(fsl_imx6q_ccm) = {
	.name		= "fsl_imx6q_ccm",
	.id		= UCLASS_CLK,
	.of_match	= imx6q_clk_ids,
	.priv_auto_alloc_size	= sizeof(struct imx6q_clk_priv),
	.ofdata_to_platdata	= imx6q_clk_ofdata_to_platdata,
	.ops		= &imx6q_clk_ops,
	.probe		= imx6q_clk_probe,
};
