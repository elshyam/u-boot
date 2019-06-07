/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Freescale i.MX28 GPIO
 *
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 */

#ifndef	__MX28_GPIO_H__
#define	__MX28_GPIO_H__

#ifdef	CONFIG_MXS_GPIO
void mxs_gpio_init(void);
#else
inline void mxs_gpio_init(void) {}
#endif

#if defined(CONFIG_MX28) && CONFIG_IS_ENABLED(DM_GPIO)
/*
 * According to i.MX28 Reference Manual:
 * 'i.MX28 Applications Processor Reference Manual, Rev. 1, 2010'
 * The i.MX28 has following number of GPIOs available:
 * Bank 0: 0-28 -> 29 PINS
 * Bank 1: 0-31 -> 32 PINS
 * Bank 2: 0-27 -> 28 PINS
 * Bank 3: 0-30 -> 31 PINS
 * Bank 4: 0-20 -> 21 PINS
 */
#define IMX28_GPIO_BANK0_PIN_NR 29
#define IMX28_GPIO_BANK1_PIN_NR 32
#define IMX28_GPIO_BANK2_PIN_NR 28
#define IMX28_GPIO_BANK3_PIN_NR 31
#define IMX28_GPIO_BANK4_PIN_NR 21
#define IMX28_GPIO_BANK_NR 5

struct mxs_gpio_platdata {
	u32 gpio_nr_of_bank_pins[IMX28_GPIO_BANK_NR];
	u32 gpio_base_nr[IMX28_GPIO_BANK_NR];
	u32 ngpio;
};

extern const struct mxs_gpio_platdata mxs_gpio_def;
#define IMX_GPIO_NR(port, index) (mxs_gpio_def.gpio_base_nr[(port)] + (index))
#endif

#endif	/* __MX28_GPIO_H__ */
