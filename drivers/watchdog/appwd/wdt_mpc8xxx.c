/* MPC8xx/MPC83xx/MPC86xx appliance watchdog.
 *   drivers/watchdog/appwd/mpc8xxx.c
 *
 * Copyright: Prevas A/S 2009-2012
 * Authors: Esben Haabendal <esben.haabendal@prevas.dk>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <sysdev/fsl_soc.h>

#include "appwd_int.h"

#define DRV_NAME "appwd_wdt_mpc8xxx"


struct mpc8xxx_wdt {
	__be32 res0;
	__be32 swcrr; /* System watchdog control register */
#define SWCRR_SWTC 0xFFFF0000 /* Software Watchdog Time Count. */
#define SWCRR_SWEN 0x00000004 /* Watchdog Enable bit. */
#define SWCRR_SWRI 0x00000002 /* Software Watchdog Reset/Interrupt Select bit.*/
#define SWCRR_SWPR 0x00000001 /* Software Watchdog Counter Prescale bit. */
	__be32 swcnr; /* System watchdog count register */
	u8 res1[2];
	__be16 swsrr; /* System watchdog service register */
	u8 res2[0xF0];
};
static struct mpc8xxx_wdt __iomem *wd_base;

struct mpc8xxx_wdt_type {
	int prescaler;
};

/*
 * We always prescale, but if someone really doesn't want to they can set this
 * to 0
 */
static int prescale = 1;
/*
 * Watchdog Interrupt/Reset Mode. 0 = interrupt, 1 = reset
 */
static int reset = 1;
static unsigned int timeout;
static DEFINE_SPINLOCK(wdt_spinlock);

struct wdt_mpc8xxx_data {
	int heartbeat;
	u32 timeout_ms;
};


static void wdt_mpc8xxx_keepalive(void * _data)
{
	/* Ping the WDT */
	spin_lock(&wdt_spinlock);
	out_be16(&wd_base->swsrr, 0x556c);
	out_be16(&wd_base->swsrr, 0xaa39);
	spin_unlock(&wdt_spinlock);
}

static struct wdt_operations wdt_mpc8xxx_ops = {
	.keepalive	= wdt_mpc8xxx_keepalive,
};


static int
wdt_mpc8xxx_probe(struct platform_device *pdev,
		  const struct of_device_id *match)
{

	struct device_node *np = pdev->dev.of_node;
	struct mpc8xxx_wdt_type *wdt_type = match->data;
	struct wdt_mpc8xxx_data * data;
	u32 freq = fsl_get_sys_freq();
	const u32 *val;
	int err;
	u32 tmp = SWCRR_SWEN;

	if (!freq || freq == -1)
		return -EINVAL;


	wd_base = of_iomap(np, 0);
	if (!wd_base)
		return -ENOMEM;

	if (in_be32(&wd_base->swcrr) & SWCRR_SWEN)
		pr_info("mpc8xxx_wdt: was previously enabled\n");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		pr_err("out of memory\n");
		err = -ENOMEM;
		goto err_unmap;
	}

	if (prescale)
		tmp |= SWCRR_SWPR;
	if (reset)
		tmp |= SWCRR_SWRI;


	val = of_get_property(np, "timeout", NULL);
	if (val) {
		pr_info("wdt_mpc8xxx timeout=%d\n", *val);
		data->timeout_ms = *val;
	}

	if (prescale)
		timeout = (data->timeout_ms * (freq / 1000)) / wdt_type->prescaler;
	else
		timeout = (data->timeout_ms * (freq / 1000));

	tmp |= timeout << 16;

	out_be32(&wd_base->swcrr, tmp);

	val = of_get_property(np, "heartbeat", NULL);
	if (val) {
		pr_info("wdt_mpc8xxx heartbeat=%d\n", *val);
		data->heartbeat = *val * HZ / 1000;
		if (data->heartbeat < 0) {
			pr_err("heartbeat delay must be at least 1 jiffy\n");
			err = -EINVAL;
			goto invalid_heartbeat;
		}
	}

	pr_info("WDT driver for MPC8xxx initialized. mode:%s timeout=%d "
		"(%d ms)\n", reset ? "reset" : "interrupt", timeout,
		data->timeout_ms);

	wdt_mpc8xxx_keepalive(data);

	err = appwd_wdt_register(DRV_NAME, &wdt_mpc8xxx_ops, data->heartbeat,
				data);
	if (err < 0) {
		pr_err("failed to register wdt_mpc8xxx: %d\n", err);
		goto appwd_wdt_register_failed;
	}

	return 0;

invalid_heartbeat:
appwd_wdt_register_failed:
	kfree(data);

err_unmap:
	iounmap(wd_base);
	wd_base = NULL;
	return err;
}

static const struct of_device_id wdt_mpc8xxx_match[] = {
	{
		.compatible = "appwd-mpc8xxx",
		.data = &(struct mpc8xxx_wdt_type) {
			.prescaler = 0x10000,
		},

	},
	{},
};
MODULE_DEVICE_TABLE(of, wdt_mpc8xxx_match);


static struct of_platform_driver wdt_mpc8xxx_driver = {
	.probe		= wdt_mpc8xxx_probe,
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= wdt_mpc8xxx_match,
	},
};

static int __init wdt_mpc8xxx_init(void)
{
	pr_info("initializing appwd_mpc8xxx driver\n");
	return of_register_platform_driver(&wdt_mpc8xxx_driver);
}
device_initcall(wdt_mpc8xxx_init);

/*
 * Local Variables:
 * compile-command: "make -C ../../.. M=drivers/watchdog/appwd wdt_mpc8xxx.o"
 * End:
 */
