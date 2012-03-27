/* Dummy WDT driver for appliance watchdog.
 *   drivers/watchdog/appwd/wdt_dummy.c
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

#include "appwd_int.h"

#define DRV_NAME "appwd_wdt_dummy"


struct wdt_dummy_data {
	int heartbeat;
};


static void
wdt_dummy_keepalive(void * _data)
{
	pr_info("wdt_dummy_keepalive\n");
}

static struct wdt_operations wdt_dummy_ops = {
	.keepalive	= wdt_dummy_keepalive,
};

static int __init wdt_gpio_init(void)
{
	int err;
	struct wdt_dummy_data * data;

	pr_debug("initializing appwd wdt_dummy driver\n");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR "out of memory");
		return -ENOMEM;
	}

	wdt_dummy_keepalive(data);

	err = appwd_wdt_register(DRV_NAME, &wdt_dummy_ops, data->heartbeat,
				 data);
	if (err < 0) {
		printk(KERN_ERR "failed to register wdt_dummy: %d\n", err);
		goto appwd_wdt_register_failed;
	}

	return 0;

appwd_wdt_register_failed:
	kfree(data);
	return err;
}
device_initcall(wdt_gpio_init);

/*
 * Local Variables:
 * compile-command: "make -C ../.. M=drivers/watchdog/appwd"
 * End:
 */
