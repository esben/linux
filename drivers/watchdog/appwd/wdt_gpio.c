/* GPIO based WDT driver for appliance watchdog.
 *   drivers/watchdog/appwd/wdt_gpio.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "appwd_int.h"

#define DRV_NAME "appwd_wdt_gpio"


struct wdt_gpio_data {
	unsigned gpio;
	int level;
	int heartbeat;
};


static void
wdt_gpio_keepalive(void * _data)
{
	struct wdt_gpio_data * data = _data;
	data->level = data->level ? 0 : 1;
	gpio_set_value_cansleep(data->gpio, data->level);
}

static struct wdt_operations wdt_gpio_ops = {
	.keepalive	= wdt_gpio_keepalive,
};

static int __init
wdt_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int err;
	struct wdt_gpio_data * data;
	enum of_gpio_flags flags;
	const u32 * val;

	pr_debug("%s\n", __func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR "out of memory");
		return -ENOMEM;
	}
	data->gpio = of_get_gpio_flags(np, 0, &flags);

	val = of_get_property(np, "heartbeat", NULL);
	if (val) {
		dev_info(&pdev->dev, "heartbeat=%d\n", *val);
		data->heartbeat = *val * HZ / 1000;
		if (data->heartbeat < 0) {
			printk(KERN_ERR "heartbeat delay must be at least 1 jiffy\n");
			err = -EINVAL;
			goto invalid_heartbeat;
		}
	}

	if (!gpio_is_valid(data->gpio)) {
		printk(KERN_ERR "invalid gpio: %d\n", data->gpio);
		err = -EINVAL;
		goto invalid_gpio;
	}

	err = gpio_request(data->gpio, DRV_NAME);
	if (err < 0) {
		printk(KERN_ERR "failed to request gpio %d: %d\n",
		       data->gpio, err);
		goto gpio_request_failed;
	}

	err = gpio_direction_output(data->gpio, 0);
	if (err < 0) {
		printk(KERN_ERR "failed to set gpio %d as output: %d\n",
		       data->gpio, err);
		goto gpio_direction_output_failed;
	}

	udelay(5);
	wdt_gpio_keepalive(data);

	err = appwd_wdt_register(DRV_NAME, &wdt_gpio_ops, data->heartbeat,
				 data);
	if (err < 0) {
		printk(KERN_ERR "failed to register wdt_gpio: %d\n", err);
		goto appwd_wdt_register_failed;
	}

	dev_set_drvdata(&pdev->dev, data);

	return 0;

appwd_wdt_register_failed:

gpio_direction_output_failed:

	gpio_free(data->gpio);
gpio_request_failed:

invalid_gpio:
invalid_heartbeat:

	kfree(data);
	return err;
}


static const struct of_device_id wdt_gpio_match[] = {
	{
		.compatible = "appwd-wdt-gpio",
	},
	{},
};
MODULE_DEVICE_TABLE(of, wdt_gpio_match);


static struct platform_driver wdt_gpio_driver __initdata = {
	.probe		= wdt_gpio_probe,
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= wdt_gpio_match,
	},
};


static int __init wdt_gpio_init(void)
{
	pr_debug("initializing appwd wdt_gpio driver\n");
	return platform_driver_register(&wdt_gpio_driver);
}
device_initcall(wdt_gpio_init);

/*
 * Local Variables:
 * compile-command: "make -C ../.. M=drivers/watchdog/appwd"
 * End:
 */
