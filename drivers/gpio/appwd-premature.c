/* Premature watchdog support for apliance watchdog GPIO WDT driver
 *
 * Copyright: Prevas A/S 2014
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
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/watchdog.h>

struct wdt_gpio_premature {
	unsigned gpio;
	int level;
};

static struct wdt_gpio_premature data = { -1, 0 };

static void wdt_gpio_premature_reset(void)
{
	if (!gpio_is_valid(data.gpio))
		return;
	data.level = data.level ? 0 : 1;
	gpio_set_value_cansleep(data.gpio, data.level);
}

static int __init wdt_gpio_premature_init(void)
{
	struct device_node *node;
	int err;

	for_each_compatible_node(node, NULL, "appwd-wdt-gpio") {
		if (!of_get_property(node, "premature-keepalive", NULL))
			continue;

		data.gpio = of_get_gpio(node, 0);
		pr_debug("%s: node=%s %d\n", __func__, node->full_name, data.gpio);
		if (!gpio_is_valid(data.gpio)) {
			pr_err("%s: gpio is not valid: %d\n", __func__, data.gpio);
			continue;
		}
		err = premature_watchdog_register(wdt_gpio_premature_reset,
						  NULL);
		if (err) {
			pr_err("%s: premature_watchdog_register failed: %d\n",
			       __func__, err);
			data.gpio = -1;
			continue;
		}
	}
	return 0;
}
arch_initcall(wdt_gpio_premature_init);
