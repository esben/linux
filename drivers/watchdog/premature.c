/* Premature watchdog support - keep watchdog alive during kernel init
 *
 * Copyright: DoréDevelopment ApS
 * Authors: Esben Haabendal <eha@doredevelopment.dk>
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
#include <linux/errno.h>
#include <linux/watchdog.h>

static premature_watchdog_reset reset_func = NULL;
static premature_watchdog_exit exit_func = NULL;

int
premature_watchdog_register(premature_watchdog_reset _reset_func,
			    premature_watchdog_exit _exit_func)
{
	if (reset_func != NULL) {
		printk(KERN_ERR "Premature watchdog keepalive already started\n");
		return -EBUSY;
	}
	reset_func = _reset_func;
	exit_func = _exit_func;
	(*reset_func)();
	printk(KERN_INFO "Premature watchdog keepalive started\n");
	return 0;
}

void
premature_watchdog_settle(void)
{
	(*reset_func)();
	printk(KERN_INFO "Premature watchdog keepalive stopped\n");
	(*exit_func)();
	reset_func = NULL;
	exit_func = NULL;
}

void
premature_watchdog_keepalive(void)
{
	if (reset_func != NULL) {
		(*reset_func)();
	}
}

/*
 * Local Variables:
 * compile-command: "make -C ../../.. M=drivers/watchdog/appwd wdm.o"
 * End:
 */
