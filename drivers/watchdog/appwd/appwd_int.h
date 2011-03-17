/* Appliance Watchdog - Internal API
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

#ifndef _APPWD_INT_H

/* Common workqueue task used for wdm and wdd state-machines */
extern struct workqueue_struct * appwd_workq;

/* Watchdog Monitor */
void wdd_timeout(struct work_struct *);

/* Watchdog Timer */
struct wdt_operations {
	void (*keepalive)(void *);
};

/* Watchdog Device */
struct wdd_config {
	char				name[12];
	unsigned int			init_timeout;
	unsigned int			keepalive_timeout;
	unsigned int			restart_timeout;
	unsigned int			recover_timeout;
	int				nowayout;
};
int __devinit wdd_register(struct wdd_config *);
void __devinit wdd_deregister(void *);
void wdd_init_start(void);
int appwd_wdt_register(const char *, const struct wdt_operations *,
		       unsigned int, void *);

#endif /* _APPWD_INT_H */
