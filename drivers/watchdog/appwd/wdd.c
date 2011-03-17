/* Appliance Watchdog - Watchdog Device driver
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
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/watchdog.h>
#include <linux/uaccess.h>

#include <linux/appwd.h>
#include "appwd_int.h"

#define DRV_NAME "appwd_wdd"


#define wdd_emerg(wdd, format, arg...) \
	dev_emerg(wdd->miscdev.this_device, format, ## arg)
#define wdd_alert(wdd, format, arg...) \
	dev_alert(wdd->miscdev.this_device, format, ## arg)
#define wdd_crit(wdd, format, arg...) \
	dev_crit(wdd->miscdev.this_device, format, ## arg)
#define wdd_err(wdd, format, arg...) \
	dev_err(wdd->miscdev.this_device, format, ## arg)
#define wdd_warn(wdd, format, arg...) \
	dev_warn(wdd->miscdev.this_device, format, ## arg)
#define wdd_notice(wdd, format, arg...) \
	dev_notice(wdd->miscdev.this_device, format, ## arg)
#define wdd_info(wdd, format, arg...) \
	dev_info(wdd->miscdev.this_device, format, ## arg)
#define wdd_dbg(wdd, format, arg...) \
	dev_dbg(wdd->miscdev.this_device, format, ## arg)


/* Watchdog Device states */
enum wdd_state {
	WDD_STATE_INIT = 0,
	WDD_STATE_READY,
	WDD_STATE_ACTIVE,
	WDD_STATE_MAGIC,
	WDD_STATE_LATE,
	WDD_STATE_RESTART,
	WDD_STATE_DYING,
	WDD_STATE_RECOVER,
	WDD_STATE_DEAD,
};


/* Watchdog Device private data */
struct wdd_private {
	/* Boot-time configuration */
	const struct wdd_config *	config;

	/* Run-time configuration values */
	unsigned int			init_timeout;
	unsigned int			keepalive_timeout;
	unsigned int			restart_timeout;
	unsigned int			recover_timeout;

	/* This spinlock must be held in all file_operation functions,
	 * thus ensuring atomic file_operations */
	spinlock_t			open_lock;

	/* Enforce exclusive access */
	int				is_open;

	/* State-machine implemented on top of a workqueue, with
	 * events implemented as works */
	enum wdd_state			state;
	struct work_struct		open_event;
	struct work_struct		close_event;
	struct work_struct		keepalive_event;
	struct work_struct		magic_event;
	struct delayed_work		init_timeout_event;
	struct delayed_work		keepalive_timeout_event;
	struct delayed_work		restart_timeout_event;
	struct delayed_work		recover_timeout_event;

	/* Events to send to the Watchdog Monitor state-machine */
	struct work_struct		app_fail_event;

	/* The user-space watchdog device interface */
	struct miscdevice		miscdev;

	/* PID of the current user-space process owning the watchdog device */
	struct pid *			pid;

	/* List of all the watchdog devices */
	struct list_head		list;

	/* Flags returned by WDIOC_GETBOOTSTATUS */
	int				bootstatus_flags;
	/* Flags returned by WDIOC_GETSTATUS */
	int				status_flags;
};


LIST_HEAD(wdd_list);


static int wdd_open(struct inode *, struct file *);
static int wdd_close(struct inode *, struct file *);
static ssize_t wdd_write(struct file *, const char __user *, size_t, loff_t *);
static long wdd_ioctl(struct file *, unsigned, unsigned long);

static struct file_operations wdd_fops = {
	.owner		= THIS_MODULE,
	.open		= wdd_open,
	.release	= wdd_close,
	.write		= wdd_write,
	.unlocked_ioctl	= wdd_ioctl,
	.llseek		= no_llseek,
};

static void
init_timeout_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(container_of(work, struct delayed_work, work),
			     struct wdd_private, init_timeout_event);

	wdd_dbg(wdd, "init_timeout_event\n");
	switch (wdd->state) {

	case WDD_STATE_INIT:
		wdd_crit(wdd, "init timeout!\n");
		wdd->state = WDD_STATE_DEAD;
		/* Send app_init_timeout event to Watchdog Monitor */
		queue_work(appwd_workq, &wdd->app_fail_event);
		break;

	case WDD_STATE_READY:
	case WDD_STATE_ACTIVE:
	case WDD_STATE_MAGIC:
	case WDD_STATE_LATE:
	case WDD_STATE_RESTART:
	case WDD_STATE_DYING:
	case WDD_STATE_RECOVER:
	case WDD_STATE_DEAD:
		wdd_err(wdd, "init_timeout in invalid state: %d\n", wdd->state);
		break;
	}
}


static void
open_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(work, struct wdd_private, open_event);

	wdd_dbg(wdd, "open_event\n");
	switch (wdd->state) {

	case WDD_STATE_INIT:
		wdd->state = WDD_STATE_ACTIVE;
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_READY:
		wdd->state = WDD_STATE_ACTIVE;
		cancel_delayed_work_sync(&wdd->keepalive_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_RESTART:
		wdd_info(wdd, "restart success\n");
		wdd->state = WDD_STATE_ACTIVE;
		cancel_delayed_work_sync(&wdd->restart_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_RECOVER:
		wdd_info(wdd, "recover success\n");
		wdd->state = WDD_STATE_ACTIVE;
		cancel_delayed_work_sync(&wdd->recover_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_ACTIVE:
	case WDD_STATE_MAGIC:
	case WDD_STATE_LATE:
	case WDD_STATE_DYING:
	case WDD_STATE_DEAD:
		wdd_err(wdd, "open_event in invalid state: %d\n", wdd->state);
		break;
	}
}


static void
close_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(work, struct wdd_private, close_event);

	wdd_dbg(wdd, "close_event\n");
	switch (wdd->state) {

	case WDD_STATE_ACTIVE:
		wdd_warn(wdd, "closed: recover timeout in %u ms\n",
			 wdd->recover_timeout * 1000 / HZ);
		wdd->state = WDD_STATE_RECOVER;
		cancel_delayed_work(&wdd->keepalive_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->recover_timeout_event,
				   wdd->recover_timeout);
		break;

	case WDD_STATE_MAGIC:
		wdd_warn(wdd, "closed with magic\n");
		wdd->state = WDD_STATE_READY;
		cancel_delayed_work(&wdd->keepalive_timeout_event);
		break;

	case WDD_STATE_LATE:
		wdd->state = WDD_STATE_RESTART;
		break;

	case WDD_STATE_DYING:
		wdd->state = WDD_STATE_RECOVER;
		break;

	case WDD_STATE_INIT:
	case WDD_STATE_READY:
	case WDD_STATE_RESTART:
	case WDD_STATE_RECOVER:
	case WDD_STATE_DEAD:
		wdd_err(wdd, "close_event in invalid state: %d\n", wdd->state);
		break;
	}
}


static void
keepalive_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(work, struct wdd_private, keepalive_event);

	wdd_dbg(wdd, "keepalive_event\n");
	switch (wdd->state) {

	case WDD_STATE_ACTIVE:
		cancel_delayed_work_sync(&wdd->keepalive_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_MAGIC:
		wdd->state = WDD_STATE_ACTIVE;
		cancel_delayed_work_sync(&wdd->keepalive_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_LATE:
	case WDD_STATE_DYING:
	case WDD_STATE_DEAD:
		wdd_dbg(wdd, "Late keepalive_event received in state: %d\n",
			wdd->state);
		break;

	case WDD_STATE_INIT:
	case WDD_STATE_READY:
	case WDD_STATE_RESTART:
	case WDD_STATE_RECOVER:
		wdd_err(wdd, "keepalive_event in invalid state: %d\n",
			wdd->state);
		break;
	}
}


static void
magic_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(work, struct wdd_private, magic_event);

	wdd_dbg(wdd, "magic_event\n");
	switch (wdd->state) {

	case WDD_STATE_ACTIVE:
		wdd->state = WDD_STATE_MAGIC;
		cancel_delayed_work_sync(&wdd->keepalive_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_MAGIC:
		cancel_delayed_work_sync(&wdd->keepalive_timeout_event);
		queue_delayed_work(appwd_workq, &wdd->keepalive_timeout_event,
				   wdd->keepalive_timeout);
		break;

	case WDD_STATE_LATE:
	case WDD_STATE_DYING:
	case WDD_STATE_DEAD:
		wdd_dbg(wdd, "Late magic_event received in state: %d\n",
			wdd->state);
		break;

	case WDD_STATE_INIT:
	case WDD_STATE_READY:
	case WDD_STATE_RESTART:
	case WDD_STATE_RECOVER:
		wdd_err(wdd, "magic_event in invalid state: %d\n",
			wdd->state);
		break;
	}
}


static void
keepalive_timeout_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(container_of(work, struct delayed_work, work),
			     struct wdd_private, keepalive_timeout_event);

	wdd_dbg(wdd, "keepalive_timeout_event\n");
	switch (wdd->state) {

	case WDD_STATE_ACTIVE:
	case WDD_STATE_MAGIC:
		wdd_warn(wdd, "keepalive timeout: restart timeout in %u ms\n",
			 wdd->restart_timeout * 1000 / HZ);
		wdd->state = WDD_STATE_LATE;
		kill_pid(wdd->pid, SIGHUP, 1);
		queue_delayed_work(appwd_workq, &wdd->restart_timeout_event,
				   wdd->restart_timeout);
		break;

	case WDD_STATE_INIT:
	case WDD_STATE_READY:
	case WDD_STATE_LATE:
	case WDD_STATE_RESTART:
	case WDD_STATE_DYING:
	case WDD_STATE_RECOVER:
	case WDD_STATE_DEAD:
		wdd_err(wdd, "keepalive_timeout_event in invalid state: %d\n",
			wdd->state);
		break;
	}
}


static void
restart_timeout_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(container_of(work, struct delayed_work, work),
			     struct wdd_private, restart_timeout_event);

	wdd_dbg(wdd, "restart_timeout_event\n");
	switch (wdd->state) {

	case WDD_STATE_LATE:
		wdd_warn(wdd, "restart timeout: recover timeout in %u ms\n",
			 wdd->recover_timeout * 1000 / HZ);
		wdd->state = WDD_STATE_DYING;
		kill_pid(wdd->pid, SIGKILL, 1);
		queue_delayed_work(appwd_workq, &wdd->recover_timeout_event,
				   wdd->recover_timeout);
		break;

	case WDD_STATE_RESTART:
		wdd_warn(wdd, "restart timeout: recover timeout in %u ms\n",
			 wdd->recover_timeout * 1000 / HZ);
		wdd->state = WDD_STATE_RECOVER;
		kill_pid(wdd->pid, SIGKILL, 1);
		queue_delayed_work(appwd_workq, &wdd->recover_timeout_event,
				   wdd->recover_timeout);
		break;

	case WDD_STATE_INIT:
	case WDD_STATE_READY:
	case WDD_STATE_ACTIVE:
	case WDD_STATE_MAGIC:
	case WDD_STATE_DYING:
	case WDD_STATE_RECOVER:
	case WDD_STATE_DEAD:
		wdd_err(wdd, "restart_timeout_event in invalid state: %d\n",
			wdd->state);
		break;
	}
}


static void
recover_timeout_event(struct work_struct * work)
{
	struct wdd_private * wdd =
		container_of(container_of(work, struct delayed_work, work),
			     struct wdd_private, recover_timeout_event);

	wdd_dbg(wdd, "recover_timeout_event\n");
	switch (wdd->state) {

	case WDD_STATE_DYING:
	case WDD_STATE_RECOVER:
		wdd_crit(wdd, "recover timeout!\n");
		wdd->state = WDD_STATE_DEAD;
		queue_work(appwd_workq, &wdd->app_fail_event);
		break;

	case WDD_STATE_INIT:
	case WDD_STATE_READY:
	case WDD_STATE_ACTIVE:
	case WDD_STATE_MAGIC:
	case WDD_STATE_LATE:
	case WDD_STATE_RESTART:
	case WDD_STATE_DEAD:
		wdd_err(wdd, "recover_timeout_event in invalid state: %d\n",
			wdd->state);
		break;
	}
}

int __devinit
wdd_register(struct wdd_config *config)
{
	int err;
	struct wdd_private * wdd;

	pr_debug("wdd_register %s\n", config->name);

	wdd = kzalloc(sizeof(*wdd), GFP_KERNEL);
	if (wdd == NULL) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	spin_lock_init(&wdd->open_lock);

	wdd->config = config;
	wdd->init_timeout = config->init_timeout;
	wdd->keepalive_timeout = config->keepalive_timeout;
	wdd->restart_timeout = config->restart_timeout;
	wdd->recover_timeout = config->recover_timeout;

	INIT_WORK(&wdd->open_event, open_event);
	INIT_WORK(&wdd->close_event, close_event);
	INIT_WORK(&wdd->keepalive_event, keepalive_event);
	INIT_WORK(&wdd->magic_event, magic_event);
	INIT_DELAYED_WORK(&wdd->init_timeout_event,
			  init_timeout_event);
	INIT_DELAYED_WORK(&wdd->keepalive_timeout_event,
			  keepalive_timeout_event);
	INIT_DELAYED_WORK(&wdd->restart_timeout_event,
			  restart_timeout_event);
	INIT_DELAYED_WORK(&wdd->recover_timeout_event,
			  recover_timeout_event);
	INIT_WORK(&wdd->app_fail_event, wdd_timeout);

	wdd->miscdev.name = config->name;
	wdd->miscdev.fops = &wdd_fops;
	wdd->miscdev.minor = MISC_DYNAMIC_MINOR;
	err = misc_register(&wdd->miscdev);
	if (err) {
		pr_err("misc_register failed: %d\n", err);
		goto fail_misc_register;
	}
	wdd_dbg(wdd, "misc_register wdd_list=%p wdd=%p minor=%u\n",
		&wdd_list, wdd, wdd->miscdev.minor);

	INIT_LIST_HEAD(&wdd->list);
	list_add_tail(&wdd->list, &wdd_list);

	return 0;

	misc_deregister(&wdd->miscdev);
fail_misc_register:
	kfree(wdd);

	return err;
}


/* FIXME: cleanup!
void __init
wdd_deregister(void * _wdd)
{
	struct wdd_private * wdd = (struct wdd_private *)_wdd;
	misc_deregister(&wdd->miscdev);
}
*/


static int
wdd_open(struct inode * inode, struct file * filp)
{
	struct wdd_private * wdd;
	int err;

	/* Find the wdd pointer for the inode */
	err = -ENODEV;
	list_for_each_entry(wdd, &wdd_list, list) {
		if (wdd->miscdev.minor == iminor(inode)) {
			err = 0;
			break;
		}
	}
	if (err < 0)
		return -ENODEV;

	spin_lock(&wdd->open_lock);

	/* Enforce exclusive access */
	if (wdd->is_open) {
		err = -EBUSY;
		goto out;
	}
	wdd->is_open = 1;

	/* Cancel init_timeout_event if pending */
	if (wdd->init_timeout)
		cancel_delayed_work(&wdd->init_timeout_event);

	/* Send open_event to state-machine */
	if (unlikely(queue_work(appwd_workq, &wdd->open_event) == 0)) {
		/* And handle the unlikely race-conditions where
		 * previous open_event(s) are not handled yet */
		do {
			flush_workqueue(appwd_workq);
		} while (queue_work(appwd_workq, &wdd->open_event) == 0);
	}

	wdd->pid = get_pid(task_pid(current));
	filp->private_data = wdd;
	err = nonseekable_open(inode, filp);

out:
	spin_unlock(&wdd->open_lock);
	return err;
}


static int
wdd_close(struct inode * inode, struct file * filp)
{
	struct wdd_private * wdd = filp->private_data;

	spin_lock(&wdd->open_lock);

	/* Send close_event to state-machine */
	if (unlikely(queue_work(appwd_workq, &wdd->close_event) == 0)) {
		/* And handle the unlikely race-conditions where
		 * previous close_event(s) are not handled yet */
		do {
			flush_workqueue(appwd_workq);
		} while (queue_work(appwd_workq, &wdd->close_event) == 0);
	}

	wdd->is_open = 0;
	spin_unlock(&wdd->open_lock);

	return 0;
}


static void
wdd_keepalive(struct wdd_private * wdd, int magic)
{
	wdd->status_flags |= WDIOF_KEEPALIVEPING;

	if (magic && !wdd->config->nowayout)
		queue_work(appwd_workq, &wdd->magic_event);
	else
		queue_work(appwd_workq, &wdd->keepalive_event);
}


static ssize_t
wdd_write(struct file * filp, const char __user * buf,
	  size_t count, loff_t * offp)
{
	struct wdd_private * wdd = filp->private_data;
	int err=0;

	if (count) {
		size_t i;
		int magic = 0;

		for (i=0; i!=count; i++) {
			char c;

			if (get_user(c, buf + i)) {
				err = -EFAULT;
				break;
			}

			if (c == 'V') {
				magic = 1;
				break;
			}
		}

		wdd_keepalive(wdd, magic);
	}

	return err ? err : count;
}


static long
wdd_ioctl(struct file * filp, unsigned cmd, unsigned long arg)
{
	struct wdd_private *wdd = filp->private_data;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	long err = 0;
	unsigned msecdiv = 1;

	static struct watchdog_info ident = {
		.options =              WDIOF_SETTIMEOUT |
					WDIOF_KEEPALIVEPING |
					WDIOF_MAGICCLOSE,
		.firmware_version =     0,
		.identity =             "Appliance Watchdog"
	};

	if (cmd == WDIOC_SETTIMEOUTMSEC ||
	    cmd == WDIOC_GETTIMEOUTMSEC ||
	    cmd == WDIOC_SETRESTARTTIMEOUTMSEC ||
	    cmd == WDIOC_GETRESTARTTIMEOUTMSEC ||
	    cmd == WDIOC_SETRECOVERTIMEOUTMSEC ||
	    cmd == WDIOC_GETRECOVERTIMEOUTMSEC)
		msecdiv = 1000;

	switch (cmd) {

	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS: {
		int status = wdd->status_flags;
		wdd->status_flags &= ~WDIOF_KEEPALIVEPING;
		err = put_user(status, p);
		break;
	}

	case WDIOC_GETBOOTSTATUS:
		err = put_user(wdd->bootstatus_flags, p);
		break;

	case WDIOC_KEEPALIVE:
		wdd_keepalive(wdd, 0);
		break;

	case WDIOC_SETTIMEOUT:
	case WDIOC_SETTIMEOUTMSEC: {
		int new_timeout;
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			break;
		}
		if (get_user(new_timeout, p)) {
			err = -EFAULT;
			break;
		}
		wdd->keepalive_timeout = (new_timeout * HZ) / msecdiv;
		/* Fall through to WDIOC_GETTIMEOUT */
	}

	case WDIOC_GETTIMEOUT:
	case WDIOC_GETTIMEOUTMSEC:
		err = put_user(((wdd->keepalive_timeout * msecdiv) / HZ), p);
		break;

		case WDIOC_SETRESTARTTIMEOUT:
		case WDIOC_SETRESTARTTIMEOUTMSEC: {
		int new_timeout;
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			break;
		}
		if (get_user(new_timeout, p)) {
			err = -EFAULT;
			break;
		}
		wdd->restart_timeout = (new_timeout * HZ) / msecdiv;
		/* Fall through to WDIOC_GETRESTARTTIMEOUT */
	}

	case WDIOC_GETRESTARTTIMEOUT:
	case WDIOC_GETRESTARTTIMEOUTMSEC:
		err = put_user(((wdd->restart_timeout * msecdiv) / HZ), p);
		break;

	case WDIOC_SETRECOVERTIMEOUT:
	case WDIOC_SETRECOVERTIMEOUTMSEC: {
		int new_timeout;
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			break;
		}
		if (get_user(new_timeout, p)) {
			err = -EFAULT;
			break;
		}
		wdd->recover_timeout = (new_timeout * HZ) / msecdiv;
		/* Fall through to WDIOC_GETRECOVERTIMEOUT */
	}

	case WDIOC_GETRECOVERTIMEOUT:
	case WDIOC_GETRECOVERTIMEOUTMSEC:
		err = put_user(((wdd->recover_timeout * msecdiv) / HZ), p);
		break;

	case WDIOC_SETOPTIONS:
		break;

	default:
		wdd_dbg(wdd, "unsupported ioctl: %08x", cmd);
		err = -ENOTTY;
		break;
	}

	return err;
}


void
wdd_init_start(void)
{
	struct wdd_private * wdd;

	list_for_each_entry(wdd, &wdd_list, list) {
		if (wdd->init_timeout) {
			wdd_info(wdd, "init timeout in %u ms\n",
				 wdd->init_timeout * 1000 / HZ);
			queue_delayed_work(appwd_workq,
					   &wdd->init_timeout_event,
					   wdd->init_timeout);
		}
	}
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Esben Haabendal <eha@doredevelopment.dk>");
MODULE_DESCRIPTION("Appliance Watchdog Device");
MODULE_VERSION("0.1");
/*
 * Local Variables:
 * compile-command: "make -C ../../.. M=drivers/watchdog/appwd wdd.o"
 * End:
 */
