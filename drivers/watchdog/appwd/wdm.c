/* Appliance Watchdog - Watchdog Monitor
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
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>

#ifdef CONFIG_MPC8xxx_RSTE
#include <linux/mpc8xxx_rste.h>
#else
#define mpc8xxx_rste_cause(arg) do {} while (0)
#endif

#include <linux/appwd.h>
#include "appwd_int.h"

#define DRV_NAME "appwd_wdm"


/* Watchdog Monitor states */
enum wdm_state {
	WDM_STATE_BOOT = 0,
	WDM_STATE_ACTIVE,
	WDM_STATE_REBOOT,
	WDM_STATE_ZOMBIE,
};


/* Watchdog Timer private data */
struct wdm_private_wdt {
	const char *			name;
	struct wdt_operations		ops;
	unsigned int			heartbeat_delay;
	struct delayed_kthread_work	heartbeat;
	void *				data;
};


struct wdm_private_wdd {
	struct wdd_config		config;
};

/* Watchdog Monitor private data */
struct wdm_private {
	unsigned int			boot_timeout;
	unsigned int			reboot_timeout;

	/* The state-machine is implemented on top of a workqueue,
	 * with events implemented as work. */
	enum wdm_state			state;
	struct kthread_work		boot_done_event;
	struct delayed_kthread_work	boot_timeout_event;
	struct delayed_kthread_work	reboot_timeout_event;
	struct kthread_work		wdd_timeout_event;
	struct kthread_work		system_down_event;

	unsigned int			num_wdd;
	struct wdm_private_wdd		wdd[];
};

static struct wdm_private *		wdm = NULL;
static struct wdm_private_wdt *		wdt = NULL;
struct kthread_worker			worker;
struct task_struct *			worker_task = NULL;

static void
queue_heartbeat(struct wdm_private_wdt * wdt)
{
	queue_delayed_kthread_work(
		&worker, &wdt->heartbeat, wdt->heartbeat_delay);
}


static void
wdt_heartbeat(struct kthread_work * work)
{
	struct wdm_private_wdt * wdt =
		container_of(container_of(work, struct delayed_kthread_work,
					  work),
			     struct wdm_private_wdt, heartbeat);

#ifdef CONFIG_PREMATURE_WATCHDOG
	static int premature_active = 1;
	/* Stop premature watchdog keepalive (if enabled) */
	if (premature_active) {
		premature_watchdog_settle();
		premature_active = 0;
	}
#endif

	/* Early heartbeats where monitor is not initialized yet */
	if (wdm == NULL) {
		wdt->ops.keepalive(wdt->data);
		queue_heartbeat(wdt);
		return;
	}

	switch (wdm->state) {

	case WDM_STATE_BOOT:
	case WDM_STATE_ACTIVE:
	case WDM_STATE_REBOOT:
		wdt->ops.keepalive(wdt->data);
		queue_heartbeat(wdt);
		break;

	case WDM_STATE_ZOMBIE:
		/* Don't feed a zombie */
		break;

	}
}


static int
init_appwd_worker(void)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 10 };

	if (worker_task != NULL)
		return 0;

	/* Setup work queue */
	init_kthread_worker(&worker);
	worker_task = kthread_run(kthread_worker_fn, &worker, "appwd");
	if (IS_ERR(worker_task)) {
		pr_err("creating appwd worker failed: %ld\n",
		       PTR_ERR(worker_task));
		return PTR_ERR(worker_task);
	}

	/* Use FIFO scheduler as this is naturally realtime sensitive */
	sched_setscheduler(worker_task, SCHED_FIFO, &param);

	return 0;
}

static void
stop_appwd_worker(void)
{
	if (worker_task == NULL)
		return;

	kthread_stop(worker_task);
}

int
appwd_wdt_register(const char * name, const struct wdt_operations * ops,
		   unsigned int heartbeat_delay, void * data)
{
	int err, i;

	if (name == NULL || ops == NULL || heartbeat_delay == 0 ||
	    ops->keepalive == NULL) {
		pr_warning("invalid arguments\n");
		return -EINVAL;
	}

	/* Let's pet the dog in a hurry */
	ops->keepalive(data);

	if (wdt == NULL) {
		wdt = kzalloc(sizeof(*wdt) * CONFIG_APPWD_MAX_WDT,
			      GFP_KERNEL);
		if (wdt == NULL) {
			pr_err("Out of memory\n");
			return -ENOMEM;
		}
	}

	/* Setup work queue */
	err = init_appwd_worker();
	if (err)
		goto create_worker_failed;

	for (i=0 ; i < CONFIG_APPWD_MAX_WDT ; i++) {
		if (wdt[i].name != NULL)
			continue;

		wdt[i].name = name;
		memcpy(&wdt[i].ops, ops, sizeof(ops));
		wdt[i].heartbeat_delay = heartbeat_delay;
		wdt[i].data = data;
		init_delayed_kthread_work(&wdt[i].heartbeat, wdt_heartbeat);
		break;
	}

	if (i == CONFIG_APPWD_MAX_WDT) {
		pr_warning("out of wdt slots, increase CONFIG_APPWD_WDM_MAX_WDT\n");
		return -EBUSY;
	}

	pr_info("appwd: registered %s with heartbeat_delay %u ms as wdt[%d]\n",
		name, heartbeat_delay * 1000 / HZ, i);

	/* Start the WDT heartbeat */
	queue_heartbeat(&wdt[i]);

	return 0;

create_worker_failed:
	kfree(wdt);
	wdt = NULL;

	return err;
}


/* Notifier for system down */
static int wdm_reboot_notice(struct notifier_block *this, unsigned long code,
			     void *unused)
{
	if (wdm == NULL)
		return NOTIFY_DONE;

	if (code==SYS_DOWN || code==SYS_HALT || code==SYS_POWER_OFF) {
		queue_kthread_work(&worker, &wdm->system_down_event);
	}

	return NOTIFY_DONE;
}

static struct notifier_block wdm_reboot_notifier = {
	.notifier_call	= wdm_reboot_notice,
};


void
appwd_init_post_hook(void)
{
	pr_debug("%s\n", __func__);

	if (wdm == NULL)
		return;

	cancel_delayed_kthread_work_sync(&wdm->boot_timeout_event);

	queue_kthread_work(&worker, &wdm->boot_done_event);
}


static void
boot_done(struct kthread_work * work)
{
	switch (wdm->state) {

	case WDM_STATE_BOOT:
		pr_notice("Appliance Watchdog boot completed\n");
		wdm->state = WDM_STATE_ACTIVE;
		wdd_init_start();
		break;

	case WDM_STATE_REBOOT:
		pr_debug("boot_done in REBOOT state, to late!\n");
		break;

	case WDM_STATE_ACTIVE:
	case WDM_STATE_ZOMBIE:
		pr_err("boot_done in invalid state: %d\n", wdm->state);
		break;
	}
}


static void
wdm_reboot(void)
{
	queue_delayed_kthread_work(&worker, &wdm->reboot_timeout_event,
				   wdm->reboot_timeout);

	/* Send SIGINT to init process, which is similar to what
	 * CTRL-ALT-DEL keypress does */
	kill_cad_pid(SIGINT, 1);
}


static void
boot_timeout(struct kthread_work * work)
{
	switch (wdm->state) {

	case WDM_STATE_BOOT:
		wdm->state = WDM_STATE_REBOOT;
		mpc8xxx_rste_cause(RESET_CAUSE_BOOT_TIMEOUT);
		pr_alert("boot_timeout: rebooting system\n");
		wdm_reboot();
		break;

	case WDM_STATE_REBOOT:
		pr_debug("boot_timeout in REBOOT state\n");
		break;

	case WDM_STATE_ACTIVE:
	case WDM_STATE_ZOMBIE:
		pr_err("boot_timeout in invalid state: %d\n", wdm->state);
		break;
	}
}


void
wdd_timeout(struct kthread_work * work)
{
	switch (wdm->state) {

	case WDM_STATE_ACTIVE:
		wdm->state = WDM_STATE_REBOOT;
		mpc8xxx_rste_cause(RESET_CAUSE_APP_TIMEOUT);
		pr_alert("Appliance Watchdog: rebooting system\n");
		wdm_reboot();
		break;

	case WDM_STATE_REBOOT:
		pr_debug("wdd_timeout in REBOOT state\n");
		break;

	case WDM_STATE_BOOT:
	case WDM_STATE_ZOMBIE:
		pr_err("wdd_timeout in invalid state: %d\n", wdm->state);
		break;
	}
}


static void
reboot_timeout(struct kthread_work * work)
{
	switch (wdm->state) {

	case WDM_STATE_REBOOT:
		wdm->state = WDM_STATE_ZOMBIE;
		mpc8xxx_rste_cause(RESET_CAUSE_REBOOT_TIMEOUT);
		pr_emerg("reboot_timeout: kernel restart and watchdog timers will timeout soon!\n");
		/* Restarting NOW! */
		kernel_restart(NULL);
		break;

	case WDM_STATE_BOOT:
	case WDM_STATE_ACTIVE:
	case WDM_STATE_ZOMBIE:
		pr_err("reboot_timeout in invalid state: %d\n", wdm->state);
		break;
	}
}


static void
system_down(struct kthread_work * work)
{
	pr_info("system_down\n");
	switch (wdm->state) {

	// FIXME: anything to do here?

	case WDM_STATE_BOOT:
		pr_notice("system_down in BOOT state\n");
		break;

	case WDM_STATE_ACTIVE:
		pr_notice("system_down in ACTIVE state\n");
		break;

	case WDM_STATE_REBOOT:
		pr_notice("system_down in REBOOT state\n");
		break;

	case WDM_STATE_ZOMBIE:
		pr_notice("system_down in ZOMBIE state\n");
		break;
	}
}


static int
__wdm_init(struct wdm_private * tmp)
{
	int err, i;

	/* Setup work queue */
	err = init_appwd_worker();
	if (err)
		goto create_worker_failed;

	/* Initialize event work structures */
	init_kthread_work(&tmp->boot_done_event, boot_done);
	init_kthread_work(&tmp->wdd_timeout_event, wdd_timeout);
	init_kthread_work(&tmp->system_down_event, system_down);
	init_delayed_kthread_work(&tmp->boot_timeout_event, boot_timeout);
	init_delayed_kthread_work(&tmp->reboot_timeout_event, reboot_timeout);

	/* The wdm_private struct is now initialized enough to handle
	 * async events */
	wdm = tmp;

	if (wdm->boot_timeout) {
		unsigned long delay = wdm->boot_timeout
			- (jiffies - INITIAL_JIFFIES);
		if (delay > 0) {
			queue_delayed_kthread_work(
				&worker, &wdm->boot_timeout_event, delay);
		} else {
			pr_alert("boot_timeout very early\n");
			queue_kthread_work(
				&worker, &wdm->boot_timeout_event.work);
		}
	}

	err = register_reboot_notifier(&wdm_reboot_notifier);
	if (err) {
		pr_err("failed to register reboot notifier: %d\n", err);
		goto register_reboot_notifier_failed;
	}

	for (i = 0; i < wdm->num_wdd ; i++) {
		snprintf(wdm->wdd[i].config.name,
			 sizeof(wdm->wdd[i].config.name), "watchdog%d", i);
		err = wdd_register(&wdm->wdd[i].config);
		if (err) {
			pr_err("wdd_register failed: %d\n", err);
			/* But we will continue anyway... */
		}
	}

	return 0;

	unregister_reboot_notifier(&wdm_reboot_notifier);
register_reboot_notifier_failed:
	stop_appwd_worker();
create_worker_failed:
	kfree(tmp);

	return err;
}


#ifdef CONFIG_OF_APPWD


/* This will typically be indirectly called as device_initcall from the
 * of_platform_bus_probe() call in machine_device_initcall from board
 * file */
static int
wdm_probe(struct platform_device *pdev)
{
	int i;
	struct wdm_private * tmp;
	struct device_node *wdm_np = pdev->dev.of_node, *wdd_np;
	const u32 *val;
	/* FIXME: default values for wdd parameters
	unsigned int init_timeout, keepalive_timeout, recover_timeout;
	int nowayout;
	*/

	i = 0;
	for_each_child_of_node(wdm_np, wdd_np) {
		i++;
	}

	tmp = kzalloc(sizeof(*tmp) + sizeof(tmp->wdd[0]) * i, GFP_KERNEL);
	if (tmp == NULL) {
		pr_warning("Out of memory\n");
		return -ENOMEM;
	}

	tmp->num_wdd = i;

	/* Get configuration values from fdt properties */
	val = of_get_property(wdm_np, "boot-timeout", NULL);
	if (val)
		tmp->boot_timeout = *val * HZ / 1000;

	val = of_get_property(wdm_np, "reboot-timeout", NULL);
	if (val)
		tmp->reboot_timeout = *val * HZ / 1000;

	/* Go through all fdt configured watchdog devices */
	i = 0;
	for_each_child_of_node(wdm_np, wdd_np) {

		val = of_get_property(wdd_np, "init-timeout", NULL);
		if (val)
			tmp->wdd[i].config.init_timeout = *val * HZ / 1000;

		val = of_get_property(wdd_np, "keepalive-timeout", NULL);
		if (val)
			tmp->wdd[i].config.keepalive_timeout = *val * HZ / 1000;

		val = of_get_property(wdd_np, "restart-timeout", NULL);
		if (val)
			tmp->wdd[i].config.restart_timeout = *val * HZ / 1000;

		val = of_get_property(wdd_np, "recover-timeout", NULL);
		if (val)
			tmp->wdd[i].config.recover_timeout = *val * HZ / 1000;

		if (of_find_property(wdd_np, "nowayout", NULL))
			tmp->wdd[i].config.nowayout = 1;

		i++;
	}

	return __wdm_init(tmp);
}


static const struct of_device_id wdm_match[] = {
	{
		.compatible = "appwd-wdm",
	},
	{},
};
MODULE_DEVICE_TABLE(of, wdm_match);


static struct platform_driver wdm_driver = {
	.probe		= wdm_probe,
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= wdm_match,
	},
};


static int wdm_register(void)
{
	int err;
	pr_info("Initializing appliance watchdog core\n");
	err = platform_driver_register(&wdm_driver);
	if (err < 0)
		pr_err("platform_driver_register failed: %d\n", err);
	return err;
}
subsys_initcall(wdm_register);


#else /* CONFIG_OF_APPWD */


static int __init
wdm_init(void)
{
	int i;
	struct wdm_private * tmp;

	tmp = kzalloc(sizeof(*tmp) + sizeof(*tmp->wdd) * CONFIG_APPWD_NUM_WDD,
		      GFP_KERNEL);
	if (tmp == NULL) {
		pr_warning("Out of memory\n");
		return -ENOMEM;
	}

	tmp->num_wdd = CONFIG_APPWD_NUM_WDD;

	tmp->boot_timeout = CONFIG_APPWD_BOOT_TIMEOUT * HZ / 1000;
	tmp->reboot_timeout = CONFIG_APPWD_REBOOT_TIMEOUT * HZ / 1000;

	for (i = 0 ; i < CONFIG_APPWD_NUM_WDD ; i++) {
		tmp->wdd[i].config.init_timeout =
			CONFIG_APPWD_WDD_INIT_TIMEOUT * HZ / 1000;
		tmp->wdd[i].config.keepalive_timeout =
			CONFIG_APPWD_WDD_KEEPALIVE_TIMEOUT * HZ / 1000;
		tmp->wdd[i].config.restart_timeout =
			CONFIG_APPWD_WDD_RESTART_TIMEOUT * HZ / 1000;
		tmp->wdd[i].config.recover_timeout =
			CONFIG_APPWD_WDD_RECOVER_TIMEOUT * HZ / 1000;
#ifdef CONFIG_APPWD_WDD_NOWAYOUT
		tmp->wdd[i].config.nowayout = 1;
#endif
	}

	return __wdm_init(tmp);
}
subsys_initcall(wdm_init);


#endif /* CONFIG_OF_APPWD */
/*
 * Local Variables:
 * compile-command: "make -C ../../.. M=drivers/watchdog/appwd wdm.o"
 * End:
 */
