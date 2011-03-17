
#ifndef _LINUX_APPWD_H
#define _LINUX_APPWD_H

#include <linux/watchdog.h>

/* Skip to #16 to avoid near-future merge problems if standard
 * watchdog API is extended */
#define	WDIOC_SETRESTARTTIMEOUT		_IOWR(WATCHDOG_IOCTL_BASE, 16, int)
#define	WDIOC_GETRESTARTTIMEOUT		_IOR(WATCHDOG_IOCTL_BASE, 17, int)
#define	WDIOC_SETRECOVERTIMEOUT		_IOWR(WATCHDOG_IOCTL_BASE, 18, int)
#define	WDIOC_GETRECOVERTIMEOUT		_IOR(WATCHDOG_IOCTL_BASE, 19, int)
#define	WDIOC_SETTIMEOUTMSEC		_IOWR(WATCHDOG_IOCTL_BASE, 20, int)
#define	WDIOC_GETTIMEOUTMSEC		_IOR(WATCHDOG_IOCTL_BASE, 21, int)
#define	WDIOC_SETRESTARTTIMEOUTMSEC	_IOWR(WATCHDOG_IOCTL_BASE, 22, int)
#define	WDIOC_GETRESTARTTIMEOUTMSEC	_IOR(WATCHDOG_IOCTL_BASE, 23, int)
#define	WDIOC_SETRECOVERTIMEOUTMSEC	_IOWR(WATCHDOG_IOCTL_BASE, 24, int)
#define	WDIOC_GETRECOVERTIMEOUTMSEC	_IOR(WATCHDOG_IOCTL_BASE, 25, int)

#ifdef __KERNEL__

#ifdef CONFIG_APPWD
extern void appwd_init_post_hook(void);
#else
#define appwd_init_post_hook() do {} while (0)
#endif /* CONFIG_APPWD */

#endif /* __KERNEL__ */

#endif  /* _LINUX_APPWD_H */
