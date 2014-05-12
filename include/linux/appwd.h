#ifndef _LINUX_APPWD_H
#define _LINUX_APPWD_H

#include <uapi/linux/appwd.h>

#ifdef CONFIG_APPWD
extern void appwd_init_post_hook(void);
#else
#define appwd_init_post_hook() do {} while (0)
#endif /* CONFIG_APPWD */

#ifdef CONFIG_APPWD_WDT_GPIO
extern int __init appwd_wdt_gpio_init(unsigned, int);
#else
#define appwd_wdt_gpio_init(a, b) do {} while (0)
#endif /* CONFIG_APPWD_WDT_GPIO */

#endif  /* _LINUX_APPWD_H */
