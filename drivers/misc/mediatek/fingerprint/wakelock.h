/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal wake_lock compatibility shim for the Oplus fingerprint driver.
 * Mainline removed the old embedded-wakeup_source API, so keep a pointer
 * registered through wakeup_source_register().
 */
#ifndef _LINUX_WAKELOCK_H
#define _LINUX_WAKELOCK_H

#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/jiffies.h>

enum {
    WAKE_LOCK_SUSPEND,
    WAKE_LOCK_TYPE_COUNT
};

struct wake_lock {
    struct wakeup_source *ws;
};

static inline void wake_lock_init(struct wake_lock *lock, int type,
                                  const char *name)
{
    if (!lock)
        return;
    lock->ws = wakeup_source_register(NULL, name);
}

static inline void wake_lock_destroy(struct wake_lock *lock)
{
    if (!lock || !lock->ws)
        return;
    wakeup_source_unregister(lock->ws);
    lock->ws = NULL;
}

static inline void wake_lock(struct wake_lock *lock)
{
    if (lock && lock->ws)
        __pm_stay_awake(lock->ws);
}

static inline void wake_lock_timeout(struct wake_lock *lock, long timeout)
{
    if (lock && lock->ws)
        __pm_wakeup_event(lock->ws, jiffies_to_msecs(timeout));
}

static inline void wake_unlock(struct wake_lock *lock)
{
    if (lock && lock->ws)
        __pm_relax(lock->ws);
}

static inline int wake_lock_active(struct wake_lock *lock)
{
    return 0;
}

#endif /* _LINUX_WAKELOCK_H */
