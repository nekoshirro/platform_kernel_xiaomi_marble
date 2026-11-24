/*
 * Author: andip71, 01.09.2017
 *
 * Version 1.2.0
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Change log:
 *
 * 1.2.0 (2026-08-30)
 *   - Curate the default list for portability across MTK/QCOM
 *   - Match whole ';'-delimited names, not any substring
 *   - Never block a hard wakeup event; restore pm_system_wakeup()
 *   - Blocked events no longer arm the wakeup-source expiry timer
 *   - Split out a side-effect-free predicate for the diagnostic walker
 *   - Fix the active flag, list capacities and terminator bounds
 *
 * 1.1.0 (01.09.2017)
 *   - By default, the following wakelocks are blocked in an own list
 *     qcom_rx_wakelock, wlan, wlan_wow_wl, wlan_extscan_wl, NETLINK
 *
 * 1.0.1 (29.08.2017)
 *   - Add killing wakelock when currently active
 *
 * 1.0.0 (28.08.2017)
 *   - Initial version
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/rcupdate.h>
#include <linux/string.h>
#include "boeffla_wl_blocker.h"


/*****************************************/
// Variables
/*****************************************/

static DEFINE_MUTEX(wl_blocker_mutex);
static char list_wl[LENGTH_LIST_WL] = {0};
static char list_wl_default[LENGTH_LIST_WL_DEFAULT] = {0};
static char wl_search_bufs[2][LENGTH_LIST_WL_SEARCH] = { {0}, {0} };
static unsigned int wl_search_idx;


/*****************************************/
// internal functions
/*****************************************/

static bool list_has_entries(const char *list)
{
	while (*list) {
		if (*list != ';')
			return true;
		list++;
	}
	return false;
}

static void build_search_string(const char *list1, const char *list2, bool sync_rcu)
{
	unsigned int next = wl_search_idx ^ 1;
	bool active = list_has_entries(list1) || list_has_entries(list2);

	/*
	 * Publish into an inactive ping-pong slot so lockless RCU readers in
	 * the wakeup path never observe a half-written string or stale tail.
	 * When neither list has entries, publish NULL so the wakeup hotpath
	 * exits on a single rcu_access_pointer() check without scanning.
	 */
	if (active) {
		scnprintf(wl_search_bufs[next], LENGTH_LIST_WL_SEARCH,
			  ";%s;%s;", list1, list2);
		rcu_assign_pointer(wl_search_active, wl_search_bufs[next]);
		wl_search_idx = next;
	} else {
		RCU_INIT_POINTER(wl_search_active, NULL);
	}
	WRITE_ONCE(wl_blocker_active, active);

	if (sync_rcu)
		synchronize_rcu();
}

static ssize_t store_wl_list(char *dst, size_t cap, const char *buf, size_t n)
{
	size_t len = n;

	while (len && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
		len--;

	/* '>=': the NUL terminator needs the last byte; reject multi-line/NUL payloads. */
	if (len >= cap || memchr(buf, '\0', len) ||
	    memchr(buf, '\n', len) || memchr(buf, '\r', len))
		return -EINVAL;

	mutex_lock(&wl_blocker_mutex);
	memcpy(dst, buf, len);
	dst[len] = '\0';
	build_search_string(list_wl_default, list_wl, true);
	mutex_unlock(&wl_blocker_mutex);

	return n;
}


/*****************************************/
// sysfs interface functions
/*****************************************/

// show list of user configured wakelocks
static ssize_t wakelock_blocker_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;

	mutex_lock(&wl_blocker_mutex);
	ret = scnprintf(buf, PAGE_SIZE, "%s\n", list_wl);
	mutex_unlock(&wl_blocker_mutex);

	return ret;
}


// store list of user configured wakelocks
static ssize_t wakelock_blocker_store(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	return store_wl_list(list_wl, sizeof(list_wl), buf, n);
}


// show list of default, predefined wakelocks
static ssize_t wakelock_blocker_default_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	ssize_t ret;

	mutex_lock(&wl_blocker_mutex);
	ret = scnprintf(buf, PAGE_SIZE, "%s\n", list_wl_default);
	mutex_unlock(&wl_blocker_mutex);

	return ret;
}


// store list of default, predefined wakelocks
static ssize_t wakelock_blocker_default_store(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	return store_wl_list(list_wl_default, sizeof(list_wl_default), buf, n);
}


// show debug information of driver internals
static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *search;
	ssize_t ret;

	mutex_lock(&wl_blocker_mutex);
	search = rcu_dereference_protected(wl_search_active,
					   lockdep_is_held(&wl_blocker_mutex));
	ret = scnprintf(buf, PAGE_SIZE,
			"Debug status: %d\n\nUser list: %s\nDefault list: %s\n"
			"Search list: %s\nActive: %d\n",
			READ_ONCE(wl_blocker_debug), list_wl, list_wl_default,
			search ?: ";;;", READ_ONCE(wl_blocker_active));
	mutex_unlock(&wl_blocker_mutex);

	return ret;
}


// store debug mode on/off (1/0)
static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	WRITE_ONCE(wl_blocker_debug, val);

	return count;
}


static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	// return version information
	return scnprintf(buf, PAGE_SIZE, "%s\n", BOEFFLA_WL_BLOCKER_VERSION);
}



/*****************************************/
// Initialize sysfs objects
/*****************************************/

// define objects
static DEVICE_ATTR_RW(wakelock_blocker);
static DEVICE_ATTR_RW(wakelock_blocker_default);
static DEVICE_ATTR_RW(debug);
static DEVICE_ATTR_RO(version);

// define attributes
static struct attribute *boeffla_wl_blocker_attributes[] = {
	&dev_attr_wakelock_blocker.attr,
	&dev_attr_wakelock_blocker_default.attr,
	&dev_attr_debug.attr,
	&dev_attr_version.attr,
	NULL
};

// define attribute group
static struct attribute_group boeffla_wl_blocker_control_group = {
	.attrs = boeffla_wl_blocker_attributes,
};

static const struct attribute_group *boeffla_wl_blocker_groups[] = {
	&boeffla_wl_blocker_control_group,
	NULL
};

// define control device
static struct miscdevice boeffla_wl_blocker_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "boeffla_wakelock_blocker",
	.groups = boeffla_wl_blocker_groups,
};


/*****************************************/
// Driver init and exit functions
/*****************************************/

static int __init boeffla_wl_blocker_init(void)
{
	int ret;

	BUILD_BUG_ON(sizeof(LIST_WL_DEFAULT) > LENGTH_LIST_WL_DEFAULT);

	/* Initialize default list before exposing sysfs controls. */
	strscpy(list_wl_default, LIST_WL_DEFAULT, sizeof(list_wl_default));
	build_search_string(list_wl_default, list_wl, false);

	ret = misc_register(&boeffla_wl_blocker_control_device);
	if (ret) {
		pr_err("Boeffla WL blocker: failed to register misc device (%d)\n",
		       ret);
		return ret;
	}

	pr_info("Boeffla WL blocker: driver version %s started\n",
		BOEFFLA_WL_BLOCKER_VERSION);

	return 0;
}


static void __exit boeffla_wl_blocker_exit(void)
{
	misc_deregister(&boeffla_wl_blocker_control_device);

	pr_info("Boeffla WL blocker: driver stopped\n");
}


/* define driver entry points */
module_init(boeffla_wl_blocker_init);
module_exit(boeffla_wl_blocker_exit);
