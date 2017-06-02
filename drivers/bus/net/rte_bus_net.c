/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 6WIND S.A.
 *   Author: Gaetan Rivet
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of 6WIND nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>

#include <rte_devargs.h>
#include <rte_dev.h>
#include <rte_bus.h>
#include <rte_errno.h>

#include "rte_bus_net.h"
#include "private.h"

struct net_device_list net_device_list =
	TAILQ_HEAD_INITIALIZER(net_device_list);
struct net_driver_list net_driver_list =
	TAILQ_HEAD_INITIALIZER(net_driver_list);

struct rte_devargs *
net_devargs_lookup(const char *name)
{
	struct rte_devargs *da;

	TAILQ_FOREACH(da, &devargs_list, next) {
		if (da->bus != &rte_bus_net)
			continue;
		if (!strcmp(da->name, name))
			return da;
	}
	return NULL;
}

struct rte_net_driver *
net_drv_find(const char *name)
{
	struct rte_net_driver *drv = NULL;
	char drv_kmod[32] = "";

	if (rte_bus_net_driver(name, drv_kmod, sizeof(drv_kmod)))
		return NULL;
	FOREACH_NET_DRIVER(drv)
		if (!strcmp(drv->kmod, drv_kmod))
			break;
	return drv;
}

/*
 * typeof(addr): struct rte_net_device *
 */
int
net_parse(const char *name, void *addr)
{
	struct rte_net_device *dev = addr;
	struct rte_net_driver *drv = NULL;
	struct rte_devargs devargs;
	struct rte_devargs *da;
	struct rte_devargs *sub;

	if (rte_bus_net_dev_stat(name))
		return 1;
	if (addr == NULL)
		return 0;
	drv = net_drv_find(name);
	if (drv == NULL) {
		DEBUG("unable to find driver");
		rte_errno = EFAULT;
		return -1;
	}
	dev->driver = drv;
	da = dev->device.devargs;
	if (!da) {
		da = &devargs;
		snprintf(da->name, sizeof(da->name), "%s", name);
		da->args = NULL;
		sub = calloc(1, sizeof(*sub));
		dev->device.name = sub->name;
	} else {
		sub = rte_eal_devargs_clone(da);
		dev->device.name = da->name;
	}
	if (sub == NULL) {
		DEBUG("unable to clone devargs");
		rte_errno = ENOMEM;
		return -1;
	}
	dev->da = sub;
	if (drv->xfrm(da, sub)) {
		DEBUG("unable to parse device name");
		rte_errno = ENODEV;
		return -1;
	}
	sub->bus = rte_bus_from_dev(sub->name);
	return !sub->bus;
}

struct rte_net_device *
net_scan_one(const char *name)
{
	struct rte_net_device *dev;

	dev = calloc(1, sizeof(*dev));
	if (dev == NULL) {
		ERROR("failed to allocate memory");
		return NULL;
	}
	snprintf(dev->name, sizeof(dev->name), "%s", name);
	dev->device.devargs = net_devargs_lookup(dev->name);
	if (!net_parse(dev->name, dev)) {
		INSERT_NET_DEVICE(dev);
		return dev;
	}
	free(dev);
	return NULL;
}

static int
net_probe_one(struct rte_net_device *dev)
{
	struct rte_devargs *da;
	struct rte_device *rdev;
	struct rte_bus *bus;
	int ret;

	da = dev->da;
	bus = da->bus;
	if (bus->plug == NULL) {
		ERROR("bus %s does not support plugin", bus->name);
		return -1;
	}
	rdev = bus->plug(da);
	ret = rdev ? 0 : -rte_errno;
	if (rdev)
		dev->sh_dev = rdev;
	return ret;
}

static int
net_probe(void)
{
	struct rte_net_device *dev;
	int ret;

	FOREACH_NET_DEVICE(dev) {
		ret = net_probe_one(dev);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static struct rte_device *
net_find_device(int (*match)(const struct rte_device *dev, const void *data),
		const void *data)
{
	struct rte_net_device *dev;

	FOREACH_NET_DEVICE(dev)
		if (match(&dev->device, data))
			return &dev->device;
	return NULL;
}

void
rte_net_register(struct rte_net_driver *driver)
{
	if (driver->xfrm == NULL)
		driver->xfrm = rte_bus_net_pci_xfrm;
	INSERT_NET_DRIVER(driver);
}

void
rte_net_unregister(struct rte_net_driver *driver)
{
	REMOVE_NET_DRIVER(driver);
}

const char *
net_get_sysfs_path(void)
{
	const char *path = NULL;

	path = getenv("SYSFS_NET_DEVICES");
	if (path == NULL)
		return SYSFS_NET_DEVICES;
	return path;
}

int
rte_bus_net_virtual_xfrm(const struct rte_devargs *src,
			 struct rte_devargs *dst)
{
	size_t argslen;

	/* dev name. */
	snprintf(dst->name, sizeof(dst->name),
		 "net_tap_%s", src->name);
	/* dev args. */
	if (dst->args)
		free(dst->args);
	argslen = snprintf(NULL, 0, "iface=tap_%s,remote=%s,%s",
			   src->name, src->name, src->args) + 1;
	dst->args = calloc(1, argslen);
	if (dst->args == NULL)
		return -1;
	snprintf(dst->args, argslen, "iface=tap_%s,remote=%s,%s",
		 src->name, src->name, src->args);
	return 0;
}

int
rte_bus_net_pci_xfrm(const struct rte_devargs *src,
		     struct rte_devargs *dst)
{
	char buf[32] = "";
	int ret;

	ret = rte_bus_net_pci_loc(src->name, buf, sizeof(buf));
	if (ret)
		return ret;
	ret = snprintf(dst->name, sizeof(dst->name), "%s", buf);
	dst->args = strdup(src->args);
	return ret < 0 || ret > (int)sizeof(dst->name) || dst->args == NULL;
}

static struct rte_device *
net_plug(struct rte_devargs *da)
{
	struct rte_net_device *dev;

	dev = net_scan_one(da->name);
	if (dev == NULL) {
		rte_errno = EFAULT;
		return NULL;
	}
	if (net_probe_one(dev)) {
		rte_errno = ENODEV;
		return NULL;
	}
	return dev->sh_dev;
}

static int
net_unplug(struct rte_device *dev)
{
	struct rte_net_device *ndev;
	void *tmp;
	int ret;
	int err;

	FOREACH_NET_DEVICE_SAFE(ndev, tmp) {
		struct rte_device *rdev;
		struct rte_devargs *da;
		struct rte_devargs *sub;

		if (dev != &ndev->device &&
		    dev != ndev->sh_dev)
			continue;
		rdev = ndev->sh_dev;
		if (rdev == NULL)
			continue;
		da = ndev->device.devargs;
		sub = rdev->devargs;
		ret = sub->bus->unplug(rdev);
		if (ret) {
			err = rte_errno;
			ERROR("unplug failed");
			rte_errno = err;
			return ret;
		}
		free(sub);
		rte_eal_devargs_rmv(da);
		REMOVE_NET_DEVICE(ndev);
		free(ndev);
		break;
	}
	if (ndev == NULL) {
		ERROR("no such device");
		return -ENODEV;
	}
	return 0;
}

struct rte_bus rte_bus_net = {
	.scan = net_scan,
	.probe = net_probe,
	.find_device = net_find_device,
	.parse = net_parse,
	.plug = net_plug,
	.unplug = net_unplug,
	.conf = {
		.scan_mode = RTE_BUS_SCAN_UNDEFINED,
	},
};
RTE_REGISTER_BUS(RTE_BUS_NET_NAME, rte_bus_net);
