/*-
 *   BSD LICENSE
 *
 *   Copyright 2017 6WIND S.A.
 *   Copyright 2017 Mellanox.
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
 *     * Neither the name of 6WIND S.A. nor the names of its
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

#include <rte_malloc.h>

#include "failsafe_private.h"

static struct rte_eth_dev *
fs_find_ethdev(const struct rte_device *dev)
{
	struct rte_eth_dev *eth_dev;
	uint8_t i;

	RTE_ETH_FOREACH_DEV(i) {
		eth_dev = &rte_eth_devices[i];
		if (eth_dev->device == dev)
			return eth_dev;
	}
	return NULL;
}

static int
fs_bus_init(struct rte_eth_dev *dev)
{
	struct sub_device *sdev;
	struct rte_device *rdev;
	struct rte_devargs *da;
	uint8_t i;
	int ret;

	FOREACH_SUBDEV(sdev, i, dev) {
		if (sdev->state != DEV_PARSED)
			continue;
		da = &sdev->devargs;
		if (!sdev->bus->plug) {
			ERROR("Bus %s used for sub_device %d does not support hotplug, skipping",
			      sdev->bus->name, i);
			return -EINVAL;
		}
		rdev = sdev->bus->plug(da);
		ret = rdev ? 0 : -rte_errno;
		if (ret) {
			ERROR("sub_device %d probe failed %s%s%s", i,
			      errno ? "(" : "",
			      errno ? strerror(errno) : "",
			      errno ? ")" : "");
			continue;
		}
		ETH(sdev) = fs_find_ethdev(rdev);
		if (ETH(sdev) == NULL) {
			ERROR("sub_device %d init went wrong", i);
			return -ENODEV;
		}
		sdev->dev = ETH(sdev)->device;
		ETH(sdev)->state = RTE_ETH_DEV_DEFERRED;
		sdev->state = DEV_PROBED;
	}
	return 0;
}

int
failsafe_eal_init(struct rte_eth_dev *dev)
{
	struct sub_device *sdev;
	uint8_t i;
	int ret;

	ret = fs_bus_init(dev);
	if (ret)
		return ret;
	/*
	 * We only update TX_SUBDEV if we are not started.
	 * If a sub_device is emitting, we will switch the TX_SUBDEV to the
	 * preferred port only upon starting it, so that the switch is smoother.
	 */
	if (PREFERRED_SUBDEV(dev)->state >= DEV_PROBED) {
		if (TX_SUBDEV(dev) != PREFERRED_SUBDEV(dev) &&
		    (TX_SUBDEV(dev) == NULL ||
		     (TX_SUBDEV(dev) && TX_SUBDEV(dev)->state < DEV_STARTED))) {
			DEBUG("Switching tx_dev to preferred sub_device");
			PRIV(dev)->subs_tx = 0;
		}
	} else {
		if ((TX_SUBDEV(dev) && TX_SUBDEV(dev)->state < DEV_PROBED) ||
		    TX_SUBDEV(dev) == NULL) {
			/* Using first probed device */
			FOREACH_SUBDEV_ST(sdev, i, dev, DEV_PROBED) {
				DEBUG("Switching tx_dev to sub_device %d",
				      i);
				PRIV(dev)->subs_tx = i;
				break;
			}
		}
	}
	return 0;
}

static int
fs_bus_uninit(struct rte_eth_dev *dev)
{
	struct sub_device *sdev = NULL;
	struct rte_device *rdev;
	uint8_t i;
	int ret;

	FOREACH_SUBDEV_ST(sdev, i, dev, DEV_PROBED) {
		rdev = sdev->dev;
		if (!sdev->bus->unplug) {
			ERROR("Bus does not support device removal for sub_device %u (%s)",
			      i, rdev->name);
			continue;
		}
		ret = sdev->bus->unplug(rdev);
		if (ret) {
			ERROR("Failed to remove requested device %s",
			      rdev->name);
			continue;
		}
		sdev->state = DEV_PROBED - 1;
	}
	return 0;
}

int
failsafe_eal_uninit(struct rte_eth_dev *dev)
{
	int ret;

	ret = fs_bus_uninit(dev);
	if (ret)
		return ret;
	return 0;
}
