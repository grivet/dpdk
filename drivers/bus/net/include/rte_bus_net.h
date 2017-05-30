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

#ifndef _RTE_NET_BUS_H_
#define _RTE_NET_BUS_H_

#include <sys/queue.h>
#include <stdint.h>

#include <rte_devargs.h>
#include <rte_dev.h>
#include <rte_bus.h>

/**
 * @file
 *
 * RTE Net bus.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Name of net bus. */
#define RTE_BUS_NET_NAME "net"

/**
 * Structure describing an RTE net device.
 */
struct rte_net_device {
	TAILQ_ENTRY(rte_net_device) next; /**< Next net device. */
	struct rte_device device; /**< Inherit core device. */
	struct rte_net_driver *driver; /**< Associated driver. */
	struct rte_devargs *da; /**< Device parameters. */
	struct rte_device *sh_dev; /**< Shadow device. */
	char name[33]; /**< Kernel netdev name. */
};

/**
 * Name transform callback.
 * Transform a netdevice name into its DPDK counterpart.
 *
 * @param[in] src
 *   original devargs to transform.
 *
 * @param[out] dst
 *   rte_devargs resulting from the transformation.
 *
 * @return
 *   0 for no error.
 *   !0 otherwise.
 */
typedef int (*rte_bus_net_xfrm_t)(const struct rte_devargs *src,
				  struct rte_devargs *dst);

/**
 * Name transform base implementations.
 */
int rte_bus_net_virtual_xfrm(const struct rte_devargs *src,
			     struct rte_devargs *dst);
int rte_bus_net_pci_xfrm(const struct rte_devargs *src,
			 struct rte_devargs *dst);

/**
 * Structure describing an RTE net driver.
 */
struct rte_net_driver {
	TAILQ_ENTRY(rte_net_driver) next; /**< Next net driver. */
	struct rte_driver driver; /**< Inherit core driver. */
	char kmod[33]; /**< Kernel driver name. */
	rte_bus_net_xfrm_t xfrm; /**< Name transform. */
};

/**
 * Register a net driver.
 *
 * @param driver
 *   A pointer to an rte_net_driver structure describing the driver
 *   to be registered.
 */
void rte_net_register(struct rte_net_driver *driver);

/** Helper for net device registration from driver instance. */
#define RTE_PMD_REGISTER_NET(nm, net_drv) \
RTE_INIT(netinitfn_ ##nm); \
static void netinitfn_ ##nm(void) \
{ \
	(net_drv).driver.name = RTE_STR(nm); \
	rte_net_register(&net_drv); \
} \
RTE_PMD_EXPORT_NAME(netdev_ ##nm, __COUNTER__)

/**
 * Unregister a net driver.
 *
 * @param driver
 *   A pointer to an rte_net_driver structure describing the driver
 *   to be unregistered.
 */
void rte_net_unregister(struct rte_net_driver *driver);

/**
 * Get netdevice status.
 *
 * @param name
 *   Net device name.
 *
 * @return
 *   0 if device exist as a net device.
 *   !0 otherwise.
 */
int rte_bus_net_dev_stat(const char *name);

/**
 * Get netdevice driver name.
 *
 * @param[in] name
 *   Net device name.
 *
 * @param[out] out
 *   Buffer to write the driver name to.
 *
 * @param[in] size
 *   Buffer length.
 *
 * @return
 *   0 if driver was successfully written.
 *   !0 otherwise.
 */
int rte_bus_net_driver(const char *name, char *out, size_t size);

/**
 * Get netdevice PCI location.
 *
 * @param[in] name
 *   Net device name.
 *
 * @param[out] out
 *   Buffer to write the PCI address to.
 *
 * @param[in] size
 *   Buffer length.
 *
 * @return
 *   0 if PCI address was successfully written.
 *   !0 otherwise.
 */
int rte_bus_net_pci_loc(const char *name, char *out, size_t size);

/**
 * Read the content of a file.
 *
 * @param[out] out
 *   Buffer to write the content to.
 *
 * @param[in] size
 *   Buffer length.
 *
 * @param[in] format
 *   printf-like format string describing the path
 *   of the file to read.
 *
 * @return
 *   0 if all was written successfully.
 *   !0 otherwise. errno is set.
 */
int rte_bus_net_path_read(char *out, size_t size,
			  const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_NET_BUS_H_ */
