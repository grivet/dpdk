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

#ifndef _NET_PRIVATE_H_
#define _NET_PRIVATE_H_

#include <sys/queue.h>

#include <rte_bus.h>
#include <rte_tailq.h>

#include "rte_bus_net.h"

/* Double linked list of devices. */
TAILQ_HEAD(net_device_list, rte_net_device);
/* Double linked list of drivers. */
TAILQ_HEAD(net_driver_list, rte_net_driver);

extern struct net_device_list net_device_list;
extern struct net_driver_list net_driver_list;

/* Bus handle. */
extern struct rte_bus rte_bus_net;

/** Default net devices path. */
extern const char SYSFS_NET_DEVICES[];

/* Find the rte_net_driver of a kernel device. */
struct rte_net_driver *net_drv_find(const char *name);

/* Parse device name. */
int net_parse(const char *name, void *addr);

/* Scan a single device. */
struct rte_net_device *net_scan_one(const char *name);

/* Scan sysfs. */
int net_scan(void);

/* Get sysfs path. */
const char *net_get_sysfs_path(void);

/* Find devargs of a device. */
struct rte_devargs *net_devargs_lookup(const char *name);

/* Read file content. */
int path_read(char *out, size_t size, const char *format, ...)
	__attribute__((format(printf, 3, 0)));

/* Net Bus iterators. */
#define FOREACH_NET_DEVICE(p) \
	TAILQ_FOREACH(p, &(net_device_list), next)

#define FOREACH_NET_DEVICE_SAFE(p, tmp) \
	TAILQ_FOREACH_SAFE(p, &(net_device_list), next, tmp)

#define FOREACH_NET_DRIVER(p) \
	TAILQ_FOREACH(p, &(net_driver_list), next)

#define FOREACH_NET_DRIVER_SAFE(p, tmp) \
	TAILQ_FOREACH_SAFE(p, &(net_driver_list), next, tmp)

/* Net lists operators. */
#define INSERT_NET_DEVICE(dev) \
	TAILQ_INSERT_TAIL(&net_device_list, dev, next)

#define REMOVE_NET_DEVICE(dev) \
	TAILQ_REMOVE(&net_device_list, dev, next)

#define INSERT_NET_DRIVER(drv) \
	TAILQ_INSERT_TAIL(&net_driver_list, drv, next)

#define REMOVE_NET_DRIVER(drv) \
	TAILQ_REMOVE(&net_driver_list, drv, next)

/* Debugging */
#ifndef NDEBUG
#include <stdio.h>
#define DEBUG__(m, ...)						\
	(fprintf(stderr, "%s:%d: %s(): " m "%c",		\
		 __FILE__, __LINE__, __func__, __VA_ARGS__),	\
	 fflush(stderr),					\
	 (void)0)
/*
 * Save/restore errno around DEBUG__().
 * XXX somewhat undefined behavior, but works.
 */
#define DEBUG_(...)				\
	(errno = ((int []){			\
		*(volatile int *)&errno,	\
		(DEBUG__(__VA_ARGS__), 0)	\
	})[0])
#define DEBUG(...) DEBUG_(__VA_ARGS__, '\n')
#define INFO(...) DEBUG(__VA_ARGS__)
#define WARN(...) DEBUG(__VA_ARGS__)
#define ERROR(...) DEBUG(__VA_ARGS__)
#else /* NDEBUG */
#define LOG__(level, m, ...) \
	RTE_LOG(level, EAL, "NET: " m "%c", __VA_ARGS__)
#define LOG_(level, ...) LOG__(level, __VA_ARGS__, '\n')
#define DEBUG(...) (void)0
#define INFO(...) LOG_(INFO, __VA_ARGS__)
#define WARN(...) LOG_(WARNING, "WARNING: " __VA_ARGS__)
#define ERROR(...) LOG_(ERR, "ERROR: " __VA_ARGS__)
#endif /* NDEBUG */

/* Allocate a buffer on the stack and fill it with a printf format string. */
#define MKSTR(name, ...) \
	char name[snprintf(NULL, 0, __VA_ARGS__) + 1]; \
	\
	snprintf(name, sizeof(name), __VA_ARGS__)

#endif /* _NET_PRIVATE_H_ */
