/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 NXP
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
 *     * Neither the name of NXP nor the names of its
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

#include <stdio.h>
#include <string.h>
#include <sys/queue.h>

#include <rte_bus.h>

#include "eal_private.h"

struct rte_bus_list rte_bus_list =
	TAILQ_HEAD_INITIALIZER(rte_bus_list);

void
rte_bus_register(struct rte_bus *bus)
{
	RTE_VERIFY(bus);
	RTE_VERIFY(bus->name && strlen(bus->name));
	/* A bus should mandatorily have the scan implemented */
	RTE_VERIFY(bus->scan);
	RTE_VERIFY(bus->probe);
	/* Buses supporting hotplug also require unplug. */
	RTE_VERIFY(!bus->plug || bus->unplug);

	TAILQ_INSERT_TAIL(&rte_bus_list, bus, next);
	RTE_LOG(DEBUG, EAL, "Registered [%s] bus.\n", bus->name);
}

void
rte_bus_unregister(struct rte_bus *bus)
{
	TAILQ_REMOVE(&rte_bus_list, bus, next);
	RTE_LOG(DEBUG, EAL, "Unregistered [%s] bus.\n", bus->name);
}

/* Scan all the buses for registered devices */
int
rte_bus_scan(void)
{
	int ret;
	struct rte_bus *bus = NULL;

	TAILQ_FOREACH(bus, &rte_bus_list, next) {
		ret = bus->scan();
		if (ret) {
			RTE_LOG(ERR, EAL, "Scan for (%s) bus failed.\n",
				bus->name);
			return ret;
		}
	}

	return 0;
}

/* Probe all devices of all buses */
int
rte_bus_probe(void)
{
	int ret;
	struct rte_bus *bus, *vbus = NULL;

	TAILQ_FOREACH(bus, &rte_bus_list, next) {
		if (!strcmp(bus->name, "virtual")) {
			vbus = bus;
			continue;
		}

		ret = bus->probe();
		if (ret) {
			RTE_LOG(ERR, EAL, "Bus (%s) probe failed.\n",
				bus->name);
			return ret;
		}
	}

	if (vbus) {
		ret = vbus->probe();
		if (ret) {
			RTE_LOG(ERR, EAL, "Bus (%s) probe failed.\n",
				vbus->name);
			return ret;
		}
	}

	return 0;
}

/* Dump information of a single bus */
static int
bus_dump_one(FILE *f, struct rte_bus *bus)
{
	int ret;

	/* For now, dump only the bus name */
	ret = fprintf(f, " %s\n", bus->name);

	/* Error in case of inability in writing to stream */
	if (ret < 0)
		return ret;

	return 0;
}

void
rte_bus_dump(FILE *f)
{
	int ret;
	struct rte_bus *bus;

	TAILQ_FOREACH(bus, &rte_bus_list, next) {
		ret = bus_dump_one(f, bus);
		if (ret) {
			RTE_LOG(ERR, EAL, "Unable to write to stream (%d)\n",
				ret);
			break;
		}
	}
}

struct rte_bus *
rte_bus_find(rte_bus_cmp_t cmp,
	     const void *data,
	     const struct rte_bus *start)
{
	struct rte_bus *bus = NULL;
	int started = start == NULL;

	TAILQ_FOREACH(bus, &rte_bus_list, next) {
		if (!started) {
			if (bus == start)
				started = 1;
			continue;
		}
		if (cmp(bus, data) == 0)
			break;
	}
	return bus;
}

static int
cmp_rte_device(const struct rte_device *dev, const void *_dev2)
{
	const struct rte_device *dev2 = _dev2;

	return !(dev == dev2);
}

static int
bus_find_device(const struct rte_bus *bus, const void *_dev)
{
	struct rte_device *dev;

	if (!bus->find_device)
		return -1;
	dev = bus->find_device(cmp_rte_device, _dev);
	return !dev;
}

struct rte_bus *rte_bus_find_by_device(const struct rte_device *dev)
{
	return rte_bus_find(bus_find_device, (const void *)dev, NULL);
}

struct rte_device *
rte_bus_find_device(const struct rte_device *start,
		    rte_dev_cmp_t cmp, const void *data)
{
	struct rte_bus *bus;
	struct rte_device *dev = NULL;
	int started = start == NULL;

	TAILQ_FOREACH(bus, &rte_bus_list, next) {
		if (!bus->find_device)
			continue;
		if (!started) {
			dev = bus->find_device(cmp_rte_device, start);
			if (dev)
				started = 1;
			continue;
		}
		dev = bus->find_device(cmp, data);
		if (dev)
			break;
	}
	return dev;
}

static int
bus_cmp_name(const struct rte_bus *bus, const void *_name)
{
	const char *name = _name;

	return strncmp(bus->name, name,
		       strlen(bus->name));
}

/* find a bus by its name */
struct rte_bus *
rte_bus_from_name(const char *str)
{
	return rte_bus_find(bus_cmp_name, str, NULL);
}
