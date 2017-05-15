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

#ifndef _RTE_BUS_H_
#define _RTE_BUS_H_

/**
 * @file
 *
 * DPDK device bus interface
 *
 * This file exposes API and interfaces for bus abstraction
 * over the devices and drivers in EAL.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <sys/queue.h>

#include <rte_log.h>
#include <rte_dev.h>

/** Double linked list of buses */
TAILQ_HEAD(rte_bus_list, rte_bus);

/**
 * Bus specific scan for devices attached on the bus.
 * For each bus object, the scan would be responsible for finding devices and
 * adding them to its private device list.
 *
 * A bus should mandatorily implement this method.
 *
 * @return
 *	0 for successful scan
 *	<0 for unsuccessful scan with error value
 */
typedef int (*rte_bus_scan_t)(void);

/**
 * Implementation specific probe function which is responsible for linking
 * devices on that bus with applicable drivers.
 *
 * This is called while iterating over each registered bus.
 *
 * @return
 *	0 for successful probe
 *	!0 for any error while probing
 */
typedef int (*rte_bus_probe_t)(void);

/**
 * Device iterator to find a particular device on a bus.
 */
typedef struct rte_device * (*rte_bus_find_device_t)(rte_dev_cmp_t match,
						     const void *data);

/**
 * Implementation specific probe function which is responsible for linking
 * devices on that bus with applicable drivers.
 * The plugged device might already have been used previously by the bus,
 * in which case some buses might prefer to detect and re-use the relevant
 * information pertaining to this device.
 *
 * @param da
 *	Device declaration.
 *
 * @return
 *	The pointer to a valid rte_device usable by the bus on success.
 *	NULL on error. rte_errno is then set.
 */
typedef struct rte_device * (*rte_bus_plug_t)(struct rte_devargs *da);

/**
 * Implementation specific remove function which is responsible for unlinking
 * devices on that bus from assigned driver.
 *
 * @param dev
 *	Device pointer that was returned by a previous device plug call.
 *
 * @return
 *	0 on success.
 *	!0 on error. rte_errno is then set.
 */
typedef int (*rte_bus_unplug_t)(struct rte_device *dev);

/**
 * Bus specific parsing function.
 * Validates the syntax used in the textual representation of a device,
 * If the syntax is valid and ``addr`` is not NULL, writes the bus-specific
 * device representation to ``addr``.
 *
 * @param[in] name
 *	device textual description
 *
 * @param[out] addr
 *	device information location address, into which parsed info
 *	should be written. If NULL, nothing should be written, which
 *	is not an error.
 *
 * @return
 *	0 if parsing was successful.
 *	!0 for any error.
 */
typedef int (*rte_bus_parse_t)(const char *name, void *addr);

/**
 * Bus scan policies
 */
enum rte_bus_scan_mode {
	RTE_BUS_SCAN_UNDEFINED,
	RTE_BUS_SCAN_WHITELIST,
	RTE_BUS_SCAN_BLACKLIST,
};

/**
 * A structure used to configure bus operations.
 */
struct rte_bus_conf {
	enum rte_bus_scan_mode scan_mode; /**< Scan policy. */
};

/**
 * A structure describing a generic bus.
 */
struct rte_bus {
	TAILQ_ENTRY(rte_bus) next;   /**< Next bus object in linked list */
	const char *name;            /**< Name of the bus */
	rte_bus_scan_t scan;         /**< Scan for devices attached to bus */
	rte_bus_probe_t probe;       /**< Probe devices on bus */
	rte_bus_find_device_t find_device; /**< Find device on bus */
	rte_bus_plug_t plug;         /**< Probe single device for drivers */
	rte_bus_unplug_t unplug;     /**< Remove single device from driver */
	rte_bus_parse_t parse;       /**< Parse a device name */
	struct rte_bus_conf conf;    /**< Bus configuration */
};

/**
 * Register a Bus handler.
 *
 * @param bus
 *   A pointer to a rte_bus structure describing the bus
 *   to be registered.
 */
void rte_bus_register(struct rte_bus *bus);

/**
 * Unregister a Bus handler.
 *
 * @param bus
 *   A pointer to a rte_bus structure describing the bus
 *   to be unregistered.
 */
void rte_bus_unregister(struct rte_bus *bus);

/**
 * Scan all the buses.
 *
 * @return
 *   0 in case of success in scanning all buses
 *  !0 in case of failure to scan
 */
int rte_bus_scan(void);

/**
 * For each device on the buses, perform a driver 'match' and call the
 * driver-specific probe for device initialization.
 *
 * @return
 *	 0 for successful match/probe
 *	!0 otherwise
 */
int rte_bus_probe(void);

/**
 * Dump information of all the buses registered with EAL.
 *
 * @param f
 *	 A valid and open output stream handle
 *
 * @return
 *	 0 in case of success
 *	!0 in case there is error in opening the output stream
 */
void rte_bus_dump(FILE *f);

/**
 * Bus comparison function.
 *
 * @param bus
 *	Bus under test.
 *
 * @param data
 *	Data to compare against.
 *
 * @return
 *	0 if the bus matches the data.
 *	!0 if the bus does not match.
 *	<0 if ordering is possible and the bus is lower than the data.
 *	>0 if ordering is possible and the bus is greater than the data.
 */
typedef int (*rte_bus_cmp_t)(const struct rte_bus *bus, const void *data);

/**
 * Bus iterator to find a particular bus.
 *
 * If the callback returns zero this function will stop iterating over
 * any more buses.
 * If the start parameter is non-NULL, the comparison will only be determined
 * past this element.
 *
 * @param cmp
 *	Comparison function.
 *
 * @param data
 *	 Data to pass to cmp callback
 *
 * @param start
 *	Starting point for the iteration.
 *
 * @return
 *	 A pointer to a rte_bus structure or NULL in case no bus matches
 */
struct rte_bus *rte_bus_find(rte_bus_cmp_t cmp,
			     const void *data,
			     const struct rte_bus *start);

/**
 * Bus iterator to find a particular device.
 *
 * If the callback returns non-zero this function will stop iterating over any
 * more buses and devices. To continue a search the device of a previous search
 * is passed via the start parameters.
 *
 * @param start
 *	 Start device of the iteration.
 *
 * @param cmp
 *	 Callback function to check device.
 *
 * @param data
 *	 Data to pass to match callback.
 *
 * @return
 *	 A pointer to a rte_bus structure or NULL in case no bus matches.
 */
struct rte_device *
rte_bus_find_device(const struct rte_device *start,
		    rte_dev_cmp_t cmp, const void *data);

/**
 * Find the registered bus for a particular device.
 */
struct rte_bus *rte_bus_find_by_device(const struct rte_device *dev);

/*
 * Find a bus handle by its name.
 * Compares the name of each bus up until any invalid character
 * in the matched pattern.
 *
 * @param str
 *	A null terminated character string.
 *
 * @return
 *	A pointer to a bus if found.
 *	NULL if no bus matches.
 */
struct rte_bus *rte_bus_from_name(const char *str);

/**
 * Find a bus capable of identifying a device.
 *
 * @param str
 *   A device identifier (PCI address, virtual PMD name, ...).
 *
 * @return
 *   A valid bus handle if found.
 *   NULL if no bus is able to parse this device.
 */
struct rte_bus *rte_bus_from_dev(const char *str);

/**
 * Helper for Bus registration.
 * The constructor has higher priority than PMD constructors.
 */
#define RTE_REGISTER_BUS(nm, bus) \
static void __attribute__((constructor(101), used)) businitfn_ ##nm(void) \
{\
	(bus).name = nm;\
	rte_bus_register(&bus); \
}

#ifdef __cplusplus
}
#endif

#endif /* _RTE_BUS_H */
