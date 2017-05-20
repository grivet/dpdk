/*-
 *   BSD LICENSE
 *
 *   Copyright 2014 6WIND S.A.
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
 *     * Neither the name of 6WIND S.A nor the names of its contributors
 *       may be used to endorse or promote products derived from this
 *       software without specific prior written permission.
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

/* This file manages the list of devices and their arguments, as given
 * by the user at startup
 *
 * Code here should not call rte_log since the EAL environment
 * may not be initialized.
 */

#include <stdio.h>
#include <string.h>

#include <rte_devargs.h>
#include "eal_private.h"

/** Global list of user devices */
struct rte_devargs_list devargs_list =
	TAILQ_HEAD_INITIALIZER(devargs_list);

int
rte_eal_parse_devargs_str(const char *devargs_str,
			char **drvname, char **drvargs)
{
	char *sep;

	if ((devargs_str) == NULL || (drvname) == NULL || (drvargs == NULL))
		return -1;

	*drvname = strdup(devargs_str);
	if (*drvname == NULL)
		return -1;

	/* set the first ',' to '\0' to split name and arguments */
	sep = strchr(*drvname, ',');
	if (sep != NULL) {
		sep[0] = '\0';
		*drvargs = strdup(sep + 1);
	} else {
		*drvargs = strdup("");
	}

	if (*drvargs == NULL) {
		free(*drvname);
		*drvname = NULL;
		return -1;
	}
	return 0;
}

int
rte_eal_devargs_parse(const char *dev,
		      struct rte_devargs *da)
{
	struct rte_bus *bus;
	const char *c;
	const size_t maxlen = sizeof(da->name);
	size_t i;

	if ((dev) == NULL || (da) == NULL)
		return -EINVAL;
	c = dev;
	/* Retrieve eventual bus info */
	bus = rte_bus_from_name(dev);
	if (bus) {
		i = strlen(bus->name);
		if (dev[i] == '\0') {
			fprintf(stderr, "WARNING: device name matches a bus name.\n");
			bus = NULL;
		} else if (rte_bus_from_dev(dev)) {
			/* false positive on bus name. */
			bus = NULL;
		} else {
			c = &dev[i+1];
		}
	}
	/* Store device name */
	i = 0;
	while (c[i] != '\0' && c[i] != ',') {
		da->name[i] = c[i];
		i++;
		if (i == maxlen) {
			fprintf(stderr, "WARNING: Parsing \"%s\": device name should be shorter than %zu\n",
				dev, maxlen);
			da->name[i-1] = '\0';
			return -EINVAL;
		}
	}
	da->name[i] = '\0';
	if (!bus) {
		bus = rte_bus_from_dev(da->name);
		if (!bus) {
			fprintf(stderr, "ERROR: failed to parse bus info from device \"%s\"\n",
				da->name);
			return -EFAULT;
		}
	}
	da->bus = bus;
	/* Parse eventual device arguments */
	if (c[i] == ',')
		da->args = strdup(&c[i+1]);
	else
		da->args = strdup("");
	if (da->args == NULL) {
		fprintf(stderr, "ERROR: not enough memory to parse arguments\n");
		return -ENOMEM;
	}
	return 0;
}

/* store a whitelist parameter for later parsing */
int
rte_eal_devargs_add(enum rte_devtype devtype, const char *devargs_str)
{
	struct rte_devargs *devargs = NULL;
	const char *dev = devargs_str;
	struct rte_bus *bus;

	/* use calloc instead of rte_zmalloc as it's called early at init */
	devargs = calloc(1, sizeof(*devargs));
	if (devargs == NULL)
		goto fail;

	if (rte_eal_devargs_parse(dev, devargs))
		goto fail;
	devargs->type = devtype;
	bus = devargs->bus;
	if (devargs->type == RTE_DEVTYPE_WHITELISTED) {
		if (bus->conf.scan_mode == RTE_BUS_SCAN_UNDEFINED) {
			bus->conf.scan_mode = RTE_BUS_SCAN_WHITELIST;
		} else if (bus->conf.scan_mode == RTE_BUS_SCAN_BLACKLIST) {
			fprintf(stderr, "ERROR: incompatible device type and bus scan mode\n");
			goto fail;
		}
	} else if (devargs->type == RTE_DEVTYPE_BLACKLISTED) {
		if (bus->conf.scan_mode == RTE_BUS_SCAN_UNDEFINED) {
			bus->conf.scan_mode = RTE_BUS_SCAN_BLACKLIST;
		} else if (bus->conf.scan_mode == RTE_BUS_SCAN_WHITELIST) {
			fprintf(stderr, "ERROR: incompatible device type and bus scan mode\n");
			goto fail;
		}
	}

	TAILQ_INSERT_TAIL(&devargs_list, devargs, next);
	return 0;

fail:
	if (devargs) {
		free(devargs->args);
		free(devargs);
	}

	return -1;
}

/* count the number of devices of a specified type */
unsigned int
rte_eal_devargs_type_count(enum rte_devtype devtype)
{
	struct rte_devargs *devargs;
	unsigned int count = 0;

	TAILQ_FOREACH(devargs, &devargs_list, next) {
		if (devargs->type != devtype)
			continue;
		count++;
	}
	return count;
}

/* dump the user devices on the console */
void
rte_eal_devargs_dump(FILE *f)
{
	struct rte_devargs *devargs;

	fprintf(f, "User device list:\n");
	TAILQ_FOREACH(devargs, &devargs_list, next) {
		fprintf(f, "  [%s]: %s %s\n",
			(devargs->bus ? devargs->bus->name : "??"),
			devargs->name, devargs->args);
	}
}
