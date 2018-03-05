/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Gaëtan Rivet
 */

#include <string.h>
#include <stdbool.h>

#include <rte_class.h>
#include <rte_compat.h>
#include <rte_errno.h>
#include <rte_kvargs.h>
#include <rte_log.h>

#include "rte_ethdev.h"
#include "rte_ethdev_core.h"

static int
eth_dev_str_cmp(const char *key __rte_unused,
		const char *value,
		void *_str)
{
	const char *str = _str;

	return strcmp(str, value);
}

static bool
eth_dev_does_not_match(const struct rte_eth_dev *edev,
		       struct rte_kvargs *kvlist)
{
	struct rte_eth_dev_data *data;
	unsigned int i;

	data = edev->data;
	for (i = 0; i < kvlist->count; i++) {
		if (rte_kvargs_process(kvlist, "name",
				&eth_dev_str_cmp, data->name))
			return true;
	}
	return false;
}

static int
eth_dev_compare(const void *_edev,
		const void *_str)
{
	const struct rte_eth_dev *edev = _edev;
	struct rte_kvargs *kvargs;
	const char *cstr = _str;
	bool no_match = true;
	char *slash;
	char *str;

	str = strdup(cstr);
	if (str == NULL) {
		RTE_LOG(ERR, EAL, "cannot allocate string copy\n");
		return no_match;
	}
	slash = strchr(str, '/');
	slash = slash ? slash : strchr(str, '\0');
	if (slash == NULL) {
		RTE_LOG(ERR, EAL, "malformed string\n");
		goto free_str;
	}
	slash[0] = '\0';
	kvargs = rte_kvargs_parse(str, NULL);
	if (kvargs == NULL) {
		RTE_LOG(ERR, EAL, "cannot parse argument list\n");
		goto free_str;
	}
	no_match = eth_dev_does_not_match(edev, kvargs);
	if (kvargs)
		rte_kvargs_free(kvargs);
free_str:
	free(str);
	return no_match;
}

static const void *
eth_dev_dev_iterate(const void *_start,
		    const void *_dev)
{
	const struct rte_eth_dev *start = _start;
	const struct rte_device *dev = _dev;
	struct rte_eth_dev *edev;
	uint64_t o = RTE_ETH_DEV_NO_OWNER;
	uint16_t p = 0;

	if (start)
		p = start->data->port_id;
	for (p = rte_eth_find_next_owned_by(p, o); \
	     p < RTE_MAX_ETHPORTS; \
	     p = rte_eth_find_next_owned_by(p + 1, o)) {
		edev = &rte_eth_devices[p];
		if (edev->device == dev)
			return edev;
	}
	return NULL;
}

struct rte_class rte_class_eth = {
	.dev_iterate = eth_dev_dev_iterate,
	.dev_compare = eth_dev_compare,
};

RTE_REGISTER_CLASS(eth, rte_class_eth);
