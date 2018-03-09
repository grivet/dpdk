/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Gaëtan Rivet
 */

#include <string.h>

#include <rte_bus.h>
#include <rte_class.h>
#include <rte_compat.h>
#include <rte_errno.h>
#include <rte_log.h>
#include <rte_parse.h>

#include "eal_private.h"

__rte_experimental int
rte_parse_iterator(const char *str,
		  struct rte_iterator *it)
{
	struct rte_bus *bus = NULL;
	struct rte_class *cls = NULL;
	struct rte_kvarg kv;

	/* Safety checks and prep-work */
	if (rte_parse_kv(str, &kv)) {
		RTE_LOG(ERR, EAL, "Could not parse: %s\n", str);
		return -1;
	}
	it->busstr = NULL;
	it->clsstr = NULL;
	it->device = NULL;
	if (strcmp(kv.key, "bus") == 0) {
		char *slash;

		bus = rte_bus_find_by_name(kv.value);
		it->busstr = str;
		slash = strchr(str, '/');
		if (slash != NULL) {
			if (rte_parse_kv(slash + 1, &kv))
				return -1;
			cls = rte_class_find_by_name(kv.value);
			it->clsstr = slash + 1;
		}
	} else if (strcmp(kv.key, "class") == 0) {
		cls = rte_class_find_by_name(kv.value);
		it->clsstr = str;
	} else {
		rte_errno = EINVAL;
		return -rte_errno;
	}
	if (bus == NULL && cls == NULL) {
		rte_errno = EINVAL;
		return -rte_errno;
	}
	it->devstr = str;
	it->bus = bus;
	it->cls = cls;
	return 0;
}

__rte_experimental int
rte_parse_kv(const char *str, struct rte_kvarg *kv)
{
	const char *equal;
	const char *end;

	if (str == NULL || str[0] == '\0')
		return 1;
	equal = strchr(str, '=');
	if (equal == NULL) {
		rte_errno = EINVAL;
		return -1;
	}
	end = strchr(equal + 1, ',');
	end = end ? end : strchr(equal + 1, '/');
	end = end ? end : strchr(equal + 1, '\0');
	if (end == NULL) {
		rte_errno = ENODEV;
		return -1;
	}
	if (kv == NULL)
		return 0;
	snprintf(kv->data, sizeof(kv->data), "%s", str);
	kv->key = &kv->data[0];
	strchr(kv->data, end[0])[0] = '\0';
	if (strchr(kv->data, '=')) {
		kv->value = strchr(kv->data, '=') + 1;
		strchr(kv->data, '=')[0] = '\0';
	}
	if (end[0] == '\0')
		kv->next = NULL;
	else
		kv->next = end + 1;
	return 0;
}
