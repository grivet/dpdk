/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Gaëtan Rivet
 */

#include <string.h>

#include <rte_compat.h>
#include <rte_errno.h>
#include <rte_log.h>
#include <rte_parse.h>

#include "eal_private.h"

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
