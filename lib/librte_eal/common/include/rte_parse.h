/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2018 6WIND S.A.
 */

#ifndef _RTE_PARSE_H_
#define _RTE_PARSE_H_

#include <rte_compat.h>

/**
 * @file
 *
 * DPDK parsing generics.
 *
 * This file exposes API and interfaces for describing
 * parsable objects accessible to the EAL.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Generic comparison function.
 *
 * Compare two elements and returns a value depending on
 * their relation to each other.
 *
 * @param lhs
 *   Left-hand-side parameter.
 *
 * @param rhs
 *   Right-hand-side parameter.
 *
 * @return
 *   0 if both operands match each other.
 *   <0 if lhs is inferior to rhs.
 *   >0 if lhs is superior to rhs.
 */
typedef int (*rte_cmp_t)(const void *lhs, const void *rhs);

/**
 * Generic iteration function.
 *
 * Iterate over a set of elements, starting from one until
 * a stopping condition is met.
 *
 * @param start
 *   Starting condition.
 *
 * @param stop
 *   Stopping condition.
 *
 * @return
 *   The address of the current element when the stopping
 *   condition became valid.
 */
typedef const void *
(*rte_iterate_t)(const void *start, const void *stop);

/**
 * Iteration context.
 *
 * This context carries over the current iteration state.
 */
struct rte_iterator {
	const char *devstr; /**< device string. */
	struct rte_bus *bus; /**< bus handle. */
	const char *busstr; /**< bus-related part of device string. */
	struct rte_class *cls; /**< class handle. */
	const char *clsstr; /**< class-related part of device string. */
	struct rte_device *device; /**< current position */
};

__rte_experimental int
rte_parse_iterator(const char *str,
		   struct rte_iterator *it);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_PARSE_H_ */
