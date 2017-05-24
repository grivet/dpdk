/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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
 *     * Neither the name of the copyright holder nor the names of its
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

#ifndef _RTE_CRYPTODEV_VDEV_H_
#define _RTE_CRYPTODEV_VDEV_H_

#include <rte_vdev.h>

#define RTE_CRYPTODEV_VDEV_DEFAULT_MAX_NB_QUEUE_PAIRS	8
#define RTE_CRYPTODEV_VDEV_DEFAULT_MAX_NB_SESSIONS	2048

#define RTE_CRYPTODEV_VDEV_NAME				("name")
#define RTE_CRYPTODEV_VDEV_MAX_NB_QP_ARG		("max_nb_queue_pairs")
#define RTE_CRYPTODEV_VDEV_MAX_NB_SESS_ARG		("max_nb_sessions")
#define RTE_CRYPTODEV_VDEV_SOCKET_ID			("socket_id")

static const char *cryptodev_vdev_valid_params[] = {
	RTE_CRYPTODEV_VDEV_NAME,
	RTE_CRYPTODEV_VDEV_MAX_NB_QP_ARG,
	RTE_CRYPTODEV_VDEV_MAX_NB_SESS_ARG,
	RTE_CRYPTODEV_VDEV_SOCKET_ID
};

/**
 * @internal
 * Initialisation parameters for virtual crypto devices
 */
struct rte_crypto_vdev_init_params {
	unsigned int max_nb_queue_pairs;
	unsigned int max_nb_sessions;
	uint8_t socket_id;
	char name[RTE_CRYPTODEV_NAME_MAX_LEN];
};

/**
 * @internal
 * Parse name from argument
 */
static int
parse_name_arg(const char *key __rte_unused,
		const char *value, void *extra_args)
{
	struct rte_crypto_vdev_init_params *params = extra_args;

	if (strlen(value) >= RTE_CRYPTODEV_NAME_MAX_LEN - 1) {
		CDEV_LOG_ERR("Invalid name %s, should be less than "
				"%u bytes", value,
				RTE_CRYPTODEV_NAME_MAX_LEN - 1);
		return -1;
	}

	strncpy(params->name, value, RTE_CRYPTODEV_NAME_MAX_LEN);

	return 0;
}

/**
 * @internal
 * Parse integer from argument
 */
static int
parse_integer_arg(const char *key __rte_unused,
		const char *value, void *extra_args)
{
	int *i = extra_args;

	*i = atoi(value);
	if (*i < 0) {
		CDEV_LOG_ERR("Argument has to be positive.");
		return -1;
	}

	return 0;
}

/**
 * @internal
 * Return number of sockets with memory reserved
 */
static uint8_t
number_of_sockets(void)
{
	int sockets = 0;
	int i;
	const struct rte_memseg *ms = rte_eal_get_physmem_layout();

	for (i = 0; ((i < RTE_MAX_MEMSEG) && (ms[i].addr != NULL)); i++) {
		if (sockets < ms[i].socket_id)
			sockets = ms[i].socket_id;
	}

	/* Number of sockets = maximum socket_id + 1 */
	return ++sockets;
}

/**
 * @internal
 * Creates a new virtual crypto device and returns the pointer
 * to that device.
 *
 * @param	name			PMD type name
 * @param	dev_private_size	Size of crypto PMDs private data
 * @param	socket_id		Socket to allocate resources on.
 * @param	vdev			Pointer to virtual device structure.
 *
 * @return
 *   - Cryptodev pointer if device is successfully created.
 *   - NULL if device cannot be created.
 */
static inline struct rte_cryptodev *
rte_cryptodev_pmd_virtual_dev_init(const char *name, size_t dev_private_size,
		int socket_id, struct rte_vdev_device *vdev)
{
	struct rte_cryptodev *cryptodev;

	/* allocate device structure */
	cryptodev = rte_cryptodev_pmd_allocate(name, socket_id);
	if (cryptodev == NULL)
		return NULL;

	/* allocate private device structure */
	if (rte_eal_process_type() == RTE_PROC_PRIMARY) {
		cryptodev->data->dev_private =
				rte_zmalloc_socket("cryptodev device private",
						dev_private_size,
						RTE_CACHE_LINE_SIZE,
						socket_id);

		if (cryptodev->data->dev_private == NULL)
			rte_panic("Cannot allocate memzone for private device"
					" data");
	}

	cryptodev->device = &vdev->device;

	/* initialise user call-back tail queue */
	TAILQ_INIT(&(cryptodev->link_intr_cbs));

	return cryptodev;
}

/**
 * @internal
 * Parse virtual device initialisation parameters input arguments
 *
 * @params	params		Initialisation parameters with defaults set.
 * @params	input_args	Command line arguments
 *
 * @return
 * 0 on successful parse
 * <0 on failure to parse
 */
static inline int
rte_cryptodev_parse_vdev_init_params(struct rte_crypto_vdev_init_params *params,
		const char *input_args)
{
	struct rte_kvargs *kvlist = NULL;
	int ret = 0;

	if (params == NULL)
		return -EINVAL;

	if (input_args) {
		kvlist = rte_kvargs_parse(input_args,
				cryptodev_vdev_valid_params);
		if (kvlist == NULL)
			return -1;

		ret = rte_kvargs_process(kvlist,
					RTE_CRYPTODEV_VDEV_MAX_NB_QP_ARG,
					&parse_integer_arg,
					&params->max_nb_queue_pairs);
		if (ret < 0)
			goto free_kvlist;

		ret = rte_kvargs_process(kvlist,
					RTE_CRYPTODEV_VDEV_MAX_NB_SESS_ARG,
					&parse_integer_arg,
					&params->max_nb_sessions);
		if (ret < 0)
			goto free_kvlist;

		ret = rte_kvargs_process(kvlist, RTE_CRYPTODEV_VDEV_SOCKET_ID,
					&parse_integer_arg,
					&params->socket_id);
		if (ret < 0)
			goto free_kvlist;

		ret = rte_kvargs_process(kvlist, RTE_CRYPTODEV_VDEV_NAME,
					&parse_name_arg,
					params);
		if (ret < 0)
			goto free_kvlist;

		if (params->socket_id >= number_of_sockets()) {
			CDEV_LOG_ERR("Invalid socket id specified to create "
				"the virtual crypto device on");
			goto free_kvlist;
		}
	}

free_kvlist:
	rte_kvargs_free(kvlist);
	return ret;
}

#endif /* _RTE_CRYPTODEV_VDEV_H_ */
