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

#ifndef _RTE_CRYPTODEV_PCI_H_
#define _RTE_CRYPTODEV_PCI_H_

#include <rte_pci.h>
#include <rte_cryptodev.h>
#include <rte_cryptodev_pmd.h>

/**
 * Initialisation function of a crypto driver invoked for each matching
 * crypto PCI device detected during the PCI probing phase.
 *
 * @param	dev	The dev pointer is the address of the *rte_cryptodev*
 *			structure associated with the matching device and which
 *			has been [automatically] allocated in the
 *			*rte_crypto_devices* array.
 *
 * @return
 *   - 0: Success, the device is properly initialised by the driver.
 *        In particular, the driver MUST have set up the *dev_ops* pointer
 *        of the *dev* structure.
 *   - <0: Error code of the device initialisation failure.
 */
typedef int (*cryptodev_pci_init_t)(struct rte_cryptodev *dev);

/**
 * Finalisation function of a driver invoked for each matching
 * PCI device detected during the PCI closing phase.
 *
 * @param	dev	The dev pointer is the address of the *rte_cryptodev*
 *			structure associated with the matching device and which
 *			has been [automatically] allocated in the
 *			*rte_crypto_devices* array.
 *
 *  * @return
 *   - 0: Success, the device is properly finalised by the driver.
 *        In particular, the driver MUST free the *dev_ops* pointer
 *        of the *dev* structure.
 *   - <0: Error code of the device initialisation failure.
 */
typedef int (*cryptodev_pci_uninit_t)(struct rte_cryptodev *dev);

/**
 * @internal
 * Wrapper for use by pci drivers as a .probe function to attach to a crypto
 * interface.
 */
static inline int
rte_cryptodev_pci_generic_probe(struct rte_pci_device *pci_dev,
			size_t private_data_size,
			cryptodev_pci_init_t dev_init)
{
	struct rte_cryptodev *cryptodev;

	char cryptodev_name[RTE_CRYPTODEV_NAME_MAX_LEN];

	int retval;

	rte_pci_device_name(&pci_dev->addr, cryptodev_name,
			sizeof(cryptodev_name));

	cryptodev = rte_cryptodev_pmd_allocate(cryptodev_name, rte_socket_id());
	if (cryptodev == NULL)
		return -ENOMEM;

	if (rte_eal_process_type() == RTE_PROC_PRIMARY) {
		cryptodev->data->dev_private =
				rte_zmalloc_socket(
						"cryptodev private structure",
						private_data_size,
						RTE_CACHE_LINE_SIZE,
						rte_socket_id());

		if (cryptodev->data->dev_private == NULL)
			rte_panic("Cannot allocate memzone for private "
					"device data");
	}

	cryptodev->device = &pci_dev->device;

	/* init user callbacks */
	TAILQ_INIT(&(cryptodev->link_intr_cbs));

	/* Invoke PMD device initialization function */
	RTE_FUNC_PTR_OR_ERR_RET(*dev_init, -EINVAL);
	retval = dev_init(cryptodev);
	if (retval == 0)
		return 0;

	CDEV_LOG_ERR("driver %s: crypto_dev_init(vendor_id=0x%x device_id=0x%x)"
			" failed", pci_dev->device.driver->name,
			(unsigned int) pci_dev->id.vendor_id,
			(unsigned int) pci_dev->id.device_id);

	if (rte_eal_process_type() == RTE_PROC_PRIMARY)
		rte_free(cryptodev->data->dev_private);

	/* free crypto device */
	rte_cryptodev_pmd_release_device(cryptodev);

	return -ENXIO;
}

/**
 * @internal
 * Wrapper for use by pci drivers as a .remove function to detach a crypto
 * interface.
 */
static inline int
rte_cryptodev_pci_generic_remove(struct rte_pci_device *pci_dev,
		cryptodev_pci_uninit_t dev_uninit)
{
	struct rte_cryptodev *cryptodev;
	char cryptodev_name[RTE_CRYPTODEV_NAME_MAX_LEN];
	int ret;

	if (pci_dev == NULL)
		return -EINVAL;

	rte_pci_device_name(&pci_dev->addr, cryptodev_name,
			sizeof(cryptodev_name));

	cryptodev = rte_cryptodev_pmd_get_named_dev(cryptodev_name);
	if (cryptodev == NULL)
		return -ENODEV;

	/* Invoke PMD device uninit function */
	if (dev_uninit) {
		ret = dev_uninit(cryptodev);
		if (ret)
			return ret;
	}

	/* free crypto device */
	rte_cryptodev_pmd_release_device(cryptodev);

	if (rte_eal_process_type() == RTE_PROC_PRIMARY)
		rte_free(cryptodev->data->dev_private);

	cryptodev->device = NULL;
	cryptodev->data = NULL;

	return 0;
}

#endif /* _RTE_CRYPTODEV_PCI_H_ */
