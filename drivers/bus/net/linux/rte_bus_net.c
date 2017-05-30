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

#include <string.h>
#include <dirent.h>
#include <stdarg.h>
#include <unistd.h>

#include <linux/limits.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <rte_bus.h>

#include "rte_bus_net.h"
#include "private.h"

/* Default path */
const char SYSFS_NET_DEVICES[] = "/sys/class/net";

int
rte_bus_net_path_read(char *out, size_t size,
		      const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	char path[vsnprintf(NULL, 0, format, ap) + 1];
	FILE *file;
	int ret;
	int err;

	va_end(ap);
	va_start(ap, format);
	vsnprintf(path, sizeof(path), format, ap);
	file = fopen(path, "rb");
	if (file == NULL) {
		ERROR("failed to open file %s (%s)",
		      path, strerror(errno));
		va_end(ap);
		return -1;
	}
	ret = fread(out, size, 1, file);
	err = errno;
	if (((size_t)ret < size) && (ferror(file))) {
		va_end(ap);
		ret = -1;
	}
	fclose(file);
	errno = err;
	va_end(ap);
	return ret;
}

static int
parse_u32(const char *val, uint32_t *out)
{
	char *end;
	uint64_t tmp;

	errno = 0;
	tmp = strtoull(val, &end, 0);
	if (errno != 0 || val[0] == '\0' || end == NULL)
		return -EINVAL;
	if (tmp > UINT32_MAX)
		return -ERANGE;
	*out = (uint32_t) tmp;
	return 0;
}

static uint32_t
dev_flags(const char *name)
{
	char buf[32] = "";
	uint32_t val;

	if (rte_bus_net_path_read(buf, sizeof(buf), "%s/%s/flags",
				  net_get_sysfs_path(), name)) {
		ERROR("failed to read flags on device %s", name);
		return 0;
	}
	if (parse_u32(buf, &val)) {
		ERROR("failed to parse %s", buf);
		return 0;
	}
	return val;
}

int
net_scan(void)
{
	struct dirent *e;
	DIR *dir;
	const char *path;
	int ret = 0;
	int scan_blist = 0;

	path = net_get_sysfs_path();
	dir = opendir(path);
	if (dir == NULL) {
		ret = errno;
		ERROR("opendir %s failed: %s", path, strerror(errno));
		return -1;
	}
	if (rte_bus_net.conf.scan_mode == RTE_BUS_SCAN_BLACKLIST)
		scan_blist = 1;
	while ((e = readdir(dir)) != NULL) {
		if (e->d_name[0] == '.')
			continue;
		if (dev_flags(e->d_name) & 0x8)
			continue;
		if (scan_blist ^
		    !net_devargs_lookup(e->d_name))
			continue;
		/* Scan errors are ignored in blacklist mode. */
		if (!net_scan_one(e->d_name) && !scan_blist)
			goto out;
	}
out:
	closedir(dir);
	return ret;
}

int
rte_bus_net_dev_stat(const char *name)
{
	struct stat buf;
	int err;
	MKSTR(path, "%s/%s", net_get_sysfs_path(), name);

	if (name[0] == '\0')
		return -1;
	snprintf(path, sizeof(path), "%s/%s",
		 net_get_sysfs_path(), name);
	err = errno;
	errno = 0;
	if (stat(path, &buf)) {
		if (errno != ENOENT)
			ERROR("stat(%s) failed: %s", path, strerror(errno));
		errno = err;
		return -1;
	}
	return 0;
}

static int
link_read(const char *path, char *out, size_t size)
{
	struct stat buf;
	int err;

	err = errno;
	errno = 0;
	if (stat(path, &buf)) {
		if (errno != ENOENT)
			ERROR("stat(%s) failed: %s", path, strerror(errno));
		errno = err;
		return -1;
	}
	if (size > 0) {
		char buf[PATH_MAX + 1] = "";
		ssize_t wlen;
		char *s;

		wlen = readlink(path, buf, sizeof(buf) - 1);
		if (wlen > 0) {
			out[wlen] = '\0';
		} else {
			ERROR("readlink(%s) failed: %s", path,
			      strerror(errno));
			errno = err;
			return -1;
		}
		for (s = &buf[wlen]; *s != '/'; s--)
			;
		snprintf(out, size, "%s", &s[1]);
	}
	return 0;
}

int
rte_bus_net_driver(const char *name, char *out, size_t size)
{
	MKSTR(path, "%s/%s/device/driver", net_get_sysfs_path(), name);

	return link_read(path, out, size);
}

int
rte_bus_net_pci_loc(const char *name, char *out, size_t size)
{
	MKSTR(path, "%s/%s/device", net_get_sysfs_path(), name);

	return link_read(path, out, size);
}
