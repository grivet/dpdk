#   BSD LICENSE
#
#   Copyright(c) 2017 6WIND S.A.
#   Author: Gaetan Rivet
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of 6WIND nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

include $(RTE_SDK)/mk/rte.vars.mk

LIB = librte_bus_net.a
LIBABIVER := 1

CFLAGS += $(WERROR_FLAGS)
ifeq ($(DEBUG),1)
CONFIG_RTE_LIBRTE_NET_BUS_DEBUG := y
endif
ifeq ($(CONFIG_RTE_LIBRTE_NET_BUS_DEBUG),y)
CFLAGS += -UNDEBUG -g -O0
else
CFLAGS += -DNDEBUG -O3
endif
CFLAGS += -I$(RTE_SDK)/drivers/bus/net
CFLAGS += -I$(RTE_SDK)/drivers/bus/net/include

ifneq ($(CONFIG_RTE_EXEC_ENV_LINUXAPP),)
SYSTEM := linux
endif
ifneq ($(CONFIG_RTE_EXEC_ENV_BDSAPP),)
SYSTEM := bsd
endif

EXPORT_MAP := $(SYSTEM)/rte_bus_net_version.map
CFLAGS += -I$(RTE_SDK)/drivers/bus/net/$(SYSTEM)

include $(RTE_SDK)/drivers/bus/net/$(SYSTEM)/Makefile
SRCS-$(CONFIG_RTE_LIBRTE_NET_BUS) := $(addprefix $(SYSTEM)/,$(SRCS))
SRCS-$(CONFIG_RTE_LIBRTE_NET_BUS) += rte_bus_net.c

SYMLINK-$(CONFIG_RTE_LIBRTE_PCI_BUS)-include += include/rte_bus_net.h

include $(RTE_SDK)/mk/rte.lib.mk
