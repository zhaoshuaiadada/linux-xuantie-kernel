# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_BPF_JIT) += bpf_jit_core.o

ifeq ($(CONFIG_ARCH_RV32I),y)
	obj-$(CONFIG_BPF_JIT) += bpf_jit_comp32.o
else
	obj-$(CONFIG_BPF_JIT) += bpf_jit_comp64.o
endif
