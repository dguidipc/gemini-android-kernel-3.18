/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2013 ARM Limited
 */

#ifndef __ASM_PSCI_H
#define __ASM_PSCI_H

struct psci_power_state {
	u16	id;
	u8	type;
	u8	affinity_level;
};

enum psci_conduit {
	PSCI_CONDUIT_NONE,
	PSCI_CONDUIT_SMC,
	PSCI_CONDUIT_HVC,
};

enum smccc_version {
	SMCCC_VERSION_1_0,
	SMCCC_VERSION_1_1,
};

struct psci_operations {
	int (*get_version)(void);
	int (*cpu_suspend)(struct psci_power_state state,
			   unsigned long entry_point);
	int (*cpu_off)(struct psci_power_state state);
	int (*cpu_on)(unsigned long cpuid, unsigned long entry_point);
	int (*migrate)(unsigned long cpuid);
	int (*affinity_info)(unsigned long target_affinity,
			unsigned long lowest_affinity_level);
	int (*migrate_info_type)(void);
	enum psci_conduit conduit;
	enum smccc_version smccc_version;
};

extern struct psci_operations psci_ops;

int psci_init(void);

#endif /* __ASM_PSCI_H */
