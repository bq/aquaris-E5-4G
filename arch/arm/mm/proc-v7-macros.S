/*
 * linux/arch/arm/mm/proc-v7-macros.S
 *
 * Copyright (C) 2011 ARM Ltd
 *
 * Setup macro for the v7 architecture (both classic pages and LPAE)
 */

#define PMD_FLAGS_UP	(PMD_SECT_WB)
#define PMD_FLAGS_SMP	(PMD_SECT_WBWA|PMD_SECT_S)

.macro __v7_proc initfunc, mm_mmuflags = 0, io_mmuflags = 0, hwcaps = 0
	ALT_SMP(.long	PMD_TYPE_SECT | PMD_SECT_AP_WRITE | PMD_SECT_AP_READ | \
			PMD_FLAGS_SMP | \mm_mmuflags)
	ALT_UP(.long	PMD_TYPE_SECT | PMD_SECT_AP_WRITE | PMD_SECT_AP_READ | \
			PMD_FLAGS_UP | \mm_mmuflags)
	.long	PMD_TYPE_SECT | PMD_SECT_XN | PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ | \io_mmuflags
	W(b)	\initfunc
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP | HWCAP_HALF | HWCAP_THUMB | HWCAP_FAST_MULT | \
		HWCAP_EDSP | HWCAP_TLS | \hwcaps
	.long	cpu_v7_name
	.long	v7_processor_functions
	.long	v7wbi_tlb_fns
	.long	v6_user_fns
	.long	v7_cache_fns
.endm
