/*
 * IOMMU API for ARM architected SMMU implementations.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) 2013 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 *
 * This driver currently supports:
 *	- SMMUv1 and v2 implementations
 *	- Stream-matching and stream-indexing
 *	- v7/v8 long-descriptor format
 *	- Non-secure access to the SMMU
 *	- 4k and 64k pages, with contiguous pte hints.
 *	- Up to 48-bit addressing (dependent on VA_BITS)
 *	- Context fault reporting
 */

#define pr_fmt(fmt) "arm-smmu: " fmt

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/amba/bus.h>

#include <asm/pgalloc.h>

/* Maximum number of stream IDs assigned to a single device */
#define MAX_MASTER_STREAMIDS		MAX_PHANDLE_ARGS

/* Maximum number of context banks per SMMU */
#define ARM_SMMU_MAX_CBS		128

/* Maximum number of mapping groups per SMMU */
#define ARM_SMMU_MAX_SMRS		128

/* SMMU global address space */
#define ARM_SMMU_GR0(smmu)		((smmu)->base)
#define ARM_SMMU_GR1(smmu)		((smmu)->base + (1 << (smmu)->pgshift))

/*
 * SMMU global address space with conditional offset to access secure
 * aliases of non-secure registers (e.g. nsCR0: 0x400, nsGFSR: 0x448,
 * nsGFSYNR0: 0x450)
 */
#define ARM_SMMU_GR0_NS(smmu)						\
	((smmu)->base +							\
		((smmu->options & ARM_SMMU_OPT_SECURE_CFG_ACCESS)	\
			? 0x400 : 0))

/* Page table bits */
#define ARM_SMMU_PTE_XN			(((pteval_t)3) << 53)
#define ARM_SMMU_PTE_CONT		(((pteval_t)1) << 52)
#define ARM_SMMU_PTE_AF			(((pteval_t)1) << 10)
#define ARM_SMMU_PTE_SH_NS		(((pteval_t)0) << 8)
#define ARM_SMMU_PTE_SH_OS		(((pteval_t)2) << 8)
#define ARM_SMMU_PTE_SH_IS		(((pteval_t)3) << 8)
#define ARM_SMMU_PTE_PAGE		(((pteval_t)3) << 0)

#if PAGE_SIZE == SZ_4K
#define ARM_SMMU_PTE_CONT_ENTRIES	16
#elif PAGE_SIZE == SZ_64K
#define ARM_SMMU_PTE_CONT_ENTRIES	32
#else
#define ARM_SMMU_PTE_CONT_ENTRIES	1
#endif

#define ARM_SMMU_PTE_CONT_SIZE		(PAGE_SIZE * ARM_SMMU_PTE_CONT_ENTRIES)
#define ARM_SMMU_PTE_CONT_MASK		(~(ARM_SMMU_PTE_CONT_SIZE - 1))

/* Stage-1 PTE */
#define ARM_SMMU_PTE_AP_UNPRIV		(((pteval_t)1) << 6)
#define ARM_SMMU_PTE_AP_RDONLY		(((pteval_t)2) << 6)
#define ARM_SMMU_PTE_ATTRINDX_SHIFT	2
#define ARM_SMMU_PTE_nG			(((pteval_t)1) << 11)

/* Stage-2 PTE */
#define ARM_SMMU_PTE_HAP_FAULT		(((pteval_t)0) << 6)
#define ARM_SMMU_PTE_HAP_READ		(((pteval_t)1) << 6)
#define ARM_SMMU_PTE_HAP_WRITE		(((pteval_t)2) << 6)
#define ARM_SMMU_PTE_MEMATTR_OIWB	(((pteval_t)0xf) << 2)
#define ARM_SMMU_PTE_MEMATTR_NC		(((pteval_t)0x5) << 2)
#define ARM_SMMU_PTE_MEMATTR_DEV	(((pteval_t)0x1) << 2)

/* Configuration registers */
#define ARM_SMMU_GR0_sCR0		0x0
#define sCR0_CLIENTPD			(1 << 0)
#define sCR0_GFRE			(1 << 1)
#define sCR0_GFIE			(1 << 2)
#define sCR0_GCFGFRE			(1 << 4)
#define sCR0_GCFGFIE			(1 << 5)
#define sCR0_USFCFG			(1 << 10)
#define sCR0_VMIDPNE			(1 << 11)
#define sCR0_PTM			(1 << 12)
#define sCR0_FB				(1 << 13)
#define sCR0_BSU_SHIFT			14
#define sCR0_BSU_MASK			0x3

/* Identification registers */
#define ARM_SMMU_GR0_ID0		0x20
#define ARM_SMMU_GR0_ID1		0x24
#define ARM_SMMU_GR0_ID2		0x28
#define ARM_SMMU_GR0_ID3		0x2c
#define ARM_SMMU_GR0_ID4		0x30
#define ARM_SMMU_GR0_ID5		0x34
#define ARM_SMMU_GR0_ID6		0x38
#define ARM_SMMU_GR0_ID7		0x3c
#define ARM_SMMU_GR0_sGFSR		0x48
#define ARM_SMMU_GR0_sGFSYNR0		0x50
#define ARM_SMMU_GR0_sGFSYNR1		0x54
#define ARM_SMMU_GR0_sGFSYNR2		0x58
#define ARM_SMMU_GR0_PIDR0		0xfe0
#define ARM_SMMU_GR0_PIDR1		0xfe4
#define ARM_SMMU_GR0_PIDR2		0xfe8

#define ID0_S1TS			(1 << 30)
#define ID0_S2TS			(1 << 29)
#define ID0_NTS				(1 << 28)
#define ID0_SMS				(1 << 27)
#define ID0_ATOSNS			(1 << 26)
#define ID0_PTFS_SHIFT			24
#define ID0_PTFS_MASK			0x2
#define ID0_PTFS_V8_ONLY		0x2
#define ID0_NUMIRPT_SHIFT		16
#define ID0_NUMIRPT_MASK		0xff
#define ID0_CTTW			(1 << 14)
#define ID0_BTM				(1 << 13)
#define ID0_NUMSIDB_SHIFT		9
#define ID0_NUMSIDB_MASK		0xf
#define ID0_EXIDS			(1 << 8)
#define ID0_NUMSMRG_SHIFT		0
#define ID0_NUMSMRG_MASK		0xff

#define ID1_PAGESIZE			(1 << 31)
#define ID1_NUMPAGENDXB_SHIFT		28
#define ID1_NUMPAGENDXB_MASK		7
#define ID1_NUMS2CB_SHIFT		16
#define ID1_NUMS2CB_MASK		0xff
#define ID1_SMCD			(1 << 15)
#define ID1_NUMCB_SHIFT			0
#define ID1_NUMCB_MASK			0xff

#define ID2_OAS_SHIFT			4
#define ID2_OAS_MASK			0xf
#define ID2_IAS_SHIFT			0
#define ID2_IAS_MASK			0xf
#define ID2_UBS_SHIFT			8
#define ID2_UBS_MASK			0xf
#define ID2_PTFS_4K			(1 << 12)
#define ID2_PTFS_16K			(1 << 13)
#define ID2_PTFS_64K			(1 << 14)

#define PIDR2_ARCH_SHIFT		4
#define PIDR2_ARCH_MASK			0xf

/* Global TLB invalidation */
#define ARM_SMMU_GR0_TLBIVMID		0x64
#define ARM_SMMU_GR0_TLBIALLNSNH	0x68
#define ARM_SMMU_GR0_TLBIALLH		0x6c
#define ARM_SMMU_GR0_sTLBGSYNC		0x70
#define ARM_SMMU_GR0_sTLBGSTATUS	0x74
#define sTLBGSTATUS_GSACTIVE		(1 << 0)
#define TLB_LOOP_TIMEOUT		1000000	/* 1s! */

/* Stream mapping registers */
#define ARM_SMMU_GR0_SMR(n)		(0x800 + ((n) << 2))
#define SMR_VALID			(1 << 31)
#define SMR_MASK_SHIFT			16
#define SMR_MASK_MASK			0x7fff
#define SMR_ID_SHIFT			0
#define SMR_ID_MASK			0x7fff

#define ARM_SMMU_GR0_S2CR(n)		(0xc00 + ((n) << 2))
#define S2CR_CBNDX_SHIFT		0
#define S2CR_CBNDX_MASK			0xff
#define S2CR_TYPE_SHIFT			16
#define S2CR_TYPE_MASK			0x3
#define S2CR_TYPE_TRANS			(0 << S2CR_TYPE_SHIFT)
#define S2CR_TYPE_BYPASS		(1 << S2CR_TYPE_SHIFT)
#define S2CR_TYPE_FAULT			(2 << S2CR_TYPE_SHIFT)

/* Context bank attribute registers */
#define ARM_SMMU_GR1_CBAR(n)		(0x0 + ((n) << 2))
#define CBAR_VMID_SHIFT			0
#define CBAR_VMID_MASK			0xff
#define CBAR_S1_BPSHCFG_SHIFT		8
#define CBAR_S1_BPSHCFG_MASK		3
#define CBAR_S1_BPSHCFG_NSH		3
#define CBAR_S1_S2_CBNDX_SHIFT		8
#define CBAR_S1_S2_CBNDX_MASK		0xff
#define CBAR_S1_HYPC			(1 << 10)
#define CBAR_S1_MEMATTR_SHIFT		12
#define CBAR_S1_MEMATTR_MASK		0xf
#define CBAR_S1_MEMATTR_WB		0xf
#define CBAR_TYPE_SHIFT			16
#define CBAR_TYPE_MASK			0x3
#define CBAR_TYPE_S2_TRANS		(0 << CBAR_TYPE_SHIFT)
#define CBAR_TYPE_S1_TRANS_S2_BYPASS	(1 << CBAR_TYPE_SHIFT)
#define CBAR_TYPE_S1_TRANS_S2_FAULT	(2 << CBAR_TYPE_SHIFT)
#define CBAR_TYPE_S1_TRANS_S2_TRANS	(3 << CBAR_TYPE_SHIFT)
#define CBAR_S1_BSU_SHIFT		18
#define CBAR_S1_BSU_MASK		3
#define CBAR_IRPTNDX_SHIFT		24
#define CBAR_IRPTNDX_MASK		0xff

/* Just rolls off the tongue... */
#define ARM_SMMU_GR1_CBFRSYNRA(n)	(0x400 + ((n) << 2))

#define ARM_SMMU_GR1_CBA2R(n)		(0x800 + ((n) << 2))
#define CBA2R_RW64_32BIT		(0 << 0)
#define CBA2R_RW64_64BIT		(1 << 0)

/* Translation context bank */
#define ARM_SMMU_CB_BASE(smmu)		((smmu)->base + ((smmu)->size >> 1))
#define ARM_SMMU_CB(smmu, n)		((n) * (1 << (smmu)->pgshift))

#define ARM_SMMU_CB_SCTLR		0x0
#define ARM_SMMU_CB_ACTLR		0x4
#define ARM_SMMU_CB_RESUME		0x8
#define ARM_SMMU_CB_TTBCR2		0x10
#define ARM_SMMU_CB_TTBR0_LO		0x20
#define ARM_SMMU_CB_TTBR0_HI		0x24
#define ARM_SMMU_CB_TTBCR		0x30
#define ARM_SMMU_CB_S1_MAIR0		0x38
#define ARM_SMMU_CB_FSR			0x58
#define ARM_SMMU_CB_FAR_LO		0x60
#define ARM_SMMU_CB_FAR_HI		0x64
#define ARM_SMMU_CB_FSYNR0		0x68
#define ARM_SMMU_CB_S1_TLBIASID		0x610

#define SCTLR_S1_ASIDPNE		(1 << 12)
#define SCTLR_CFCFG			(1 << 7)
#define SCTLR_CFIE			(1 << 6)
#define SCTLR_CFRE			(1 << 5)
#define SCTLR_E				(1 << 4)
#define SCTLR_AFE			(1 << 2)
#define SCTLR_TRE			(1 << 1)
#define SCTLR_M				(1 << 0)
#define SCTLR_EAE_SBOP			(SCTLR_AFE | SCTLR_TRE)

#define RESUME_RETRY			(0 << 0)
#define RESUME_TERMINATE		(1 << 0)

#define TTBCR_EAE			(1 << 31)

#define TTBCR_PASIZE_SHIFT		16
#define TTBCR_PASIZE_MASK		0x7

#define TTBCR_TG0_4K			(0 << 14)
#define TTBCR_TG0_64K			(1 << 14)

#define TTBCR_SH0_SHIFT			12
#define TTBCR_SH0_MASK			0x3
#define TTBCR_SH_NS			0
#define TTBCR_SH_OS			2
#define TTBCR_SH_IS			3

#define TTBCR_ORGN0_SHIFT		10
#define TTBCR_IRGN0_SHIFT		8
#define TTBCR_RGN_MASK			0x3
#define TTBCR_RGN_NC			0
#define TTBCR_RGN_WBWA			1
#define TTBCR_RGN_WT			2
#define TTBCR_RGN_WB			3

#define TTBCR_SL0_SHIFT			6
#define TTBCR_SL0_MASK			0x3
#define TTBCR_SL0_LVL_2			0
#define TTBCR_SL0_LVL_1			1

#define TTBCR_T1SZ_SHIFT		16
#define TTBCR_T0SZ_SHIFT		0
#define TTBCR_SZ_MASK			0xf

#define TTBCR2_SEP_SHIFT		15
#define TTBCR2_SEP_MASK			0x7

#define TTBCR2_PASIZE_SHIFT		0
#define TTBCR2_PASIZE_MASK		0x7

/* Common definitions for PASize and SEP fields */
#define TTBCR2_ADDR_32			0
#define TTBCR2_ADDR_36			1
#define TTBCR2_ADDR_40			2
#define TTBCR2_ADDR_42			3
#define TTBCR2_ADDR_44			4
#define TTBCR2_ADDR_48			5

#define TTBRn_HI_ASID_SHIFT		16

#define MAIR_ATTR_SHIFT(n)		((n) << 3)
#define MAIR_ATTR_MASK			0xff
#define MAIR_ATTR_DEVICE		0x04
#define MAIR_ATTR_NC			0x44
#define MAIR_ATTR_WBRWA			0xff
#define MAIR_ATTR_IDX_NC		0
#define MAIR_ATTR_IDX_CACHE		1
#define MAIR_ATTR_IDX_DEV		2

#define FSR_MULTI			(1 << 31)
#define FSR_SS				(1 << 30)
#define FSR_UUT				(1 << 8)
#define FSR_ASF				(1 << 7)
#define FSR_TLBLKF			(1 << 6)
#define FSR_TLBMCF			(1 << 5)
#define FSR_EF				(1 << 4)
#define FSR_PF				(1 << 3)
#define FSR_AFF				(1 << 2)
#define FSR_TF				(1 << 1)

#define FSR_IGN				(FSR_AFF | FSR_ASF | \
					 FSR_TLBMCF | FSR_TLBLKF)
#define FSR_FAULT			(FSR_MULTI | FSR_SS | FSR_UUT | \
					 FSR_EF | FSR_PF | FSR_TF | FSR_IGN)

#define FSYNR0_WNR			(1 << 4)

static int force_stage;
module_param_named(force_stage, force_stage, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(force_stage,
	"Force SMMU mappings to be installed at a particular stage of translation. A value of '1' or '2' forces the corresponding stage. All other values are ignored (i.e. no stage is forced). Note that selecting a specific stage will disable support for nested translation.");

enum arm_smmu_arch_version {
	ARM_SMMU_V1 = 1,
	ARM_SMMU_V2,
};

struct arm_smmu_smr {
	u8				idx;
	u16				mask;
	u16				id;
};

struct arm_smmu_master_cfg {
	int				num_streamids;
	u16				streamids[MAX_MASTER_STREAMIDS];
	struct arm_smmu_smr		*smrs;
};

struct arm_smmu_master {
	struct device_node		*of_node;
	struct rb_node			node;
	struct arm_smmu_master_cfg	cfg;
};

struct arm_smmu_device {
	struct device			*dev;

	void __iomem			*base;
	unsigned long			size;
	unsigned long			pgshift;

	u32				idr[3];
#define ARM_SMMU_FEAT_COHERENT_WALK	(1 << 0)
#define ARM_SMMU_FEAT_STREAM_MATCH	(1 << 1)
#define ARM_SMMU_FEAT_TRANS_S1		(1 << 2)
#define ARM_SMMU_FEAT_TRANS_S2		(1 << 3)
#define ARM_SMMU_FEAT_TRANS_NESTED	(1 << 4)
	u32				features;

#define ARM_SMMU_OPT_SECURE_CFG_ACCESS (1 << 0)
	u32				options;
	enum arm_smmu_arch_version	version;

	u32				num_context_banks;
	u32				num_s2_context_banks;
	DECLARE_BITMAP(context_map, ARM_SMMU_MAX_CBS);
	atomic_t			irptndx;

	u32				num_mapping_groups;
	DECLARE_BITMAP(smr_map, ARM_SMMU_MAX_SMRS);

	unsigned long			s1_input_size;
	unsigned long			s1_output_size;
	unsigned long			s2_input_size;
	unsigned long			s2_output_size;

	u32				num_global_irqs;
	u32				num_context_irqs;
	unsigned int			*irqs;

	struct list_head		list;
	struct rb_root			masters;
};

struct arm_smmu_cfg {
	u8				cbndx;
	u8				irptndx;
	u32				cbar;
	pgd_t				*pgd;
};
#define INVALID_IRPTNDX			0xff

#define ARM_SMMU_CB_ASID(cfg)		((cfg)->cbndx)
#define ARM_SMMU_CBNDX_TO_VMID(cbndx)	((cbndx) + 1)
#define ARM_SMMU_CB_VMID(cfg)		ARM_SMMU_CBNDX_TO_VMID((cfg)->cbndx)

enum arm_smmu_domain_stage {
	ARM_SMMU_DOMAIN_S1 = 0,
	ARM_SMMU_DOMAIN_S2,
	ARM_SMMU_DOMAIN_NESTED,
};

struct arm_smmu_domain {
	struct arm_smmu_device		*smmu;
	struct arm_smmu_cfg		cfg;
	enum arm_smmu_domain_stage	stage;
	spinlock_t			lock;
};

static DEFINE_SPINLOCK(arm_smmu_devices_lock);
static LIST_HEAD(arm_smmu_devices);

struct arm_smmu_option_prop {
	u32 opt;
	const char *prop;
};

static struct arm_smmu_option_prop arm_smmu_options[] = {
	{ ARM_SMMU_OPT_SECURE_CFG_ACCESS, "calxeda,smmu-secure-config-access" },
	{ 0, NULL},
};

static void parse_driver_options(struct arm_smmu_device *smmu)
{
	int i = 0;

	do {
		if (of_property_read_bool(smmu->dev->of_node,
						arm_smmu_options[i].prop)) {
			smmu->options |= arm_smmu_options[i].opt;
			dev_notice(smmu->dev, "option %s\n",
				arm_smmu_options[i].prop);
		}
	} while (arm_smmu_options[++i].opt);
}

static struct device_node *dev_get_dev_node(struct device *dev)
{
	if (dev_is_pci(dev)) {
		struct pci_bus *bus = to_pci_dev(dev)->bus;

		while (!pci_is_root_bus(bus))
			bus = bus->parent;
		return bus->bridge->parent->of_node;
	}

	return dev->of_node;
}

static struct arm_smmu_master *find_smmu_master(struct arm_smmu_device *smmu,
						struct device_node *dev_node)
{
	struct rb_node *node = smmu->masters.rb_node;

	while (node) {
		struct arm_smmu_master *master;

		master = container_of(node, struct arm_smmu_master, node);

		if (dev_node < master->of_node)
			node = node->rb_left;
		else if (dev_node > master->of_node)
			node = node->rb_right;
		else
			return master;
	}

	return NULL;
}

static struct arm_smmu_master_cfg *
find_smmu_master_cfg(struct device *dev)
{
	struct arm_smmu_master_cfg *cfg = NULL;
	struct iommu_group *group = iommu_group_get(dev);

	if (group) {
		cfg = iommu_group_get_iommudata(group);
		iommu_group_put(group);
	}

	return cfg;
}

static int insert_smmu_master(struct arm_smmu_device *smmu,
			      struct arm_smmu_master *master)
{
	struct rb_node **new, *parent;

	new = &smmu->masters.rb_node;
	parent = NULL;
	while (*new) {
		struct arm_smmu_master *this
			= container_of(*new, struct arm_smmu_master, node);

		parent = *new;
		if (master->of_node < this->of_node)
			new = &((*new)->rb_left);
		else if (master->of_node > this->of_node)
			new = &((*new)->rb_right);
		else
			return -EEXIST;
	}

	rb_link_node(&master->node, parent, new);
	rb_insert_color(&master->node, &smmu->masters);
	return 0;
}

static int register_smmu_master(struct arm_smmu_device *smmu,
				struct device *dev,
				struct of_phandle_args *masterspec)
{
	int i;
	struct arm_smmu_master *master;

	master = find_smmu_master(smmu, masterspec->np);
	if (master) {
		dev_err(dev,
			"rejecting multiple registrations for master device %s\n",
			masterspec->np->name);
		return -EBUSY;
	}

	if (masterspec->args_count > MAX_MASTER_STREAMIDS) {
		dev_err(dev,
			"reached maximum number (%d) of stream IDs for master device %s\n",
			MAX_MASTER_STREAMIDS, masterspec->np->name);
		return -ENOSPC;
	}

	master = devm_kzalloc(dev, sizeof(*master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;

	master->of_node			= masterspec->np;
	master->cfg.num_streamids	= masterspec->args_count;

	for (i = 0; i < master->cfg.num_streamids; ++i) {
		u16 streamid = masterspec->args[i];

		if (!(smmu->features & ARM_SMMU_FEAT_STREAM_MATCH) &&
		     (streamid >= smmu->num_mapping_groups)) {
			dev_err(dev,
				"stream ID for master device %s greater than maximum allowed (%d)\n",
				masterspec->np->name, smmu->num_mapping_groups);
			return -ERANGE;
		}
		master->cfg.streamids[i] = streamid;
	}
	return insert_smmu_master(smmu, master);
}

static struct arm_smmu_device *find_smmu_for_device(struct device *dev)
{
	struct arm_smmu_device *smmu;
	struct arm_smmu_master *master = NULL;
	struct device_node *dev_node = dev_get_dev_node(dev);

	spin_lock(&arm_smmu_devices_lock);
	list_for_each_entry(smmu, &arm_smmu_devices, list) {
		master = find_smmu_master(smmu, dev_node);
		if (master)
			break;
	}
	spin_unlock(&arm_smmu_devices_lock);

	return master ? smmu : NULL;
}

static int __arm_smmu_alloc_bitmap(unsigned long *map, int start, int end)
{
	int idx;

	do {
		idx = find_next_zero_bit(map, end, start);
		if (idx == end)
			return -ENOSPC;
	} while (test_and_set_bit(idx, map));

	return idx;
}

static void __arm_smmu_free_bitmap(unsigned long *map, int idx)
{
	clear_bit(idx, map);
}

/* Wait for any pending TLB invalidations to complete */
static void arm_smmu_tlb_sync(struct arm_smmu_device *smmu)
{
	int count = 0;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);

	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_sTLBGSYNC);
	while (readl_relaxed(gr0_base + ARM_SMMU_GR0_sTLBGSTATUS)
	       & sTLBGSTATUS_GSACTIVE) {
		cpu_relax();
		if (++count == TLB_LOOP_TIMEOUT) {
			dev_err_ratelimited(smmu->dev,
			"TLB sync timed out -- SMMU may be deadlocked\n");
			return;
		}
		udelay(1);
	}
}

static void
arm_smmu_tlb_inv_context_by_vmid(struct arm_smmu_device *smmu, u32 vmid)
{
	void __iomem *base = ARM_SMMU_GR0(smmu);
	writel_relaxed(vmid, base + ARM_SMMU_GR0_TLBIVMID);
}

static void arm_smmu_tlb_inv_context(struct arm_smmu_domain *smmu_domain)
{
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *base = ARM_SMMU_GR0(smmu);
	bool stage1 = cfg->cbar != CBAR_TYPE_S2_TRANS;

	if (stage1) {
		base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
		writel_relaxed(ARM_SMMU_CB_ASID(cfg),
			       base + ARM_SMMU_CB_S1_TLBIASID);
	} else {
		arm_smmu_tlb_inv_context_by_vmid(smmu, ARM_SMMU_CB_VMID(cfg));
	}

	arm_smmu_tlb_sync(smmu);
}

static irqreturn_t arm_smmu_context_fault(int irq, void *dev)
{
	int flags, ret;
	u32 fsr, far, fsynr, resume;
	unsigned long iova;
	struct iommu_domain *domain = dev;
	struct arm_smmu_domain *smmu_domain = domain->priv;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *cb_base;

	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
	fsr = readl_relaxed(cb_base + ARM_SMMU_CB_FSR);

	if (!(fsr & FSR_FAULT))
		return IRQ_NONE;

	if (fsr & FSR_IGN)
		dev_err_ratelimited(smmu->dev,
				    "Unexpected context fault (fsr 0x%x)\n",
				    fsr);

	fsynr = readl_relaxed(cb_base + ARM_SMMU_CB_FSYNR0);
	flags = fsynr & FSYNR0_WNR ? IOMMU_FAULT_WRITE : IOMMU_FAULT_READ;

	far = readl_relaxed(cb_base + ARM_SMMU_CB_FAR_LO);
	iova = far;
#ifdef CONFIG_64BIT
	far = readl_relaxed(cb_base + ARM_SMMU_CB_FAR_HI);
	iova |= ((unsigned long)far << 32);
#endif

	if (!report_iommu_fault(domain, smmu->dev, iova, flags)) {
		ret = IRQ_HANDLED;
		resume = RESUME_RETRY;
	} else {
		dev_err_ratelimited(smmu->dev,
		    "Unhandled context fault: iova=0x%08lx, fsynr=0x%x, cb=%d\n",
		    iova, fsynr, cfg->cbndx);
		ret = IRQ_NONE;
		resume = RESUME_TERMINATE;
	}

	/* Clear the faulting FSR */
	writel(fsr, cb_base + ARM_SMMU_CB_FSR);

	/* Retry or terminate any stalled transactions */
	if (fsr & FSR_SS)
		writel_relaxed(resume, cb_base + ARM_SMMU_CB_RESUME);

	return ret;
}

static irqreturn_t arm_smmu_global_fault(int irq, void *dev)
{
	u32 gfsr, gfsynr0, gfsynr1, gfsynr2;
	struct arm_smmu_device *smmu = dev;
	void __iomem *gr0_base = ARM_SMMU_GR0_NS(smmu);

	gfsr = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSR);
	gfsynr0 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR0);
	gfsynr1 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR1);
	gfsynr2 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR2);

	if (!gfsr)
		return IRQ_NONE;

	dev_err_ratelimited(smmu->dev,
		"Unexpected global fault, this could be serious\n");
	dev_err_ratelimited(smmu->dev,
		"\tGFSR 0x%08x, GFSYNR0 0x%08x, GFSYNR1 0x%08x, GFSYNR2 0x%08x\n",
		gfsr, gfsynr0, gfsynr1, gfsynr2);

	writel(gfsr, gr0_base + ARM_SMMU_GR0_sGFSR);
	return IRQ_HANDLED;
}

static void arm_smmu_flush_pgtable(struct arm_smmu_device *smmu, void *addr,
				   size_t size)
{
	unsigned long offset = (unsigned long)addr & ~PAGE_MASK;


	/* Ensure new page tables are visible to the hardware walker */
	if (smmu->features & ARM_SMMU_FEAT_COHERENT_WALK) {
		dsb(ishst);
	} else {
		/*
		 * If the SMMU can't walk tables in the CPU caches, treat them
		 * like non-coherent DMA since we need to flush the new entries
		 * all the way out to memory. There's no possibility of
		 * recursion here as the SMMU table walker will not be wired
		 * through another SMMU.
		 */
		dma_map_page(smmu->dev, virt_to_page(addr), offset, size,
				DMA_TO_DEVICE);
	}
}

static void arm_smmu_init_context_bank(struct arm_smmu_domain *smmu_domain)
{
	u32 reg;
	bool stage1;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *cb_base, *gr1_base = ARM_SMMU_GR1(smmu);

	stage1 = cfg->cbar != CBAR_TYPE_S2_TRANS;
	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);

	/* CBAR */
	reg = cfg->cbar;
	if (smmu->version == ARM_SMMU_V1)
		reg |= cfg->irptndx << CBAR_IRPTNDX_SHIFT;

	/*
	 * Use the weakest shareability/memory types, so they are
	 * overridden by the ttbcr/pte.
	 */
	if (stage1) {
		reg |= (CBAR_S1_BPSHCFG_NSH << CBAR_S1_BPSHCFG_SHIFT) |
			(CBAR_S1_MEMATTR_WB << CBAR_S1_MEMATTR_SHIFT);
	} else {
		reg |= ARM_SMMU_CB_VMID(cfg) << CBAR_VMID_SHIFT;
	}
	writel_relaxed(reg, gr1_base + ARM_SMMU_GR1_CBAR(cfg->cbndx));

	if (smmu->version > ARM_SMMU_V1) {
		/* CBA2R */
#ifdef CONFIG_64BIT
		reg = CBA2R_RW64_64BIT;
#else
		reg = CBA2R_RW64_32BIT;
#endif
		writel_relaxed(reg,
			       gr1_base + ARM_SMMU_GR1_CBA2R(cfg->cbndx));

		/* TTBCR2 */
		switch (smmu->s1_input_size) {
		case 32:
			reg = (TTBCR2_ADDR_32 << TTBCR2_SEP_SHIFT);
			break;
		case 36:
			reg = (TTBCR2_ADDR_36 << TTBCR2_SEP_SHIFT);
			break;
		case 39:
		case 40:
			reg = (TTBCR2_ADDR_40 << TTBCR2_SEP_SHIFT);
			break;
		case 42:
			reg = (TTBCR2_ADDR_42 << TTBCR2_SEP_SHIFT);
			break;
		case 44:
			reg = (TTBCR2_ADDR_44 << TTBCR2_SEP_SHIFT);
			break;
		case 48:
			reg = (TTBCR2_ADDR_48 << TTBCR2_SEP_SHIFT);
			break;
		}

		switch (smmu->s1_output_size) {
		case 32:
			reg |= (TTBCR2_ADDR_32 << TTBCR2_PASIZE_SHIFT);
			break;
		case 36:
			reg |= (TTBCR2_ADDR_36 << TTBCR2_PASIZE_SHIFT);
			break;
		case 39:
		case 40:
			reg |= (TTBCR2_ADDR_40 << TTBCR2_PASIZE_SHIFT);
			break;
		case 42:
			reg |= (TTBCR2_ADDR_42 << TTBCR2_PASIZE_SHIFT);
			break;
		case 44:
			reg |= (TTBCR2_ADDR_44 << TTBCR2_PASIZE_SHIFT);
			break;
		case 48:
			reg |= (TTBCR2_ADDR_48 << TTBCR2_PASIZE_SHIFT);
			break;
		}

		if (stage1)
			writel_relaxed(reg, cb_base + ARM_SMMU_CB_TTBCR2);
	}

	/* TTBR0 */
	arm_smmu_flush_pgtable(smmu, cfg->pgd,
			       PTRS_PER_PGD * sizeof(pgd_t));
	reg = __pa(cfg->pgd);
	writel_relaxed(reg, cb_base + ARM_SMMU_CB_TTBR0_LO);
	reg = (phys_addr_t)__pa(cfg->pgd) >> 32;
	if (stage1)
		reg |= ARM_SMMU_CB_ASID(cfg) << TTBRn_HI_ASID_SHIFT;
	writel_relaxed(reg, cb_base + ARM_SMMU_CB_TTBR0_HI);

	/*
	 * TTBCR
	 * We use long descriptor, with inner-shareable WBWA tables in TTBR0.
	 */
	if (smmu->version > ARM_SMMU_V1) {
		if (PAGE_SIZE == SZ_4K)
			reg = TTBCR_TG0_4K;
		else
			reg = TTBCR_TG0_64K;

		if (!stage1) {
			reg |= (64 - smmu->s2_input_size) << TTBCR_T0SZ_SHIFT;

			switch (smmu->s2_output_size) {
			case 32:
				reg |= (TTBCR2_ADDR_32 << TTBCR_PASIZE_SHIFT);
				break;
			case 36:
				reg |= (TTBCR2_ADDR_36 << TTBCR_PASIZE_SHIFT);
				break;
			case 40:
				reg |= (TTBCR2_ADDR_40 << TTBCR_PASIZE_SHIFT);
				break;
			case 42:
				reg |= (TTBCR2_ADDR_42 << TTBCR_PASIZE_SHIFT);
				break;
			case 44:
				reg |= (TTBCR2_ADDR_44 << TTBCR_PASIZE_SHIFT);
				break;
			case 48:
				reg |= (TTBCR2_ADDR_48 << TTBCR_PASIZE_SHIFT);
				break;
			}
		} else {
			reg |= (64 - smmu->s1_input_size) << TTBCR_T0SZ_SHIFT;
		}
	} else {
		reg = 0;
	}

	reg |= TTBCR_EAE |
	      (TTBCR_SH_IS << TTBCR_SH0_SHIFT) |
	      (TTBCR_RGN_WBWA << TTBCR_ORGN0_SHIFT) |
	      (TTBCR_RGN_WBWA << TTBCR_IRGN0_SHIFT);

	if (!stage1)
		reg |= (TTBCR_SL0_LVL_1 << TTBCR_SL0_SHIFT);

	writel_relaxed(reg, cb_base + ARM_SMMU_CB_TTBCR);

	/* MAIR0 (stage-1 only) */
	if (stage1) {
		reg = (MAIR_ATTR_NC << MAIR_ATTR_SHIFT(MAIR_ATTR_IDX_NC)) |
		      (MAIR_ATTR_WBRWA << MAIR_ATTR_SHIFT(MAIR_ATTR_IDX_CACHE)) |
		      (MAIR_ATTR_DEVICE << MAIR_ATTR_SHIFT(MAIR_ATTR_IDX_DEV));
		writel_relaxed(reg, cb_base + ARM_SMMU_CB_S1_MAIR0);
	}

	/* SCTLR */
	reg = SCTLR_CFCFG | SCTLR_CFIE | SCTLR_CFRE | SCTLR_M | SCTLR_EAE_SBOP;
	if (stage1)
		reg |= SCTLR_S1_ASIDPNE;
#ifdef __BIG_ENDIAN
	reg |= SCTLR_E;
#endif
	writel_relaxed(reg, cb_base + ARM_SMMU_CB_SCTLR);
}

static int arm_smmu_init_domain_context(struct iommu_domain *domain,
					struct arm_smmu_device *smmu)
{
	int irq, start, ret = 0;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = domain->priv;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;

	spin_lock_irqsave(&smmu_domain->lock, flags);
	if (smmu_domain->smmu)
		goto out_unlock;

	/*
	 * Mapping the requested stage onto what we support is surprisingly
	 * complicated, mainly because the spec allows S1+S2 SMMUs without
	 * support for nested translation. That means we end up with the
	 * following table:
	 *
	 * Requested        Supported        Actual
	 *     S1               N              S1
	 *     S1             S1+S2            S1
	 *     S1               S2             S2
	 *     S1               S1             S1
	 *     N                N              S2
	 *     N              S1+S2            S2
	 *     N                S2             S2
	 *     N                S1             S1
	 *
	 * Note that you can't actually request stage-2 mappings.
	 */
	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S1))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S2;
	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S2))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S1;

	switch (smmu_domain->stage) {
	case ARM_SMMU_DOMAIN_S1:
		cfg->cbar = CBAR_TYPE_S1_TRANS_S2_BYPASS;
		start = smmu->num_s2_context_banks;
		break;
	case ARM_SMMU_DOMAIN_NESTED:
	case ARM_SMMU_DOMAIN_S2:
		cfg->cbar = CBAR_TYPE_S2_TRANS;
		start = 0;
		break;
	default:
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = __arm_smmu_alloc_bitmap(smmu->context_map, start,
				      smmu->num_context_banks);
	if (IS_ERR_VALUE(ret))
		goto out_unlock;

	cfg->cbndx = ret;
	if (smmu->version == ARM_SMMU_V1) {
		cfg->irptndx = atomic_inc_return(&smmu->irptndx);
		cfg->irptndx %= smmu->num_context_irqs;
	} else {
		cfg->irptndx = cfg->cbndx;
	}

	ACCESS_ONCE(smmu_domain->smmu) = smmu;
	arm_smmu_init_context_bank(smmu_domain);
	spin_unlock_irqrestore(&smmu_domain->lock, flags);

	irq = smmu->irqs[smmu->num_global_irqs + cfg->irptndx];
	ret = request_irq(irq, arm_smmu_context_fault, IRQF_SHARED,
			  "arm-smmu-context-fault", domain);
	if (IS_ERR_VALUE(ret)) {
		dev_err(smmu->dev, "failed to request context IRQ %d (%u)\n",
			cfg->irptndx, irq);
		cfg->irptndx = INVALID_IRPTNDX;
	}

	return 0;

out_unlock:
	spin_unlock_irqrestore(&smmu_domain->lock, flags);
	return ret;
}

static void arm_smmu_destroy_domain_context(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = domain->priv;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	void __iomem *cb_base;
	int irq;

	if (!smmu)
		return;

	/* Disable the context bank and nuke the TLB before freeing it. */
	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
	writel_relaxed(0, cb_base + ARM_SMMU_CB_SCTLR);
	arm_smmu_tlb_inv_context(smmu_domain);

	if (cfg->irptndx != INVALID_IRPTNDX) {
		irq = smmu->irqs[smmu->num_global_irqs + cfg->irptndx];
		free_irq(irq, domain);
	}

	__arm_smmu_free_bitmap(smmu->context_map, cfg->cbndx);
}

static int arm_smmu_domain_init(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain;
	pgd_t *pgd;

	/*
	 * Allocate the domain and initialise some of its data structures.
	 * We can't really do anything meaningful until we've added a
	 * master.
	 */
	smmu_domain = kzalloc(sizeof(*smmu_domain), GFP_KERNEL);
	if (!smmu_domain)
		return -ENOMEM;

	pgd = kcalloc(PTRS_PER_PGD, sizeof(pgd_t), GFP_KERNEL);
	if (!pgd)
		goto out_free_domain;
	smmu_domain->cfg.pgd = pgd;

	spin_lock_init(&smmu_domain->lock);
	domain->priv = smmu_domain;
	return 0;

out_free_domain:
	kfree(smmu_domain);
	return -ENOMEM;
}

static void arm_smmu_free_ptes(pmd_t *pmd)
{
	pgtable_t table = pmd_pgtable(*pmd);

	__free_page(table);
}

static void arm_smmu_free_pmds(pud_t *pud)
{
	int i;
	pmd_t *pmd, *pmd_base = pmd_offset(pud, 0);

	pmd = pmd_base;
	for (i = 0; i < PTRS_PER_PMD; ++i) {
		if (pmd_none(*pmd))
			continue;

		arm_smmu_free_ptes(pmd);
		pmd++;
	}

	pmd_free(NULL, pmd_base);
}

static void arm_smmu_free_puds(pgd_t *pgd)
{
	int i;
	pud_t *pud, *pud_base = pud_offset(pgd, 0);

	pud = pud_base;
	for (i = 0; i < PTRS_PER_PUD; ++i) {
		if (pud_none(*pud))
			continue;

		arm_smmu_free_pmds(pud);
		pud++;
	}

	pud_free(NULL, pud_base);
}

static void arm_smmu_free_pgtables(struct arm_smmu_domain *smmu_domain)
{
	int i;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	pgd_t *pgd, *pgd_base = cfg->pgd;

	/*
	 * Recursively free the page tables for this domain. We don't
	 * care about speculative TLB filling because the tables should
	 * not be active in any context bank at this point (SCTLR.M is 0).
	 */
	pgd = pgd_base;
	for (i = 0; i < PTRS_PER_PGD; ++i) {
		if (pgd_none(*pgd))
			continue;
		arm_smmu_free_puds(pgd);
		pgd++;
	}

	kfree(pgd_base);
}

static void arm_smmu_domain_destroy(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = domain->priv;

	/*
	 * Free the domain resources. We assume that all devices have
	 * already been detached.
	 */
	arm_smmu_destroy_domain_context(domain);
	arm_smmu_free_pgtables(smmu_domain);
	kfree(smmu_domain);
}

static int arm_smmu_master_configure_smrs(struct arm_smmu_device *smmu,
					  struct arm_smmu_master_cfg *cfg)
{
	int i;
	struct arm_smmu_smr *smrs;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);

	if (!(smmu->features & ARM_SMMU_FEAT_STREAM_MATCH))
		return 0;

	if (cfg->smrs)
		return -EEXIST;

	smrs = kmalloc_array(cfg->num_streamids, sizeof(*smrs), GFP_KERNEL);
	if (!smrs) {
		dev_err(smmu->dev, "failed to allocate %d SMRs\n",
			cfg->num_streamids);
		return -ENOMEM;
	}

	/* Allocate the SMRs on the SMMU */
	for (i = 0; i < cfg->num_streamids; ++i) {
		int idx = __arm_smmu_alloc_bitmap(smmu->smr_map, 0,
						  smmu->num_mapping_groups);
		if (IS_ERR_VALUE(idx)) {
			dev_err(smmu->dev, "failed to allocate free SMR\n");
			goto err_free_smrs;
		}

		smrs[i] = (struct arm_smmu_smr) {
			.idx	= idx,
			.mask	= 0, /* We don't currently share SMRs */
			.id	= cfg->streamids[i],
		};
	}

	/* It worked! Now, poke the actual hardware */
	for (i = 0; i < cfg->num_streamids; ++i) {
		u32 reg = SMR_VALID | smrs[i].id << SMR_ID_SHIFT |
			  smrs[i].mask << SMR_MASK_SHIFT;
		writel_relaxed(reg, gr0_base + ARM_SMMU_GR0_SMR(smrs[i].idx));
	}

	cfg->smrs = smrs;
	return 0;

err_free_smrs:
	while (--i >= 0)
		__arm_smmu_free_bitmap(smmu->smr_map, smrs[i].idx);
	kfree(smrs);
	return -ENOSPC;
}

static void arm_smmu_master_free_smrs(struct arm_smmu_device *smmu,
				      struct arm_smmu_master_cfg *cfg)
{
	int i;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	struct arm_smmu_smr *smrs = cfg->smrs;

	if (!smrs)
		return;

	/* Invalidate the SMRs before freeing back to the allocator */
	for (i = 0; i < cfg->num_streamids; ++i) {
		u8 idx = smrs[i].idx;

		writel_relaxed(~SMR_VALID, gr0_base + ARM_SMMU_GR0_SMR(idx));
		__arm_smmu_free_bitmap(smmu->smr_map, idx);
	}

	cfg->smrs = NULL;
	kfree(smrs);
}

static int arm_smmu_domain_add_master(struct arm_smmu_domain *smmu_domain,
				      struct arm_smmu_master_cfg *cfg)
{
	int i, ret;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);

	/* Devices in an IOMMU group may already be configured */
	ret = arm_smmu_master_configure_smrs(smmu, cfg);
	if (ret)
		return ret == -EEXIST ? 0 : ret;

	for (i = 0; i < cfg->num_streamids; ++i) {
		u32 idx, s2cr;

		idx = cfg->smrs ? cfg->smrs[i].idx : cfg->streamids[i];
		s2cr = S2CR_TYPE_TRANS |
		       (smmu_domain->cfg.cbndx << S2CR_CBNDX_SHIFT);
		writel_relaxed(s2cr, gr0_base + ARM_SMMU_GR0_S2CR(idx));
	}

	return 0;
}

static void arm_smmu_domain_remove_master(struct arm_smmu_domain *smmu_domain,
					  struct arm_smmu_master_cfg *cfg)
{
	int i;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);

	/* An IOMMU group is torn down by the first device to be removed */
	if ((smmu->features & ARM_SMMU_FEAT_STREAM_MATCH) && !cfg->smrs)
		return;

	/*
	 * We *must* clear the S2CR first, because freeing the SMR means
	 * that it can be re-allocated immediately.
	 */
	for (i = 0; i < cfg->num_streamids; ++i) {
		u32 idx = cfg->smrs ? cfg->smrs[i].idx : cfg->streamids[i];

		writel_relaxed(S2CR_TYPE_BYPASS,
			       gr0_base + ARM_SMMU_GR0_S2CR(idx));
	}

	arm_smmu_master_free_smrs(smmu, cfg);
}

static int arm_smmu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	int ret;
	struct arm_smmu_domain *smmu_domain = domain->priv;
	struct arm_smmu_device *smmu, *dom_smmu;
	struct arm_smmu_master_cfg *cfg;

	smmu = find_smmu_for_device(dev);
	if (!smmu) {
		dev_err(dev, "cannot attach to SMMU, is it on the same bus?\n");
		return -ENXIO;
	}

	if (dev->archdata.iommu) {
		dev_err(dev, "already attached to IOMMU domain\n");
		return -EEXIST;
	}

	/*
	 * Sanity check the domain. We don't support domains across
	 * different SMMUs.
	 */
	dom_smmu = ACCESS_ONCE(smmu_domain->smmu);
	if (!dom_smmu) {
		/* Now that we have a master, we can finalise the domain */
		ret = arm_smmu_init_domain_context(domain, smmu);
		if (IS_ERR_VALUE(ret))
			return ret;

		dom_smmu = smmu_domain->smmu;
	}

	if (dom_smmu != smmu) {
		dev_err(dev,
			"cannot attach to SMMU %s whilst already attached to domain on SMMU %s\n",
			dev_name(smmu_domain->smmu->dev), dev_name(smmu->dev));
		return -EINVAL;
	}

	/* Looks ok, so add the device to the domain */
	cfg = find_smmu_master_cfg(dev);
	if (!cfg)
		return -ENODEV;

	ret = arm_smmu_domain_add_master(smmu_domain, cfg);
	if (!ret)
		dev->archdata.iommu = domain;
	return ret;
}

static void arm_smmu_detach_dev(struct iommu_domain *domain, struct device *dev)
{
	struct arm_smmu_domain *smmu_domain = domain->priv;
	struct arm_smmu_master_cfg *cfg;

	cfg = find_smmu_master_cfg(dev);
	if (!cfg)
		return;

	dev->archdata.iommu = NULL;
	arm_smmu_domain_remove_master(smmu_domain, cfg);
}

static bool arm_smmu_pte_is_contiguous_range(unsigned long addr,
					     unsigned long end)
{
	return !(addr & ~ARM_SMMU_PTE_CONT_MASK) &&
		(addr + ARM_SMMU_PTE_CONT_SIZE <= end);
}

static int arm_smmu_alloc_init_pte(struct arm_smmu_device *smmu, pmd_t *pmd,
				   unsigned long addr, unsigned long end,
				   unsigned long pfn, int prot, int stage)
{
	pte_t *pte, *start;
	pteval_t pteval = ARM_SMMU_PTE_PAGE | ARM_SMMU_PTE_AF;

	if (pmd_none(*pmd)) {
		/* Allocate a new set of tables */
		pgtable_t table = alloc_page(GFP_ATOMIC|__GFP_ZERO);

		if (!table)
			return -ENOMEM;

		arm_smmu_flush_pgtable(smmu, page_address(table), PAGE_SIZE);
		pmd_populate(NULL, pmd, table);
		arm_smmu_flush_pgtable(smmu, pmd, sizeof(*pmd));
	}

	if (stage == 1) {
		pteval |= ARM_SMMU_PTE_AP_UNPRIV | ARM_SMMU_PTE_nG;
		if (!(prot & IOMMU_WRITE) && (prot & IOMMU_READ))
			pteval |= ARM_SMMU_PTE_AP_RDONLY;

		if (prot & IOMMU_CACHE)
			pteval |= (MAIR_ATTR_IDX_CACHE <<
				   ARM_SMMU_PTE_ATTRINDX_SHIFT);
	} else {
		pteval |= ARM_SMMU_PTE_HAP_FAULT;
		if (prot & IOMMU_READ)
			pteval |= ARM_SMMU_PTE_HAP_READ;
		if (prot & IOMMU_WRITE)
			pteval |= ARM_SMMU_PTE_HAP_WRITE;
		if (prot & IOMMU_CACHE)
			pteval |= ARM_SMMU_PTE_MEMATTR_OIWB;
		else
			pteval |= ARM_SMMU_PTE_MEMATTR_NC;
	}

	if (prot & IOMMU_NOEXEC)
		pteval |= ARM_SMMU_PTE_XN;

	/* If no access, create a faulting entry to avoid TLB fills */
	if (!(prot & (IOMMU_READ | IOMMU_WRITE)))
		pteval &= ~ARM_SMMU_PTE_PAGE;

	pteval |= ARM_SMMU_PTE_SH_IS;
	start = pmd_page_vaddr(*pmd) + pte_index(addr);
	pte = start;

	/*
	 * Install the page table entries. This is fairly complicated
	 * since we attempt to make use of the contiguous hint in the
	 * ptes where possible. The contiguous hint indicates a series
	 * of ARM_SMMU_PTE_CONT_ENTRIES ptes mapping a physically
	 * contiguous region with the following constraints:
	 *
	 *   - The region start is aligned to ARM_SMMU_PTE_CONT_SIZE
	 *   - Each pte in the region has the contiguous hint bit set
	 *
	 * This complicates unmapping (also handled by this code, when
	 * neither IOMMU_READ or IOMMU_WRITE are set) because it is
	 * possible, yet highly unlikely, that a client may unmap only
	 * part of a contiguous range. This requires clearing of the
	 * contiguous hint bits in the range before installing the new
	 * faulting entries.
	 *
	 * Note that re-mapping an address range without first unmapping
	 * it is not supported, so TLB invalidation is not required here
	 * and is instead performed at unmap and domain-init time.
	 */
	do {
		int i = 1;

		pteval &= ~ARM_SMMU_PTE_CONT;

		if (arm_smmu_pte_is_contiguous_range(addr, end)) {
			i = ARM_SMMU_PTE_CONT_ENTRIES;
			pteval |= ARM_SMMU_PTE_CONT;
		} else if (pte_val(*pte) &
			   (ARM_SMMU_PTE_CONT | ARM_SMMU_PTE_PAGE)) {
			int j;
			pte_t *cont_start;
			unsigned long idx = pte_index(addr);

			idx &= ~(ARM_SMMU_PTE_CONT_ENTRIES - 1);
			cont_start = pmd_page_vaddr(*pmd) + idx;
			for (j = 0; j < ARM_SMMU_PTE_CONT_ENTRIES; ++j)
				pte_val(*(cont_start + j)) &=
					~ARM_SMMU_PTE_CONT;

			arm_smmu_flush_pgtable(smmu, cont_start,
					       sizeof(*pte) *
					       ARM_SMMU_PTE_CONT_ENTRIES);
		}

		do {
			*pte = pfn_pte(pfn, __pgprot(pteval));
		} while (pte++, pfn++, addr += PAGE_SIZE, --i);
	} while (addr != end);

	arm_smmu_flush_pgtable(smmu, start, sizeof(*pte) * (pte - start));
	return 0;
}

static int arm_smmu_alloc_init_pmd(struct arm_smmu_device *smmu, pud_t *pud,
				   unsigned long addr, unsigned long end,
				   phys_addr_t phys, int prot, int stage)
{
	int ret;
	pmd_t *pmd;
	unsigned long next, pfn = __phys_to_pfn(phys);

#ifndef __PAGETABLE_PMD_FOLDED
	if (pud_none(*pud)) {
		pmd = (pmd_t *)get_zeroed_page(GFP_ATOMIC);
		if (!pmd)
			return -ENOMEM;

		arm_smmu_flush_pgtable(smmu, pmd, PAGE_SIZE);
		pud_populate(NULL, pud, pmd);
		arm_smmu_flush_pgtable(smmu, pud, sizeof(*pud));

		pmd += pmd_index(addr);
	} else
#endif
		pmd = pmd_offset(pud, addr);

	do {
		next = pmd_addr_end(addr, end);
		ret = arm_smmu_alloc_init_pte(smmu, pmd, addr, next, pfn,
					      prot, stage);
		phys += next - addr;
		pfn = __phys_to_pfn(phys);
	} while (pmd++, addr = next, addr < end);

	return ret;
}

static int arm_smmu_alloc_init_pud(struct arm_smmu_device *smmu, pgd_t *pgd,
				   unsigned long addr, unsigned long end,
				   phys_addr_t phys, int prot, int stage)
{
	int ret = 0;
	pud_t *pud;
	unsigned long next;

#ifndef __PAGETABLE_PUD_FOLDED
	if (pgd_none(*pgd)) {
		pud = (pud_t *)get_zeroed_page(GFP_ATOMIC);
		if (!pud)
			return -ENOMEM;

		arm_smmu_flush_pgtable(smmu, pud, PAGE_SIZE);
		pgd_populate(NULL, pgd, pud);
		arm_smmu_flush_pgtable(smmu, pgd, sizeof(*pgd));

		pud += pud_index(addr);
	} else
#endif
		pud = pud_offset(pgd, addr);

	do {
		next = pud_addr_end(addr, end);
		ret = arm_smmu_alloc_init_pmd(smmu, pud, addr, next, phys,
					      prot, stage);
		phys += next - addr;
	} while (pud++, addr = next, addr < end);

	return ret;
}

static int arm_smmu_handle_mapping(struct arm_smmu_domain *smmu_domain,
				   unsigned long iova, phys_addr_t paddr,
				   size_t size, int prot)
{
	int ret, stage;
	unsigned long end;
	phys_addr_t input_mask, output_mask;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	pgd_t *pgd = cfg->pgd;
	unsigned long flags;

	if (cfg->cbar == CBAR_TYPE_S2_TRANS) {
		stage = 2;
		input_mask = (1ULL << smmu->s2_input_size) - 1;
		output_mask = (1ULL << smmu->s2_output_size) - 1;
	} else {
		stage = 1;
		input_mask = (1ULL << smmu->s1_input_size) - 1;
		output_mask = (1ULL << smmu->s1_output_size) - 1;
	}

	if (!pgd)
		return -EINVAL;

	if (size & ~PAGE_MASK)
		return -EINVAL;

	if ((phys_addr_t)iova & ~input_mask)
		return -ERANGE;

	if (paddr & ~output_mask)
		return -ERANGE;

	spin_lock_irqsave(&smmu_domain->lock, flags);
	pgd += pgd_index(iova);
	end = iova + size;
	do {
		unsigned long next = pgd_addr_end(iova, end);

		ret = arm_smmu_alloc_init_pud(smmu, pgd, iova, next, paddr,
					      prot, stage);
		if (ret)
			goto out_unlock;

		paddr += next - iova;
		iova = next;
	} while (pgd++, iova != end);

out_unlock:
	spin_unlock_irqrestore(&smmu_domain->lock, flags);

	return ret;
}

static int arm_smmu_map(struct iommu_domain *domain, unsigned long iova,
			phys_addr_t paddr, size_t size, int prot)
{
	struct arm_smmu_domain *smmu_domain = domain->priv;

	if (!smmu_domain)
		return -ENODEV;

	return arm_smmu_handle_mapping(smmu_domain, iova, paddr, size, prot);
}

static size_t arm_smmu_unmap(struct iommu_domain *domain, unsigned long iova,
			     size_t size)
{
	int ret;
	struct arm_smmu_domain *smmu_domain = domain->priv;

	ret = arm_smmu_handle_mapping(smmu_domain, iova, 0, size, 0);
	arm_smmu_tlb_inv_context(smmu_domain);
	return ret ? 0 : size;
}

static phys_addr_t arm_smmu_iova_to_phys(struct iommu_domain *domain,
					 dma_addr_t iova)
{
	pgd_t *pgdp, pgd;
	pud_t pud;
	pmd_t pmd;
	pte_t pte;
	struct arm_smmu_domain *smmu_domain = domain->priv;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;

	pgdp = cfg->pgd;
	if (!pgdp)
		return 0;

	pgd = *(pgdp + pgd_index(iova));
	if (pgd_none(pgd))
		return 0;

	pud = *pud_offset(&pgd, iova);
	if (pud_none(pud))
		return 0;

	pmd = *pmd_offset(&pud, iova);
	if (pmd_none(pmd))
		return 0;

	pte = *(pmd_page_vaddr(pmd) + pte_index(iova));
	if (pte_none(pte))
		return 0;

	return __pfn_to_phys(pte_pfn(pte)) | (iova & ~PAGE_MASK);
}

static bool arm_smmu_capable(enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		/*
		 * Return true here as the SMMU can always send out coherent
		 * requests.
		 */
		return true;
	case IOMMU_CAP_INTR_REMAP:
		return true; /* MSIs are just memory writes */
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}

static int __arm_smmu_get_pci_sid(struct pci_dev *pdev, u16 alias, void *data)
{
	*((u16 *)data) = alias;
	return 0; /* Continue walking */
}

static void __arm_smmu_release_pci_iommudata(void *data)
{
	kfree(data);
}

static int arm_smmu_add_device(struct device *dev)
{
	struct arm_smmu_device *smmu;
	struct arm_smmu_master_cfg *cfg;
	struct iommu_group *group;
	void (*releasefn)(void *) = NULL;
	int ret;

	smmu = find_smmu_for_device(dev);
	if (!smmu)
		return -ENODEV;

	group = iommu_group_alloc();
	if (IS_ERR(group)) {
		dev_err(dev, "Failed to allocate IOMMU group\n");
		return PTR_ERR(group);
	}

	if (dev_is_pci(dev)) {
		struct pci_dev *pdev = to_pci_dev(dev);

		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			goto out_put_group;
		}

		cfg->num_streamids = 1;
		/*
		 * Assume Stream ID == Requester ID for now.
		 * We need a way to describe the ID mappings in FDT.
		 */
		pci_for_each_dma_alias(pdev, __arm_smmu_get_pci_sid,
				       &cfg->streamids[0]);
		releasefn = __arm_smmu_release_pci_iommudata;
	} else {
		struct arm_smmu_master *master;

		master = find_smmu_master(smmu, dev->of_node);
		if (!master) {
			ret = -ENODEV;
			goto out_put_group;
		}

		cfg = &master->cfg;
	}

	iommu_group_set_iommudata(group, cfg, releasefn);
	ret = iommu_group_add_device(group, dev);

out_put_group:
	iommu_group_put(group);
	return ret;
}

static void arm_smmu_remove_device(struct device *dev)
{
	iommu_group_remove_device(dev);
}

static int arm_smmu_domain_get_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	struct arm_smmu_domain *smmu_domain = domain->priv;

	switch (attr) {
	case DOMAIN_ATTR_NESTING:
		*(int *)data = (smmu_domain->stage == ARM_SMMU_DOMAIN_NESTED);
		return 0;
	default:
		return -ENODEV;
	}
}

static int arm_smmu_domain_set_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	struct arm_smmu_domain *smmu_domain = domain->priv;

	switch (attr) {
	case DOMAIN_ATTR_NESTING:
		if (smmu_domain->smmu)
			return -EPERM;
		if (*(int *)data)
			smmu_domain->stage = ARM_SMMU_DOMAIN_NESTED;
		else
			smmu_domain->stage = ARM_SMMU_DOMAIN_S1;

		return 0;
	default:
		return -ENODEV;
	}
}

static const struct iommu_ops arm_smmu_ops = {
	.capable		= arm_smmu_capable,
	.domain_init		= arm_smmu_domain_init,
	.domain_destroy		= arm_smmu_domain_destroy,
	.attach_dev		= arm_smmu_attach_dev,
	.detach_dev		= arm_smmu_detach_dev,
	.map			= arm_smmu_map,
	.unmap			= arm_smmu_unmap,
	.map_sg			= default_iommu_map_sg,
	.iova_to_phys		= arm_smmu_iova_to_phys,
	.add_device		= arm_smmu_add_device,
	.remove_device		= arm_smmu_remove_device,
	.domain_get_attr	= arm_smmu_domain_get_attr,
	.domain_set_attr	= arm_smmu_domain_set_attr,
	.pgsize_bitmap		= (SECTION_SIZE |
				   ARM_SMMU_PTE_CONT_SIZE |
				   PAGE_SIZE),
};

static void arm_smmu_device_reset(struct arm_smmu_device *smmu)
{
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	void __iomem *cb_base;
	int i = 0;
	u32 reg;

	/* clear global FSR */
	reg = readl_relaxed(ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sGFSR);
	writel(reg, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sGFSR);

	/* Mark all SMRn as invalid and all S2CRn as bypass */
	for (i = 0; i < smmu->num_mapping_groups; ++i) {
		writel_relaxed(0, gr0_base + ARM_SMMU_GR0_SMR(i));
		writel_relaxed(S2CR_TYPE_BYPASS,
			gr0_base + ARM_SMMU_GR0_S2CR(i));
	}

	/* Make sure all context banks are disabled and clear CB_FSR  */
	for (i = 0; i < smmu->num_context_banks; ++i) {
		cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, i);
		writel_relaxed(0, cb_base + ARM_SMMU_CB_SCTLR);
		writel_relaxed(FSR_FAULT, cb_base + ARM_SMMU_CB_FSR);
	}

	/* Invalidate the TLB, just in case */
	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_TLBIALLH);
	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_TLBIALLNSNH);

	reg = readl_relaxed(ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);

	/* Enable fault reporting */
	reg |= (sCR0_GFRE | sCR0_GFIE | sCR0_GCFGFRE | sCR0_GCFGFIE);

	/* Disable TLB broadcasting. */
	reg |= (sCR0_VMIDPNE | sCR0_PTM);

	/* Enable client access, but bypass when no mapping is found */
	reg &= ~(sCR0_CLIENTPD | sCR0_USFCFG);

	/* Disable forced broadcasting */
	reg &= ~sCR0_FB;

	/* Don't upgrade barriers */
	reg &= ~(sCR0_BSU_MASK << sCR0_BSU_SHIFT);

	/* Push the button */
	arm_smmu_tlb_sync(smmu);
	writel(reg, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);
}

static int arm_smmu_id_size_to_bits(int size)
{
	switch (size) {
	case 0:
		return 32;
	case 1:
		return 36;
	case 2:
		return 40;
	case 3:
		return 42;
	case 4:
		return 44;
	case 5:
	default:
		return 48;
	}
}

static int arm_smmu_device_cfg_probe(struct arm_smmu_device *smmu)
{
	unsigned long size;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	u32 id;

	dev_notice(smmu->dev, "probing hardware configuration...\n");
	dev_notice(smmu->dev, "SMMUv%d with:\n", smmu->version);

	/* ID0 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID0);
#ifndef CONFIG_64BIT
	if (((id >> ID0_PTFS_SHIFT) & ID0_PTFS_MASK) == ID0_PTFS_V8_ONLY) {
		dev_err(smmu->dev, "\tno v7 descriptor support!\n");
		return -ENODEV;
	}
#endif

	/* Restrict available stages based on module parameter */
	if (force_stage == 1)
		id &= ~(ID0_S2TS | ID0_NTS);
	else if (force_stage == 2)
		id &= ~(ID0_S1TS | ID0_NTS);

	if (id & ID0_S1TS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_S1;
		dev_notice(smmu->dev, "\tstage 1 translation\n");
	}

	if (id & ID0_S2TS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_S2;
		dev_notice(smmu->dev, "\tstage 2 translation\n");
	}

	if (id & ID0_NTS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_NESTED;
		dev_notice(smmu->dev, "\tnested translation\n");
	}

	if (!(smmu->features &
		(ARM_SMMU_FEAT_TRANS_S1 | ARM_SMMU_FEAT_TRANS_S2))) {
		dev_err(smmu->dev, "\tno translation support!\n");
		return -ENODEV;
	}

	if (id & ID0_CTTW) {
		smmu->features |= ARM_SMMU_FEAT_COHERENT_WALK;
		dev_notice(smmu->dev, "\tcoherent table walk\n");
	}

	if (id & ID0_SMS) {
		u32 smr, sid, mask;

		smmu->features |= ARM_SMMU_FEAT_STREAM_MATCH;
		smmu->num_mapping_groups = (id >> ID0_NUMSMRG_SHIFT) &
					   ID0_NUMSMRG_MASK;
		if (smmu->num_mapping_groups == 0) {
			dev_err(smmu->dev,
				"stream-matching supported, but no SMRs present!\n");
			return -ENODEV;
		}

		smr = SMR_MASK_MASK << SMR_MASK_SHIFT;
		smr |= (SMR_ID_MASK << SMR_ID_SHIFT);
		writel_relaxed(smr, gr0_base + ARM_SMMU_GR0_SMR(0));
		smr = readl_relaxed(gr0_base + ARM_SMMU_GR0_SMR(0));

		mask = (smr >> SMR_MASK_SHIFT) & SMR_MASK_MASK;
		sid = (smr >> SMR_ID_SHIFT) & SMR_ID_MASK;
		if ((mask & sid) != sid) {
			dev_err(smmu->dev,
				"SMR mask bits (0x%x) insufficient for ID field (0x%x)\n",
				mask, sid);
			return -ENODEV;
		}

		dev_notice(smmu->dev,
			   "\tstream matching with %u register groups, mask 0x%x",
			   smmu->num_mapping_groups, mask);
	} else {
		smmu->num_mapping_groups = (id >> ID0_NUMSIDB_SHIFT) &
					   ID0_NUMSIDB_MASK;
	}
	smmu->idr[0] = id;

	/* ID1 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID1);
	smmu->pgshift = (id & ID1_PAGESIZE) ? 16 : 12;

	/* Check for size mismatch of SMMU address space from mapped region */
	size = 1 <<
		(((id >> ID1_NUMPAGENDXB_SHIFT) & ID1_NUMPAGENDXB_MASK) + 1);
	size *= 2 << smmu->pgshift;
	if (smmu->size != size)
		dev_warn(smmu->dev,
			"SMMU address space size (0x%lx) differs from mapped region size (0x%lx)!\n",
			size, smmu->size);

	smmu->num_s2_context_banks = (id >> ID1_NUMS2CB_SHIFT) &
				      ID1_NUMS2CB_MASK;
	smmu->num_context_banks = (id >> ID1_NUMCB_SHIFT) & ID1_NUMCB_MASK;
	if (smmu->num_s2_context_banks > smmu->num_context_banks) {
		dev_err(smmu->dev, "impossible number of S2 context banks!\n");
		return -ENODEV;
	}
	dev_notice(smmu->dev, "\t%u context banks (%u stage-2 only)\n",
		   smmu->num_context_banks, smmu->num_s2_context_banks);
	smmu->idr[1] = id;

	/* ID2 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID2);
	size = arm_smmu_id_size_to_bits((id >> ID2_IAS_SHIFT) & ID2_IAS_MASK);
	smmu->s1_output_size = min_t(unsigned long, PHYS_MASK_SHIFT, size);

	/* Stage-2 input size limited due to pgd allocation (PTRS_PER_PGD) */
#ifdef CONFIG_64BIT
	smmu->s2_input_size = min_t(unsigned long, VA_BITS, size);
#else
	smmu->s2_input_size = min(32UL, size);
#endif

	/* The stage-2 output mask is also applied for bypass */
	size = arm_smmu_id_size_to_bits((id >> ID2_OAS_SHIFT) & ID2_OAS_MASK);
	smmu->s2_output_size = min_t(unsigned long, PHYS_MASK_SHIFT, size);

	if (smmu->version == ARM_SMMU_V1) {
		smmu->s1_input_size = 32;
	} else {
#ifdef CONFIG_64BIT
		size = (id >> ID2_UBS_SHIFT) & ID2_UBS_MASK;
		size = min(VA_BITS, arm_smmu_id_size_to_bits(size));
#else
		size = 32;
#endif
		smmu->s1_input_size = size;

		if ((PAGE_SIZE == SZ_4K && !(id & ID2_PTFS_4K)) ||
		    (PAGE_SIZE == SZ_64K && !(id & ID2_PTFS_64K)) ||
		    (PAGE_SIZE != SZ_4K && PAGE_SIZE != SZ_64K)) {
			dev_err(smmu->dev, "CPU page size 0x%lx unsupported\n",
				PAGE_SIZE);
			return -ENODEV;
		}
	}

	if (smmu->features & ARM_SMMU_FEAT_TRANS_S1)
		dev_notice(smmu->dev, "\tStage-1: %lu-bit VA -> %lu-bit IPA\n",
			   smmu->s1_input_size, smmu->s1_output_size);

	if (smmu->features & ARM_SMMU_FEAT_TRANS_S2)
		dev_notice(smmu->dev, "\tStage-2: %lu-bit IPA -> %lu-bit PA\n",
			   smmu->s2_input_size, smmu->s2_output_size);

	smmu->idr[2] = id;
	return 0;
}

static const struct of_device_id arm_smmu_of_match[] = {
	{ .compatible = "arm,smmu-v1", .data = (void *)ARM_SMMU_V1 },
	{ .compatible = "arm,smmu-v2", .data = (void *)ARM_SMMU_V2 },
	{ .compatible = "arm,mmu-400", .data = (void *)ARM_SMMU_V1 },
	{ .compatible = "arm,mmu-401", .data = (void *)ARM_SMMU_V1 },
	{ .compatible = "arm,mmu-500", .data = (void *)ARM_SMMU_V2 },
	{ },
};
MODULE_DEVICE_TABLE(of, arm_smmu_of_match);

static int arm_smmu_device_dt_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct resource *res;
	struct arm_smmu_device *smmu;
	struct device *dev = &pdev->dev;
	struct rb_node *node;
	struct of_phandle_args masterspec;
	int num_irqs, i, err;

	smmu = devm_kzalloc(dev, sizeof(*smmu), GFP_KERNEL);
	if (!smmu) {
		dev_err(dev, "failed to allocate arm_smmu_device\n");
		return -ENOMEM;
	}
	smmu->dev = dev;

	of_id = of_match_node(arm_smmu_of_match, dev->of_node);
	smmu->version = (enum arm_smmu_arch_version)of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smmu->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(smmu->base))
		return PTR_ERR(smmu->base);
	smmu->size = resource_size(res);

	if (of_property_read_u32(dev->of_node, "#global-interrupts",
				 &smmu->num_global_irqs)) {
		dev_err(dev, "missing #global-interrupts property\n");
		return -ENODEV;
	}

	num_irqs = 0;
	while ((res = platform_get_resource(pdev, IORESOURCE_IRQ, num_irqs))) {
		num_irqs++;
		if (num_irqs > smmu->num_global_irqs)
			smmu->num_context_irqs++;
	}

	if (!smmu->num_context_irqs) {
		dev_err(dev, "found %d interrupts but expected at least %d\n",
			num_irqs, smmu->num_global_irqs + 1);
		return -ENODEV;
	}

	smmu->irqs = devm_kzalloc(dev, sizeof(*smmu->irqs) * num_irqs,
				  GFP_KERNEL);
	if (!smmu->irqs) {
		dev_err(dev, "failed to allocate %d irqs\n", num_irqs);
		return -ENOMEM;
	}

	for (i = 0; i < num_irqs; ++i) {
		int irq = platform_get_irq(pdev, i);

		if (irq < 0) {
			dev_err(dev, "failed to get irq index %d\n", i);
			return -ENODEV;
		}
		smmu->irqs[i] = irq;
	}

	err = arm_smmu_device_cfg_probe(smmu);
	if (err)
		return err;

	i = 0;
	smmu->masters = RB_ROOT;
	while (!of_parse_phandle_with_args(dev->of_node, "mmu-masters",
					   "#stream-id-cells", i,
					   &masterspec)) {
		err = register_smmu_master(smmu, dev, &masterspec);
		if (err) {
			dev_err(dev, "failed to add master %s\n",
				masterspec.np->name);
			goto out_put_masters;
		}

		i++;
	}
	dev_notice(dev, "registered %d master devices\n", i);

	parse_driver_options(smmu);

	if (smmu->version > ARM_SMMU_V1 &&
	    smmu->num_context_banks != smmu->num_context_irqs) {
		dev_err(dev,
			"found only %d context interrupt(s) but %d required\n",
			smmu->num_context_irqs, smmu->num_context_banks);
		err = -ENODEV;
		goto out_put_masters;
	}

	for (i = 0; i < smmu->num_global_irqs; ++i) {
		err = request_irq(smmu->irqs[i],
				  arm_smmu_global_fault,
				  IRQF_SHARED,
				  "arm-smmu global fault",
				  smmu);
		if (err) {
			dev_err(dev, "failed to request global IRQ %d (%u)\n",
				i, smmu->irqs[i]);
			goto out_free_irqs;
		}
	}

	INIT_LIST_HEAD(&smmu->list);
	spin_lock(&arm_smmu_devices_lock);
	list_add(&smmu->list, &arm_smmu_devices);
	spin_unlock(&arm_smmu_devices_lock);

	arm_smmu_device_reset(smmu);
	return 0;

out_free_irqs:
	while (i--)
		free_irq(smmu->irqs[i], smmu);

out_put_masters:
	for (node = rb_first(&smmu->masters); node; node = rb_next(node)) {
		struct arm_smmu_master *master
			= container_of(node, struct arm_smmu_master, node);
		of_node_put(master->of_node);
	}

	return err;
}

static int arm_smmu_device_remove(struct platform_device *pdev)
{
	int i;
	struct device *dev = &pdev->dev;
	struct arm_smmu_device *curr, *smmu = NULL;
	struct rb_node *node;

	spin_lock(&arm_smmu_devices_lock);
	list_for_each_entry(curr, &arm_smmu_devices, list) {
		if (curr->dev == dev) {
			smmu = curr;
			list_del(&smmu->list);
			break;
		}
	}
	spin_unlock(&arm_smmu_devices_lock);

	if (!smmu)
		return -ENODEV;

	for (node = rb_first(&smmu->masters); node; node = rb_next(node)) {
		struct arm_smmu_master *master
			= container_of(node, struct arm_smmu_master, node);
		of_node_put(master->of_node);
	}

	if (!bitmap_empty(smmu->context_map, ARM_SMMU_MAX_CBS))
		dev_err(dev, "removing device with active domains!\n");

	for (i = 0; i < smmu->num_global_irqs; ++i)
		free_irq(smmu->irqs[i], smmu);

	/* Turn the thing off */
	writel(sCR0_CLIENTPD, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);
	return 0;
}

static struct platform_driver arm_smmu_driver = {
	.driver	= {
		.name		= "arm-smmu",
		.of_match_table	= of_match_ptr(arm_smmu_of_match),
	},
	.probe	= arm_smmu_device_dt_probe,
	.remove	= arm_smmu_device_remove,
};

/*
 * Virtual SMMU (vSMMU) interface for KVM.
 * Theory of operation:
 *
 * We expose a virtual SMMU interface to a guest OS. This virtual interface
 * has the following properties:
 *
 * - A single combined interrupt
 * - Stream-indexing only (i.e. no SMRs)
 * - One context bank per virtual StreamID (vSID)
 * - At least one S2CR entry per vSID (i.e. capped by max vSID)
 * - Stage-1 translation only
 * - Backed by a single physical SMMU (i.e. 1:1 mapping between virtual
 *   and physical interfaces)
 *
 * When the host creates a nested domain on a physical SMMU, we only
 * allocate and configure a stage-2 context initially. A stage-1 context
 * is later allocated by the vSMMU code for each device in the domain.
 *
 * Userspace initialises a virtual SMMU interface via the KVM_CREATE_DEVICE
 * ioctl; VFIO groups are added to the vSMMU using the
 * KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_ADD attribute whilst other attributes
 * are provided to configure and probe the basic geometry of the vSMMU
 * device and its connected masters.
 *
 * Once a vSMMU has been instantiated with the KVM_DEV_ARM_SMMU_V2_CFG_INIT
 * attribute, other attributes can no longer be modified and are treated as
 * read-only from that point on. In an ideal world, instantiation allocates
 * stage-1 contexts on the corresponding physical SMMU and maps then directly
 * into the guest address space (i.e. no trapping). However, hardware issues
 * (e.g. combined context interrupts) may force us to trap access to the
 * stage-1 context banks too.  For trapping regions of the vSMMU, they are
 * emulated as follows (note that each region occupies 1 << smmu->pgshift
 * bytes, which can differ from the PAGE_SIZE in use by Linux):
 *
 *   Trapped Region (in order from offset 0x0)             |       Behaviour
 *   ------------------------------------------------------+--------------------
 *   Global register space 0:
 *     Global Configuration                                |       Emulate (r/w)
 *     Identification                                      |       Emulate (r)
 *     Global Faults                                       |         RAZ/WI
 *     Global TLBI                                         |       Emulate (r/w)
 *     Global ATOS                                         |         RAZ/WI
 *     SMRs                                                |         RAZ/WI
 *     S2CRs                                               |       Emulate (r/w)
 *   Global register space 1:
 *     CBARs                                               |       Emulate (r/w)
 *     Context Faults                                      |       Emulate (r/w)
 *     CBA2Rs                                              |         RAZ/WI
 *   IMPDEF:                                               |         RAZ/WI
 *   PMU:                                                  |         RAZ/WI
 *   SSD:                                                  |         RAZ/WI
 *   V2PAD:                                                |         RAZ/WI
 *   IMPDEF (extending to GLOBAL_TOP):                     |         RAZ/WI
 *   ------------------------------------------------------+--------------------
 *
 * An identically sized region follows, containing the mapped stage-1 context
 * banks as a prefix (then padded with RAZ/WI).
 *
 * Most of the emulation highlighted above boils down to masking/forcing
 * bits in the register values being read/written. However, writes to the
 * S2CR are a lot more interesting.
 *
 * When the guest writes a vS2CR, it will write to index vSID and attempt
 * to install a linkage to vCBARn for the stage-1 mapping. The vSMMU will
 * actually look up CBARn (allocated by the vSMMU at instantiation time),
 * modify it to be CBAR_TYPE_S1_TRANS_S2_TRANS and install a linkage to
 * the stage-2 CBAR currently indexed by the S2CR.
 */

#ifdef CONFIG_KVM
#include <linux/kvm_host.h>
#include <linux/uaccess.h>

/* Why on Earth isn't this in /include ? */
#include "../../virt/kvm/iodev.h"

/*
 * We need both of these, as KVM_PHYS_MASK is in different places for arm
 * and arm64.
 * */
#include <asm/kvm_arm.h>
#include <asm/kvm_mmu.h>

enum arm_vsmmu_global_trap_page {
	ARM_VSMMU_TRAP_PAGE_GR0 = 0,
	ARM_VSMMU_TRAP_PAGE_GR1,
	ARM_VSMMU_TRAP_PAGE_IMPDEF,
	ARM_VSMMU_TRAP_PAGE_PMU,
	ARM_VSMMU_TRAP_PAGE_SSD,

	/* We must have a power-of-2 number of pages to populate IDR1 */
	ARM_VSMMU_TRAP_PAGE_PAD0,
	ARM_VSMMU_TRAP_PAGE_PAD1,
	ARM_VSMMU_TRAP_PAGE_PAD2,

	ARM_VSMMU_MIN_GLOBAL_PAGES,
};

/* R/W registers in global register space 0 */
struct arm_vsmmu_gr0_reg_state {
	#define GR0_SCR0_RESET_VAL	sCR0_CLIENTPD
	#define GR0_SCR0_RAZ_WI		0xf020c3f8
	u32	scr0;

	#define GR0_S2CR_RAZ_WI		0xf00c0000
	u32	s2cr[ARM_SMMU_MAX_SMRS];
};

/* R/W registers in global register space 1 */
struct arm_vsmmu_gr1_reg_state {
	#define GR1_CBAR_RESET_VAL	CBAR_TYPE_S1_TRANS_S2_BYPASS
	#define GR1_CBAR_WI		((CBAR_TYPE_MASK << CBAR_TYPE_SHIFT) |\
					 (CBAR_S1_BSU_MASK << CBAR_S1_BSU_SHIFT))
	u32	cbar[ARM_SMMU_MAX_CBS];
};

struct arm_vsmmu_device {
	/* KVM context for the virtual machine using this vSMMU */
	struct kvm			*kvm;
	/* Emulated accesses */
	struct kvm_io_device		mmio_gr_dev;
	/* We really shouldn't need to trap this */
	struct kvm_io_device		mmio_cb_dev;

	/* Geometry */
	phys_addr_t			base;
	phys_addr_t			size;
	atomic_t			num_context_banks;
	unsigned int			virq;

	/* Virtual register state */
	struct arm_vsmmu_gr0_reg_state	gr0;
	struct arm_vsmmu_gr1_reg_state	gr1;

	/* Virtual StreamID allocation */
	DECLARE_BITMAP(vsid_map, ARM_SMMU_MAX_SMRS);

	/* Virtual -> s1 physical context bank mapping */
	int				*cbs;

	/* vSID -> s2 physical context bank mapping */
	int				s2_cbs[ARM_SMMU_MAX_CBS];

	/* vSID -> IOMMU group mapping */
	struct iommu_group		*groups[ARM_SMMU_MAX_SMRS];

	/* Corresponding physical SMMU */
	struct arm_smmu_device		*smmu;

	/* Lock to protect vSMMU state */
	spinlock_t			lock;
};

static int arm_vsmmu_global_top(struct arm_vsmmu_device *vsmmu)
{
	int num_cbs;
	u32 numpages;
	unsigned long pagesize = 1 << vsmmu->smmu->pgshift;

	num_cbs = atomic_read(&vsmmu->num_context_banks);
	if (num_cbs > 0)
		num_cbs = roundup_pow_of_two(num_cbs);

	numpages = max_t(u32, num_cbs, ARM_VSMMU_MIN_GLOBAL_PAGES);
	return numpages * pagesize;
}

static int arm_smmu_id_bits_to_size(int bits)
{
	switch (bits) {
	case 32:
		return 0;
	case 36:
		return 1;
	case 39: /* Advertise 39-bit input size as 40-bit */
	case 40:
		return 2;
	case 42:
		return 3;
	case 44:
		return 4;
	case 48:
	default:
		return 5;
	}
}

static int
arm_vsmmu_read_id(struct arm_vsmmu_device *vsmmu, u32 offset, u32 *val)
{
	u32 data, numpagendxb, ubs, oas;
	int num_cbs;
	struct arm_smmu_device *smmu = vsmmu->smmu;

	switch (offset) {
	case ARM_SMMU_GR0_ID0:
		data = smmu->idr[0];

		data |= ID0_S1TS | ID0_ATOSNS |
			(ID0_NUMSIDB_MASK << ID0_NUMSIDB_SHIFT);
		data &= ~(ID0_S2TS | ID0_NTS | ID0_SMS |
			  (ID0_NUMIRPT_MASK << ID0_NUMIRPT_SHIFT) |
			  ID0_BTM | ID0_EXIDS |
			  (ID0_NUMSMRG_MASK << ID0_NUMSMRG_SHIFT));

		*val = cpu_to_le32(data);
		break;
	case ARM_SMMU_GR0_ID1:
		data = smmu->idr[1];
		num_cbs = atomic_read(&vsmmu->num_context_banks);
		numpagendxb =
			ilog2(arm_vsmmu_global_top(vsmmu) >> smmu->pgshift) - 1;

		data &= ~((ID1_NUMPAGENDXB_MASK << ID1_NUMPAGENDXB_SHIFT) |
			  (ID1_NUMS2CB_MASK << ID1_NUMS2CB_SHIFT) |
			  ID1_SMCD |
			  (ID1_NUMCB_MASK << ID1_NUMCB_SHIFT));
		data |= ((numpagendxb << ID1_NUMPAGENDXB_SHIFT) |
			 (num_cbs << ID1_NUMCB_SHIFT));

		*val = cpu_to_le32(data);
		break;
	case ARM_SMMU_GR0_ID2:
		data = smmu->idr[2];
		ubs = arm_smmu_id_bits_to_size(smmu->s1_input_size);
		oas = arm_smmu_id_bits_to_size(smmu->s2_input_size);

		data &= ~((ID2_UBS_MASK << ID2_UBS_SHIFT) |
			  (ID2_OAS_MASK << ID2_OAS_SHIFT) |
			  (ID2_IAS_MASK << ID2_IAS_SHIFT));
		data |= ((ubs << ID2_UBS_SHIFT) |
			 (oas << ID2_OAS_SHIFT) |
			 (oas << ID2_IAS_SHIFT));

		*val = cpu_to_le32(data);
		break;
	default:
		return -EFAULT;
	}

	return 0;
}

static int
arm_vsmmu_gr0_read(struct arm_vsmmu_device *vsmmu, u32 offset, u32 *val)
{
	int vs2crndx;

	/* Global config */
	if (offset == ARM_SMMU_GR0_sCR0) {
		*val = vsmmu->gr0.scr0;
		return 0;
	}

	/* Identification */
	if (offset >= ARM_SMMU_GR0_ID0 && offset <= ARM_SMMU_GR0_ID7)
		return arm_vsmmu_read_id(vsmmu, offset, val);

	/* TODO: fault registers */

	/* S2CRs */
	if (offset >= ARM_SMMU_GR0_S2CR(0) &&
			offset < ARM_SMMU_GR0_S2CR(ARM_SMMU_MAX_SMRS)) {
		vs2crndx = (offset - ARM_SMMU_GR0_S2CR(0)) >> 2;
		*val = vsmmu->gr0.s2cr[vs2crndx];
		return 0;
	}

	/* PrimecellID. It's IMPDEF, but this driver prints it out... */
	if (offset == ARM_SMMU_GR0_PIDR2) {
		*val = cpu_to_le32(1 << PIDR2_ARCH_SHIFT);
		return 0;
	}

	*val = 0;
	return 0;
}

static int
arm_vsmmu_inject_cfg_fault(struct arm_vsmmu_device *vsmmu, int vcbndx)
{
	/*
	 * TODO: Report global cfg fault for vcbndx. This means:
	 *
	 * - Updating GFSR (cfg, multi), GFSYNR0 (0), GFAR (vcb offset)
	 * - Injecting a virq
	 */

	if (!vsmmu->virq)
		return 0;

	return kvm_vgic_inject_irq(vsmmu->kvm, 0, vsmmu->virq, 1);
}

static int
arm_vsmmu_sync_s2crs_get_vcbndx(struct arm_vsmmu_device *vsmmu, int vsid)
{
	u32 s2cr;
	bool s1_bypass;
	int i, vcbndx = -1;
	void __iomem *gr0_base = ARM_SMMU_GR0(vsmmu->smmu);
	struct iommu_group *group = vsmmu->groups[vsid];
	struct arm_smmu_master_cfg *cfg = iommu_group_get_iommudata(group);

	/* If global bypass is enabled, force stage-2 only */
	s1_bypass = !!(vsmmu->gr0.scr0 & sCR0_CLIENTPD);

	/* Parse the vS2CR */
	s2cr = vsmmu->gr0.s2cr[vsid];
	switch (s2cr & (S2CR_TYPE_MASK << S2CR_TYPE_SHIFT)) {
	case S2CR_TYPE_TRANS:
		/* Follow the breadcrumbs */
		vcbndx = (s2cr >> S2CR_CBNDX_SHIFT) & S2CR_CBNDX_MASK;
		break;
	case S2CR_TYPE_BYPASS:
		s1_bypass = true;
		break;
	case S2CR_TYPE_FAULT:
		break;
	default:
		s2cr = S2CR_TYPE_FAULT;
		/* End of the line. This translation terminates here. */
	}

	if (s1_bypass) {
		u32 s2cbndx = vsmmu->s2_cbs[vsid];
		/* Convert to stage-2 translation only */
		s2cr &= ~((S2CR_TYPE_MASK << S2CR_TYPE_SHIFT) |
			  (S2CR_CBNDX_MASK << S2CR_CBNDX_SHIFT));
		s2cr |= S2CR_TYPE_TRANS | (s2cbndx << S2CR_CBNDX_SHIFT);
	}

	/* Update physical S2CRs */
	for (i = 0; i < cfg->num_streamids; ++i) {
		u32 idx = cfg->smrs ? cfg->smrs[i].idx : cfg->streamids[i];
		writel_relaxed(s2cr, gr0_base + ARM_SMMU_GR0_S2CR(idx));
	}

	return vcbndx < atomic_read(&vsmmu->num_context_banks) ? vcbndx : -1;
}

static void
arm_vsmmu_sync_s1_cbar(struct arm_vsmmu_device *vsmmu, int vsid, u32 vcbndx)
{
	u32 cbar, s1cbndx = vsmmu->cbs[vcbndx], s2cbndx = vsmmu->s2_cbs[vsid];
	void __iomem *gr1_base = ARM_SMMU_GR1(vsmmu->smmu);

	/* Grab the vCBAR and check that it's valid */
	cbar = vsmmu->gr1.cbar[vcbndx];

	/*
	 * We can't give the guest a hypervisor context and
	 * God only knows why the architecture allows this.
	 */
	if (cbar & CBAR_S1_HYPC)
		arm_vsmmu_inject_cfg_fault(vsmmu, vcbndx);

	/*
	 * Weird and whacky VMIDs strike again! If the guest
	 * tries to use them, slap its wrists.
	 */
	if (cbar & (CBAR_VMID_MASK << CBAR_VMID_SHIFT))
		arm_vsmmu_inject_cfg_fault(vsmmu, vcbndx);

	/*
	 * Link the two context banks for nested translation.
	 * We use the same VMID as stage-2 so that TLB-invalidation
	 * isn't insane.
	 */
	cbar &= ~((CBAR_VMID_MASK << CBAR_VMID_SHIFT) |
		  (CBAR_TYPE_MASK << CBAR_TYPE_SHIFT) |
		  (CBAR_S1_S2_CBNDX_MASK << CBAR_S1_S2_CBNDX_SHIFT));
	cbar |= (ARM_SMMU_CBNDX_TO_VMID(s2cbndx) << CBAR_VMID_SHIFT) |
		CBAR_TYPE_S1_TRANS_S2_TRANS |
		(s2cbndx << CBAR_S1_S2_CBNDX_SHIFT);

	/* Update physical stage-1 CBAR */
	writel_relaxed(cbar, gr1_base + ARM_SMMU_GR1_CBAR(s1cbndx));
}

static void
arm_vsmmu_tlb_inv_context_by_vsid(struct arm_vsmmu_device *vsmmu, int vsid)
{
	u32 vmid = ARM_SMMU_CBNDX_TO_VMID(vsmmu->s2_cbs[vsid]);
	arm_smmu_tlb_inv_context_by_vmid(vsmmu->smmu, vmid);
}

/*
 * Synchronise the real SMMU hardware based on the vSMMU state. This
 * could be optimised by keeping track of the dirty portions of the
 * vSMMU register file, but simply recompute everything for now (dirty
 * tracking would require a lock around the fault handling code)
 */
static void arm_vsmmu_sync_vsid(struct arm_vsmmu_device *vsmmu, int vsid)
{
	/* Program S2CRs and determine the vCB index */
	int vcbndx = arm_vsmmu_sync_s2crs_get_vcbndx(vsmmu, vsid);
	if (vcbndx >= 0) {
		/* We have a vCB, so update the stage-1 pCB */
		arm_vsmmu_sync_s1_cbar(vsmmu, vsid, vcbndx);
	}

	/* Nuke the TLB. This is where the dirty tracking would really help */
	arm_vsmmu_tlb_inv_context_by_vsid(vsmmu, vsid);
}

static void arm_vsmmu_sync(struct arm_vsmmu_device *vsmmu)
{
	int vsid;

	for_each_set_bit(vsid, vsmmu->vsid_map, ARM_SMMU_MAX_SMRS)
		arm_vsmmu_sync_vsid(vsmmu, vsid);

	arm_smmu_tlb_sync(vsmmu->smmu);
}

static int
arm_vsmmu_gr0_write(struct arm_vsmmu_device *vsmmu, u32 offset, u32 data)
{
	int vs2crndx;

	/* Global config */
	if (offset == ARM_SMMU_GR0_sCR0) {
		data &= ~GR0_SCR0_RAZ_WI;
		vsmmu->gr0.scr0 = data;
		return 0;
	}

	/* TODO: fault registers */

	/* S2CRs */
	if (offset >= ARM_SMMU_GR0_S2CR(0) &&
	    offset < ARM_SMMU_GR0_S2CR(ARM_SMMU_MAX_SMRS)) {
		vs2crndx = (offset - ARM_SMMU_GR0_S2CR(0)) >> 2;
		data &= ~GR0_S2CR_RAZ_WI;
		vsmmu->gr0.s2cr[vs2crndx] = data;
		return 0;
	}

	/*
	 * The SMMU architecture has a spaced out understanding of VMIDs, so
	 * just nuke the entire TLB for the relevant CBs and get on with our
	 * lives.
	 */
	if (offset >= ARM_SMMU_GR0_TLBIVMID &&
	    offset <= ARM_SMMU_GR0_TLBIALLH) {
		return 0;
	}

	return 0;
}

static int
arm_vsmmu_gr1_read(struct arm_vsmmu_device *vsmmu, u32 offset, u32 *val)
{
	int num_cbs, vcbndx;

	/* CBARs */
	num_cbs = atomic_read(&vsmmu->num_context_banks);
	if (offset >= ARM_SMMU_GR1_CBAR(0) &&
	    offset < ARM_SMMU_GR1_CBAR(num_cbs)) {
		vcbndx = (offset - ARM_SMMU_GR1_CBAR(0)) >> 2;
		*val = vsmmu->gr1.cbar[vcbndx];
		return 0;
	}

	/* CBFRSYNRAs */
	if (offset >= ARM_SMMU_GR1_CBFRSYNRA(0) &&
	    offset < ARM_SMMU_GR1_CBFRSYNRA(num_cbs)) {
		/* TODO: fault reporting */
		return -EFAULT;
	}

	*val = 0;
	return 0;
}

static int
arm_vsmmu_gr1_write(struct arm_vsmmu_device *vsmmu, u32 offset, u32 data)
{
	int num_cbs, vcbndx;

	/* CBARs */
	num_cbs = atomic_read(&vsmmu->num_context_banks);
	if (offset >= ARM_SMMU_GR1_CBAR(0) &&
	    offset < ARM_SMMU_GR1_CBAR(num_cbs)) {
		vcbndx = (offset - ARM_SMMU_GR1_CBAR(0)) >> 2;
		data &= ~GR1_CBAR_WI;
		vsmmu->gr1.cbar[vcbndx] = data;
		return 0;
	}

	return 0;
}

static int
arm_vsmmu_gr_read(struct kvm_io_device *this, gpa_t addr, int len, void *val)
{
	struct arm_vsmmu_device *vsmmu
		= container_of(&this->ops, struct arm_vsmmu_device,
			       mmio_gr_dev.ops);
	u32 pgshift = vsmmu->smmu->pgshift;
	u32 page = (addr - vsmmu->base) >> pgshift;
	u32 offset = addr & ((1 << pgshift) - 1);

	if ((addr & 0x3) || (len != 4))
		return -EFAULT;

	switch (page) {
	case ARM_VSMMU_TRAP_PAGE_GR0:
		return arm_vsmmu_gr0_read(vsmmu, offset, val);
	case ARM_VSMMU_TRAP_PAGE_GR1:
		return arm_vsmmu_gr1_read(vsmmu, offset, val);
	default:
		/* RAZ */
		memset(val, 0, len);
	}

	return 0;
}

static int arm_vsmmu_gr_write(struct kvm_io_device *this, gpa_t addr, int len,
			      const void *val)
{
	struct arm_vsmmu_device *vsmmu
		= container_of(&this->ops, struct arm_vsmmu_device,
			       mmio_gr_dev.ops);
	u32 pgshift = vsmmu->smmu->pgshift;
	u32 page = (addr - vsmmu->base) >> pgshift;
	u32 offset = addr & ((1 << pgshift) - 1);
	int ret;
	u32 data;

	if ((addr & 0x3) || (len != 4))
		return -EFAULT;

	data = *(u32 *)val;

	switch (page) {
	case ARM_VSMMU_TRAP_PAGE_GR0:
		ret = arm_vsmmu_gr0_write(vsmmu, offset, data);
		break;
	case ARM_VSMMU_TRAP_PAGE_GR1:
		ret = arm_vsmmu_gr1_write(vsmmu, offset, data);
		break;
	}

	if (!ret)
		arm_vsmmu_sync(vsmmu);

	/* WI */
	return 0;
}

static struct kvm_io_device_ops arm_vsmmu_mmio_gr_ops = {
	.read	= arm_vsmmu_gr_read,
	.write	= arm_vsmmu_gr_write,
};

/*
 * TODO: Context interrupts are difficult to get right.
 * We can't let the guest have direct access to the fault registers, because
 * it could spam the host with physical interrupts. Instead, we need to install
 * a handler on the s1 context, then on an exception we do:
 *
 *  - Kill CFIE so we don't see further irqs
 *  - Inject a level-triggered virq for the vcb
 *  - When the guest has handled the virq, unmask the physical interrupt
 *    -> Note that IRQ forwarding won't work because the irq : virq relation
 *       isn't 1:1.
 *
 * To detect that the guest has handled the virq, need to look at writes to the
 * fsr (cb and global).
 *
 */
static int
arm_vsmmu_cb_read(struct kvm_io_device *this, gpa_t addr, int len, void *val)
{
	struct arm_vsmmu_device *vsmmu
		= container_of(&this->ops, struct arm_vsmmu_device,
			       mmio_cb_dev.ops);
	struct arm_smmu_device *smmu = vsmmu->smmu;
	u32 pgshift = smmu->pgshift;
	u32 offset = addr & ((1 << pgshift) - 1);
	phys_addr_t vcb_base = vsmmu->base + arm_vsmmu_global_top(vsmmu);
	u32 vcbndx = (addr - vcb_base) >> pgshift;
	void __iomem *base = ARM_SMMU_CB_BASE(smmu);
	u32 data;

	if ((addr & 0x3) || (len != 4))
		return -EFAULT;

	if (vcbndx >= atomic_read(&vsmmu->num_context_banks))
		return -EFAULT;

	/* Filter out tricky registers */
	switch (offset) {
	case ARM_SMMU_CB_ACTLR:
		/* Oh no you don't! */
		*(u32 *)val = 0;
		return 0;
	}

	data = readl_relaxed(base + ARM_SMMU_CB(smmu, vsmmu->cbs[vcbndx]));
	*(u32 *)val = data;
	return 0;
}

static int arm_vsmmu_cb_write(struct kvm_io_device *this, gpa_t addr, int len,
			      const void *val)
{
	struct arm_vsmmu_device *vsmmu
		= container_of(&this->ops, struct arm_vsmmu_device,
			       mmio_cb_dev.ops);
	struct arm_smmu_device *smmu = vsmmu->smmu;
	u32 pgshift = smmu->pgshift;
	u32 offset = addr & ((1 << pgshift) - 1);
	phys_addr_t vcb_base = vsmmu->base + arm_vsmmu_global_top(vsmmu);
	u32 vcbndx = (addr - vcb_base) >> pgshift;
	void __iomem *base = ARM_SMMU_CB_BASE(smmu);
	u32 data;

	if ((addr & 0x3) || (len != 4))
		return -EFAULT;

	if (vcbndx >= atomic_read(&vsmmu->num_context_banks))
		return -EFAULT;

	switch (offset) {
	case ARM_SMMU_CB_ACTLR:
		return 0;
	}

	data = *(u32 *)val;
	writel_relaxed(data, base + ARM_SMMU_CB(smmu, vsmmu->cbs[vcbndx]));
	return 0;
}

static struct kvm_io_device_ops arm_vsmmu_mmio_cb_ops = {
	.read	= arm_vsmmu_cb_read,
	.write	= arm_vsmmu_cb_write,
};

static int arm_vsmmu_alloc_s1_contexts(struct arm_vsmmu_device *vsmmu)
{
	int i, num_vcbs, start, end, ret;
	struct arm_smmu_device *smmu = vsmmu->smmu;

	start = smmu->num_s2_context_banks;
	end = smmu->num_context_banks;
	num_vcbs = atomic_read(&vsmmu->num_context_banks);

	if (WARN(vsmmu->cbs, "vSMMU context map already initialised?!"))
		return -EEXIST;

	vsmmu->cbs = kmalloc_array(num_vcbs, sizeof(*vsmmu->cbs), GFP_KERNEL);
	if (!vsmmu->cbs)
		return -ENOMEM;

	for (i = 0; i < num_vcbs; ++i) {
		ret = __arm_smmu_alloc_bitmap(smmu->context_map, start, end);
		if (IS_ERR_VALUE(ret))
			goto out_free_cbs;

		vsmmu->cbs[i] = ret;
	}

	/*
	 * TODO: request_irq for s1 context fault handlers. This is a
	 * PITA because shared irqs on the smmu mean we can't just pass
	 * the vsmmu pointer as data.
	 */

	return 0;

out_free_cbs:
	while (--i >= 0)
		__arm_smmu_free_bitmap(smmu->context_map, vsmmu->cbs[i]);

	kfree(vsmmu->cbs);
	vsmmu->cbs = NULL;
	return ret;
}

static int arm_vsmmu_size(struct kvm_device *dev, u64 __user *addr)
{
	u64 size;
	struct arm_vsmmu_device *vsmmu = dev->private;

	spin_lock(&vsmmu->lock);
	size = arm_vsmmu_global_top(vsmmu) * 2;
	spin_unlock(&vsmmu->lock);

	return put_user(size, addr);
}

static void arm_vsmmu_init_register_file(struct arm_vsmmu_device *vsmmu)
{
	int i, num_vcbs = atomic_read(&vsmmu->num_context_banks);

	vsmmu->gr0.scr0 = GR0_SCR0_RESET_VAL;

	for (i = 0; i < num_vcbs; ++i)
		vsmmu->gr1.cbar[i] = GR1_CBAR_RESET_VAL;
}

static int arm_vsmmu_init(struct kvm_device *dev, u64 __user *addr)
{
	int len, ret;
	u64 base;
	phys_addr_t size;
	struct arm_vsmmu_device *vsmmu = dev->private;
	struct arm_smmu_device *smmu = vsmmu->smmu;

	if (!smmu)
		return -ENODEV;

	/* FIXME: I think get_user_8 is going in for 3.17 */
	/* get_user can't deal with 64-bit quantities on ARM */
	if (copy_from_user(&base, addr, sizeof(base)))
		return -EFAULT;

	if (base & ((1 << smmu->pgshift) - 1))
		return -EINVAL;

	/* Guard against parallel instantiation */
	spin_lock(&vsmmu->lock);
	if (vsmmu->size) {
		ret = -EEXIST;
		goto err_unlock;
	}

	len = arm_vsmmu_global_top(vsmmu);
	size = len * 2;
	if ((base + size) & ~KVM_PHYS_MASK) {
		ret = -E2BIG;
		goto err_unlock;
	}

	vsmmu->base = base;
	vsmmu->size = size;
	vsmmu->mmio_gr_dev.ops = &arm_vsmmu_mmio_gr_ops;
	vsmmu->mmio_cb_dev.ops = &arm_vsmmu_mmio_cb_ops;
	spin_unlock(&vsmmu->lock);

	mutex_lock(&dev->kvm->slots_lock);
	ret = kvm_io_bus_register_dev(dev->kvm, KVM_MMIO_BUS, base, len,
				      &vsmmu->mmio_gr_dev);
	if (ret)
		goto err_reset;

	base += len;
	ret = kvm_io_bus_register_dev(dev->kvm, KVM_MMIO_BUS, base, len,
				      &vsmmu->mmio_cb_dev);
	if (ret)
		goto err_reset;
	mutex_unlock(&dev->kvm->slots_lock);

	ret = arm_vsmmu_alloc_s1_contexts(vsmmu);
	if (ret)
		return ret;

	arm_vsmmu_init_register_file(vsmmu);
	return 0;

err_reset:
	mutex_unlock(&dev->kvm->slots_lock);
	spin_lock(&vsmmu->lock);
	vsmmu->base = vsmmu->size = 0;
err_unlock:
	spin_unlock(&vsmmu->lock);
	return ret;
}

static int arm_vsmmu_irq(struct kvm_device *dev, u32 __user *addr)
{
	u32 virq;
	int ret = 0;
	struct arm_vsmmu_device *vsmmu = dev->private;

	if (get_user(virq, addr))
		return -EFAULT;

	spin_lock(&vsmmu->lock);
	if (vsmmu->size)
		ret = -EBUSY;
	else
		vsmmu->virq = virq;
	spin_unlock(&vsmmu->lock);

	return ret;
}

static int arm_vsmmu_cfg_set(struct kvm_device *dev, u64 attr, u64 addr)
{
	switch (attr) {
	case KVM_DEV_ARM_SMMU_V2_CFG_INIT:
		return arm_vsmmu_init(dev, (u64 __user *)(unsigned long)addr);
	case KVM_DEV_ARM_SMMU_V2_CFG_IRQ:
		return arm_vsmmu_irq(dev, (u32 __user *)(unsigned long)addr);
	default:
		return -ENXIO;
	}
}

static int arm_vsmmu_cfg_get(struct kvm_device *dev, u64 attr, u64 addr)
{
	switch (attr) {
	case KVM_DEV_ARM_SMMU_V2_CFG_SIZE:
		return arm_vsmmu_size(dev, (u64 __user *)(unsigned long)addr);
	default:
		return -ENXIO;
	}
}

#ifdef CONFIG_VFIO
#include <linux/file.h>
#include <linux/vfio.h>

/* For IOMMU groups, find the first device in the group */
static int __arm_vsmmu_get_group_dev(struct device *dev, void *data)
{
	struct device **devp = data;

	*devp = dev;
	return 1;
}

static int arm_vsmmu_get_s2_cbndx(struct device *dev)
{
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain;
	struct iommu_domain *domain = dev->archdata.iommu;
	int ret = -ENODEV;

	if (!domain)
		return ret;

	smmu_domain = domain->priv;
	if (!smmu_domain)
		return ret;

	spin_lock_irqsave(&smmu_domain->lock, flags);
	if (smmu_domain->stage == ARM_SMMU_DOMAIN_NESTED)
		ret = smmu_domain->cfg.cbndx;
	spin_unlock_irqrestore(&smmu_domain->lock, flags);

	return ret;
}

static int arm_vsmmu_find_vsid_by_group(struct arm_vsmmu_device *vsmmu,
					struct iommu_group *group)
{
	int i;

	for (i = 0; i < ARM_SMMU_MAX_SMRS; ++i)
		if (vsmmu->groups[i] == group)
			return i;

	return -ENODEV;
}

/*
 * Add an IOMMU group to the vsmmu. Note that we hold a reference to
 * the VFIO group, so we can rely on the stage-2 mapping staying around
 * in the physical SMMU.
 */
static int arm_vsmmu_iommu_group_add(struct arm_vsmmu_device *vsmmu,
				     struct iommu_group *group,
				     u16 vsid)
{
	struct arm_smmu_device *smmu;
	struct arm_smmu_master_cfg *cfg;
	struct device *dev;
	int cbndx, ret = 0;

	if (vsid >= ARM_SMMU_MAX_SMRS)
		return -ERANGE;

	iommu_group_for_each_dev(group, &dev, __arm_vsmmu_get_group_dev);
	if (!dev)
		return -ENODEV;

	smmu = find_smmu_for_device(dev);
	if (!smmu)
		return -ENODEV;

	cfg = find_smmu_master_cfg(dev);
	if (!cfg)
		return -ENODEV;

	/* Check that we have a stage-2 configured for nesting */
	cbndx = arm_vsmmu_get_s2_cbndx(dev);
	if (IS_ERR_VALUE(cbndx))
		return cbndx;

	spin_lock(&vsmmu->lock);

	/* Avoid duplicate registrations */
	if (arm_vsmmu_find_vsid_by_group(vsmmu, group) > 0) {
		ret = -EEXIST;
		goto err_unlock;
	}

	/* Allocate the vSID on the vSMMU */
	if (__test_and_set_bit(vsid, vsmmu->vsid_map)) {
		ret = -ENOSPC;
		goto err_unlock;
	}

	if (vsmmu->size) {
		ret = -EBUSY;
		goto err_free_vsid;
	}

	if (!vsmmu->smmu) {
		if (smmu->version > 1 &&
		    (smmu->features & ARM_SMMU_FEAT_TRANS_NESTED)) {
			vsmmu->smmu = smmu;
		} else {
			ret = -EOPNOTSUPP;
			goto err_free_vsid;
		}
	} else if (vsmmu->smmu != smmu) {
		ret = -EINVAL;
		goto err_free_vsid;
	}

	vsmmu->groups[vsid] = group;
	vsmmu->s2_cbs[vsid] = cbndx;
	atomic_inc(&vsmmu->num_context_banks);

	spin_unlock(&vsmmu->lock);
	return ret;

err_free_vsid:
	__arm_smmu_free_bitmap(vsmmu->vsid_map, vsid);
err_unlock:
	spin_unlock(&vsmmu->lock);
	return ret;
}

static int arm_vsmmu_iommu_group_del(struct arm_vsmmu_device *vsmmu,
				     struct iommu_group *group)
{
	int vsid, ret = 0;

	spin_lock(&vsmmu->lock);
	if (vsmmu->size) {
		ret = -EBUSY;
		goto out_unlock;
	}

	vsid = arm_vsmmu_find_vsid_by_group(vsmmu, group);
	if (vsid < 0) {
		ret = -ENODEV;
		goto out_unlock;
	}

	vsmmu->groups[vsid] = NULL;
	__arm_smmu_free_bitmap(vsmmu->vsid_map, vsid);

	if (!atomic_dec_return(&vsmmu->num_context_banks))
		vsmmu->smmu = NULL;

out_unlock:
	spin_unlock(&vsmmu->lock);
	return ret;
}

/* External vfio_group accessors copied blindly from virt/kvm/vfio.c */
static struct vfio_group *kvm_vfio_group_get_external_user(struct file *filep)
{
	struct vfio_group *vfio_group;
	struct vfio_group *(*fn)(struct file *);

	fn = symbol_get(vfio_group_get_external_user);
	if (!fn)
		return ERR_PTR(-EINVAL);

	vfio_group = fn(filep);

	symbol_put(vfio_group_get_external_user);

	return vfio_group;
}

static void kvm_vfio_group_put_external_user(struct vfio_group *vfio_group)
{
	void (*fn)(struct vfio_group *);

	fn = symbol_get(vfio_group_put_external_user);
	if (!fn)
		return;

	fn(vfio_group);

	symbol_put(vfio_group_put_external_user);
}

static int arm_vsmmu_vfio_external_user_iommu_id(struct vfio_group *vfio_group)
{
	int ret;
	int (*fn)(struct vfio_group *);

	fn = symbol_get(vfio_external_user_iommu_id);
	if (!fn)
		return -EINVAL;

	ret = fn(vfio_group);

	symbol_put(vfio_external_user_iommu_id);

	return ret;
}

static struct vfio_group *arm_vsmmu_get_vfio_group(int fd)
{
	struct fd f;
	struct vfio_group *vfio_group;

	f = fdget(fd);
	if (!f.file)
		return ERR_PTR(-EBADF);

	vfio_group = kvm_vfio_group_get_external_user(f.file);
	fdput(f);

	return vfio_group;
}

static void arm_vsmmu_put_vfio_group(struct vfio_group *vfio_group)
{
	kvm_vfio_group_put_external_user(vfio_group);
}

static struct iommu_group *
arm_vsmmu_vfio_to_iommu_group(struct vfio_group *vfio_group)
{
	struct iommu_group *iommu_group;
	int id = arm_vsmmu_vfio_external_user_iommu_id(vfio_group);

	if (id < 0)
		return ERR_PTR(id);

	iommu_group = iommu_group_get_by_id(id);
	return iommu_group ?: ERR_PTR(-ENODEV);
}

static int arm_vsmmu_vfio_set(struct kvm_device *dev, u64 attr, u64 addr)
{
	int fd, ret;
	struct vfio_group *vfio_group;
	struct iommu_group *iommu_group;
	struct arm_smmu_v2_vfio_group_sid group_sid;
	struct arm_vsmmu_device *vsmmu = dev->private;
	void __user *uaddr = (void __user *)(unsigned long)addr;

	switch (attr) {
	case KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_ADD:
		if (copy_from_user(&group_sid, uaddr, sizeof(group_sid)))
			return -EFAULT;
		fd = group_sid.fd;
		break;
	case KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_DEL:
		if (get_user(fd, (int __user *)uaddr))
			return -EFAULT;
		break;
	default:
		return -ENXIO;
	}

	vfio_group = arm_vsmmu_get_vfio_group(fd);
	if (IS_ERR(vfio_group))
		return PTR_ERR(vfio_group);

	iommu_group = arm_vsmmu_vfio_to_iommu_group(vfio_group);
	if (IS_ERR(iommu_group))
		goto out_put_group;

	switch (attr) {
	case KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_ADD:
		ret = arm_vsmmu_iommu_group_add(vsmmu, iommu_group,
						group_sid.sid);
		break;
	case KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_DEL:
		ret = arm_vsmmu_iommu_group_del(vsmmu, iommu_group);
		break;
	}

out_put_group:
	arm_vsmmu_put_vfio_group(vfio_group);
	return ret;
}
#else
static int arm_vsmmu_vfio_set(struct kvm_device *dev, u64 attr, u64 addr)
{
	return -ENXIO;
}
#endif	/* CONFIG_VFIO */

static int arm_vsmmu_create(struct kvm_device *dev, u32 type)
{
	struct arm_vsmmu_device *vsmmu;

	vsmmu = kzalloc(sizeof(*vsmmu), GFP_KERNEL);
	if (!vsmmu)
		return -ENOMEM;

	spin_lock_init(&vsmmu->lock);
	dev->private = vsmmu;
	vsmmu->kvm = dev->kvm;
	return 0;
}

static void arm_vsmmu_destroy(struct kvm_device *dev)
{
	int i, num_vcbs;
	struct arm_vsmmu_device *vsmmu = dev->private;

	if (!vsmmu->size)
		goto out_free_vsmmu;

	num_vcbs = atomic_read(&vsmmu->num_context_banks);
	for (i = 0; i < num_vcbs; ++i)
		__arm_smmu_free_bitmap(vsmmu->smmu->context_map, vsmmu->cbs[i]);

	kfree(vsmmu->cbs);
out_free_vsmmu:
	kfree(vsmmu);
}

static int
arm_vsmmu_set_attr(struct kvm_device *dev, struct kvm_device_attr *attr)
{
	switch (attr->group) {
	case KVM_DEV_ARM_SMMU_V2_CFG:
		return arm_vsmmu_cfg_set(dev, attr->attr, attr->addr);
	case KVM_DEV_ARM_SMMU_V2_VFIO:
		return arm_vsmmu_vfio_set(dev, attr->attr, attr->addr);
	default:
		return -ENXIO;
	}
}

static int
arm_vsmmu_get_attr(struct kvm_device *dev, struct kvm_device_attr *attr)
{
	switch (attr->group) {
	case KVM_DEV_ARM_SMMU_V2_CFG:
		return arm_vsmmu_cfg_get(dev, attr->attr, attr->addr);
	default:
		return -ENXIO;
	}
}

static int
arm_vsmmu_has_attr(struct kvm_device *dev, struct kvm_device_attr *attr)
{
	switch (attr->group) {
	case KVM_DEV_ARM_SMMU_V2_CFG:
		switch (attr->attr) {
		case KVM_DEV_ARM_SMMU_V2_CFG_INIT:
		case KVM_DEV_ARM_SMMU_V2_CFG_IRQ:
		case KVM_DEV_ARM_SMMU_V2_CFG_SIZE:
			return 0;
		}
		break;
#ifdef CONFIG_VFIO
	case KVM_DEV_ARM_SMMU_V2_VFIO:
		switch (attr->attr) {
		case KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_ADD:
		case KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_DEL:
			return 0;
		}
		break;
#endif
	}

	return -ENXIO;
}

static struct kvm_device_ops kvm_arm_vsmmu_v2_ops = {
	.name		= "kvm-arm-vsmmu-v2",
	.create		= arm_vsmmu_create,
	.destroy	= arm_vsmmu_destroy,
	.set_attr	= arm_vsmmu_set_attr,
	.get_attr	= arm_vsmmu_get_attr,
	.has_attr	= arm_vsmmu_has_attr,
};

#endif /* CONFIG_KVM */

static int __init arm_smmu_init(void)
{
	struct device_node *np;
	int ret;

	/*
	 * Play nice with systems that don't have an ARM SMMU by checking that
	 * an ARM SMMU exists in the system before proceeding with the driver
	 * and IOMMU bus operation registration.
	 */
	np = of_find_matching_node(NULL, arm_smmu_of_match);
	if (!np)
		return 0;

	of_node_put(np);

	ret = platform_driver_register(&arm_smmu_driver);
	if (ret)
		return ret;

	/* Oh, for a proper bus abstraction */
	if (!iommu_present(&platform_bus_type))
		bus_set_iommu(&platform_bus_type, &arm_smmu_ops);

#ifdef CONFIG_ARM_AMBA
	if (!iommu_present(&amba_bustype))
		bus_set_iommu(&amba_bustype, &arm_smmu_ops);
#endif

#ifdef CONFIG_PCI
	if (!iommu_present(&pci_bus_type))
		bus_set_iommu(&pci_bus_type, &arm_smmu_ops);
#endif

#ifdef CONFIG_KVM
	ret = kvm_register_device_ops(&kvm_arm_vsmmu_v2_ops,
				      KVM_DEV_TYPE_ARM_SMMU_V2);
#endif
	return ret;
}

static void __exit arm_smmu_exit(void)
{
	return platform_driver_unregister(&arm_smmu_driver);
}

subsys_initcall(arm_smmu_init);
module_exit(arm_smmu_exit);

MODULE_DESCRIPTION("IOMMU API for ARM architected SMMU implementations");
MODULE_AUTHOR("Will Deacon <will.deacon@arm.com>");
MODULE_LICENSE("GPL v2");
