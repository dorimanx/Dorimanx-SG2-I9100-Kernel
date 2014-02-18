#ifdef CONFIG_MMU

/* the upper-most page table pointer */
extern pmd_t *top_pmd;

#define TOP_PTE(x)	pte_offset_kernel(top_pmd, x)

/*
 * 0xffff8000 to 0xffffffff is reserved for any ARM architecture
 * specific hacks for copying pages efficiently, while 0xffff4000
 * is reserved for VIPT aliasing flushing by generic code.
 *
 * Note that we don't allow VIPT aliasing caches with SMP.
 */
#define COPYPAGE_MINICACHE	0xffff8000
#define COPYPAGE_V6_FROM	0xffff8000
#define COPYPAGE_V6_TO		0xffffc000
/* PFN alias flushing, for VIPT caches */
#define FLUSH_ALIAS_START	0xffff4000

static inline void set_top_pte(unsigned long va, pte_t pte)
{
	set_pte_ext(TOP_PTE(va), pte, 0);
	local_flush_tlb_kernel_page(va);
}

static inline pte_t get_top_pte(unsigned long va)
{
	return *TOP_PTE(va);
}

static inline pmd_t *pmd_off_k(unsigned long virt)
{
	return pmd_offset(pud_offset(pgd_offset_k(virt), virt), virt);
}

struct mem_type {
	pteval_t prot_pte;
	pteval_t prot_pte_s2;
	pmdval_t prot_l1;
	pmdval_t prot_sect;
	unsigned int domain;
};

const struct mem_type *get_mem_type(unsigned int type);

extern void __flush_dcache_page(struct address_space *mapping, struct page *page);

#endif

#ifdef CONFIG_ZONE_DMA
extern phys_addr_t arm_dma_limit;
extern unsigned long arm_dma_pfn_limit;
#else
#define arm_dma_limit ((phys_addr_t)~0)
#define arm_dma_pfn_limit (~0ul >> PAGE_SHIFT)
#endif

#ifdef CONFIG_DMA_CMA
extern phys_addr_t arm_lowmem_limit;
#endif

void __init bootmem_init(void);
void arm_mm_memblock_reserve(void);
#ifdef CONFIG_DMA_CMA
void dma_contiguous_remap(void);
#endif

