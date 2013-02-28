/*
 * Copyright IBM Corp. 2011
 * Author(s): Jan Glauber <jang@linux.vnet.ibm.com>
 */
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <asm/pgtable.h>

static void change_page_attr(unsigned long addr, int numpages,
			     pte_t (*set) (pte_t))
{
	pte_t *ptep, pte;
	pmd_t *pmdp;
	pud_t *pudp;
	pgd_t *pgdp;
	int i;

	for (i = 0; i < numpages; i++) {
		pgdp = pgd_offset(&init_mm, addr);
		pudp = pud_offset(pgdp, addr);
		pmdp = pmd_offset(pudp, addr);
		if (pmd_huge(*pmdp)) {
			WARN_ON_ONCE(1);
			continue;
		}
		ptep = pte_offset_kernel(pmdp, addr);

		pte = *ptep;
		pte = set(pte);
		__ptep_ipte(addr, ptep);
		*ptep = pte;
		addr += PAGE_SIZE;
	}
}

int set_memory_ro(unsigned long addr, int numpages)
{
	change_page_attr(addr, numpages, pte_wrprotect);
	return 0;
}
EXPORT_SYMBOL_GPL(set_memory_ro);

int set_memory_rw(unsigned long addr, int numpages)
{
	change_page_attr(addr, numpages, pte_mkwrite);
	return 0;
}
EXPORT_SYMBOL_GPL(set_memory_rw);

/* not possible */
int set_memory_nx(unsigned long addr, int numpages)
{
	return 0;
}
EXPORT_SYMBOL_GPL(set_memory_nx);

int set_memory_x(unsigned long addr, int numpages)
{
	return 0;
}

#ifdef CONFIG_DEBUG_PAGEALLOC
void kernel_map_pages(struct page *page, int numpages, int enable)
{
	unsigned long address;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	int i;

	for (i = 0; i < numpages; i++) {
		address = page_to_phys(page + i);
		pgd = pgd_offset_k(address);
		pud = pud_offset(pgd, address);
		pmd = pmd_offset(pud, address);
		pte = pte_offset_kernel(pmd, address);
		if (!enable) {
			__ptep_ipte(address, pte);
			pte_val(*pte) = _PAGE_TYPE_EMPTY;
			continue;
		}
		pte_val(*pte) = __pa(address);
	}
}

#ifdef CONFIG_HIBERNATION
bool kernel_page_present(struct page *page)
{
	unsigned long addr;
	int cc;

	addr = page_to_phys(page);
	asm volatile(
		"	lra	%1,0(%1)\n"
		"	ipm	%0\n"
		"	srl	%0,28"
		: "=d" (cc), "+a" (addr) : : "cc");
	return cc == 0;
}
#endif /* CONFIG_HIBERNATION */

#endif /* CONFIG_DEBUG_PAGEALLOC */
