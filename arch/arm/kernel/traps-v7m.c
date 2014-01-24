#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/bootmem.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/sections.h>
#include <asm/page.h>
#include <asm/setup.h>
#include <asm/mpu.h>
#include <asm/mach/arch.h>
#include <asm/exception.h>

#define SCS_REGS_BASE   0xE000E000
/* configurable fault status register */
#define CFSR_ADDR       ((void *)((SCS_REGS_BASE)+0xed28))
/* hardfault status register */
#define HFSR_ADDR       ((void *)((SCS_REGS_BASE)+0xed2c))

enum interrupts {
	HARDFAULT	= 3,
	BUSFAULT	= 5,
	USAGEFAULT	= 6,
};

/*
 * The function prints information about the reason of the exception
 * @param name		name of current executable (process or kernel)
 * @param regs		state of registers when the exception occurs
 * @param in		IPSR, the number of the exception
 * @param addr		address caused the interrupt, or current pc
 * @param hstatus	status register for hard fault
 * @param lstatus	status register for local fault
 */
static void traps_v7m_print_message(char *name, struct pt_regs *regs, enum interrupts in, unsigned long addr,
		                            unsigned long hstatus, unsigned long lstatus)
{
	int i;
	printk("\n\n%s: fault at 0x%08lx [pc=0x%08lx, sp=0x%08lx]\n",
			name, addr, instruction_pointer(regs), regs->ARM_sp);
	printk("\n");
}

/*
 * Common routine for high-level exception handlers.
 * @param regs		state of registers when the exception occurs
 * @param in		IPSR, the number of the exception
 */
void traps_v7m_common(struct pt_regs *regs, enum interrupts in)
{
    unsigned long hstatus;
	unsigned long lstatus;
    unsigned long addr = ~0;

    /* read fault status and clear it */
	hstatus = readl(HFSR_ADDR);
	lstatus = readl(CFSR_ADDR);

	writel(hstatus, HFSR_ADDR);
	writel(lstatus, CFSR_ADDR);

    if (user_mode(regs)) {
        /* for user we kill process */
        send_sig(SIGSEGV, current, 0);
    } else {
        /* fault in kernel is not recoverable => panic */
        traps_v7m_print_message("KERNEL", regs, in, addr, hstatus, lstatus);
		show_regs(regs);
		panic(0);
	}
}

/*
 * High-level exception handler for exception 3 (Hard fault).
 * @param regs		state of registers when the exception occurs
 */
asmlinkage void __exception do_hardfault(struct pt_regs *regs)
{
	traps_v7m_common(regs, HARDFAULT);
}

/*
 * High-level exception handler for exception 5 (Bus fault).
 * @param regs		state of registers when the exception occurs
 */
asmlinkage void __exception do_busfault(struct pt_regs *regs)
{
	traps_v7m_common(regs, BUSFAULT);
}

/*
 * High-level exception handler for exception 6 (Usage fault).
 * @param regs		state of registers when the exception occurs
 */
asmlinkage void __exception do_usagefault(struct pt_regs *regs)
{
	traps_v7m_common(regs, USAGEFAULT);
}

