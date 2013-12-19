/*
 *  linux/arch/arm/common/nvic.c
 *
 *  Copyright (C) 2008 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Support for the Nested Vectored Interrupt Controller found on the
 * ARMv7-M CPUs (Cortex-M3)
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/smp.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <mach/nvic.h>

#include <asm/exception.h>

asmlinkage void __exception_irq_entry nvic_handle_irq(irq_hw_number_t hwirq, struct pt_regs *regs)
{
	handle_IRQ(hwirq, regs);
}

/*
 * Routines to acknowledge, disable and enable interrupts
 *
 * Linux assumes that when we're done with an interrupt we need to
 * unmask it, in the same way we need to unmask an interrupt when
 * we first enable it.
 *
 * The NVIC has a separate notion of "end of interrupt" to re-enable
 * an interrupt after handling, in order to support hardware
 * prioritisation.
 *
 * We can make the NVIC behave in the way that Linux expects by making
 * our "acknowledge" routine disable the interrupt, then mark it as
 * complete.
 */
static void nvic_ack_irq(struct irq_data *d)
{
	u32 mask = 1 << ((d->irq) % 32);

	writel_relaxed(mask, IOMEM(NVIC_CLEAR_ENABLE + (d->irq) / 32 * 4));
}

static void nvic_mask_irq(struct irq_data *d)
{
	u32 mask = 1 << ((d->irq) % 32);

	writel_relaxed(mask, IOMEM(NVIC_CLEAR_ENABLE + (d->irq) / 32 * 4));
}

static void nvic_unmask_irq(struct irq_data *d)
{
	u32 mask = 1 << ((d->irq) % 32);

	writel_relaxed(mask, IOMEM(NVIC_SET_ENABLE + (d->irq) / 32 * 4));
}

static struct irq_chip nvic_chip = {
	.name		= "NVIC",
	.irq_ack    = nvic_ack_irq,
	.irq_mask   = nvic_mask_irq,
	.irq_unmask = nvic_unmask_irq,
};

void __init nvic_init(void)
{
	unsigned int max_irq, i;

	max_irq = ((readl_relaxed(IOMEM(NVIC_INTR_CTRL)) & 0x1f) + 1) * 32;

	/* Disable all interrupts */
	for (i = 0; i < max_irq / 32; i++)
		writel_relaxed(~0, IOMEM(NVIC_CLEAR_ENABLE + i *4));

    /* Clear pending interrupts */
    for (i = 0; i < max_irq / 32; i++)
        writel_relaxed(~0, IOMEM(NVIC_CLEAR_PENDING + i *4));

	/*
	 * Set priority on all interrupts.
	 */
	for (i = 0; i < max_irq; i += 4)
		writel_relaxed(0, IOMEM(NVIC_PRIORITY + i));

	/*
	 * Setup the Linux IRQ subsystem.
	 */
	for (i = 0; i < NR_IRQS; i++) {
		irq_set_chip(i, &nvic_chip);
		irq_set_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}
}
