#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/nvic.h>
#include <asm/mach/time.h>
#include <mach/usart.h>

extern void stm32429i_i2c_init(void);
extern void stm32429i_i2s_init(void);
extern void stm32429i_sound_init(void);

/* prototype */
static void __init stm32429i_init(void);
static void __init stm32429i_init_irq(void);
static void __init stm32429i_timer_init(void);

static void __init stm32429i_init(void)
{
    /* gpio for uart1 */
    *((volatile unsigned int *)0x40020000) |= 0x280000;
    *((volatile unsigned int *)0x40020024) |= 0x770;
    *((volatile unsigned int *)0x40023844) |= 0x10;

    /* clocks for uart1 */
    *((volatile unsigned int *)0x40011008) = 0x222;/* 38400 baud */
    *((volatile unsigned int *)0x4001100c) = 0x200c;

    /* init uart */
    stm32429i_uart_init();

    stm32429i_i2c_init();
    stm32429i_i2s_init();
    stm32429i_sound_init();
}

/*
 * STM32 plaform machine description.
 */
MACHINE_START(STM32429I, "STMicro STM32429I-EVAL")
    .init_irq	    = stm32429i_init_irq,
	.init_machine	= stm32429i_init,
    .init_time	    = stm32429i_timer_init,
MACHINE_END

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init stm32429i_init_irq(void)
{
	/*
	 * Initialize NVIC. All interrupts are masked initially.
	 */
	nvic_init();
}

static irqreturn_t tim6_timer_interrupt(int irq, void *dev_id)
{
	timer_tick();
    /* clear interrupt */
    *((volatile unsigned int *)0x40001010) = 0;

	return IRQ_HANDLED;
}

static struct irqaction tim6_timer_irq = {
	.name		= "STM32429I-EVAL Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= tim6_timer_interrupt,
};

static void __init stm32429i_timer_init(void)
{
    /* use timer 6 */
    /* register interrupt handler */
    setup_irq(54, &tim6_timer_irq);
    /* enable clock */
    *((volatile unsigned int *)0x40023840) |= 0x10;
    /* setup prescaler and reload */
    *((volatile unsigned int *)0x40001028) = 1000;
    *((volatile unsigned int *)0x4000102c) = 200;
    /* enable interrupt */
    *((volatile unsigned int *)0x4000100c) |= 0x1;
    /* start it */
    *((volatile unsigned int *)0x40001000) = 0x5;
}

