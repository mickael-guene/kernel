#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/tty.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/nvic.h>
#include <asm/mach/time.h>

/* prototype */
static void __init stm32429i_init(void);
static void __init stm32429i_init_irq(void);
static void __init stm32429i_timer_init(void);

#define STM32_USART_NAME	"ttyS"
#define STM32_USART_DRV_NAME	"stm32serial"
#define STM32_USART_PORT	"STM32 USART Port"

static struct uart_port ports[1];

static void stm_console_putchar(struct uart_port *port, int ch)
{
    while((*((volatile unsigned int *)0x40011000) & 0x40)  == 0) ;
    *((volatile unsigned int *)0x40011004) = ch;
}

static void stm_console_write(struct console *co, const char *s, unsigned int count)
{
    uart_console_write(&ports[0], s, count, stm_console_putchar);
}

static int stm_port_startup(struct uart_port *port);
static int __init stm_console_setup(struct console *co, char *options)
{
    stm_port_startup(&ports[0]);
    return 0;
}

static struct uart_driver	stm32429i_uart_driver;
static struct console		stm32429i_console = {
	.name	= STM32_USART_NAME,
	.device	= uart_console_device,
	.write	= stm_console_write,
	.setup	= stm_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &stm32429i_uart_driver,
};

/*
 * UART driver instance
 */
static struct uart_driver	stm32429i_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= STM32_USART_DRV_NAME,
	.dev_name	= STM32_USART_NAME,
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		    = 1,
	.cons		= &stm32429i_console,
};

static u32 stm_port_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static void stm_port_set_mctrl(struct uart_port *port, u32 mctrl)
{
}

static u32 stm_port_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void stm_port_stop_tx(struct uart_port *port)
{
}

static void stm_port_start_tx(struct uart_port *port)
{
    struct circ_buf *xmit = &port->state->xmit;

    if (port->x_char) {
        stm_console_putchar(port, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
	}
    while(!uart_circ_empty(xmit)) {
        stm_console_putchar(port, xmit->buf[xmit->tail]);
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	    port->icount.tx++;
    }
}

static void stm_port_stop_rx(struct uart_port *port)
{
}

static void stm_set_baud_rate(struct uart_port *port, int baudrate)
{
}

static int stm_port_startup(struct uart_port *port)
{
    *((volatile unsigned int *)0x40020000) |= 0x280000;
    *((volatile unsigned int *)0x40020024) |= 0x770;
    *((volatile unsigned int *)0x40023844) |= 0x10;
    *((volatile unsigned int *)0x40011008) = 0x305;
    *((volatile unsigned int *)0x4001100c) = 0x200c;

    return 0;
}

static void stm_port_shutdown(struct uart_port *port)
{
}

static const char *stm_port_type(struct uart_port *port)
{
	return STM32_USART_PORT;
}

static void stm_port_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
}

static int stm_port_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/*
	 * We don't want the core code to modify any port params
	 */

	return -EINVAL;
}

static void stm_port_release_port(struct uart_port *port)
{
}

static int stm_port_request_port(struct uart_port *port)
{
	return 0;
}

static void stm_port_config_port(struct uart_port *port, int flags)
{
	if (!stm_port_request_port(port))
		port->type = 91/*PORT_STM32USART*/;
}

static void stm_port_enable_ms(struct uart_port *port)
{
}

static void stm_port_break_ctl(struct uart_port *port, int ctl)
{
}

static struct uart_ops stm32_uart_ops = {
	.tx_empty	= stm_port_tx_empty,
	.set_mctrl	= stm_port_set_mctrl,
	.get_mctrl	= stm_port_get_mctrl,
	.stop_tx	= stm_port_stop_tx,
	.start_tx	= stm_port_start_tx,
	.stop_rx	= stm_port_stop_rx,
	.enable_ms	= stm_port_enable_ms,
	.break_ctl	= stm_port_break_ctl,
	.startup	= stm_port_startup,
	.shutdown	= stm_port_shutdown,
	.set_termios	= stm_port_set_termios,
	.type		= stm_port_type,
	.release_port	= stm_port_release_port,
	.request_port	= stm_port_request_port,
	.config_port	= stm_port_config_port,
	.verify_port	= stm_port_verify_port
};


static void __init stm32429i_init(void)
{
    int rv;
    struct uart_port *port = &ports[0];

    rv = uart_register_driver(&stm32429i_uart_driver);
	if (rv) {
		printk(KERN_ERR "%s: uart_register_driver failed (%d)\n", __func__, rv);
        return;
	}
    
    port->iotype  = SERIAL_IO_MEM,
	port->irq     = 37;
	port->flags   = UPF_BOOT_AUTOCONF;
	port->line    = 0;
    port->type    = 91;
	port->ops     = &stm32_uart_ops;

    rv = uart_add_one_port(&stm32429i_uart_driver, port);

    register_console(&stm32429i_console);
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
    *((volatile unsigned int *)0x4000102c) = 445;
    /* enable interrupt */
    *((volatile unsigned int *)0x4000100c) |= 0x1;
    /* start it */
    *((volatile unsigned int *)0x40001000) = 0x5;
}

#if 0
/* console stuff */
static void test_console_write(struct console *co, const char *s, unsigned count)
{
    int i;

    for(i=0;i<count;i++) {
        if (*s == '\n') {
            *((volatile unsigned int *)0x40011004) = '\r';
            while((*((volatile unsigned int *)0x40011000) & 0x40)  == 0) ;
        }
        *((volatile unsigned int *)0x40011004) |= *s++;
        while((*((volatile unsigned int *)0x40011000) & 0x40)  == 0) ;
    }
}

static int test_console_setup(struct console *co, char *options)
{
    return 0;
}

static struct tty_driver *test_device(struct console *co, int *index)
{
    return NULL;
}

static struct console sercons = {
       .name     = "ttyS",
       .write    = test_console_write,
       .setup    = test_console_setup,
       .flags    = CON_PRINTBUFFER | CON_ENABLED,
       .device   = test_device,
};

static int __init stm32429i_console_init(void)
{
     /* init a console */
    *((volatile unsigned int *)0x40020000) |= 0x280000;
    *((volatile unsigned int *)0x40020024) |= 0x770;
    *((volatile unsigned int *)0x40023844) |= 0x10;
    *((volatile unsigned int *)0x40011008) = 0x305;
    *((volatile unsigned int *)0x4001100c) = 0x200c;
    register_console(&sercons);
    
    return 0;
}
#if 1
console_initcall(stm32429i_console_init);
#else
static void __init stm32429i_init_early()
{
    stm32429i_console_init();
}
#endif
#endif
