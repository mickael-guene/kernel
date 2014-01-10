#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <asm/io.h>

#include <mach/usart.h>

#define STM_USART_PORT  "STM USART Port"

#define STM_USART_NAME  "ttyS"

#define STM_USART_NB    6

#define PREAD(port, reg) readl((port)->membase + (reg))
#define PWRITE(port, reg, data) writel((data), (port)->membase + (reg))

static const unsigned int port2Irq[]={37,38,39,52,53,71};
static const resource_size_t port2BaseAddress[]={0x40011000, 0x40004400, 0x40004800,
                                                 0x40004c00, 0x40005000, 0x40011400};
    
static struct uart_port ports[STM_USART_NB];

/* helpers */
static void stm_console_putchar(struct uart_port *port, int ch)
{
    while ((PREAD(port, USART_SR) & USART_SR_TXE) == 0) ;
    PWRITE(port, USART_DR, ch);
}

/* irq handler */
static void stm_port_stop_tx(struct uart_port *port);
static irqreturn_t stm_usart_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
    unsigned int sr = PREAD(port, USART_SR);
    struct circ_buf *xmit = &port->state->xmit;

    /* receipt buffer */
    if (sr & USART_SR_ORE) {
        /* we lost some input data ... avoid driver being stuck */
        /* a read to USART_DR is need to clear interrupt */
        struct tty_port *tty = &port->state->port;
        unsigned int ch = PREAD(port, USART_DR);
    } else if (sr & USART_SR_RXNE) {
        struct tty_port *tty = &port->state->port;
        unsigned int ch = PREAD(port, USART_DR);
       
        tty_insert_flip_char(tty, ch, TTY_NORMAL);
        tty_flip_buffer_push(tty);
    }

    /* transmit buffer */
    if (sr & USART_SR_TXE) {
        /* test x_char */
        if (port->x_char) {
		    stm_console_putchar(port, port->x_char);
		    port->x_char = 0;
		    port->icount.tx++;
		    goto out;
        }
        /* if buffer is empty or stop then disable interrupt */
        if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		    stm_port_stop_tx(port);
		    goto out;
        }
        /* transmit one character */
        stm_console_putchar(port, xmit->buf[xmit->tail]);
	    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	    port->icount.tx++;
    }

out:
	return IRQ_HANDLED;
}

/* console */
static void stm_console_write(struct console *co, const char *s, unsigned int count)
{
    struct uart_port *port = &ports[co->index];
    unsigned int cr1 = PREAD(port, USART_CR1);
    
    PWRITE(port, USART_CR1, USART_CR1_TE | USART_CR1_UE);
    uart_console_write(ports, s, count, stm_console_putchar);
    
    PWRITE(port, USART_CR1, cr1);
}

static int stm_port_startup(struct uart_port *port);
static int __init stm_console_setup(struct console *co, char *options)
{
    struct uart_port *port;
	int rv;
	/*
	 * Set of console default values. They will be used only if
	 * no options are given, may be changed at will.
	 */
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= STM_USART_NB) {
		rv = -EINVAL;
		goto out;
	}
	port = &ports[co->index];

	/* If options are present parse them and overide defaults */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	/* Set options to console and port termios */
	rv = uart_set_options(port, co, baud, parity, bits, flow);
out:
	return rv;
}


/* uart port ops */
static u32 stm_port_tx_empty(struct uart_port *port)
{
	return (PREAD(port, USART_SR) & USART_SR_TXE)?TIOCSER_TEMT:0;
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
    unsigned int cr1;

    cr1 = PREAD(port, USART_CR1);
    cr1 &= ~USART_CR1_TXEIE;
    PWRITE(port, USART_CR1, cr1);
}

static void stm_port_start_tx(struct uart_port *port)
{
    unsigned int cr1;

    cr1 = PREAD(port, USART_CR1);
    cr1 |= USART_CR1_TXEIE;
    PWRITE(port, USART_CR1, cr1);
}

static void stm_port_stop_rx(struct uart_port *port)
{
    unsigned int cr1;

    cr1 = PREAD(port, USART_CR1);
    cr1 &= ~USART_CR1_RE;
    PWRITE(port, USART_CR1, cr1);
}

static void stm_set_baud_rate(struct uart_port *port, int baudrate)
{
    //FIXME : for now set 115200
    //PWRITE(port, USART_BRR, 0x305);
    PWRITE(port, USART_BRR, 0x0c1);
}

static int stm_port_startup(struct uart_port *port)
{
    int rv;

    /* request irq */
    rv = request_irq(port2Irq[port->line], stm_usart_isr, 0, STM_USART_PORT, port);
    if (rv) {
		printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n", __func__, port2Irq[port->line], rv);
		goto out;
	}
    
    /* configure USART */
     /* 1 stop bit */
    PWRITE(port, USART_CR2, 0);
     /* set baudrate */
    stm_set_baud_rate(port, 115200);
     /* Enable tx and rx + rxneir and enable it*/
    PWRITE(port, USART_CR1, USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE);
    
    rv = 0;

out:
    return rv;
}

static void stm_port_shutdown(struct uart_port *port)
{
    unsigned int cr1;

    cr1 = PREAD(port, USART_CR1);
    cr1 &= ~(USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE | USART_CR1_TXEIE);
    PWRITE(port, USART_CR1, cr1);
    free_irq(port2Irq[port->line], port);
}

static const char *stm_port_type(struct uart_port *port)
{
	return STM_USART_PORT;
}

static void stm_port_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
    //FIXME
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

static struct uart_ops stm_uart_ops = {
	.tx_empty	    = stm_port_tx_empty,
	.set_mctrl	    = stm_port_set_mctrl,
	.get_mctrl	    = stm_port_get_mctrl,
	.stop_tx	    = stm_port_stop_tx,
	.start_tx	    = stm_port_start_tx,
	.stop_rx	    = stm_port_stop_rx,
	.enable_ms	    = stm_port_enable_ms,
	.break_ctl	    = stm_port_break_ctl,
	.startup	    = stm_port_startup,
	.shutdown	    = stm_port_shutdown,
	.set_termios	= stm_port_set_termios,
	.type		    = stm_port_type,
	.release_port	= stm_port_release_port,
	.request_port	= stm_port_request_port,
	.config_port	= stm_port_config_port,
	.verify_port	= stm_port_verify_port
};

static struct uart_driver	stm32429i_uart_driver;
static struct console		stm32429i_console = {
	.name	= STM_USART_NAME,
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
	.owner		    = THIS_MODULE,
	.driver_name	= "stm32429i serial driver",
	.dev_name	    = STM_USART_NAME,
	.major		    = TTY_MAJOR,
	.minor		    = 64,
	.nr		        = STM_USART_NB,
	.cons		    = &stm32429i_console,
};

void __init stm32429i_uart_init()
{
    int rv;
    int i;

    rv = uart_register_driver(&stm32429i_uart_driver);
	if (rv) {
		printk(KERN_ERR "%s: uart_register_driver failed (%d)\n", __func__, rv);
        return;
	}

    for(i=0;i<STM_USART_NB;i++) {
        struct uart_port *port = &ports[i];

        port->iotype  = SERIAL_IO_MEM,
	    port->irq     = port2Irq[i];
	    port->flags   = UPF_BOOT_AUTOCONF;
	    port->line    = i;
        port->type    = 91;
	    port->ops     = &stm_uart_ops;
        port->membase = IOMEM(port2BaseAddress[i]);

        rv = uart_add_one_port(&stm32429i_uart_driver, port);
        if (rv)
            printk(KERN_ERR "%s: uart_add_one_port %d (%d)\n", __func__, i, rv);
    }
}

