#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#undef DEBUG
//#define DEBUG

#ifdef DEBUG
#define debug(fmt,args...)	    printk(KERN_INFO fmt, ##args)
#else
#define debug(fmt,args...)
#endif	/* DEBUG */
#define info(fmt,args...)       printk(KERN_INFO fmt, ##args)
#define warning(fmt,args...)    printk(KERN_WARNING fmt, ##args)
#define error(fmt,args...)	    printk(KERN_ERR fmt, ##args)

/* registers */
struct stm32_i2c_regs {
    u32 cr1;            /* 0x00 *//* control register 1 */
    u32 cr2;            /* 0x04 *//* control register 2 */
    u32 oar1;           /* 0x08 *//* own address register 1 */
    u32 oar2;           /* 0x0c *//* own address register 2 */
    u32 dr;             /* 0x10 *//* data register */
    u32 sr1;            /* 0x14 *//* status register 1 */
    u32 sr2;            /* 0x18 *//* status register 2 */
    u32 ccr;            /* 0x1c *//* clock control register */
    u32 trise;          /* 0x20 *//* trise register */
    u32 fltr;           /* 0x24 *//* fltr register */
};
/* cr1 */
#define CR1_PE      (1 << 0)
#define CR1_START   (1 << 8)
#define CR1_STOP    (1 << 9)
#define CR1_ACK     (1 << 10)
#define CR1_POS     (1 << 11)
#define CR1_SWRST   (1 << 15)
/* cr2 */
#define CR2_ITBUFEN (1 << 10)
#define CR2_ITEVTEN (1 << 9)
#define CR2_ITERREN (1 << 8)

/* sr1 */
#define SR1_SB      (1 << 0)
#define SR1_ADDR    (1 << 1)
#define SR1_BTF     (1 << 2)
#define SR1_AF      (1 << 10)

/* sr2 */
#define SR2_TRA     (1 << 2)

/* i2c driver */
struct i2c_stm32 {
	struct platform_device *pdev;
    int bus;
    int irq;
	volatile struct stm32_i2c_regs *regs;
	struct i2c_msg* msg;
	int msg_nb;
	int msg_index;
    volatile int msg_status;
	struct i2c_adapter adap;
	wait_queue_head_t		wait;		/* Wait queue */
};

static irqreturn_t i2c_stm32_irq(int irq, void *d)
{
    struct i2c_stm32 *c = (struct i2c_stm32 *) d;
    unsigned int sr1 = c->regs->sr1;
    unsigned int sr2 = 0;
    int disable_irq = 0;

    /* spurious */
    if (!sr1)
        return IRQ_NONE;

    //debug("sr1=0x%08x\n", sr1);
    /* state machine */
    if (sr1 & SR1_SB) {
        //debug("Got SB\n");
        c->regs->cr1 &= ~CR1_START;
        /* send address + read/write */
        c->regs->dr = (c->msg->addr << 1) | (c->msg->flags&I2C_M_RD?1:0);
        //debug("a : 0x%08x\n", (c->msg->addr << 1) | (c->msg->flags&I2C_M_RD?1:0));
    } else if (sr1 & SR1_ADDR) {
        //debug("Got ADDR\n");
        /* in transmitter then we send first byte */
        /* FIXME : is zero length message possible ? */
        if (!(c->msg->flags&I2C_M_RD)) {
            /* this clear interrupt */
            sr2 = c->regs->sr2;
            c->regs->dr = c->msg->buf[c->msg_index++];
        } else {
            /* in case we reveive more than one byte we need to ack reception */
            if (c->msg->len == 1) {
                c->regs->cr1 &= ~CR1_ACK;
                c->regs->cr1 &= ~CR1_POS;
                if ((c->msg_nb) == 1) {
                    c->regs->cr1 |= CR1_STOP;
                } else {
                    c->regs->cr1 |= CR1_START;
                }
            } else if (c->msg->len == 2) {
                c->regs->cr1 &= ~CR1_ACK;
                c->regs->cr1 |= CR1_POS;
            } else {
                c->regs->cr1 |= CR1_ACK;
                c->regs->cr1 &= ~CR1_POS;
            }
            /* this clear interrupt */
            sr2 = c->regs->sr2;
        }
    } else if (sr1 & SR1_BTF) {
        //debug("Got BTF\n");
        /* in transmit mode, indicate one byte has been transmitted successfully */
        /* in receive mode, indicate one byte has been received successfully */
        if (c->msg->flags&I2C_M_RD) {
            /* receive mode */
            if (c->msg_index + 3 == c->msg->len) {
                c->regs->cr1 &= ~CR1_ACK;
                c->regs->cr1 |= CR1_POS;
            }
            if (c->msg->len == 1) {
                c->msg->buf[c->msg_index++] = c->regs->dr;
                if (--(c->msg_nb) == 0) {
                    /* no more message */
                    c->msg_status = 0;
				    disable_irq = 1;
                } else {
                    /* start next one with repeated start */
                    c->msg++;
                    c->msg_index = 0;
                }
            } else if (c->msg_index + 2 == c->msg->len) {
                if ((c->msg_nb) == 1) {
                    c->regs->cr1 |= CR1_STOP;
                } else {
                    c->regs->cr1 |= CR1_START;
                }
                c->msg->buf[c->msg_index++] = c->regs->dr;
                c->msg->buf[c->msg_index++] = c->regs->dr;
                if (--(c->msg_nb) == 0) {
                    /* no more message */
                    c->msg_status = 0;
				    disable_irq = 1;
                } else {
                    /* start next one with repeated start */
                    c->msg++;
                    c->msg_index = 0;
                }
            } else {
                c->msg->buf[c->msg_index++] = c->regs->dr;
            }
        } else {
            /* transmit mode */
            if (c->msg_index == c->msg->len) {
                /* we finish to transfert one message */
                if (--(c->msg_nb) == 0) {
                    /* no more message */
                    c->msg_status = 0;
				    disable_irq = 1;
                    c->regs->cr1 |= CR1_STOP;
                } else {
                    /* start next one with repeated start */
                    c->msg++;
                    c->msg_index = 0;
                    c->regs->cr1 |= CR1_START;
                }
            } else {
                c->regs->dr = c->msg->buf[c->msg_index++];
            }
        }
    } else if (sr1 & SR1_AF) {
        /* nobody acknoledge */
        c->regs->sr1 &= ~SR1_AF;
        c->msg_status = -ENODEV;
        disable_irq = 1;
    } else {
        /* we are in error */
        debug("Got an error, we are stuck 0x%08x\n", sr1);
        c->msg_status = -EIO;
		disable_irq = 1;
    }

    /* is transfert finish */
    if (disable_irq) {
		disable_irq_nosync(c->irq);
	}
    if (c->msg_status != -EBUSY) {
		wake_up(&c->wait);
    }

    return IRQ_HANDLED;
}

static int i2c_stm32_hw_init(struct i2c_stm32 *c)
{
    int freq = 11;

    /* reset+clock */
    if (c->bus == 0) {
        /* toggle reset */
        *((volatile int *)0x40023820) |= (1 << 21);
        *((volatile int *)0x40023820) &= ~(1 << 21);
        /* enable clock */
        *((volatile int *)0x40023840) |= (1 << 21);
    }
    /* soft reset */
    c->regs->cr1 |= CR1_SWRST;
    c->regs->cr1 &= ~CR1_SWRST;
    /* setup clock */
    c->regs->cr2 = freq; /* we run 10.5 Mhz */
    c->regs->trise = 12;//freq+1;
    c->regs->ccr = 5*11;//5*(freq);/* can be increase to redu ce speed */
    c->regs->fltr = 12;
    /* enable interrupts */
    c->regs->cr2 |= CR2_ITEVTEN | CR2_ITERREN;
    /* enable controller */
    c->regs->cr1 |= CR1_PE;


    debug("ccr=0x%08x\n", c->regs->ccr);

    return 0;
}

static int i2c_stm32_transfer(struct i2c_adapter *a, struct i2c_msg *m, int n)
{
    struct i2c_stm32 *c = (struct i2c_stm32 *) a->algo_data;
    int ret = 0;
    int i;

    debug("IN : i2c_stm32_transfer : %d messages\n", n);
    for(i=0;i<n;i++) {
        debug(" - msg[%d].addr = 0x%08x\n", i, m[i].addr);
        debug(" - msg[%d].flags = 0x%08x\n", i, m[i].flags);
        debug(" - msg[%d].len = 0x%08x\n", i, m[i].len);
        if (!(m[i].flags&I2C_M_RD)) {
            int j;
            for(j=0;j<m[i].len;j++) {
                debug("  - data[%d] = 0x%02x\n", j, m[i].buf[j]);
            }
        }
    }
    c->msg = &m[0]; /* can we do that ? check message is not on stack */
    c->msg_index = 0;
	c->msg_nb = n;
	c->msg_status = -EBUSY;

    /* start transfert */
    c->regs->cr1 &= ~CR1_STOP;
    c->regs->cr1 |= CR1_START;

    /* enable irq */
    enable_irq(c->irq);

    /* wait for end of transfert */
    if (wait_event_timeout(c->wait, c->msg_status != -EBUSY, 5*HZ) == 0) {
        disable_irq_nosync(c->irq);
        ret = -ETIMEDOUT;
    } else {
        ret = c->msg_status;
        if (!ret)
            ret = n;
    }

    debug("OUT : i2c_stm32_transfer : %d messages\n", n);
    for(i=0;i<n;i++) {
        debug(" - msg[%d].addr = 0x%08x\n", i, m[i].addr);
        debug(" - msg[%d].flags = 0x%08x\n", i, m[i].flags);
        debug(" - msg[%d].len = 0x%08x\n", i, m[i].len);
        if ((m[i].flags&I2C_M_RD)) {
            int j;
            for(j=0;j<m[i].len;j++) {
                debug("  - data[%d] = 0x%02x\n", j, m[i].buf[j]);
            }
        }
    }

    debug("i2c_stm32_transfer finish with %d\n", ret);

    return ret;
}

static unsigned int i2c_stm32_functionality(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_stm32_algorithm = {
	.functionality	= i2c_stm32_functionality,
	.master_xfer	= i2c_stm32_transfer,
};

static int i2c_stm32_probe(struct platform_device *dev)
{
    struct i2c_stm32 *c = NULL;
    int bus = 0;
    int irq = 31;
    void *regs = (void *) 0x40005400;
    int ret;

    /* allocate driver structure */
    c = kzalloc(sizeof(struct i2c_stm32), GFP_KERNEL);
    if (!c) {
		error("unable to allocate memory for i2c_stm32_probe()\n");
		ret = -ENOMEM;
		goto error_allocate_driver;
	}
    c->pdev = dev;
    c->bus = bus;
    c->regs = (volatile struct stm32_i2c_regs *) regs;

    /* register interrupt handlers */
    ret = request_irq(irq, i2c_stm32_irq, 0, dev_name(&dev->dev), c);
    if (ret) {
        error("Error requesting irq %d : %d\n", irq, ret);
        goto error_request_event_irq;
    }
    disable_irq_nosync(irq);
	c->irq = irq;

    ret = request_irq(irq + 1, i2c_stm32_irq, 0, dev_name(&dev->dev), c);
    if (ret) {
        error("Error requesting irq %d : %d\n", irq, ret);
        goto error_request_error_irq;
    }

    platform_set_drvdata(dev, c);
    /* init i2c adapter structure */
    c->adap.owner = THIS_MODULE;
    c->adap.nr = 0;
    snprintf(c->adap.name, sizeof(c->adap.name), "i2c_stm32.%u", bus);
    c->adap.algo = &i2c_stm32_algorithm;
    c->adap.algo_data = c;
	c->adap.dev.parent = &dev->dev;

    /* register i2c adapter */
    ret = i2c_add_numbered_adapter(&c->adap);
    if (ret) {
        error("Failed to add i2c adapter : %d\n", ret);
        goto error_adapter;
    }
    init_waitqueue_head(&c->wait);

    /* init hardware */
    ret = i2c_stm32_hw_init(c);
    if (ret) {
        error("Failed to init i2c hardware : %d\n", ret);
        goto error_hw;
    }

    ret = 0;
error_hw:
    if (ret)
        i2c_del_adapter(&c->adap);
error_adapter:
    if (ret)
        free_irq(irq + 1, c);
error_request_error_irq:
    if (ret)
        free_irq(irq, c);
error_request_event_irq:
    if (ret)
        kfree(c);
error_allocate_driver:
    return ret;
}

/* driver init */
static struct platform_driver i2c_stm32_drv = {
	.probe	= i2c_stm32_probe,
	.driver = {
		.name = "i2c_stm32",
		.owner = THIS_MODULE,
	},
};

static int __init i2c_stm32_module_init(void)
{
	int ret;
	
	ret = platform_driver_register(&i2c_stm32_drv);

	debug("drv=%s,ret=%d\n", i2c_stm32_drv.driver.name, ret);
	return ret;
}
module_init(i2c_stm32_module_init);

/* register i2c1 device */
/* FIXME :
    - move it
    - add ressource
 */
static struct i2c_board_info stm32429i_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("wm8994", 0x1a),
    },
};

static struct platform_device i2c_stm32429i_dev1 = {
	.name           = "i2c_stm32",
	.id             = 0,
};

void __init stm32429i_i2c_init(void)
{
    i2c_register_board_info(0, stm32429i_i2c_board_info, ARRAY_SIZE(stm32429i_i2c_board_info));
    platform_device_register(&i2c_stm32429i_dev1);
}

