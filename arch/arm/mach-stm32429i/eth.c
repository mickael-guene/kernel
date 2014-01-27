#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <asm/setup.h>
#include <linux/genalloc.h>

#include <mach/eth.h>

/*
 * Define this to enable debugging msgs
 */
#undef DEBUG
#define DEBUG

#ifdef DEBUG
#define debug(fmt,args...)	    printk(KERN_DEBUG fmt, ##args)
#else
#define debug(fmt,args...)
#endif	/* DEBUG */
#define info(fmt,args...)       printk(KERN_INFO fmt, ##args)
#define warning(fmt,args...)    printk(KERN_WARNING fmt, ##args)
#define error(fmt,args...)	    printk(KERN_ERR fmt, ##args)

/* dma parameters */
#ifdef MODULE_PARAM_PREFIX
#undef MODULE_PARAM_PREFIX
#endif
#define MODULE_PARAM_PREFIX "stm32429i."

static unsigned int dma_addr = 0x20000000;
module_param(dma_addr, uint, S_IRUGO);
static unsigned int dma_size = 128*1024;
module_param(dma_size, uint, S_IRUGO);
static struct gen_pool *dma_pool;

/* registers description */
struct stm32_mac_regs {
	u32	maccr;          /* 0x0000 *//* Ethernet MAC configuration register */
	u32	macffr;         /* 0x0004 *//* Ethernet MAC frame filter register */
	u32	machthr;        /* 0x0008 *//* Ethernet MAC hash table high register */
	u32	machtlr;        /* 0x000c *//* Ethernet MAC hash table low register */
	u32	macmiiar;       /* 0x0010 *//* Ethernet MAC MII address register */
	u32	macmiidr;       /* 0x0014 *//* Ethernet MAC MII data register */
	u32	macfcr;         /* 0x0018 *//* Ethernet MAC flow control register */
	u32	macvlantr;      /* 0x001c *//* Ethernet MAC VLAN tag register */
	u32	reserved0[(0x0028 - 0x001c) / sizeof(u32) - 1];
	u32	macrwuffr;      /* 0x0028 *//* Ethernet MAC remote wakeup frame filter register */
	u32	macpmtcsr;      /* 0x002c *//* Ethernet MAC PMT control and status register */
	u32	reserved1;
	u32	macdbgr;        /* 0x0034 *//* Ethernet MAC debug register */
	u32	macsr;          /* 0x0038 *//* Ethernet MAC interrupt status register */
	u32	macimr;         /* 0x003c *//* Ethernet MAC interrupt mask register */
	u32	maca0hr;        /* 0x0040 *//* Ethernet MAC address 0 high register */
	u32	maca0lr;        /* 0x0044 *//* Ethernet MAC address 0 low register */
	u32	maca1hr;        /* 0x0048 *//* Ethernet MAC address 1 high register */
	u32	maca1lr;        /* 0x004c *//* Ethernet MAC address 1 low register */
	u32	maca2hr;        /* 0x0050 *//* Ethernet MAC address 2 high register */
	u32	maca2lr;        /* 0x0054 *//* Ethernet MAC address 2 low register */
	u32	maca3hr;        /* 0x0058 *//* Ethernet MAC address 3 high register */
	u32	maca3lr;        /* 0x005c *//* Ethernet MAC address 3 low register */
	u32	reserved2[(0x0100 - 0x005c) / sizeof(u32) - 1];
	u32	mmccr;          /* 0x0100 *//*Ethernet MMC control register */
	u32	mmcrir;         /* 0x0104 *//*Ethernet MMC receive interrupt register */
	u32	mmctir;         /* 0x0108 *//*Ethernet MMC transmit interrupt register */
	u32	mmcrimr;        /* 0x010c *//*Ethernet MMC receive interrupt mask register */
	u32	mmctimr;        /* 0x0110 *//*Ethernet MMC transmit interrupt mask register */
	u32	reserved3[(0x014c - 0x0110) / sizeof(u32) - 1];
	u32	mmctgfsccr;     /* 0x014c *//*Ethernet MMC transmitted good frames after a single collision counter register */
	u32	mmctgfmsccr;    /* 0x0150 *//*Ethernet MMC transmitted good frames after more than a single collisioncounter register */
	u32	reserved4[(0x0168 - 0x0150) / sizeof(u32) - 1];
	u32	mmctgfcr;       /* 0x0168 *//* Ethernet MMC transmitted good frames counter register */
	u32	reserved5[(0x0194 - 0x0168) / sizeof(u32) - 1];
	u32	mmcrfcecr;      /* 0x0194 *//* Ethernet MMC received frames with CRC error counter register */
	u32	mmcrfaecr;      /* 0x0198 *//* Ethernet MMC received frames with alignment error counter register */
	u32	reserved6[(0x01c4 - 0x0198) / sizeof(u32) - 1];
	u32	mmcrgufcr;      /* 0x01c4 *//* MMC received good unicast frames counter register */
	u32	reserved7[(0x0700 - 0x01c4) / sizeof(u32) - 1];
	u32	ptptscr;        /* 0x0700 *//* Ethernet PTP time stamp control register */
	u32	ptpssir;        /* 0x0704 *//* Ethernet PTP subsecond increment register */
	u32	ptptshr;        /* 0x0708 *//* Ethernet PTP time stamp high register */
	u32	ptptslr;        /* 0x070c *//* Ethernet PTP time stamp low register */
	u32	ptptshur;       /* 0x0710 *//* Ethernet PTP time stamp high update register */
	u32	ptptslur;       /* 0x0714 *//* Ethernet PTP time stamp low update register */
	u32	ptptsar;        /* 0x0718 *//* Ethernet PTP time stamp addend register */
	u32	ptptthr;        /* 0x071c *//* Ethernet PTP target time high register */
	u32	ptpttlr;        /* 0x0720 *//* Ethernet PTP target time low register */
	u32	reserved8[(0x0728 - 0x0720) / sizeof(u32) - 1];
	u32	ptptssr;        /* 0x0728 *//* Ethernet PTP time stamp status register */
	u32	ptpppscr;       /* 0x072c *//* Ethernet PTP PPS control register */
	u32	reserved9[(0x1000 - 0x072c) / sizeof(u32) - 1];
	u32	dmabmr;         /* 0x1000 *//* Ethernet DMA bus mode register */
	u32	dmatpdr;        /* 0x1004 *//* Ethernet DMA transmit poll demand register */
	u32	dmarpdr;        /* 0x1008 *//* Ethernet DMA receive poll demand register */
	u32	dmardlar;       /* 0x100c *//* Ethernet DMA receive descriptor list address register */
	u32	dmatdlar;       /* 0x1010 *//* Ethernet DMA transmit descriptor list address register */
	u32	dmasr;          /* 0x1014 *//* Ethernet DMA status register */
	u32	dmaomr;         /* 0x1018 *//* Ethernet DMA operation mode register */
	u32	dmaier;         /* 0x101c *//* Ethernet DMA interrupt enable register */
	u32	dmamfbocr;      /* 0x1020 *//* Ethernet DMA missed frame and buffer overflow counter register */
	u32	dmarswtr;       /* 0x1024 *//* Ethernet DMA receive status watchdog timer register */
	u32	reserved10[(0x1048 - 0x1024) / sizeof(u32) - 1];
	u32	dmachtdr;       /* 0x1048 *//* Ethernet DMA current host transmit descriptor register */
	u32	dmachrdr;       /* 0x104c *//* Ethernet DMA current host receive descriptor register */
	u32	dmachtbar;      /* 0x1050 *//* Ethernet DMA current host transmit buffer address register */
	u32	dmachrbar;      /* 0x1054 *//* Ethernet DMA current host receive buffer address register */
};

/* dma transmit buffer descriptor bits */
#define STM32_DMA_TBD_DMA_OWN   (1 << 31)	/* buffer ownership */
#define STM32_DMA_TBD_DMA_IC    (1 << 30)	/* generate interrupt on completion */
#define STM32_DMA_TBD_LS        (1 << 29)	/* last segment */
#define STM32_DMA_TBD_FS        (1 << 28)	/* first segment */
#define STM32_DMA_TBD_TCH       (1 << 20)	/* chain mode */
#define STM32_DMA_TBD_ES        (1 << 15)	/* Error summary */
#define STM32_DMA_TBD_NC        (1 << 10)	/* No carrier */
#define STM32_DMA_TBD_EC        (1 << 8)	/* Excessive collision*/
#define STM32_DMA_TBD_CC_BIT    3		    /* Collistion count */
#define STM32_DMA_TBD_CC_MSK    0xF
#define STM32_DMA_TBD_UF        (1 << 1)	/* Underflow */

/* DMA receive buffer descriptor bits */
#define STM32_DMA_RBD_DMA_OWN   (1 << 31)	/* buffer ownership */
#define STM32_DMA_RBD_FL_BIT    16		    /* Frame length	      */
#define STM32_DMA_RBD_FL_MSK    0x3FFF
#define STM32_DMA_RBD_ES        (1 << 15)	/* Error summary */
#define STM32_DMA_RBD_LE        (1 << 12)	/* Length error */
#define STM32_DMA_RBD_OE        (1 << 11)	/* Overrun error */
#define STM32_DMA_RBD_FS        (1 << 9)	/* First descriptor */
#define STM32_DMA_RBD_LS        (1 << 8)	/* Last descriptor */
#define STM32_DMA_RBD_CE        (1 << 1)	/* CRC error */

#define STM32_DMA_RBD_RCH       (1 << 14)	/* chain mode */

/* we use chain mode descriptor */
struct stm32_eth_dma_bd {
	u32 stat; /* status and control */
	u32 ctrl; /* buffer length */
	dma_addr_t buf;	/* physical buffer address */
	dma_addr_t next; /* pointer to next descriptor */
};

/* Private ethernet device data */
struct stm32_eth_priv {
    struct net_device *dev;
    struct napi_struct napi;
    spinlock_t rx_lock;
	spinlock_t tx_lock;
    struct net_device_stats stat;
    /* mii */
    spinlock_t lock;
    struct mii_bus *mii_bus;
    struct phy_device *phy_dev;
    int link;
	int speed;
	int duplex;
    /* registers */
    volatile struct stm32_mac_regs *regs;
    /* interrupt */
    int irq;
    /* dma stuff */
     /* descriptors */
    struct stm32_eth_dma_bd *tx_bd;
    struct stm32_eth_dma_bd *rx_bd;
    void **tx_buffer;
    void **rx_buffer;
    dma_addr_t tx_bd_dma_phys_addr;
    dma_addr_t rx_bd_dma_phys_addr;
     /* indexes */
      /* tx */
    u32 tx_next_free_buffer_index;
    u32 tx_next_release_buffer_index;
    u32 tx_buffer_nb_in_queue;
    u32 tx_is_blocked;
     /* rx */
    u32 rx_index;
    /* settings */
    u32 frame_max_size;
	u32 rx_buf_num;
	u32 tx_buf_num;
};

/* added in kernel 3.13 only */
static void *gen_pool_dma_alloc(struct gen_pool *pool, size_t size, dma_addr_t *dma)
{
        unsigned long vaddr;

        if (!pool)
                return NULL;

        vaddr = gen_pool_alloc(pool, size);
        if (!vaddr)
                return NULL;

        *dma = gen_pool_virt_to_phys(pool, vaddr);

        return (void *)vaddr;
}

/* dma memory pool */
static int init_dma_mem_pool(void)
{
    void __iomem *virt;
    int rv;

    info("stm32429i ethernet dma address = 0x%08x\n", dma_addr);
    info("stm32429i ethernet dma size = %d bytes\n", dma_size);
    /* create dma pool */
    dma_pool = gen_pool_create(ilog2(16), -1);
    if (!dma_pool) {
        error("Unable to create ethernet dma pool memory\n");
        rv = -ENOMEM;
        goto create_pool_error;
    }
    /* add memory to it */
    virt = ioremap((phys_addr_t) dma_addr, dma_size);
    rv = gen_pool_add_virt(dma_pool, (unsigned long) virt, (phys_addr_t) dma_addr, dma_size, -1);
    if (rv < 0) {
        error("Unable to add memory to dma pool\n");
        rv = -ENOMEM;
        goto add_chunk_error;
    }

    rv = 0;

add_chunk_error:
    if (rv) {
        gen_pool_destroy(dma_pool);
        iounmap(virt);
    }
create_pool_error:

    return rv;
}

/* real job functions */
static void stm32_eth_stop(struct net_device *dev)
{
    struct stm32_eth_priv *stm = netdev_priv(dev);
    
    debug("stm32_eth_stop\n");
    /* stop dma */
    stm->regs->dmaomr &= ~(STM32_MAC_DMAOMR_SR | STM32_MAC_DMAOMR_ST);
}

static int stm32_eth_start(struct net_device *dev)
{
    struct stm32_eth_priv *stm = netdev_priv(dev);
    int rv;
    int i;

    debug("stm32_eth_start\n");
    /* be sure we have a clean state */
    stm32_eth_stop(dev);

    /* be sure reset is finish */
    if (stm->regs->dmabmr & STM32_MAC_DMABMR_SR) {
		rv = -EBUSY;
		goto out;
    }
    /* setup dma mode */
    stm->regs->dmabmr = (32 << STM32_MAC_DMABMR_PBL_BIT) | /* tx dma max burst length */
			            (32 << STM32_MAC_DMABMR_RDP_BIT) | /* rx dma max burst length */
			            (STM32_MAC_DMABMR_RTPR_2_1 << STM32_MAC_DMABMR_RTPR_BIT) | /* rx:tx ratio is 2:1 */
			            STM32_MAC_DMABMR_FB | /* fix burst */
                        STM32_MAC_DMABMR_USP | /* separate burst length */
			            STM32_MAC_DMABMR_AAB; /* aligned */
    /* setup mac address */
    stm->regs->maca0hr = (dev->dev_addr[5] << 8) | (dev->dev_addr[4] << 0);
	stm->regs->maca0lr = (dev->dev_addr[3] << 24) | (dev->dev_addr[2] << 16) |(dev->dev_addr[1] << 8) | (dev->dev_addr[0] << 0);
    /* setup buffer descriptors */
    stm->regs->dmardlar = stm->rx_bd_dma_phys_addr;
	stm->regs->dmatdlar = stm->tx_bd_dma_phys_addr;

    /* flush transmit fifo */
    stm->regs->dmaomr |= STM32_MAC_DMAOMR_FTF;
    for (i = 0; i < 10; i++) {
		if (!(stm->regs->dmaomr & STM32_MAC_DMAOMR_FTF))
			break;
		msleep(1);
	}
	if (i == 10) {
        warning("FIFO flush timeout\n");
		rv = -EBUSY;
		goto out;
	}
    /* start ethernet controller */
    stm->regs->maccr |= STM32_MAC_CR_TE | STM32_MAC_CR_RE;
    /* start dma */
    stm->regs->dmaomr |= STM32_MAC_DMAOMR_ST | STM32_MAC_DMAOMR_SR;

	rv = 0;
out:
    debug("stm32_eth_start exit with %d\n", rv);

	return rv;
}

static void stm32_eth_buffers_free(struct net_device *dev)
{
    struct stm32_eth_priv *stm = netdev_priv(dev);
    int i;

    /* transmit buffers */
    if (stm->tx_bd) {
        /* free buffers */
        for (i = 0; i < stm->tx_buf_num; i++) {
            if (stm->tx_buffer[i]) {
                gen_pool_free(dma_pool, (unsigned long) stm->tx_buffer[i], stm->frame_max_size + 4);
                stm->tx_buffer[i] = NULL;
                stm->tx_bd[i].buf = 0;
            }
        }
        /* free buffer descriptors */
        gen_pool_free(dma_pool, (unsigned long) stm->tx_bd, sizeof(struct stm32_eth_dma_bd) * stm->tx_buf_num);
        stm->tx_bd = NULL;
        stm->tx_bd_dma_phys_addr = 0;
    }

    /* receive buffers */
    if (stm->rx_bd) {
        /* free buffers */
        for (i = 0; i < stm->rx_buf_num; i++) {
            if (stm->rx_buffer[i]) {
                gen_pool_free(dma_pool, (unsigned long) stm->rx_buffer[i], stm->frame_max_size + 4);
                stm->rx_buffer[i] = NULL;
                stm->rx_bd[i].buf = 0;
            }
        }
        /* free buffer descriptors */
        gen_pool_free(dma_pool, (unsigned long) stm->rx_bd, sizeof(struct stm32_eth_dma_bd) * stm->rx_buf_num);
        stm->rx_bd = NULL;
        stm->rx_bd_dma_phys_addr = 0;
    }
}

static int stm32_eth_buffers_alloc(struct net_device *dev)
{
    struct stm32_eth_priv *stm = netdev_priv(dev);
    int i;
    int rv;

    debug("stm32_eth_buffers_alloc\n");
    /* allocate buffer descriptor memory */
    stm->rx_bd = (struct stm32_eth_dma_bd *) gen_pool_dma_alloc(dma_pool, sizeof(struct stm32_eth_dma_bd) * stm->rx_buf_num,
                                                                &stm->rx_bd_dma_phys_addr);
    stm->tx_bd = (struct stm32_eth_dma_bd *) gen_pool_dma_alloc(dma_pool, sizeof(struct stm32_eth_dma_bd) * stm->tx_buf_num,
                                                                &stm->tx_bd_dma_phys_addr);

    if (!stm->rx_bd || !stm->tx_bd) {
		rv = -ENOMEM;
		goto out;
	}

    /* now setup rx buffer descriptors */
    for (i = 0; i < stm->rx_buf_num; i++) {
        stm->rx_bd[i].stat = STM32_DMA_RBD_DMA_OWN;
        stm->rx_bd[i].ctrl = STM32_DMA_RBD_RCH | stm->frame_max_size;
        stm->rx_buffer[i] = gen_pool_dma_alloc(dma_pool, stm->frame_max_size + 4, & stm->rx_bd[i].buf);
        if (!stm->rx_buffer[i]) {
            rv = -ENOMEM;
            goto out;
            }
        stm->rx_bd[i].next = stm->rx_bd_dma_phys_addr + (sizeof(struct stm32_eth_dma_bd) * ((i + 1) % stm->rx_buf_num));
    }

    /* for tx we use skbs receive from kernel. setup what we can for tx buffer descriptors */
    for (i = 0; i < stm->tx_buf_num; i++) {
		stm->tx_bd[i].stat = STM32_DMA_TBD_TCH;
		stm->tx_bd[i].ctrl = 0;
        stm->tx_buffer[i] = gen_pool_dma_alloc(dma_pool, stm->frame_max_size + 4, & stm->tx_bd[i].buf);
        if (!stm->tx_buffer[i]) {
            rv = -ENOMEM;
            goto out;
            }
		stm->tx_bd[i].next = stm->tx_bd_dma_phys_addr + (sizeof(struct stm32_eth_dma_bd) * ((i + 1) % stm->tx_buf_num));
    }

	rv = 0;
out:
    debug("stm32_eth_buffers_alloc exit with %d\n", rv);
	if (rv != 0)
		stm32_eth_buffers_free(dev);

	return rv;
}

static void stm32_eth_tx_complete(struct net_device *dev)
{
    struct stm32_eth_priv *stm = netdev_priv(dev);
    
    /* at least one frame has been transmitted */
    spin_lock(&stm->tx_lock);
    while (stm->tx_buffer_nb_in_queue) {
        u32 index = stm->tx_next_release_buffer_index;
        volatile struct stm32_eth_dma_bd *bd = &stm->tx_bd[index];
        u32 status;
        
        status = bd->stat;
        /* if buffer still own by dma then exit */
        if (status & STM32_DMA_TBD_DMA_OWN)
			break;
        /* update stats */
		if (status & STM32_DMA_TBD_ES) {
			stm->stat.tx_errors++;
			if (status & STM32_DMA_TBD_NC)
				stm->stat.tx_carrier_errors++;
			if (status & STM32_DMA_TBD_UF)
				stm->stat.tx_fifo_errors++;
		} else {
			stm->stat.tx_packets++;
			stm->stat.tx_bytes += stm->tx_bd[index].ctrl;
		}
		if (status & STM32_DMA_TBD_EC) {
			stm->stat.collisions += 16;
		} else {
			stm->stat.collisions += (status >> STM32_DMA_TBD_CC_BIT) & STM32_DMA_TBD_CC_MSK;
		}
        /* update pointers */
        stm->tx_buffer_nb_in_queue--;
        stm->tx_next_release_buffer_index = (stm->tx_next_release_buffer_index + 1) % stm->tx_buf_num;
    }

    /* if queue was full then restart it */
	if (unlikely(stm->tx_is_blocked)) {
		stm->tx_is_blocked = 0;
		netif_wake_queue(dev);
	}

	spin_unlock(&stm->tx_lock);
}

static int stm32_eth_rx_get(struct net_device *dev, int processed, int budget)
{
	struct stm32_eth_priv *stm = netdev_priv(dev);

    while (processed < budget) {
        volatile struct stm32_eth_dma_bd *bd;
        u32 index = stm->rx_index;
        u32 status;
        u32 len;
        struct sk_buff *skb;

        bd =  &stm->rx_bd[index];
        status = bd->stat;
        /* test if dma has release buffer descriptor. if not exit loop */
        if (status & STM32_DMA_RBD_DMA_OWN)
            break;
        /* update error stats */
		if (status & STM32_DMA_RBD_ES) {
			stm->stat.rx_errors++;
			if (status & STM32_DMA_RBD_LE)
				stm->stat.rx_length_errors++;
			if (status & STM32_DMA_RBD_OE)
				stm->stat.rx_fifo_errors++;
			if (status & STM32_DMA_RBD_CE)
				stm->stat.rx_crc_errors++;
			if ((status & (STM32_DMA_RBD_FS | STM32_DMA_RBD_LS)) != (STM32_DMA_RBD_FS | STM32_DMA_RBD_LS)) {
				stm->stat.rx_frame_errors++;
			}

			goto next;
		}
        /* ok we have a new good frame to process */
        len = ((status >> STM32_DMA_RBD_FL_BIT) & STM32_DMA_RBD_FL_MSK) - 4;
        /* allocate skbuff and copy data in it */        
        skb = netdev_alloc_skb_ip_align(dev, len);
        if (unlikely(!skb)) {
			stm->stat.rx_dropped++;
			goto next;
		}
        skb_copy_to_linear_data(skb, stm->rx_buffer[index], len);
        skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, dev);
        netif_receive_skb(skb);
        /* update stats */        
        stm->stat.rx_packets++;
		stm->stat.rx_bytes += len;

next:
        /* give ownwer ship to dma and move index */
        bd->stat = STM32_DMA_RBD_DMA_OWN;
        stm->rx_index = (stm->rx_index + 1) % stm->rx_buf_num;
        processed++;
    }

    return processed;
}

static irqreturn_t stm32_eth_irq(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct stm32_eth_priv *stm;
	irqreturn_t rv;
	u32 status;

	stm = netdev_priv(dev);
    /* get irq status and acknoledge */
    status = stm->regs->dmasr;
	stm->regs->dmasr = status;

    if (!status) {
        /* spurious */
        rv = IRQ_NONE;
		goto out;
	}

    /* test reception */
    if (status & STM32_MAC_DMASR_RX_MSK) {
        if (status & STM32_MAC_DMASR_ROS)
			stm->stat.rx_over_errors++;
        /* start napi stuff */
        spin_lock(&stm->rx_lock);
		if (likely(napi_schedule_prep(&stm->napi))) {
			stm->regs->dmaier &= ~STM32_MAC_DMAIER_RIE;
			__napi_schedule(&stm->napi);
		}
		spin_unlock(&stm->rx_lock);
    }
    /* test transmit */
    if (status & STM32_MAC_DMASR_TX_MSK) {
        if (status & STM32_MAC_DMASR_TUS)
			stm->stat.tx_fifo_errors++;
        stm32_eth_tx_complete(dev);
    }

	rv = IRQ_HANDLED;
out:
	return rv;
}

/* napi method */
static int stm32_eth_rx_poll(struct napi_struct *napi, int budget)
{
	struct stm32_eth_priv *stm = container_of(napi, struct stm32_eth_priv,napi);
	struct net_device *dev = stm->dev;
	int rx = 0;

    rx = stm32_eth_rx_get(dev, rx, budget);
    spin_lock(&stm->rx_lock);
    stm->regs->dmaier |= STM32_MAC_DMAIER_RIE;
    napi_complete(napi);
    spin_unlock(&stm->rx_lock);

    return rx;
}

/* net device methods */

/* MDIO interface */
static int stm32_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 data)
{
    struct stm32_eth_priv *stm = bus->priv;
    u32 val = 0;
    int i;

    debug("stm32_mdio_write\n");
    phy_id &= STM32_MAC_MIIAR_PA_MSK;
    val = phy_id << STM32_MAC_MIIAR_PA_BIT;
    reg &= STM32_MAC_MIIAR_MR_MSK;
    val |= reg << STM32_MAC_MIIAR_MR_BIT;
    val |= 3 << STM32_MAC_MIIAR_CR_BIT;/* FIXME */
    val |= STM32_MAC_MIIAR_MW | STM32_MAC_MIIAR_MB;

    /* start write */
    stm->regs->macmiidr = data;
	stm->regs->macmiiar = val;

    /* wait untill write is finish */
    for (i = 0; i < 10; i++) {
        if (!(stm->regs->macmiiar & STM32_MAC_MIIAR_MB))
			break;
		msleep(1);
	}
    if (i == 10)
        warning("stm32_mdio_write: failed to write\n");
    return 0;
}

static int stm32_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
    struct stm32_eth_priv *stm = bus->priv;
    u32 val = 0;
    int i;
    int res;

    debug("stm32_mdio_read phy%d[%d]\n", phy_id, reg);
    phy_id &= STM32_MAC_MIIAR_PA_MSK;
    val = phy_id << STM32_MAC_MIIAR_PA_BIT;
    reg &= STM32_MAC_MIIAR_MR_MSK;
    val |= reg << STM32_MAC_MIIAR_MR_BIT;
    val |= 3 << STM32_MAC_MIIAR_CR_BIT;/* FIXME */
    val |= STM32_MAC_MIIAR_MB;

    /* start read */
	stm->regs->macmiiar = val;
    
    /* wait until read is finish */
    for (i = 0; i < 10; i++) {
        if (!(stm->regs->macmiiar & STM32_MAC_MIIAR_MB))
			break;
		msleep(1);
	}
    if (i != 10) {
		res = stm->regs->macmiidr;
	} else {
		warning("stm32_mdio_read: failed to read\n");
		res = 0xFFFF;
	}

    debug(" phy%d[%d] = 0x%08x\n", phy_id, reg, res);

	return res;
}

static void stm32_params_setup(struct stm32_eth_priv *stm)
{
	u32 tmp = stm->regs->maccr;

	if (stm->duplex == DUPLEX_FULL) {
		tmp |= STM32_MAC_CR_DM;
	} else {
		tmp &= ~STM32_MAC_CR_DM;
	}

	if (stm->speed == SPEED_100) {
		tmp |= STM32_MAC_CR_FES;
	} else {
		tmp &= ~STM32_MAC_CR_FES;
	}
	stm->regs->maccr = tmp;
}

static void stm32_handle_link_change(struct net_device *ndev)
{
    struct stm32_eth_priv *stm = netdev_priv(ndev);
	struct phy_device *phydev = stm->phy_dev;
	unsigned long flags;
	s32 status_change = 0;

    /* test for real changes */
	spin_lock_irqsave(&stm->lock, flags);
	if (phydev->link) {
		if ((stm->speed != phydev->speed) ||
		    (stm->duplex != phydev->duplex)) {
			stm->speed = phydev->speed;
			stm->duplex = phydev->duplex;
			status_change = 1;
		}
	}
	if (phydev->link != stm->link) {
		if (!phydev->link) {
			stm->speed = 0;
			stm->duplex = -1;
		}
		stm->link = phydev->link;

		status_change = 1;
	}
	spin_unlock_irqrestore(&stm->lock, flags);

    /* update registers */
	if (status_change) {
        debug("stm32_handle_link_change : status has change\n");
		phy_print_status(phydev);
		stm32_params_setup(stm);
	}
}

static int stm32_mii_probe(struct net_device *ndev)
{
    struct stm32_eth_priv *stm = netdev_priv(ndev);
	struct phy_device *phydev = NULL;
    int phy_addr;
    
    debug("stm32_mii_probe");
	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (stm->mii_bus->phy_map[phy_addr]) {
			phydev = stm->mii_bus->phy_map[phy_addr];
			info("found PHY id 0x%x addr %d\n", phydev->phy_id, phydev->addr);
			break;
		}
	}

	if (!phydev) {
		warning("%s: no PHY found\n", ndev->name);
		return -ENODEV;
	}

	/* Attach to the PHY */
	info("%s: using MII interface\n", ndev->name);
	phydev = phy_connect(ndev, dev_name(&phydev->dev), &stm32_handle_link_change, PHY_INTERFACE_MODE_MII);

	if (IS_ERR(phydev)) {
		error("%s: Could not attach to PHY\n", ndev->name);
		return PTR_ERR(phydev);
	}

	/* mask with MAC supported features */
	phydev->supported &= PHY_BASIC_FEATURES;
	phydev->advertising = phydev->supported;

    stm->link = 0;
	stm->speed = 0;
	stm->duplex = -1;
	stm->phy_dev = phydev;

	return 0;
}

static int stm32_mii_init(struct stm32_eth_priv	*stm)
{
    int err = -ENOMEM;
    int i;

    stm->mii_bus = mdiobus_alloc();
    if (!stm->mii_bus) {
        warning("mdiobus_alloc failed\n");
        goto err_out;
    }
    /* setup mii info */
    stm->mii_bus->name = "stm32429i_mii_bus";
	stm->mii_bus->read = &stm32_mdio_read;
	stm->mii_bus->write = &stm32_mdio_write;
    snprintf(stm->mii_bus->id, MII_BUS_ID_SIZE, "%02x", 1);
    stm->mii_bus->priv = stm;
	stm->mii_bus->parent = NULL;
	stm->mii_bus->phy_mask = 0xFFFFFFF0;
    stm->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!stm->mii_bus->irq) {
        warning("failed to alloc memory for mii irq\n");
		goto err_out_1;
	}
    for (i = 0; i < PHY_MAX_ADDR; i++)
		stm->mii_bus->irq[i] = PHY_POLL;

    /* register */
    err = mdiobus_register(stm->mii_bus);
	if (err) {
        warning("mdiobus_register has failed\n");
		goto err_out_free_mdio_irq;
	}

    /* probe */
    err = stm32_mii_probe(stm->dev);
	if (err) {
		goto err_out_unregister_bus;
	}
    
    err = 0;
	goto err_out;

err_out_unregister_bus:
	mdiobus_unregister(stm->mii_bus);
err_out_free_mdio_irq:
	kfree(stm->mii_bus->irq);
err_out_1:
	mdiobus_free(stm->mii_bus);
err_out:
	return err;
}

/* Net device interface */
static int stm32_netdev_open(struct net_device *dev)
{
	struct stm32_eth_priv *stm = netdev_priv(dev);
	int rv;

    debug("stm32_netdev_open 0x%p\n", stm);
    /* auto-negotiation */
    stm->link = 0;
	stm->duplex = -1;
	stm->speed = -1;
    phy_start_aneg(stm->phy_dev);
    /* start phy */
	phy_start(stm->phy_dev);

    /* allocate buffers */
    rv = stm32_eth_buffers_alloc(dev);
	if (rv) {
		goto init_err;
	}
    napi_enable(&stm->napi);
   
    /* start hardware */ 
    rv = stm32_eth_start(dev);
	if (rv) {
		napi_disable(&stm->napi);
		stm32_eth_buffers_free(dev);
		goto init_err;
	}

    spin_lock_init(&stm->rx_lock);
	spin_lock_init(&stm->tx_lock);

    /* setup indexes */
    stm->tx_next_free_buffer_index = 0;
    stm->tx_next_release_buffer_index = 0;
    stm->tx_buffer_nb_in_queue = 0;
    stm->tx_is_blocked = 0;
    stm->rx_index = 0;

    /* setup interrupt handler */
    rv = request_irq(stm->irq, stm32_eth_irq, IRQF_SHARED, dev->name, dev);
	if (rv) {
		napi_disable(&stm->napi);
		stm32_eth_stop(dev);
		stm32_eth_buffers_free(dev);
		goto init_err;
	}

    /* enable interrupts */
    stm->regs->dmaier |= STM32_MAC_DMAIER_TIE | STM32_MAC_DMAIER_RIE | STM32_MAC_DMAIER_AISE | STM32_MAC_DMAIER_NISE;

    /* notify we are ready to work */
    netif_start_queue(dev);
	rv = 0;

init_err:
    debug("stm32_netdev_open exit with %d\n", rv);
	if (rv)
		phy_stop(stm->phy_dev);
    return rv;
}

static int stm32_netdev_close(struct net_device *dev)
{
	struct stm32_eth_priv *stm = netdev_priv(dev);

    debug("stm32_netdev_close\n");
	napi_disable(&stm->napi);
	netif_stop_queue(dev);

	stm->regs->dmaier &= ~(STM32_MAC_DMAIER_TIE | STM32_MAC_DMAIER_RIE | STM32_MAC_DMAIER_AISE | STM32_MAC_DMAIER_NISE);
	free_irq(stm->irq, dev);

	stm32_eth_stop(dev);
	stm32_eth_buffers_free(dev);

	return 0;
}

static int stm32_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct stm32_eth_priv *stm = netdev_priv(dev);
	unsigned long flags;
	int rv;
    int index;
    
    /* sanity check */
    if (unlikely(skb->len > stm->frame_max_size)) {
		stm->stat.tx_dropped++;
		dev_kfree_skb(skb);
		rv = NETDEV_TX_OK;
		goto out;
	}

    /* update pointers and get buffer index */
    spin_lock_irqsave(&stm->tx_lock, flags);
     /* should not occur */
    if (stm->tx_is_blocked || stm->tx_buffer_nb_in_queue >= stm->tx_buf_num) {
        spin_unlock_irqrestore(&stm->tx_lock, flags);
		rv = NETDEV_TX_BUSY;
		warning("stm32_netdev_xmit fifo full => driver bug ? (don't think so in case another client has already start to push data)\n");
		goto out;
	}
    stm->tx_buffer_nb_in_queue++;
    index = stm->tx_next_free_buffer_index;
    stm->tx_next_free_buffer_index = (stm->tx_next_free_buffer_index + 1) % stm->tx_buf_num;
    spin_unlock_irqrestore(&stm->tx_lock, flags);

    /* setup dma */
    dev->trans_start = jiffies;
    stm->tx_bd[index].ctrl = skb->len;
    skb_copy_from_linear_data(skb, stm->tx_buffer[index], skb->len);
    dev_kfree_skb(skb);
    /* full ethernet frame is in this buffer */
    stm->tx_bd[index].stat |= STM32_DMA_TBD_FS | STM32_DMA_TBD_LS | STM32_DMA_TBD_DMA_OWN | STM32_DMA_TBD_DMA_IC;

    /* request dma to poll current buffer descritor in case it was in suspend mode */
    stm->regs->dmatpdr = 0;

    /* check if our fifo is full and we need to stop client */
    spin_lock_irqsave(&stm->tx_lock, flags);
    if (stm->tx_buffer_nb_in_queue == stm->tx_buf_num) {
        stm->tx_is_blocked = 1;
        netif_stop_queue(dev);
		spin_unlock_irqrestore(&stm->tx_lock, flags);
        info("tx queue fifo is full\n");
    } else
        spin_unlock_irqrestore(&stm->tx_lock, flags);

	rv = NETDEV_TX_OK;
out:
	return rv;
}

static int stm32_netdev_ioctl(struct net_device *dev, struct ifreq *ifr,
			      int cmd)
{
	struct stm32_eth_priv *stm = netdev_priv(dev);
	struct phy_device *phydev = stm->phy_dev;

    debug("stm32_netdev_ioctl\n");
	if (!netif_running(dev))
	{
		return -EINVAL;
	}

	if (!phydev)
	{
		return -ENODEV;
	}

	return phy_mii_ioctl(phydev, ifr, cmd);
}

static struct net_device_stats *stm32_netdev_get_stats(struct net_device *dev)
{
	struct stm32_eth_priv	*stm = netdev_priv(dev);

    debug("stm32_netdev_get_stats\n");
	return &stm->stat;
}

static const struct net_device_ops stm32_netdev_ops = {
    .ndo_open               = stm32_netdev_open,
	.ndo_stop               = stm32_netdev_close,
	.ndo_start_xmit         = stm32_netdev_xmit,
	.ndo_get_stats          = stm32_netdev_get_stats,
	.ndo_do_ioctl           = stm32_netdev_ioctl,

	.ndo_validate_addr      = eth_validate_addr,
	.ndo_change_mtu         = eth_change_mtu,
	.ndo_set_mac_address    = eth_mac_addr,
};

static int __init stm32429i_eth_init(void)
{
    struct net_device *dev;
    struct stm32_eth_priv *stm;
    char *p;
    int rv;

    debug("stm32429i_eth_init\n");
    /* create memory pool for buffers */
    rv = init_dma_mem_pool();

    /* create ethernet device */
    dev = alloc_etherdev(sizeof(struct stm32_eth_priv));
    if (!dev) {
        warning("alloc_etherdev failed\n");
        rv = -ENOMEM;
		goto out;
    } else
        debug("alloc_etherdev success 0x%p\n", dev);
    SET_NETDEV_DEV(dev, NULL);
    
    /* mac address */
    p = strnstr(boot_command_line, "ethaddr=", COMMAND_LINE_SIZE);
    if (p) {
		char ethaddr[18];
		int	i;

		memcpy(ethaddr, &p[strlen("ethaddr=")], sizeof(ethaddr));
		p = ethaddr;
		for (i = 0; i < ETH_ALEN; i++) {
			dev->dev_addr[i] = (simple_strtol(p, &p, 16) << 0) | (simple_strtol(p, &p, 16) << 4);
			p++;
		}
        info("using mac address %s\n", &p[strlen("ethaddr=")]);
	} else {
        dev->dev_addr[0] = 0xa2;
        dev->dev_addr[1] = 0xca;
        dev->dev_addr[2] = 0xff;
        dev->dev_addr[3] = 0xee;
        dev->dev_addr[4] = 0xba;
        dev->dev_addr[5] = 0xbe;
        warning("using hardwired mac address %02x:%02x:%02x:%02x:%02x:%02x\n", dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
                                                                               dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
    }

    /* setup ops */
    dev->netdev_ops = &stm32_netdev_ops;

    /* add napi ops */
    stm = netdev_priv(dev);
    stm->dev = dev;
    netif_napi_add(dev, &stm->napi, stm32_eth_rx_poll, 64);

    /* setup stm private parameters */
    stm->regs = (volatile struct stm32_mac_regs *) 0x40028000;
    stm->irq = 61;
    stm->frame_max_size = 2044;
    stm->rx_buf_num = 4;
	stm->tx_buf_num = 4;
    stm->tx_next_free_buffer_index = 0;
    stm->tx_next_release_buffer_index = 0;
    stm->tx_buffer_nb_in_queue = 0;
    stm->tx_is_blocked = 0;
    stm->rx_index = 0;

    /* allocate mem for buffer array */
    stm->rx_buffer = kmalloc(stm->rx_buf_num * sizeof(void *), GFP_KERNEL);
    stm->tx_buffer = kmalloc(stm->tx_buf_num * sizeof(void *), GFP_KERNEL);
    if (!stm->rx_buffer || !stm->tx_buffer) {
		error("rx/tx (%d/%d) buffer array allocation failed\n", stm->rx_buf_num, stm->tx_buf_num);
		rv = -ENOMEM;
		goto out;
	}

    /* register driver */
    rv = register_netdev(dev);
    if (rv) {
        warning("netdev registration failed\n");
        rv = -ENODEV;
		goto out;
    } else {
        debug("netdev registration success\n");
    }

    /* init mii */
    if (stm32_mii_init(stm) != 0) {
        warning("stm32_mii_init\n");
		goto out;
	} else {
        info("stm32_mii_init success\n");
    }

    rv = 0;
out:
    debug("stm32429i_eth_init exit with code %d\n", rv);
    return rv;
}

module_init(stm32429i_eth_init);

MODULE_ALIAS("need_to_be_define");
MODULE_DESCRIPTION("stm32429i MAC driver");
MODULE_AUTHOR("me");
MODULE_LICENSE("GPL");
