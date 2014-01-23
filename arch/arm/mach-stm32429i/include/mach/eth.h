#ifndef _MACH_STM32429I_ETH_H_
#define _MACH_STM32429I_ETH_H_

/* maccr */
#define STM32_MAC_CR_RE			(1 << 2)	/* Receiver enable    */
#define STM32_MAC_CR_TE			(1 << 3)	/* Transmitter enable */
#define STM32_MAC_CR_DM			(1 << 11)	/* Duplex mode	      */
#define STM32_MAC_CR_FES		(1 << 14)	/* Fast Eth speed     */

/* macmiiar */
#define STM32_MAC_MIIAR_MB		(1 << 0)	/* MII busy */
#define STM32_MAC_MIIAR_MW		(1 << 1)	/* MII write */

#define STM32_MAC_MIIAR_CR_BIT		2		/* Clock range */
#define STM32_MAC_MIIAR_CR_MSK		0x7

#define STM32_MAC_MIIAR_MR_BIT		6		/* MII register */
#define STM32_MAC_MIIAR_MR_MSK		0x1F

#define STM32_MAC_MIIAR_PA_BIT		11		/* PHY address */
#define STM32_MAC_MIIAR_PA_MSK		0x1F

/* dmabmr */
#define STM32_MAC_DMABMR_SR         (1 << 0)    /* Software reset */

#define STM32_MAC_DMABMR_PBL_BIT	8		    /* Burst length */
#define STM32_MAC_DMABMR_PBL_MSK	0x3F

#define STM32_MAC_DMABMR_RTPR_BIT	14          /* Rx:Tx priority ratio */
#define STM32_MAC_DMABMR_RTPR_MSK	0x3
#define STM32_MAC_DMABMR_RTPR_1_1	0x0         /* 1 : 1 */
#define STM32_MAC_DMABMR_RTPR_2_1	0x1         /* 2 : 1 */
#define STM32_MAC_DMABMR_RTPR_3_1	0x2         /* 3 : 1 */
#define STM32_MAC_DMABMR_RTPR_4_1	0x3         /* 4 : 1 */

#define STM32_MAC_DMABMR_FB         (1 << 16)	/* Fixed burst */

#define STM32_MAC_DMABMR_RDP_BIT	17          /* RX DMA PBL */
#define STM32_MAC_DMABMR_RDP_MSK	0x3F

#define STM32_MAC_DMABMR_USP		(1 << 23)   /* Use separate PBL   */
#define STM32_MAC_DMABMR_AAB		(1 << 25)   /* Adr-aligned beats  */

/* dmasr */
#define STM32_MAC_DMASR_TS          (1 << 0)	/* Transmission done  */
#define STM32_MAC_DMASR_TBUS        (1 << 2)	/* Tx buf unavailable */
#define STM32_MAC_DMASR_ROS         (1 << 4)	/* Receive overflow   */
#define STM32_MAC_DMASR_TUS         (1 << 5)	/* Transmit underflow */
#define STM32_MAC_DMASR_RS          (1 << 6)	/* Reception done     */
#define STM32_MAC_DMASR_RBUS        (1 << 7)	/* Rx buf unavailable */
#define STM32_MAC_DMASR_AIS         (1 << 15)	/* Abnormal summary   */
#define STM32_MAC_DMASR_NIS         (1 << 16)	/* Normal int summary */

#define STM32_MAC_DMASR_TX_MSK		(STM32_MAC_DMASR_TS | STM32_MAC_DMASR_TBUS | STM32_MAC_DMASR_TUS)
#define STM32_MAC_DMASR_RX_MSK		(STM32_MAC_DMASR_RS | STM32_MAC_DMASR_ROS  | STM32_MAC_DMASR_RBUS)

/* dmaomr */
#define STM32_MAC_DMAOMR_SR         (1 << 1)    /* Start/stop rx */
#define STM32_MAC_DMAOMR_ST         (1 << 13)   /* Start/stop tx */
#define STM32_MAC_DMAOMR_FTF        (1 << 20)   /* Flush tansmit fifo */

/* dmaier */
#define STM32_MAC_DMAIER_TIE		(1 << 0)	/* Tx done interrupt  */
#define STM32_MAC_DMAIER_RIE		(1 << 6)	/* Rx done interrupt  */
#define STM32_MAC_DMAIER_AISE		(1 << 15)	/* Abnormal int summary enable*/
#define STM32_MAC_DMAIER_NISE		(1 << 16)	/* Normal int summary enable */

#endif

