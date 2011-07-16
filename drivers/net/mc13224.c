/*
 * Freescale MC13224 802.15.4 SOC radio
 *
 * Copyright (C) 2011 Jon Smirl
 * Author: Jon Smirl <jonsmirl@gmail.com>
 * based on enc28j60.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/mc13224.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/ipv6.h>
#include <linux/if_arp.h>
#include <net/af_ieee802154.h>


#include "mc13224_hw.h"

#define DRV_NAME	"mc13224"
#define DRV_VERSION	"1.01"

#define SPI_OPLEN	1

#undef KERN_DEBUG
#define KERN_DEBUG

#define MC13224_MSG_DEFAULT 0xFFFF
//#define MC13224_MSG_DEFAULT (NETIF_MSG_PROBE | NETIF_MSG_IFUP | NETIF_MSG_IFDOWN | NETIF_MSG_LINK)

/* Buffer size required for the largest SPI transfer (i.e., reading a
 * frame). */
#define SPI_TRANSFER_BUF_LEN	(4 + MAX_FRAMELEN)

#define TX_TIMEOUT	(4 * HZ)

/* Max TX retries in case of collision as suggested by errata datasheet */
#define MAX_TX_RETRYCOUNT	16

enum {
	RXFILTER_NORMAL,
	RXFILTER_MULTI,
	RXFILTER_PROMISC
};

/* Driver local data */
struct mc13224_net {
	struct net_device *netdev;
	struct spi_device *spi;
	struct mutex lock;
	struct sk_buff *tx_skb;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct work_struct restart_work;
	u8 bank;		/* current register bank selected */
	u16 next_pk_ptr;	/* next packet pointer within FIFO */
	u16 max_pk_counter;	/* statistics: max packet counter */
	u16 tx_retry_count;
	bool hw_enable;
	int rxfilter;
	u32 msg_enable;
	u8 spi_transfer_buf[SPI_TRANSFER_BUF_LEN];
	struct mc13224_platform_data *pdata;
};


/* use ethtool to change the level for any given device */
static struct {
	u32 msg_enable;
} debug = { 0xFFFF };

/*
 * SPI read buffer
 * wait for the SPI transfer and copy received data to destination
 */
static int
spi_read_buf(struct mc13224_net *priv, int len, u8 *data)
{
	u8 *rx_buf = priv->spi_transfer_buf + 4;
	u8 *tx_buf = priv->spi_transfer_buf;
	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = SPI_OPLEN + len,
	};
	struct spi_message msg;
	int ret;
printk("mc13224 - spi_read_buf %d\n", len);

	tx_buf[0] = MC13224_READ_BUF_MEM;
	tx_buf[1] = tx_buf[2] = tx_buf[3] = 0;	/* don't care */

	spi_message_init(&msg);
	spi_message_add_tail(&t, &msg);
	ret = spi_sync(priv->spi, &msg);
	if (ret == 0) {
		memcpy(data, &rx_buf[SPI_OPLEN], len);
		ret = msg.status;
	}
	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__func__, ret);

	return ret;
}

/*
 * SPI write buffer
 */
static int spi_write_buf(struct mc13224_net *priv, int len,
			 const u8 *data)
{
	int ret;

printk("mc13224 - spi_write_buf %d\n", len);
	if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0)
		ret = -EINVAL;
	else {
//		priv->spi_transfer_buf[0] = MC13224_WRITE_BUF_MEM;
		memcpy(&priv->spi_transfer_buf[0], data, len);
		ret = spi_write(priv->spi, priv->spi_transfer_buf, len);
		if (ret && netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
				__func__, ret);
	}
	return ret;
}

static void mc13224_soft_reset(struct mc13224_net *priv)
{
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	//spi_write_op(priv, MC13224_SOFT_RESET, 0, MC13224_SOFT_RESET);
	/* Errata workaround #1, CLKRDY check is unreliable,
	 * delay at least 1 mS instead */
	udelay(2000);
}

/*
 * select the current register bank if necessary
 */
static void mc13224_set_bank(struct mc13224_net *priv, u8 addr)
{
	u8 b = (addr & BANK_MASK) >> 5;

	/* These registers (EIE, EIR, ESTAT, ECON2, ECON1)
	 * are present in all banks, no need to switch bank
	 */
	if (addr >= EIE && addr <= ECON1)
		return;
#if 0
	/* Clear or set each bank selection bit as needed */
	if ((b & ECON1_BSEL0) != (priv->bank & ECON1_BSEL0)) {
		if (b & ECON1_BSEL0)
			spi_write_op(priv, MC13224_BIT_FIELD_SET, ECON1,
					ECON1_BSEL0);
		else
			spi_write_op(priv, MC13224_BIT_FIELD_CLR, ECON1,
					ECON1_BSEL0);
	}
	if ((b & ECON1_BSEL1) != (priv->bank & ECON1_BSEL1)) {
		if (b & ECON1_BSEL1)
			spi_write_op(priv, MC13224_BIT_FIELD_SET, ECON1,
					ECON1_BSEL1);
		else
			spi_write_op(priv, MC13224_BIT_FIELD_CLR, ECON1,
					ECON1_BSEL1);
	}
#endif
	priv->bank = b;
}

/*
 * Register access routines through the SPI bus.
 * Every register access comes in two flavours:
 * - nolock_xxx: caller needs to invoke mutex_lock, usually to access
 *   atomically more than one register
 * - locked_xxx: caller doesn't need to invoke mutex_lock, single access
 *
 * Some registers can be accessed through the bit field clear and
 * bit field set to avoid a read modify write cycle.
 */

/*
 * Buffer memory read
 * Select the starting address and execute a SPI buffer read
 */
static void mc13224_mem_read(struct mc13224_net *priv,
				     u16 addr, int len, u8 *data)
{
	mutex_lock(&priv->lock);
	//nolock_regw_write(priv, ERDPTL, addr);
#ifdef CONFIG_MC13224_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u16 reg;
		//reg = nolock_regw_read(priv, ERDPTL);
		if (reg != addr)
			printk(KERN_DEBUG DRV_NAME ": %s() error writing ERDPT "
				"(0x%04x - 0x%04x)\n", __func__, reg, addr);
	}
#endif
	spi_read_buf(priv, len, data);
	mutex_unlock(&priv->lock);
}

/*
 * Write packet to mc13224 TX buffer memory
 */
static void
mc13224_packet_write(struct mc13224_net *priv, int len, const u8 *data)
{
	mutex_lock(&priv->lock);
	/* Set the write pointer to start of transmit buffer area */
	//nolock_regw_write(priv, EWRPTL, TXSTART_INIT);
#ifdef CONFIG_MC13224_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u16 reg;
		//reg = nolock_regw_read(priv, EWRPTL);
		if (reg != TXSTART_INIT)
			printk(KERN_DEBUG DRV_NAME
				": %s() ERWPT:0x%04x != 0x%04x\n",
				__func__, reg, TXSTART_INIT);
	}
#endif
#if 0
	/* Set the TXND pointer to correspond to the packet size given */
	//nolock_regw_write(priv, ETXNDL, TXSTART_INIT + len);
	/* write per-packet control byte */
	spi_write_op(priv, MC13224_WRITE_BUF_MEM, 0, 0x00);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME
			": %s() after control byte ERWPT:0x%04x\n",
			__func__, nolock_regw_read(priv, EWRPTL));
#endif
	/* copy the packet into the transmit buffer */
	spi_write_buf(priv, len, data);
#if 0
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME
			 ": %s() after write packet ERWPT:0x%04x, len=%d\n",
			 __func__, nolock_regw_read(priv, EWRPTL), len);
#endif
	mutex_unlock(&priv->lock);
}

static unsigned long msec20_to_jiffies;

static int poll_ready(struct mc13224_net *priv, u8 reg, u8 mask, u8 val)
{
	unsigned long timeout = jiffies + msec20_to_jiffies;

	/* 20 msec timeout read */
#if 0
	while ((nolock_regb_read(priv, reg) & mask) != val) {
		if (time_after(jiffies, timeout)) {
			if (netif_msg_drv(priv))
				dev_dbg(&priv->spi->dev,
					"reg %02x ready timeout!\n", reg);
			return -ETIMEDOUT;
		}
		cpu_relax();
	}
#endif
	return 0;
}

/*
 * Wait until the PHY operation is complete.
 */
static int wait_phy_ready(struct mc13224_net *priv)
{
	return poll_ready(priv, MISTAT, MISTAT_BUSY, 0) ? 0 : 1;
}

/*
 * PHY register read
 * PHY registers are not accessed directly, but through the MII
 */
static u16 mc13224_phy_read(struct mc13224_net *priv, u8 address)
{
	u16 ret;

	mutex_lock(&priv->lock);
	/* set the PHY register address */
	//nolock_regb_write(priv, MIREGADR, address);
	/* start the register read operation */
	//nolock_regb_write(priv, MICMD, MICMD_MIIRD);
	/* wait until the PHY read completes */
	wait_phy_ready(priv);
	/* quit reading */
	//nolock_regb_write(priv, MICMD, 0x00);
	/* return the data */
	//ret  = nolock_regw_read(priv, MIRDL);
	mutex_unlock(&priv->lock);

	return ret;
}

static int mc13224_phy_write(struct mc13224_net *priv, u8 address, u16 data)
{
	int ret;

	mutex_lock(&priv->lock);
	/* set the PHY register address */
	//nolock_regb_write(priv, MIREGADR, address);
	/* write the PHY data */
	//nolock_regw_write(priv, MIWRL, data);
	/* wait until the PHY write completes and return */
	ret = wait_phy_ready(priv);
	mutex_unlock(&priv->lock);

	return ret;
}

/*
 * Debug routine to dump useful register contents
 */
static void mc13224_dump_regs(struct mc13224_net *priv, const char *msg)
{
	mutex_lock(&priv->lock);
	/* dump state */
	mutex_unlock(&priv->lock);
}

/*
 * ERXRDPT need to be set always at odd addresses, refer to errata datasheet
 */
static u16 erxrdpt_workaround(u16 next_packet_ptr, u16 start, u16 end)
{
	u16 erxrdpt;

	if ((next_packet_ptr - 1 < start) || (next_packet_ptr - 1 > end))
		erxrdpt = end;
	else
		erxrdpt = next_packet_ptr - 1;

	return erxrdpt;
}

/*
 * Calculate wrap around when reading beyond the end of the RX buffer
 */
static u16 rx_packet_start(u16 ptr)
{
	if (ptr + RSV_SIZE > RXEND_INIT)
		return (ptr + RSV_SIZE) - (RXEND_INIT - RXSTART_INIT + 1);
	else
		return ptr + RSV_SIZE;
}

static void nolock_rxfifo_init(struct mc13224_net *priv, u16 start, u16 end)
{
	u16 erxrdpt;

	if (start > 0x1FFF || end > 0x1FFF || start > end) {
		if (netif_msg_drv(priv))
			printk(KERN_ERR DRV_NAME ": %s(%d, %d) RXFIFO "
				"bad parameters!\n", __func__, start, end);
		return;
	}
	/* set receive buffer start + end */
	priv->next_pk_ptr = start;
	//nolock_regw_write(priv, ERXSTL, start);
	erxrdpt = erxrdpt_workaround(priv->next_pk_ptr, start, end);
	//nolock_regw_write(priv, ERXRDPTL, erxrdpt);
	//nolock_regw_write(priv, ERXNDL, end);
}

static void nolock_txfifo_init(struct mc13224_net *priv, u16 start, u16 end)
{
	if (start > 0x1FFF || end > 0x1FFF || start > end) {
		if (netif_msg_drv(priv))
			printk(KERN_ERR DRV_NAME ": %s(%d, %d) TXFIFO "
				"bad parameters!\n", __func__, start, end);
		return;
	}
	/* set transmit buffer start + end */
	//nolock_regw_write(priv, ETXSTL, start);
	//nolock_regw_write(priv, ETXNDL, end);
}

static int mc13224_hw_init(struct mc13224_net *priv)
{
	mutex_lock(&priv->lock);
	/* first reset the chip */
	mutex_unlock(&priv->lock);

	if (netif_msg_hw(priv))
		mc13224_dump_regs(priv, "Hw initialized.");

	return 1;
}

static void mc13224_hw_enable(struct mc13224_net *priv)
{
	/* enable interrupts */
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enabling interrupts.\n",
			__func__);

	mc13224_phy_write(priv, PHIE, PHIE_PGEIE | PHIE_PLNKIE);

	mutex_lock(&priv->lock);
	//nolock_reg_bfclr(priv, EIR, EIR_DMAIF | EIR_LINKIF | EIR_TXIF | EIR_TXERIF | EIR_RXERIF | EIR_PKTIF);
	//nolock_regb_write(priv, EIE, EIE_INTIE | EIE_PKTIE | EIE_LINKIE | EIE_TXIE | EIE_TXERIE | EIE_RXERIE);

	/* enable receive logic */
	//nolock_reg_bfset(priv, ECON1, ECON1_RXEN);
	priv->hw_enable = true;
	mutex_unlock(&priv->lock);
}

static void mc13224_hw_disable(struct mc13224_net *priv)
{
	mutex_lock(&priv->lock);
	/* disable interrutps and packet reception */
	//nolock_regb_write(priv, EIE, 0x00);
	//nolock_reg_bfclr(priv, ECON1, ECON1_RXEN);
	priv->hw_enable = false;
	mutex_unlock(&priv->lock);
}

/*
 * Read the Transmit Status Vector
 */
static void mc13224_read_tsv(struct mc13224_net *priv, u8 tsv[TSV_SIZE])
{
	int endptr;

	//endptr = locked_regw_read(priv, ETXNDL);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": reading TSV at addr:0x%04x\n",
			 endptr + 1);
	mc13224_mem_read(priv, endptr + 1, sizeof(tsv), tsv);
}

static void mc13224_dump_tsv(struct mc13224_net *priv, const char *msg,
				u8 tsv[TSV_SIZE])
{
	u16 tmp1, tmp2;

	printk(KERN_DEBUG DRV_NAME ": %s - TSV:\n", msg);
	tmp1 = tsv[1];
	tmp1 <<= 8;
	tmp1 |= tsv[0];

	tmp2 = tsv[5];
	tmp2 <<= 8;
	tmp2 |= tsv[4];

	printk(KERN_DEBUG DRV_NAME ": ByteCount: %d, CollisionCount: %d,"
		" TotByteOnWire: %d\n", tmp1, tsv[2] & 0x0f, tmp2);
	printk(KERN_DEBUG DRV_NAME ": TxDone: %d, CRCErr:%d, LenChkErr: %d,"
		" LenOutOfRange: %d\n", TSV_GETBIT(tsv, TSV_TXDONE),
		TSV_GETBIT(tsv, TSV_TXCRCERROR),
		TSV_GETBIT(tsv, TSV_TXLENCHKERROR),
		TSV_GETBIT(tsv, TSV_TXLENOUTOFRANGE));
	printk(KERN_DEBUG DRV_NAME ": Multicast: %d, Broadcast: %d, "
		"PacketDefer: %d, ExDefer: %d\n",
		TSV_GETBIT(tsv, TSV_TXMULTICAST),
		TSV_GETBIT(tsv, TSV_TXBROADCAST),
		TSV_GETBIT(tsv, TSV_TXPACKETDEFER),
		TSV_GETBIT(tsv, TSV_TXEXDEFER));
	printk(KERN_DEBUG DRV_NAME ": ExCollision: %d, LateCollision: %d, "
		 "Giant: %d, Underrun: %d\n",
		 TSV_GETBIT(tsv, TSV_TXEXCOLLISION),
		 TSV_GETBIT(tsv, TSV_TXLATECOLLISION),
		 TSV_GETBIT(tsv, TSV_TXGIANT), TSV_GETBIT(tsv, TSV_TXUNDERRUN));
	printk(KERN_DEBUG DRV_NAME ": ControlFrame: %d, PauseFrame: %d, "
		 "BackPressApp: %d, VLanTagFrame: %d\n",
		 TSV_GETBIT(tsv, TSV_TXCONTROLFRAME),
		 TSV_GETBIT(tsv, TSV_TXPAUSEFRAME),
		 TSV_GETBIT(tsv, TSV_BACKPRESSUREAPP),
		 TSV_GETBIT(tsv, TSV_TXVLANTAGFRAME));
}

/*
 * Receive Status vector
 */
static void mc13224_dump_rsv(struct mc13224_net *priv, const char *msg,
			      u16 pk_ptr, int len, u16 sts)
{
	printk(KERN_DEBUG DRV_NAME ": %s - NextPk: 0x%04x - RSV:\n",
		msg, pk_ptr);
	printk(KERN_DEBUG DRV_NAME ": ByteCount: %d, DribbleNibble: %d\n", len,
		 RSV_GETBIT(sts, RSV_DRIBBLENIBBLE));
	printk(KERN_DEBUG DRV_NAME ": RxOK: %d, CRCErr:%d, LenChkErr: %d,"
		 " LenOutOfRange: %d\n", RSV_GETBIT(sts, RSV_RXOK),
		 RSV_GETBIT(sts, RSV_CRCERROR),
		 RSV_GETBIT(sts, RSV_LENCHECKERR),
		 RSV_GETBIT(sts, RSV_LENOUTOFRANGE));
	printk(KERN_DEBUG DRV_NAME ": Multicast: %d, Broadcast: %d, "
		 "LongDropEvent: %d, CarrierEvent: %d\n",
		 RSV_GETBIT(sts, RSV_RXMULTICAST),
		 RSV_GETBIT(sts, RSV_RXBROADCAST),
		 RSV_GETBIT(sts, RSV_RXLONGEVDROPEV),
		 RSV_GETBIT(sts, RSV_CARRIEREV));
	printk(KERN_DEBUG DRV_NAME ": ControlFrame: %d, PauseFrame: %d,"
		 " UnknownOp: %d, VLanTagFrame: %d\n",
		 RSV_GETBIT(sts, RSV_RXCONTROLFRAME),
		 RSV_GETBIT(sts, RSV_RXPAUSEFRAME),
		 RSV_GETBIT(sts, RSV_RXUNKNOWNOPCODE),
		 RSV_GETBIT(sts, RSV_RXTYPEVLAN));
}

static void dump_packet(const char *msg, int len, const char *data)
{
	printk(KERN_DEBUG DRV_NAME ": %s - packet len:%d\n", msg, len);
	print_hex_dump(KERN_DEBUG "", "pk data: ", DUMP_PREFIX_OFFSET, 16, 1,
			data, len, true);
}

/*
 * Hardware receive function.
 * Read the buffer memory, update the FIFO pointer to free the buffer,
 * check the status vector and decrement the packet counter.
 */
static void mc13224_hw_rx(struct net_device *ndev)
{
	struct mc13224_net *priv = netdev_priv(ndev);
	struct sk_buff *skb = NULL;
	u16 erxrdpt, next_packet, rxstat;
	u8 rsv[RSV_SIZE];
	int len;

	if (netif_msg_rx_status(priv))
		printk(KERN_DEBUG DRV_NAME ": RX pk_addr:0x%04x\n",
			priv->next_pk_ptr);

	if (unlikely(priv->next_pk_ptr > RXEND_INIT)) {
		if (netif_msg_rx_err(priv))
			dev_err(&ndev->dev,
				"%s() Invalid packet address!! 0x%04x\n",
				__func__, priv->next_pk_ptr);
		/* packet address corrupted: reset RX logic */
		mutex_lock(&priv->lock);
		//nolock_reg_bfclr(priv, ECON1, ECON1_RXEN);
		//nolock_reg_bfset(priv, ECON1, ECON1_RXRST);
		//nolock_reg_bfclr(priv, ECON1, ECON1_RXRST);
		nolock_rxfifo_init(priv, RXSTART_INIT, RXEND_INIT);
		//nolock_reg_bfclr(priv, EIR, EIR_RXERIF);
		//nolock_reg_bfset(priv, ECON1, ECON1_RXEN);
		mutex_unlock(&priv->lock);
		ndev->stats.rx_errors++;
		return;
	}
	/* Read next packet pointer and rx status vector */
	mc13224_mem_read(priv, priv->next_pk_ptr, sizeof(rsv), rsv);

	next_packet = rsv[1];
	next_packet <<= 8;
	next_packet |= rsv[0];

	len = rsv[3];
	len <<= 8;
	len |= rsv[2];

	rxstat = rsv[5];
	rxstat <<= 8;
	rxstat |= rsv[4];

	if (netif_msg_rx_status(priv))
		mc13224_dump_rsv(priv, __func__, next_packet, len, rxstat);

	if (!RSV_GETBIT(rxstat, RSV_RXOK) || len > MAX_FRAMELEN) {
		if (netif_msg_rx_err(priv))
			dev_err(&ndev->dev, "Rx Error (%04x)\n", rxstat);
		ndev->stats.rx_errors++;
		if (RSV_GETBIT(rxstat, RSV_CRCERROR))
			ndev->stats.rx_crc_errors++;
		if (RSV_GETBIT(rxstat, RSV_LENCHECKERR))
			ndev->stats.rx_frame_errors++;
		if (len > MAX_FRAMELEN)
			ndev->stats.rx_over_errors++;
	} else {
		skb = dev_alloc_skb(len + NET_IP_ALIGN);
		if (!skb) {
			if (netif_msg_rx_err(priv))
				dev_err(&ndev->dev,
					"out of memory for Rx'd frame\n");
			ndev->stats.rx_dropped++;
		} else {
			skb->dev = ndev;
			skb_reserve(skb, NET_IP_ALIGN);
			/* copy the packet from the receive buffer */
			mc13224_mem_read(priv,
				rx_packet_start(priv->next_pk_ptr),
				len, skb_put(skb, len));
			if (netif_msg_pktdata(priv))
				dump_packet(__func__, skb->len, skb->data);
			skb->protocol = eth_type_trans(skb, ndev);
			/* update statistics */
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
			netif_rx_ni(skb);
		}
	}
	/*
	 * Move the RX read pointer to the start of the next
	 * received packet.
	 * This frees the memory we just read out
	 */
	erxrdpt = erxrdpt_workaround(next_packet, RXSTART_INIT, RXEND_INIT);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() ERXRDPT:0x%04x\n",
			__func__, erxrdpt);

	mutex_lock(&priv->lock);
	//nolock_regw_write(priv, ERXRDPTL, erxrdpt);
#ifdef CONFIG_MC13224_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u16 reg;
		//reg = nolock_regw_read(priv, ERXRDPTL);
		if (reg != erxrdpt)
			printk(KERN_DEBUG DRV_NAME ": %s() ERXRDPT verify "
				"error (0x%04x - 0x%04x)\n", __func__,
				reg, erxrdpt);
	}
#endif
	priv->next_pk_ptr = next_packet;
	/* we are done with this packet, decrement the packet counter */
	//nolock_reg_bfset(priv, ECON2, ECON2_PKTDEC);
	mutex_unlock(&priv->lock);
}

/*
 * Calculate free space in RxFIFO
 */
static int mc13224_get_free_rxfifo(struct mc13224_net *priv)
{
	int epkcnt, erxst, erxnd, erxwr, erxrd;
	int free_space;

	mutex_lock(&priv->lock);
	//epkcnt = nolock_regb_read(priv, EPKTCNT);
	if (epkcnt >= 255)
		free_space = -1;
	else {
		//erxst = nolock_regw_read(priv, ERXSTL);
		//erxnd = nolock_regw_read(priv, ERXNDL);
		//erxwr = nolock_regw_read(priv, ERXWRPTL);
		//erxrd = nolock_regw_read(priv, ERXRDPTL);

		if (erxwr > erxrd)
			free_space = (erxnd - erxst) - (erxwr - erxrd);
		else if (erxwr == erxrd)
			free_space = (erxnd - erxst);
		else
			free_space = erxrd - erxwr - 1;
	}
	mutex_unlock(&priv->lock);
	if (netif_msg_rx_status(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() free_space = %d\n",
			__func__, free_space);
	return free_space;
}

static void mc13224_tx_clear(struct net_device *ndev, bool err)
{
	struct mc13224_net *priv = netdev_priv(ndev);

	if (err)
		ndev->stats.tx_errors++;
	else
		ndev->stats.tx_packets++;

	if (priv->tx_skb) {
		if (!err)
			ndev->stats.tx_bytes += priv->tx_skb->len;
		dev_kfree_skb(priv->tx_skb);
		priv->tx_skb = NULL;
	}
	//locked_reg_bfclr(priv, ECON1, ECON1_TXRTS);
	netif_wake_queue(ndev);
}

/*
 * RX handler
 * ignore PKTIF because is unreliable! (look at the errata datasheet)
 * check EPKTCNT is the suggested workaround.
 * We don't need to clear interrupt flag, automatically done when
 * mc13224_hw_rx() decrements the packet counter.
 * Returns how many packet processed.
 */
static int mc13224_rx_interrupt(struct net_device *ndev)
{
	struct mc13224_net *priv = netdev_priv(ndev);
	int pk_counter, ret;

	//pk_counter = locked_regb_read(priv, EPKTCNT);
	pk_counter = 0;
	if (pk_counter && netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": intRX, pk_cnt: %d\n", pk_counter);
	if (pk_counter > priv->max_pk_counter) {
		/* update statistics */
		priv->max_pk_counter = pk_counter;
		if (netif_msg_rx_status(priv) && priv->max_pk_counter > 1)
			printk(KERN_DEBUG DRV_NAME ": RX max_pk_cnt: %d\n",
				priv->max_pk_counter);
	}
	ret = pk_counter;
	while (pk_counter-- > 0)
		mc13224_hw_rx(ndev);

	return ret;
}

static void mc13224_irq_work_handler(struct work_struct *work)
{
	struct mc13224_net *priv =
		container_of(work, struct mc13224_net, irq_work);
	struct net_device *ndev = priv->netdev;
	int intflags, loop;

	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);
	/* disable further interrupts */
	//locked_reg_bfclr(priv, EIE, EIE_INTIE);

	do {
		loop = 0;
		//intflags = locked_regb_read(priv, EIR);
		/* DMA interrupt handler (not currently used) */
		intflags = 0;
		if ((intflags & EIR_DMAIF) != 0) {
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intDMA(%d)\n", loop);
			//locked_reg_bfclr(priv, EIR, EIR_DMAIF);
		}
		/* LINK changed handler */
		if ((intflags & EIR_LINKIF) != 0) {
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intLINK(%d)\n", loop);
			//mc13224_check_link_status(ndev);
			/* read PHIR to clear the flag */
			mc13224_phy_read(priv, PHIR);
		}
		/* TX complete handler */
		if ((intflags & EIR_TXIF) != 0) {
			bool err = false;
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intTX(%d)\n", loop);
			priv->tx_retry_count = 0;
#if 0
			if (locked_regb_read(priv, ESTAT) & ESTAT_TXABRT) {
				if (netif_msg_tx_err(priv))
					dev_err(&ndev->dev,
						"Tx Error (aborted)\n");
				err = true;
			}
#endif
			if (netif_msg_tx_done(priv)) {
				u8 tsv[TSV_SIZE];
				mc13224_read_tsv(priv, tsv);
				mc13224_dump_tsv(priv, "Tx Done", tsv);
			}
			mc13224_tx_clear(ndev, err);
			//locked_reg_bfclr(priv, EIR, EIR_TXIF);
		}
		/* TX Error handler */
		if ((intflags & EIR_TXERIF) != 0) {
			u8 tsv[TSV_SIZE];

			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intTXErr(%d)\n", loop);
			//locked_reg_bfclr(priv, ECON1, ECON1_TXRTS);
			mc13224_read_tsv(priv, tsv);
			if (netif_msg_tx_err(priv))
				mc13224_dump_tsv(priv, "Tx Error", tsv);
			/* Reset TX logic */
			mutex_lock(&priv->lock);
			//nolock_reg_bfset(priv, ECON1, ECON1_TXRST);
			//nolock_reg_bfclr(priv, ECON1, ECON1_TXRST);
			//nolock_txfifo_init(priv, TXSTART_INIT, TXEND_INIT);
			mutex_unlock(&priv->lock);
			/* Transmit Late collision check for retransmit */
			if (TSV_GETBIT(tsv, TSV_TXLATECOLLISION)) {
				if (netif_msg_tx_err(priv))
					printk(KERN_DEBUG DRV_NAME
						": LateCollision TXErr (%d)\n",
						priv->tx_retry_count);
				if (priv->tx_retry_count++ < MAX_TX_RETRYCOUNT) {
					//locked_reg_bfset(priv, ECON1, ECON1_TXRTS);
				} else
					mc13224_tx_clear(ndev, true);
			} else
				mc13224_tx_clear(ndev, true);
			//locked_reg_bfclr(priv, EIR, EIR_TXERIF);
		}
		/* RX Error handler */
		if ((intflags & EIR_RXERIF) != 0) {
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intRXErr(%d)\n", loop);
			/* Check free FIFO space to flag RX overrun */
			if (mc13224_get_free_rxfifo(priv) <= 0) {
				if (netif_msg_rx_err(priv))
					printk(KERN_DEBUG DRV_NAME
						": RX Overrun\n");
				ndev->stats.rx_dropped++;
			}
			//locked_reg_bfclr(priv, EIR, EIR_RXERIF);
		}
		/* RX handler */
		if (mc13224_rx_interrupt(ndev))
			loop++;
	} while (loop);

	/* re-enable interrupts */
	//locked_reg_bfset(priv, EIE, EIE_INTIE);
	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() exit\n", __func__);
}

/*
 * Hardware transmit function.
 * Fill the buffer memory and send the contents of the transmit buffer
 * onto the network
 */
static void mc13224_hw_tx(struct mc13224_net *priv)
{
	if (netif_msg_tx_queued(priv))
		printk(KERN_DEBUG DRV_NAME
			": Tx Packet Len:%d\n", priv->tx_skb->len);

	if (netif_msg_pktdata(priv))
		dump_packet(__func__,
			    priv->tx_skb->len, priv->tx_skb->data);
	mc13224_packet_write(priv, priv->tx_skb->len, priv->tx_skb->data);

#if 0
	/* readback and verify written data */
	if (netif_msg_drv(priv)) {
		int test_len, k;
		u8 test_buf[64]; /* limit the test to the first 64 bytes */
		int okflag;

		test_len = priv->tx_skb->len;
		if (test_len > sizeof(test_buf))
			test_len = sizeof(test_buf);

		/* + 1 to skip control byte */
		mc13224_mem_read(priv, TXSTART_INIT + 1, test_len, test_buf);
		okflag = 1;
		for (k = 0; k < test_len; k++) {
			if (priv->tx_skb->data[k] != test_buf[k]) {
				printk(KERN_DEBUG DRV_NAME
					 ": Error, %d location differ: "
					 "0x%02x-0x%02x\n", k,
					 priv->tx_skb->data[k], test_buf[k]);
				okflag = 0;
			}
		}
		if (!okflag)
			printk(KERN_DEBUG DRV_NAME ": Tx write buffer, "
				"verify ERROR!\n");
	}
#endif
	/* set TX request flag */
	//locked_reg_bfset(priv, ECON1, ECON1_TXRTS);
}

static netdev_tx_t mc13224_send_packet(struct sk_buff *skb,
					struct net_device *dev)
{
	struct mc13224_net *priv = netdev_priv(dev);

	if (netif_msg_tx_queued(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	/* If some error occurs while trying to transmit this
	 * packet, you should return '1' from this function.
	 * In such a case you _may not_ do anything to the
	 * SKB, it is still owned by the network queueing
	 * layer when an error is returned.  This means you
	 * may not modify any SKB fields, you may not free
	 * the SKB, etc.
	 */
	netif_stop_queue(dev);

	/* save the timestamp */
	priv->netdev->trans_start = jiffies;
	/* Remember the skb for deferred processing */
	priv->tx_skb = skb;
	schedule_work(&priv->tx_work);

	return NETDEV_TX_OK;
}

static void mc13224_tx_work_handler(struct work_struct *work)
{
	struct mc13224_net *priv =
		container_of(work, struct mc13224_net, tx_work);

	/* actual delivery of data */
	mc13224_hw_tx(priv);
}

static irqreturn_t mc13224_irq(int irq, void *dev_id)
{
	struct mc13224_net *priv = dev_id;

	/*
	 * Can't do anything in interrupt context because we need to
	 * block (spi_sync() is blocking) so fire of the interrupt
	 * handling workqueue.
	 * Remember that we access mc13224 registers through SPI bus
	 * via spi_sync() call.
	 */

	printk("Attention\n");
	//gpio_set_value(priv->pdata->gpio_cs, 0);

	schedule_work(&priv->irq_work);

	return IRQ_HANDLED;
}

static void mc13224_tx_timeout(struct net_device *ndev)
{
	struct mc13224_net *priv = netdev_priv(ndev);

	if (netif_msg_timer(priv))
		dev_err(&ndev->dev, DRV_NAME " tx timeout\n");

	ndev->stats.tx_errors++;
	/* can't restart safely under softirq */
	schedule_work(&priv->restart_work);
}

/*
 * Open/initialize the board. This is called (in the current kernel)
 * sometime after booting when the 'ifconfig' program is run.
 *
 * This routine should set everything up anew at each open, even
 * registers that "should" only need to be set once at boot, so that
 * there is non-reboot way to recover if something goes wrong.
 */
static int mc13224_net_open(struct net_device *dev)
{
	struct mc13224_net *priv = netdev_priv(dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	/* Reset the hardware here (and take it out of low power mode) */
	mc13224_hw_disable(priv);
	if (!mc13224_hw_init(priv)) {
		if (netif_msg_ifup(priv))
			dev_err(&dev->dev, "hw_reset() failed\n");
		return -EINVAL;
	}
	/* Enable interrupts */
	mc13224_hw_enable(priv);

	/* We are now ready to accept transmit requests from
	 * the queueing layer of the networking.
	 */
	netif_start_queue(dev);

	return 0;
}

/* The inverse routine to net_open(). */
static int mc13224_net_close(struct net_device *dev)
{
	struct mc13224_net *priv = netdev_priv(dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	mc13224_hw_disable(priv);
	netif_stop_queue(dev);

	return 0;
}

static void mc13224_restart_work_handler(struct work_struct *work)
{
	struct mc13224_net *priv =
			container_of(work, struct mc13224_net, restart_work);
	struct net_device *ndev = priv->netdev;
	int ret;

	rtnl_lock();
	if (netif_running(ndev)) {
		mc13224_net_close(ndev);
		ret = mc13224_net_open(ndev);
		if (unlikely(ret)) {
			dev_info(&ndev->dev, " could not restart %d\n", ret);
			dev_close(ndev);
		}
	}
	rtnl_unlock();
}

static int mc13224_chipset_init(struct net_device *dev)
{
	struct mc13224_net *priv = netdev_priv(dev);

	return mc13224_hw_init(priv);
}

static int mc13224_net_init(struct net_device *dev)
{
	dev->mtu		= IPV6_MIN_MTU;
	dev->type		= ARPHRD_SLIP;
//	dev->watchdog_timeo	= 20*HZ;
	dev->watchdog_timeo	= 3*HZ;
	return 0;
}

static const struct net_device_ops mc13224_netdev_ops = {
	.ndo_init		= mc13224_net_init,
	.ndo_open		= mc13224_net_open,
	.ndo_stop		= mc13224_net_close,
	.ndo_start_xmit		= mc13224_send_packet,
	.ndo_tx_timeout		= mc13224_tx_timeout,
	.ndo_change_mtu		= eth_change_mtu,
};

static irqreturn_t ea313x_mc13224_ready(int irq, void *dev_id)
{
	struct mc13224_net *priv = dev_id;

	if (gpio_get_value(priv->pdata->gpio_ready)) {
		/* select the opposite level senstivity */
		set_irq_type(priv->pdata->irq_ready, IRQ_TYPE_LEVEL_LOW);
		printk("Ready\n");
	} else {
		/* select the opposite level senstivity */
		set_irq_type(priv->pdata->irq_ready, IRQ_TYPE_LEVEL_HIGH);
		printk("Ok to xmit\n");
		//gpio_set_value(priv->pdata->gpio_cs, 1);
	}
	return IRQ_HANDLED;
}

/* Hook the destructor so we can free slip devices at the right point in time */
static void mc13224_free_netdev(struct net_device *dev)
{
//	int i = dev->base_addr;
	free_netdev(dev);
//	slip_devs[i] = NULL;
}

static void mc13224_setup(struct net_device *dev)
{
	dev->netdev_ops		= &mc13224_netdev_ops;
	dev->destructor		= mc13224_free_netdev;

	dev->hard_header_len	= 0;
	dev->addr_len		= IEEE802154_ADDR_LEN;
	dev->tx_queue_len	= 10;

	/* New-style flags. */
	dev->flags		= IFF_NOARP|IFF_POINTOPOINT|IFF_MULTICAST;
}

static int __devinit mc13224_probe(struct spi_device *spi)
{
	struct net_device *dev;
	struct mc13224_net *priv;
	int level;
	int ret = 0;

	if (netif_msg_drv(&debug))
		dev_info(&spi->dev, DRV_NAME " 802.15.4 driver %s loaded\n",
			DRV_VERSION);

	dev = alloc_netdev(sizeof(struct mc13224_net), DRV_NAME, mc13224_setup);
	if (!dev) {
		if (netif_msg_drv(&debug))
			dev_err(&spi->dev, DRV_NAME
				": unable to alloc new netdev\n");
		ret = -ENOMEM;
		goto error_alloc;
	}
	priv = netdev_priv(dev);

	priv->netdev = dev;	/* priv to netdev reference */
	priv->spi = spi;	/* priv to spi reference */
	priv->pdata = spi->dev.platform_data;
	priv->msg_enable = netif_msg_init(debug.msg_enable,
						MC13224_MSG_DEFAULT);
	mutex_init(&priv->lock);
	INIT_WORK(&priv->tx_work, mc13224_tx_work_handler);
	INIT_WORK(&priv->irq_work, mc13224_irq_work_handler);
	INIT_WORK(&priv->restart_work, mc13224_restart_work_handler);
	dev_set_drvdata(&spi->dev, priv);	/* spi to priv reference */
	SET_NETDEV_DEV(dev, &spi->dev);

	if (!mc13224_chipset_init(dev)) {
		if (netif_msg_probe(priv))
			dev_info(&spi->dev, DRV_NAME " chip not found\n");
		ret = -EIO;
		goto error_irq;
	}

	gpio_request(priv->pdata->gpio_attention, DRV_NAME " attention");
	gpio_direction_input(priv->pdata->gpio_attention);

	ret = request_irq(spi->irq, mc13224_irq, IRQ_TYPE_EDGE_RISING, DRV_NAME " attention", priv);
	if (ret < 0) {
		if (netif_msg_probe(priv))
			dev_err(&spi->dev, DRV_NAME ": request irq %d failed "
				"(ret = %d)\n", spi->irq, ret);
		goto error_irq;
	}

	gpio_request(priv->pdata->gpio_ready, DRV_NAME " ready");
	gpio_direction_input(priv->pdata->gpio_ready);
	level = gpio_get_value(priv->pdata->gpio_ready) ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH;	

	ret = request_irq(priv->pdata->irq_ready,
			ea313x_mc13224_ready,
			level,
			DRV_NAME " ready", 
			priv);

	gpio_request(priv->pdata->gpio_cs, DRV_NAME " CS");
	gpio_direction_output(priv->pdata->gpio_cs, 1);

	//dev->if_port = IF_PORT_10BASET;
	dev->irq = spi->irq;
	dev->netdev_ops = &mc13224_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;

	ret = register_netdev(dev);
	if (ret) {
		if (netif_msg_probe(priv))
			dev_err(&spi->dev, "register netdev " DRV_NAME
				" failed (ret = %d)\n", ret);
		goto error_register;
	}
	dev_info(&dev->dev, DRV_NAME " driver registered\n");

	return 0;

error_register:
	free_irq(spi->irq, priv);
error_irq:
	free_netdev(dev);
error_alloc:
	return ret;
}

static int __devexit mc13224_remove(struct spi_device *spi)
{
	struct mc13224_net *priv = dev_get_drvdata(&spi->dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": remove\n");

	unregister_netdev(priv->netdev);
	free_irq(spi->irq, priv);
	free_netdev(priv->netdev);

	return 0;
}

static struct spi_driver mc13224_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
	 },
	.probe = mc13224_probe,
	.remove = __devexit_p(mc13224_remove),
};

static int __init mc13224_init(void)
{
	msec20_to_jiffies = msecs_to_jiffies(20);

	return spi_register_driver(&mc13224_driver);
}

module_init(mc13224_init);

static void __exit mc13224_exit(void)
{
	spi_unregister_driver(&mc13224_driver);
}

module_exit(mc13224_exit);

MODULE_DESCRIPTION(DRV_NAME " 802.15.4 driver");
MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_LICENSE("GPL");
module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., ffff=all)");
MODULE_ALIAS("spi:" DRV_NAME);
