/*
 * LPC313x MultiMedia Card Interface driver
 *
 * drivers/mmc/host/lpc313x_mmc.c
 *
 * Copyright (C) 2009 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include "lpc313x_mmc.h"
#include <mach/irqs.h>
#include <linux/mmc/host.h>
#include <mach/board.h>
/* for time being use arch specific DMA framework instead of generic framework */
#include <mach/dma.h>

#define USE_DMA
#define BURST_DMA

#define LPC313x_MCI_DATA_ERROR_FLAGS	(SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_HTO | SDMMC_INT_SBE | SDMMC_INT_EBE)
#define LPC313x_MCI_CMD_ERROR_FLAGS	(SDMMC_INT_RTO | SDMMC_INT_RCRC | SDMMC_INT_RESP_ERR | SDMMC_INT_HLE)
#define LPC313x_MCI_ERROR_FLAGS		(LPC313x_MCI_DATA_ERROR_FLAGS | LPC313x_MCI_CMD_ERROR_FLAGS | SDMMC_INT_HLE)
#define LPC313x_MCI_SEND_STATUS		1
#define LPC313x_MCI_RECV_STATUS		2
#define LPC313x_MCI_DMA_THRESHOLD	16

enum {
	EVENT_CMD_COMPLETE = 0,
	EVENT_XFER_COMPLETE,
	EVENT_DATA_COMPLETE,
	EVENT_DATA_ERROR,
	EVENT_XFER_ERROR
};


enum lpc313x_mci_state {
	STATE_IDLE = 0,
	STATE_SENDING_CMD,
	STATE_SENDING_DATA,
	STATE_DATA_BUSY,
	STATE_SENDING_STOP,
	STATE_DATA_ERROR,
};
/*forward declaration */
struct lpc313x_mci_slot;

struct lpc313x_mci {
	spinlock_t		lock;
	void __iomem		*regs;

	struct scatterlist	*sg;
	unsigned int		pio_offset;

	struct lpc313x_mci_slot	*cur_slot;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;

#ifdef USE_DMA
	int			dma_chn;
	dma_addr_t		sg_dma;
	dma_sg_ll_t		*sg_cpu;
#endif
	u32			cmd_status;
	u32			data_status;
	u32			stop_cmdr;
	u32			dir_status;
	struct tasklet_struct	tasklet;
	unsigned long		pending_events;
	unsigned long		completed_events;
	enum lpc313x_mci_state	state;
	struct list_head	queue;

	u32			bus_hz;
	u32			current_speed;
	struct platform_device	*pdev;
	struct lpc313x_mci_board *pdata;
	struct lpc313x_mci_slot	*slot[MAX_MCI_SLOTS];
};

struct lpc313x_mci_slot {
	struct mmc_host		*mmc;
	struct lpc313x_mci	*host;

	u32			ctype;

	struct mmc_request	*mrq;
	struct list_head	queue_node;

	unsigned int		clock;
	unsigned long		flags;
#define LPC313x_MMC_CARD_PRESENT	0
#define LPC313x_MMC_CARD_NEED_INIT	1
#define LPC313x_MMC_SHUTDOWN		2
	int			id;
	int			irq;

	struct timer_list	detect_timer;
};

#define lpc313x_mci_test_and_clear_pending(host, event)		\
	test_and_clear_bit(event, &host->pending_events)
#define lpc313x_mci_set_completed(host, event)			\
	set_bit(event, &host->completed_events)

#define lpc313x_mci_set_pending(host, event)				\
	set_bit(event, &host->pending_events)

#if defined (CONFIG_DEBUG_FS)
/*
 * The debugfs stuff below is mostly optimized away when
 * CONFIG_DEBUG_FS is not set.
 */
static int lpc313x_mci_req_show(struct seq_file *s, void *v)
{
	struct lpc313x_mci_slot	*slot = s->private;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_command	*stop;
	struct mmc_data		*data;

	/* Make sure we get a consistent snapshot */
	spin_lock(&slot->host->lock);
	mrq = slot->mrq;

	if (mrq) {
		cmd = mrq->cmd;
		data = mrq->data;
		stop = mrq->stop;

		if (cmd)
			seq_printf(s,
				"CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				cmd->opcode, cmd->arg, cmd->flags,
				cmd->resp[0], cmd->resp[1], cmd->resp[2],
				cmd->resp[2], cmd->error);
		if (data)
			seq_printf(s, "DATA %u / %u * %u flg %x err %d\n",
				data->bytes_xfered, data->blocks,
				data->blksz, data->flags, data->error);
		if (stop)
			seq_printf(s,
				"CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				stop->opcode, stop->arg, stop->flags,
				stop->resp[0], stop->resp[1], stop->resp[2],
				stop->resp[2], stop->error);
	}

	spin_unlock(&slot->host->lock);

	return 0;
}

static int lpc313x_mci_req_open(struct inode *inode, struct file *file)
{
	return single_open(file, lpc313x_mci_req_show, inode->i_private);
}

static const struct file_operations lpc313x_mci_req_fops = {
	.owner		= THIS_MODULE,
	.open		= lpc313x_mci_req_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int lpc313x_mci_regs_show(struct seq_file *s, void *v)
{
	struct lpc313x_mci	*host = s->private;

	seq_printf(s, "STATUS:\t0x%08x\n",SDMMC_STATUS);
	seq_printf(s, "RINTSTS:\t0x%08x\n",SDMMC_RINTSTS);
	seq_printf(s, "CMD:\t0x%08x\n", SDMMC_CMD);
	seq_printf(s, "CTRL:\t0x%08x\n", SDMMC_CTRL);
	seq_printf(s, "INTMASK:\t0x%08x\n", SDMMC_INTMASK);
	seq_printf(s, "CLKENA:\t0x%08x\n", SDMMC_CLKENA);

	return 0;
}

static int lpc313x_mci_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lpc313x_mci_regs_show, inode->i_private);
}

static const struct file_operations lpc313x_mci_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= lpc313x_mci_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void lpc313x_mci_init_debugfs(struct lpc313x_mci_slot *slot)
{
	struct mmc_host		*mmc = slot->mmc;
	struct lpc313x_mci	*host = slot->host;
	struct dentry		*root;
	struct dentry		*node;

	root = mmc->debugfs_root;
	if (!root)
		return;

	node = debugfs_create_file("regs", S_IRUSR, root, host,
			&lpc313x_mci_regs_fops);
	if (IS_ERR(node))
		return;
	if (!node)
		goto err;

	node = debugfs_create_file("req", S_IRUSR, root, slot, &lpc313x_mci_req_fops);
	if (!node)
		goto err;

	node = debugfs_create_u32("state", S_IRUSR, root, (u32 *)&host->state);
	if (!node)
		goto err;

	node = debugfs_create_x32("pending_events", S_IRUSR, root,
				     (u32 *)&host->pending_events);
	if (!node)
		goto err;

	node = debugfs_create_x32("completed_events", S_IRUSR, root,
				     (u32 *)&host->completed_events);
	if (!node)
		goto err;

	return;

err:
	dev_err(&mmc->class_dev, "failed to initialize debugfs for slot\n");
}
#endif

static inline unsigned ns_to_clocks(unsigned clkrate, unsigned ns)
{
	u32 clks;
	if (clkrate > 1000000)
		clks =  (ns * (clkrate / 1000000) + 999) / 1000;
	else
		clks =  ((ns/1000) * (clkrate / 1000) + 999) / 1000;

	return clks;
}

static void lpc313x_mci_set_timeout(struct lpc313x_mci *host,
		struct lpc313x_mci_slot *slot, struct mmc_data *data)
{
	unsigned timeout;

	timeout = ns_to_clocks(slot->clock, data->timeout_ns) + data->timeout_clks;

	dev_vdbg(&slot->mmc->class_dev, "tmo req:%d + %d reg:%d clk:%d\n", 
		data->timeout_ns, data->timeout_clks, timeout, slot->clock);
	/* the standard response timeout value (Ncr) is 64 clocks. 
	 * Let give 4 additional clocks for response.
	 */
	mci_writel(TMOUT, /*0xffffffff); */ (timeout << 8) | (70));
}

static u32 lpc313x_mci_prepare_command(struct mmc_host *mmc,
				 struct mmc_command *cmd)
{
	struct mmc_data	*data;
	u32		cmdr;
	
	cmd->error = -EINPROGRESS;
	cmdr = cmd->opcode;

	if(cmdr == 12) 
		cmdr |= SDMMC_CMD_STOP;
	else 
		cmdr |= SDMMC_CMD_PRV_DAT_WAIT;

	if (cmd->flags & MMC_RSP_PRESENT) {
		cmdr |= SDMMC_CMD_RESP_EXP; // expect the respond, need to set this bit
		if (cmd->flags & MMC_RSP_136) 
			cmdr |= SDMMC_CMD_RESP_LONG; // expect long respond
		
		if(cmd->flags & MMC_RSP_CRC) 
			cmdr |= SDMMC_CMD_RESP_CRC;
	}

	data = cmd->data;
	if (data) {
		cmdr |= SDMMC_CMD_DAT_EXP;
		if (data->flags & MMC_DATA_STREAM) 
			cmdr |= SDMMC_CMD_STRM_MODE; //  set stream mode
		if (data->flags & MMC_DATA_WRITE) 
		    cmdr |= SDMMC_CMD_DAT_WR;
		
#if 0 /* Jerry, need to confirm the specification does we need to set this bit if blocks > 1 */
		if(data->blocks > 1) 
		    cmdr |= SDMMC_CMD_SEND_STOP;
		
#endif
	}
	return cmdr;
}


static void lpc313x_mci_start_command(struct lpc313x_mci *host,
		struct mmc_command *cmd, u32 cmd_flags)
{
 	int tmo = 50;
 	host->cmd = cmd;
	dev_vdbg(&host->pdev->dev,
			"start cmd:%d ARGR=0x%08x CMDR=0x%08x\n",
			cmd->opcode, cmd->arg, cmd_flags);
	mci_writel(CMDARG, cmd->arg); // write to CMDARG register
	mci_writel(CMD, cmd_flags | SDMMC_CMD_START); // write to CMD register

	/* wait until CIU accepts the command */
	while (--tmo && (mci_readl(CMD) & SDMMC_CMD_START)) 
		cpu_relax();
}

static void send_stop_cmd(struct lpc313x_mci *host, struct mmc_data *data)
{
	lpc313x_mci_start_command(host, data->stop, host->stop_cmdr);
}


#ifdef USE_DMA

static void lpc313x_mci_dma_cleanup(struct lpc313x_mci *host)
{
	struct mmc_data			*data = host->data;

	if (data) 
		dma_unmap_sg(&host->pdev->dev, data->sg, data->sg_len,
		     ((data->flags & MMC_DATA_WRITE)
		      ? DMA_TO_DEVICE : DMA_FROM_DEVICE));
}

static void lpc313x_mci_stop_dma(struct lpc313x_mci *host)
{
	if (host->dma_chn > 0) {
		dma_stop_channel(host->dma_chn);
		lpc313x_mci_dma_cleanup(host);
	} else {
		/* Data transfer was stopped by the interrupt handler */
		lpc313x_mci_set_pending(host, EVENT_XFER_COMPLETE);
	}
}

/* This function is called by the DMA driver from tasklet context. */
static void lpc313x_mci_dma_complete(int chn, dma_irq_type_t type, void *arg)
{
	struct lpc313x_mci	*host = arg;
	struct mmc_data		*data = host->data;

	dev_vdbg(&host->pdev->dev, "DMA complete\n");

	spin_lock(&host->lock);
	lpc313x_mci_dma_cleanup(host);

	/*
	 * If the card was removed, data will be NULL. No point trying
	 * to send the stop command or waiting for NBUSY in this case.
	 */
	if (data) {
		lpc313x_mci_set_pending(host, EVENT_XFER_COMPLETE);
		tasklet_schedule(&host->tasklet);
	}
	spin_unlock(&host->lock);
}

static int lpc313x_mci_submit_data_dma(struct lpc313x_mci *host, struct mmc_data *data)
{
	struct scatterlist		*sg;
	unsigned int			i, direction, sg_len;
	unsigned int			j, trans_len;

	/* If we don't have a channel, we can't do DMA */
	if (host->dma_chn < 0)
		return -ENODEV;

	/*
	 * We don't do DMA on "complex" transfers, i.e. with
	 * non-word-aligned buffers or lengths. Also, we don't bother
	 * with all the DMA setup overhead for short transfers.
	 */
	if (data->blocks * data->blksz < LPC313x_MCI_DMA_THRESHOLD)
		return -EINVAL;
	if (data->blksz & 3)
		return -EINVAL;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3)
			return -EINVAL;
	}

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	sg_len = dma_map_sg(&host->pdev->dev, data->sg, data->sg_len,
				   direction);

	dev_vdbg(&host->pdev->dev, "sd sg_cpu: 0x%08x sg_dma:0x%08x sg_len:%d \n",
		(u32)host->sg_cpu, (u32)host->sg_dma, sg_len);

	for (i = 0, j = 0; i < sg_len; i++) {
		unsigned int length = sg_dma_len(&data->sg[i]);
		u32 mem_addr = sg_dma_address(&data->sg[i]);

		while (length) {

			host->sg_cpu[j].setup.cfg = DMA_CFG_CMP_CH_EN | DMA_CFG_CMP_CH_NR(host->dma_chn);

			if (data->flags & MMC_DATA_READ) {
				host->sg_cpu[j].setup.src_address = SDMMC_DATA_ADR;
				host->sg_cpu[j].setup.dest_address = mem_addr;
				host->sg_cpu[j].setup.cfg |= DMA_CFG_RD_SLV_NR(DMA_SLV_SDMMC);
			} else {
				host->sg_cpu[j].setup.src_address = mem_addr;
				host->sg_cpu[j].setup.dest_address = SDMMC_DATA_ADR;
				host->sg_cpu[j].setup.cfg |= DMA_CFG_WR_SLV_NR(DMA_SLV_SDMMC);
			}
			host->sg_cpu[j].next_entry = host->sg_dma + (j + 1) *
						sizeof(dma_sg_ll_t);

#ifdef BURST_DMA
			host->sg_cpu[j].setup.cfg |= DMA_CFG_TX_BURST;
      /* 16 bytes per transfer */
			trans_len = (length >> 4) - 1;
#else
      /* 4 bytes per transfer */
			trans_len = (length >> 2) - 1;
#endif

			if (trans_len > DMA_MAX_TRANSFERS) {
				trans_len = DMA_MAX_TRANSFERS;
				length -= (DMA_MAX_TRANSFERS + 1) << 2;
				mem_addr += ((DMA_MAX_TRANSFERS + 1) << 2);
			}
			else {
				length = 0;
			}

			host->sg_cpu[j].setup.trans_length = trans_len;

			dev_vdbg(&host->pdev->dev, "sd src: 0x%08x dest:0x%08x cfg:0x%08x nxt:0x%08x len:%d \n",
				host->sg_cpu[j].setup.src_address, host->sg_cpu[j].setup.dest_address,
				host->sg_cpu[j].setup.cfg, host->sg_cpu[j].next_entry,
				host->sg_cpu[j].setup.trans_length);

			/* move to next transfer descriptor */
			j++;
		}
	}
	host->sg_cpu[j].setup.src_address = host->sg_dma;
	host->sg_cpu[j].setup.dest_address = DMACH_SOFT_INT_PHYS;
	host->sg_cpu[j].setup.trans_length = 1;
	host->sg_cpu[j].setup.cfg = 0;
	// disable irq of RX & TX, let DMA handle it
	//SDMMC_INTMASK &= ~(SDMMC_INT_RXDR | SDMMC_INT_TXDR);
	SDMMC_CTRL |= SDMMC_CTRL_DMA_ENABLE; // enable dma
	dma_prog_sg_channel(host->dma_chn, host->sg_dma);
	wmb();
	/* Go! */
	dma_start_channel(host->dma_chn);

	return 0;
}

#else
static int lpc313x_mci_submit_data_dma(struct lpc313x_mci *host, struct mmc_data *data)
{
	return -ENOSYS;
}

static void lpc313x_mci_stop_dma(struct lpc313x_mci *host)
{
	/* Data transfer was stopped by the interrupt handler */
	lpc313x_mci_set_pending(host, EVENT_XFER_COMPLETE);
}
#endif

static void lpc313x_mci_submit_data(struct lpc313x_mci *host, struct mmc_data *data)
{
	data->error = -EINPROGRESS;

	WARN_ON(host->data);
	host->sg = NULL;
	host->data = data;

	if (lpc313x_mci_submit_data_dma(host, data)) {
		host->sg = data->sg;
		host->pio_offset = 0;
		if (data->flags & MMC_DATA_READ)
			host->dir_status = LPC313x_MCI_RECV_STATUS;
		else 
			host->dir_status = LPC313x_MCI_SEND_STATUS;

		//SDMMC_INTMASK |= (SDMMC_INT_RXDR | SDMMC_INT_TXDR);
		SDMMC_CTRL &= ~SDMMC_CTRL_DMA_ENABLE; // enable dma
	}

}

#define mci_send_cmd(cmd,arg) {	\
    SDMMC_CMDARG = arg;		\
    SDMMC_CMD = SDMMC_CMD_START | cmd;\
    while (SDMMC_CMD & SDMMC_CMD_START); \
}

void lpc313x_mci_setup_bus(struct lpc313x_mci_slot *slot)
{
	struct lpc313x_mci *host = slot->host;
	u32 div;

	if (slot->clock != host->current_speed) {
		div  = (((host->bus_hz + (host->bus_hz / 5)) / slot->clock)) >> 1;

		dev_dbg(&slot->mmc->class_dev, "Bus speed (slot %d) = %dHz div:%d (actual %dHz)\n",
			slot->id, slot->clock, div, (host->bus_hz / div) >> 1);
		
		/* store the actual clock for calculations */
		slot->clock = (host->bus_hz / div) >> 1;
		/* disable clock */
		mci_writel(CLKENA, 0);
		mci_writel(CLKSRC,0);
		/* inform CIU */
		mci_send_cmd( SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
		/* set clock to desired speed */
		mci_writel(CLKDIV, div);
		/* inform CIU */
		mci_send_cmd( SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
		/* enable clock */
		mci_writel(CLKENA, SDMMC_CLKEN_ENABLE);
		/* inform CIU */
		 mci_send_cmd( SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		host->current_speed = slot->clock;
	}

	/* Set the current slot bus width */
	mci_writel(CTYPE, slot->ctype);
}

static void lpc313x_mci_start_request(struct lpc313x_mci *host,
		struct lpc313x_mci_slot *slot)
{
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	u32			cmdflags;

	mrq = slot->mrq;
	/* now select the proper slot */
	if (host->pdata->select_slot)
		host->pdata->select_slot(slot->id);

	/* Slot specific timing and width adjustment */
	lpc313x_mci_setup_bus(slot);

	host->cur_slot = slot;
	host->mrq = mrq;

	host->pending_events = 0;
	host->completed_events = 0;
	host->data_status = 0;

	data = mrq->data;
	if (data) {
		lpc313x_mci_set_timeout(host, slot, data);
		mci_writel(BYTCNT,data->blksz*data->blocks);
		mci_writel(BLKSIZ,data->blksz);
	}

	cmd = mrq->cmd;
	cmdflags = lpc313x_mci_prepare_command(slot->mmc, cmd);

	if (unlikely(test_and_clear_bit(LPC313x_MMC_CARD_NEED_INIT, &slot->flags))) 
	    cmdflags |= SDMMC_CMD_INIT; //this is the first command, let set send the initializtion clock
	
	if (data) //we may need to move this code to mci_start_command
		lpc313x_mci_submit_data(host, data);

	lpc313x_mci_start_command(host, cmd, cmdflags);

	if (mrq->stop) 
		host->stop_cmdr = lpc313x_mci_prepare_command(slot->mmc, mrq->stop);
	
}



static void lpc313x_mci_queue_request(struct lpc313x_mci *host,
		struct lpc313x_mci_slot *slot, struct mmc_request *mrq)
{
	dev_vdbg(&slot->mmc->class_dev, "queue request: state=%d\n",
			host->state);

	//printk("#");
	spin_lock(&host->lock);
	slot->mrq = mrq;
	if (host->state == STATE_IDLE) {
		host->state = STATE_SENDING_CMD;
		lpc313x_mci_start_request(host, slot);
	} else {
		list_add_tail(&slot->queue_node, &host->queue);
	}
	spin_unlock(&host->lock);
}


static void lpc313x_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct lpc313x_mci_slot	*slot = mmc_priv(mmc);
	struct lpc313x_mci	*host = slot->host;

	WARN_ON(slot->mrq);

	if (!test_bit(LPC313x_MMC_CARD_PRESENT, &slot->flags)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	lpc313x_mci_queue_request(host, slot, mrq);
}

static void lpc313x_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lpc313x_mci_slot	*slot = mmc_priv(mmc);

	slot->ctype = 0; // set default 1 bit mode
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		slot->ctype = 0;
		break;
	case MMC_BUS_WIDTH_4:
		slot->ctype = SDMMC_CTYPE_4BIT;
		break;
	}


	if (ios->clock) {
		spin_lock(&slot->host->lock);
		/*
		 * Use mirror of ios->clock to prevent race with mmc
		 * core ios update when finding the minimum.
		 */
		slot->clock = ios->clock;

		spin_unlock(&slot->host->lock);
	} else {
		spin_lock(&slot->host->lock);
		slot->clock = 0;
		spin_unlock(&slot->host->lock);
	}

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		set_bit(LPC313x_MMC_CARD_NEED_INIT, &slot->flags);
		break;
	default:
		break;
	}
}



static int lpc313x_mci_get_ro(struct mmc_host *mmc)
{
	int			read_only = -ENOSYS;
	struct lpc313x_mci_slot	*slot = mmc_priv(mmc);
	struct lpc313x_mci_board *brd = slot->host->pdata;

	if (brd->get_ro != NULL) {
		read_only = brd->get_ro(slot->id);
		dev_dbg(&mmc->class_dev, "card is %s\n",
				read_only ? "read-only" : "read-write");
	}

	return read_only;
}


static int lpc313x_mci_get_cd(struct mmc_host *mmc)
{
	int			present = -ENOSYS;
	struct lpc313x_mci_slot	*slot = mmc_priv(mmc);
	struct lpc313x_mci_board *brd = slot->host->pdata;

	present = !brd->get_cd(slot->id);
	dev_vdbg(&mmc->class_dev, "card is %spresent\n", present ? "" : "not ");

	return present;
}

static const struct mmc_host_ops lpc313x_mci_ops = {
	.request	= lpc313x_mci_request,
	.set_ios	= lpc313x_mci_set_ios,
	.get_ro		= lpc313x_mci_get_ro,
	.get_cd		= lpc313x_mci_get_cd,
};

static void lpc313x_mci_request_end(struct lpc313x_mci *host, struct mmc_request *mrq)
	__releases(&host->lock)
	__acquires(&host->lock)
{
	struct lpc313x_mci_slot	*slot = NULL;
	struct mmc_host		*prev_mmc = host->cur_slot->mmc;

	WARN_ON(host->cmd || host->data);

	host->cur_slot->mrq = NULL;
	host->mrq = NULL;
	if (!list_empty(&host->queue)) {
		slot = list_entry(host->queue.next,
				struct lpc313x_mci_slot, queue_node);
		list_del(&slot->queue_node);
		dev_vdbg(&host->pdev->dev, "list not empty: %s is next\n",
				mmc_hostname(slot->mmc));
		host->state = STATE_SENDING_CMD;
		lpc313x_mci_start_request(host, slot);
	} else {
		dev_vdbg(&host->pdev->dev, "list empty\n");
		host->state = STATE_IDLE;
	}

	//printk("-");

	spin_unlock(&host->lock);
	mmc_request_done(prev_mmc, mrq);

	spin_lock(&host->lock);
}

static void lpc313x_mci_command_complete(struct lpc313x_mci *host,
			struct mmc_command *cmd)
{
	u32		status = host->cmd_status;

	host->cmd_status = 0;

	if(cmd->flags & MMC_RSP_PRESENT) {

	    if(cmd->flags & MMC_RSP_136) {

		/* Read the response from the card (up to 16 bytes).
		 * LPC313x MMC controller saves bits 127-96 in RESP3
		 * for easy parsing. But the UNSTUFF_BITS macro in core/mmc.c
		 * core/sd.c expect those bits be in resp[0]. Hence
		 * reverse the response word order.
		 */
		cmd->resp[3] = mci_readl(RESP0);
		cmd->resp[2] = mci_readl(RESP1);
		cmd->resp[1] = mci_readl(RESP2);
		cmd->resp[0] = mci_readl(RESP3);
	    } else {
	        cmd->resp[0] = mci_readl(RESP0);
		cmd->resp[1] = 0;
		cmd->resp[2] = 0;
		cmd->resp[3] = 0;
	    }
	}

	if (status & SDMMC_INT_RTO)
		cmd->error = -ETIMEDOUT;
	else if ((cmd->flags & MMC_RSP_CRC) && (status & SDMMC_INT_RCRC))
		cmd->error = -EILSEQ;
	else if (status & SDMMC_INT_RESP_ERR)
		cmd->error = -EIO;
	else
		cmd->error = 0;

	if (cmd->error) {
		dev_vdbg(&host->pdev->dev,
			"command error: status=0x%08x resp=0x%08x\n"
			"cmd=0x%08x arg=0x%08x flg=0x%08x err=%d\n", 
			status, cmd->resp[0], 
			cmd->opcode, cmd->arg, cmd->flags, cmd->error);

		if (cmd->data) {
			host->data = NULL;
			lpc313x_mci_stop_dma(host);
		}
	} 
}

static void lpc313x_mci_tasklet_func(unsigned long priv)
{
	struct lpc313x_mci	*host = (struct lpc313x_mci *)priv;
	struct mmc_request	*mrq = host->mrq;
	struct mmc_data		*data = host->data;
	struct mmc_command	*cmd = host->cmd;
	enum lpc313x_mci_state	state = host->state;
	enum lpc313x_mci_state	prev_state;
	u32			status;

	spin_lock(&host->lock);

	state = host->state;
#if 0
	dev_vdbg(&host->pdev->dev,
		"tasklet: state %u pending/completed/mask %lx/%lx/%x\n",
		state, host->pending_events, host->completed_events,
		mci_readl(host, IMR)); // check reg
#endif
	do {
		prev_state = state;

		switch (state) {
		case STATE_IDLE:
			break;

		case STATE_SENDING_CMD:
			if (!lpc313x_mci_test_and_clear_pending(host,
						EVENT_CMD_COMPLETE))
				break;

			host->cmd = NULL;
			lpc313x_mci_set_completed(host, EVENT_CMD_COMPLETE);
			lpc313x_mci_command_complete(host, mrq->cmd);
			if (!mrq->data || cmd->error) {
				lpc313x_mci_request_end(host, host->mrq);
				goto unlock;
			}

			prev_state = state = STATE_SENDING_DATA;
			/* fall through */

		case STATE_SENDING_DATA:
			if (lpc313x_mci_test_and_clear_pending(host,
						EVENT_DATA_ERROR)) {
				lpc313x_mci_stop_dma(host);
				if (data->stop)
					send_stop_cmd(host, data);
				state = STATE_DATA_ERROR;
				break;
			}

			if (!lpc313x_mci_test_and_clear_pending(host,
						EVENT_XFER_COMPLETE))
				break;

			lpc313x_mci_set_completed(host, EVENT_XFER_COMPLETE);
			prev_state = state = STATE_DATA_BUSY;
			/* fall through */

		case STATE_DATA_BUSY:
			if (!lpc313x_mci_test_and_clear_pending(host,
						EVENT_DATA_COMPLETE))
				break;

			host->data = NULL;
			lpc313x_mci_set_completed(host, EVENT_DATA_COMPLETE);
			status = host->data_status;

			if (unlikely(status & LPC313x_MCI_DATA_ERROR_FLAGS)) {
				if (status & SDMMC_INT_DTO) {
					dev_err(&host->pdev->dev,
							"data timeout error\n");
					data->error = -ETIMEDOUT;
				} else if (status & SDMMC_INT_DCRC) {
					dev_err(&host->pdev->dev,
							"data CRC error\n");
					data->error = -EILSEQ;
				} else {
					dev_err(&host->pdev->dev,
						"data FIFO error (status=%08x)\n",
						status);
					data->error = -EIO;
				}
			}
			else {
				data->bytes_xfered = data->blocks * data->blksz;
				data->error = 0;
			}

			if (!data->stop) {
				lpc313x_mci_request_end(host, host->mrq);
				goto unlock;
			}

			prev_state = state = STATE_SENDING_STOP;
			if (!data->error)
				send_stop_cmd(host, data);
			/* fall through */

		case STATE_SENDING_STOP:
			if (!lpc313x_mci_test_and_clear_pending(host,
						EVENT_CMD_COMPLETE))
				break;

			host->cmd = NULL;
			lpc313x_mci_command_complete(host, mrq->stop);
			lpc313x_mci_request_end(host, host->mrq);
			goto unlock;
		case STATE_DATA_ERROR:
			if (!lpc313x_mci_test_and_clear_pending(host,
						EVENT_XFER_COMPLETE))
				break;

			state = STATE_DATA_BUSY;
			break;
		}
	} while (state != prev_state);

	host->state = state;

unlock:
	spin_unlock(&host->lock);

}



inline static void lpc313x_mci_push_data(void *buf,int cnt)
{
    u32* pData = (u32*)buf;

    if (cnt % 4 != 0) 
	    printk("error not align 4\n");

    cnt = cnt >> 2;
    while (cnt > 0) {
        SDMMC_DATA = *pData++ ;
        cnt--;
    }
}

inline static void lpc313x_mci_pull_data(void *buf,int cnt)
{
    u32* pData = (u32*)buf;

    if (cnt % 4 != 0) 
	    printk("error not align 4\n");
    cnt = cnt >> 2;
    while (cnt > 0) {
        *pData++ = SDMMC_DATA;
        cnt--;
    }
}

static void lpc313x_mci_read_data_pio(struct lpc313x_mci *host)
{
	struct scatterlist	*sg = host->sg;
	void			*buf = sg_virt(sg);
	unsigned int		offset = host->pio_offset;
	struct mmc_data		*data = host->data;
	u32			status;
	unsigned int		nbytes = 0,len,old_len,count =0;

	do {
		len = SDMMC_GET_FCNT(mci_readl(STATUS)) << 2;
		if(count == 0) 
			old_len = len;
		if (likely(offset + len <= sg->length)) {
			lpc313x_mci_pull_data((void *)(buf + offset),len);

			offset += len;
			nbytes += len;

			if (offset == sg->length) {
				flush_dcache_page(sg_page(sg));
				host->sg = sg = sg_next(sg);
				if (!sg)
					goto done;
				offset = 0;
				buf = sg_virt(sg);
			}
		} else {
			unsigned int remaining = sg->length - offset;
			lpc313x_mci_pull_data((void *)(buf + offset),remaining);
			nbytes += remaining;

			flush_dcache_page(sg_page(sg));
			host->sg = sg = sg_next(sg);
			if (!sg)
				goto done;
			offset = len - remaining;
			buf = sg_virt(sg);
			lpc313x_mci_pull_data(buf,offset);
			nbytes += offset;
		}

		status = mci_readl(MINTSTS);
		mci_writel(RINTSTS,SDMMC_INT_RXDR); // clear RXDR interrupt
		if (status & LPC313x_MCI_DATA_ERROR_FLAGS) {
			host->data_status = status;
			data->bytes_xfered += nbytes;
			smp_wmb();
			lpc313x_mci_set_pending(host, EVENT_DATA_ERROR);
			tasklet_schedule(&host->tasklet);
			return;
		}
		count ++;
	} while (status & SDMMC_INT_RXDR); // if the RXDR is ready let read again
	len = SDMMC_GET_FCNT(mci_readl(STATUS));
	host->pio_offset = offset;
	data->bytes_xfered += nbytes;
	return;

done:
	data->bytes_xfered += nbytes;
	smp_wmb();
	lpc313x_mci_set_pending(host, EVENT_XFER_COMPLETE);
}

static void lpc313x_mci_write_data_pio(struct lpc313x_mci *host)
{
	struct scatterlist	*sg = host->sg;
	void			*buf = sg_virt(sg);
	unsigned int		offset = host->pio_offset;
	struct mmc_data		*data = host->data;
	u32			status;
	unsigned int		nbytes = 0,len;

	do {

		len = SDMMC_FIFO_SZ - (SDMMC_GET_FCNT(mci_readl(STATUS)) << 2);
		if (likely(offset + len <= sg->length)) {
			lpc313x_mci_push_data((void *)(buf + offset),len);

			offset += len;
			nbytes += len;
			if (offset == sg->length) {
				host->sg = sg = sg_next(sg);
				if (!sg)
					goto done;

				offset = 0;
				buf = sg_virt(sg);
			}
		} else {
			unsigned int remaining = sg->length - offset;

			lpc313x_mci_push_data((void *)(buf + offset), remaining);
			nbytes += remaining;

			host->sg = sg = sg_next(sg);
			if (!sg) {
				goto done;
			}

			offset = len - remaining;
			buf = sg_virt(sg);
			lpc313x_mci_push_data((void *)buf, offset);
			nbytes += offset;
		}

		status = mci_readl(MINTSTS);
		mci_writel(RINTSTS,SDMMC_INT_TXDR); // clear RXDR interrupt
		if (status & LPC313x_MCI_DATA_ERROR_FLAGS) {
			host->data_status = status;
			data->bytes_xfered += nbytes;
			smp_wmb();
			lpc313x_mci_set_pending(host, EVENT_DATA_ERROR);
			tasklet_schedule(&host->tasklet);
			return;
		}
	} while (status & SDMMC_INT_TXDR); // if TXDR, let write again

	host->pio_offset = offset;
	data->bytes_xfered += nbytes;

	return;

done:
	data->bytes_xfered += nbytes;
	smp_wmb();
	lpc313x_mci_set_pending(host, EVENT_XFER_COMPLETE);
}

static void lpc313x_mci_cmd_interrupt(struct lpc313x_mci *host, u32 status)
{
	if(!host->cmd_status) 
		host->cmd_status = status;

	smp_wmb();
	lpc313x_mci_set_pending(host, EVENT_CMD_COMPLETE);
	tasklet_schedule(&host->tasklet);
}

static irqreturn_t lpc313x_mci_interrupt(int irq, void *dev_id)
{
	struct lpc313x_mci	*host = dev_id;
	u32			status,  pending;
	unsigned int		pass_count = 0;

	spin_lock(&host->lock);
	do {
		status = mci_readl(RINTSTS);
		pending = mci_readl(MINTSTS);// read only mask reg
		if (!pending)
			break;
		if(pending & LPC313x_MCI_CMD_ERROR_FLAGS) {
		    mci_writel(RINTSTS,LPC313x_MCI_CMD_ERROR_FLAGS);  //  clear interrupt
		    host->cmd_status = status;
		    smp_wmb();
		    lpc313x_mci_set_pending(host, EVENT_CMD_COMPLETE);
		    tasklet_schedule(&host->tasklet);
		}

		if (pending & LPC313x_MCI_DATA_ERROR_FLAGS) { // if there is an error, let report DATA_ERROR
			mci_writel(RINTSTS,LPC313x_MCI_DATA_ERROR_FLAGS);  // clear interrupt
			host->data_status = status;
			smp_wmb();
			lpc313x_mci_set_pending(host, EVENT_DATA_ERROR);
			tasklet_schedule(&host->tasklet);
		}


		if(pending & SDMMC_INT_DATA_OVER) {
		    mci_writel(RINTSTS,SDMMC_INT_DATA_OVER);  // clear interrupt
		    if (!host->data_status)
			host->data_status = status;
		    smp_wmb();
		    if(host->dir_status == LPC313x_MCI_RECV_STATUS) {
			if(host->sg != NULL) 
				lpc313x_mci_read_data_pio(host);
		    }
		    lpc313x_mci_set_pending(host, EVENT_DATA_COMPLETE);
		    tasklet_schedule(&host->tasklet);
		}

		if (pending & SDMMC_INT_RXDR) {
		    mci_writel(RINTSTS,SDMMC_INT_RXDR);  //  clear interrupt
		    if(host->sg) 
			    lpc313x_mci_read_data_pio(host);
		}

		if (pending & SDMMC_INT_TXDR) {
		    mci_writel(RINTSTS,SDMMC_INT_TXDR);  //  clear interrupt
		    if(host->sg) {
			lpc313x_mci_write_data_pio(host);
		    }
		}

		if (pending & SDMMC_INT_CMD_DONE) {
		    mci_writel(RINTSTS,SDMMC_INT_CMD_DONE);  //  clear interrupt
		    lpc313x_mci_cmd_interrupt(host, status);
		}
	} while (pass_count++ < 5);
	
	spin_unlock(&host->lock);

	return pass_count ? IRQ_HANDLED : IRQ_NONE;
}

/*
 *
 * MMC card detect thread, kicked off from detect interrupt, 1 timer per slot
 *
 */
static void lpc313x_mci_detect_change(unsigned long slot_data)
{
	struct lpc313x_mci_slot *slot = (struct lpc313x_mci_slot *) slot_data;
	struct lpc313x_mci *host;
	struct mmc_request *mrq;
	bool present;
	bool present_old;

	host = slot->host;
	/*
	 * lpc313x_mci_cleanup_slot() sets the ATMCI_SHUTDOWN flag before
	 * freeing the interrupt. We must not re-enable the interrupt
	 * if it has been freed, and if we're shutting down, it
	 * doesn't really matter whether the card is present or not.
	 */
	smp_rmb();
	if (test_bit(LPC313x_MMC_SHUTDOWN, &slot->flags))
		return;

	enable_irq(slot->irq);
	present = !host->pdata->get_cd(slot->id);
	present_old = test_bit(LPC313x_MMC_CARD_PRESENT, &slot->flags);
	dev_vdbg(&slot->mmc->class_dev, "detect change: %d (was %d)\n",
			present, present_old);

	if (present != present_old) {

		dev_info(&slot->mmc->class_dev, "card %s\n",
			present ? "inserted" : "removed");

		spin_lock(&host->lock);

		/* Power up slot */
		if (present != 0) {
			if (host->pdata->setpower)
				host->pdata->setpower(slot->id, slot->mmc->ocr_avail);

			set_bit(LPC313x_MMC_CARD_PRESENT, &slot->flags);
		} else {
			if (host->pdata->setpower)
				host->pdata->setpower(slot->id, 0);

			clear_bit(LPC313x_MMC_CARD_PRESENT, &slot->flags);
		}			


		/* Clean up queue if present */
		mrq = slot->mrq;
		if (mrq) {
			if (mrq == host->mrq) {
			  	/* reset all blocks */
			  	mci_writel(CTRL,(SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));
			  	/* wait till resets clear */
			  	while (mci_readl(CTRL) & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));

				host->data = NULL;
				host->cmd = NULL;

				switch (host->state) {
				case STATE_IDLE:
					break;
				case STATE_SENDING_CMD:
					mrq->cmd->error = -ENOMEDIUM;
					if (!mrq->data)
						break;
					/* fall through */
				case STATE_SENDING_DATA:
					mrq->data->error = -ENOMEDIUM;
					lpc313x_mci_stop_dma(host);
					break;
				case STATE_DATA_BUSY:
				case STATE_DATA_ERROR:
					if (mrq->data->error == -EINPROGRESS)
						mrq->data->error = -ENOMEDIUM;
					if (!mrq->stop)
						break;
					/* fall through */
				case STATE_SENDING_STOP:
					mrq->stop->error = -ENOMEDIUM;
					break;
				}

				lpc313x_mci_request_end(host, mrq);
			} else {
				list_del(&slot->queue_node);
				mrq->cmd->error = -ENOMEDIUM;
				if (mrq->data)
					mrq->data->error = -ENOMEDIUM;
				if (mrq->stop)
					mrq->stop->error = -ENOMEDIUM;

				spin_unlock(&host->lock);
				mmc_request_done(slot->mmc, mrq);
				spin_lock(&host->lock);
			}

		}

		spin_unlock(&host->lock);
		mmc_detect_change(slot->mmc, 0);
	}
}

static irqreturn_t lpc313x_mci_detect_interrupt(int irq, void *dev_id)
{
	struct lpc313x_mci_slot	*slot = dev_id;

	/*
	 * Disable interrupts until the pin has stabilized and check
	 * the state then. Use mod_timer() since we may be in the
	 * middle of the timer routine when this interrupt triggers.
	 */
	disable_irq_nosync(irq);
	mod_timer(&slot->detect_timer, jiffies + msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

static int __init
lpc313x_mci_init_slot(struct lpc313x_mci *host, unsigned int id)
{
	struct mmc_host			*mmc;
	struct lpc313x_mci_slot		*slot;

	mmc = mmc_alloc_host(sizeof(struct lpc313x_mci_slot), &host->pdev->dev);

	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);
	slot->id = id;
	slot->mmc = mmc;
	slot->host = host;

	mmc->ops = &lpc313x_mci_ops;
	mmc->f_min = DIV_ROUND_UP(host->bus_hz, 510);
	mmc->f_max = host->bus_hz/2; //max f is clock to mmc_clk/2
	if (host->pdata->get_ocr)
		mmc->ocr_avail = host->pdata->get_ocr(id);
	else
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	/* Start with slot power disabled, will be enabled when card is detected */
	if (host->pdata->setpower)
		host->pdata->setpower(id, 0);

	mmc->caps = 0;
	if (host->pdata->get_bus_wd)
		if (host->pdata->get_bus_wd(slot->id) >= 4)
			mmc->caps |= MMC_CAP_4_BIT_DATA;

	mmc->max_phys_segs = 64;
	mmc->max_hw_segs = 64;
	mmc->max_blk_size = 65536; /* BLKSIZ is 16 bits*/
	mmc->max_blk_count = 512;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	/* call board init */
	slot->irq = host->pdata->init(id, lpc313x_mci_detect_interrupt, slot);
	/* Assume card is present initially */
	if(!host->pdata->get_cd(id))
		set_bit(LPC313x_MMC_CARD_PRESENT, &slot->flags);
	else
		clear_bit(LPC313x_MMC_CARD_PRESENT, &slot->flags);

	host->slot[id] = slot;
	mmc_add_host(mmc);


#if defined (CONFIG_DEBUG_FS)
	lpc313x_mci_init_debugfs(slot);
#endif

	/* Create card detect handler thread for the slot */
	setup_timer(&slot->detect_timer, lpc313x_mci_detect_change,
			(unsigned long)slot);

	return 0;
}

static void lpc313x_mci_cleanup_slot(struct lpc313x_mci_slot *slot,
		unsigned int id)
{
	/* Shutdown detect IRQ and kill detect thread */
	if (slot->host->pdata->exit)
		slot->host->pdata->exit(id);
	del_timer_sync(&slot->detect_timer);

	/* Debugfs stuff is cleaned up by mmc core */
	set_bit(LPC313x_MMC_SHUTDOWN, &slot->flags);
	smp_wmb();
	mmc_remove_host(slot->mmc);
	slot->host->slot[id] = NULL;
	mmc_free_host(slot->mmc);
}


static int lpc313x_mci_probe(struct platform_device *pdev)
{
	struct lpc313x_mci		*host;
	struct resource			*regs;
	struct lpc313x_mci_board	*pdata;
	int				irq;
	int				ret = 0;
	int i;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;


	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	host = kzalloc(sizeof(struct lpc313x_mci), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->pdev = pdev;
	host->pdata = pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Platform data missing\n");
		ret = -ENODEV;
		goto err_freehost;
	}

	if (((pdata->num_slots > 1) && !(pdata->select_slot)) ||
	     !(pdata->get_ro) || !(pdata->get_cd) || !(pdata->init)) {
		dev_err(&pdev->dev, "Platform data wrong\n");
		ret = -ENODEV;
		goto err_freehost;
	}

	spin_lock_init(&host->lock);
	INIT_LIST_HEAD(&host->queue);

	ret = -ENOMEM;
	host->regs = ioremap(regs->start, regs->end - regs->start);
	if (!host->regs)
	    goto err_freehost;

	/* enable the clock to MCI module */
	cgu_clk_en_dis(CGU_SB_SD_MMC_HCLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_SD_MMC_CCLK_IN_ID, 1);

	/* reset SD/MMC/MCI modules through CGU */
	/* clear and set the register */
	CGU_CFG->resetn_soft[SD_MMC_PNRES_SOFT] = 0;
	CGU_CFG->resetn_soft[SD_MMC_NRES_CCLK_IN_SOFT] = 0;
	/* introduce some delay */
	udelay(1);
	CGU_CFG->resetn_soft[SD_MMC_NRES_CCLK_IN_SOFT] = CGU_CONFIG_SOFT_RESET;
	CGU_CFG->resetn_soft[SD_MMC_PNRES_SOFT] = CGU_CONFIG_SOFT_RESET;

#ifdef USE_DMA
	host->dma_chn = dma_request_sg_channel("MCI",  lpc313x_mci_dma_complete, host, 1);
	host->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &host->sg_dma, GFP_KERNEL);
	if (host->sg_cpu == NULL) {
		dev_err(&pdev->dev,
			 "%s: could not alloc dma memory \n", __func__);
		goto err_freemap;
	}
#endif
	host->bus_hz = cgu_get_clk_freq(CGU_SB_SD_MMC_CCLK_IN_ID); //40000000;

	/* Set IOCONF to MCI pins */
	SYS_SDMMC_DELAYMODES = 0;
	SYS_MUX_GPIO_MCI = 1;

	/* set the pins as driven by IP in IOCONF */
	GPIO_DRV_IP(IOCONF_EBI_MCI, 0xF0000003);

	/* set delay gates */
	SYS_SDMMC_DELAYMODES = 0x1B;

  	/* reset all blocks */
  	mci_writel(CTRL,(SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));
  	/* wait till resets clear */
  	while (mci_readl(CTRL) & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));

	 /* Clear the interrupts for the host controller */
	mci_writel(RINTSTS, 0xFFFFFFFF);
	mci_writel(INTMASK, 0); // disable all mmc interrupt first

  	/* Put in max timeout */
  	mci_writel(TMOUT, 0xFFFFFFFF);

  	/* FIFO threshold settings  */
#ifdef BURST_DMA
	mci_writel(FIFOTH, ((0x1 << 28) | (0xB << 16) | (0xC << 0))); // RXMark = 11, TXMark = 12, DMA Size = 4
#else
  	mci_writel(FIFOTH, ((0x2 << 28) | (0x10 << 16) | (0x10 << 0))); // RXMark = 16, TXMark = 16, DMA Size = 8
#endif

	/* disable clock to CIU */
	mci_writel(CLKENA,0);
	mci_writel(CLKSRC,0);

	tasklet_init(&host->tasklet, lpc313x_mci_tasklet_func, (unsigned long)host);
	ret = request_irq(irq, lpc313x_mci_interrupt, 0, dev_name(&pdev->dev), host);
	if (ret)
	    goto err_dmaunmap;

	platform_set_drvdata(pdev, host);

	/* We need at least one slot to succeed ####pd####*/
	for (i = 0; i < host->pdata->num_slots; i++) {
		ret = lpc313x_mci_init_slot(host, i);
		if (ret) {
		    ret = -ENODEV;
		    goto err_init_slot;
		}
	}

	// enable interrupt for command done, data over, data empty, receive ready and error such as transmit, receive timeout, crc error
	mci_writel(RINTSTS, 0xFFFFFFFF);
	mci_writel(INTMASK,SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER | SDMMC_INT_TXDR | SDMMC_INT_RXDR | LPC313x_MCI_ERROR_FLAGS);
	mci_writel(CTRL,SDMMC_CTRL_INT_ENABLE); // enable mci interrupt


	dev_info(&pdev->dev, "LPC313x MMC controller at irq %d\n", irq);

	return 0;

err_init_slot:
	/* De-init any initialized slots */
	while (i > 0) {
		if (host->slot[i])
			lpc313x_mci_cleanup_slot(host->slot[i], i);
		i--;
	}
	free_irq(irq, host);
err_dmaunmap:
#ifdef USE_DMA
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	dma_release_sg_channel(host->dma_chn);
err_freemap:
#endif
	iounmap(host->regs);
err_freehost:
	kfree(host);
	return ret;
}



static int __exit lpc313x_mci_remove(struct platform_device *pdev)
{
	struct lpc313x_mci *host = platform_get_drvdata(pdev);
	int i;

	mci_writel(RINTSTS, 0xFFFFFFFF);
	mci_writel(INTMASK, 0); // disable all mmc interrupt first

	platform_set_drvdata(pdev, NULL);

	for (i = 0; i < host->pdata->num_slots; i++) {
		dev_dbg(&pdev->dev, "remove slot %d\n", i);
		if (host->slot[i])
			lpc313x_mci_cleanup_slot(host->slot[i], i);
	}

	/* disable clock to CIU */
	mci_writel(CLKENA,0);
	mci_writel(CLKSRC,0);

	/*  turn off the mci clock here */
	cgu_clk_en_dis(CGU_SB_SD_MMC_HCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_SD_MMC_CCLK_IN_ID, 0);

	free_irq(platform_get_irq(pdev, 0), host);
#ifdef USE_DMA
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	dma_release_sg_channel(host->dma_chn);
#endif
	iounmap(host->regs);

	kfree(host);
	return 0;
}

static int lpc313x_mci_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM
	/* Disable Card clock */
	mci_writel(CLKENA,0);

	/* Disable IP clocks */
	cgu_clk_en_dis(CGU_SB_SD_MMC_HCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_SD_MMC_CCLK_IN_ID, 0);
#endif
	return 0;
}

static int lpc313x_mci_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	/* Enable IP Clocks */
	cgu_clk_en_dis(CGU_SB_SD_MMC_HCLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_SD_MMC_CCLK_IN_ID, 1);

	/* Enable Card clock */
	mci_writel(CLKENA ,SDMMC_CLKEN_ENABLE);
#endif
	return 0;
}

static struct platform_driver lpc313x_mci_driver = {
	.suspend    = lpc313x_mci_suspend,
	.resume     = lpc313x_mci_resume,
	.remove		= __exit_p(lpc313x_mci_remove),
	.driver		= {
		.name		= "lpc313x_mmc",
	},
};

static int __init lpc313x_mci_init(void)
{
	return platform_driver_probe(&lpc313x_mci_driver, lpc313x_mci_probe);
}

static void __exit lpc313x_mci_exit(void)
{
	platform_driver_unregister(&lpc313x_mci_driver);
}

module_init(lpc313x_mci_init);
module_exit(lpc313x_mci_exit);

MODULE_DESCRIPTION("LPC313x Multimedia Card Interface driver");
MODULE_AUTHOR("NXP Semiconductor VietNam");
MODULE_LICENSE("GPL v2");
