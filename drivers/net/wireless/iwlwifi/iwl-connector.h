/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2008 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 * Tomas Winkler <tomas.winkler@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2008 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/**
 * This file defines the connector interface.
 */

#ifndef __iwl_connector_h__
#define __iwl_connector_h__

#include <linux/etherdevice.h>
#include <net/mac80211.h>
#include <asm/unaligned.h>
#include <linux/connector.h>
#include "iwl-dev.h"
#include "iwl-commands.h"
#include "iwl-core.h"

extern struct cb_id connector_id;
extern char *connector_name;

void connector_send_msg(const u8 *data, const u32 size, const u8 code);
void iwl_connector_set_priv(struct iwl_priv *p);
void connector_tasklet(unsigned long trash);

#define IWL_CONN_BFEE_NOTIF	REPLY_BFEE_NOTIFICATION		/* 0xbb */
#define IWL_CONN_RX_PHY		REPLY_RX_PHY_CMD		/* 0xc0 */
#define IWL_CONN_RX_MPDU	REPLY_RX_MPDU_CMD		/* 0xc1 */
#define IWL_CONN_RX		REPLY_RX			/* 0xc3 */
#define IWL_CONN_NOISE		0xd0		/* new ID not a command */
#define IWL_CONN_TX_RESP	REPLY_TX			/* 0x1c */
#define IWL_CONN_TX_BLOCK_AGG	REPLY_COMPRESSED_BA		/* 0xc5 */
#define IWL_CONN_STATUS		0xd1		/* new ID not a command */

enum {
	IWL_CONN_BFEE_NOTIF_MSK		= (1 << 0),
	IWL_CONN_RX_PHY_MSK		= (1 << 1),
	IWL_CONN_RX_MPDU_MSK		= (1 << 2),
	IWL_CONN_RX_MSK			= (1 << 3),
	IWL_CONN_NOISE_MSK		= (1 << 4),
	IWL_CONN_TX_RESP_MSK		= (1 << 5),
	IWL_CONN_TX_BLOCK_AGG_MSK	= (1 << 6),
	IWL_CONN_STATUS_MSK		= (1 << 7),
};

void connector_callback(struct cn_msg *msg, struct netlink_skb_parms *nsp);
int iwlagn_register_connector(void);
void iwlagn_unregister_connector(void);

/*
 * Struct to send TX block aggregation information down to the host.
 */
struct tx_agg_ba_connector_msg {
	u32 successes;
	u32 frame_count;
};

#endif /* __iwl_connector_h__ */
