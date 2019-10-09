/*
 * TC9562 ethernet driver.
 *
 * dwmac4_descs.c
 *
 * Copyright (C) 2015 STMicroelectronics Ltd
 * Copyright (C) 2019 Toshiba Electronic Devices & Storage Corporation
 *
 * This file has been derived from the STMicro Linux driver,
 * and developed or modified for TC9562.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*! History:
 *  30 Sep 2019 : Base lined
 *  VERSION     : 01-00
 */

#include "tc9562mac_inc.h"
//#include <linux/tc9562mac.h>
#include "common.h"
#include "dwmac4_descs.h"

int dwmac4_wrback_get_tx_status(void *data, struct tc9562mac_extra_stats *x,
				       struct dma_desc *p,
				       void __iomem *ioaddr)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	unsigned int tdes3;
	int ret = tx_done;

	DBGPR_FUNC("-->dwmac4_wrback_get_tx_status\n");

	tdes3 = le32_to_cpu(p->des3);

	/* Get tx owner first */
	if (unlikely(tdes3 & TDES3_OWN))
		return tx_dma_own;

	/* Verify tx error by looking at the last segment. */
	if (likely(!(tdes3 & TDES3_LAST_DESCRIPTOR)))
		return tx_not_ls;

	if (unlikely(tdes3 & TDES3_ERROR_SUMMARY)) {
		printk("dwmac4_wrback_get_tx_status  TDES3_ERROR_SUMMARY\n");
		if (unlikely(tdes3 & TDES3_JABBER_TIMEOUT))
			x->tx_jabber++;
		if (unlikely(tdes3 & TDES3_PACKET_FLUSHED))
			x->tx_frame_flushed++;
		if (unlikely(tdes3 & TDES3_LOSS_CARRIER)) {
			x->tx_losscarrier++;
			stats->tx_carrier_errors++;
		}
		if (unlikely(tdes3 & TDES3_NO_CARRIER)) {
			x->tx_carrier++;
			stats->tx_carrier_errors++;
		}
		if (unlikely((tdes3 & TDES3_LATE_COLLISION) ||
			     (tdes3 & TDES3_EXCESSIVE_COLLISION)))
			stats->collisions +=
			    (tdes3 & TDES3_COLLISION_COUNT_MASK)
			    >> TDES3_COLLISION_COUNT_SHIFT;

		if (unlikely(tdes3 & TDES3_EXCESSIVE_DEFERRAL))
			x->tx_deferred++;

		if (unlikely(tdes3 & TDES3_UNDERFLOW_ERROR))
			x->tx_underflow++;

		if (unlikely(tdes3 & TDES3_IP_HDR_ERROR))
			x->tx_ip_header_error++;

		if (unlikely(tdes3 & TDES3_PAYLOAD_ERROR))
			x->tx_payload_error++;

		ret = tx_err;
	}

	if (unlikely(tdes3 & TDES3_DEFERRED))
		x->tx_deferred++;

	DBGPR_FUNC("<--dwmac4_wrback_get_tx_status\n");

	return ret;
}

int dwmac4_wrback_get_rx_status(void *data, struct tc9562mac_extra_stats *x,
				       struct dma_desc *p)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	unsigned int rdes1 = le32_to_cpu(p->des1);
	unsigned int rdes2 = le32_to_cpu(p->des2);
	unsigned int rdes3 = le32_to_cpu(p->des3);
	int message_type;
	int ret = good_frame;

	DBGPR_FUNC("-->dwmac4_wrback_get_rx_status\n");

	if (unlikely(rdes3 & RDES3_OWN))
		return dma_own;

	/* Verify rx error by looking at the last segment. */
	if (likely(!(rdes3 & RDES3_LAST_DESCRIPTOR)))
		return discard_frame;

	if (unlikely(rdes3 & RDES3_ERROR_SUMMARY)) {
		if (unlikely(rdes3 & RDES3_GIANT_PACKET))
			stats->rx_length_errors++;
		if (unlikely(rdes3 & RDES3_OVERFLOW_ERROR))
			x->rx_gmac_overflow++;

		if (unlikely(rdes3 & RDES3_RECEIVE_WATCHDOG))
			x->rx_watchdog++;

		if (unlikely(rdes3 & RDES3_RECEIVE_ERROR))
			x->rx_mii++;

		if (unlikely(rdes3 & RDES3_CRC_ERROR)) {
			x->rx_crc_errors++;
			stats->rx_crc_errors++;
		}

		if (unlikely(rdes3 & RDES3_DRIBBLE_ERROR))
			x->dribbling_bit++;

		ret = discard_frame;
	}

	message_type = (rdes1 & ERDES4_MSG_TYPE_MASK) >> 8;

	if (rdes1 & RDES1_IP_HDR_ERROR)
		x->ip_hdr_err++;
	if (rdes1 & RDES1_IP_CSUM_BYPASSED)
		x->ip_csum_bypassed++;
	if (rdes1 & RDES1_IPV4_HEADER)
		x->ipv4_pkt_rcvd++;
	if (rdes1 & RDES1_IPV6_HEADER)
		x->ipv6_pkt_rcvd++;
	if(rdes1 & RDES1_IP_CSUM_ERROR){
		//printk("ip_csum(l4)_error \n");
	}
		//printk("ip payload type : %x \n\n", (rdes1 & RDES1_IP_PAYLOAD_TYPE_MASK));

	if (message_type == RDES_EXT_NO_PTP)
		x->no_ptp_rx_msg_type_ext++;
	else if (message_type == RDES_EXT_SYNC)
		x->ptp_rx_msg_type_sync++;
	else if (message_type == RDES_EXT_FOLLOW_UP)
		x->ptp_rx_msg_type_follow_up++;
	else if (message_type == RDES_EXT_DELAY_REQ)
		x->ptp_rx_msg_type_delay_req++;
	else if (message_type == RDES_EXT_DELAY_RESP)
		x->ptp_rx_msg_type_delay_resp++;
	else if (message_type == RDES_EXT_PDELAY_REQ)
		x->ptp_rx_msg_type_pdelay_req++;
	else if (message_type == RDES_EXT_PDELAY_RESP)
		x->ptp_rx_msg_type_pdelay_resp++;
	else if (message_type == RDES_EXT_PDELAY_FOLLOW_UP)
		x->ptp_rx_msg_type_pdelay_follow_up++;
	else if (message_type == RDES_PTP_ANNOUNCE)
		x->ptp_rx_msg_type_announce++;
	else if (message_type == RDES_PTP_MANAGEMENT)
		x->ptp_rx_msg_type_management++;
	else if (message_type == RDES_PTP_PKT_RESERVED_TYPE)
		x->ptp_rx_msg_pkt_reserved_type++;

	if (rdes1 & RDES1_PTP_PACKET_TYPE)
		x->ptp_frame_type++;
	if (rdes1 & RDES1_PTP_VER)
		x->ptp_ver++;
	if (rdes1 & RDES1_TIMESTAMP_DROPPED)
		x->timestamp_dropped++;

	if (unlikely(rdes2 & RDES2_SA_FILTER_FAIL)) {
		x->sa_rx_filter_fail++;
		//ret = discard_frame; /*Commented for RA Packet filter mode*/
	}
	if (unlikely(rdes2 & RDES2_DA_FILTER_FAIL)) {
		x->da_rx_filter_fail++;
		//ret = discard_frame; /*Commented for RA Packet filter mode*/
	}

	if (rdes2 & RDES2_L3_FILTER_MATCH)
		x->l3_filter_match++;
	if (rdes2 & RDES2_L4_FILTER_MATCH)
		x->l4_filter_match++;
	if ((rdes2 & RDES2_L3_L4_FILT_NB_MATCH_MASK)
	    >> RDES2_L3_L4_FILT_NB_MATCH_SHIFT)
		x->l3_l4_filter_no_match++;

	DBGPR_FUNC("<--dwmac4_wrback_get_rx_status\n");

	return ret;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
int dwmac4_rd_get_tx_len(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_rd_get_tx_len\n");
	return (le32_to_cpu(p->des2) & TDES2_BUFFER1_SIZE_MASK);
}

int dwmac4_get_tx_owner(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_get_tx_owner\n");
	return (le32_to_cpu(p->des3) & TDES3_OWN) >> TDES3_OWN_SHIFT;
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

void dwmac4_set_tx_owner(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_set_tx_owner\n");
	p->des3 |= cpu_to_le32(TDES3_OWN);
	DBGPR_FUNC("<--dwmac4_set_tx_owner\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
void dwmac4_set_rx_owner(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_set_rx_owner\n");
	p->des3 |= cpu_to_le32(RDES3_OWN);
	DBGPR_FUNC("<--dwmac4_set_rx_owner\n");
}

int dwmac4_get_tx_ls(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_get_tx_ls\n");
	return (le32_to_cpu(p->des3) & TDES3_LAST_DESCRIPTOR)
		>> TDES3_LAST_DESCRIPTOR_SHIFT;
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

int dwmac4_wrback_get_rx_frame_len(struct dma_desc *p, int rx_coe)
{
	DBGPR_FUNC("-->dwmac4_wrback_get_rx_frame_len\n");
	return (le32_to_cpu(p->des3) & RDES3_PACKET_SIZE_MASK);
}

void dwmac4_rd_enable_tx_timestamp(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_rd_enable_tx_timestamp\n");
	p->des2 |= cpu_to_le32(TDES2_TIMESTAMP_ENABLE);
	DBGPR_FUNC("<--dwmac4_rd_enable_tx_timestamp\n");
}

int dwmac4_wrback_get_tx_timestamp_status(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_wrback_get_tx_timestamp_status\n");
	/* Context type from W/B descriptor must be zero */
	if (le32_to_cpu(p->des3) & TDES3_CONTEXT_TYPE)
		return 0;

	/* Tx Timestamp Status is 1 so des0 and des1'll have valid values */
	if (le32_to_cpu(p->des3) & TDES3_TIMESTAMP_STATUS)
		return 1;

	DBGPR_FUNC("<--dwmac4_wrback_get_tx_timestamp_status\n");
	
	return 0;
}

inline u64 dwmac4_get_timestamp(void *desc, u32 ats)
{
	struct dma_desc *p = (struct dma_desc *)desc;
	u64 ns;

	DBGPR_FUNC("-->dwmac4_get_timestamp\n");
	ns = le32_to_cpu(p->des0);
	/* convert high/sec time stamp value to nanosecond */
	ns += le32_to_cpu(p->des1) * 1000000000ULL;
	DBGPR_FUNC("<--dwmac4_get_timestamp\n");
	return ns;
}

int dwmac4_rx_check_timestamp(void *desc)
{
	struct dma_desc *p = (struct dma_desc *)desc;
	u32 own, ctxt;
	int ret = 1;

	DBGPR_FUNC("-->dwmac4_rx_check_timestamp\n");

	own = p->des3 & RDES3_OWN;
	ctxt = ((p->des3 & RDES3_CONTEXT_DESCRIPTOR)
		>> RDES3_CONTEXT_DESCRIPTOR_SHIFT);

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
    rmb();
#else
    dma_rmb();
#endif

	if (likely(!own && ctxt)) {
		if ((p->des0 == 0xffffffff) && (p->des1 == 0xffffffff))
			/* Corrupted value */
			ret = -EINVAL;
		else
			/* A valid Timestamp is ready to be read */
			ret = 0;
	}

	DBGPR_FUNC("<--dwmac4_rx_check_timestamp\n");
	/* Timestamp not ready */
	return ret;
}

int dwmac4_wrback_get_rx_timestamp_status(void *desc, void *next_desc, u32 ats)
{
	struct dma_desc *p = (struct dma_desc *)desc;
	int ret = -EINVAL;

	DBGPR_FUNC("-->dwmac4_wrback_get_rx_timestamp_status\n");
	/* Get the status from normal w/b descriptor */
	if (likely(p->des3 & TDES3_RS1V)) {
		if (likely(le32_to_cpu(p->des1) & RDES1_TIMESTAMP_AVAILABLE)) {
			int i = 0;

			/* Check if timestamp is OK from context descriptor */
			do {
                ret = dwmac4_rx_check_timestamp(next_desc);				
				if (ret < 0)
					goto exit;
				i++;
                
                if(ret == 1)
                    udelay(1);
			} while ((ret == 1) && (i < 25));

			if (i == 25)
				ret = -EBUSY;
		}
	}
exit:
	if (likely(ret == 0))
		return 1;

	DBGPR_FUNC("<--dwmac4_wrback_get_rx_timestamp_status\n");

	return 0;
}

void dwmac4_rd_init_rx_desc(struct dma_desc *p, int disable_rx_ic,
				   int mode, int end)
{
	DBGPR_FUNC("-->dwmac4_rd_init_rx_desc\n");
	p->des3 = cpu_to_le32(RDES3_OWN | RDES3_BUFFER1_VALID_ADDR);

	p->des3 |= cpu_to_le32(RDES3_INT_ON_COMPLETION_EN);   
	DBGPR_FUNC("<--dwmac4_rd_init_rx_desc\n");
}

void dwmac4_rd_init_tx_desc(struct dma_desc *p, int mode, int end)
{
	DBGPR_FUNC("-->dwmac4_rd_init_tx_desc\n");
	p->des0 = 0;
	p->des1 = 0;
	p->des2 = 0;
	p->des3 = 0;
	DBGPR_FUNC("<--dwmac4_rd_init_tx_desc\n");
}

void dwmac4_rd_prepare_tx_desc(struct dma_desc *p, int is_fs, int len,
				      bool csum_flag, int mode, bool tx_own,
				      bool ls, unsigned int tot_pkt_len)
{
	unsigned int tdes3 = le32_to_cpu(p->des3);
	
	DBGPR_FUNC("-->dwmac4_rd_prepare_tx_desc\n");
	
	p->des2 |= cpu_to_le32(len & TDES2_BUFFER1_SIZE_MASK);
#ifdef TC9562_TX_TIMESTAMP_ALL
	p->des2 |= cpu_to_le32(TDES2_TIMESTAMP_ENABLE);   // Enable Timestamp
#endif
	tdes3 |= tot_pkt_len & TDES3_PACKET_SIZE_MASK;
	if (is_fs)
		tdes3 |= TDES3_FIRST_DESCRIPTOR;
	else
		tdes3 &= ~TDES3_FIRST_DESCRIPTOR;

	if (likely(csum_flag))
		tdes3 |= (TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT);
	else
		tdes3 &= ~(TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT);

	if (ls)
		tdes3 |= TDES3_LAST_DESCRIPTOR;
	else
		tdes3 &= ~TDES3_LAST_DESCRIPTOR;

	/* Finally set the OWN bit. Later the DMA will start! */
	if (tx_own)
		tdes3 |= TDES3_OWN;

	if (is_fs & tx_own)
		/* When the own bit, for the first frame, has to be set, all
		 * descriptors for the same frame has to be set before, to
		 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
        wmb();
#else
        dma_wmb();
#endif

	p->des3 = cpu_to_le32(tdes3);

	DBGPR_FUNC("<--dwmac4_rd_prepare_tx_desc\n");
	
}

void dwmac4_rd_prepare_tso_tx_desc(struct dma_desc *p, int is_fs,
					  int len1, int len2, bool tx_own,
					  bool ls, unsigned int tcphdrlen,
					  unsigned int tcppayloadlen)
{
	unsigned int tdes3 = le32_to_cpu(p->des3);

	DBGPR_FUNC("-->dwmac4_rd_prepare_tso_tx_desc\n");

	if (len1)
		p->des2 |= cpu_to_le32((len1 & TDES2_BUFFER1_SIZE_MASK));

	if (len2)
		p->des2 |= cpu_to_le32((len2 << TDES2_BUFFER2_SIZE_MASK_SHIFT)
			    & TDES2_BUFFER2_SIZE_MASK);

	if (is_fs) {
		tdes3 |= TDES3_FIRST_DESCRIPTOR |
			 TDES3_TCP_SEGMENTATION_ENABLE |
			 ((tcphdrlen << TDES3_HDR_LEN_SHIFT) &
			  TDES3_SLOT_NUMBER_MASK) |
			 ((tcppayloadlen & TDES3_TCP_PKT_PAYLOAD_MASK));
	} else {
		tdes3 &= ~TDES3_FIRST_DESCRIPTOR;
	}

	if (ls)
		tdes3 |= TDES3_LAST_DESCRIPTOR;
	else
		tdes3 &= ~TDES3_LAST_DESCRIPTOR;

	/* Finally set the OWN bit. Later the DMA will start! */
	if (tx_own)
		tdes3 |= TDES3_OWN;

	if (is_fs & tx_own)
		/* When the own bit, for the first frame, has to be set, all
		 * descriptors for the same frame has to be set before, to
		 * avoid race condition.
		 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
        wmb();
#else
        dma_wmb();
#endif

	p->des3 = cpu_to_le32(tdes3);

	DBGPR_FUNC("<--dwmac4_rd_prepare_tso_tx_desc\n");
	
}

void dwmac4_release_tx_desc(struct dma_desc *p, int mode)
{
	DBGPR_FUNC("-->dwmac4_release_tx_desc\n");

	p->des2 = 0;
	p->des3 = 0;

	DBGPR_FUNC("<--dwmac4_release_tx_desc\n");
}

void dwmac4_rd_set_tx_ic(struct dma_desc *p)
{
	DBGPR_FUNC("-->dwmac4_rd_set_tx_ic\n");
	p->des2 |= cpu_to_le32(TDES2_INTERRUPT_ON_COMPLETION);
	DBGPR_FUNC("<--dwmac4_rd_set_tx_ic\n");
}

void dwmac4_display_ring(void *head, unsigned int size, bool rx)
{
	struct dma_desc *p = (struct dma_desc *)head;
	int i;

	DBGPR_FUNC("-->dwmac4_display_ring\n");

	pr_info("%s descriptor ring:\n", rx ? "RX" : "TX");

	for (i = 0; i < size; i++) {
		pr_info("%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
			i, (unsigned int)virt_to_phys(p),
			le32_to_cpu(p->des0), le32_to_cpu(p->des1),
			le32_to_cpu(p->des2), le32_to_cpu(p->des3));
		p++;
	}

	DBGPR_FUNC("<--dwmac4_display_ring\n");

}

void dwmac4_set_mss_ctxt(struct dma_desc *p, unsigned int mss)
{
	DBGPR_FUNC("-->dwmac4_set_mss_ctxt\n");

	p->des0 = 0;
	p->des1 = 0;
	p->des2 = cpu_to_le32(mss);
	p->des3 = cpu_to_le32(TDES3_CONTEXT_TYPE | TDES3_CTXT_TCMSSV);

	DBGPR_FUNC("<--dwmac4_set_mss_ctxt\n");

	
}

const struct tc9562mac_desc_ops dwmac4_desc_ops = {
	.tx_status = dwmac4_wrback_get_tx_status,
	.rx_status = dwmac4_wrback_get_rx_status,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE			
	.get_tx_len = dwmac4_rd_get_tx_len,
	.get_tx_owner = dwmac4_get_tx_owner,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */			
	.set_tx_owner = dwmac4_set_tx_owner,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE		
	.set_rx_owner = dwmac4_set_rx_owner,
	.get_tx_ls = dwmac4_get_tx_ls,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.get_rx_frame_len = dwmac4_wrback_get_rx_frame_len,
	.enable_tx_timestamp = dwmac4_rd_enable_tx_timestamp,
	.get_tx_timestamp_status = dwmac4_wrback_get_tx_timestamp_status,
	.get_rx_timestamp_status = dwmac4_wrback_get_rx_timestamp_status,
	.get_timestamp = dwmac4_get_timestamp,
	.set_tx_ic = dwmac4_rd_set_tx_ic,
	.prepare_tx_desc = dwmac4_rd_prepare_tx_desc,
	.prepare_tso_tx_desc = dwmac4_rd_prepare_tso_tx_desc,
	.release_tx_desc = dwmac4_release_tx_desc,
	.init_rx_desc = dwmac4_rd_init_rx_desc,
	.init_tx_desc = dwmac4_rd_init_tx_desc,
	.display_ring = dwmac4_display_ring,
	.set_mss = dwmac4_set_mss_ctxt,
};

const struct tc9562mac_mode_ops dwmac4_ring_mode_ops = { };
