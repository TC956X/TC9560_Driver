/*
 * TC9562 ethernet driver.
 *
 * dwmac4_enh_desc.c
 *
 * Copyright (C) 2019 Toshiba Electronic Devices & Storage Corporation
 *
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
#include "dwmac4_enh_desc.h"
#include "dwmac4_descs.h"

int dwmac4_wrback_get_etx_status(void *data, struct tc9562mac_extra_stats *x,
				       struct dma_enhanced_desc *p,
				       void __iomem *ioaddr)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	unsigned int tdes3;
	int ret = tx_done;

	tdes3 = le32_to_cpu(p->basic.des3);

	DBGPR_FUNC("-->dwmac4_wrback_get_etx_status\n");

	/* Get tx owner first */
	if (unlikely(tdes3 & TDES3_OWN))
		return tx_dma_own;

	/* Verify tx error by looking at the last segment. */
	if (likely(!(tdes3 & TDES3_LAST_DESCRIPTOR)))
		return tx_not_ls;

	if (unlikely(tdes3 & TDES3_ERROR_SUMMARY)) {
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
	DBGPR_FUNC("<--dwmac4_wrback_get_etx_status\n");
	return ret;
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
int dwmac4_rd_get_etx_len(struct dma_enhanced_desc *p)
{
	DBGPR_FUNC("-->dwmac4_rd_get_etx_len\n");
	return (le32_to_cpu(p->basic.des2) & TDES2_BUFFER1_SIZE_MASK);
}

int dwmac4_get_etx_owner(struct dma_enhanced_desc *p)
{
	DBGPR_FUNC("-->dwmac4_get_etx_owner\n");
	return (le32_to_cpu(p->basic.des3) & TDES3_OWN) >> TDES3_OWN_SHIFT;
}

void dwmac4_set_etx_owner(struct dma_enhanced_desc *p)
{
	DBGPR_FUNC("-->dwmac4_set_etx_owner\n");
	p->basic.des3 |= cpu_to_le32(TDES3_OWN);
	DBGPR_FUNC("<--dwmac4_set_etx_owner\n");

}


int dwmac4_get_etx_ls(struct dma_enhanced_desc *p)
{
	DBGPR_FUNC("-->dwmac4_get_etx_ls\n");
	return (le32_to_cpu(p->basic.des3) & TDES3_LAST_DESCRIPTOR)
		>> TDES3_LAST_DESCRIPTOR_SHIFT;
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */


void dwmac4_rd_enable_etx_timestamp(struct dma_enhanced_desc *p)
{
	DBGPR_FUNC("-->dwmac4_rd_enable_etx_timestamp\n");
	p->basic.des2 |= cpu_to_le32(TDES2_TIMESTAMP_ENABLE);
	DBGPR_FUNC("<--dwmac4_rd_enable_etx_timestamp\n");
}

int dwmac4_wrback_get_etx_timestamp_status(struct dma_enhanced_desc *p)
{
	DBGPR_FUNC("-->dwmac4_wrback_get_etx_timestamp_status\n");
	/* Context type from W/B descriptor must be zero */
	if (le32_to_cpu(p->basic.des3) & TDES3_CONTEXT_TYPE)
		return 0;

	/* Tx Timestamp Status is 1 so des0 and des1'll have valid values */
	if (le32_to_cpu(p->basic.des3) & TDES3_TIMESTAMP_STATUS)
		return 1;
	DBGPR_FUNC("<--dwmac4_wrback_get_etx_timestamp_status\n");
	return 0;
}

inline u64 dwmac4_get_en_timestamp(void *desc, u32 ats)
{
	struct dma_enhanced_desc *p = (struct dma_enhanced_desc *)desc;
	u64 ns;
    
	DBGPR_FUNC("-->dwmac4_get_en_timestamp\n");
	ns = le32_to_cpu(p->basic.des0);
	/* convert high/sec time stamp value to nanosecond */
	ns += le32_to_cpu(p->basic.des1) * 1000000000ULL;
	DBGPR_FUNC("<--dwmac4_get_en_timestamp\n");

	return ns;
}



void dwmac4_rd_init_etx_desc(struct dma_enhanced_desc *p, int mode, int end)
{
	DBGPR_FUNC("-->dwmac4_rd_init_etx_desc\n");
	p->basic.des0 = 0;
	p->basic.des1 = 0;
	p->basic.des2 = 0;
	p->basic.des3 = 0;
	p->des4 = 0;
	p->des5 = 0;
	p->des6 = 0;
	p->des7 = 0;
	DBGPR_FUNC("<--dwmac4_rd_init_etx_desc\n");
}

void dwmac4_rd_prepare_etx_desc(struct dma_enhanced_desc *p, int is_fs, int len,
				      bool csum_flag, int mode, bool tx_own,
				      bool ls, unsigned int tot_pkt_len, int GSN, unsigned int launch_time)
{
	unsigned int tdes3 = le32_to_cpu(p->basic.des3);

	DBGPR_FUNC("-->dwmac4_rd_prepare_etx_desc\n");

	p->basic.des2 |= cpu_to_le32(len & TDES2_BUFFER1_SIZE_MASK);
	p->basic.des2 |= cpu_to_le32(TDES2_TIMESTAMP_ENABLE);   // Enable Timestamp

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
		 * avoid race condition.
		 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 140))
        wmb();
#else
        dma_wmb();
#endif

	p->basic.des3 = cpu_to_le32(tdes3);

	p->des4 |= cpu_to_le32((GSN << 8) & ETDES4_GSN_MASK);
	if(mode){ // fake to use mode to tv_flag
		p->des4 |= (0x1 << 31) | (launch_time >> 24); //LTV 
		p->des5 |= launch_time << 8;
	}else{
		p->des4 &= 0x7fffffff;//LTV disabled
	}
	p->des6 |= 0x0;
	p->des7 |= 0x0;	
	DBGPR_FUNC("<--dwmac4_rd_prepare_etx_desc\n");
	
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
void dwmac4_rd_prepare_tso_etx_desc(struct dma_enhanced_desc *p, int is_fs,
					  int len1, int len2, bool tx_own,
					  bool ls, unsigned int tcphdrlen,
					  unsigned int tcppayloadlen)
{
	unsigned int tdes3 = le32_to_cpu(p->basic.des3);

	DBGPR_FUNC("-->dwmac4_rd_prepare_tso_etx_desc\n");

	if (len1)
		p->basic.des2 |= cpu_to_le32((len1 & TDES2_BUFFER1_SIZE_MASK));

	if (len2)
		p->basic.des2 |= cpu_to_le32((len2 << TDES2_BUFFER2_SIZE_MASK_SHIFT)
			    & TDES2_BUFFER2_SIZE_MASK);

	if (is_fs) {
		p->des4 |= ETDES4_LTV;   /*LTV field is set if it a first Descriptor */
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

	p->basic.des3 = cpu_to_le32(tdes3);
	DBGPR_FUNC("<--dwmac4_rd_prepare_tso_etx_desc\n");

}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

void dwmac4_release_etx_desc(struct dma_enhanced_desc *p, int mode)
{
	DBGPR_FUNC("-->dwmac4_release_etx_desc\n");
	p->basic.des0 = 0;
	p->basic.des1 = 0;
	p->basic.des2 = 0;
	p->basic.des3 = 0;
	p->des4 = 0;
	p->des5=0;
	p->des6=0;
	DBGPR_FUNC("<--dwmac4_release_etx_desc\n");

}

void dwmac4_rd_set_etx_ic(struct dma_enhanced_desc *p)
{
	DBGPR_FUNC("-->dwmac4_rd_set_etx_ic\n");
	p->basic.des2 |= cpu_to_le32(TDES2_INTERRUPT_ON_COMPLETION);
	DBGPR_FUNC("<--dwmac4_rd_set_etx_ic\n");
}

void dwmac4_display_en_ring(void *head, unsigned int size, bool rx)
{
	struct dma_enhanced_desc *p = (struct dma_enhanced_desc *)head;
	int i;

	DBGPR_FUNC("-->dwmac4_display_en_ring\n");
	pr_info("%s descriptor ring:\n", rx ? "RX" : "TX");

	for (i = 0; i < size; i++) {
		pr_info("%d [0x%x]: 0x%x 0x%x 0x%x 0x%x",
			i, (unsigned int)virt_to_phys(p),
			(p->basic.des0), (p->basic.des1),
			(p->basic.des2), (p->basic.des3));
		pr_info(" : 0x%x 0x%x 0x%x 0x%x\n",
			(p->des4), (p->des5),
			(p->des6),(p->des7));

		p++;
	}
	DBGPR_FUNC("<--dwmac4_display_en_ring\n");
}

#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
void dwmac4_set_en_mss_ctxt(struct dma_enhanced_desc *p, unsigned int mss)
{
	DBGPR_FUNC("-->dwmac4_set_en_mss_ctxt\n");
	p->basic.des0 = 0;
	p->basic.des1 = 0;
	p->basic.des2 = cpu_to_le32(mss);
	p->basic.des3 = cpu_to_le32(TDES3_CONTEXT_TYPE | TDES3_CTXT_TCMSSV);
	DBGPR_FUNC("<--dwmac4_set_en_mss_ctxt\n");
}
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */

const struct tc9562mac_desc_ops dwmac4_edesc_ops = {
	.etx_status = dwmac4_wrback_get_etx_status,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE			
	.get_etx_len = dwmac4_rd_get_etx_len,
	.get_etx_owner = dwmac4_get_etx_owner,
	.set_etx_owner = dwmac4_set_etx_owner,
	.get_etx_ls = dwmac4_get_etx_ls,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
	.enable_etx_timestamp = dwmac4_rd_enable_etx_timestamp,
	.get_etx_timestamp_status = dwmac4_wrback_get_etx_timestamp_status,
	.get_en_timestamp = dwmac4_get_en_timestamp,
	.set_etx_ic = dwmac4_rd_set_etx_ic,
	.prepare_etx_desc = dwmac4_rd_prepare_etx_desc,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
	.prepare_tso_etx_desc = dwmac4_rd_prepare_tso_etx_desc,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
	.release_etx_desc = dwmac4_release_etx_desc,
	.init_etx_desc = dwmac4_rd_init_etx_desc,
	.display_en_ring = dwmac4_display_en_ring,
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE	
	.set_en_mss = dwmac4_set_en_mss_ctxt,
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */	
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


