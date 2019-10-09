/*
 * TC9562 ethernet driver.
 *
 * chain_mode.c 
 *
 * Copyright (C) 2011 STMicroelectronics Ltd
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

#include "tc9562mac.h"
#ifdef TC9562_UNSUPPORTED_UNTESTED_FEATURE
static int tc9562mac_jumbo_frm(void *p, struct sk_buff *skb, int csum)
{
	struct tc9562mac_tx_queue *tx_q = (struct tc9562mac_tx_queue *)p;
	unsigned int nopaged_len = skb_headlen(skb);
	struct tc9562mac_priv *priv = tx_q->priv_data;
	unsigned int entry = tx_q->cur_tx;
	unsigned int bmax, des2;
	unsigned int i = 1, len;
	struct dma_desc *desc;

	desc = tx_q->dma_tx + entry;

	if (priv->plat->enh_desc)
		bmax = BUF_SIZE_8KiB;
	else
		bmax = BUF_SIZE_2KiB;

	len = nopaged_len - bmax;

	des2 = dma_map_single(priv->device, skb->data,
			      bmax, DMA_TO_DEVICE);
	desc->des2 = cpu_to_le32(des2);
	if (dma_mapping_error(priv->device, des2))
		return -1;
	tx_q->tx_skbuff_dma[entry].buf = des2;
	tx_q->tx_skbuff_dma[entry].len = bmax;
	/* do not close the descriptor and do not set own bit */
	priv->hw->desc->prepare_tx_desc(desc, 1, bmax, csum, TC9562MAC_CHAIN_MODE,
					0, false, skb->len);

	while (len != 0) {
		tx_q->tx_skbuff[entry] = NULL;
		entry = TC9562MAC_GET_ENTRY(entry, DMA_TX_SIZE);
		desc = tx_q->dma_tx + entry;

		if (len > bmax) {
			des2 = dma_map_single(priv->device,
					      (skb->data + bmax * i),
					      bmax, DMA_TO_DEVICE);
			desc->des2 = cpu_to_le32(des2);
			if (dma_mapping_error(priv->device, des2))
				return -1;
			tx_q->tx_skbuff_dma[entry].buf = des2;
			tx_q->tx_skbuff_dma[entry].len = bmax;
			priv->hw->desc->prepare_tx_desc(desc, 0, bmax, csum,
							TC9562MAC_CHAIN_MODE, 1,
							false, skb->len);
			len -= bmax;
			i++;
		} else {
			des2 = dma_map_single(priv->device,
					      (skb->data + bmax * i), len,
					      DMA_TO_DEVICE);
			desc->des2 = cpu_to_le32(des2);
			if (dma_mapping_error(priv->device, des2))
				return -1;
			tx_q->tx_skbuff_dma[entry].buf = des2;
			tx_q->tx_skbuff_dma[entry].len = len;
			/* last descriptor can be set now */
			priv->hw->desc->prepare_tx_desc(desc, 0, len, csum,
							TC9562MAC_CHAIN_MODE, 1,
							true, skb->len);
			len = 0;
		}
	}

	tx_q->cur_tx = entry;

	return entry;
}

static unsigned int tc9562mac_is_jumbo_frm(int len, int enh_desc)
{
	unsigned int ret = 0;

	if ((enh_desc && (len > BUF_SIZE_8KiB)) ||
	    (!enh_desc && (len > BUF_SIZE_2KiB))) {
		ret = 1;
	}

	return ret;
}

static void tc9562mac_init_dma_chain(void *des, dma_addr_t phy_addr,
				  unsigned int size, unsigned int extend_desc)
{
	/*
	 * In chained mode the des3 points to the next element in the ring.
	 * The latest element has to point to the head.
	 */
	int i;
	dma_addr_t dma_phy = phy_addr;

	if (extend_desc) {
		struct dma_extended_desc *p = (struct dma_extended_desc *)des;
		for (i = 0; i < (size - 1); i++) {
			dma_phy += sizeof(struct dma_extended_desc);
			p->basic.des3 = cpu_to_le32((unsigned int)dma_phy);
			p++;
		}
		p->basic.des3 = cpu_to_le32((unsigned int)phy_addr);

	} else {
		struct dma_desc *p = (struct dma_desc *)des;
		for (i = 0; i < (size - 1); i++) {
			dma_phy += sizeof(struct dma_desc);
			p->des3 = cpu_to_le32((unsigned int)dma_phy);
			p++;
		}
		p->des3 = cpu_to_le32((unsigned int)phy_addr);
	}
}

static void tc9562mac_refill_desc3(void *priv_ptr, struct dma_desc *p)
{
	struct tc9562mac_rx_queue *rx_q = (struct tc9562mac_rx_queue *)priv_ptr;
	struct tc9562mac_priv *priv = rx_q->priv_data;

	if (priv->hwts_rx_en && !priv->extend_desc)
		/* NOTE: Device will overwrite des3 with timestamp value if
		 * 1588-2002 time stamping is enabled, hence reinitialize it
		 * to keep explicit chaining in the descriptor.
		 */
		p->des3 = cpu_to_le32((unsigned int)(rx_q->dma_rx_phy +
				      (((rx_q->dirty_rx) + 1) %
				       DMA_RX_SIZE) *
				      sizeof(struct dma_desc)));
}

static void tc9562mac_clean_desc3(void *priv_ptr, struct dma_desc *p)
{
	struct tc9562mac_tx_queue *tx_q = (struct tc9562mac_tx_queue *)priv_ptr;
	struct tc9562mac_priv *priv = tx_q->priv_data;
	unsigned int entry = tx_q->dirty_tx;

	if (tx_q->tx_skbuff_dma[entry].last_segment && !priv->extend_desc &&
	    priv->hwts_tx_en)
		/* NOTE: Device will overwrite des3 with timestamp value if
		 * 1588-2002 time stamping is enabled, hence reinitialize it
		 * to keep explicit chaining in the descriptor.
		 */
		p->des3 = cpu_to_le32((unsigned int)((tx_q->dma_tx_phy +
				      ((tx_q->dirty_tx + 1) % DMA_TX_SIZE))
				      * sizeof(struct dma_desc)));
}

const struct tc9562mac_mode_ops chain_mode_ops = {
	.init = tc9562mac_init_dma_chain,
	.is_jumbo_frm = tc9562mac_is_jumbo_frm,
	.jumbo_frm = tc9562mac_jumbo_frm,
	.refill_desc3 = tc9562mac_refill_desc3,
	.clean_desc3 = tc9562mac_clean_desc3,
};
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
