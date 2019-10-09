/*
 * TC9562 ethernet driver.
 *
 * ring_mode.c
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
	unsigned int bmax, len, des2;
	struct dma_desc *desc;
	DBGPR_FUNC("-->tc9562mac_jumbo_frm\n");
	if (priv->extend_desc)
		desc = (struct dma_desc *)(tx_q->dma_etx + entry);
	else
		desc = tx_q->dma_tx + entry;

	if (priv->plat->enh_desc)
		bmax = BUF_SIZE_8KiB;
	else
		bmax = BUF_SIZE_2KiB;

	len = nopaged_len - bmax;

	if (nopaged_len > BUF_SIZE_8KiB) {

		des2 = dma_map_single(priv->device, skb->data, bmax,
				      DMA_TO_DEVICE);
		desc->des2 = cpu_to_le32(des2);
		if (dma_mapping_error(priv->device, des2))
			return -1;

		tx_q->tx_skbuff_dma[entry].buf = des2;
		tx_q->tx_skbuff_dma[entry].len = bmax;
		tx_q->tx_skbuff_dma[entry].is_jumbo = true;

		desc->des3 = cpu_to_le32(des2 + BUF_SIZE_4KiB);
		priv->hw->desc->prepare_tx_desc(desc, 1, bmax, csum,
						TC9562MAC_RING_MODE, 0,
						false, skb->len);
		tx_q->tx_skbuff[entry] = NULL;
		entry = TC9562MAC_GET_ENTRY(entry, DMA_TX_SIZE);

		if (priv->extend_desc)
			desc = (struct dma_desc *)(tx_q->dma_etx + entry);
		else
			desc = tx_q->dma_tx + entry;

		des2 = dma_map_single(priv->device, skb->data + bmax, len,
				      DMA_TO_DEVICE);
		desc->des2 = cpu_to_le32(des2);
		if (dma_mapping_error(priv->device, des2))
			return -1;
		tx_q->tx_skbuff_dma[entry].buf = des2;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].is_jumbo = true;

		desc->des3 = cpu_to_le32(des2 + BUF_SIZE_4KiB);
		priv->hw->desc->prepare_tx_desc(desc, 0, len, csum,
						TC9562MAC_RING_MODE, 1,
						true, skb->len);
	} else {
		des2 = dma_map_single(priv->device, skb->data,
				      nopaged_len, DMA_TO_DEVICE);
		desc->des2 = cpu_to_le32(des2);
		if (dma_mapping_error(priv->device, des2))
			return -1;
		tx_q->tx_skbuff_dma[entry].buf = des2;
		tx_q->tx_skbuff_dma[entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[entry].is_jumbo = true;
		desc->des3 = cpu_to_le32(des2 + BUF_SIZE_4KiB);
		priv->hw->desc->prepare_tx_desc(desc, 1, nopaged_len, csum,
						TC9562MAC_RING_MODE, 0,
						true, skb->len);
	}

	tx_q->cur_tx = entry;
	DBGPR_FUNC("<--tc9562mac_jumbo_frm\n");
	return entry;
}

static unsigned int tc9562mac_is_jumbo_frm(int len, int enh_desc)
{
	unsigned int ret = 0;
	DBGPR_FUNC("-->tc9562mac_is_jumbo_frm\n");
	if (len >= BUF_SIZE_4KiB)
		ret = 1;
	DBGPR_FUNC("<--tc9562mac_is_jumbo_frm\n");
	return ret;
}

static void tc9562mac_refill_desc3(void *priv_ptr, struct dma_desc *p)
{
	struct tc9562mac_priv *priv = (struct tc9562mac_priv *)priv_ptr;
	DBGPR_FUNC("-->tc9562mac_refill_desc3\n");
	/* Fill DES3 in case of RING mode */
	if (priv->dma_buf_sz >= BUF_SIZE_8KiB)
		p->des3 = cpu_to_le32(le32_to_cpu(p->des2) + BUF_SIZE_8KiB);
	DBGPR_FUNC("<--tc9562mac_refill_desc3\n");
}

/* In ring mode we need to fill the desc3 because it is used as buffer */
static void tc9562mac_init_desc3(struct dma_desc *p)
{
	DBGPR_FUNC("-->tc9562mac_init_desc3\n");
	p->des3 = cpu_to_le32(le32_to_cpu(p->des2) + BUF_SIZE_8KiB);
	DBGPR_FUNC("<--tc9562mac_init_desc3\n");
}

static void tc9562mac_clean_desc3(void *priv_ptr, struct dma_desc *p)
{
	struct tc9562mac_tx_queue *tx_q = (struct tc9562mac_tx_queue *)priv_ptr;
	struct tc9562mac_priv *priv = tx_q->priv_data;
	unsigned int entry = tx_q->dirty_tx;
	DBGPR_FUNC("-->tc9562mac_clean_desc3\n");
	/* des3 is only used for jumbo frames tx or time stamping */
	if (unlikely(tx_q->tx_skbuff_dma[entry].is_jumbo ||
		     (tx_q->tx_skbuff_dma[entry].last_segment &&
		      !priv->extend_desc && priv->hwts_tx_en)))
		p->des3 = 0;
	DBGPR_FUNC("<--tc9562mac_clean_desc3\n");
}

static int tc9562mac_set_16kib_bfsize(int mtu)
{
	int ret = 0;
	DBGPR_FUNC("-->tc9562mac_set_16kib_bfsize\n");
	if (unlikely(mtu >= BUF_SIZE_8KiB))
		ret = BUF_SIZE_16KiB;
	DBGPR_FUNC("<--tc9562mac_set_16kib_bfsize\n");
	return ret;
}

const struct tc9562mac_mode_ops ring_mode_ops = {
	.is_jumbo_frm = tc9562mac_is_jumbo_frm,
	.jumbo_frm = tc9562mac_jumbo_frm,
	.refill_desc3 = tc9562mac_refill_desc3,
	.init_desc3 = tc9562mac_init_desc3,
	.clean_desc3 = tc9562mac_clean_desc3,
	.set_16kib_bfsize = tc9562mac_set_16kib_bfsize,
};
#endif /* TC9562_UNSUPPORTED_UNTESTED_FEATURE */
