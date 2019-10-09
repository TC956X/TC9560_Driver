/*
 * TC9562 ethernet driver.
 *
 * dwmac4_enh_desc.h
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

#ifndef __DWMAC4_ENH_DESCS_H__
#define __DWMAC4_ENH_DESCS_H__

#include <linux/bitops.h>

/* Normal transmit descriptor defines (without split feature) */

/* Added for Enhanced Descriptors */
/* ETDS4 use for both format (read and write back) */
#define ETDES4_LT_MASK        GENMASK(8,0)	 /* launch time */
#define ETDES4_GSN_MASK		  GENMASK(11, 8)

#define ETDES4_LTV			   BIT(31)

/* ETDS5 use for both format (read and write back) */
#define ETDES5_LT        		GENMASK(31,8)  /* launch time*/


int dwmac4_wrback_get_etx_status(void *data, struct tc9562mac_extra_stats *x,
				       struct dma_enhanced_desc *p,void __iomem *ioaddr);
int dwmac4_rd_get_etx_len(struct dma_enhanced_desc *p);
int dwmac4_get_etx_owner(struct dma_enhanced_desc *p);
void dwmac4_set_etx_owner(struct dma_enhanced_desc *p);
int dwmac4_get_etx_ls(struct dma_enhanced_desc *p);
void dwmac4_rd_enable_etx_timestamp(struct dma_enhanced_desc *p);
int dwmac4_wrback_get_etx_timestamp_status(struct dma_enhanced_desc *p);
inline u64 dwmac4_get_en_timestamp(void *desc, u32 ats);
void dwmac4_rd_init_etx_desc(struct dma_enhanced_desc *p, int mode, int end);
void dwmac4_rd_prepare_etx_desc(struct dma_enhanced_desc *p, int is_fs, int len,
				      bool csum_flag, int mode, bool tx_own,
				      bool ls, unsigned int tot_pkt_len, int GSN, unsigned int launch_time);
void dwmac4_rd_prepare_tso_etx_desc(struct dma_enhanced_desc *p, int is_fs,
					  int len1, int len2, bool tx_own,
					  bool ls, unsigned int tcphdrlen,
					  unsigned int tcppayloadlen);
void dwmac4_release_etx_desc(struct dma_enhanced_desc *p, int mode);
void dwmac4_rd_set_etx_ic(struct dma_enhanced_desc *p);
void dwmac4_display_en_ring(void *head, unsigned int size, bool rx);
void dwmac4_set_en_mss_ctxt(struct dma_enhanced_desc *p, unsigned int mss);



#endif /* __DWMAC4_ENH_DESCS_H__ */
